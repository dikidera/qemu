.. _ecu-emulator:

ECU emulation (SH-2/SH-2E and M32C/87)
======================================

.. note::

   This is a local experiment in a QEMU fork, written with an AI coding
   assistant.  Per ``docs/devel/code-provenance.rst`` it is not intended
   for, and must not be submitted to, upstream QEMU.

This fork turns QEMU into an engine control unit (ECU) emulator: an
unmodified ECU firmware image runs on an emulated microcontroller whose
peripherals are driven by a simulated engine.  The firmware reads sensor
voltages through its A/D converters, crank/cam/speed pulses through its
timer inputs and switches through its ports, exactly as on the car, and
the simulator measures what the firmware drives back: injector pulse
widths and timing, ignition dwell and advance, relays and PWM outputs.
CAN and serial (K-line) traffic can be bridged to the host.

Building
--------

::

    ./configure --target-list=sh4eb-softmmu,m32c-softmmu
    make

``qemu-system-sh4eb`` provides the SH-2/SH-2E machines (the SH705x parts
are big endian), ``qemu-system-m32c`` the M32C/87 machine.

Machines
--------

============== ========================= ========== ============ =========
Machine        MCU / core                Flash      RAM          CAN
============== ========================= ========== ============ =========
``ecu-sh7052`` SH7052, SH-2              256 KiB @0 12 KiB        1x HCAN
                                                    @FFFF8000
``ecu-sh7054`` SH7054, SH-2              384 KiB @0 16 KiB        1x HCAN
                                                    @FFFF8000
``ecu-sh7055`` SH7055, SH-2E (FPU)       512 KiB @0 32 KiB        2x HCAN
                                                    @FFFF6000
``ecu-sh7058`` SH7058, SH-2E (FPU)       1 MiB @0   48 KiB        2x HCAN2
                                                    @FFFF0000
``ecu-m32c87`` M32C/87, M32C/80 core     1 MiB at   48 KiB @400   2x CAN
                                         top of 16M
============== ========================= ========== ============ =========

The firmware is a raw dump of the internal flash, given with ``-bios``::

    qemu-system-sh4eb -M ecu-sh7058 -bios rom.bin -display none \
        -serial stdio -monitor telnet::4444,server,nowait

    qemu-system-m32c -M ecu-m32c87 -bios flash.bin -display none \
        -serial stdio -monitor telnet::4444,server,nowait

For the SH705x the reset vector (PC at 0, SP at 4) is taken from the
image.  For the M32C/87 the image is mapped so that it ends at 0xFFFFFF
(reset vector at 0xFFFFFC); use ``-global m32c87-soc.rom-size=0x80000``
etc. for smaller flash parts.

The new CPU models are listed by ``-cpu help`` as ``sh2``/``sh2e``
(target ``sh4eb``) and ``m32c80`` (target ``m32c``); the ECU machines
pick the right one for their MCU.

Peripheral clock
~~~~~~~~~~~~~~~~

Baud rates and timer rates depend on the peripheral clock, which differs
between ECUs.  The defaults are 20 MHz (SH705x) and 32 MHz (M32C/87);
change them with ``-global sh705x-soc.pclk-hz=...`` or
``-global m32c87-soc.pclk-hz=...``.  A quick way to find the right value
is the SCI/UART bit rate register the firmware programs for its K-line
(10400 baud): on SH705x ``BRR = pclk / (32 * 10400) - 1``.

Serial ports
~~~~~~~~~~~~

SH705x SCI0..4 and M32C/87 UART0..4 are connected to ``-serial`` 0..4.
Transmission and reception are paced at the programmed bit rate.  Real
K-line transceivers echo transmitted bytes back to the receiver; enable
this with ``-global sh705x-soc.kline-echo=on`` (or
``m32c87-soc.kline-echo=on``) if the firmware expects it.

CAN
~~~

The CAN controllers attach to QEMU ``can-bus`` objects through the
``canbus0``/``canbus1`` machine properties.  To connect them to a Linux
SocketCAN interface (e.g. a virtual one for testing with ``candump`` /
``cansend``)::

    sudo ip link add dev vcan0 type vcan && sudo ip link set vcan0 up
    qemu-system-sh4eb -M ecu-sh7058,canbus0=can0 -bios rom.bin \
        -object can-bus,id=can0 \
        -object can-host-socketcan,id=sc0,if=vcan0,canbus=can0 ...

Both controllers of one chip may share a bus.  Transmissions complete
immediately and are always acknowledged; bit timing and error counters
are not modelled.

Engine simulator
----------------

Every ECU machine contains an ``ecu-engine`` device.  It has *signals*
(sensor outputs, digital inputs, measured ECU outputs) which are
*bound* to MCU pin names.

Analog sensors (voltage presented on an ``ANn`` input):
``map maf clt iat tps tps2 app app2 o2 o2b wbo2 vbat knock knock2 baro
fuel_temp fuel_press oil_press oil_temp egt fuel_level ac_press ref5 gnd
anaux1..4``

Digital inputs driven by the simulator:
``crank cam cam2 vss ign_sw start_sw neutral_sw clutch_sw brake_sw ac_sw
ps_sw din1..4``

ECU outputs measured by the simulator:
``inj1..12 ign1..12 fpump fan1 fan2 cel tach isc isc2 purge ac_clutch
main_relay vvt egr boost o2_heater aux1..4``

Engine model
~~~~~~~~~~~~

* Crank wheel: regular N-M wheel (``wheel 36 2``, ``wheel 60 2`` ...) or
  any tooth pattern (``pattern crank 0:5,30:5,...`` in degrees per
  revolution); cam patterns are given over 720 degrees.  Cylinder 1 TDC
  is ``tdc`` degrees after the first tooth, firing order with
  ``firing 1342``.
* MAP follows throttle (``tps``) and ``baro``; MAF is computed by speed
  density from MAP, IAT, displacement and ``ve`` (or set directly).  The
  MAF voltage uses ``maf_v0 + maf_gain * sqrt(g/s)`` or a table copied
  from the ROM's MAF scaling (``maf_curve 1.0:2.5,1.5:6.1,...``).
* Temperature sensors are NTCs (``ntc_r25``, ``ntc_beta``,
  ``ntc_pullup``).
* Fuel: the open time of every injector (minus ``inj_dead``) times
  ``inj_flow`` gives the fuel flow, the mixture gives lambda and the O2
  sensor voltages (narrowband, rear and 0-5 V wideband).
* ``mode 0`` (default) keeps the rpm where you set it.  ``mode 1`` lets
  rpm follow the engine: it only runs while the cylinders get fuel and
  spark with a burnable mixture, cranks at 250 rpm while ``start_sw`` is
  on, and rises with throttle.
* Status shows per cylinder the injector pulse width and end of
  injection, and the ignition advance (degrees BTDC at the end of dwell)
  and dwell.

Controlling the simulator
~~~~~~~~~~~~~~~~~~~~~~~~~

Through the monitor (``ecu <command>``), a character device with the same
line protocol, or at startup::

    -global ecu-engine.script="set rpm 3000;set clt 90;wheel 36 2"
    -global ecu-engine.bind="map=AN3,crank=TI10,inj1=PB4+PB5"
    -chardev socket,id=ecuctl,port=5555,server=on,wait=off \
    -global ecu-engine.chardev=ecuctl

Commands::

    status                     engine state and measured outputs
    stream <ms>                print status periodically (chardev only)
    set <param> <value>        rpm, tps, clt, iat, vbat, speed, mode, ...
    set ign_sw 1               drive a digital input
    set map 80 | set afr 12.5  override a modelled value; "auto map" to
                               return to the model
    set AN5 2.34               force a raw pin voltage ("auto AN5")
    force o2 0.8               force a sensor voltage
    get <name>                 parameter, signal (voltage / level / pulse
                               measurement) or pin
    bind <signal> <PIN[+PIN]>  connect a signal to MCU pins
    unbind <signal>
    bindings | signals | params | pins
    wheel <teeth> <missing>
    pattern crank|cam|cam2 <start:width,...>
    firing <order>
    maf_curve <V:gps,...>

Pin names
~~~~~~~~~

The default bindings of each machine are only a starting point: every
ECU wires its sensors differently.  Look at the ``bindings`` command and
re-bind according to the ECU's pinout or the ROM disassembly.

SH705x:

* analog inputs ``AN0``..``AN31`` (A/D0: 0-11, A/D1: 12-23, A/D2: 24-31)
* ATU-II: ``TI0A``..``TI0D`` (channel 0 input capture), ``TIO1A``..
  ``TIO1H``, ``TIO2A``..``TIO2H``, ``TIO3A``..``TIO5D``, ``TIO11A/B``
  (capture/compare), ``TO6A``..``TO7D`` (PWM), ``TO8A``..``TO8P``
  (one-shot pulses), ``TI9A``..``TI9F`` (event counters), ``TI10``
  (crank angle channel)
* ports ``PA0``..``PL13``, interrupts ``IRQ0``..``IRQ7``, ``NMI``

M32C/87:

* analog inputs ``AN0``..``AN7`` (P10), ``AN8``..``AN15`` (P0, APS=2),
  ``AN16``..``AN23`` (P2, APS=3), ``AN24``..``AN31`` (P15, APS=1)
* timers ``TA0IN``..``TA4IN``, ``TA0OUT``..``TA4OUT``, ``TB0IN``..
  ``TB5IN``
* ports ``P0_0``..``P15_7``, interrupts ``INT0``..``INT5``, ``NMI``

A signal may be bound to several pins, e.g. the crank to both the timer
input and the port pin the firmware polls (``bind crank TI10+PF1``).

Model fidelity and limitations
------------------------------

SH-2/SH-2E core
  Built on QEMU's SH-4 translator: flat address space, SH-2 exception
  model (SR/PC stacked, vectors via VBR, RTE), interrupt priority masks,
  FPU-less SH-2 traps FPU opcodes, SH-2E single precision FPU.

SH705x peripherals
  Register addresses, vector numbers and bit layouts come from the
  Renesas iodefine headers / nissutils register lists.  Modelled: INTC,
  WDT, CMT, ports, A/D0-2 (single and scan modes), SCI0-4, ATU-II
  channels 0-11, HCAN/HCAN2.  Simplified: the channel 10 angle clock
  multiplier, ATU DMA/A-D trigger links other than the interval timer,
  PFC pin multiplexing (port and timer pins are separate names).  Not
  modelled: DMAC, UBC, H-UDI, flash programming (writes to the flash
  registers are stored but the ROM is read only), power down modes.

M32C/80 core
  New ``m32c`` target.  The opcode table is generated from Ghidra's
  M16C_80 SLEIGH specification; semantics follow the M32C/80 software
  manual.  All instructions incl. indirect and INDEX/BITINDEX prefixes,
  string, decimal and context switching instructions, register banks,
  variable/fixed vectors and the high-speed interrupt are implemented.
  Execution is interpreted (one helper call per instruction).

M32C/87 peripherals
  Interrupt control registers, timers A (timer, event, one-shot, PWM) and
  B (timer, event, period/pulse width measurement), A/D0, UART0-4 (UART
  mode), ports, INT/NMI pins, watchdog, CAN0/CAN1.  The CAN register
  layout (``can_layout`` in ``hw/m32c/m32c87.c``) and the CAN interrupt
  vectors (``-global m32c87-soc.can0-vec=...``) are best-effort guesses
  and should be checked against the M32C/87 hardware manual; the UART0/1
  base addresses and the interrupt control register table are also worth
  verifying for a given part.  Intelligent I/O, DMAC, three-phase motor
  control and the D/A converter are not modelled.

General
  Timing is driven by QEMU's virtual clock, not by instruction counts, so
  CPU speed relative to the timers is not cycle accurate (``-icount``
  can be used for deterministic runs).  External chips found on ECU
  boards (watchdog/supervisor ICs, EEPROMs on SPI/I2C, knock sensor
  ASICs, ignition/injector driver diagnostics) are not modelled; their
  inputs can be emulated with the generic digital and analog signals.
  Watchdog resets can be disabled with ``-global sh705x-soc.wdt-reset=off``
  / ``m32c87-soc.wdt-reset=off``.

Debugging
---------

* ``-d int`` logs every exception/interrupt with its vector, ``-d
  guest_errors,unimp`` reports accesses to unmodelled registers.
* ``-s -S`` starts the gdbstub.  For the SH705x use an ``sh-elf`` gdb
  (``set architecture sh2e``).  The M32C gdbstub exposes r0-r3, a0, a1,
  fb, sb, usp, isp, pc, intb and flg.

Tests
-----

``tests/ecu/run-tests.sh [build-dir]`` assembles small test ROMs
(``tests/ecu/shasm.py``, ``tests/ecu/m32casm.py``) and checks serial
output, A/D readings, interrupts, CAN transmit/receive and the injector
and coil pulse measurements on every machine, plus an M32C instruction
self test.
