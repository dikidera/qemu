.. _ecu-emulator:

ECU emulation (SH-2/SH-2E, M32C/87 and MC68376)
===============================================

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

    ./configure --target-list=sh4eb-softmmu,m32c-softmmu,m68k-softmmu
    make

``qemu-system-sh4eb`` provides the SH-2/SH-2E machines (the SH705x parts
are big endian), ``qemu-system-m32c`` the M32C/87 machine and
``qemu-system-m68k`` the MC68376 machine.

Machines
--------

================ ========================= ========== ============ =========
Machine          MCU / core                Flash      RAM          CAN
================ ========================= ========== ============ =========
``ecu-sh7052``   SH7052, SH-2              256 KiB @0 12 KiB        1x HCAN
                                                      @FFFF8000
``ecu-sh7054``   SH7054, SH-2              384 KiB @0 16 KiB        1x HCAN
                                                      @FFFF8000
``ecu-sh7055``   SH7055, SH-2E (FPU)       512 KiB @0 32 KiB        2x HCAN
                                                      @FFFF6000
``ecu-sh7058``   SH7058, SH-2E (FPU)       1 MiB @0   48 KiB        2x HCAN2
                                                      @FFFF0000
``ecu-m32c87``   M32C/87, M32C/80 core     1 MiB at   48 KiB @400   2x CAN
                                           top of 16M
``ecu-mc68376``  MC68376, CPU32            external,  4 KiB SRAM +  1x TouCAN
                                           512 KiB on 3.5 KiB
                                           CSBOOT @0  TPURAM,
                                                      relocatable
================ ========================= ========== ============ =========

The firmware is a raw dump of the internal flash, given with ``-bios``::

    qemu-system-sh4eb -M ecu-sh7058 -bios rom.bin -display none \
        -serial stdio -monitor telnet::4444,server,nowait

    qemu-system-m32c -M ecu-m32c87 -bios flash.bin -display none \
        -serial stdio -monitor telnet::4444,server,nowait

For the SH705x the reset vector (PC at 0, SP at 4) is taken from the
image, likewise for the MC68376 (SSP at 0, PC at 4: the image is the
external boot flash that the CSBOOT chip select maps at 0 after reset;
size ``-global mc68376-soc.flash-size=...``, default 512 KiB).  For the M32C/87 the image is mapped so that it ends at 0xFFFFFF
(reset vector at 0xFFFFFC); use ``-global m32c87-soc.rom-size=0x80000``
etc. for smaller flash parts.

The new CPU models are listed by ``-cpu help`` as ``sh2``/``sh2e``
(target ``sh4eb``), ``m32c80`` (target ``m32c``) and ``cpu32`` (target
``m68k``); the ECU machines pick the right one for their MCU.

Peripheral clock
~~~~~~~~~~~~~~~~

Baud rates and timer rates depend on the peripheral clock, which differs
between ECUs.  The defaults are 20 MHz (SH705x; this is what npkern, which runs
on these ECUs, assumes for its SCI and ATU settings) and 32 MHz (M32C/87);
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

Flash programming
~~~~~~~~~~~~~~~~~

The SH705x on-chip flash can be erased and programmed by the firmware
(self-programming) or by a reflash kernel such as npkern, following the
sequences npkern uses on real ECUs.  The flash contents live in the
machine, not in the ``-bios`` file: a system reset keeps reprogrammed
data, the image file is never written.  Pulse timings are not checked;
one program/erase pulse always succeeds.

* 350 nm flash (``ecu-sh7052``, ``ecu-sh7054``, ``ecu-sh7055``):
  ``FLMCR1``/``FLMCR2``/``EBR1``/``EBR2``.  With ``SWEn`` set, byte writes
  to a 128-byte unit load the program latches and a ``P`` pulse (with
  ``PSU``) programs them; an ``E`` pulse (with ``ESU``) erases the blocks
  selected in ``EBR1``/``EBR2``; the dummy writes of program/erase verify
  (``PV``/``EV``) are ignored.  ``FLMCR1`` controls the flash below
  0x40000, ``FLMCR2`` the rest.  Erase blocks follow npkern (the SH7054
  uses the first 14 SH7055 blocks).  On the SH7052 the block layout is not
  known, so erase/program pulses are only logged.

* 180 nm flash (``ecu-sh7058``, and ``ecu-sh7055`` with
  ``-global sh705x-soc.flash-node=180``): the download method
  (``FCCS``/``FPCS``/``FECS``/``FKEY``/``FMATS``/``FTDAR``).  The on-chip
  programming/erasing routines are Renesas internal and not available, so
  they are emulated behind their documented calling interface:
  ``FCCS.SCO = 1`` with ``FKEY = 0xA5`` writes the result (``DPFR``) to the
  first byte of the download area (on-chip RAM start + ``FTDAR`` x 2 KiB),
  and a small stub at download address + 32 (initialisation, ``R4`` =
  ``FPEFEQ``) and + 16 (write: ``R4`` = source, ``R5`` = 128-byte aligned
  flash destination; erase: ``R4`` = block number) that hands the call to
  QEMU through private registers at 0xFFFFE810..0xFFFFE81F and returns
  ``FPFR`` in ``R0`` (0 = success).  Write and erase need ``FKEY = 0x5A``.
  Only the first 64 bytes of the download area are written.  The ``FPFR``
  error bit assignment is an approximation; firmware should only rely on
  0 meaning success.  ``FCCS`` reads 0x80 (``FWE`` pin high, no error).

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
  injection (degrees BTDC of the compression TDC, -360..360), and the
  ignition advance (degrees BTDC at the end of dwell, referred to the
  nearest TDC of that cylinder so wasted-spark coils read the same as
  coil-on-plug, -180..180) and dwell.

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

MC68376 (the TPU, CTM4 and QADC names are registered by those models):

* analog inputs ``AN0``..``AN3`` and ``AN48``..``AN59`` (QADC channel
  numbers, Table 5-16)
* TPU channels ``TPU0``..``TPU15``; CTM4 PWM ``CPWM5``..``CPWM8``
* SIM ports ``PE0``..``PE7``, ``PF0``..``PF7``, ``PC0``..``PC6``;
  interrupt pins ``IRQ1``..``IRQ7`` (the same pins as ``PF1``..``PF7``:
  a pin reads low when either of its names is driven low; both idle
  high)
* QSM port ``PQS0``..``PQS7`` (MISO, MOSI, SCK, PCS0/SS, PCS1-3, TXD)

The MC68376 default bindings (crank/cam/vss on ``TPU0``-``TPU2``,
injectors on ``TPU4``-``TPU7``, coils on ``TPU8``-``TPU11``, actuators on
``CPWM5``-``CPWM8``, sensors on ``AN0``-``AN3``/``AN48``-``AN59``) are
placeholders; every ECU uses its own TPU function assignment.

A signal may be bound to several pins, e.g. the crank to both the timer
input and the port pin the firmware polls (``bind crank TI10+PF1``).

Model fidelity and limitations
------------------------------

SH-2/SH-2E core
  Built on QEMU's SH-4 translator: flat address space, SH-2 exception
  model (SR/PC stacked, vectors via VBR, RTE), interrupt priority masks,
  FPU-less SH-2 traps FPU opcodes, SH-2E single precision FPU (SH-4 only
  FPU instructions such as FSQRT, FIPR, FTRV, FSCHG, FRCHG and the double
  conversions are illegal instructions).  NMI and edge-detected IRQn
  requests are cleared when the CPU accepts them.

SH705x peripherals
  Register addresses, vector numbers and bit layouts come from the
  Renesas iodefine headers / nissutils register lists.  Modelled: INTC,
  WDT, CMT, ports, A/D0-2 (single and scan modes), SCI0-4, ATU-II
  channels 0-11, HCAN/HCAN2.  Simplified: the channel 10 angle clock
  multiplier, ATU DMA/A-D trigger links other than the interval timer,
  PFC pin multiplexing (port and timer pins are separate names).  Flash
  programming: see "Flash programming" above.  Not modelled: DMAC, UBC, H-UDI, flash RAM
  emulation (``RAMER``), the user boot MAT, power down modes.

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
  mode), ports, INT/NMI pins, key input interrupt (KI0-KI3 on
  P10_4-P10_7), watchdog, CAN0/CAN1.  Core SFR addresses
  follow Ghidra's M16C_80 processor definition; the M32C/87 specific ones
  (UART0 at 0364h, UART1 at 02E4h, FMR0/FMR1 at 0057h/0055h, the CAN
  register map and the CAN interrupt control registers) were checked
  against the M32C/87 SFR table.  CAN interrupts 0..5 share the
  interrupt control registers of intelligent I/O interrupts 9, 10, 11, 0,
  1 and 5 and set bit 7 of the matching ``IIOnIR``.  Which CAN interrupt
  a slot or error event uses could not be confirmed without the hardware
  manual (REJ09B0180); it is set with ``-global m32c87-soc.can0-irq=N``,
  ``can0-err-irq``, ``can1-irq``, ``can1-err-irq`` (defaults 0, 2, 1, 2).
  The bit position of ``BANKSEL`` in ``CiCTLR1`` (bit 3) is likewise
  unconfirmed.  Received characters are held back until the firmware
  reads the receive buffer, so the UART error flags (overrun, framing,
  parity) never set.  Not modelled: timer trigger select (``TRGSR``),
  two-phase pulse mode, intelligent I/O, DMAC, flash
  programming (CPU rewrite mode), clock/PLL and protect registers
  (``PRCR`` writes are not enforced), clock synchronous and I2C UART
  modes, the D/A converter and three-phase motor control.

General
  Timing is driven by QEMU's virtual clock, not by instruction counts, so
  CPU speed relative to the timers is not cycle accurate (``-icount``
  can be used for deterministic runs).  External chips found on ECU
  boards (watchdog/supervisor ICs, EEPROMs on SPI/I2C, knock sensor
  ASICs, ignition/injector driver diagnostics) are not modelled; their
  inputs can be emulated with the generic digital and analog signals.
  Watchdog resets can be disabled with ``-global sh705x-soc.wdt-reset=off``
  / ``m32c87-soc.wdt-reset=off``.

MC68376
-------

Reference: MC68336/376 User's Manual (MC68336376UM/D); section numbers
below refer to it.  Mode select pins at reset are assumed in their
default (pulled-up) state: 16-bit CSBOOT, CS[10:0] as chip selects,
port E/F pins as bus control/IRQ pins, MODCLK high (PLL on), no BDM.
Options: ``-global mc68376-soc.<prop>=...``.

Clock (5.3)
  ``fsys = fref / 128 * 4 (Y+1) 2^(2W+X)`` from ``SYNCR``; ``fref`` is
  the ``extal-hz`` property (default 4.194304 MHz, the manual's typical
  crystal, 5.3.1).  Reset ``SYNCR`` = $3F00 gives 8.39 MHz;
  ECUs usually program 16.78 MHz ($7F00) or 20.97 MHz.  The VCO locks
  instantly (``SLOCK`` reads 1).  Other modules read the clock with
  ``mc68376_sysclk_hz()`` and can register a change notifier.

Address map (5.2.1, 5.9, 6, 7, 12)
  24-bit bus mirrored over the 4 GiB CPU space.  The module block is at
  $FFF000 (``SIMCR.MM`` = 1, reset) or $7FF000; ``MM`` is write-once.
  Real aliases cover $00000000 and $FF000000 (absolute short addresses);
  the other 254 mirrors are forwarded through an I/O region (data only,
  no code execution).  Chip selects: CSBOOT decodes the boot flash
  (``CSBARBT``/``CSORBT``, mirrored through the block when the flash is
  smaller, up to 16 copies); optional external RAM
  (``ext-ram-size``) is decoded by chip select ``ext-ram-cs`` or mapped
  at ``ext-ram-base`` when ``ext-ram-cs`` = -1.  A chip select decodes
  when its pin is assigned as a chip select, ``BYTE`` and ``R/W`` are not
  "disable" and ``SPACE`` is not CPU space; byte-lane, read/write-only,
  user/supervisor and wait-state options are not applied.  The other
  chip selects are stored only (their pins are not modelled).

SIM (5, D.2)
  ``SIMCR`` (IARB used for the PIT and IRQ pins), ``SYNCR``, ``RSR``
  (power-on, external, watchdog), ``SYPCR`` (write-once; software
  watchdog per 5.4.5 with time-out ``128 x 2^n / fref``, service $55/$AA;
  time-out resets the MCU and sets ``RSR.SW`` unless ``wdt-reset=off``;
  bus/halt monitor fields are stored only), ``PICR``/``PITR`` (periodic
  interrupt timer, ``128 x PITM x 4 (x512) / fref``; the request is
  negated by the interrupt acknowledge), ports E, F, C, chip-select
  registers; test registers are stored.  ``IRQ1``-``IRQ7`` are
  level-sensitive and always autovectored (vector 24+n; an external
  vector or chip-select AVEC needs external hardware), level 7 edge/NMI
  behaviour is handled by the cpu32 core.

Interrupt arbitration (5.8)
  The highest level wins, then the highest IARB; on the interrupt
  acknowledge the winning module supplies its vector.  A request whose
  module has IARB = 0 gives the spurious interrupt exception (vector
  24).  Equal IARB values (a programming error on the chip) resolve in
  registration order (PIT before IRQ pins, QSPI before SCI).
  User/supervisor restrictions (``SUPV``, ``RASP``, ``ASPC``) are not
  enforced.

Standby RAM, TPURAM and MRM (6, 12, 7)
  SRAM: 4 KiB, in low-power stop at reset, base in ``RAMBAH``/``RAMBAL``
  (writable only while stopped and unlocked).  TPURAM: 3.5 KiB, enabled
  by the first (and only) ``TRAMBAR`` write unless it overlaps the module
  block; inaccessible to the CPU in TPU emulation mode
  (``mc68376_tpuram_set_emulation()``).  ``TRAMMCR.STOP`` resets to 0
  as in the register diagram (D.9.1; the text in 12.7/12.8 says reset
  sets it).  MRM: generic blank-ROM reset values (enabled at $FF0000,
  ``BOOT`` = 1); the contents are factory masked and unknown, give an
  image with ``mrm-image=file`` (otherwise $FF), ``mrm-enabled=off``
  models DATA14 pulled low.  The bootstrap words are not used.

QSM (9, D.6)
  SCI on ``-serial`` 0 (``sci`` property): baud ``fsys / (32 SCBR)``,
  10/11-bit frames, parity generation/check, TE preamble, the
  read-SCSR-then-access-SCDR flag clearing, IDLE, receiver wake-up,
  LOOPS, ``kline-echo``.  Received bytes are held back while RDR is full
  (no overrun); break, noise and framing errors and the ninth data bit
  are not modelled.  QSPI master mode: the queue runs with the programmed
  SCK rate and delays, ``SPIF``/``HALTA``/``MODF``, wrap-around, HALT,
  ``LOOPQ``; PCS pins are driven on the ``PQSn`` pins.  With no device
  attached (``mc68376_qspi_attach()``) MISO reads as ones.  Slave mode
  and ``QSMCR.STOP`` are not modelled.

QADC (8)
  Placeholder: register window only (accesses logged as unimplemented).

CTM4 (10)
  Placeholder: register window only (accesses logged as unimplemented).

TPU (11)
  Placeholder: register window only (accesses logged as unimplemented).

TouCAN (13, D.10)
  Attached to ``-machine canbus0=<can-bus id>``.  16 message buffers
  (13.4.1) with the receive/transmit codes of Tables 13-2/13-3: receive
  into the lowest matching EMPTY buffer, else the lowest matching
  FULL/OVERRUN one (OVERRUN); acceptance masks ``RXGMSK`` (buffers
  0-13), ``RX14MSK``, ``RX15MSK`` (RTR never compared, IDE always);
  standard and extended IDs; remote frames trigger the buffers with
  code %1010 and an exact ID (13.5.5), a transmitted remote frame turns
  its buffer into an EMPTY receive buffer.  Lock by reading a
  control/status word, release by reading ``TIMER`` or locking another
  buffer; a frame for a locked buffer waits in the serial message buffer
  (13.5.4.2).  BUSY is never shown (transfers are instantaneous).
  Transmit arbitration: lowest ID first, or lowest buffer first with
  ``LBUF``.  A frame lasts its length at the programmed bit rate
  (``fsys / (PRESDIV+1)`` S-clock, ``4 + PROPSEG + PSEG1 + PSEG2`` time
  quanta per bit) without stuff bits: 47 + 8n bits (standard) or 67 + 8n
  (extended) including the intermission.  ``TIMER`` counts bits while
  the prescaler runs; time stamps (high byte in the control/status
  word, 16 bits in ``ID_LOW`` of standard buffers) are taken at the
  identifier field; ``TSYNC``.  Own frames are received into an EMPTY
  matching buffer (13.5.3.2); ``LOOP`` keeps frames off the bus and
  ignores it.  Every transmission is acknowledged and no bus errors are
  modelled: ``ESTAT`` reports error active (error passive / warning only
  from counter values written in debug mode), ``ACKERR`` and the other
  error bits, bus off and the error interrupt never occur; successful
  frames decrement the error counters.  ``CANMCR``: reset in debug mode
  (``HALT``, ``FRZ``, ``FRZACK``, ``NOTRDY``), entered after the current
  frame, left after 11 bit times; low-power ``STOP`` (only ``CANMCR``
  accessible, wake-up interrupt and ``SELFWAKE`` on a frame from the
  bus); ``SOFTRST``.  ``APS`` is stored only; the IMB FREEZE line, the
  test register ``CANTCR``, pin polarity (``RXMODE``/``TXMODE``),
  ``SAMP`` and ``RJW`` have no effect.  Interrupts: one request at
  ``ILCAN`` with ``IARB``, vector ``IVBA:source`` (buffers 0-15, bus
  off, error, wake-up; lowest number first, Table 13-9), $0F until
  ``CANICR`` is initialised.  Frames from the bus are received the
  moment the sender completes them, also while the TouCAN transmits (no
  arbitration against them).  ``-d int`` logs every frame.

Debugging
---------

* ``-d int`` logs every exception/interrupt with its vector, ``-d
  guest_errors,unimp`` reports accesses to unmodelled registers.
* ``-s -S`` starts the gdbstub.  For the SH705x use an ``sh-elf`` gdb
  (``set architecture sh2e``).  The M32C gdbstub uses the raw register
  layout of GDB's ``m32c`` target (both register banks, control and DMA
  registers), so ``m32c-elf-gdb`` can connect directly.

Tests
-----

``tests/ecu/run-tests.sh [build-dir]`` assembles small test ROMs
(``tests/ecu/shasm.py``, ``tests/ecu/m32casm.py``,
``tests/ecu/m68kasm.py``) and checks serial
output, A/D readings, interrupts, CAN transmit/receive and the injector
and coil pulse measurements on every machine, plus an M32C instruction
self test.
