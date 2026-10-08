#!/bin/sh
# Smoke tests for the ECU emulator machines.
#
#   tests/ecu/run-tests.sh [build-dir]
#
# Assembles the test ROMs, runs every ECU machine for a few seconds and
# checks the firmware's serial output and the engine simulator's
# measurements.
# SPDX-License-Identifier: GPL-2.0-or-later

BUILD=${1:-build}
HERE=$(cd "$(dirname "$0")" && pwd)
TMP=$(mktemp -d)
trap 'rm -rf "$TMP"' EXIT
fail=0

check() {
    # check <name> <file> <pattern>
    if grep -q -- "$3" "$2"; then
        echo "PASS $1: $3"
    else
        echo "FAIL $1: '$3' not found in:"
        sed 's/^/    /' "$2"
        fail=1
    fi
}

run() {
    # run <qemu> <machine> <rom> <tag> [extra args...]
    qemu=$1 machine=$2 rom=$3 tag=$4
    shift 4
    timeout 4 "$BUILD/$qemu" -M "$machine" -bios "$rom" -display none \
        -monitor none -chardev file,id=s0,path="$TMP/$tag.out" \
        -serial chardev:s0 \
        -chardev file,id=ecu,path="$TMP/$tag.ecu" \
        -global ecu-engine.chardev=ecu \
        -global ecu-engine.script="set rpm 3000;stream 500" \
        -object can-bus,id=can0 "$@" >/dev/null 2>&1
}

if [ -x "$BUILD/qemu-system-sh4eb" ]; then
    for v in 7058 7055 7054 7052; do
        python3 "$HERE/sh7058_test_rom.py" "$TMP/sh$v.bin" $v || exit 1
        run qemu-system-sh4eb ecu-sh$v "$TMP/sh$v.bin" sh$v \
            -machine canbus0=can0,canbus1=can0
        check sh$v "$TMP/sh$v.out" "BOOT"
        check sh$v "$TMP/sh$v.out" "00000072 0000006B CANT"
        check sh$v "$TMP/sh$v.out" "^T0000"
        check sh$v "$TMP/sh$v.ecu" "inj pw=2.500ms"
        check sh$v "$TMP/sh$v.ecu" "dwell=3.00ms"
        case $v in
        7058|7055) check sh$v "$TMP/sh$v.out" "RX1=51454D55" ;;
        esac
        case $v in
        7058|7055|7054) check sh$v "$TMP/sh$v.out" "FLASH OK" ;;
        esac
    done
    # SH7055 with 180 nm flash (download method), as on the SH7058
    python3 "$HERE/sh7058_test_rom.py" "$TMP/sh7055s.bin" 7055 180 || exit 1
    run qemu-system-sh4eb ecu-sh7055 "$TMP/sh7055s.bin" sh7055s \
        -global sh705x-soc.flash-node=180
    check sh7055-180nm "$TMP/sh7055s.out" "FLASH OK"
fi

if [ -x "$BUILD/qemu-system-m32c" ]; then
    python3 "$HERE/m32c87_test_rom.py" "$TMP/m32c.bin" || exit 1
    run qemu-system-m32c ecu-m32c87 "$TMP/m32c.bin" m32c \
        -machine canbus0=can0,canbus1=can0
    check m32c87 "$TMP/m32c.out" "BOOT"
    check m32c87 "$TMP/m32c.out" "0072 006B CANT"
    check m32c87 "$TMP/m32c.out" "RX1=4551"
    check m32c87 "$TMP/m32c.ecu" "inj pw=2.500ms"

    python3 "$HERE/m32c_cpu_test_rom.py" "$TMP/m32ccpu.bin" || exit 1
    run qemu-system-m32c ecu-m32c87 "$TMP/m32ccpu.bin" m32ccpu
    check m32c-cpu "$TMP/m32ccpu.out" "CPU OK"
fi

if [ -x "$BUILD/qemu-system-m68k" ]; then
    # MC68376: SIM (clock, PIT, watchdog reset, ports, IRQ pins, CSBOOT),
    # SRAM/TPURAM relocation, IMB arbitration, SCI and QSPI
    python3 "$HERE/mc68376_test_rom.py" "$TMP/mc68376.bin" || exit 1
    run qemu-system-m68k ecu-mc68376 "$TMP/mc68376.bin" mc68376 \
        -machine canbus0=can0
    for p in "MC68376 BOOT RSR=40 SIMCR=00CF" "SRAM OK" "TPURAM OK" \
             "PE=A5 PF=5A" "IRQ2 OK" "QSPI=1234" "MIRROR OK" \
             "SPURIOUS OK" "SCI IRQ OK" "PIT OK" "RATE OK" "WDT TEST" \
             "MC68376 BOOT RSR=20" "DONE"; do
        check mc68376 "$TMP/mc68376.out" "$p"
    done
fi

if [ -x "$BUILD/qemu-system-m68k" ]; then
    # CPU32 core self-test (TBLx, LPSTOP, exception frames, VBR, MULx.L/
    # DIVx.L, CHK2/CMP2, address/bus errors, interrupts) on the minimal
    # cpu32-test machine
    python3 "$HERE/cpu32_test_rom.py" "$TMP/cpu32.bin" || exit 1
    run qemu-system-m68k cpu32-test "$TMP/cpu32.bin" cpu32
    check cpu32 "$TMP/cpu32.out" "CPU32 OK"
fi

exit $fail
