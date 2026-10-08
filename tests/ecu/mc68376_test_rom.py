#!/usr/bin/env python3
# Build a small MC68376 test ROM (512 KiB boot flash image on CSBOOT at 0).
#
#   SYNCR  : 16.78 MHz (X = 1, Y = 63 from the 4.194 MHz reference)
#   SIMCR  : MM written twice (second write must be ignored)
#   SRAM   : relocated to $100000, stack and variables
#   TPURAM : TRAMBAR -> $101000, pattern test
#   SCI    : 9600 baud 8N1, banner and results
#   SYPCR  : watchdog 62.5 ms, serviced with $55/$AA; on the first boot
#            the service stops once to provoke a watchdog reset, the
#            second boot must report RSR = $20 (SW)
#   PIT    : ~1 ms, level 6, vector $40, counts ticks; tick rate checked
#            against the SCI transmit time
#   ports  : PE/PF driven as outputs and read back as inputs; PF2 low
#            with PFPAR2 set must raise an IRQ2 autovector interrupt
#   QSPI   : one 16-bit transfer in loopback (LOOPQ)
#
# The module block is accessed with absolute short addresses ($FFxx sign
# extends to $FFFFFFxx), which exercises the 24-bit bus mirroring.
#
# SPDX-License-Identifier: GPL-2.0-or-later
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from m68kasm import *  # noqa: E402,F403

# module registers (MM = 1), as absolute short addresses
SIMCR = absw(0xfa00)
SYNCR = absw(0xfa04)
RSR = absw(0xfa07)
PORTE = absw(0xfa11)
DDRE = absw(0xfa15)
PEPAR = absw(0xfa17)
PORTF = absw(0xfa19)
DDRF = absw(0xfa1d)
PFPAR = absw(0xfa1f)
SYPCR = absw(0xfa21)
PICR = absw(0xfa22)
PITR = absw(0xfa24)
SWSR = absw(0xfa27)
TRAMBAR = absw(0xfb04)
RAMMCR = absw(0xfb40)
RAMBAH = absw(0xfb44)
RAMBAL = absw(0xfb46)
SCCR0 = absw(0xfc08)
SCCR1 = absw(0xfc0a)
SCSR = absw(0xfc0c)
SCDR = absw(0xfc0e)
SPCR0 = absw(0xfc18)
SPCR1 = absw(0xfc1a)
SPCR2 = absw(0xfc1c)
SPCR3 = absw(0xfc1e)
SPSR = absw(0xfc1f)
TR0 = absw(0xfd20)
RR0 = absw(0xfd00)

SRAM = 0x100000
TPURAM = 0x101000
TICKS = absl(SRAM + 0x10)
IRQ2F = absl(SRAM + 0x14)
T0 = absl(SRAM + 0x18)
SPURF = absl(SRAM + 0x1c)
SCIF = absl(SRAM + 0x20)
QSMCR = absw(0xfc00)
QILR = absw(0xfc04)

a = Asm(0, 0x80000)

# vector table: SSP, PC, then everything to 'unexpected'
a.org(0)
a.dc_l(SRAM + 0x1000, 'start')
for v in range(2, 256):
    a.dc_l('unexpected')
a.org(26 * 4)                   # level 2 autovector
a.dc_l('irq2_isr')
a.org(0x40 * 4)                 # PIT vector
a.dc_l('pit_isr')
a.org(24 * 4)                   # spurious interrupt
a.dc_l('spur_isr')
a.org(0x50 * 4)                 # QSM: SCI (INTV0 = 0)
a.dc_l('sci_isr')

a.org(0x1000)
a.label('start')
# watchdog: SWE = 1, SWT = %01 -> 2^11 * 128 / 4.194304 MHz = 62.5 ms
a.move_b(imm(0x90), SYPCR)
# 16.78 MHz
a.move_w(imm(0x7f00), SYNCR)
# SIMCR: MM stays 1, IARB = $F; the second MM write must be ignored
a.move_w(imm(0x00cf), SIMCR)
a.move_w(imm(0x008f), SIMCR)

# SRAM to $100000, out of low-power stop
a.move_w(imm(SRAM >> 16), RAMBAH)
a.move_w(imm(SRAM & 0xffff), RAMBAL)
a.move_w(imm(0x0000), RAMMCR)
a.lea(absl(SRAM + 0x1000), SP)
a.move_l(imm(0x5aa5c33c), absl(SRAM))
a.clr_l(TICKS)
a.clr_l(IRQ2F)

# SCI: SCBR = 16777216 / (32 * 9600) = 55, TE | RE
a.move_w(imm(55), SCCR0)
a.move_w(imm(0x000c), SCCR1)
a.lea(absl('s_boot'), A0)
a.bsr('puts')
a.move_b(RSR, D0)
a.bsr('puthex2')
a.lea(absl('s_simcr'), A0)
a.bsr('puts')
a.move_w(SIMCR, D0)
a.bsr('puthex4')
a.bsr('newline')

# SRAM check
a.cmpi_l(0x5aa5c33c, absl(SRAM))
a.bne('sram_bad')
a.lea(absl('s_sram'), A0)
a.bsr('puts')
a.label('sram_bad')

# TPURAM: base $101000 (TRAMBAR[15:4] = ADDR[23:12]), RAMDS must clear
a.move_w(imm(TPURAM >> 8), TRAMBAR)
a.move_w(TRAMBAR, D0)
a.btst(0, D0)
a.bne('tpuram_bad')
a.move_l(imm(0x13572468), absl(TPURAM + 0xdfc))
a.cmpi_l(0x13572468, absl(TPURAM + 0xdfc))
a.bne('tpuram_bad')
a.lea(absl('s_tpuram'), A0)
a.bsr('puts')
a.label('tpuram_bad')

# PIT: level 6, vector $40; PITM = 8 -> 128 * 8 * 4 / fref = 976.6 us
a.move_w(imm(0x0640), PICR)
a.move_w(imm(0x0008), PITR)
a.move_w(imm(0x2000), SR)

# port E: drive $A5, then read the pins back as inputs
a.move_b(imm(0x00), PEPAR)
a.move_b(imm(0xa5), PORTE)
a.move_b(imm(0xff), DDRE)
a.move_b(imm(0x00), DDRE)
a.lea(absl('s_pe'), A0)
a.bsr('puts')
a.move_b(PORTE, D0)
a.bsr('puthex2')
# port F: drive $5A, read back
a.move_b(imm(0x5a), PORTF)
a.move_b(imm(0xff), DDRF)
a.move_b(imm(0x00), PFPAR)
a.move_b(imm(0x00), DDRF)
a.lea(absl('s_pf'), A0)
a.bsr('puts')
a.move_b(PORTF, D0)
a.bsr('puthex2')
a.bsr('newline')
# PF2 is low: assigning it to IRQ2 requests a level 2 interrupt
a.move_b(imm(0x04), PFPAR)
a.move_l(TICKS, D1)                      # time out after ~30 ms
a.addi_l(30, D1)
a.label('irq2_wait')
a.tst_l(IRQ2F)
a.bne('irq2_ok')
a.bsr('service')
a.cmp_l(TICKS, D1)
a.bhi('irq2_wait')
a.bra('irq2_done')
a.label('irq2_ok')
a.lea(absl('s_irq2'), A0)
a.bsr('puts')
a.label('irq2_done')

# QSPI loopback: master, 16 bits, SPBR = 8, one entry, LOOPQ
a.move_w(imm(0x1234), TR0)
a.move_b(imm(0x40), absw(0xfd40))         # CR0: BITSE
a.move_w(imm(0x8008), SPCR0)
a.move_w(imm(0x0000), SPCR2)
a.move_w(imm(0x0400), SPCR3)
a.move_w(imm(0x8404), SPCR1)              # SPE
a.label('qspi_wait')
a.bsr('service')
a.btst(7, SPSR)
a.beq('qspi_wait')
a.lea(absl('s_qspi'), A0)
a.bsr('puts')
a.move_w(RR0, D0)
a.bsr('puthex4')
a.bsr('newline')

# 24-bit bus: SIMCR seen through a forwarded mirror ($12FFFA00)
a.move_w(absl(0x12fffa00), D0)
a.cmpi_w(0x00cf, D0)
a.bne('mirror_bad')
# CSBOOT: 512 KiB flash repeats in its 1 MiB block
a.move_l(absl(0x80004), D0)
a.cmp_l(absl(0x4), D0)
a.bne('mirror_bad')
a.lea(absl('s_mirror'), A0)
a.bsr('puts')
a.label('mirror_bad')

# IMB arbitration: SCI TDRE interrupt at level 3 with QSM IARB = 0 must
# end as a spurious interrupt; with IARB = 5 the QSM supplies vector $50
a.clr_l(SPURF)
a.clr_l(SCIF)
a.move_w(imm(0x0350), QILR)               # ILSCI = 3, QIVR = $50
a.move_w(imm(0x0000), QSMCR)              # IARB = 0
a.move_w(imm(0x008c), SCCR1)              # TIE | TE | RE
a.move_l(TICKS, D1)                      # time out after ~30 ms
a.addi_l(30, D1)
a.label('spur_wait')
a.tst_l(SPURF)
a.bne('spur_ok')
a.bsr('service')
a.cmp_l(TICKS, D1)
a.bhi('spur_wait')
a.bra('spur_done')
a.label('spur_ok')
a.lea(absl('s_spur'), A0)
a.bsr('puts')
a.label('spur_done')
a.move_w(imm(0x0005), QSMCR)              # IARB = 5
a.move_w(imm(0x008c), SCCR1)
a.move_l(TICKS, D1)                      # time out after ~30 ms
a.addi_l(30, D1)
a.label('sci_wait')
a.tst_l(SCIF)
a.bne('sci_ok')
a.bsr('service')
a.cmp_l(TICKS, D1)
a.bhi('sci_wait')
a.bra('sci_done')
a.label('sci_ok')
a.lea(absl('s_sciirq'), A0)
a.bsr('puts')
a.label('sci_done')

# PIT: wait for 200 ticks
a.label('pit_wait')
a.bsr('service')
a.cmpi_l(200, TICKS)
a.blo('pit_wait')
a.lea(absl('s_pit'), A0)
a.bsr('puts')

# PIT rate vs SCI: 20 characters at 9532 baud take ~21 ms
a.move_l(TICKS, T0)
a.lea(absl('s_20'), A0)
a.bsr('puts')
a.move_l(TICKS, D0)
a.sub_l(T0, D0)
a.cmpi_l(17, D0)
a.blo('rate_bad')
a.cmpi_l(25, D0)
a.bhi('rate_bad')
a.lea(absl('s_rate'), A0)
a.bsr('puts')
a.label('rate_bad')

# first boot: stop servicing the watchdog
a.btst(5, RSR)
a.bne('second_boot')
a.lea(absl('s_wdt'), A0)
a.bsr('puts')
a.label('hang')
a.bra('hang')

a.label('second_boot')
a.lea(absl('s_done'), A0)
a.bsr('puts')
a.label('idle')
a.bsr('service')
a.bra('idle')

# --- subroutines ------------------------------------------------------
a.label('service')
a.move_b(imm(0x55), SWSR)
a.move_b(imm(0xaa), SWSR)
a.rts()

# putc D0.b: wait TDRE (reading SCSR arms the clear), write SCDR
a.label('putc')
a.bsr('service')
a.btst(0, SCSR)                 # TDRE = SCSR bit 8 (high byte bit 0)
a.beq('putc')
a.andi_w(0xff, D0)
a.move_w(D0, SCDR)
a.rts()

a.label('puts')
a.move_b(postinc(A0), D0)
a.beq('puts_end')
a.bsr('putc')
a.bra('puts')
a.label('puts_end')
a.rts()

a.label('newline')
a.moveq(10, D0)
a.bra('putc')

a.label('puthex4')
a.move_w(D0, D2)
a.lsr_w(8, D0)
a.bsr('puthex2')
a.move_w(D2, D0)
# fall through
a.label('puthex2')
a.move_b(D0, D3)
a.lsr_b(4, D0)
a.bsr('nibble')
a.move_b(D3, D0)
a.label('nibble')
a.andi_w(0x0f, D0)
a.move_b(pcidx('hexdig', D0, 'w', 1), D0)
a.bra('putc')
a.label('hexdig')
a.ascii('0123456789ABCDEF')
a.align(2)

a.label('pit_isr')
a.addq_l(1, TICKS)
a.rte()

a.label('irq2_isr')
a.move_b(imm(0x00), PFPAR)      # release the request
a.move_l(imm(1), IRQ2F)
a.rte()

a.label('spur_isr')
a.move_w(imm(0x000c), SCCR1)    # TIE off: drop the request
a.move_l(imm(1), SPURF)
a.rte()

a.label('sci_isr')
a.move_w(imm(0x000c), SCCR1)
a.move_l(imm(1), SCIF)
a.rte()

a.label('unexpected')
a.lea(absl('s_exc'), A0)
a.bsr('puts')
a.label('dead')
a.bra('dead')

for name, text in (('s_boot', 'MC68376 BOOT RSR='), ('s_simcr', ' SIMCR='),
                   ('s_sram', 'SRAM OK\n'), ('s_tpuram', 'TPURAM OK\n'),
                   ('s_pe', 'PE='), ('s_pf', ' PF='),
                   ('s_irq2', 'IRQ2 OK\n'), ('s_qspi', 'QSPI='),
                   ('s_pit', 'PIT OK\n'), ('s_20', '01234567890123456789'),
                   ('s_rate', '\nRATE OK\n'), ('s_wdt', 'WDT TEST\n'),
                   ('s_done', 'DONE\n'),
                   ('s_mirror', 'MIRROR OK\n'), ('s_spur', 'SPURIOUS OK\n'),
                   ('s_sciirq', 'SCI IRQ OK\n'), ('s_exc', 'UNEXPECTED EXCEPTION\n')):
    a.label(name)
    a.asciz(text)
    a.align(2)

with open(sys.argv[1] if len(sys.argv) > 1 else 'mc68376.bin', 'wb') as f:
    f.write(a.bytes())
