m68k_ss = ss.source_set()
m68k_ss.add(when: 'CONFIG_AN5206', if_true: files('an5206.c', 'mcf5206.c'))
m68k_ss.add(when: 'CONFIG_MCF5208', if_true: files('mcf5208.c', 'mcf_intc.c'))
m68k_ss.add(when: 'CONFIG_NEXTCUBE', if_true: files('next-kbd.c', 'next-cube.c'))
m68k_ss.add(when: 'CONFIG_Q800', if_true: files('q800.c', 'q800-glue.c'))
m68k_ss.add(when: 'CONFIG_M68K_VIRT', if_true: files('virt.c'))
m68k_ss.add(when: 'CONFIG_CPU32_TEST', if_true: files('cpu32-test.c'))
m68k_ss.add(when: 'CONFIG_ECU_MC68376', if_true: files('ecu_mc68376.c', 'mc68376.c',
  'mc68376_qsm.c', 'mc68376_qadc.c', 'mc68376_ctm4.c', 'mc68376_tpu.c',
  'mc68376_toucan.c'))

hw_arch += {'m68k': m68k_ss}
