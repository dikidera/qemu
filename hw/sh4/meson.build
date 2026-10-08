sh4_ss = ss.source_set()
sh4_ss.add(when: 'CONFIG_SH7750', if_true: files(
  'sh7750.c',
  'sh7750_regnames.c',
))
sh4_ss.add(when: 'CONFIG_R2D', if_true: files('r2d.c'))
sh4_ss.add(when: 'CONFIG_ECU_SH705X', if_true: files(
  'sh705x.c',
  'sh705x_atu.c',
  'sh705x_hcan.c',
  'sh705x_flash.c',
  'ecu_sh705x.c',
))

hw_arch += {'sh4': sh4_ss}
