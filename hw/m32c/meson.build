m32c_ss = ss.source_set()
m32c_ss.add(when: 'CONFIG_ECU_M32C87', if_true: files(
  'm32c87.c',
  'ecu_m32c87.c',
))

hw_arch += {'m32c': m32c_ss}
