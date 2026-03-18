# Don't generate PERIPHCONF TLV for remote when building with split slots
set_config_bool(${ZCMAKE_APPLICATION} CONFIG_NCS_MCUBOOT_PERIPHCONF_TLV n)
