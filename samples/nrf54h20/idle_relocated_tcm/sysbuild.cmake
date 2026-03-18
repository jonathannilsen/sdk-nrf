# Ensure that remote edt.pickle is available when radio_loader CMake runs
sysbuild_add_dependencies(CONFIGURE radio_loader remote)
add_dependencies(radio_loader remote)

# CONFIG_NCS_MCUBOOT_PERIPHCONF_TLV is set to y from nrf/sysbuild/CMakeLists.txt
# and must be disabled explicitly to prevent the PERIPHCONF TLV script from running.
# Since the sysbuild generated .config takes priority over values from prj.conf this
# is done via Image configuration script.
set_property(TARGET remote APPEND PROPERTY IMAGE_CONF_SCRIPT
  ${APP_DIR}/sysbuild_disable_periphconf_tlv.cmake
)
