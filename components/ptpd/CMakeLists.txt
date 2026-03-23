idf_component_register(SRCS "ptpd.c"
                       PRIV_REQUIRES esp_eth esp_netif
                       INCLUDE_DIRS "." "./include")

target_sources(${COMPONENT_LIB} PRIVATE "ptpd.c")
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-unused-function)

# Uncomment to disable PTP PI frequency discipline (free-running clock test)
# target_compile_definitions(${COMPONENT_LIB} PRIVATE AES67_DISABLE_PTP_PI)
