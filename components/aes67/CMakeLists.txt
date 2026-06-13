idf_component_register(
    SRCS
        "src/aes67.c"
        "src/aes67_ptp.c"
        "src/aes67_rtp.c"
        "src/aes67_sap.c"
        "src/aes67_sdp.c"
        "src/aes67_session.c"
        "src/aes67_audio.c"
        "src/aes67_convert.c"
        "src/aes67_net.c"
        "src/aes67_mdns.c"
        "src/aes67_hw_timer.c"
        "src/aes67_l2tap_rx.c"
        "src/aes67_webui.c"
        # Bundled third-party: a minimal IEEE-1588 PTP daemon (Apache NuttX
        # ptpd, ported to ESP-IDF by Espressif) and the EMAC time wrapper from
        # the ESP-IDF ethernet/ptp example. Vendored here so the component is
        # self-contained on the Component Registry. See third_party/*/README.md
        # and the SPDX headers for provenance and licensing.
        "third_party/ptpd/ptpd.c"
        "third_party/esp_eth_time/esp_eth_time.c"
    INCLUDE_DIRS
        "include"
    PRIV_INCLUDE_DIRS
        "private_include"
        "third_party/ptpd"
        "third_party/ptpd/include"
        "third_party/esp_eth_time"
    PRIV_REQUIRES
        esp_eth
        esp_netif
        esp_timer
        esp_driver_i2s
        mdns
        lwip
        esp_psram
        esp_mm
        esp_http_server
    EMBED_FILES
        "web/index.html.gz"
)

# ptpd.c is a third-party port with intentionally-unused helpers behind
# Kconfig; silence that one warning for the bundled file only.
set_source_files_properties(
    third_party/ptpd/ptpd.c
    PROPERTIES COMPILE_OPTIONS "-Wno-unused-function"
)

# Gzip the web frontend for embedding in flash
add_custom_command(
    OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/web/index.html.gz
    COMMAND gzip -9 -k -f ${CMAKE_CURRENT_SOURCE_DIR}/web/index.html
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/web/index.html
    COMMENT "Compressing web/index.html -> index.html.gz"
)
