set(RAW_HTML "${CMAKE_CURRENT_LIST_DIR}/www/index.html")
set(EMBEDDED_HTML_C "${CMAKE_CURRENT_BINARY_DIR}/embedded_html.c")

# Guard code-generation commands so they are not executed during ESP-IDF's
# early "get requirements" script phase (where add_custom_command is disallowed).
if(NOT CMAKE_SCRIPT_MODE_FILE)
    # Use a small Python helper to reliably generate quoted C string lines per input line
    find_package(Python3 COMPONENTS Interpreter REQUIRED)

    add_custom_command(
        OUTPUT "${EMBEDDED_HTML_C}"
        COMMAND ${Python3_EXECUTABLE} "${CMAKE_CURRENT_LIST_DIR}/tools/embed_html.py" "${RAW_HTML}" "${EMBEDDED_HTML_C}"
        DEPENDS "${RAW_HTML}" "${CMAKE_CURRENT_LIST_DIR}/tools/embed_html.py"
        COMMENT "Generating embedded_html.c from ${RAW_HTML}"
        VERBATIM
    )
endif()

idf_component_register(SRCS "app_main.c" "${EMBEDDED_HTML_C}"
                    PRIV_REQUIRES mqtt nvs_flash esp_netif esp_wifi wifi_provisioning esp_http_server
                    INCLUDE_DIRS "."
                    REQUIRES driver)
