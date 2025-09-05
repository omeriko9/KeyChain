# Copy the UI file into this component to ensure stable embed symbol names
# Ensure we copy the HTML from the project root into this component to embed it with a stable symbol name
if(NOT EXISTS ${CMAKE_CURRENT_LIST_DIR}/../spiffs/index.html)
    message(FATAL_ERROR "Expected file not found: ${CMAKE_CURRENT_LIST_DIR}/../spiffs/index.html")
endif()
configure_file(${CMAKE_CURRENT_LIST_DIR}/../spiffs/index.html ${CMAKE_CURRENT_LIST_DIR}/embedded_index.html COPYONLY)

idf_component_register(
    SRCS "main.cc" "tft.cc" "webserver.cc"
    PRIV_REQUIRES spi_flash lvgl esp_lcd esp_timer log driver esp_http_server esp_wifi nvs_flash esp_netif spiffs
    EMBED_TXTFILES embedded_index.html
)
# INCLUDE_DIRS "include"
