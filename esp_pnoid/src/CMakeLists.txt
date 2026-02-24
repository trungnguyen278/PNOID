FILE(GLOB_RECURSE app_sources
    ${CMAKE_SOURCE_DIR}/src/*.c
    ${CMAKE_SOURCE_DIR}/src/*.cpp
)

idf_component_register(
    SRCS ${app_sources}
    INCLUDE_DIRS "."
    PRIV_REQUIRES
        nvs_flash
        esp_wifi
        esp_event
        esp_netif
        mqtt
        bt
        driver
)
