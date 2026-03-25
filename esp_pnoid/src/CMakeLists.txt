# Enable C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Collect src .cpp sources only (lib/ is handled by PlatformIO separately)
file(GLOB_RECURSE app_sources
    ${CMAKE_SOURCE_DIR}/src/*.cpp
)

idf_component_register(
    SRCS ${app_sources}
    INCLUDE_DIRS
        "${CMAKE_SOURCE_DIR}/src"
        "${CMAKE_SOURCE_DIR}/include"
        "${CMAKE_SOURCE_DIR}/lib"
        "${CMAKE_SOURCE_DIR}/lib/network"
        "${CMAKE_SOURCE_DIR}/lib/spi"
    REQUIRES
        driver
        esp_wifi
        esp_event
        nvs_flash
        esp_http_server
        esp_websocket_client
        mqtt
        json
        bt
)
