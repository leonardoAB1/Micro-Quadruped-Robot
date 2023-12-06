idf_component_register(SRCS "main.c"
    logging/logging_utils.c
    connect_wifi/connect_wifi.c
    http_handlers/http_handlers.c
    web_server/web_server.c
    gpio_utils/gpio_utils.c
    servo_driver/servo_driver.c
    kinematics/forwardKinematics.c
    INCLUDE_DIRS "")

# Set the paths for the configuration and header files
set(CONFIG_FILE "${CMAKE_CURRENT_SOURCE_DIR}/../config/wifi.config")
set(HEADER_FILE "${CMAKE_CURRENT_SOURCE_DIR}/config.h")

# Read the SSID and password from the configuration file
file(STRINGS ${CONFIG_FILE} CONFIG_CONTENTS)

foreach(CONFIG_LINE ${CONFIG_CONTENTS})
    string(REGEX MATCH "^SSID=(.*)" SSID_MATCH ${CONFIG_LINE})
    string(REGEX MATCH "^PASSWORD=(.*)" PASSWORD_MATCH ${CONFIG_LINE})

    if (SSID_MATCH)
        string(REGEX REPLACE "^SSID=(.*)" "\\1" SSID ${CONFIG_LINE})
    elseif (PASSWORD_MATCH)
        string(REGEX REPLACE "^PASSWORD=(.*)" "\\1" PASSWORD ${CONFIG_LINE})
    endif()
endforeach()

# Generate the header file with preprocessor directives
file(WRITE ${HEADER_FILE} "#ifndef CONFIG_H_\n")
file(APPEND ${HEADER_FILE} "#define CONFIG_H_\n")
file(APPEND ${HEADER_FILE} "\n")
file(APPEND ${HEADER_FILE} "#define WIFI_SSID \"${SSID}\"\n")
file(APPEND ${HEADER_FILE} "#define WIFI_PASSWORD \"${PASSWORD}\"\n")
file(APPEND ${HEADER_FILE} "\n")
file(APPEND ${HEADER_FILE} "#endif\n")