idf_component_register(
    SRC_DIRS "."
    INCLUDE_DIRS "."
)

if (CMAKE_COMPILER_IS_GNUCC)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-missing-field-initializers -Wno-switch -Wno-deprecated-enum-enum-conversion")
endif()

# The custom target depends on a non-existing file to ensure that the
# secrets are refreshed form the environment variables always.

add_custom_target(
    generate_secrets_header ALL
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/../build/config/secrets__SENTINAL.h
)

add_custom_command(
    OUTPUT
        ${CMAKE_CURRENT_SOURCE_DIR}/../build/config/secrets__SENTINAL.h
        ${CMAKE_CURRENT_SOURCE_DIR}/../build/config/secrets.h
    COMMAND python "${CMAKE_CURRENT_SOURCE_DIR}/../scripts/generate-secrets.py" "${CMAKE_CURRENT_SOURCE_DIR}/../build/config/secrets.h"
)
