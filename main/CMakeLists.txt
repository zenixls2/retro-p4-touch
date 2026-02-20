set(RETRO_GO_REPOSITORY "https://github.com/ducalex/retro-go.git")
set(RETRO_GO_COMMIT "cfce0fef5a7a8da889915a87250de52ce5facb56")
set(RETRO_GO_PATCH_FILE "${CMAKE_CURRENT_LIST_DIR}/patches/retro-go-gbsp-memory.patch")
set(RETRO_GO_SOURCE_DIR "${CMAKE_BINARY_DIR}/_deps/retro-go-${RETRO_GO_COMMIT}")

find_package(Git REQUIRED)

if(NOT EXISTS "${RETRO_GO_SOURCE_DIR}/.git")
    file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/_deps")
    execute_process(
        COMMAND "${GIT_EXECUTABLE}" clone --filter=blob:none "${RETRO_GO_REPOSITORY}" "${RETRO_GO_SOURCE_DIR}"
        RESULT_VARIABLE RETRO_GO_CLONE_RESULT
        OUTPUT_VARIABLE RETRO_GO_CLONE_OUT
        ERROR_VARIABLE RETRO_GO_CLONE_ERR
    )

    if(NOT RETRO_GO_CLONE_RESULT EQUAL 0)
        message(FATAL_ERROR "Failed to clone retro-go:\n${RETRO_GO_CLONE_OUT}\n${RETRO_GO_CLONE_ERR}")
    endif()
endif()

execute_process(
    COMMAND "${GIT_EXECUTABLE}" -C "${RETRO_GO_SOURCE_DIR}" checkout --force "${RETRO_GO_COMMIT}"
    RESULT_VARIABLE RETRO_GO_CHECKOUT_RESULT
    OUTPUT_VARIABLE RETRO_GO_CHECKOUT_OUT
    ERROR_VARIABLE RETRO_GO_CHECKOUT_ERR
)

if(NOT RETRO_GO_CHECKOUT_RESULT EQUAL 0)
    execute_process(
        COMMAND "${GIT_EXECUTABLE}" -C "${RETRO_GO_SOURCE_DIR}" fetch origin "${RETRO_GO_COMMIT}"
        RESULT_VARIABLE RETRO_GO_FETCH_RESULT
        OUTPUT_VARIABLE RETRO_GO_FETCH_OUT
        ERROR_VARIABLE RETRO_GO_FETCH_ERR
    )

    if(NOT RETRO_GO_FETCH_RESULT EQUAL 0)
        message(FATAL_ERROR "Failed to fetch retro-go commit ${RETRO_GO_COMMIT}:\n${RETRO_GO_FETCH_OUT}\n${RETRO_GO_FETCH_ERR}")
    endif()

    execute_process(
        COMMAND "${GIT_EXECUTABLE}" -C "${RETRO_GO_SOURCE_DIR}" checkout --force "${RETRO_GO_COMMIT}"
        RESULT_VARIABLE RETRO_GO_CHECKOUT_RETRY_RESULT
        OUTPUT_VARIABLE RETRO_GO_CHECKOUT_RETRY_OUT
        ERROR_VARIABLE RETRO_GO_CHECKOUT_RETRY_ERR
    )

    if(NOT RETRO_GO_CHECKOUT_RETRY_RESULT EQUAL 0)
        message(FATAL_ERROR "Failed to checkout retro-go commit ${RETRO_GO_COMMIT}:\n${RETRO_GO_CHECKOUT_RETRY_OUT}\n${RETRO_GO_CHECKOUT_RETRY_ERR}")
    endif()
endif()

set(GPSP_DIR "${RETRO_GO_SOURCE_DIR}/gbsp/components/gbsp-libretro")
set(GBA_ROM_SOURCE_FILE "${PROJECT_DIR}/pokemon.gba")
set(GBA_ROM_ZLIB_FILE "${CMAKE_CURRENT_LIST_DIR}/pokemon.gba.zlib")
set(GBA_ROM_GEN_SCRIPT "${CMAKE_CURRENT_LIST_DIR}/tools/generate_pokemon_zlib.py")

if(NOT EXISTS "${GPSP_DIR}/gba_memory.c")
    message(FATAL_ERROR "retro-go source at ${RETRO_GO_COMMIT} does not contain gbsp-libretro")
endif()

if(NOT EXISTS "${RETRO_GO_PATCH_FILE}")
    message(FATAL_ERROR "Required patch file missing: ${RETRO_GO_PATCH_FILE}")
endif()

execute_process(
    COMMAND "${GIT_EXECUTABLE}" -C "${RETRO_GO_SOURCE_DIR}" apply --check "${RETRO_GO_PATCH_FILE}"
    RESULT_VARIABLE RETRO_GO_PATCH_CHECK_RESULT
    OUTPUT_VARIABLE RETRO_GO_PATCH_CHECK_OUT
    ERROR_VARIABLE RETRO_GO_PATCH_CHECK_ERR
)

if(RETRO_GO_PATCH_CHECK_RESULT EQUAL 0)
    execute_process(
        COMMAND "${GIT_EXECUTABLE}" -C "${RETRO_GO_SOURCE_DIR}" apply --whitespace=nowarn "${RETRO_GO_PATCH_FILE}"
        RESULT_VARIABLE RETRO_GO_PATCH_RESULT
        OUTPUT_VARIABLE RETRO_GO_PATCH_OUT
        ERROR_VARIABLE RETRO_GO_PATCH_ERR
    )

    if(NOT RETRO_GO_PATCH_RESULT EQUAL 0)
        message(FATAL_ERROR "Failed to patch fetched retro-go source:\n${RETRO_GO_PATCH_OUT}\n${RETRO_GO_PATCH_ERR}")
    endif()
else()
    execute_process(
        COMMAND "${GIT_EXECUTABLE}" -C "${RETRO_GO_SOURCE_DIR}" apply --reverse --check "${RETRO_GO_PATCH_FILE}"
        RESULT_VARIABLE RETRO_GO_PATCH_REVERSE_RESULT
        OUTPUT_VARIABLE RETRO_GO_PATCH_REVERSE_OUT
        ERROR_VARIABLE RETRO_GO_PATCH_REVERSE_ERR
    )

    if(NOT RETRO_GO_PATCH_REVERSE_RESULT EQUAL 0)
        message(FATAL_ERROR "retro-go patch state is invalid:\n${RETRO_GO_PATCH_CHECK_OUT}\n${RETRO_GO_PATCH_CHECK_ERR}\n${RETRO_GO_PATCH_REVERSE_OUT}\n${RETRO_GO_PATCH_REVERSE_ERR}")
    endif()
endif()

if(NOT EXISTS "${GBA_ROM_GEN_SCRIPT}")
    message(FATAL_ERROR "Missing ROM generator script: ${GBA_ROM_GEN_SCRIPT}")
endif()

set(GBA_ROM_GEN_TARGET "")
if(EXISTS "${GBA_ROM_SOURCE_FILE}")
    idf_build_get_property(idf_python PYTHON)
    if(NOT idf_python)
        message(FATAL_ERROR "Python interpreter from ESP-IDF is unavailable")
    endif()

    if(NOT CMAKE_SCRIPT_MODE_FILE)
        add_custom_command(
            OUTPUT "${GBA_ROM_ZLIB_FILE}"
            COMMAND "${idf_python}" "${GBA_ROM_GEN_SCRIPT}" "${GBA_ROM_SOURCE_FILE}" "${GBA_ROM_ZLIB_FILE}"
            DEPENDS "${GBA_ROM_SOURCE_FILE}" "${GBA_ROM_GEN_SCRIPT}"
            COMMENT "Generating pokemon.gba.zlib from project-root pokemon.gba"
            VERBATIM
        )

        set(GBA_ROM_GEN_TARGET generate_pokemon_gba_zlib)
        add_custom_target(${GBA_ROM_GEN_TARGET} DEPENDS "${GBA_ROM_ZLIB_FILE}")
    endif()
elseif(NOT EXISTS "${GBA_ROM_ZLIB_FILE}")
    message(FATAL_ERROR "Missing ${GBA_ROM_ZLIB_FILE}. Provide ${GBA_ROM_SOURCE_FILE} to auto-generate it.")
endif()

idf_component_register(
    SRCS
        "main.c"
        "${GPSP_DIR}/cheats.c"
        "${GPSP_DIR}/cpu_threaded.c"
        "${GPSP_DIR}/cpu.cpp"
        "${GPSP_DIR}/gba_cc_lut.c"
        "${GPSP_DIR}/gba_memory.c"
        "${GPSP_DIR}/gbp.c"
        "${GPSP_DIR}/input.c"
        "${GPSP_DIR}/main.c"
        "${GPSP_DIR}/memmap.c"
        "${GPSP_DIR}/rfu.c"
        "${GPSP_DIR}/savestate.c"
        "${GPSP_DIR}/serial.c"
        "${GPSP_DIR}/sound.c"
        "${GPSP_DIR}/video.cpp"
    INCLUDE_DIRS
        "."
        "${GPSP_DIR}"
        "${GPSP_DIR}/libretro/libretro-common/include"
    REQUIRES
        esp_driver_ppa
        esp_wifi
        esp_netif
        esp_event
        nvs_flash
        esp_http_client
        fatfs
        sdmmc
        bt
    EMBED_FILES
        "pokemon.gba.zlib"
        "open_gba_bios.bin"
)

idf_component_get_property(FATFS_COMPONENT_LIB fatfs COMPONENT_LIB)
target_compile_options(${FATFS_COMPONENT_LIB} PRIVATE
    -include
    "${CMAKE_CURRENT_LIST_DIR}/fatfs_exfat_override.h"
)

set_source_files_properties(
    "${GPSP_DIR}/cpu.cpp"
    "${GPSP_DIR}/video.cpp"
    PROPERTIES COMPILE_OPTIONS "-Wno-error"
)

target_compile_options(${COMPONENT_LIB} PRIVATE
    -Wno-error
    -Wno-error=maybe-uninitialized
    -Wno-format
    -Wno-sign-compare
    -Wno-maybe-uninitialized
    -Wno-unused-function
    -Wno-unused-variable
    -Wno-unused-but-set-variable
    -Wno-missing-field-initializers
)

if(GBA_ROM_GEN_TARGET)
    add_dependencies(${COMPONENT_LIB} ${GBA_ROM_GEN_TARGET})
endif()
