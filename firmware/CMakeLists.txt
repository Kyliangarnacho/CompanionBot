cmake_minimum_required(VERSION 3.20)
project(companionbot_firmware LANGUAGES C ASM)
get_filename_component(CB_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/.." ABSOLUTE)
set(STM32CUBE_G4_ROOT "${CB_ROOT}/.venv/embedded/upstream/STM32CubeG4" CACHE PATH "External official CubeG4 v1.6.1")
set(CB_DEVICE "${STM32CUBE_G4_ROOT}/Drivers/CMSIS/Device/ST/STM32G4xx")
set(CB_HAL "${STM32CUBE_G4_ROOT}/Drivers/STM32G4xx_HAL_Driver")
if(NOT EXISTS "${CB_HAL}/Src/stm32g4xx_hal.c")
  message(FATAL_ERROR "Fetch pinned upstream with tools/embedded/fetch_upstream.py first")
endif()
set(CB_BUILD_TESTS OFF CACHE BOOL "No host tests in firmware" FORCE)
set(BUILD_SHARED_LIBS OFF)
add_subdirectory("${CB_ROOT}/ros2_ws/src/companionbot_control/native" core)
set(CB_BOARD "${CMAKE_CURRENT_SOURCE_DIR}/boards/nucleo_g474re")
add_executable(companionbot_g474
  ${CB_BOARD}/Core/Src/main.c ${CB_BOARD}/Core/Src/stm32g4xx_hal_msp.c
  ${CB_BOARD}/Core/Src/stm32g4xx_it.c ${CB_BOARD}/board_io.c
  app/app.c
  ${CB_DEVICE}/Source/Templates/system_stm32g4xx.c
  ${CB_DEVICE}/Source/Templates/gcc/startup_stm32g474xx.s)
foreach(module hal hal_cortex hal_dma hal_dma_ex hal_exti hal_flash hal_flash_ex hal_flash_ramfunc hal_gpio hal_iwdg hal_pwr hal_pwr_ex hal_rcc hal_rcc_ex hal_tim hal_tim_ex hal_uart hal_uart_ex)
  target_sources(companionbot_g474 PRIVATE "${CB_HAL}/Src/stm32g4xx_${module}.c")
endforeach()
target_compile_features(companionbot_g474 PRIVATE c_std_99)
target_compile_definitions(companionbot_g474 PRIVATE USE_HAL_DRIVER STM32G474xx)
target_include_directories(companionbot_g474 PRIVATE
  ${CB_BOARD}/Core/Inc ${CB_BOARD} app generated ${CB_HAL}/Inc
  ${CB_DEVICE}/Include ${STM32CUBE_G4_ROOT}/Drivers/CMSIS/Include)
target_compile_options(companionbot_g474 PRIVATE $<$<COMPILE_LANGUAGE:C>:-Wall;-Wextra;-Werror;-Wno-unused-parameter;-fstack-usage>)
target_link_libraries(companionbot_g474 PRIVATE companionbot_core)
# Reuse the ST linker script, including its notices. Conservative 96 KiB main
# SRAM: G474's total 128 KiB includes 32 KiB CCM; never put DMA buffers in CCM.
file(READ "${STM32CUBE_G4_ROOT}/Projects/NUCLEO-G474RE/Templates/STM32CubeIDE/STM32G474RETX_FLASH.ld" CB_LD)
string(REPLACE "LENGTH = 128K" "LENGTH = 96K" CB_LD "${CB_LD}")
string(REPLACE "_Min_Heap_Size = 0x200" "_Min_Heap_Size = 0x0" CB_LD "${CB_LD}")
string(REPLACE "_Min_Stack_Size = 0x400" "_Min_Stack_Size = 0x2000" CB_LD "${CB_LD}")
file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/stm32g474.ld" "${CB_LD}")
target_link_options(companionbot_g474 PRIVATE
  "-T${CMAKE_CURRENT_BINARY_DIR}/stm32g474.ld" --specs=nano.specs --specs=nosys.specs
  -Wl,--gc-sections -Wl,--print-memory-usage "-Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/companionbot_g474.map")
set_target_properties(companionbot_g474 PROPERTIES SUFFIX .elf)
add_custom_command(TARGET companionbot_g474 POST_BUILD
  COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:companionbot_g474> ${CMAKE_CURRENT_BINARY_DIR}/companionbot_g474.hex
  COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:companionbot_g474> ${CMAKE_CURRENT_BINARY_DIR}/companionbot_g474.bin
  COMMAND ${CMAKE_SIZE} $<TARGET_FILE:companionbot_g474>)
