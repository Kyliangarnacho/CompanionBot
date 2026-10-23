# ST 上游来源与许可证

本仓库不 vendor HAL、CMSIS、startup 或 ST linker 全包。`upstream.lock.json` 固定官方 CubeG4 v1.6.1 和两个子模块提交，`tools/embedded/fetch_upstream.py` 在 `.venv/embedded/upstream/STM32CubeG4` 获取/核验；目录被 `.gitignore` 排除。

| 复用内容 | 官方来源 | 许可证/处理 |
| --- | --- | --- |
| CubeG4 HAL C sources / headers | [stm32g4xx_hal_driver，锁定提交](https://github.com/STMicroelectronics/stm32g4xx_hal_driver/tree/d69997c6dcdc2d672afae31fee03713fb033c4a8) | `Drivers/STM32G4xx_HAL_Driver/LICENSE.md`，ST BSD 三条款；原文件未修改。 |
| CMSIS device、system、GCC/ARM startup | [cmsis_device_g4，锁定提交](https://github.com/STMicroelectronics/cmsis_device_g4/tree/84d8c1d2e301e873bccd9f4b9a544f1930f66d42) | `License.md` Apache-2.0；各文件头的来源/版权保留。 |
| CMSIS Cortex-M4 core headers | [CubeG4 CMSIS](https://github.com/STMicroelectronics/STM32CubeG4/tree/v1.6.1/Drivers/CMSIS) | 各 ARM CMSIS headers 中 Apache-2.0 标识保留。 |
| linker、MDK project、board template | [官方 NUCLEO-G474RE Templates](https://github.com/STMicroelectronics/STM32CubeG4/tree/v1.6.1/Projects/NUCLEO-G474RE/Templates) | `Templates/LICENSE.md` 及源文件自带声明保留。CMake 只在 build 目录派生 linker；Keil XML 为经脚本裁剪/适配的板级配置。 |
| CubeMX UART DMA 配置参考 | [ST ReceptionToIdle example](https://github.com/STMicroelectronics/STM32CubeG4/tree/v1.6.1/Projects/NUCLEO-G474RE/Examples/UART/UART_ReceptionToIdle_CircularDMA) | `.ioc` 经 `generate_board_projects.py` 适配 USART2/TIM6/IWDG，不复制整个 example 或 BSP。 |

G474 的主 SRAM按96 KiB、CCM按32 KiB处理；下载的旧模板 linker 将主 RAM 写成128 KiB，本地派生时保守修正为96 KiB，并设8 KiB栈、0 heap，DMA不放CCM。MDK ARM startup 的 Stack_Size/Heap_Size只在 ignored `.venv/embedded/generated/` 产生派生副本，保留原版权与完整汇编，不改下载的上游文件。

发放固件二进制时仍需随包保留这些第三方许可证；当前本地构建不构成重新授权。F4 上游仅调研，未在仓库伪造已支持的 F4 BSP。其余借鉴项目与 Adopt/Reject 依据见 [调研与选型](../models/minisegway/cross_language_embedded/UPSTREAM_AND_HARDWARE.md)。
