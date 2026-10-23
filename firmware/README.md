# CompanionBot STM32 C/HAL

可独立ARM构建的NUCLEO-G474RE工程，共用`ros2_ws/src/companionbot_control/native`的C算法，不含ROS/RTOS。实现HAL clock、USART2 circular RX/TX DMA、TIM6 2ms调度、IWDG、故障/物理arm gate和状态；未确定的IMU/encoder/motor留在`board_io`，默认unavailable、actual未知、输出禁用。

根目录运行`bash scripts/build_firmware.sh <本工程results下的build目录>`，生成ELF/HEX/BIN/map；首次依赖、主机C/ROS构建、协议、Keil/CubeMX及用户烧录操作见 [跨语言合同](../docs/CROSS_LANGUAGE_EMBEDDED.md)。参考硬件及未验证项在`reference_hardware.json`；第三方来源、许可证、startup/linker派生说明见 [UPSTREAM](UPSTREAM.md)。

`.ioc`和MDK5/ARM Compiler6工程已生成并检查引用；未运行CubeMX/MDK、未烧录、无实物验证。固件当前实现nominal实时主干，未声称具备Python高级slope/payload运行时。`COLCON_IGNORE`保持ROS仍为七包。

`app`按TIM6 ISR释放age及新tick在actuator写出前检查期限，`cb_timing`记录供SWD观察的DWT路径最大值。`test/`编译真实app.c和明确的host HAL夹具，仅验证逻辑/DMA边界；虚拟cycles不是执行时间。ARM分析、2ms待验收预算及未测WCET见同一工程final_report的补充验证部分。
