"""Adapt pinned ST CubeMX/MDK templates to the implemented CompanionBot board.

The generated configuration is reviewable without CubeMX/Windows. Opening and
regenerating with CubeMX and compiling in MDK remain explicit hardware-owner
checks; this script never claims to have run either application.
"""
from pathlib import Path
import argparse
import os
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[2]
UPSTREAM = ROOT / ".venv/embedded/upstream/STM32CubeG4"
BOARD = ROOT / "firmware/boards/nucleo_g474re"
HAL_MODULES = "hal hal_cortex hal_dma hal_dma_ex hal_exti hal_flash hal_flash_ex hal_flash_ramfunc hal_gpio hal_iwdg hal_pwr hal_pwr_ex hal_rcc hal_rcc_ex hal_tim hal_tim_ex hal_uart hal_uart_ex".split()

def ioc():
    source = UPSTREAM / "Projects/NUCLEO-G474RE/Examples/UART/UART_ReceptionToIdle_CircularDMA/UART_ReceptionToIdle_CircularDMA.ioc"
    data = {}
    for line in source.read_text().splitlines():
        if "=" in line:
            key, value = line.split("=", 1)
            data[key.replace("LPUART1", "USART2")] = value.replace("LPUART1", "USART2")
    data.update({
        "Mcu.CPN": "STM32G474RET6", "Mcu.UserName": "STM32G474RETx",
        "Mcu.IP0": "DMA", "Mcu.IP1": "GPIO", "Mcu.IP2": "IWDG", "Mcu.IP3": "NVIC",
        "Mcu.IP4": "RCC", "Mcu.IP5": "SYS", "Mcu.IP6": "TIM6", "Mcu.IP7": "USART2", "Mcu.IPNb": "8",
        "Mcu.Pin0": "PA2", "Mcu.Pin1": "PA3", "Mcu.Pin2": "PA5", "Mcu.Pin3": "PA13",
        "Mcu.Pin4": "PA14", "Mcu.Pin5": "VP_SYS_VS_Systick", "Mcu.Pin6": "VP_SYS_VS_DBSignals",
        "Mcu.Pin7": "VP_TIM6_VS_ClockSourceINT", "Mcu.Pin8": "VP_IWDG_VS_IWDG", "Mcu.PinsNb": "9",
        "Dma.Request0": "USART2_RX", "Dma.Request1": "USART2_TX", "Dma.RequestsNb": "2",
        "Dma.USART2_RX.0.Priority": "DMA_PRIORITY_HIGH",
        "USART2.BaudRate": "921600", "USART2.IPParameters": "BaudRate",
        "PA2.Signal": "USART2_TX", "PA3.Signal": "USART2_RX",
        "PA5.Signal": "GPIO_Output", "PA5.GPIO_Label": "LD2", "PA5.GPIOParameters": "GPIO_Label",
        "PA5.Locked": "true", "PA13.Signal": "SYS_JTMS-SWDIO", "PA13.Mode": "Serial_Wire",
        "PA14.Signal": "SYS_JTCK-SWCLK", "PA14.Mode": "Serial_Wire",
        "TIM6.Prescaler": "169", "TIM6.Period": "1999", "TIM6.IPParameters": "Prescaler,Period",
        "VP_TIM6_VS_ClockSourceINT.Mode": "Enable_Timer", "VP_TIM6_VS_ClockSourceINT.Signal": "TIM6_VS_ClockSourceINT",
        "IWDG.Prescaler": "IWDG_PRESCALER_64", "IWDG.Reload": "124", "IWDG.IPParameters": "Prescaler,Reload",
        "VP_IWDG_VS_IWDG.Mode": "IWDG_Activate", "VP_IWDG_VS_IWDG.Signal": "IWDG_VS_IWDG",
        "RCC.PLLM": "RCC_PLLM_DIV4", "RCC.PLLN": "85", "RCC.PLLSourceVirtual": "RCC_PLLSOURCE_HSI",
        "RCC.SYSCLKSource": "RCC_SYSCLKSOURCE_PLLCLK", "RCC.VCOInputFreq_Value": "4000000",
        "RCC.VCOOutputFreq_Value": "340000000", "RCC.PLLRCLKFreq_Value": "170000000",
        "RCC.PLLQoutputFreq_Value": "170000000", "RCC.PLLPoutputFreq_Value": "170000000",
        "RCC.LSIState": "RCC_LSI_ON", "RCC.PLLR": "RCC_PLLR_DIV2", "RCC.PLLQ": "RCC_PLLQ_DIV2",
        "RCC.PLLP": "RCC_PLLP_DIV2",
        "ProjectManager.FirmwarePackage": "STM32Cube FW_G4 V1.6.1", "ProjectManager.LastFirmware": "false",
        "ProjectManager.ProjectName": "companionbot_g474", "ProjectManager.ProjectFileName": "companionbot_g474.ioc",
        "ProjectManager.TargetToolchain": "MDK-ARM V5", "ProjectManager.MainLocation": "Core/Src",
        "ProjectManager.StackSize": "0x2000", "ProjectManager.HeapSize": "0x0",
        "ProjectManager.DeletePrevious": "false", "ProjectManager.KeepUserCode": "true",
        "ProjectManager.LibraryCopy": "2", "ProjectManager.Example": "",
        "ProjectManager.ExampleSource": "", "board": "NUCLEO-G474RE", "boardIOC": "true",
        "ProjectManager.functionlistsort": "1-SystemClock_Config-RCC-false-HAL-false,2-MX_GPIO_Init-GPIO-false-HAL-true,3-MX_DMA_Init-DMA-false-HAL-true,4-MX_USART2_UART_Init-USART2-false-HAL-true,5-MX_TIM6_Init-TIM6-false-HAL-true,6-MX_IWDG_Init-IWDG-false-HAL-true",
    })
    for suffix in ("Direction", "EventEnable", "MemDataAlignment", "MemInc", "PeriphDataAlignment", "PeriphInc", "Polarity", "Priority", "RequestNumber", "RequestParameters", "SignalID", "SyncEnable", "SyncPolarity", "SyncRequestNumber", "SyncSignalID"):
        data["Dma.USART2_TX.1." + suffix] = data["Dma.USART2_RX.0." + suffix]
    data.update({"Dma.USART2_TX.1.Instance": "DMA1_Channel2", "Dma.USART2_TX.1.Mode": "DMA_NORMAL",
                 "Dma.USART2_TX.1.Direction": "DMA_MEMORY_TO_PERIPH", "Dma.USART2_TX.1.Priority": "DMA_PRIORITY_LOW"})
    for key in ("AHBFreq_Value", "APB1Freq_Value", "APB1TimFreq_Value", "APB2Freq_Value", "APB2TimFreq_Value", "CortexFreq_Value", "FCLKCortexFreq_Value", "HCLKFreq_Value", "HRTIM1Freq_Value", "SYSCLKFreq_VALUE", "USART2Freq_Value"):
        data["RCC."+key] = "170000000"
    rcc = {key[4:] for key in data if key.startswith("RCC.") and key != "RCC.IPParameters"}
    data["RCC.IPParameters"] = ",".join(sorted(rcc))
    for name, priority in (("DMA1_Channel1_IRQn", 2), ("DMA1_Channel2_IRQn", 2), ("USART2_IRQn", 2), ("TIM6_DAC_IRQn", 1), ("SysTick_IRQn", 15)):
        data["NVIC."+name] = f"true\\:{priority}\\:0\\:false\\:false\\:true\\:true\\:true\\:true"
    return "#MicroXplorer Configuration settings - do not modify\n" + "\n".join(f"{k}={v}" for k,v in sorted(data.items())) + "\n"

def keil():
    tree = ET.parse(UPSTREAM / "Projects/NUCLEO-G474RE/Templates/MDK-ARM/Project.uvprojx")
    root = tree.getroot()
    def set_value(path, value):
        element = root.find(path)
        if element is None: raise ValueError(path)
        element.text = value
    prefix = "./Targets/Target/"
    common = prefix + "TargetOption/TargetCommonOption/"
    set_value(prefix+"TargetName", "CompanionBot_G474RE")
    set_value(common+"Cpu", 'IROM(0x08000000,0x80000) IRAM(0x20000000,0x18000) IRAM2(0x10000000,0x8000) CLOCK(170000000) FPU2 CPUTYPE("Cortex-M4")')
    output_dir=os.path.relpath(ROOT/"models/minisegway/cross_language_embedded/results/mdk_output",BOARD/"MDK-ARM").replace("/","\\")+"\\"
    set_value(common+"OutputDirectory",output_dir)
    set_value(common+"ListingPath",output_dir)
    set_value(common+"OutputName", "companionbot_g474")
    set_value(common+"CreateHexFile", "1")
    ads = prefix+"TargetOption/TargetArmAds/"
    set_value(ads+"Cads/VariousControls/Define", "STM32G474xx,USE_HAL_DRIVER")
    set_value(ads+"Cads/VariousControls/MiscControls", "-std=c99 -ffp-contract=off")
    set_value(ads+"LDads/umfTarg", "0")
    set_value(ads+"LDads/useFile", "1")
    set_value(ads+"LDads/ScatterFile", "companionbot_g474.sct")
    base = BOARD / "MDK-ARM"
    rel = lambda p: os.path.relpath(p,base).replace("/", "\\")
    includes = [BOARD/"Core/Inc", BOARD, ROOT/"firmware/app", ROOT/"firmware/generated",
                ROOT/"ros2_ws/src/companionbot_control/native/include", UPSTREAM/"Drivers/STM32G4xx_HAL_Driver/Inc",
                UPSTREAM/"Drivers/CMSIS/Include", UPSTREAM/"Drivers/CMSIS/Device/ST/STM32G4xx/Include"]
    set_value(ads+"Cads/VariousControls/IncludePath", ";".join(map(rel,includes)))
    groups = root.find(prefix+"Groups"); groups.clear()
    files = {
        "CompanionBot app and board": list((BOARD/"Core/Src").glob("*.c")) + [BOARD/"board_io.c", ROOT/"firmware/app/app.c"],
        "Shared C control core": list((ROOT/"ros2_ws/src/companionbot_control/native/src").glob("*.c")),
        "ST HAL external": [UPSTREAM/"Drivers/STM32G4xx_HAL_Driver/Src"/f"stm32g4xx_{module}.c" for module in HAL_MODULES],
        "ST CMSIS external": [UPSTREAM/"Drivers/CMSIS/Device/ST/STM32G4xx/Source/Templates/system_stm32g4xx.c",
                              ROOT/".venv/embedded/generated/startup_stm32g474xx.s"],
    }
    for name, paths in files.items():
        group = ET.SubElement(groups,"Group"); ET.SubElement(group,"GroupName").text=name
        parent = ET.SubElement(group,"Files")
        for path in sorted(paths):
            if not path.is_file(): raise FileNotFoundError(path)
            entry=ET.SubElement(parent,"File"); ET.SubElement(entry,"FileName").text=path.name
            ET.SubElement(entry,"FileType").text="2" if path.suffix==".s" else "1"
            ET.SubElement(entry,"FilePath").text=rel(path)
    ET.indent(tree)
    return ET.tostring(root,encoding="unicode",xml_declaration=True)+"\n"

def main():
    cli=argparse.ArgumentParser(description=__doc__)
    cli.add_argument("--check",action="store_true")
    args=cli.parse_args()
    # Adapt only the startup's stack/heap constants in the ignored dependency
    # cache; retain ST's full assembly and licensing, never vendor it in the repo.
    source=UPSTREAM/"Drivers/CMSIS/Device/ST/STM32G4xx/Source/Templates/arm/startup_stm32g474xx.s"
    startup=source.read_text().replace("Stack_Size      EQU     0x400", "Stack_Size      EQU     0x2000")
    startup=startup.replace("Heap_Size       EQU     0x200", "Heap_Size       EQU     0x0")
    if startup==source.read_text(): raise SystemExit("ST startup constants changed; inspect before adapting")
    target=ROOT/".venv/embedded/generated/startup_stm32g474xx.s"
    if args.check:
        if not target.is_file() or target.read_text()!=startup: raise SystemExit("Generated Keil startup differs")
    else:
        target.parent.mkdir(parents=True,exist_ok=True);target.write_text(startup)
    outputs = {BOARD/"companionbot_g474.ioc": ioc(), BOARD/"MDK-ARM/companionbot_g474.uvprojx":keil(),
        BOARD/"MDK-ARM/companionbot_g474.sct":
        "; CompanionBot: 512 KiB Flash, 96 KiB DMA-accessible SRAM; ST startup is external.\n"
        "LR_IROM1 0x08000000 0x00080000 {\n  ER_IROM1 0x08000000 0x00080000 {\n"
        "    *.o (RESET, +First)\n    *(InRoot$$Sections)\n    .ANY (+RO)\n  }\n"
        "  RW_IRAM1 0x20000000 0x00018000 {\n    .ANY (+RW +ZI)\n  }\n}\n"}
    for path,text in outputs.items():
        if args.check:
            if not path.is_file() or path.read_text()!=text: raise SystemExit(f"Board config differs: {path}")
        else:
            path.parent.mkdir(parents=True,exist_ok=True); path.write_text(text)
    print("Board configuration " + ("checked" if args.check else "generated from pinned ST templates"))

if __name__=="__main__": main()
