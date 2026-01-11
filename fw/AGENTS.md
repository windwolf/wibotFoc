# FOC 电机驱动固件项目

## 项目概述

这是一个基于 STM32G431 微控制器的磁场定向控制（Field Oriented Control, FOC）电机驱动嵌入式固件项目。项目使用 Azure RTOS ThreadX 实时操作系统，并采用现代 C++23 和 C17 标准开发。

**产品名称**: AQ-BF01  
**版本**: 0.0.1  
**目标硬件**: STM32G431xx  
**开发工具**: CMake + Ninja, ARM GCC 工具链

## 项目架构

### 核心技术栈

- **微控制器**: STM32G431xx (Cortex-M4, 170MHz)
- **RTOS**: Azure RTOS ThreadX
- **编程语言**: C++23 (应用层), C17 (HAL层)
- **构建系统**: CMake 3.22+
- **工具链**: ARM GCC (arm-none-eabi)

### FOC电机控制特性

项目实现了完整的FOC电机驱动系统，包含以下关键功能：

1. **空间矢量PWM (SVPWM)**
   - 位置: [apps/svpwm.cpp](apps/svpwm.cpp), [apps/svpwm.hpp](apps/svpwm.hpp)
   - 用于三相逆变器的高效PWM调制

2. **外设配置**
   - **ADC2**: 电流采样 (相电流检测)
   - **TIM16**: 高分辨率定时器 (PWM生成)
   - **SPI3**: 编码器/传感器通信
   - **USART2**: 调试和通信接口
   - **DMA**: 高速数据传输

3. **控制环路**
   - 使用 wibot 框架的控制循环模块
   - 实时电流环、速度环控制

## 目录结构

### 用户代码目录

```
fw/
├── apps/                          # 应用层代码 (C++)
│   ├── components/               # 硬件无关代码（算法、业务逻辑等）
│   │   └── ...                   # 通用组件和算法模块
│   ├── drivers/                  # 硬件相关代码（设备驱动封装）
│   │   └── ...                   # 硬件驱动适配层
│   ├── app.cpp                   # 应用程序主入口
│   ├── config.hpp                # 配置参数
│   └── svpwm.cpp/hpp            # SVPWM算法实现
│
├── libs/                         # 第三方库
│   └── wibotlib2/               # Wibot控制框架库（第三方）
└── tests/                        # 单元测试目录，测试用例以 ThreadX 线程形式运行
```

### STM32CubeMX自动生成目录

以下目录由 **STM32CubeMX 自动生成**，通常不应手动修改：

```
fw/
├── Core/                         # STM32 HAL层和初始化（CubeMX生成）
│   ├── Inc/                      # 头文件
│   │   ├── main.h
│   │   ├── adc.h, tim.h, spi.h
│   │   └── app_threadx.h
│   └── Src/                      # 源文件
│       ├── main.c                # 系统入口
│       ├── adc.c, tim.c, spi.c  # 外设初始化
│       └── app_threadx.c         # ThreadX应用
│
├── AZURE_RTOS/                   # ThreadX RTOS配置（CubeMX生成）
│   └── App/
│       ├── app_azure_rtos.c/h
│       └── app_azure_rtos_config.h
│
├── Drivers/                      # STM32 HAL驱动（CubeMX生成）
│   ├── STM32G4xx_HAL_Driver/    # HAL库
│   └── CMSIS/                    # CMSIS核心和设备支持
│
├── Middlewares/                  # 中间件（CubeMX生成）
│   └── ST/threadx/              # ThreadX源码
│
├── cmake/                        # CMake构建脚本（CubeMX生成）
│   ├── gcc-arm-none-eabi.cmake  # GCC工具链配置
│   └── stm32cubemx/             # CubeMX生成文件集成
│
├── CMakeLists.txt               # 主CMake配置（部分CubeMX生成）
├── project-defs.cmake           # 项目定义
├── CMakePresets.json            # CMake预设配置
├── STM32G431XX_FLASH.ld        # 链接脚本（CubeMX生成）
└── startup_stm32g431xx.s       # 启动汇编代码（CubeMX生成）
```

### 目录组织原则

- **apps/components/**: 存放硬件无关的代码，如控制算法、数学库、业务逻辑等，便于移植和单元测试
- **apps/drivers/**: 存放硬件相关的代码，如对HAL的封装、传感器驱动、执行器驱动等
- **libs/wibotlib2/**: 第三方库，提供应用框架、控制循环、日志系统等基础功能
- **tests/**: 单元测试目录，测试用例以 ThreadX 线程方式运行，贴近实际调度. 并使用 wibotlib 提供的线程包装模型.
- **其他目录**: 由STM32CubeMX自动生成和管理，修改硬件配置时使用CubeMX工具重新生成

## 构建系统

### 构建配置

项目使用 CMake 作为构建系统，支持以下特性：

- **预设配置**: 通过 `CMakePresets.json` 管理不同构建配置
- **编译数据库**: 自动生成 `compile_commands.json` 支持 LSP (clangd)
- **构建类型**: Debug (默认)
- **标准**: C++23, C17

### 编译命令

```bash
# 配置项目
cmake --preset=default

# 构建
cmake --build build/Debug

# 或使用 Ninja
cd build/Debug
ninja
```

## 开发指南

### 编码约束

- 整个项目为嵌入式场景，避免依赖 C/C++ 标准库；如需常用功能，优先使用 wibotlib/轻量自实现方案
- 明确禁用: C++ 异常、RTTI（typeid/dynamic_cast）以及运行时动态内存分配（new/malloc/realloc/free）
- 谨慎使用易导致代码尺寸膨胀的特性（过度模板化）；保持可预测的 ROM/RAM 占用

### 应用程序入口

应用程序从 [apps/app.cpp](apps/app.cpp) 的 `bootApp()` 函数启动，通过 Wibot 框架的启动机制初始化：

```cpp
extern "C" void bootApp() { 
    BootFrom<App>::boot("AQ-BF01", "0.0.1"); 
}
```

### 添加新功能

1. **硬件无关代码**: 在 `apps/components/` 目录添加控制算法、数学库等通用模块
2. **硬件相关代码**: 在 `apps/drivers/` 目录添加设备驱动封装和硬件接口适配
3. **外设配置**: 使用 STM32CubeMX 打开 `wibotFoc.g4.ioc` 重新生成配置（不要手动修改 `Core/` 下的代码）
4. **控制算法**: 继承使用 wibotlib2 框架提供的控制循环基类

### ThreadX任务管理

RTOS任务在 [Core/Src/app_threadx.c](Core/Src/app_threadx.c) 中定义和创建。可以根据需求添加新的线程：

- 主控制线程（实时优先级）
- 通信线程
- 监控/诊断线程

## 硬件资源映射

### 关键外设用途

| 外设   | 用途                 | 配置文件                             |
| ------ | -------------------- | ------------------------------------ |
| ADC2   | 相电流采样 (A/B/C相) | [Core/Inc/adc.h](Core/Inc/adc.h)     |
| TIM16  | PWM生成 (电机驱动)   | [Core/Inc/tim.h](Core/Inc/tim.h)     |
| SPI3   | 位置编码器接口       | [Core/Inc/spi.h](Core/Inc/spi.h)     |
| USART2 | 串口通信/调试        | [Core/Inc/usart.h](Core/Inc/usart.h) |
| DMA    | ADC/SPI数据传输      | [Core/Inc/dma.h](Core/Inc/dma.h)     |

### 内存布局

由 [STM32G431XX_FLASH.ld](STM32G431XX_FLASH.ld) 定义：
- **Flash**: 程序存储
- **RAM**: 变量和堆栈
- **ThreadX**: 专用内存区域

## 依赖库

### Wibot库 (wibotlib2)

位于 `libs/wibotlib2/`，提供：
- 应用框架 (`app-framework.hpp`)
- 控制环路基础设施 (`control-loop.hpp`)
- 日志系统 (`logger.hpp`)

### STM32 HAL

- **版本**: STM32G4xx HAL Driver
- **位置**: `Drivers/STM32G4xx_HAL_Driver/`
- **配置**: [Core/Inc/stm32g4xx_hal_conf.h](Core/Inc/stm32g4xx_hal_conf.h)

### Azure RTOS ThreadX

- **位置**: `Middlewares/ST/threadx/`
- **配置**: [Core/Inc/tx_user.h](Core/Inc/tx_user.h)

## 调试和测试

### 日志系统

使用 Wibot 日志框架：

```cpp
#include "logger.hpp"
LOGGER("module_name")

// 日志输出
LOG_INFO("电机启动");
LOG_ERROR("电流超限: %d A", current);
```

### 单元测试

- 目录: [tests](tests)
- 运行方式: 每个用例以 ThreadX 线程形式执行，贴近实际任务调度场景
- 线程封装: 使用 wibotlib 的线程包装模型创建/管理 ThreadX 线程，保持测试用例与框架一致的生命周期接口

### 编译器输出

编译后生成：
- `wibotFoc.g4.elf`: ELF格式可执行文件（用于调试）
- `wibotFoc.g4.bin`: 二进制文件（用于烧录）
- `wibotFoc.g4.hex`: Intel HEX格式

## 配置文件

### 重要配置文件

1. **[project-defs.cmake](project-defs.cmake)**: 项目级别定义（宏、编译选项）
2. **[apps/config.hpp](apps/config.hpp)**: 应用配置参数（当前为空，可扩展）
3. **[wibotFoc.g4.ioc](wibotFoc.g4.ioc)**: STM32CubeMX项目文件
4. **[AZURE_RTOS/App/app_azure_rtos_config.h](AZURE_RTOS/App/app_azure_rtos_config.h)**: ThreadX配置

### 修改硬件配置

1. 使用 STM32CubeMX 打开 `wibotFoc.g4.ioc`
2. 修改外设配置
3. 重新生成代码（保留用户代码段）
4. 重新构建项目

## 开发环境要求

### 必需工具

- **CMake**: >= 3.22
- **ARM GCC**: arm-none-eabi-gcc (推荐 10.3+)
- **Ninja**: 构建工具
- **STM32CubeMX**: 硬件配置工具（可选）

### 推荐IDE

- **VS Code**: 使用 CMake Tools、Cortex-Debug 扩展
- **CLion**: 原生支持 CMake 和嵌入式开发
- **STM32CubeIDE**: STM官方IDE（基于Eclipse）

### 烧录和调试

- **调试器**: ST-Link V2/V3
- **烧录工具**: 
  - STM32CubeProgrammer
  - OpenOCD + GDB
  - J-Link (如有)

## 性能和实时性

### 控制周期

- **PWM频率**: 由TIM16配置决定 (典型20-40kHz)
- **电流环**: PWM中断同步 (50μs - 25μs)
- **速度环**: 1-10ms
- **位置环**: 10-100ms

### 实时优先级

ThreadX任务优先级分配（0最高）：
- 0-5: 控制环路（硬实时）
- 6-10: 通信和数据处理
- 11-15: 监控和诊断

## 扩展和定制

### 添加新的控制算法

1. 在 `apps/components/` 创建新的算法模块（硬件无关）
2. 继承 `ControlLoop` 基类
3. 实现控制逻辑
4. 如需硬件接口，在 `apps/drivers/` 创建驱动适配层
5. 在主应用中注册

### 通信协议集成

可以在 USART2 或 SPI3 上实现：
- CANopen
- Modbus RTU
- 自定义协议

## 故障排除

### 常见问题

1. **编译错误**: 检查工具链路径和CMake配置
2. **链接错误**: 验证链接脚本和内存配置
3. **运行时崩溃**: 检查堆栈大小和ThreadX配置
4. **PWM不输出**: 验证定时器时钟和GPIO配置

### 调试技巧

- 使用 SWD 接口的实时调试
- 通过 USART2 输出日志
- 使用 ThreadX 的追踪功能分析任务调度

## 许可证

请参考项目根目录的 LICENSE 文件和各个子组件的许可证声明。

## 联系和支持

- **项目**: FOC Motor Driver (wibotFoc)
- **平台**: STM32G431
- **框架**: Wibot Library v2

---

*本文档描述了项目结构和开发指南。建议AI助手在协助开发时参考此文档理解项目架构。*
