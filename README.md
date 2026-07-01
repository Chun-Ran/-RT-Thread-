# 基于 RT-Thread 的电力设施极早期过热热释离子感知与预警系统

基于 RT-Thread 实时操作系统的电力设施极早期过热检测与预警系统，通过热释离子感知技术实现对电力设备过热故障的超前预警。

## 硬件平台

- **MCU**: STM32F103C8T6 (ARM Cortex-M3)
- **传感器**: HP203B（温度/气压）
- **通信**: UART2（JSON 数据上报）
- **采样**: ADC1 + DMA + TIM2 触发，10kHz 采样率

## 软件架构

```
├── applications/       # 应用层（主程序、传感器驱动）
│   ├── main.c          # 主程序：ADC 采样、数据处理、上报任务
│   ├── i2c_ee.c/h      # HP203B 传感器 I2C 驱动
│   └── i2c_gpio.c/h    # 软件 I2C GPIO 模拟
├── drivers/            # STM32 外设驱动
├── libraries/          # STM32 HAL 库
├── packages/           # RT-Thread 在线软件包
├── rt-thread/          # RT-Thread 内核及组件
└── cubemx/             # STM32CubeMX 配置文件
```

## 功能特性

- **ADC 高速采样**: TIM2 触发 ADC1，DMA 双缓冲循环采集，10kHz 采样率
- **多任务并发**: RT-Thread 多线程，ADC 处理任务 + 数据上报任务并行
- **传感器融合**: HP203B 温度/气压传感器数据采集
- **JSON 输出**: 通过 UART2 输出结构化 JSON 数据
- **PWM 输出**: 预留 PWM3 通道用于外部控制

## 构建方法

### 环境要求

- **RT-Thread Studio** 或 **Env 工具 + SCons**
- ARM GCC 工具链 (`arm-none-eabi-gcc`)

### 编译

```bash
# 使用 SCons 构建
scons

# 或使用 RT-Thread Studio 直接导入项目
```

### 烧录

生成的固件文件位于 `build/` 目录下，使用 ST-Link 或串口烧录：

```bash
# 使用 ST-Flash 烧录 (示例)
st-flash write build/rt-thread.bin 0x08000000
```

## 使用说明

1. 连接 STM32F103C8T6 开发板
2. 连接 HP203B 传感器至 I2C 接口（PB10-SCL, PB11-SDA）
3. 连接 UART2（PA2-TX）至串口调试工具，波特率 9600
4. 上电后系统自动开始采样和上报

UART2 输出格式：

```json
{"voltage":1.650,"temperature":25.30,"pressure":101.32}
```

## 许可证

本项目使用 **Apache License 2.0** 许可证，与 RT-Thread 保持一致。

本项目基于 [RT-Thread](https://github.com/RT-Thread/rt-thread) 实时操作系统构建。
