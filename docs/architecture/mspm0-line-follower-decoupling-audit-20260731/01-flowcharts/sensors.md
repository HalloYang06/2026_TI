# Sensors 当前流程

```mermaid
flowchart TD
    subgraph Line["Line"]
        LineGPIO["8 路上拉 GPIO<br/>wit-oled-hardware-spi.syscfg:116-162"] --> ReRead["一次决策内多次读引脚<br/>track.c:209-303"]
        ReRead --> Policy["采样、策略与副作用混合<br/>track.c:162-311"]
        SoftI2C["另一套软件 I2C 灰度输入<br/>gray.c:4-143"] --> Unused["无调用者、无 valid/timestamp<br/>gray.c:124-143"]
    end
    subgraph Encoder["Encoder"]
        Edge["AB 相 GPIO 边沿<br/>wit-oled-hardware-spi.syscfg:81-107"] --> Decode["GROUP1 ISR 解码<br/>encoder.c:9-65"]
        Decode --> Count["写 main.c 裸全局计数<br/>main.c:76"]
        Count --> Delta["20 ms 非原子差分<br/>main.c:151-164"]
    end
    subgraph Yaw["WIT yaw"]
        DMA["UART3 timeout + DMA<br/>wit-oled-hardware-spi.syscfg:289-312"] --> Parse["ISR 校验并解析 11B 包<br/>interrupt.c:63-106"]
        Parse --> Global["逐字段写 wit_data<br/>wit.c:3-5"]
        Global --> Consume["20 ms ISR 直接读取 yaw<br/>main.c:164-170"]
    end
```

## 边界问题

- Line 没有“一帧快照”；同次策略判断可能混用不同时刻的引脚值。
- 软件 I2C 灰度路径与并行 GPIO 路径重复，当前未接入且缺少 ACK/超时处理。
- 编码器驱动反向写入 `main.c` 拥有的计数，差分读取不是原子操作。
- WIT ISR 同时负责 DMA、组帧、解析和发布，数据没有 valid、timestamp 或 sequence。
- 三类输入都把“未更新/断线”静默解释为正常旧值或零值。

## 外部依赖

- SysConfig/DriverLib GPIO、DMA、UART、Timer、SysTick。
- App 全局状态、PID 单例和 Motor HAL。
- 当前未初始化的 SysTick 时间基准。
