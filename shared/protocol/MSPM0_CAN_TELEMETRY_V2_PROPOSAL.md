# MSPM0_CAN_TELEMETRY_V2：源时间语义提案

Status: Proposed; not implemented

## 目的

V1已经冻结11位Classic CAN的ID、DLC和毫单位，能够稳定传输，但8字节向量帧没有
空间同时携带源采样序号、源时间和质量位。V2只解决“这组数据是什么时刻真正采到的”
问题，不扩大MSPM0的控制权限，也不启用任何执行器命令。

## 兼容方案

保留V1的`0x100/0x101/0x103`布局，并增加一个11位标准帧：

| ID | 字节0..1 | 字节2..3 | 字节4..7 |
|---:|---|---|---|
| `0x104` | `imu_epoch:u16` | `sample_mask:u16` | `source_time_ms:u32` |

`sample_mask`定义：

| 位 | 含义 |
|---:|---|
| 0 | 本epoch含新加速度 |
| 1 | 本epoch含新角速度 |
| 2 | 本epoch含新姿态 |
| 3 | 三类已组成完整一致样本 |
| 4 | UART解析曾丢弃残帧或校验失败 |
| 5 | 传感器报告饱和/超量程 |

三类向量帧的字节0..1都写相同`imu_epoch`。只有MSPM0收齐新`0x51/0x52/0x53`
并原子提交缓冲区后才增加epoch；重复镜像保持原epoch。`source_time_ms`是MSPM0
完成这一epoch的单调时刻，不是CAN发送时刻。

## 接收规则

1. M33只有在三类向量的epoch相同且`0x104.COMPLETE=1`时才生成完整IMU输入。
2. 相同epoch的重复帧只计镜像次数，不刷新源age。
3. `now-source_time_ms`超过阈值、三类epoch不一致、`COMPLETE=0`或心跳未置
   `IMU_VALID`时，滚球前馈无效。
4. V1接收端会忽略未知`0x104`，因此可以先升级MSPM0，再升级EdgeTalk。
5. 设备重启通过心跳uptime回退识别；重启后清空等待中的不完整epoch。

## 源端实现要求

- UART解析器使用跨DMA块状态机或环形缓冲，不能清空未消费的1到10个残留字节。
- 为加速度、角速度、姿态分别记录有效帧数、校验失败数、最后源时间和量程状态。
- 用双缓冲或generation seqlock原子提交完整样本；CAN线程不能读到一半新、一半旧。
- SysTick只产生due位。浮点/定点换算和MCAN写入放在主循环或有预算的低优先级任务。
- 若目标为200个完整WIT组/s，三类11字节8N1至少需要`66 kbit/s`；若目标为500组/s，
  至少需要`165 kbit/s`。实际配置应留调度和线缆裕量，500 Hz候选优先`460800 bit/s`，
  但必须先确认具体WIT模块支持的波特率和输出率。

## 迁移验收

- 逻辑分析仪统计60 s：三类源计数之差始终不超过1，完整epoch率达到配置目标的99%。
- 人为暂停WIT数据时，最后epoch不再增加，EdgeTalk在阈值内将IMU标为stale。
- 重放重复CAN帧时，M33的源age持续增长，不能被重复帧清零。
- 注入单类丢帧时，`COMPLETE`清零，三类不同步计数增加，控制保持shadow/禁止解锁。
- 自动化和HIL全程保持`MOTOR_COMMAND_TX=0`、`ACTUATOR_TX=0`。
