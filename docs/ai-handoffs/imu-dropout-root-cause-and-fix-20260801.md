# MSPM0 IMU频繁掉线根因与实机修复验证（2026-08-01）

## 结论

本次掉线不是单一的“队列太小”，而是三个问题叠加：

1. WIT DMA先开启，随后LCD初始化连续阻塞约250 ms，启动阶段必然积压。
2. 协作调度器最多保留3个IMU release，旧实现每个release只处理32 B；前台恢复后最多补
   96 B，不能快速消化长阻塞形成的积压。
3. MSPM0主栈只有2 KiB。实机读取到栈底以下的IMU统计区出现`0x2020xxxx`指针值，证明
   主栈曾向下越界并破坏IMU运行状态。

最终修复不是继续无限扩大缓存，而是先消除确定性阻塞、补全阻塞后的有界追赶，再保留
1024 B队列作为短时抖动缓冲，并把主栈扩到4 KiB。最终固件复位后约10 s实测：队列
丢字节为0、当前积压为0、峰值363/1024，三类IMU帧持续增长，栈剩余约3160 B。

## 故障链路

```text
LCD初始化/前台长操作
        ↓
IMU UART DMA继续每32 B入队
        ↓
协作调度release上限为3，恢复预算仅3 × 32 B
        ↓
队列逐渐到达容量并丢字节，解析器短时失去完整帧
        ↓
加速度/角速度/姿态帧计数超过100 ms不更新
        ↓
MSPM0清除IMU_VALID，M33 READY退回PREPARING
        ↓
LCD显示MISS: IMU，SW1落在窗口内时启动被安全门拒绝
```

2 KiB主栈越界会进一步破坏队列和帧统计变量，因此故障表现可能从周期MISS扩大为随机
掉线、计数异常，甚至状态机行为不稳定。

## 修复前实机证据

旧固件在MSPM0 RAM中读取到：

| 指标 | 修复前值 |
|---|---:|
| `wit_queue_depth` | 0 |
| `wit_queue_dropped_byte_count` | `0x33C1` = 13249 B |
| `wit_queue_enqueued_byte_count` | `0x4C1A3` = 311715 B |
| `wit_queue_high_water` | `0x0100` = 256 B，达到旧容量上限 |
| 主栈范围 | `0x202019B0`～`0x202021B0`，共2 KiB |
| 现场SP | `0x20201C00`，离栈底仅592 B |

`wit_rx_byte_count`、`wit_serviced_byte_count`等位于栈底下方的变量出现多个
`0x2020xxxx`值。这些值形态与栈内RAM指针一致，而不是合理的单调字节计数，说明更深的
函数调用曾越过`0x202019B0`栈底。

队列深度读数为0不代表没有溢出；它只表示读取瞬间已经排空。`dropped=13249`和
`high_water=capacity`才是历史溢出的直接证据。

## 为什么之前的96 B追赶仍不够

提交`a5f6fd3`已经修复了“多个release只调用一次服务”的错误，使一次poll最多执行三次
32 B解析，但它只能兑现96 B预算。

JY901S当前输出加速度、角速度和姿态三类11 B帧，典型总负载约6600 B/s。LCD复位延迟为
30 ms + 100 ms + 120 ms，期间可进入约1650 B有效数据，明显超过96 B追赶预算和旧256 B
队列。调度release又只保留3个，无法用release数量表达250 ms的真实欠账。

所以本次修复把“实时调度次数”和“字节积压恢复”分开：调度仍保持有界，但每次IMU服务
最多处理256 B。正常状态队列只有少量数据，函数遇到空队列立即返回；发生短时阻塞后，
一次poll最多兑现3 × 256 B，随后下一个1 ms周期完成剩余恢复。

## 代码修复

### 1. 消除启动阶段确定性阻塞

竞赛`APP_MODE_LAP_TEST`先执行`lcd_init()`，完成LCD复位的250 ms阻塞后才调用
`WIT_Init()`开启UART DMA。这样启动期间尚未接收IMU字节，不会制造必然溢出。

提交：`27ba993 fix(mspm0): start IMU after blocking LCD reset`

### 2. 提高有界积压恢复预算

`WIT_FOREGROUND_BUDGET_PER_SERVICE`由32 B调整为256 B，但解析临时缓冲仍固定为
`WIT_DMA_TRANSFER_SIZE`，即32 B。服务函数分块解析，不会额外在栈上申请256 B。

提交：`145cc34 fix(mspm0): catch up IMU backlog after stalls`

### 3. 队列作为短时抖动缓冲

字节队列由256 B调整为1024 B，约能覆盖89 ms的115200 bit/s线路时间。它用于吸收LCD
绘制、按键蜂鸣和偶发前台抖动，不作为掩盖无限阻塞的手段。

提交：`8906e6f fix(mspm0): enlarge IMU byte queue`

### 4. 修复主栈越界

Keil启动文件原有0x100 B栈，加上自定义扩展后总栈由0x800 B提高到0x1000 B；构建脚本
同步把最低栈预算改为4 KiB并检查`__initial_sp`和连续STACK段。

提交：`758bb05 fix(mspm0): increase target stack reserve`

## 最终实机验证

使用Horco CMSIS-DAP（UID `2d2670f3`）烧录最终AXF。随后明确执行`reset`和`go`，运行约
10 s，再短暂停核读取RAM并立即恢复：

| 指标 | 最终值 | 结论 |
|---|---:|---|
| `wit_queue_dropped_byte_count` | 0 | 无丢字节 |
| `wit_queue_depth` | 0 | 积压已排空 |
| `wit_queue_high_water` | 363 / 1024 | 有余量，未触顶 |
| `wit_queue_enqueued_byte_count` | `0x1250F` | DMA持续接收 |
| `wit_serviced_byte_count` | `0x1250F` | 接收字节全部处理 |
| `wit_gyro_frame_count` | `0x08E1` | 角速度帧持续增长 |
| `wit_valid_frame_count` | `0x1AA4` | 三类有效帧持续增长 |
| 当前SP | `0x20202908` | 正常 |
| 新栈底 | `0x20201CB0` | 剩余`0xC58` = 3160 B |

Keil构建输出同时确认：

```text
Verified target stack: 0x1000 bytes
Built ...\Keil\Objects\wit-oled-hardware-spi.axf
```

pyOCD烧录成功，最终固件已经在MSPM0G3507运行。

## 调试器读取注意事项

pyOCD调试命令如果停核后没有执行`go`，IMU外设仍可能继续产生数据，而前台解析和状态机
停止，随后读到的`dropped`和deadline MISS会被调试行为污染。现场曾出现一次这种假增长。

读取运行计数必须采用：

```text
attach → halt → read32/reg → go → exit
```

需要建立干净基线时采用：

```text
attach → reset → go → 等待观测窗口 → halt → read → go
```

严禁把调试停核期间的增长直接归因于正常固件运行。

## 后续回归标准

1. 菜单静置60 s，LCD不得出现周期性`MISS: IMU`。
2. `wit_queue_dropped_byte_count`增量必须为0，`wit_queue_high_water`不得达到1024。
3. 加速度、角速度和姿态三类帧计数必须连续增长，任一类不得超过100 ms不更新。
4. Q4～Q6在READY状态下单按一次SW1应进入START_PENDING/RUNNING。
5. 若再次出现MISS，先同时记录`queue_depth/high_water/dropped`、三类帧计数和
   `imu_deadline_miss_total`，再判断是线路故障、前台阻塞还是调试器停核，不能只扩大缓存。
