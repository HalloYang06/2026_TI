# HBALL_MISSION_CAN_V1：H题任务控制面

## 边界

本协议只编排任务，不承载底盘速度、RS00位置或任何执行器命令。MSPM0是唯一操作员意图
入口，EdgeTalk M33是唯一任务仲裁器。所有帧使用`1 Mbps Classic CAN`、11位标准ID、
数据帧、DLC 8和小端整数。

每次有效SW1启动使用非零`epoch:u16`。PREPARE使用下一个候选epoch，START确认同一epoch；
相同epoch的重复帧必须幂等，旧epoch不能覆盖当前任务。任一节点重启时当前RUNNING失效，
重新SELF_TEST后才允许新的候选epoch。

## 帧

| ID | 方向/频率 | 字节0..1 | 字节2 | 字节3 | 字节4..7 |
|---:|---|---|---|---|---|
| `0x081` | MSP→M33，20 Hz | epoch | Q号2..6 | command | MSP事件时刻ms |
| `0x082` | M33→MSP，20 Hz | epoch | Q号2..6 | global state | ready mask、reason、status seq |
| `0x083` | M33→MSP，10 Hz | epoch | `ball_mm:i16`占2..3 | `target_mm:i16`占4..5 | phase、quality占6..7 |
| `0x084` | MSP→M33，50 Hz | epoch | chassis phase | event flags | elapsed ms |

command为PREPARE=1、START=2、ABORT=3、RESET=4。global state依次为BOOT、SELF_TEST、
SELECT、PREPARING、READY、START_PENDING、RUNNING、FINISHING、COMPLETED、
CONTROLLED_ABORT和FAULT_LATCHED（0..10）。READY位和底盘事件位以
`hball_mission_can.h`为唯一代码定义。

## 安全规则

- `epoch=0`、Q号不在2..6、未知command/state/reason、扩展帧、远程帧或DLC错误均拒绝。
- READY之外收到START不得排队；条件恢复后必须由操作员重新按SW1。
- `0x081..0x084`不能被任何执行器发送白名单解释为电机命令。
- 树莓派未上电时PI USB、VISION和RECORDER三个位保持0，系统不得进入RUNNING。
