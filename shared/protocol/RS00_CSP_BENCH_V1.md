# RS00 CSP人工台架控制合同 V1

## 范围

本文冻结5号RS00与EdgeTalk M33之间已经过实物验证的人工微动合同。它只用于架空无载台架、方向/角度/反馈验证，不是M55滚球算法的正式执行器接口。自动测试和M55 shadow继续保持`ACTUATOR_TX=0`。

协议依据为RobStride官方示例[`RobStride/robstride_ros_sample`](https://github.com/RobStride/robstride_ros_sample)，核对提交`d4751f3f16f3593a4e587738b6e27b949b4999ba`。总线使用1 Mbps Classic CAN、29位扩展帧、DLC 8，主机ID=`0xFD`，电机ID=`0x05`。

## 已使用帧

扩展ID按`comm_type << 24 | master_id << 8 | motor_id`编码。

| 操作 | comm_type | 数据 |
|---|---:|---|
| enable | `0x03` | 全0 |
| stop | `0x04` | `data[0]=0`，其余为0 |
| 读单参数 | `0x11` | `data[0:2]=index`小端 |
| 写单参数 | `0x12` | `data[0:2]=index`小端，值放在`data[4:8]` |

V1只允许以下写索引：

| 索引 | 类型 | 本次值 | 含义 |
|---|---|---:|---|
| `0x7005` | `u8` | `5` | CSP运行模式 |
| `0x7016` | `float32 LE` | 当前保持点或受限目标 | `loc_ref` |
| `0x7017` | `float32 LE` | `0.5 rad/s` | 速度上限 |
| `0x7018` | `float32 LE` | `0.8 A` | 电流上限，不是电流命令 |

禁止提供set-zero、速度参考、Iq参考和力矩参考API。单次微动绝对值不超过`0.02 rad`，相对本次初始位置总包络不超过`+-0.05 rad`。

## 人工状态机

```text
SAFE/STOPPED
  -> PREPARING: stop -> mode=5 -> limit_spd -> limit_cur -> hold loc_ref
  -> PREPARED: 读回0x7005确认等于5，电机仍未使能
  -> ARMING: 再写保持点 -> enable
  -> ARMED: 必须收到使能后的0x02反馈且fault=0
  -> SMALL_STEP: 只允许一次有界相对位置命令
  -> RETURNING: 写回初始保持点
  -> STOPPED: 回位容差满足、超时、反馈异常或人工stop均停机
```

准备、使能和微动命令必须带`CONFIRM_NO_LOAD`；return和stop不需要口令，stop可覆盖等待中的请求。所有CAN发送集中在M33 CAN worker，开机不自动准备、不自动使能、不自动运动。

使能后的`0x02`是命令响应，不作为周期心跳。人工会话以100 Hz交替读取`mechPos(0x7019)`和`mechVel(0x701B)`，用于轨迹证据、freshness和回位判定；普通六参数轮询在控制会话期间暂停。

## 2026-07-31实物结果

- 供电：22 V；软件电流上限0.8 A；摆杆架空无载；断电急停；操作员在场。
- 初始位置`1.684 rad`，命令`+0.010 rad`，目标`1.694 rad`，100 Hz定向读回到`1.694 rad`。
- 回位后读回`1.687 rad`，距初始位置3 mrad，随后自动stop；最终fault=`0x00`。
- CAN发送`769/769/0`，TEC/REC=0，FIFO full/lost=0。

这证明人工CSP命令、模式读回、使能反馈、位置读回、回位和停机闭环可用；不代表M55算法到200 Hz CSP发布已经开放。
