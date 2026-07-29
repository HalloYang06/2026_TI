# VISION_MEASUREMENT_V1：钢球视觉测量帧

## 用途

树莓派完成灰度 ROI、阈值/形态学、轮廓筛选和圆心计算，只把测量结果发给 EdgeTalk M33。控制链不传完整图像、灰度ROI像素、二值图或轮廓点集；`roi_x/y/w/h`仅是本帧裁剪区域的元数据。原始画面只在视觉板本地用于显示、录像和离线调参。

目标输入为 `120 Hz`，EdgeTalk 接收能力按不低于 `240 Hz` 验收。M55 的估计与 LQG 仍以 `200 Hz` 运行：有新视觉序号时更新观测，无新帧时只做模型预测，禁止重复融合旧测量。

## 字节布局

所有整数和 IEEE-754 `float32` 都按小端编码。帧固定为 64 字节，不直接映射或强转为 C 结构体；收发端按偏移显式读写，避免填充、未对齐和编译器 ABI 差异。

| 偏移 | 字节 | 字段 | 单位/含义 |
|---:|---:|---|---|
| 0 | 2 | `magic:u16` | 固定 `0x5AA5` |
| 2 | 1 | `version:u8` | 固定 `1` |
| 3 | 1 | `message_type:u8` | 固定 `1`，钢球测量 |
| 4 | 2 | `frame_size:u16` | 固定 `64` |
| 6 | 2 | `flags:u16` | 检测和质量标志 |
| 8 | 4 | `sequence:u32` | 每个视觉进程单调递增，可回绕 |
| 12 | 8 | `capture_time_us:u64` | 视觉板单调时钟，曝光中点时间 |
| 20 | 4 | `processing_time_us:u32` | 图像交付到测量编码完成的耗时 |
| 24 | 4 | `center_x_px:f32` | 全图坐标系圆心 x |
| 28 | 4 | `center_y_px:f32` | 全图坐标系圆心 y |
| 32 | 4 | `radius_px:f32` | 拟合半径 |
| 36 | 4 | `ball_position_m:f32` | 标定后沿凹槽坐标，目标中心为 0 |
| 40 | 4 | `confidence:f32` | `[0,1]`，轮廓质量综合置信度 |
| 44 | 4 | `contour_area_px2:f32` | 被选轮廓面积，供异常诊断 |
| 48 | 2 | `roi_x:u16` | 本帧 ROI 左上角 x |
| 50 | 2 | `roi_y:u16` | 本帧 ROI 左上角 y |
| 52 | 2 | `roi_w:u16` | 本帧 ROI 宽度 |
| 54 | 2 | `roi_h:u16` | 本帧 ROI 高度 |
| 56 | 4 | `exposure_us:u32` | 相机曝光时间 |
| 60 | 4 | `crc32c:u32` | 覆盖字节 0..59 |

CRC 使用 CRC-32C/Castagnoli：反射多项式 `0x82F63B78`、初值和结果异或均为 `0xFFFFFFFF`；`"123456789"` 的校验值为 `0xE3069283`。

## flags

| 位 | 名称 | 置位含义 |
|---:|---|---|
| 0 | `DETECTED` | 找到候选钢球轮廓 |
| 1 | `POSITION_VALID` | 米制位置已通过标定和边界检查，可进入估计器 |
| 2 | `ROI_PREDICTED` | ROI 由上一状态预测得到 |
| 3 | `ROI_CLIPPED` | ROI 被图像边界裁剪，置信度应降低 |
| 4 | `EXPOSURE_STABLE` | 曝光未处在跳变/自动曝光收敛阶段 |
| 5 | `CONTOUR_ROUND` | 圆度、面积和半径一致性检查通过 |

未知高位必须忽略，不能导致旧版本接收端越界。未检测到球时仍可发送心跳帧，但必须清除 `DETECTED/POSITION_VALID` 并把 `confidence` 设为 0。

## 接收规则

1. 按 magic、版本、类型、长度、CRC、有限浮点数和数值范围依次检查。
2. USB CDC 是字节流；一次读取可能不足一帧或包含多帧。解析器必须跨读取缓存，并在坏帧后扫描下一个 magic 重同步。
3. 只有严格更新的序号可以覆盖 M33 最新视觉状态；重复/乱序包只计数，不进入 M55。
4. `capture_time_us` 通过独立时钟同步换算到 M33 时间域。控制器按采集时刻计算视觉 age，不能用 USB 到达时间替代。
5. 连续约 3 帧缺失（约 25 ms）开始膨胀观测协方差；`50 ms` 进入降级，`100 ms` 判定视觉失锁。最终阈值需按实测 P99 延迟调整。

## 带宽与频率

| 帧率 | 纯协议负载 | 结论 |
|---:|---:|---|
| 120 Hz | 7,680 B/s | 正式目标，周期约 8.33 ms |
| 200 Hz | 12,800 B/s | 与 M55 控制周期相当 |
| 240 Hz | 15,360 B/s | USB接收和解析验收档 |
| 500 Hz | 32,000 B/s | 只作链路压力档，不代表相机需要500 FPS |

EdgeTalk 已以 High-Speed 480 Mbps 枚举。这里的风险主要是 Linux 调度、视觉处理抖动、CDC 分包和 M33 线程阻塞，而不是 USB 理论带宽。

实物验收使用树莓派脚本的`achieved_rate_hz/deadline_misses`与M33的`vision_rate_x10/vision_bytes_s`两端对拍。240 Hz档要求主机实际速率不低于237.6 Hz、零调度漏槽，且M33有效帧计数、序号和CRC无损；只有主机`write()`成功不能证明端到端通过。

树莓派每得到一帧64字节测量就立即写入，不等待8帧凑满512字节。诊断/生产发送的单次写超时上限为20 ms；超时后丢弃过期测量并重连，禁止补发旧位置。M33保持官方阻塞式`USBD_CDC_Receive(..., 512, 0)`：emUSB在收到首个USB包后即可返回当前数据，参数`0`表示等待期间不设超时，并不表示必须等满512字节。

## 官方USB依据与已验证边界

- [Infineon PSoC Edge USB CDC echo示例](https://github.com/Infineon/mtb-example-psoc-edge-usb-device-cdc-echo/blob/42fbdaeeac61c8b9eae049855b488f0862d9385c/README.md#L88)规定High-Speed Bulk IN/OUT最大包长为512字节；[端点初始化](https://github.com/Infineon/mtb-example-psoc-edge-usb-device-cdc-echo/blob/42fbdaeeac61c8b9eae049855b488f0862d9385c/proj_cm33_ns/main.c#L99-L129)与当前M33实现一致。
- [emUSB设备常量](https://github.com/Infineon/emusb-device/blob/c021f17494cc8f26e4c23f27949faeea278db56a/USBD/USB.h#L85-L110)定义HS Bulk为512字节、FS Bulk为64字节；设备退到Full-Speed时由栈自动采用较小包长。
- 当前树莓派已实测枚举为High-Speed 480 Mbps，100 Hz文本探针已通过；240 Hz二进制定长帧仍标记为待实机端到端验收，配置和理论带宽不能替代计数结果。

## 视觉质量建议

当前推荐流程为：预测/固定 ROI → 灰度 → 阈值分割 → 轻量开闭运算 → 轮廓提取 → 面积、半径、圆度 `4*pi*A/P^2`、预测位置门限 → moments 与最小包围圆联合定圆心。钢球反光时不能只取最大轮廓；优先改善漫反射补光和深色背景，Hough 圆仅用于失锁恢复。

`confidence` 建议综合圆度、面积/半径一致性、预测残差、ROI裁剪和曝光稳定性。EdgeTalk 可把它映射为随帧变化的观测方差，但低置信度不能绕过创新门限。

## Golden frame

跨语言测试使用以下 64 字节帧；CRC 为 `0xDC5219D3`：

```text
a55a010140003300403020100807060504030201c40900000080f642
000036420000d8407b142ebd0000703f00000f431000140040017800
8d200000d31952dc
```
