# 树莓派钢球视觉

## 选型

正式方案优先使用现有树莓派和100 Hz摄像头。NanoPi M5作为视觉备选，不和树莓派同时进入正式控制链，避免增加相机占用、时间同步、供电和网络故障点。只有树莓派在100 Hz稳定性、延迟或识别性能上实测不达标时才切换。

## 输出合同

灰度ROI、阈值/形态学、轮廓提取、圆度/面积筛选和圆心拟合全部在树莓派本地完成。每帧只向 EdgeTalk发送控制所需的结构化结果：

- 图像曝光/采集单调时间戳。
- 沿25 cm凹槽坐标系的钢球位置，单位米或毫米，并声明符号方向和零点。
- 检测置信度、有效标志、遮挡/过曝/模糊状态。
- 圆心、半径、轮廓面积和ROI坐标，供异常诊断；ROI坐标只是4个`u16`元数据，不包含像素。
- 曝光时间和处理耗时，便于拆分采集延迟与USB延迟。

不传输灰度ROI像素、二值图、轮廓点集、完整图像或比赛录像。以`320x120`灰度ROI估算，
像素流在100 Hz已达`3.84 MB/s`，且会把图像拷贝和抖动引入M33；64字节测量帧只需
`6.40 kB/s`。

固定线协议见 [VISION_MEASUREMENT_V1](../../shared/protocol/VISION_MEASUREMENT_V1.md)。`vision_measurement_protocol.py` 提供 64 字节小端帧编码、CRC32C、解码与坏帧重同步。

在断开电机/底盘动力、轮子和执行器卸载、仅调试器/USB供电且操作员可直接拔线断电的台架上，可用以下命令验证 240 Hz 字节流。它只发送合成视觉测量，不发送 CAN 或运动命令：

```bash
python3 edgetalk_vision_stream.py --duration 30 --rate 240 --min-rate 237.6
```

主机必须报告`HOST_PASS`、`deadline_misses=0`和`achieved_rate_hz>=237.6`。结果还需和 EdgeTalk FinSH 的 `hball_usb_status` 对拍：`vision_rx`增量应等于`tx_frames`，`vision_rate_x10>=2376`，CRC、乱序和序号空洞均应为0。`500 Hz / 32 kB/s`仅作为USB、解析器和调度压力档，不要求相机达到500 FPS。

正式发送的USB单次写超时为20 ms；发生背压时跳过已经过期的采集时隙，不把旧帧排队补发。64字节帧应逐帧立即提交，不能为追求USB包利用率等待凑满512字节。

## 真实 OpenCV 相机桥接

`edgetalk_camera_bridge.py`用于已部署的`ball_vision_sender` C++服务。该服务在
`http://127.0.0.1:8080/data`提供每个新视觉序号的SSE JSON；桥接程序只取该测量元数据，
转换为`VISION_MEASUREMENT_V1`后写入EdgeTalk USB CDC。图像仍只用于电脑图传/录像，绝不
发送到控制链。

当前服务的`capture_time_us`是`camera.read()`返回后的Linux单调时刻，不是曝光中点；
桥接器也没有可靠的曝光稳定/轮廓圆度输入，因此不会伪置对应质量位。上述两项和逐帧
置信度在完成前，真实相机链路只进入shadow验证。

```bash
sudo apt install -y python3-serial
python3 edgetalk_camera_bridge.py
```

它优先使用唯一的`/dev/serial/by-id/`设备；存在多个设备时必须用`--port`传入明确的稳定路径。
发送端有独占锁，不能与仅握手的`edgetalk_usb_daemon.py`同时占用同一个CDC设备。正式接管建议
使用不硬编码用户身份的`systemd/hball-edgetalk-camera-user.service`：

```bash
mkdir -p ~/.config/hball ~/.config/systemd/user
cp systemd/edgetalk-camera.env.example ~/.config/hball/edgetalk-camera.env
cp systemd/hball-edgetalk-camera-user.service ~/.config/systemd/user/
# 编辑HBALL_CAMERA_BRIDGE为本机脚本绝对路径
systemctl --user disable --now hball-edgetalk-usb.service
systemctl --user daemon-reload
systemctl --user enable --now hball-edgetalk-camera-user.service
```

若要退回PING-only探针，先禁用真实相机服务，再启用`hball-edgetalk-usb.service`。两者使用同一个
进程锁，不能同时运行。真实相机服务只发送视觉测量帧，不会启用电机、发送CAN或产生运动命令。

球速不由树莓派用相邻两帧直接差分后作为控制量。EdgeTalk根据100 Hz带时间戳位置，
在OOSM Kalman中估计球速和扰动。

## 性能目标

- 采集与识别：稳定100 Hz，帧周期10 ms，不能用重复帧冒充帧率。
- EdgeTalk二进制接收链路继续按至少240 Hz验收，为真实100 Hz视觉保留调度余量。
- 输出噪声：先争取静止标准差小于1.2 mm。
- 单帧识别P95处理时间：第一版目标小于8 ms；超过10 ms时必须丢弃旧帧并转到最新帧。
- 端到端P95延迟：第一版目标小于25 ms，并分别记录曝光、处理和传输耗时。
- 最长连续空窗：第一版目标小于100 ms。
- 离线保存比赛要求的视频；控制链不向EdgeTalk传整帧图像。

## 标定与测试

1. 标定像素到轨道坐标的一维映射和镜头畸变。
2. 用静止钢球统计噪声、漂移、反光离群点和置信度分布。
3. 用LED/屏幕时间码或硬件触发测量采集到数据包到达的端到端延迟。
4. 保存不含个人信息的小型参数化测试样例；大视频和原始数据不提交 Git。
5. 回放测试只产生位置消息，不能通过测试脚本解锁电机或启动车辆。

## USB 开机安全守护

`edgetalk_usb_daemon.py`用于树莓派开机后的第一阶段链路维护。它按以下顺序选择串口：

1. 显式`--port`参数。
2. `/dev/serial/by-id/`中的唯一稳定路径；存在多个设备时优先名称含`HBall`的唯一项。
3. 仅在没有by-id设备时，回退到唯一的`/dev/ttyACM*`；多个候选时拒绝猜测。

守护进程等待设备出现，USB断线、READY超时或PING超时后关闭端口并重新发现。收到EdgeTalk心跳后记录`READY: sequence=... uptime_ms=...`，作为开机握手证据。串口以pyserial的`exclusive=True`打开，同时用`flock`进程锁阻止第二个实例占用同一控制链。

安全边界：此开机服务只接收`HBALL_USB_READY`并发送`PING`，不会调用`edgetalk_vision_stream.py`，不会发送64字节视觉帧，更不会用合成轨迹冒充`POSITION_VALID=1`的真实钢球位置。真实相机算法接管前保持这个PING-only状态；后续正式视觉进程在未检出钢球时必须发送`POSITION_VALID=0`，不得回退到合成有效位置。

### 安装为用户服务

树莓派先安装运行依赖：

```bash
sudo apt update
sudo apt install python3-serial
```

在仓库的`vision/raspberrypi`目录执行：

```bash
mkdir -p ~/.config/hball ~/.config/systemd/user
cp systemd/edgetalk-usb.env.example ~/.config/hball/edgetalk-usb.env
cp systemd/hball-edgetalk-usb.service ~/.config/systemd/user/
```

编辑`~/.config/hball/edgetalk-usb.env`，把`HBALL_USB_DAEMON`改为本机脚本的绝对路径。配置不包含用户名、IP、口令或设备序列号。然后启用服务：

```bash
systemctl --user daemon-reload
systemctl --user enable --now hball-edgetalk-usb.service
systemctl --user status hball-edgetalk-usb.service
```

若要求未登录时也随系统启动，由操作者显式执行`sudo loginctl enable-linger "$USER"`。日志只用于确认`WAIT`、`RECONNECT`和正常PING链路：

```bash
journalctl --user -u hball-edgetalk-usb.service -f
```

首次连接必须在底盘动力断开、轮子和执行器卸载、仅调试器或限流USB供电、急停可用且操作者能立即拔线断电的台架上进行。服务不发送CAN或任何运动命令。需要运行人工视觉吞吐测试时，先停止守护服务释放独占串口；测试结束后再重新启动，避免两个进程争抢端口。
