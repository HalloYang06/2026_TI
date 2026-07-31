# 树莓派 C++ OpenCV 钢球视觉服务

该服务以640x480 MJPEG采集USB摄像头，在安装完成后使用四点标定的固定水管区域，屏蔽两端支架并输出钢球位置。它提供原始/标注MJPEG图传、录像接收端所需的画面，以及给EdgeTalk的测量元数据。

## 检测与图传策略

- 每个采集帧仅对标定水管带（540x45）做透视校正，不对完整640x480画面做透视；标注坐标会反投影回完整画面。
- 钢球候选由自适应阈值、连通轮廓的面积/圆度/宽高比筛选得到；Hough圆仅在轮廓被反光打碎时作为恢复路径。
- 候选必须同时具有钢球的暗色球体、明亮镜面高光和强曲边纹理，低对比度光影即使近似圆形也会被拒绝。连续8帧稳定识别后才建立24x24灰度钢球身份模板，运行中不会由单帧轮廓覆盖。静止时只在预测点±12像素内做相关匹配；运动时扩大到±32像素。与模板冲突的候选须连续8帧一致才重新锁定并更新模板，手、笔等遮挡则输出无效测量而非错误位置。
- `cv::KalmanFilter`跟踪位置和速度。候选偏离预测过大时被拒绝；短暂漏检时仅显示黄色预测圈，并向EdgeTalk发送`found=false`，绝不把预测位置伪装成有效测量。
- 检测和数据流保持100 Hz；MJPEG图传只编码和发送最新的60 Hz帧，避免浏览器缓冲造成图传延迟。
- 距离按当前固定安装的六个实测标记做一维投影像素标定：`cm = (0.0628109994 * x_px - 15.4508272224) / (0.0005801149 * x_px + 1)`。它保留相机沿水管方向的残余透视；六个标记的拟合最大残差约`0.10 cm`。若移动摄像头或水管，必须重新测量并更新`calibrated_position_cm()`中的四个系数。

接口：

- `/stream.mjpg`：原始图传。
- `/detect.mjpg`：完整现场图传，叠加水管四点框、钢球和检测帧率。
- `/data`：SSE测量流，含单调采集时间、处理耗时、圆心、半径、面积、ROI、位置和有效状态。

## 树莓派部署

```bash
sudo apt install -y build-essential cmake libopencv-dev python3-serial
cd ~/2026_TI/vision/raspberrypi/ball_camera
cmake -S . -B build
cmake --build build -j2
chmod +x start_ball_camera.sh
sudo cp ../systemd/ball-camera-cpp.service /etc/systemd/system/
sudo cp ../systemd/hball-edgetalk-camera.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now ball-camera-cpp.service hball-edgetalk-camera.service
```

`start_ball_camera.sh`会从自身所在目录启动程序；systemd文件中的用户和工作目录需与实际树莓派一致。EdgeTalk通过USB连接树莓派后，桥接服务会自动选择唯一的`/dev/serial/by-id/`设备；多个CDC设备时必须显式指定稳定路径。

验收时在电脑打开`http://<树莓派IP>:8080/detect.mjpg`，在EdgeTalk端运行`hball_usb_status`，应看到`vision_rx`递增且`vision_crc=0`。

安全：上述服务仅传输视觉测量和图传，不发送CAN或电机命令。首次启用必须断开电机/底盘动力，轮子与执行器卸载，保留可立即拔线断电的接管方式。

时间戳限制：`capture_time_us`在OpenCV的`camera.read()`返回后采自Linux单调时钟，代表帧出队/交付时刻，不是曝光中点。在接入V4L2 buffer timestamp和跨板时钟同步前，它只能用于顺序与处理延迟诊断，不能直接驱动M55历史回溯。

质量标志限制：C++服务当前不单独输出曝光稳定和轮廓圆度标志，因此桥接器不会置这些flags。`confidence=0.85`仍是shadow联调的临时常数；正式闭环前必须由圆度、半径/面积一致性、预测残差和ROI状态计算逐帧置信度。
