# 树莓派 C++ OpenCV 钢球视觉服务

该服务以640x480 MJPEG采集USB摄像头，自动寻找水管、屏蔽两端黑色支架、在有效管段内检测钢球并输出位置。它提供原始/标注MJPEG图传、录像接收端所需的画面，以及给EdgeTalk的测量元数据。

接口：

- `/stream.mjpg`：原始图传。
- `/detect.mjpg`：水管ROI、端部屏蔽线、钢球和帧率标注。
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

服务路径默认用户为`halloyang`，如树莓派使用其他用户名，先改两个systemd文件和`start_ball_camera.sh`中的`/home/halloyang`。EdgeTalk通过USB连接树莓派后，桥接服务会自动选择唯一的`/dev/serial/by-id/`设备；多个CDC设备时必须显式指定稳定路径。

验收时在电脑打开`http://<树莓派IP>:8080/detect.mjpg`，在EdgeTalk端运行`hball_usb_status`，应看到`vision_rx`递增且`vision_crc=0`。

安全：上述服务仅传输视觉测量和图传，不发送CAN或电机命令。首次启用必须断开电机/底盘动力，轮子与执行器卸载，保留可立即拔线断电的接管方式。
