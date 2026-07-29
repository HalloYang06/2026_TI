# H题滚球控制算法仿真

本目录保存车载平衡滚球系统的未验证控制原型。内容属于 `experiments/`，只有在实物重复测试通过后才可提升为正式实现。

## 目标

- 建立适合 EdgeTalk 落地的非线性扰动前馈 + 增广 LQR/LQG 数学模型，验证其在转弯、视觉延迟、摩擦和执行器退化条件下的可行性。
- 模拟 60 Hz 摄像头、约 33 ms 视觉延迟、200 Hz 车体状态、1 kHz 摆杆角度内环和 20 kHz FOC 的多速率结构。
- 注入小车纵向加减速、0.5 m 半径弯道偏航、车身俯仰、传感器噪声和执行器限幅。
- 输出峰值误差、均方根误差、超出 1 cm 的时间、最大摆角、闭环极点、算法消融、题目功能项和分级压力失效边界。

## 安全边界

本目录只运行数值仿真，不产生 PWM、CAN 使能、无刷解锁或底盘运动命令。后续台架验证必须断开底盘电机动力、架空车轮、使用限流电源和硬件急停，并由操作员保留断电接管能力。

## 运行方式

脚本使用 Python 3.11.9、NumPy 2.4.6、Matplotlib 3.10.9 和 pytest 8.3.5。所有参数先作为仿真假设，真实硬件型号、连杆映射、IMU延迟和电机带宽确定后再更新。

```powershell
python -m pytest -q experiments\h_ball_control_sim\tests
python experiments\h_ball_control_sim\run_lqr_study.py
python experiments\h_ball_control_sim\run_stress_campaign.py
```

数学模型见 `LQR_MODEL.md`。`run_lqr_study.py` 输出基准、消融和 `+-5 cm` 功能测试；`run_stress_campaign.py` 运行 72 组分级压力测试。结果写入 `output/`，其中关键文件为：

- `algorithm_comparison.png`：前馈消融和静态 `+-5 cm` 跟踪。
- `stress_envelope.png`：各压力等级通过率、P95和最坏峰值。
- `boundary_failure.png`：第一失效等级的最坏种子局部时域图。
- `stress_trials.csv`：每个种子的参数和结果，可用于回填实测参数。
- `requirements_metrics.csv`：题目第 3、6 项对应的数值检查。
