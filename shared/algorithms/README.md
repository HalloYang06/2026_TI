# H-ball shared algorithms

这里仅接收跨平台、与官方Q号无关、无硬件副作用的纯算法。允许的内容包括状态估计、
LQR/LQI、滤波、稳定判据、运动学和通用限幅；禁止HAL、RTOS、CAN、USB、LVGL、全局任务
状态以及执行器发送。

代码布局固定为`include/hball_algorithms/`、`src/`和`tests/`。算法必须使用显式
`config/input/state/output`接口，并能由host和目标工具链编译。未验证实现先放
`experiments/`，通过回放对拍和重复验证后再提升到这里。

这不是按仓库目录做形式拆分：核心要求是算法函数只被上层调用，不反向读取任务状态或
调用驱动。完整函数调用关系见
[`mission-code-layout.md`](../../docs/architecture/mission-code-layout.md)。
