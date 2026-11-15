# 四足机器人行走仿真

C++17 + MuJoCo 仿真，默认加载当前的 `generated_robot-mjcf_realModel/robot.xml`。
不依赖实机串口、宇树 SDK 或真实电机 ID，不修改 `realistic` 或原始 CAD 模型文件。

## 编译

依赖 CMake、C++17 编译器、Eigen3、MuJoCo。图形窗口还需要 GLFW3 和 OpenGL。
当前环境已安装所需依赖，直接执行：

```bash
cmake -S simulation -B simulation/build -DCMAKE_BUILD_TYPE=Release
cmake --build simulation/build -j
```

CMake 会检查当前 Python/Conda 环境，以及常见的 `miniconda3/envs/mujoco` 路径。
也可以明确指定 MuJoCo 安装目录或 Python 包目录，该目录需要包含 `include/` 和共享库：

```bash
cmake -S simulation -B simulation/build \
  -DMUJOCO_ROOT=/path/to/site-packages/mujoco
```

没有图形依赖时可用 `-DDOG_SIM_VIEWER=OFF` 编译无窗口版本。

## 运行

先观察现有逻辑生成的步态：

```bash
./simulation/build/walk_sim --mode kinematic --duration 30
```

这个模式固定机身，按逆解更新八个主动关节和八个被动关节，仅用于检查轨迹、装配分支和方向映射，不能据此判断机器人能否站稳或行走。蓝色圆点表示支撑相闭合点目标，橙色圆点表示摆动相目标。

运行自由机身动力学：

```bash
./simulation/build/walk_sim --mode dynamics --feedforward on \
  --kp 0.4 --kd 0.2 --duration 30
```

默认模式为 `dynamics`，重力/支撑力前馈默认开启。动力学模式使用重力、地面接触、主动关节前馈加 PD 力矩和被动小腿铰链；只在初始化时设置一次姿态，之后由 MuJoCo 积分推进状态。当前配置中的默认增益为 `kp=0.4、kd=0.02`，上面的命令显式使用已测试的 `kd=0.2`。

窗口操作：空格暂停/继续，`R` 重置，`Esc` 退出；鼠标左键旋转、右键平移、滚轮缩放。`Ctrl+C` 也可停止。`--speed 0.5` 可减慢播放。

没有桌面时运行：

```bash
./simulation/build/walk_sim --headless --mode dynamics --duration 30 \
  --feedforward on --kp 0.4 --kd 0.2 \
  --csv /tmp/dog-walk.csv
./simulation/build/walk_sim --headless --mode kinematic --duration 2 \
  --snapshot /tmp/dog-walk.mjb
```

CSV 每 10 ms 记录机身在“右、前、上”坐标系中的位置、闭链误差、主动关节跟踪误差、最大电机力矩、地面接触数和倒地标志。新增的 `max_feedforward_nm` 表示该帧八个前馈指令的最大绝对值，`desired_support_z_n` 是计划支撑力的竖直分量之和，不是实际测得的地面反力。MJB 快照包含最终姿态的模型，可用 MuJoCo 加载检查；它不是包含速度、接触求解历史的完整续跑状态。

关闭前馈进行对照：

```bash
./simulation/build/walk_sim --headless --mode dynamics --duration 5 \
  --feedforward off --kp 0.4 --kd 0.2
```

原地站立实验使用 `--step-length 0 --step-height 0`，此时四条腿都参与承重。

调整仿真参数的例子：

```bash
./simulation/build/walk_sim --mode dynamics --period 1.2 \
  --kp 20 --kd 0.8 --step-length 0.10 --step-height 0.025
```

这些是可调整参数的示例，不代表已经验证稳定的行走参数。`--help` 查看完整选项。

## 与现有控制逻辑的对应

| 实机逻辑 | 仿真实现 |
|---|---|
| `movement::walkstright()` 四腿相位分配 | FL/RR 先摆动，FR/RL 先支撑，归一化相位按仿真时间连续循环 |
| `leg::step_support()` | 同样的固定高度、匀速向后支撑轨迹 |
| `leg::step_swing()` | 同样的水平五次多项式和竖直 $\sin^2$ 抬脚曲线 |
| `leg::getMotorPV()` | 同样的两支链位置逆解和微分速度逆解，再转换到 CAD 关节坐标 |
| `leg::to_footPos()` | 保留可配置的输出轴 PD 控制，仿真额外计算重力/支撑力前馈；当前仿真默认增益为 `0.4 / 0.02` |
| `Motor` 方向/零偏转换 | 从 CAD 初始连杆方向和各关节轴向读取模型映射，仿真不使用实机标定参数 |

默认步态周期 1 s、步长 0.14 m、支撑高度 −0.274 m、抬脚高度 0.03 m、支撑相比例 60%，仿真步长 1 ms。
这些算法在 `src/gait.cpp` 和 `src/simulator.cpp` 中实现；当前实机类把计算与硬件通信封装在一起，因此仿真保留了独立计算实现。后续修改实机轨迹时，需要同步仿真中的相应公式和参数。

每周期的相位安排保持现状：

| 相位 | FL / RR | FR / RL |
|---|---|---|
| 0～40% | 摆动 | 支撑 |
| 40～60% | 支撑 | 支撑 |
| 60～100% | 支撑 | 摆动 |

## 模型方向与闭链

当前模型的 CAD 坐标与项目机体系满足：

$$
\begin{bmatrix}x_B\\y_B\\z_B\end{bmatrix}
=\begin{bmatrix}-1&0&0\\0&-1&0\\0&0&1\end{bmatrix}
\begin{bmatrix}x_M\\y_M\\z_M\end{bmatrix}
$$

程序根据四条腿的安装位置识别这组轴向，从各支链的初始几何选择连续逆解分支。**CAD 名称中的 `front/rear` 不直接等同于 `theta1/theta2`**：

| 腿 | `theta1 = eta + alpha` 对应 CAD 支链 | `theta2 = eta - alpha` 对应 CAD 支链 |
|---|---|---|
| FL | `FL_thigh_rear` | `FL_thigh_front` |
| FR | `FR_thigh_rear` | `FR_thigh_front` |
| RL | `RL_thigh_front` | `RL_thigh_rear` |
| RR | `RR_thigh_front` | `RR_thigh_rear` |

主动关节和被动关节分别读取自己的轴向。控制台会打印最终映射；这些轴向属于仿真模型，不能直接当作实机电机的 `direction` 标定值。

逆解使用模型中测量的实际连杆长度。当前 FR/RL 的 CAD `rear` 支链小腿约为 0.260164 m，其余约为 0.260000 m；与实机名义长度有轻微差异。程序使用实际几何以保持闭链闭合。

动力学模式保留原始 17 个刚体、约 8.108 kg 总质量、惯量、16 个铰链和四个闭链约束，并在内存中添加地面和八个主动髋执行器。接触使用原始橡胶足部网格，以及一个机身包围盒；其余重复的 CAD 碰撞网格不参与接触，腿间自碰撞未启用。

执行器输入使用输出轴 N·m，传动倍率为 1，避免再次乘减速比。采用 ±23.7 N·m 峰值、30 rad/s 空载速度构成的简化线性驱动力矩包络，制动保留峰值限幅，主动关节库仑摩擦为 0.15825 N·m。前馈指令先限制到 ±23.7 N·m，再对完整的前馈加 PD 指令施加力矩/速度限幅，避免倒地或接近奇异姿态时出现不可实现的输出。它不包含驱动延迟、温升或精确电流环模型。

## 重力与支撑力前馈

控制指令现在为：

$$
\tau=\tau_{\mathrm{ff}}+K_p(q_d-q)+K_d(\dot q_d-\dot q)
$$

每帧使用当前关节姿态，在独立的 MuJoCo 数据中将速度置零计算重力项，不改变实际仿真状态。不能直接只取八个主动关节的 `qfrc_bias`：闭链腿的被动关节也承受重力，需要通过闭链约束折算到两个主动关节。

对一条腿，把闭链速度约束按主动、被动关节拆开：

$$
C_a\dot q_a+C_p\dot q_p=0,\qquad R_p=-C_p^{-1}C_a
$$

据此折算腿自身重力和足部雅可比：

$$
g_{\mathrm{red}}=g_a+R_p^Tg_p,\qquad J_{\mathrm{red}}=J_a+J_pR_p
$$

计划支撑腿均分整机重量，地面对足端的期望反力为：

$$
f_i^{\mathrm{des}}=-\frac{m}{n_s}\boldsymbol g,\qquad
\tau_{\mathrm{ff},i}=g_{\mathrm{red},i}-J_{\mathrm{red},i}^Tf_i^{\mathrm{des}}
$$

力与雅可比均在世界坐标下计算；反力作用点近似取橡胶足部网格的中心。算出的力矩已经对应 CAD 主动关节轴向。摆动腿不分配机身支撑力，但仍补偿连杆自身重力。

当前分配只满足计划支撑力的合力平衡，没有加入机身姿态/质心反馈、实际触地检测、支撑切换平滑或支撑合力矩优化，因此它是准静态前馈实验，不是完整的动态平衡控制器。

## 当前控制效果与验证

相同模型和轨迹，`kp=0.4、kd=0.2` 下的 5 s 对照：

| 工况 | 前馈关闭：机身高度 | 前馈开启：机身高度 |
|---|---:|---:|
| 原地站立 | 0.162 m | 0.289 m |
| 行走 | 0.041 m | 0.303 m |

30 s 测试中，开启前馈、`kp=0.4、kd=0.2` 的站立实验机身最终高度约 0.286 m；行走实验前进约 0.987 m，机身最终高度约 0.307 m，均未触发倒地判据。行走时仍有约 0.30 rad 的关节跟踪 RMS 误差，不能据此认为轨迹跟踪已经充分准确。

保留当前默认的 `kd=0.02` 时，开启前馈的行走实验约在 6.96 s 触发倒地判据。前馈改善了承载，反馈阻尼和支撑力分配仍然影响稳定性。以上只验证当前平地模型；运动学预览只验证几何运动。

倒地标志采用机身高度低于 0.12 m，或机身向上轴与世界向上方向的点积低于 0.5 的判据。正常结束仿真时，即使出现倒地，退出码仍为 0；请结合 `ever_fallen`、轨迹误差和 CSV 判断控制效果。加载、参数或数值稳定性错误会返回非零退出码。

```bash
ctest --test-dir simulation/build --output-on-failure
```

测试覆盖轨迹周期连续性、四腿相位、101 个姿态的模型闭链/足端位置、主动与被动关节速度差分、两个预览周期，以及自由机身动力学的数值稳定性、地面接触和力矩限幅。前馈测试额外检查支撑合力、摆动腿卸载、零重力零补偿、闭链虚功的势能差分，以及相同低增益下的站立承载改善。当前环境使用 MuJoCo 3.14.0 验证，足端位置最大误差约 $1.3\times10^{-10}$ m。

模型加载和场景添加采用 MuJoCo 官方的 [mjSpec 模型编辑接口](https://mujoco.readthedocs.io/en/stable/programming/modeledit.html)，物理时间由 [mj_step](https://mujoco.readthedocs.io/en/stable/programming/simulation.html) 推进。

## 文件

```text
simulation/
├── CMakeLists.txt
├── README.md
├── include/walk_sim.hpp
├── src/
│   ├── gait.cpp          # 与实机一致的足端轨迹
│   ├── simulator.cpp     # 场景、CAD 映射、逆解、动力学与观测
│   ├── main.cpp          # 参数、无窗口运行、CSV 和快照
│   └── viewer.cpp        # GLFW 交互窗口
├── tests/check_simulation.cpp
└── generated_robot-mjcf_realModel/   # 原始模型，保持不变
```
