# 电机驱动封装使用说明

本文对应 [`src/movement/motor.hpp`](../src/movement/motor.hpp) 的当前实现，代码示例使用 C++17。

`motor::Motor` 封装宇树 GO-M8010-6 电机 SDK。一个对象独占一个串口，管理同一总线上的多台电机，负责命令缓存、批量收发、反馈检查和单位换算。八台电机共用串口时，只创建一个对象。

## 1. 使用流程

1. 配置各台实际电机的 ID，确保同一总线上没有重复 ID。
2. 构造 `Motor`，传入串口设备名和电机 ID 列表；构造过程会立即执行一次通信校验。
3. 用 `MotorCtrl` 设置各台电机的初始目标，调用 `editCmd()` 写入缓存。
4. 调用 `enable()`，将全部电机切换到 FOC 模式，并发送缓存中的目标。
5. 在控制循环中，先更新各台电机的目标，再调用一次 `sendCmd()`，最后通过 `getData()` 读取反馈。
6. 正常结束或退出控制循环时，调用 `disable()`，检查停用命令的通信结果。

`editCmd()` 只修改缓存；`getData()` 只读取缓存。`enable()`、`disable()` 和 `sendCmd()` 都会触发串口通信。

## 2. 创建电机组

```cpp
motor::Motor motors("/dev/ttyUSB0", {0, 1, 2, 3, 4, 5, 6, 7});
```

构造函数声明：

```cpp
Motor(const std::string& serial, const std::vector<int>& ids);
```

| 参数 | 含义 |
|---|---|
| `serial` | 串口设备路径，例如 `/dev/ttyUSB0` |
| `ids` | 当前对象管理的实际电机 ID，必须在 `0～14` 内且互不重复 |

ID 可以不连续，例如 `{0, 2, 7, 14}`。公开接口中的 `id` 始终表示实际电机 ID，不表示列表中的下标。传入列表不会修改电机自身的 ID。

当前封装不限制必须使用八台电机。请传入实际接入的非空电机列表；当前实现没有拒绝空列表。ID `15` 是无反馈的广播地址，不用于本封装的逐台反馈控制。

构造过程会初始化全部命令和反馈缓冲区，然后发送一次零控制参数、工作模式为 `0` 的命令，并检查反馈。出现下列情况时，构造失败：

- ID 越界或重复：抛出 `std::invalid_argument`。
- 首轮通信、CRC、反馈 ID 或电机故障检查未通过：抛出 `std::runtime_error`。
- 串口打开、配置或底层通信异常：可能抛出 SDK 异常。

当前使用 SDK 默认串口配置：4,000,000 波特率、8 数据位、无校验、1 停止位，接收长度为单台反馈的 16 字节，超时参数为 20,000 微秒。封装尚未提供修改这些参数的公开接口。

对象禁止复制。保持同一串口由一个通信线程使用；当前类没有为并发调用提供同步保护。

## 3. 控制目标 MotorCtrl

```cpp
struct MotorCtrl {
    CtrlMode mode = CtrlMode::DamMode;
    float q = 0.0f;
    float dq = 0.0f;
    float kp = 0.0f;
    float kd = 0.0f;
    float tau = 0.0f;
};
```

所有物理量均采用输出轴侧单位：

| 字段 | 含义 | 单位 |
|---|---|---|
| `mode` | 控制目标的组合方式 | `CtrlMode` |
| `q` | 目标位置 | rad |
| `dq` | 目标角速度 | rad/s |
| `kp` | 位置增益 | N·m/rad |
| `kd` | 速度增益 | N·m·s/rad |
| `tau` | 前馈力矩 | N·m |

默认构造的 `MotorCtrl` 使用阻尼模式，所有数值为零。这里的 `mode` 用于选择控制参数，与电机底层 FOC/BRAKE 工作模式不同；调用 `editCmd()` 不会使能电机。

### 控制模式与生效字段

| 模式 | 生效字段 | 其余控制参数 |
|---|---|---|
| `CtrlMode::PosMode` | `q`、`kp`、`kd` | 清零 `dq`、`tau` |
| `CtrlMode::VelMode` | `dq`、`kd` | 清零 `q`、`kp`、`tau` |
| `CtrlMode::DamMode` | `kd` | 清零 `q`、`dq`、`kp`、`tau` |
| `CtrlMode::TorMode` | `tau` | 清零 `q`、`dq`、`kp`、`kd` |
| `CtrlMode::MITMode` | `q`、`dq`、`kp`、`kd`、`tau` | 使用全部字段 |

忽略电机内部限制和传动损耗时，可用下面的关系理解控制输出：

$$
\tau_{\mathrm{out}}^{*}
= K_p(q^{*}-q)
+ K_d(\dot q^{*}-\dot q)
+ \tau_{\mathrm{ff}}
$$

因此，位置模式中 `kd` 提供对零目标速度的阻尼；阻尼模式的输出随实际速度变化；力矩模式关闭位置和速度反馈项。

### 设置目标

```cpp
motor::MotorCtrl ctrl;
ctrl.mode = motor::CtrlMode::PosMode;
ctrl.q = 0.5f;
ctrl.kp = 10.0f;
ctrl.kd = 0.5f;

if (!motors.editCmd(0, ctrl)) {
    // ID 不存在或控制模式非法，当前电机的旧命令保持不变。
}
```

接口为 `bool editCmd(int id, const MotorCtrl& ctrl)`。数值仅用于说明接口；具体目标和增益由上层根据机构确定。

修改多台电机时，对每个 ID 调用 `editCmd()`，随后调用一次 `sendCmd()`。一台电机的更新失败不会回滚此前已成功更新的其他电机；需要整体更新时，应先在上层检查完整目标集合，再执行发送。

## 4. 使能、发送与停用

| 接口 | 行为 |
|---|---|
| `bool enable()` | 全部缓存命令切换到 FOC 模式，然后执行一轮收发 |
| `bool sendCmd()` | 按初始化列表顺序收发全部电机的命令和反馈 |
| `bool disable()` | 全部缓存命令切换到 BRAKE 模式，然后执行一轮收发 |

三个接口均在底层通信成功且全部反馈通过 CRC、ID 和电机故障检查时返回 `true`。底层 SDK 抛出的异常会向上传播，这些接口没有将所有异常转换为 `false`。

当前本地 Linux64 SDK 的批量收发接口内部逐台执行请求和反馈，因此一个 `sendCmd()` 对应一轮总线通信，不保证八台电机在同一时刻收到命令。控制周期应包含全部电机的通信时间；额外的休眠时间不等于完整控制周期。

`sendCmd()` 返回 `false` 时，部分电机可能已经接收并执行了命令。应继续按 ID 查询反馈，区分通信失败与电机故障。

`disable()` 不清空已有目标参数，也不切断电源。重新 `enable()` 会发送保留的目标，因此重新使能前应明确设置本次需要的目标。对象析构会释放串口，但当前类没有自动调用 `disable()`；关闭串口不代表已向电机发送停用命令。

## 5. 反馈状态 MotorState

```cpp
struct MotorState {
    float q = 0.0f;
    float dq = 0.0f;
    float tau = 0.0f;
    int temp = 0;
    int error = 0;
    bool valid = false;
};
```

| 字段 | 含义 |
|---|---|
| `q` | 输出轴位置，rad |
| `dq` | 输出轴角速度，rad/s |
| `tau` | 根据减速比换算的输出轴力矩估计，N·m，未计传动损耗 |
| `temp` | 电机温度，摄氏度 |
| `error` | 电机报告的故障码，仅在 `valid == true` 时有意义 |
| `valid` | 最近一轮反馈通过校验且反馈 ID 匹配 |

接口为 `MotorState getData(int id) const`，按值返回快照。后续通信不会改变已经取得的快照。

```cpp
const motor::MotorState state = motors.getData(0);

if (!state.valid) {
    // ID 不存在，或最近一轮该电机反馈无效。
} else if (state.error != 0) {
    // 通信反馈有效，但电机报告故障；仍可读取温度等字段。
} else {
    const float position = state.q;
    const float velocity = state.dq;
    const float torque = state.tau;
    // 使用本轮反馈进行上层计算。
}
```

ID 不存在、CRC/接收失败或反馈 ID 不匹配时，`getData()` 输出错误信息并返回默认状态，即 `valid == false`。此时为零的数值字段不能解释为电机的真实状态。

`getData()` 不会主动获取新反馈。构造、`enable()`、`disable()` 或 `sendCmd()` 更新反馈缓存；若一直不再通信，`valid` 不会随时间自动变成 `false`。当前状态不包含接收时间，数据新鲜度由上层根据通信循环判断。

## 6. 八台电机完整示例

下面用阻尼控制演示配置、使能、循环收发和退出。阻尼参数是接口示例值，周期中的 5 毫秒仅为附加休眠，不代表固定的 200 Hz 控制频率。

将代码保存为项目根目录下的 `motor_driver_example.cpp`：

```cpp
#include "src/movement/motor.hpp"

#include <chrono>
#include <exception>
#include <iostream>
#include <thread>
#include <vector>

bool run(motor::Motor& motors, const std::vector<int>& ids) {
    motor::MotorCtrl ctrl;
    ctrl.mode = motor::CtrlMode::DamMode;
    ctrl.kd = 0.2f;

    // 先配置所有电机，再统一使能并发送。
    for (int id : ids) {
        if (!motors.editCmd(id, ctrl)) {
            return false;
        }
    }
    if (!motors.enable()) {
        return false;
    }

    for (int step = 0; step < 100; ++step) {
        // 动态控制时，在这里计算并 editCmd() 更新各台目标。
        const bool communicationOkay = motors.sendCmd();
        bool statesOkay = true;

        for (int id : ids) {
            const auto state = motors.getData(id);
            if (!state.valid) {
                std::cerr << "电机 " << id << " 反馈无效\n";
                statesOkay = false;
            } else if (state.error != 0) {
                std::cerr << "电机 " << id
                          << " 故障码 " << state.error << '\n';
                statesOkay = false;
            } else if (step % 20 == 0) {
                std::cout << "ID=" << id
                          << " q=" << state.q
                          << " dq=" << state.dq
                          << " tau=" << state.tau
                          << " temp=" << state.temp << '\n';
            }
        }

        if (!communicationOkay || !statesOkay) {
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return true;
}

int main() {
    try {
        const std::vector<int> ids{0, 1, 2, 3, 4, 5, 6, 7};
        motor::Motor motors("/dev/ttyUSB0", ids);

        bool controlOkay = false;
        try {
            controlOkay = run(motors, ids);
        } catch (const std::exception& e) {
            std::cerr << "控制过程异常: " << e.what() << '\n';
        }

        // run() 正常结束、返回失败或抛出标准异常后，均尝试停用。
        const bool disabled = motors.disable();
        if (!disabled) {
            std::cerr << "停用命令未全部确认\n";
        }
        return controlOkay && disabled ? 0 : 1;
    } catch (const std::exception& e) {
        // 构造或停用过程中的异常在这里报告。
        std::cerr << "驱动异常: " << e.what() << '\n';
        return 1;
    }
}
```

### 编译与运行

在项目根目录执行，下面的链接命令适用于 Linux x86_64：

```bash
g++ -std=c++17 -Wall -Wextra -Wpedantic \
    -I. -Ilib/unitree_actuator_sdk/include \
    motor_driver_example.cpp \
    -Llib/unitree_actuator_sdk/lib -lUnitreeMotorSDK_Linux64 \
    -Wl,-rpath,"$PWD/lib/unitree_actuator_sdk/lib" \
    -pthread -o motor_driver_example

./motor_driver_example
```

Linux ARM64 使用同目录中的 ARM64 库，将链接选项替换为 `-lUnitreeMotorSDK_Arm64`。运行前确认实际串口路径和当前用户的串口访问权限。

## 7. 单位换算与使用边界

封装对外统一使用输出轴侧单位。设 SDK 查询到的减速比为 $r$，命令发送前进行：

$$
q_m = r q_o,\qquad
\dot q_m = r\dot q_o,\qquad
\tau_m = \frac{\tau_o}{r}
$$

$$
K_{p,m} = \frac{K_{p,o}}{r^2},\qquad
K_{d,m} = \frac{K_{d,o}}{r^2}
$$

反馈返回前进行：

$$
q_o = \frac{q_m}{r},\qquad
\dot q_o = \frac{\dot q_m}{r},\qquad
\tau_o \approx r\tau_m
$$

调用方不要再次乘除减速比。当前换算没有包含安装方向、机构零点偏置和传动效率；例如位置零点不会在构造时自动设置为当前位置。这些机构相关处理需要在上层明确实现。

当前封装不检查 `NaN`、无穷大、负增益，也不提供软件位置、速度、力矩限幅；上层需要在生成目标时完成数值和范围检查。构造成功只表示首轮通信及故障检查通过，不表示机构已完成零点标定。

当前封装只配置 GO-M8010-6。SDK 自身支持其他型号，不代表该类已支持相应型号及串口配置。

物理模型和机器人关节映射可参见 [`motor_model.md`](motor_model.md)，其中仿真适配器的 `dog::motor` 接口与本文实机封装的 `motor` 接口不同。
