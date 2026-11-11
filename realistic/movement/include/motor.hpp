#pragma once

#include <iostream>
#include "serialPort/SerialPort.h"
#include "unitreeMotor/unitreeMotor.h"
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

namespace motor {

enum class CtrlMode {
    VelMode,    // 速度模式
    PosMode,    // 位置模式
    DamMode,    // 阻尼模式
    TorMode,    // 力矩模式
    MITMode     // 混合模式
};

struct MotorCtrl {
    CtrlMode mode = CtrlMode::DamMode;
    float q = 0.0f;    // rad
    float dq = 0.0f;   // rad/s
    float kp = 0.0f;   // N*m/rad
    float kd = 0.0f;   // N*m*s/rad
    float tau = 0.0f;  // N*m
};

struct MotorState {
    float q = 0.0f;    // 输出轴位置，rad
    float dq = 0.0f;   // 输出轴速度，rad/s
    float tau = 0.0f;  // 输出轴力矩估计，N*m，未计传动损耗
    int temp = 0;     // 摄氏度
    int error = 0;    // 电机故障码，仅在 valid 为 true 时有效
    bool valid = false; // 本轮反馈通过校验且 ID 匹配，不表示无电机故障
};

class Motor {
private:
    MotorCmd cmd_;
    std::shared_ptr<SerialPort> serial_;
    MotorData data_;

public:
    // 禁止复制
    Motor(const Motor&) = delete;
    Motor& operator=(const Motor&) = delete;

    // 同一总线上的电机实例共享同一个串口，调用方串行调度收发。
    Motor(std::shared_ptr<SerialPort> serial, int id)
        : serial_(std::move(serial)) {
        if (!serial_) {
            throw std::invalid_argument("串口不能为空");
        }
        if (id < 0 || id > 14) {
            throw std::invalid_argument("电机 ID 必须在 0~14 内");
        }

        cmd_.motorType = MotorType::GO_M8010_6;
        cmd_.id = static_cast<unsigned short>(id);
        cmd_.mode = queryMotorMode(cmd_.motorType, MotorMode::BRAKE);
        cmd_.q = 0.0f;
        cmd_.dq = 0.0f;
        cmd_.tau = 0.0f;
        cmd_.kp = 0.0f;
        cmd_.kd = 0.0f;
        cmd_.Res.L = 0;

        data_.motorType = cmd_.motorType;
        data_.correct = false;

        if(!sendCmd()) {
            throw std::runtime_error("初始化失败");
        }
        std::cout << static_cast<int>(data_.motor_id) << "电机初始化成功\n";
    }

    // 电机使能
    bool enable() {
        cmd_.mode = queryMotorMode(cmd_.motorType, MotorMode::FOC);
        return sendCmd();
    }

    // 电机失能
    bool disable() {
        cmd_.mode = queryMotorMode(cmd_.motorType, MotorMode::BRAKE);
        return sendCmd();
    }

    // 一次信息发送与反馈
    bool sendCmd() {
        // 避免本次通信失败后，沿用上次的有效标记。
        data_.correct = false;

        if (!serial_->sendRecv(&cmd_, &data_) || !data_.correct) {
            data_.correct = false;
            std::cerr << "电机 " << cmd_.id << " 通信或 CRC 校验失败\n";
            return false;
        }
        if (data_.motor_id != cmd_.id) {
            data_.correct = false;
            std::cerr << "电机 ID 不匹配，预期 " << cmd_.id
                      << "，收到 " << static_cast<int>(data_.motor_id) << '\n';
            return false;
        }
        if (data_.merror != 0) {
            std::cerr << "电机 " << cmd_.id << " 故障码 " << data_.merror << '\n';
            return false;
        }
        return true;
    }

    // 修改命令
    bool editCmd(const MotorCtrl& ctrl) {
        MotorCtrl target;
        switch(ctrl.mode) {
            case CtrlMode::PosMode:
                target.q = ctrl.q;
                target.kp = ctrl.kp;
                target.kd = ctrl.kd;
                break;
            
            case CtrlMode::VelMode:
                target.dq = ctrl.dq;
                target.kd = ctrl.kd;
                break;
            
            case CtrlMode::DamMode:
                target.kd = ctrl.kd;
                break;
            
            case CtrlMode::TorMode:
                target.tau = ctrl.tau;
                break;

            case CtrlMode::MITMode:
                target = ctrl;
                break;
            
            default:
                std::cerr << "Illegal Control Mode!\n";
                return false;
        }

        const float ratio = queryGearRatio(cmd_.motorType);
        const float ratioSquared = ratio * ratio;
        cmd_.q = target.q * ratio;
        cmd_.dq = target.dq * ratio;
        cmd_.kp = target.kp / ratioSquared;
        cmd_.kd = target.kd / ratioSquared;
        cmd_.tau = target.tau / ratio;
        return true;
    }

    // 返回最近一轮反馈的值快照
    MotorState getData() const {
        if (!data_.correct || data_.motor_id != cmd_.id) {
            std::cerr << "电机 " << cmd_.id << " 本轮反馈无效\n";
            return {};
        }

        const float ratio = queryGearRatio(cmd_.motorType);
        return MotorState{
            data_.q / ratio,
            data_.dq / ratio,
            data_.tau * ratio,
            data_.temp,
            data_.merror,
            true
        };
    }
};

} // namespace motor
