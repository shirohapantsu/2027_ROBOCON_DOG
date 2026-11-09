#pragma once

#include <iostream>
#include "serialPort/SerialPort.h"
#include "unitreeMotor/unitreeMotor.h"
#include <cstddef>
#include <stdexcept>
#include <string>
#include <vector>

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
    std::vector<MotorCmd> cmd_;
    SerialPort serial_;
    std::vector<MotorData> data_;
    std::vector<int> ids_; 

public:
    // 禁止复制
    Motor(const Motor&) = delete;
    Motor& operator=(const Motor&) = delete;

    Motor(const std::string& serial,
          const std::vector<int>& ids)
        : cmd_(ids.size()),
          serial_(serial),
          data_(ids.size()),
          ids_(15, -1) {
        for (std::size_t i = 0; i < ids.size(); ++i) {
            if (ids[i] < 0 || static_cast<std::size_t>(ids[i]) >= ids_.size()) {
                throw std::invalid_argument("电机 ID 必须在 0~14 内");
            }
            if (ids_[ids[i]] != -1) {
                throw std::invalid_argument("电机 ID 不能重复");
            }

            // 初始化电机cmd
            cmd_[i].motorType = MotorType::GO_M8010_6;
            cmd_[i].id = ids[i];
            cmd_[i].mode = 0;
            cmd_[i].q = 0.0f;
            cmd_[i].dq = 0.0f;
            cmd_[i].tau = 0.0f;
            cmd_[i].kp = 0.0f;
            cmd_[i].kd = 0.0f;
            cmd_[i].Res.L = 0;

            // 初始化接收缓冲区
            data_[i].motorType = cmd_[i].motorType;
            data_[i].correct = false;

            // 初始化id表
            ids_[ids[i]] = static_cast<int>(i);
        }

        // 所有命令和接收缓冲区初始化完成后，再统一收发。
        if(!sendCmd()) {
            throw std::runtime_error("初始化失败");
        }
        for (const auto& data : data_) {
            std::cout << static_cast<int>(data.motor_id) << "电机初始化成功\n";
        }
    }

    // 电机使能
    bool enable() {
        for (auto& cmd : cmd_) {
            cmd.mode = queryMotorMode(cmd.motorType,
                                      MotorMode::FOC);
        }
        return sendCmd();
    }

    // 电机失能
    bool disable() {
        for (auto& cmd : cmd_) {
            cmd.mode = queryMotorMode(cmd.motorType,
                                      MotorMode::BRAKE);
        }
        return sendCmd();
    }

    // 一次信息发送与反馈
    bool sendCmd() {
        // 避免本次通信失败后，沿用上次的有效标记。
        for (auto& data : data_) {
            data.correct = false;
        }

        bool ok = serial_.sendRecv(cmd_, data_);

        for (std::size_t i = 0; i < cmd_.size(); ++i) {
            const auto& data = data_[i];

            if (!data.correct) {
                std::cerr << "电机 " << cmd_[i].id
                          << " 通信或 CRC 校验失败\n";
                ok = false;
                continue;
            }

            if (data.motor_id != cmd_[i].id) {
                std::cerr << "电机 ID 不匹配，预期 "
                          << cmd_[i].id << "，收到 "
                          << static_cast<int>(data.motor_id)
                          << '\n';
                ok = false;
            }

            if (data.merror != 0) {
                std::cerr << "电机 " << cmd_[i].id
                          << " 故障码 " << data.merror << '\n';
                ok = false;
            }
        }

        return ok;
    }

    // 修改命令
    bool editCmd(int id, const MotorCtrl& ctrl) {
        if (id < 0 || static_cast<std::size_t>(id) >= ids_.size() || ids_[id] == -1) {
            std::cerr << "电机id不存在: " << id << '\n';
            return false;
        }

        auto& cmd = cmd_[ids_[id]];

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

        const float ratio = queryGearRatio(cmd.motorType);
        const float ratioSquared = ratio * ratio;
        cmd.q = target.q * ratio;
        cmd.dq = target.dq * ratio;
        cmd.kp = target.kp / ratioSquared;
        cmd.kd = target.kd / ratioSquared;
        cmd.tau = target.tau / ratio;
        return true;
    }

    // 返回最近一轮反馈的值快照
    MotorState getData(int id) const {
        if (id < 0 || static_cast<std::size_t>(id) >= ids_.size() || ids_[id] == -1) {
            std::cerr << "电机id不存在: " << id << '\n';
            return {};
        }

        const auto index = static_cast<std::size_t>(ids_[id]);
        const auto& data = data_[index];
        if (!data.correct || data.motor_id != id) {
            std::cerr << "电机 " << id << " 本轮反馈无效\n";
            return {};
        }

        const float ratio = queryGearRatio(cmd_[index].motorType);
        return MotorState{
            data.q / ratio,
            data.dq / ratio,
            data.tau * ratio,
            data.temp,
            data.merror,
            true
        };
    }
};

} // namespace motor
