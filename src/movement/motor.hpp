#pragma once

#include <iostream>
#include "serialPort/SerialPort.h"
#include "unitreeMotor/unitreeMotor.h"
#include <string>
#include <fmt/core.h>

namespace motor {

class Motor {
private:
    std::vector<MotorCmd> cmd_;
    SerialPort serial_;
    std::vector<MotorData> data_;

public:
    Motor(const std::string& serial,
          const std::vector<int>& ids)
        : cmd_(ids.size()),
          serial_(serial),
          data_(ids.size()) {
        // 初始化电机cmd
        for(int i=0;i<ids.size();i++) {
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

            // 初始化校验
            if(!sendCmd()) {
                throw std::runtime_error("初始化失败");
            }

            std::cout << data_[i].motor_id << "电机初始化成功\n";
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

    // 一次信息发送与反馈
    bool sendCmd() {
         // 避免本次通信失败后，沿用上次的有效标记。
        for (auto& data : data_) {
            data.correct = false;
        }

        // 注意：批量接口传容器引用，不加 &。
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
};

}