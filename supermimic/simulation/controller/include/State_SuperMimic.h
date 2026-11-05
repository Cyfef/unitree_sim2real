#pragma once

#include "FSM/FSMState.h"
#include "isaaclab/algorithms/algorithms.h"
#include <Eigen/Dense>
#include <array>
#include <atomic>
#include <chrono>
#include <mutex>
#include <thread>
#include <fstream>

// Policy adapter only. DDS subscription, 1 kHz LowCmd publication, FSM,
// FixStand and G1Safety remain the copied unitree_rl_lab implementation.
class State_SuperMimic final : public FSMState {
public:
    State_SuperMimic(int state_mode, std::string state_string);
    void enter() override;
    void run() override;
    void exit() override;

private:
    struct Frame {
        std::array<float, 29> q{};
        Eigen::Matrix3f R = Eigen::Matrix3f::Identity();
        std::vector<float> goal;
    };
    struct Snapshot {
        std::array<float, 29> q{}, dq{};
        Eigen::Vector3f gyro = Eigen::Vector3f::Zero();
        uint16_t keys = 0;
        Eigen::Quaternionf quat = Eigen::Quaternionf::Identity();
    };
    struct Command { float q, kp, kd, dq, tau; };

    Snapshot snapshot() const;
    std::array<float, 99> observation(const Snapshot& s, const Frame& frame) const;
    std::vector<float> static_goal(const Frame& frame) const;
    void inference_loop() noexcept;
    void trip(const std::string& reason) noexcept;

    std::vector<Frame> frames_;
    std::unique_ptr<isaaclab::OrtRunner> model_;
    std::array<float, 29> default_q_{}, scale_{}, kp_{}, kd_{};
    std::array<float, 29> desired_{}, prev_action_{};
    std::array<Command, 29> start_command_{};
    std::vector<float> history_;
    Eigen::Matrix3f yaw_alignment_ = Eigen::Matrix3f::Identity();
    int history_frames_ = 0, goal_frames_ = 0;
    size_t frame_limit_ = 0;
    float transition_seconds_ = 1.f;
    bool standby_flow_ = false;
    std::chrono::steady_clock::time_point entered_;
    std::thread thread_;
    std::atomic<bool> running_{false};
    std::mutex target_mutex_;
    std::atomic<bool> orientation_failed_{false};
    std::ofstream orientation_trace_;
};

REGISTER_FSM(State_SuperMimic)
