#pragma once

#include "Types.h"
#include "param.h"
#include <array>
#include <algorithm>
#include <atomic>
#include <cmath>
#include <chrono>
#include <string>
#include <fstream>
#include <iomanip>
#include <vector>
#include <yaml-cpp/yaml.h>
#include <spdlog/spdlog.h>

// Software checks and a PD torque budget, not a hardware emergency stop.
// Model angle/effort limits plus deployment-specific speed and PD budgets.
// The optional simulation build uses training motor limits during policy execution.
class G1Safety
{
public:
    explicit G1Safety(LowState_t::SharedPtr state, YAML::Node cfg)
        : state_(std::move(state)), last_update_(Clock::now())
    {
        if (cfg && cfg["trace_path"]) {
            std::filesystem::path p=cfg["trace_path"].as<std::string>();
            if (p.is_relative()) p=param::proj_dir/p;
            trace_path_=p.string();
            trace_.reserve(trace_capacity_);
        }
        if (cfg && cfg["wrist_command_budget"]) {
            wrist_command_budget_=cfg["wrist_command_budget"].as<float>();
            if (!std::isfinite(wrist_command_budget_) || wrist_command_budget_<=0 || wrist_command_budget_>4.5f)
                throw std::runtime_error("wrist_command_budget must be in (0, 4.5] Nm");
        }
        for (int i=0;i<29;++i) stand_caps_[i]=stand_torque(i);
        if (cfg && cfg["stand_torque_limits"]) {
            auto caps=cfg["stand_torque_limits"].as<std::vector<float>>();
            if (caps.size()!=29) throw std::runtime_error("Invalid stand torque limits");
            for (int i=0;i<29;++i) {
                const float maximum=(i==16 || i==23) ? 12.5f : stand_torque(i);
                if (!std::isfinite(caps[i]) || caps[i]<=0 || caps[i]>maximum)
                    throw std::runtime_error("Stand torque budget outside permitted range");
                stand_caps_[i]=caps[i];
            }
        }
        std::lock_guard<std::mutex> lock(state_->mutex_);
        last_tick_ = state_->msg_.tick();
        if (cfg && cfg["reference"]) {
            std::filesystem::path p=cfg["reference"].as<std::string>();
            if (p.is_relative()) p=param::proj_dir/p;
            auto ref = YAML::LoadFile(p.string())["q"].as<std::vector<float>>();
            if (ref.size() != 29) throw std::runtime_error("reference must contain 29 joints");
            for (int i=0; i<29; ++i) {
                if (!std::isfinite(ref[i]) || std::abs(state_->msg_.motor_state()[i].q()-ref[i])>.02f)
                    throw std::runtime_error("startup pose differs from validated reference at joint " + std::to_string(i));
            }
        }
    }

    bool faulted() const { return fault_.load(); }

    // A policy inference fault uses the same disable path as a state fault.
    void trip(const std::string& reason) { fail(reason); }

    // Call only after stopping the FSM and publishing disabled commands.
    // File I/O must never delay the command that stops the motors.
    void write_fault_trace() const
    {
        if (!faulted() || trace_path_.empty()) return;
        std::ofstream out(trace_path_);
        if (!out) { spdlog::error("Cannot write safety trace: {}",trace_path_); return; }
        out << std::setprecision(10) << "t,policy,tick,quat_w,quat_x,quat_y,quat_z,gyro_x,gyro_y,gyro_z";
        // requested_q is the policy request before this guard's PD budget.
        for (const auto* field:{"q","dq","tau","error","requested_q","kp","kd","command_mode"})
            for (int i=0;i<29;++i) out << ',' << field << '_' << i;
        // These fields are the payload after the budget/disable checks,
        // immediately before publishing. They do not confirm motor delivery.
        for (const auto* field:{"output_q","output_dq","output_tau","output_kp","output_kd","output_mode"})
            for (int i=0;i<29;++i) out << ',' << field << '_' << i;
        out << '\n';
        const auto first=trace_.size()==trace_capacity_ ? trace_next_ : 0;
        for (size_t k=0;k<trace_.size();++k) {
            const auto& sample=trace_[(first+k)%trace_.size()];
            out << sample.t << ',' << sample.policy << ',' << sample.state.tick();
            for (auto v:sample.state.imu_state().quaternion()) out << ',' << v;
            for (auto v:sample.state.imu_state().gyroscope()) out << ',' << v;
            for (int field=0;field<8;++field) for (int i=0;i<29;++i) {
                const auto& s=sample.state.motor_state()[i];
                const auto& c=sample.command.motor_cmd()[i];
                const double values[]={s.q(),s.dq(),s.tau_est(),double(s.motorstate()),
                                       c.q(),c.kp(),c.kd(),double(c.mode())};
                out << ',' << values[field];
            }
            for (int field=0;field<6;++field) for (int i=0;i<29;++i) {
                const auto& c=sample.output.motor_cmd()[i];
                const double values[]={c.q(),c.dq(),c.tau(),c.kp(),c.kd(),double(c.mode())};
                out << ',' << values[field];
            }
            out << '\n';
        }
        spdlog::info("Saved {} safety samples to {}",trace_.size(),trace_path_);
    }

    void check(LowCmd_t::MsgType& command, const std::string& phase)
    {
        TraceSample* recorded=nullptr;
        LowState_t::MsgType state;
        {
            std::lock_guard<std::mutex> lock(state_->mutex_);
            state = state_->msg_;
        }
        const auto now = Clock::now();
        // Also record a fault raised by the trajectory/inference before this
        // guard runs, so its disabled payload appears in the saved trace.
        if (!trace_path_.empty()) {
            TraceSample sample{std::chrono::duration<double>(now-start_).count(),
                               phase=="Velocity",state,command};
            if (trace_.size()<trace_capacity_) trace_.push_back(std::move(sample));
            else trace_[trace_next_]=std::move(sample);
            recorded=&trace_[trace_.size()<trace_capacity_ ? trace_.size()-1 : trace_next_];
            trace_next_=(trace_next_+1)%trace_capacity_;
        }
        if (!faulted()) {
            if (state.tick() != last_tick_) {
                last_tick_ = state.tick();
                last_update_ = now;
            }
            if (now-last_update_ > std::chrono::milliseconds(50)) fail("state tick stale for 50 ms");
            if (crc32_core(reinterpret_cast<uint32_t*>(&state), (sizeof(state)>>2)-1) != state.crc())
                fail("invalid low-state CRC");
            if (state.mode_machine()!=5 || state.mode_pr()!=0) fail("unexpected machine/ankle mode");
            const auto& quat = state.imu_state().quaternion();
            double norm=0;
            for (auto value:quat) norm+=value*value;
            if (!std::isfinite(norm) || norm<.98 || norm>1.02) fail("invalid pelvis quaternion");
            const bool policy = phase=="Velocity";
            policy_seen_ = policy_seen_ || policy;
            // Once policy has run, preserve Passive damping semantics.
            // Do not reintroduce the local motion guards on the transition tick.
            const bool reference_motion = policy_seen_;
            // Shared reference-policy profile: orientation
            // transitions to Passive in State_SuperMimic, not a latched stop here.
            if (!reference_motion && 1-2*(quat[1]*quat[1]+quat[2]*quat[2]) < std::cos(.7)) fail("pelvis tilt exceeds 0.7 rad");
            for (auto value:state.imu_state().gyroscope())
                if (!std::isfinite(value) || (!reference_motion && std::abs(value)>6)) fail("invalid/excessive pelvis angular speed");
            for (int i=0; i<29 && !faulted(); ++i) {
                const auto& motor=state.motor_state()[i];
                auto& target=command.motor_cmd()[i];
                const float cap = policy ? rated_[i] : stand_caps_[i];
                float command_cap=policy && wrist(i) ? std::min(.9f*cap,wrist_command_budget_) : .9f*cap;
                float speed = policy ? (i<15 ? 15.f : 8.f) : (phase=="FixStand" ? 1.f : 3.f);
#ifdef SUPERMIMIC_REFERENCE_POLICY_PROTECTION
                // The same reference-policy behavior is used for simulation and hardware.
                // Training robot.py motor velocity/effort values, in SDK joint order.
                // FixStand has no training equivalent and retains its existing budgets.
                if (policy) { speed=training_speed_[i]; command_cap=cap; }
#endif
                if (motor.motorstate()!=0 || !std::isfinite(motor.q()) || !std::isfinite(motor.dq()) ||
                    !std::isfinite(motor.tau_est())) { fail("motor fault/nonfinite state at " + std::to_string(i)); break; }
                if (!reference_motion && (motor.q()<lower_[i]-.03f || motor.q()>upper_[i]+.03f)) { fail("joint outside model range at " + std::to_string(i)); break; }
                if (!reference_motion && (std::abs(motor.dq())>speed || std::abs(motor.tau_est())>cap)) {
                    fail("motor speed/torque limit at " + std::to_string(i)+
                         " phase="+phase+" dq="+std::to_string(motor.dq())+
                         " speed_limit="+std::to_string(speed)+" tau="+std::to_string(motor.tau_est())+
                         " torque_limit="+std::to_string(cap)+" q="+std::to_string(motor.q())); break;
                }
                if (!std::isfinite(target.q()) || !std::isfinite(target.dq()) || !std::isfinite(target.tau()) ||
                    !std::isfinite(target.kp()) || !std::isfinite(target.kd()) || target.kp()<0 || target.kp()>300 ||
                    target.kd()<0 || target.kd()>20 || target.mode()>1) { fail("invalid command at " + std::to_string(i)); break; }
                if (phase=="FixStand" && std::abs(target.q()-motor.q())>.25f) {
                    fail("stand tracking error exceeds 0.25 rad at " + std::to_string(i)+
                         " q="+std::to_string(motor.q())+" target="+std::to_string(target.q())+
                         " dq="+std::to_string(motor.dq())+" tau="+std::to_string(motor.tau_est())); break;
                }
                // Match the model's actuator force saturation. This bounds
                // calculated PD+FF at the received state, not physical torque.
#ifdef SUPERMIMIC_REFERENCE_POLICY_PROTECTION
                // Training / old sim2sim saturates torque in the physics actuator,
                // without rewriting the policy's position target at the DDS rate.
                if (!reference_motion) {
#endif
                if (target.kp()>0) {
                    const float other=target.kd()*(target.dq()-motor.dq())+target.tau();
                    const float effort=target.kp()*(target.q()-motor.q())+other;
                    target.q()=motor.q()+(std::clamp(effort,-command_cap,command_cap)-other)/target.kp();
                } else if (target.mode() && std::abs(target.kd()*(target.dq()-motor.dq())+target.tau())>command_cap) {
                    fail("damping/feedforward exceeds command budget at " + std::to_string(i));
                }
#ifdef SUPERMIMIC_REFERENCE_POLICY_PROTECTION
                }
#endif
            }
        }
        for (int i=29; i<35; ++i) disable(command.motor_cmd()[i]);
        if (faulted()) for (auto& motor:command.motor_cmd()) disable(motor);
        if (recorded) recorded->output=command;
    }

private:
    bool policy_seen_=false;
    using Clock=std::chrono::steady_clock;
    LowState_t::SharedPtr state_;
    std::atomic<bool> fault_{false};
    uint32_t last_tick_;
    Clock::time_point last_update_;
    struct TraceSample {
        double t;
        bool policy;
        LowState_t::MsgType state;
        LowCmd_t::MsgType command;
        LowCmd_t::MsgType output;
    };
    static constexpr size_t trace_capacity_=2048;
    std::vector<TraceSample> trace_;
    size_t trace_next_=0;
    std::string trace_path_;
    float wrist_command_budget_=4.5f;
    Clock::time_point start_=Clock::now();
    void fail(const std::string& reason) {
        if (!fault_.exchange(true)) spdlog::critical("Safety stop: {}",reason);
    }
    template<class Motor> static void disable(Motor& m) {
        m.mode()=0; m.kp()=m.kd()=m.tau()=m.dq()=0;
    }
    static bool wrist(int i) { return i==20 || i==21 || i==27 || i==28; }
    static float stand_torque(int i) {
        if (i<12) return 30;
        if (i<15) return 15;
        return wrist(i) ? 3.f : 8.f;
    }
    std::array<float,29> stand_caps_{};
#ifdef SUPERMIMIC_REFERENCE_POLICY_PROTECTION
    const std::array<float,29> training_speed_={32,20,32,20,37,37,32,20,32,20,37,37,32,37,37,
                                               37,37,37,37,37,22,22,37,37,37,37,37,22,22};
#endif
    const std::array<float,29> rated_={88,139,88,139,50,50,88,139,88,139,50,50,88,50,50,
                                    25,25,25,25,25,5,5,25,25,25,25,25,5,5};
    const std::array<float,29> lower_={-2.5307,-.5236,-2.7576,-.087267,-.87267,-.2618,
        -2.5307,-2.9671,-2.7576,-.087267,-.87267,-.2618,-2.618,-.52,-.52,
        -3.0892,-1.5882,-2.618,-1.0472,-1.97222,-1.61443,-1.61443,
        -3.0892,-2.2515,-2.618,-1.0472,-1.97222,-1.61443,-1.61443};
    const std::array<float,29> upper_={2.8798,2.9671,2.7576,2.8798,.5236,.2618,
        2.8798,.5236,2.7576,2.8798,.5236,.2618,2.618,.52,.52,
        2.6704,2.2515,2.618,2.0944,1.97222,1.61443,1.61443,
        2.6704,1.5882,2.618,2.0944,1.97222,1.61443,1.61443};
};
