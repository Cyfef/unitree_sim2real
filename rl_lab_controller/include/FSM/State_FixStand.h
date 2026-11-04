// Copyright (c) 2025, Unitree Robotics Co., Ltd.
// All rights reserved.

#pragma once

#include "FSMState.h"
#include "LinearInterpolator.h"
#include "PostureProgress.h"
#include "PostureTracking.h"

class State_FixStand : public FSMState
{
public:
    State_FixStand(int state, std::string state_string = "FixStand") 
    : FSMState(state, state_string) 
    {
        ts_ = param::config["FSM"]["FixStand"]["ts"].as<std::vector<float>>();
        qs_ = param::config["FSM"]["FixStand"]["qs"].as<std::vector<std::vector<float>>>();
        assert(ts_.size() == qs_.size());
        auto cfg=param::config["FSM"]["FixStand"];
        if (cfg["taus"]) {
            taus_=cfg["taus"].as<std::vector<std::vector<float>>>();
            if (taus_.size()!=ts_.size()) throw std::runtime_error("Invalid stand feedforward samples");
            for (const auto& tau:taus_) {
                if (tau.size()!=29) throw std::runtime_error("Invalid stand feedforward dimension");
                for (int i=0;i<29;++i)
                    if (!std::isfinite(tau[i]) || (i<15 && tau[i]!=0) || std::abs(tau[i])>12.5f)
                        throw std::runtime_error("Invalid arm gravity feedforward");
            }
        }
        entry_error_=cfg["policy_entry_max_error"] ?
            cfg["policy_entry_max_error"].as<std::vector<float>>() :
            std::vector<float>(qs_.back().size(),.15f);
        entry_speed_=cfg["policy_entry_max_speed"].as<float>(1.f);
        entry_hold_=cfg["policy_entry_hold_seconds"].as<float>(0.f);
        if (entry_error_.size()!=qs_.back().size() ||
            !std::isfinite(entry_speed_) || entry_speed_<=0 || entry_speed_>1 ||
            !std::isfinite(entry_hold_) || entry_hold_<0 || entry_hold_>5)
            throw std::runtime_error("Invalid FixStand policy entry criteria");
        for (float limit:entry_error_)
            if (!std::isfinite(limit) || limit<=0 || limit>.2f)
                throw std::runtime_error("FixStand entry error must be in (0, 0.2] rad");
        if (cfg["nominal_time"]) {
            progress_=std::make_unique<PostureProgress>(cfg["nominal_time"].as<double>(),ts_.back(),
                cfg["tracking_pause_error"].as<double>(),cfg["tracking_resume_error"].as<double>(),
                cfg["tracking_pause_timeout"].as<double>(),cfg["nominal_settle_timeout"].as<double>(),
                cfg["nominal_hold_seconds"].as<double>(),cfg["pace_recovery"].as<bool>(true));
            pause_error_=cfg["tracking_pause_error"].as<float>();
            resume_error_=cfg["tracking_resume_error"].as<float>();
            nominal_q_=linear_interpolate(cfg["nominal_time"].as<float>(),ts_,qs_);
            caps_=param::config["Safety"]["stand_torque_limits"].as<std::vector<float>>();
            if (caps_.size()!=29) throw std::runtime_error("Posture progress requires 29 torque budgets");
        }
        // Do not start a learned policy partway through the posture change.
        for (auto& check:registered_checks) {
            if (check.second!=FSMStringMap.right.at("Passive")) {
                auto requested=check.first;
                check.first=[this,requested] { return requested() && ready(); };
            }
        }
    }

    void enter()
    {
        // set gain
        static auto kp = param::config["FSM"]["FixStand"]["kp"].as<std::vector<float>>();
        static auto kd = param::config["FSM"]["FixStand"]["kd"].as<std::vector<float>>();
        for(int i(0); i < kp.size(); ++i)
        {
            auto & motor = lowcmd->msg_.motor_cmd()[i];
            motor.kp() = kp[i];
            motor.kd() = kd[i];
            motor.dq() = motor.tau() = 0;
        }


        // set initial position
        std::vector<float> q0;
        {
            std::lock_guard<std::mutex> lock(lowstate->mutex_);
            for(int i(0); i < kp.size(); ++i) {
                q0.push_back(lowstate->msg_.motor_state()[i].q());
            }
        }
        qs_[0] = q0;
        t0_ = (double)unitree::common::GetCurrentTimeMillisecond() * 1e-3;
        last_time_=0;
        tracking_.reset();
        if (progress_) {
            progress_->reset();
            spdlog::info("FixStand: RECOVER_NOMINAL; RL Lab recovery with independent safety guards");
            spdlog::info("[操作提示] 正在恢复普通站姿，请等待；现在无需按 R1+A 或 R1+X。");
        }
        ready_=false;
        stable_since_=-1;
        last_report_=-1;
    }

    void run()
    {
        const double wall = (double)unitree::common::GetCurrentTimeMillisecond() * 1e-3 - t0_;
        float t=wall;
        if (progress_) {
            const auto previous=linear_interpolate(float(progress_->time()),ts_,qs_);
            const auto next=linear_interpolate(std::min(float(progress_->time()+.01),ts_.back()),ts_,qs_);
            const auto feedforward=taus_.empty() ? std::vector<float>(29,0) :
                linear_interpolate(float(progress_->time()),ts_,taus_);
            std::array<float,29> errors{};
            int saturation_joint=-1;
            bool saturated=false, nominal_stable=true;
            {
                std::lock_guard<std::mutex> lock(lowstate->mutex_);
                for (int i=0;i<29;++i) {
                    const auto& s=lowstate->msg_.motor_state()[i];
                    const auto& c=lowcmd->msg_.motor_cmd()[i];
                    const float e=std::abs(previous[i]-s.q());
                    // Legs retain the original entry tolerances during
                    // alignment; arm tracking still pauses at 0.12 rad.
                    const float normalized=e*(i<15 ? pause_error_ : .15f)/entry_error_[i];
                    errors[i]=normalized;
                    const float effort=c.kp()*(previous[i]-s.q())-c.kd()*s.dq()+feedforward[i];
                    if (i>=15 && posture_saturation_blocks_progress(previous[i]-s.q(),
                            effort,.9f*caps_[i],next[i]-previous[i])) {
                        saturated=true;
                        saturation_joint=i;
                    }
                    nominal_stable &= std::isfinite(s.q()) && std::isfinite(s.dq()) &&
                        std::abs(nominal_q_[i]-s.q())<=entry_error_[i] && std::abs(s.dq())<=entry_speed_;
                }
            }
            PostureTracking::Block block;
            if (progress_->aligning()) block=tracking_.update(errors,pause_error_,resume_error_);
            else tracking_.reset();
            const bool aligning=progress_->aligning(), paused=progress_->paused();
            progress_->advance(wall-last_time_,block.error,saturated,nominal_stable);
            if (!aligning && progress_->aligning()) {
                spdlog::info("FixStand: ALIGN_FIRST_FRAME; nominal stand stable");
                spdlog::info("[操作提示] 普通站姿已稳定，正在对齐动作首帧；请继续等待到位提示。");
            }
            if (!paused && progress_->paused())
                spdlog::warn("FixStand trajectory paused: tracking_joint={} normalized_error={:.4f}, advancing_saturated_joint={}",
                    block.joint,block.error,saturation_joint);
            if (paused && !progress_->paused()) spdlog::info("FixStand trajectory resumed");
            if (!progress_->failure().empty() && trip_fault) trip_fault(progress_->failure());
            t=progress_->time();
        }
        last_time_=wall;
        auto q = linear_interpolate(t, ts_, qs_);
        
        for(int i(0); i < q.size(); ++i) {
            lowcmd->msg_.motor_cmd()[i].q() = q[i];
        }
        if (!taus_.empty()) {
            auto tau=linear_interpolate(t,ts_,taus_);
            for (int i=0;i<29;++i) lowcmd->msg_.motor_cmd()[i].tau()=tau[i];
        }
        if (t>=ts_.back()) update_readiness(wall);
    }

private:
    double t0_;
    double last_time_=0;
    std::unique_ptr<PostureProgress> progress_;
    PostureTracking tracking_;
    std::vector<float> nominal_q_,caps_;
    std::vector<float> ts_;
    std::vector<std::vector<float>> qs_;
    std::vector<std::vector<float>> taus_;
    std::vector<float> entry_error_;
    float entry_speed_=1.f, entry_hold_=0.f;
    float pause_error_=.12f;
    float resume_error_=.08f;
    double stable_since_=-1, last_report_=-1;
    bool ready_=false;
    bool ready() const { return ready_; }
    void update_readiness(double t)
    {
        int blocked=-1;
        float error=0, speed=0;
        {
            std::lock_guard<std::mutex> lock(lowstate->mutex_);
            for (int i=0;i<qs_.back().size();++i) {
                const auto& motor=lowstate->msg_.motor_state()[i];
                const float e=std::abs(motor.q()-qs_.back()[i]), v=std::abs(motor.dq());
                if (!std::isfinite(e) || !std::isfinite(v) || e>entry_error_[i] || v>entry_speed_) {
                    blocked=i; error=e; speed=v; break;
                }
            }
        }
        if (blocked>=0) stable_since_=-1;
        else if (stable_since_<0) stable_since_=t;
        const bool next=blocked<0 && t-stable_since_>=entry_hold_;
        if (next && !ready_) {
            spdlog::info("FixStand reached: current policy entry criteria satisfied (feet contact not measured).");
            if (param::config["FSM"]["Velocity"]["standby_flow"].as<bool>(false))
                spdlog::info("[操作提示] 首帧姿态已到位。下一步：按住 R1，点按 A，然后松开，进入策略静态待机（此时不播放动作）。");
        }
        if (!next && ready_ && param::config["FSM"]["Velocity"]["standby_flow"].as<bool>(false))
            spdlog::warn("[操作提示] 到位条件已失效，暂不能进入策略待机；请等待新的到位提示。");
        if (blocked>=0 && (ready_ || last_report_<0 || t-last_report_>=5)) {
            spdlog::warn("FixStand policy entry blocked: joint {} error {:.4f}/{:.4f} rad, speed {:.4f}/{:.4f} rad/s",
                blocked,error,entry_error_[blocked],speed,entry_speed_);
            last_report_=t;
        }
        ready_=next;
    }
};

REGISTER_FSM(State_FixStand)
