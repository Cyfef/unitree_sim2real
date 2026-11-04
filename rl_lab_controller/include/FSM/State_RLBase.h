// Copyright (c) 2025, Unitree Robotics Co., Ltd.
// All rights reserved.

#pragma once

#include "FSMState.h"
#include "isaaclab/envs/mdp/actions/joint_actions.h"
#include "isaaclab/envs/mdp/terminations.h"
#include <atomic>
#include <mutex>
#include <chrono>

class State_RLBase : public FSMState
{
public:
    State_RLBase(int state_mode, std::string state_string);
    
    void enter()
    {
        // Preserve the previous controller's targets and gains during an
        // optional gradual handover, including its supporting PD effort.
        start_commands.clear();
        for (int i = 0; i < env->robot->data.joint_stiffness.size(); ++i)
        {
            auto& command=lowcmd->msg_.motor_cmd()[i];
            start_commands.push_back({command.q(),command.kp(),command.kd(),command.dq(),command.tau()});
            if (transition_seconds==0) {
                command.kp()=env->robot->data.joint_stiffness[i];
                command.kd()=env->robot->data.joint_damping[i];
                command.dq()=command.tau()=0;
            }
        }
        transition_started=std::chrono::steady_clock::now();
        transition_reported=false;

        // Initialize observations and the first action before the FSM can
        // read them. The action manager is shared with the inference thread.
        {
            std::lock_guard<std::mutex> lock(policy_mutex);
            env->reset();
            env->step();
        }
        // Start policy thread
        policy_thread_running = true;
        policy_thread = std::thread([this]{
            using clock = std::chrono::high_resolution_clock;
            const std::chrono::duration<double> desiredDuration(env->step_dt);
            const auto dt = std::chrono::duration_cast<clock::duration>(desiredDuration);

            // Initialize timing
            auto sleepTill = clock::now() + dt;
            while (policy_thread_running)
            {
                std::this_thread::sleep_until(sleepTill);
                if (!policy_thread_running) break;
                {
                    std::lock_guard<std::mutex> lock(policy_mutex);
                    env->step();
                }
                sleepTill += dt;
                // Do not send a burst of inference steps after an overrun.
                if (sleepTill < clock::now()) sleepTill = clock::now() + dt;
            }
        });
    }

    void run();
    
    void exit()
    {
        policy_thread_running = false;
        if (policy_thread.joinable()) {
            policy_thread.join();
        }
    }

private:
    struct StartCommand { float q,kp,kd,dq,tau; };
    std::vector<StartCommand> start_commands;
    float transition_seconds=0;
    std::chrono::steady_clock::time_point transition_started;
    bool transition_reported=false;
    std::unique_ptr<isaaclab::ManagerBasedRLEnv> env;

    std::thread policy_thread;
    std::atomic<bool> policy_thread_running{false};
    std::mutex policy_mutex;
};

REGISTER_FSM(State_RLBase)
