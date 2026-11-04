#include "FSM/State_RLBase.h"
#include "unitree_articulation.h"
#include "isaaclab/envs/mdp/observations/observations.h"
#include "isaaclab/envs/mdp/actions/joint_actions.h"
#include <unordered_map>

namespace isaaclab
{
// keyboard velocity commands example
// change "velocity_commands" observation name in policy deploy.yaml to "keyboard_velocity_commands"
REGISTER_OBSERVATION(keyboard_velocity_commands)
{
    std::string key = FSMState::keyboard->key();
    static auto cfg = env->cfg["commands"]["base_velocity"]["ranges"];

    static std::unordered_map<std::string, std::vector<float>> key_commands = {
        {"w", {1.0f, 0.0f, 0.0f}},
        {"s", {-1.0f, 0.0f, 0.0f}},
        {"a", {0.0f, 1.0f, 0.0f}},
        {"d", {0.0f, -1.0f, 0.0f}},
        {"q", {0.0f, 0.0f, 1.0f}},
        {"e", {0.0f, 0.0f, -1.0f}}
    };
    std::vector<float> cmd = {0.0f, 0.0f, 0.0f};
    if (key_commands.find(key) != key_commands.end())
    {
        // TODO: smooth and limit the velocity commands
        cmd = key_commands[key];
    }
    return cmd;
}

}

State_RLBase::State_RLBase(int state_mode, std::string state_string)
: FSMState(state_mode, state_string) 
{
    auto cfg = param::config["FSM"][state_string];
    transition_seconds=cfg["transition_seconds"].as<float>(0.f);
    if (!std::isfinite(transition_seconds) || transition_seconds<0 || transition_seconds>10)
        throw std::runtime_error("transition_seconds must be in [0, 10]");
    auto policy_dir = param::parser_policy_dir(cfg["policy_dir"].as<std::string>());

    auto deploy_cfg=YAML::LoadFile(policy_dir / "params" / "deploy.yaml");
    for (const auto* key:{"lin_vel_x","lin_vel_y","ang_vel_z"}) {
        if (cfg["command_limits"] && cfg["command_limits"][key]) {
            const auto limits=cfg["command_limits"][key].as<std::vector<float>>();
            if (limits.size()!=2 || !std::isfinite(limits[0]) || !std::isfinite(limits[1]) || limits[0]>limits[1])
                throw std::runtime_error("invalid command limits");
            deploy_cfg["commands"]["base_velocity"]["ranges"][key]=limits;
        }
    }
    env = std::make_unique<isaaclab::ManagerBasedRLEnv>(
        deploy_cfg,
        std::make_shared<unitree::BaseArticulation<LowState_t::SharedPtr>>(FSMState::lowstate)
    );
    env->alg = std::make_unique<isaaclab::OrtRunner>(policy_dir / "exported" / "policy.onnx");

    this->registered_checks.emplace_back(
        std::make_pair(
            [&]()->bool{
                std::lock_guard<std::mutex> lock(policy_mutex);
                return isaaclab::mdp::bad_orientation(env.get(), 1.0);
            },
            FSMStringMap.right.at("Passive")
        )
    );
}

void State_RLBase::run()
{
    std::lock_guard<std::mutex> lock(policy_mutex);
    auto action = env->action_manager->processed_actions();
    const float elapsed=std::chrono::duration<float>(std::chrono::steady_clock::now()-transition_started).count();
    const float fraction=transition_seconds>0 ? std::clamp(elapsed/transition_seconds,0.f,1.f) : 1.f;
    const float alpha=fraction*fraction*(3-2*fraction);
    for(int i(0); i < env->robot->data.joint_ids_map.size(); i++) {
        const int sdk=env->robot->data.joint_ids_map[i];
        const auto& start=start_commands[sdk];
        auto& command=lowcmd->msg_.motor_cmd()[sdk];
        command.q()=(1-alpha)*start.q+alpha*action[i];
        command.kp()=(1-alpha)*start.kp+alpha*env->robot->data.joint_stiffness[sdk];
        command.kd()=(1-alpha)*start.kd+alpha*env->robot->data.joint_damping[sdk];
        command.dq()=(1-alpha)*start.dq;
        command.tau()=(1-alpha)*start.tau;
    }
    if (fraction==1 && transition_seconds>0 && !transition_reported) {
        spdlog::info("{} policy handover complete ({:.1f} s)",getStateString(),transition_seconds);
        transition_reported=true;
    }
}
