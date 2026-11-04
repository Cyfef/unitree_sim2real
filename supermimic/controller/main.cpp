#include "FSM/CtrlFSM.h"
#include "FSM/State_Passive.h"
#include "FSM/State_FixStand.h"
#include "FSM/State_RLBase.h"
#include "State_Mimic.h"
#include "State_SuperMimic.h"
#include "G1Safety.h"
#include <atomic>
#include <csignal>
#include <net/if.h>
#include <cstring>

namespace {
std::atomic<bool> stop_requested{false};
void request_stop(int) { stop_requested.store(true); }
}

std::unique_ptr<LowCmd_t> FSMState::lowcmd = nullptr;
std::shared_ptr<LowState_t> FSMState::lowstate = nullptr;
std::shared_ptr<Keyboard> FSMState::keyboard = std::make_shared<Keyboard>();

void init_fsm_state()
{
    auto lowcmd_sub = std::make_shared<unitree::robot::g1::subscription::LowCmd>();
    usleep(0.2 * 1e6);
    if(!lowcmd_sub->isTimeout())
    {
        spdlog::critical("The other process is using the lowcmd channel, please close it first.");
        // Never release another controller automatically or publish alongside it.
        std::exit(EXIT_FAILURE);
    }
    FSMState::lowcmd = std::make_unique<LowCmd_t>();
    FSMState::lowstate = std::make_shared<LowState_t>();
    spdlog::info("Waiting for connection to robot...");
    FSMState::lowstate->wait_for_connection();
    spdlog::info("Connected to robot.");
}

int main(int argc, char** argv)
{
    // Load parameters
    auto vm = param::helper(argc, argv);
    if (param::config["deployment_profile"].as<std::string>("") != "loaded_posture_v5") {
        std::cerr << "Requires a fresh loaded_posture_v5 candidate prepared and verified for this controller.\n";
        return 1;
    }
    spdlog::info("SuperMimic unified profile: loaded_posture_v5");
    // Full-length candidates must never open a physical network interface.
    bool simulation_only=param::config["FSM"]["Velocity"]["simulation_only"].as<bool>(false);
#ifdef SUPERMIMIC_SIMULATION_JOINT_LIMITS
    simulation_only=true;
    // Dedicated sibling build directory; preserve package-relative asset paths.
    param::proj_dir=param::bin_path.parent_path().parent_path();
    param::config_dir=param::proj_dir/"config";
    spdlog::warn("Simulation-only training joint limits; physical interfaces prohibited");
#endif
    if (simulation_only) {
        bool isolated = vm["network"].as<std::string>() == "lo";
        struct if_nameindex* interfaces = if_nameindex();
        if (!interfaces) isolated = false;
        else {
            for (auto* it=interfaces; it->if_index; ++it)
                if (std::strcmp(it->if_name,"lo") != 0) isolated = false;
            if_freenameindex(interfaces);
        }
        if (!isolated) {
            std::cerr << "Simulation-only candidate requires a network namespace containing only lo.\n";
            return 1;
        }
    }
    const auto startup=param::config["Startup"];
    bool auto_stand=startup && startup["auto_stand"].as<bool>(false);
    const double stand_delay=startup ? startup["stand_delay_seconds"].as<double>(3.) : 3.;
    if (auto_stand && (!param::config["Safety"] || !param::config["Safety"]["reference"] ||
        !std::isfinite(stand_delay) || stand_delay<1 || stand_delay>30)) {
        std::cerr << "Automatic stand requires a bound reference and a delay in [1, 30] seconds.\n";
        return 1;
    }
    std::signal(SIGINT, request_stop);
    std::signal(SIGTERM, request_stop);

    std::cout << " --- Unitree Robotics --- \n";
    std::cout << "     G1-29dof Controller \n";

    // Unitree DDS Config
    unitree::robot::ChannelFactory::Instance()->Init(0, vm["network"].as<std::string>());

    init_fsm_state();

    FSMState::lowcmd->msg_.mode_machine() = 5; // 29dof
    if(!FSMState::lowcmd->check_mode_machine(FSMState::lowstate)) {
        spdlog::critical("Unmatched robot type.");
        exit(-1);
    }
    
    auto safety=std::make_shared<G1Safety>(FSMState::lowstate,param::config["Safety"]);
    FSMState::command_guard=[safety](const std::string& phase) {
        safety->check(FSMState::lowcmd->msg_,phase);
    };
    FSMState::trip_fault=[safety](const std::string& reason) { safety->trip(reason); };
    FSMState::request_stop=[] { stop_requested.store(true); };

    // Initialize FSM
    auto fsm = std::make_unique<CtrlFSM>(param::config["FSM"]);
    fsm->start();
    const auto startup_time=std::chrono::steady_clock::now();
    if (auto_stand) spdlog::warn("Automatic FixStand requested in {:.1f} seconds; policy is not automatic",stand_delay);

    if (auto_stand)
        spdlog::info("[操作提示] 约 {:.1f} 秒后自动开始站姿恢复，无需按键；到位后终端会提示 R1+A。",stand_delay);
    else
        spdlog::info("[操作提示] 按 L2+↑ 开始站姿恢复；到位后终端会提示下一步操作。");

    while (!stop_requested && !safety->faulted())
    {
        if (auto_stand && std::chrono::duration<double>(std::chrono::steady_clock::now()-startup_time).count()>=stand_delay) {
            auto_stand=false;
            fsm->request_stand();
        }
        usleep(20000);
    }
    fsm.reset();
    FSMState::command_guard={};
    FSMState::trip_fault={};
    FSMState::request_stop={};
    // Keep publishing disabled commands briefly before closing DDS.
    for (int k=0; k<40; ++k) {
        FSMState::lowcmd->lock();
        for (auto& motor:FSMState::lowcmd->msg_.motor_cmd()) {
            motor.mode()=0;
            motor.kp()=motor.kd()=motor.tau()=motor.dq()=0;
        }
        FSMState::lowcmd->unlockAndPublish();
        usleep(5000);
    }
    FSMState::lowcmd.reset();
    safety->write_fault_trace();
    const int result=safety->faulted() ? 1 : 0;
    // Stop/drain callbacks while the state and DDS factory are both alive.
    // Leaving this static shared_ptr until process teardown races recvMC.
    FSMState::lowstate->Close();
    safety.reset();
    FSMState::lowstate.reset();
    spdlog::info("Controller shutdown complete; state subscription closed.");
    spdlog::info("[操作提示] 控制器已退出，已发送电机禁用指令。");
    return result;
}
