// Offline replay of G1 safety traces through the deployment observation/action
// pipeline. No DDS factory, subscriber or publisher is created by this tool.
#include "isaaclab/envs/manager_based_rl_env.h"
#include "isaaclab/envs/mdp/observations/observations.h"
#include "isaaclab/envs/mdp/actions/joint_actions.h"
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <array>
#include <cmath>

namespace {
using Frame=std::unordered_map<std::string,double>;
std::vector<std::string> split(const std::string& line) {
    std::vector<std::string> result;
    std::istringstream stream(line);
    for (std::string cell;std::getline(stream,cell,',');) result.push_back(cell);
    return result;
}

class RecordedRobot:public isaaclab::Articulation {
public:
    const Frame* frame=nullptr;
    void update() override {
        const auto& f=*frame;
        data.velocity_command_axes.setZero(); // Safety traces in this task used zero command.
        data.root_quat_w=Eigen::Quaternionf(f.at("quat_w"),f.at("quat_x"),f.at("quat_y"),f.at("quat_z"));
        data.projected_gravity_b=data.root_quat_w.conjugate()*data.GRAVITY_VEC_W;
        for (int i=0;i<3;++i) data.root_ang_vel_b[i]=f.at(std::array<std::string,3>{"gyro_x","gyro_y","gyro_z"}[i]);
        for (int i=0;i<data.joint_ids_map.size();++i) {
            const auto index=std::to_string(int(data.joint_ids_map[i]));
            data.joint_pos[i]=f.at("q_"+index);
            data.joint_vel[i]=f.at("dq_"+index);
        }
    }
};
}

int main(int argc,char** argv) try {
    if (argc!=4 && argc!=5) throw std::runtime_error("Usage: g1_policy_replay POLICY_DIR ZERO_COMMAND_TRACE.csv OUTPUT.csv [OBS_OUTPUT.csv]");
    const std::filesystem::path policy=argv[1],output=argv[3];
    if (std::filesystem::exists(output)) throw std::runtime_error("Output already exists");
    if (argc==5 && std::filesystem::exists(argv[4])) throw std::runtime_error("Observation output already exists");
    std::ifstream input(argv[2]);
    if (!input) throw std::runtime_error("Cannot open input trace");
    std::string line;
    std::getline(input,line);
    const auto headers=split(line);
    std::vector<Frame> frames;
    while (std::getline(input,line)) {
        auto cells=split(line);
        if (cells.size()!=headers.size()) throw std::runtime_error("Incomplete CSV row");
        Frame f;
        for (size_t i=0;i<headers.size();++i) {
            const double value=std::stod(cells[i]);
            if (!std::isfinite(value)) throw std::runtime_error("Nonfinite trace data");
            f.emplace(headers[i],value);
        }
        if (!frames.empty() && f.at("t")<=frames.back().at("t"))
            throw std::runtime_error("Trace time must increase");
        if (f.at("policy")!=1) throw std::runtime_error("Replay requires a policy-only trace");
        frames.push_back(std::move(f));
    }
    if (frames.size()<5) throw std::runtime_error("Trace too short");
    auto cfg=YAML::LoadFile(policy/"params/deploy.yaml");
    auto robot=std::make_shared<RecordedRobot>();
    robot->frame=&frames.front();
    isaaclab::ManagerBasedRLEnv env(cfg,robot);
    env.alg=std::make_unique<isaaclab::OrtRunner>(policy/"exported/policy.onnx");
    const auto scale=cfg["actions"]["JointPositionAction"]["scale"].as<std::vector<float>>();
    const auto offset=cfg["actions"]["JointPositionAction"]["offset"].as<std::vector<float>>();
    if (scale.size()!=29 || offset.size()!=29 || robot->data.joint_ids_map.size()!=29)
        throw std::runtime_error("Expected G1 29-joint policy");
    auto recorded_action=[&](const Frame& f) {
        std::vector<float> action(29);
        for (int i=0;i<29;++i) {
            if (scale[i]==0) throw std::runtime_error("Zero action scale");
            const auto sdk=std::to_string(int(robot->data.joint_ids_map[i]));
            action[i]=(f.at("requested_q_"+sdk)-offset[i])/scale[i];
        }
        return action;
    };
    // Teacher-force the recorded last action. Initial history is repeated
    // from the first frame, so exclude the first history window in analysis.
    env.action_manager->process_action(recorded_action(frames.front()));
    env.observation_manager->reset();
    std::ofstream result(output);
    if (!result) throw std::runtime_error("Cannot create output");
    result << std::setprecision(10) << "t";
    for (int i=0;i<29;++i) result << ",predicted_q_" << i;
    result << '\n';
    std::ofstream observations;
    if (argc==5) {
        observations.open(argv[4]);
        if (!observations) throw std::runtime_error("Cannot create observation output");
        observations << std::setprecision(10) << "t";
        for (int i=0;i<480;++i) observations << ",obs_" << i;
        observations << '\n';
    }
    size_t index=0,count=0;
    for (double t=frames.front().at("t");t<=frames.back().at("t");t+=env.step_dt) {
        while (index+1<frames.size() && frames[index+1].at("t")<=t) ++index;
        robot->frame=&frames[index];
        robot->update();
        env.action_manager->process_action(recorded_action(frames[index]));
        auto obs=env.observation_manager->compute();
        if (obs.size()!=1 || obs.at("obs").size()!=480)
            throw std::runtime_error("Expected one 480-element observation");
        if (observations.is_open()) {
            observations << t;
            for (float value:obs.at("obs")) observations << ',' << value;
            observations << '\n';
        }
        env.action_manager->process_action(env.alg->act(obs));
        const auto action=env.action_manager->processed_actions();
        std::array<float,29> sdk_q{};
        for (int i=0;i<29;++i) sdk_q.at(int(robot->data.joint_ids_map[i]))=action[i];
        result << t;
        for (auto q:sdk_q) result << ',' << q;
        result << '\n';
        ++count;
    }
    std::cout << "Replayed " << count << " inference steps offline; no robot connection.\n";
    return 0;
} catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
}
