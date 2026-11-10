#include "State_SuperMimic.h"
#include "ReferenceOrientationGuard.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>
#include <limits>
#include <stdexcept>

namespace {
constexpr int kJoints = 29;
constexpr int kObs = 99;
constexpr int kGoalPerFrame = 73;
constexpr auto kPeriod = std::chrono::milliseconds(20);

std::filesystem::path local_path(const YAML::Node& cfg, const char* key) {
    if (!cfg[key]) throw std::runtime_error(std::string("missing SuperMimic ") + key);
    std::filesystem::path p = cfg[key].as<std::string>();
    if (p.is_relative()) p = param::proj_dir / p;
    if (!std::filesystem::is_regular_file(p)) throw std::runtime_error("missing asset: " + p.string());
    return p;
}

std::array<float, kJoints> read_vector(const YAML::Node& meta, const char* key) {
    const auto v = meta[key].as<std::vector<float>>();
    if (v.size() != kJoints) throw std::runtime_error(std::string(key) + " needs 29 values");
    std::array<float, kJoints> out{};
    std::copy(v.begin(), v.end(), out.begin());
    for (float x : out) if (!std::isfinite(x)) throw std::runtime_error(std::string(key) + " nonfinite");
    return out;
}

Eigen::Matrix3f yaw_only(const Eigen::Matrix3f& R) {
    const float yaw = std::atan2(R(1, 0), R(0, 0));
    return Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()).toRotationMatrix();
}
} // namespace

State_SuperMimic::State_SuperMimic(int mode, std::string name)
    : FSMState(mode, name) {
    const auto cfg = param::config["FSM"][name];
    const auto meta = YAML::LoadFile(local_path(cfg, "meta_file").string());
    history_frames_ = meta["hist"].as<int>();
    goal_frames_ = meta["goal_frames"].as<int>();
    if (history_frames_ != 10 || goal_frames_ != 10)
        throw std::runtime_error("unexpected SuperMimic history/goal contract");
    default_q_ = read_vector(meta, "default_q");
    scale_ = read_vector(meta, "action_scale");
    kp_ = read_vector(meta, "kp");
    kd_ = read_vector(meta, "kd");
    for (int i=0; i<kJoints; ++i)
        if (scale_[i] <= 0 || kp_[i] <= 0 || kp_[i] > 300 || kd_[i] <= 0 || kd_[i] > 20)
            throw std::runtime_error("invalid SuperMimic gains/scale");

    model_ = std::make_unique<isaaclab::OrtRunner>(local_path(cfg, "policy_file").string());
    const auto motion = local_path(cfg, "motion_file");
    std::ifstream file(motion, std::ios::binary);
    char magic[4];
    uint32_t count=0, goal_dim=0;
    file.read(magic, 4);
    file.read(reinterpret_cast<char*>(&count), sizeof(count));
    file.read(reinterpret_cast<char*>(&goal_dim), sizeof(goal_dim));
    if (!file || std::memcmp(magic, "SMC1", 4) != 0 || count<2 || count>1000000 ||
        goal_dim!=kGoalPerFrame*goal_frames_)
        throw std::runtime_error("invalid SuperMimic motion header");
    frames_.resize(count);
    for (auto& frame : frames_) {
        float r[9];
        file.read(reinterpret_cast<char*>(frame.q.data()), sizeof(float)*kJoints);
        file.read(reinterpret_cast<char*>(r), sizeof(r));
        frame.goal.resize(goal_dim);
        file.read(reinterpret_cast<char*>(frame.goal.data()), sizeof(float)*goal_dim);
        if (!file) throw std::runtime_error("truncated SuperMimic motion");
        for (int row=0; row<3; ++row) for (int col=0; col<3; ++col)
            frame.R(row,col)=r[row*3+col];
        for (float q : frame.q) if (!std::isfinite(q)) throw std::runtime_error("nonfinite motion q");
        for (float x : frame.goal) if (!std::isfinite(x)) throw std::runtime_error("nonfinite motion goal");
        if (!frame.R.allFinite()) throw std::runtime_error("nonfinite motion orientation");
    }
    if (file.peek()!=std::char_traits<char>::eof()) throw std::runtime_error("extra motion bytes");

    standby_flow_ = cfg["standby_flow"].as<bool>(false);
    const double duration = cfg["max_seconds"].as<double>();
    const double duration_limit = cfg["simulation_only"].as<bool>(false) ? 600. : 30.;
    transition_seconds_ = cfg["transition_seconds"].as<float>(1.f);
    if (!std::isfinite(duration) || duration <= 0 || duration > duration_limit ||
        !std::isfinite(transition_seconds_) || transition_seconds_<0 || transition_seconds_>5)
        throw std::runtime_error("initial deployment requires max_seconds in (0,30], transition in [0,5]");
    frame_limit_ = std::min(frames_.size(), size_t(std::ceil(duration/0.02 - 1e-8)));
    history_.resize(size_t(history_frames_)*kObs);
    desired_ = frames_[0].q;
    spdlog::info("SuperMimic loaded {} frames; policy window {:.2f}s", frames_.size(),
                 frame_limit_*0.02f);
    // The policy thread evaluates against the exact reference used for this tick.
    // The FSM consumes the latched request and preserves Passive damping semantics.
    registered_checks.insert(registered_checks.begin(), std::make_pair([this]() {
        return orientation_failed_.load();
    }, FSMStringMap.right.at("Passive")));
    if (cfg["orientation_trace"]) {
        std::filesystem::path path=cfg["orientation_trace"].as<std::string>();
        if (!path.is_absolute() || std::filesystem::exists(path))
            throw std::runtime_error("orientation trace requires a fresh absolute path");
        orientation_trace_.open(path);
        if (!orientation_trace_) throw std::runtime_error("cannot open orientation trace");
        orientation_trace_ << "policy_t,phase,frame,motion_t,actual_tilt,reference_tilt,gravity_distance,gravity_angle,legacy_cosine_error,threshold,trigger\n";
    }


}

State_SuperMimic::Snapshot State_SuperMimic::snapshot() const {
    Snapshot s;
    std::lock_guard<std::mutex> lock(lowstate->mutex_);
    const auto& state = lowstate->msg_;
    for (int i=0;i<kJoints;++i) {
        s.q[i]=state.motor_state()[i].q();
        s.dq[i]=state.motor_state()[i].dq();
    }
    for (int i=0;i<3;++i) s.gyro[i]=state.imu_state().gyroscope()[i];
    const auto& q = state.imu_state().quaternion();
    s.quat=Eigen::Quaternionf(q[0],q[1],q[2],q[3]);
    s.keys=uint16_t(state.wireless_remote()[2]) | (uint16_t(state.wireless_remote()[3])<<8);
    return s;
}

std::array<float, kObs> State_SuperMimic::observation(const Snapshot& s, const Frame& frame) const {
    if (!s.quat.coeffs().allFinite() || std::abs(s.quat.squaredNorm()-1.f)>.02f)
        throw std::runtime_error("invalid live pelvis quaternion");
    const Eigen::Matrix3f R = yaw_alignment_*s.quat.normalized().toRotationMatrix();
    const Eigen::Matrix3f rel = R.transpose()*frame.R;
    std::array<float,kObs> out{};
    size_t j=0;
    for (int i=0;i<kJoints;++i) out[j++]=s.q[i]-default_q_[i];
    for (float x:s.dq) out[j++]=x;
    for (int i=0;i<3;++i) out[j++]=s.gyro[i];
    for (int i=0;i<3;++i) out[j++]=-R(2,i);
    for (float x:prev_action_) out[j++]=x;
    for (int row=0;row<3;++row) for (int col=0;col<2;++col) out[j++]=rel(row,col);
    if (j!=kObs) throw std::runtime_error("observation size mismatch");
    for (float x:out) if (!std::isfinite(x)) throw std::runtime_error("nonfinite observation");
    return out;
}

void State_SuperMimic::enter() {
    const Snapshot s=snapshot();
    float max_error=0, max_speed=0;
    for (int i=0;i<kJoints;++i) {
        max_error=std::max(max_error,std::abs(s.q[i]-frames_[0].q[i]));
        max_speed=std::max(max_speed,std::abs(s.dq[i]));
        const auto& c=lowcmd->msg_.motor_cmd()[i];
        start_command_[i]={c.q(),c.kp(),c.kd(),c.dq(),c.tau()};
    }
    if (max_error>.15f || max_speed>.3f)
        throw std::runtime_error("SuperMimic entry pose/speed differs from selected motion");
    yaw_alignment_=yaw_only(frames_[0].R)*yaw_only(s.quat.normalized().toRotationMatrix()).transpose();
    desired_=frames_[0].q;
    prev_action_.fill(0);
    std::fill(history_.begin(),history_.end(),0.f);
    entered_=std::chrono::steady_clock::now();
    orientation_failed_.store(false);
    running_.store(true);
    thread_=std::thread(&State_SuperMimic::inference_loop,this);
}

void State_SuperMimic::run() {
    std::array<float,kJoints> target;
    {
        std::lock_guard<std::mutex> lock(target_mutex_);
        target=desired_;
    }
    const float elapsed=std::chrono::duration<float>(std::chrono::steady_clock::now()-entered_).count();
    const float u=transition_seconds_>0 ? std::clamp(elapsed/transition_seconds_,0.f,1.f) : 1.f;
    const float alpha=u*u*(3-2*u);
    for (int i=0;i<kJoints;++i) {
        auto& c=lowcmd->msg_.motor_cmd()[i];
        c.q()=(1-alpha)*start_command_[i].q+alpha*target[i];
        c.kp()=(1-alpha)*start_command_[i].kp+alpha*kp_[i];
        c.kd()=(1-alpha)*start_command_[i].kd+alpha*kd_[i];
        c.dq()=(1-alpha)*start_command_[i].dq;
        c.tau()=(1-alpha)*start_command_[i].tau;
    }
}

void State_SuperMimic::trip(const std::string& reason) noexcept {
    running_.store(false);
    if (FSMState::trip_fault) FSMState::trip_fault("SuperMimic: "+reason);
}

std::vector<float> State_SuperMimic::static_goal(const Frame& frame) const {
    // Our 73-D training contract: dp(3), heading-relative R6(6),
    // local linear/angular velocity(6), joint q(29), scaled joint dq(29).
    std::vector<float> goal(kGoalPerFrame*goal_frames_,0.f);
    const Eigen::Matrix3f rel=yaw_only(frame.R).transpose()*frame.R;
    for (int h=0;h<goal_frames_;++h) {
        const int o=h*kGoalPerFrame;
        int j=3;
        for (int row=0;row<3;++row) for (int col=0;col<2;++col) goal[o+j++]=rel(row,col);
        for (int i=0;i<29;++i) goal[o+15+i]=frame.q[i];
    }
    return goal;
}

void State_SuperMimic::inference_loop() noexcept {
    try {
        auto next=std::chrono::steady_clock::now();
        int phase=standby_flow_ ? 0 : 1;  // PRE, MOTION, POST
        size_t frame=0, tick=0;
        double stable=0, released=0, pressed=0;
        bool armed=false, ready_logged=false, stop_combo_prev=false;
        auto phase_start=next;
        const auto pre_goal=static_goal(frames_.front());
        size_t hold_index=std::min(frame_limit_,frames_.size()-1);
        Frame hold_frame=frames_[hold_index];
        auto hold_goal=static_goal(hold_frame);
        if (standby_flow_) spdlog::info("SuperMimic: STANDBY_PRE; policy holds static first frame; R1+X starts motion after readiness");
        while (running_.load()) {
            const auto begin=std::chrono::steady_clock::now();
            const double elapsed=std::chrono::duration<double>(begin-phase_start).count();
            const Snapshot s=snapshot();
            const bool stop_combo=(s.keys & ((1<<0)|(1<<11)))==((1<<0)|(1<<11)); // R1+Y
            const bool stop_pressed=stop_combo && !stop_combo_prev;
            stop_combo_prev=stop_combo;
            if (phase==0) {
                const Eigen::Matrix3f R=yaw_alignment_*s.quat.normalized().toRotationMatrix();
                const float cosine=std::clamp(R.row(2).dot(frames_.front().R.row(2)),-1.f,1.f);
                float speed=0;
                for (float v:s.dq) speed=std::max(speed,std::abs(v));
                const bool healthy=std::acos(cosine)<.2f && speed<1.f && elapsed>transition_seconds_;
                stable=healthy ? stable+.02 : 0;
                const bool ready=elapsed>=3. && stable>=3.;
                if (ready && !ready_logged) spdlog::info("SuperMimic: STANDBY_PRE ready; fresh R1+X may start motion");
                ready_logged=ready;
                const bool combo=(s.keys & ((1<<0)|(1<<10)))==((1<<0)|(1<<10));
                if (!combo) {
                    released+=.02; pressed=0;
                    if (released>=.1) armed=true;
                } else {
                    released=0; pressed+=.02;
                    if (armed && pressed>=.06) {
                        armed=false;
                        if (ready) {
                            phase=1;frame=0;phase_start=begin;
                            spdlog::info("SuperMimic: MOTION from selected first frame; history preserved");
                        } else spdlog::warn("SuperMimic: R1+X refused; standby is not stable yet");
                    }
                }
            }
            if (phase==1 && stop_pressed) {
                const size_t stopped_frame=frame;
                hold_index=stopped_frame;
                hold_frame=frames_[hold_index];
                hold_goal=static_goal(hold_frame);
                phase=2;phase_start=begin;
                spdlog::warn("SuperMimic: EMERGENCY_HOLD; R1+Y stopped motion at frame {} ({:.2f}s); policy holds this frame; resume disabled",stopped_frame,stopped_frame*.02);
                spdlog::warn("[操作提示] 动作已停止，持续保持当前帧，无法继续播放。L2+B 切回阻尼；Ctrl+C 退出并禁用电机。");
            }
            const Frame& ref=phase==0 ? frames_.front() : phase==2 ? hold_frame : frames_[frame];
            // Evaluate before issuing another policy target. No added debounce:
            // the legacy sim2sim termination was evaluated every 20 ms as well.
            const auto error=supermimic::reference_orientation_error(
                s.quat.normalized().toRotationMatrix(),ref.R);
            const bool bad=supermimic::reference_orientation_failed(error);
            const size_t ref_index=phase==0 ? 0 : phase==2 ? hold_index : frame;
            if (orientation_trace_) {
                orientation_trace_ << std::chrono::duration<double>(begin-entered_).count() << ','
                    << phase << ',' << ref_index << ',' << ref_index*.02 << ','
                    << error.actual_tilt << ',' << error.reference_tilt << ',' << error.distance << ','
                    << error.angle << ',' << error.legacy_cosine_error << ','
                    << supermimic::kReferenceGravityDistanceLimit << ',' << bad << '\n';
                if (!orientation_trace_) throw std::runtime_error("orientation trace write failed");
            }
            if (bad) {
                spdlog::warn("Reference orientation protection: gravity_distance={} > {}; angle={} rad phase={} frame={}; switching to Passive",
                    error.distance,supermimic::kReferenceGravityDistanceLimit,error.angle,phase,ref_index);
                if (orientation_trace_) orientation_trace_.flush();
                orientation_failed_.store(true);
                return;
            }
            const auto obs=observation(s,ref);
            if (tick++==0) {
                for (int h=0;h<history_frames_;++h)
                    std::copy(obs.begin(),obs.end(),history_.begin()+h*kObs);
            } else {
                std::move(history_.begin()+kObs,history_.end(),history_.begin());
                std::copy(obs.begin(),obs.end(),history_.end()-kObs);
            }
            const auto& goal=phase==0 ? pre_goal : phase==2 ? hold_goal : ref.goal;
            const auto action=model_->act({{"obs_hist",history_},{"goal",goal}});
            if (action.size()!=kJoints) throw std::runtime_error("policy output not 29 joints");
            std::array<float,kJoints> target{};
            for (int i=0;i<kJoints;++i) {
                if (!std::isfinite(action[i]) || std::abs(action[i])>50)
                    throw std::runtime_error("invalid policy action");
                prev_action_[i]=action[i];
                target[i]=default_q_[i]+scale_[i]*action[i];
            }
            { std::lock_guard<std::mutex> lock(target_mutex_); desired_=target; }
            if (std::chrono::steady_clock::now()-begin>std::chrono::milliseconds(35))
                throw std::runtime_error("policy inference over 35 ms");
            next+=kPeriod;
            if (std::chrono::steady_clock::now()-next>std::chrono::milliseconds(15))
                throw std::runtime_error("policy schedule lag over 15 ms");
            std::this_thread::sleep_until(next);
            if (phase==1 && ++frame>=frame_limit_) {
                if (!standby_flow_) {
                    spdlog::info("SuperMimic: configured motion window finished; requesting controller shutdown");
                    if (FSMState::request_stop) FSMState::request_stop();
                    break;
                }
                phase=2;phase_start=std::chrono::steady_clock::now();
                spdlog::info("SuperMimic: STANDBY_POST; motion finished, policy holds final frame; history preserved");
            }
        }
    } catch (const std::exception& e) { trip(e.what()); }
      catch (...) { trip("unknown inference fault"); }
}

void State_SuperMimic::exit() {
    running_.store(false);
    if (thread_.joinable()) thread_.join();
    if (orientation_trace_) orientation_trace_.flush();
}
