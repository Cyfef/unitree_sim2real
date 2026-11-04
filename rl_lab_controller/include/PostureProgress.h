#pragma once

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

// Feedback-paced trajectory clock. It never changes position/torque limits.
// Time used for dwell/timeout checks is wall time, even while the path is held.
class PostureProgress {
public:
    PostureProgress(double nominal, double end, double pause, double resume,
                    double timeout, double settle_timeout, double hold, bool pace_recovery=true)
        : nominal_(nominal), end_(end), pause_(pause), resume_(resume),
          timeout_(timeout), settle_timeout_(settle_timeout), hold_(hold), pace_recovery_(pace_recovery) {
        for (double v : {nominal,end,pause,resume,timeout,settle_timeout,hold})
            if (!std::isfinite(v)) throw std::runtime_error("Nonfinite posture progress setting");
        if (nominal<=0 || end<=nominal || resume<=0 || pause<=resume || pause>.15 ||
            timeout<=0 || timeout>5 || settle_timeout<=0 || settle_timeout>10 || hold<.3 || hold>2)
            throw std::runtime_error("Invalid posture progress settings");
    }
    void reset() { time_=elapsed_=paused_for_=settle_for_=stable_for_=saturated_for_=0; paused_=aligned_=false; failure_.clear(); }
    void advance(double dt, double error, bool saturated, bool nominal_stable) {
        if (!failure_.empty()) return;
        if (time_>=end_) return; // readiness/holding is no longer path execution
        // A delayed control cycle must not cause a jump along the path.
        dt=std::max(dt,0.);
        elapsed_+=dt;
        if (elapsed_>end_+30) { failure_="posture transition exceeded wall-time allowance"; return; }
        if (!pace_recovery_ && time_<nominal_) {
            // Original RL Lab recovery is time-driven under the independent
            // 0.25 rad/speed/torque guards. A planted ankle must not freeze
            // the remaining hip/knee recovery before they can redistribute load.
            time_=std::min(time_+std::min(dt,.02),nominal_);
            return;
        }
        if (time_>=nominal_ && !aligned_) {
            settle_for_+=dt;
            stable_for_=nominal_stable ? stable_for_+dt : 0.;
            if (stable_for_>=hold_) { aligned_=true; paused_=false; paused_for_=0; }
            else {
                if (settle_for_>=settle_timeout_) failure_="nominal stand did not settle; first-frame alignment refused";
                return;
            }
        }
        // The hard torque clamp still runs each cycle. Do not latch a path
        // stall from a single quantized velocity/PD sample at the budget edge.
        saturated_for_=saturated ? saturated_for_+dt : 0.;
        if (saturated_for_>=.1 || error>=pause_) paused_=true;
        else if (paused_ && error<=resume_) { paused_=false; paused_for_=0; }
        if (paused_) {
            paused_for_+=dt;
            if (paused_for_>=timeout_) failure_="posture tracking stalled or torque budget saturated";
            return;
        }
        time_=std::min(time_+std::min(dt,.02), aligned_ ? end_ : nominal_);
    }
    double time() const { return time_; }
    bool aligning() const { return aligned_; }
    bool paused() const { return paused_; }
    const std::string& failure() const { return failure_; }
private:
    double nominal_,end_,pause_,resume_,timeout_,settle_timeout_,hold_;
    double time_=0,elapsed_=0,paused_for_=0,settle_for_=0,stable_for_=0,saturated_for_=0;
    bool paused_=false,aligned_=false;
    bool pace_recovery_=true;
    std::string failure_;
};
