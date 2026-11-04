#pragma once

#include <array>
#include <cmath>

// Hysteresis belongs to each joint that actually crossed its pause threshold.
// A different joint inside its allowed band must not prevent resumption.
class PostureTracking {
public:
    struct Block { int joint=-1; float error=0; };
    void reset() { blocked_.fill(false); }
    Block update(const std::array<float,29>& errors, float pause, float resume) {
        Block result;
        for (int i=0;i<29;++i) {
            if (errors[i]>=pause) blocked_[i]=true;
            else if (errors[i]<=resume) blocked_[i]=false;
            if (blocked_[i] && errors[i]>result.error) result={i,errors[i]};
        }
        return result;
    }
private:
    std::array<bool,29> blocked_{};
};

inline bool posture_saturation_blocks_progress(float error, float effort,
                                               float budget, float next_delta) {
    // Keep the hard command clamp in G1Safety. Pause path time only when
    // advancing this joint would demand still more torque in the clipped
    // direction. Holding its target or moving to unload it can resolve the
    // saturation; tracking-error guards remain active in those cases too.
    return std::abs(error)>.06f && std::abs(effort)>=budget && effort*next_delta>1e-6f;
}
