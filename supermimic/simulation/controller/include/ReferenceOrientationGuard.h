#pragma once
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <stdexcept>
namespace supermimic {
// Training uses abs(ref.Rzz - actual.Rzz) > 0.8. This directional
// extension agrees for upright reference and also detects opposite leans.
constexpr float kReferenceGravityDistanceLimit = 0.8f;
struct OrientationError { float distance, angle, legacy_cosine_error, actual_tilt, reference_tilt; };
inline OrientationError reference_orientation_error(const Eigen::Matrix3f& actual,
                                                   const Eigen::Matrix3f& reference) {
    Eigen::Vector3f a=actual.row(2).transpose(), r=reference.row(2).transpose();
    if (!a.allFinite() || !r.allFinite() || a.norm()<0.98f || a.norm()>1.02f ||
        r.norm()<0.98f || r.norm()>1.02f) throw std::runtime_error("invalid gravity direction");
    a.normalize(); r.normalize();
    const float dot=std::clamp(a.dot(r),-1.f,1.f);
    return {1.f-dot,std::acos(dot),std::abs(a.z()-r.z()),
            std::acos(std::clamp(a.z(),-1.f,1.f)), std::acos(std::clamp(r.z(),-1.f,1.f))};
}
inline bool reference_orientation_failed(const OrientationError& e) {
    return e.distance>kReferenceGravityDistanceLimit;
}
} // namespace supermimic
