#include "PostureProgress.h"
#include "PostureTracking.h"
#include <cassert>
#include <iostream>

int main() {
    PostureProgress p(1.,2.,.12,.08,3.,5.,.3);
    for (int i=0;i<100;++i) p.advance(.01,0,false,false);
    assert(p.time()==1. && !p.aligning());
    // Crossing nominal time never bypasses measured stability.
    for (int i=0;i<40;++i) p.advance(.01,0,false,false);
    assert(p.time()==1. && !p.aligning());
    for (int i=0;i<31;++i) p.advance(.01,0,false,true);
    assert(p.aligning() && p.time()>1.);
    const double held=p.time();
    p.advance(.01,.13,false,true);
    assert(p.paused() && p.time()==held);
    p.advance(.01,.10,false,true);
    assert(p.paused() && p.time()==held); // hysteresis
    p.advance(.01,.07,false,true);
    assert(!p.paused() && p.time()>held);
    const double before_saturation=p.time();
    for (int i=0;i<321;++i) p.advance(.01,.07,true,true);
    assert(!p.failure().empty() && p.time()>before_saturation && p.time()<=before_saturation+.11);
    p.reset();
    p.advance(1.,0,false,false);
    assert(p.time()<=.02); // scheduling delay cannot jump the target
    p.reset();
    for (int i=0;i<100;++i) p.advance(.01,0,false,false);
    for (int i=0;i<501;++i) p.advance(.01,0,false,false);
    assert(p.failure().find("nominal")!=std::string::npos);
    p.reset();
    for (int i=0;i<300;++i) p.advance(.01,0,false,true);
    assert(p.time()==2.);
    for (int i=0;i<6000;++i) p.advance(.01,0,false,true);
    assert(p.failure().empty()); // manual policy button may be pressed later
    PostureProgress recovery(1.,2.,.12,.08,3.,5.,.3,false);
    // The real 0.16 rad planted-ankle deviation must not freeze hip/knee
    // recovery. The independent G1Safety guards still run every cycle.
    for (int i=0;i<100;++i) recovery.advance(.01,.12,true,false);
    assert(recovery.time()==1. && !recovery.paused() && !recovery.aligning());
    for (int i=0;i<40;++i) recovery.advance(.01,0,false,false);
    assert(recovery.time()==1. && !recovery.aligning());
    for (int i=0;i<31;++i) recovery.advance(.01,0,false,true);
    assert(recovery.aligning());
    const double before_arm_stall=recovery.time();
    for (int i=0;i<301;++i) recovery.advance(.01,.13,true,true);
    assert(recovery.time()==before_arm_stall && !recovery.failure().empty());
    PostureTracking tracking;
    std::array<float,29> errors{};
    errors[10]=.0897f; // recorded planted ankle inside its original tolerance
    assert(tracking.update(errors,.12f,.08f).joint==-1);
    errors[23]=.13f;
    assert(tracking.update(errors,.12f,.08f).joint==23);
    errors[23]=.09f;
    assert(tracking.update(errors,.12f,.08f).joint==23);
    errors[23]=.07f;
    assert(tracking.update(errors,.12f,.08f).joint==-1);
    assert(!posture_saturation_blocks_progress(-.079f,-11.f,10.8f,0.f));
    assert(!posture_saturation_blocks_progress(-.079f,-11.f,10.8f,.001f));
    assert(posture_saturation_blocks_progress(-.079f,-11.f,10.8f,-.001f));
    p.reset();
    for (int i=0;i<150;++i) p.advance(.01,0,false,true);
    p.advance(.005,0,true,true);
    assert(!p.paused());
    for (int i=0;i<12;++i) p.advance(.01,0,true,true);
    assert(p.paused());
    p.advance(.01,tracking.update(errors,.12f,.08f).error,false,true);
    assert(!p.paused()); // unrelated ankle cannot latch a torque pause
    std::cout << "posture progress checks passed\n";
}
