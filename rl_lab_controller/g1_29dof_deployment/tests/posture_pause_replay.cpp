#include "PostureProgress.h"
#include "PostureTracking.h"
#include <cassert>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <vector>

std::vector<std::string> split(const std::string& line) {
    std::vector<std::string> out; std::stringstream s(line); std::string x;
    while (std::getline(s,x,',')) out.push_back(x);
    return out;
}

int main(int argc,char** argv) {
    if (argc!=2) return 2;
    std::ifstream input(argv[1]); std::string line;
    if (!std::getline(input,line)) return 2;
    const auto header=split(line); std::map<std::string,size_t> columns;
    for (size_t i=0;i<header.size();++i) columns[header[i]]=i;
    PostureTracking tracking;
    PostureProgress progress(1.,100.,.12,.08,3.,5.,.3,false);
    for (int i=0;i<132;++i) progress.advance(.01,0,false,true);
    assert(progress.aligning()); const double start=progress.time();
    std::array<double,2> shoulder_target{};
    bool old_global_paused=true; int frames=0,raw_saturated_frames=0;
    while (std::getline(input,line)) {
        const auto row=split(line);
        auto value=[&](const std::string& key){ return std::stod(row.at(columns.at(key))); };
        if (value("output_mode_16")==0 || value("output_mode_23")==0) continue;
        std::array<float,29> errors{}; bool blocking=false,raw_saturated=false;
        float largest=0;
        for (int i=0;i<29;++i) {
            const auto suffix="_"+std::to_string(i);
            const double error=value("requested_q"+suffix)-value("q"+suffix);
            const double tolerance=(i==4 || i==10) ? .2 : .15;
            errors[i]=std::abs(error)*(i<15 ? .12 : .15)/tolerance;
            largest=std::max(largest,errors[i]);
            if (i<15) continue;
            const double effort=value("output_kp"+suffix)*error-
                value("output_kd"+suffix)*value("dq"+suffix)+value("output_tau"+suffix);
            const double cap=(i==16 || i==23) ? 12 :
                ((i==20 || i==21 || i==27 || i==28) ? 3 : 8);
            const bool raw=std::abs(error)>.06 && std::abs(effort)>=.9*cap;
            // The recorded waypoint holds shoulder roll while the other
            // joints return; verify that only these held axes were clipping.
            if (i==16 || i==23) {
                const int k=i==16 ? 0 : 1;
                if (frames==0) shoulder_target[k]=value("requested_q"+suffix);
                assert(std::abs(shoulder_target[k]-value("requested_q"+suffix))<1e-6);
            } else assert(!raw);
            raw_saturated |= raw;
            blocking |= posture_saturation_blocks_progress(error,effort,.9*cap,0.);
        }
        if (largest<=.08f) old_global_paused=false;
        const auto block=tracking.update(errors,.12f,.08f);
        assert(block.joint==-1 && !blocking);
        progress.advance(.001,block.error,blocking,true);
        assert(!progress.paused() && progress.failure().empty());
        ++frames; raw_saturated_frames+=raw_saturated;
    }
    assert(frames>100 && raw_saturated_frames>0 && old_global_paused);
    assert(progress.time()>start+.1);
    std::cout << "recorded pause replay passed: " << frames << " live frames, "
              << raw_saturated_frames << " held-shoulder saturation frames; unrelated ankle no longer latches pause\n";
}
