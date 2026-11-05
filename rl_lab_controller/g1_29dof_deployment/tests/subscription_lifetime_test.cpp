#include <unitree/dds_wrapper/robots/g1/g1_sub.h>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <atomic>
#include <chrono>
#include <iostream>
#include <net/if.h>
#include <cstring>
#include <thread>

int main() {
    // A test publisher must never join the robot's physical DDS network.
    auto interfaces=if_nameindex();
    if (!interfaces) return 2;
    bool isolated=true;
    for (auto p=interfaces;p->if_index;++p) isolated &= std::strcmp(p->if_name,"lo")==0;
    if_freenameindex(interfaces);
    if (!isolated) return 2;
    unitree::robot::ChannelFactory::Instance()->Init(0,"lo");
    unitree::robot::ChannelPublisher<unitree_hg::msg::dds_::LowState_> pub("rt/repair_lifetime");
    pub.InitChannel();
    std::atomic<bool> running{true};
    std::thread writer([&] {
        unitree_hg::msg::dds_::LowState_ msg{};
        while (running) {
            ++msg.tick(); pub.Write(msg);
            std::this_thread::sleep_for(std::chrono::microseconds(500));
        }
    });
    int connected=0;
    for (int i=0;i<100;++i) {
        auto sub=std::make_shared<unitree::robot::g1::subscription::LowState>("rt/repair_lifetime");
        for (int j=0;j<200 && sub->isTimeout();++j)
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        connected+=!sub->isTimeout();
        if (i%2==0) { sub->Close(); sub->Close(); }
        sub.reset(); // alternate explicit close and destructor under live traffic
    }
    running=false; writer.join(); pub.CloseChannel();
    std::cout << "subscription teardown checks: " << connected << "/100 received live state\n";
    return connected==100 ? 0 : 1;
}
