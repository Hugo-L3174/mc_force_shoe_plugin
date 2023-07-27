#include "ForceShoePlugin.h"

#include <mc_control/GlobalPluginMacros.h>
#include <bits/stdc++.h>
#include <chrono>


namespace mc_plugin
{

ForceShoePlugin::~ForceShoePlugin() = default;

void ForceShoePlugin::init(mc_control::MCGlobalController & controller, const mc_rtc::Configuration & config)
{
  auto & ctl = controller.controller();
  mc_rtc::log::info("ForceShoePlugin::init called with configuration:\n{}", config.dump(true, true));
  // loading config port and baudrate
  auto comPort = config("comPort", std::string{"/dev/ttyUSB0"});
  auto baudrate = config("baudrate", 921600);
  cmt3_.reset(new xsens::Cmt3);

  // Putting mode in datastore (true is live, false is replay), true by default
  auto & liveMode = ctl.datastore().make<bool>("ForceShoesMode", liveMode_);

  if (ctl.config()("ForceShoes").has("liveMode"))
  {
    liveMode_ = ctl.config()("ForceShoes")("liveMode");
    liveMode = liveMode_;
  }

  if (liveMode_)
  {
    doHardwareConnect(baudrate, comPort);
    doMtSettings();
    UnloadedFS();

    ctl.datastore().make<sva::ForceVecd>("ForceShoePlugin::LFForce", sva::ForceVecd::Zero());
    ctl.datastore().make<sva::ForceVecd>("ForceShoePlugin::LBForce", sva::ForceVecd::Zero());
    ctl.datastore().make<sva::ForceVecd>("ForceShoePlugin::RFForce", sva::ForceVecd::Zero());
    ctl.datastore().make<sva::ForceVecd>("ForceShoePlugin::RBForce", sva::ForceVecd::Zero());
    
    ctl.datastore().make_call("ForceShoePlugin::GetLFForce", [&ctl, this]() { return ctl.datastore().get<sva::ForceVecd>("ForceShoePlugin::LFForce"); });  
    ctl.datastore().make_call("ForceShoePlugin::GetLBForce", [&ctl, this]() { return ctl.datastore().get<sva::ForceVecd>("ForceShoePlugin::LBForce"); });  
    ctl.datastore().make_call("ForceShoePlugin::GetRFForce", [&ctl, this]() { return ctl.datastore().get<sva::ForceVecd>("ForceShoePlugin::RFForce"); });  
    ctl.datastore().make_call("ForceShoePlugin::GetRBForce", [&ctl, this]() { return ctl.datastore().get<sva::ForceVecd>("ForceShoePlugin::RBForce"); });  

    packet_.reset(new Packet((unsigned short)mtCount, cmt3_->isXm()));
    th_ = std::thread([this]() { dataThread(*cmt3_, *packet_); });
  }
  else
  {
    ctl.datastore().make_call("ForceShoePlugin::GetLFForce", [&ctl, this]() { return ctl.datastore().get<sva::ForceVecd>("ReplayPlugin::LFForce"); });  
    ctl.datastore().make_call("ForceShoePlugin::GetLBForce", [&ctl, this]() { return ctl.datastore().get<sva::ForceVecd>("ReplayPlugin::LBForce"); });  
    ctl.datastore().make_call("ForceShoePlugin::GetRFForce", [&ctl, this]() { return ctl.datastore().get<sva::ForceVecd>("ReplayPlugin::RFForce"); });  
    ctl.datastore().make_call("ForceShoePlugin::GetRBForce", [&ctl, this]() { return ctl.datastore().get<sva::ForceVecd>("ReplayPlugin::RBForce"); });  
  }
  
}

void ForceShoePlugin::reset(mc_control::MCGlobalController & controller)
{
  mc_rtc::log::info("ForceShoePlugin::reset called");
}

void ForceShoePlugin::before(mc_control::MCGlobalController & controller) 
{
  if (liveMode_)
  {  
    auto & ctl = controller.controller();
    auto & LF = ctl.datastore().get<sva::ForceVecd>("ForceShoePlugin::LFForce");
    auto & LB = ctl.datastore().get<sva::ForceVecd>("ForceShoePlugin::LBForce");
    auto & RF = ctl.datastore().get<sva::ForceVecd>("ForceShoePlugin::RFForce");
    auto & RB = ctl.datastore().get<sva::ForceVecd>("ForceShoePlugin::RBForce");


    std::lock_guard<std::mutex> lock(mutex_);
    computeAmpCalMat();		
    computeUDiff();
    computeForceVec();
      
    LB = sva::ForceVecd(Eigen::Vector3d{LBforcevec[3], LBforcevec[4], LBforcevec[5]}, Eigen::Vector3d{LBforcevec[0], LBforcevec[1], LBforcevec[2]});
    LF = sva::ForceVecd(Eigen::Vector3d{LFforcevec[3], LFforcevec[4], LFforcevec[5]}, Eigen::Vector3d{LFforcevec[0], LFforcevec[1], LFforcevec[2]});
    RB = sva::ForceVecd(Eigen::Vector3d{RBforcevec[3], RBforcevec[4], RBforcevec[5]}, Eigen::Vector3d{RBforcevec[0], RBforcevec[1], RBforcevec[2]});
    RF = sva::ForceVecd(Eigen::Vector3d{RFforcevec[3], RFforcevec[4], RFforcevec[5]}, Eigen::Vector3d{RFforcevec[0], RFforcevec[1], RFforcevec[2]});
  
  }

}

void ForceShoePlugin::after(mc_control::MCGlobalController & controller)  
{
  mc_rtc::log::info("ForceShoePlugin::after");
}

mc_control::GlobalPlugin::GlobalPluginConfiguration ForceShoePlugin::configuration()
{
  mc_control::GlobalPlugin::GlobalPluginConfiguration out;
  out.should_run_before = true;
  out.should_run_after = false;
  out.should_always_run = true;
  return out;
}

} // namespace mc_plugin

EXPORT_MC_RTC_PLUGIN("ForceShoePlugin", mc_plugin::ForceShoePlugin)
