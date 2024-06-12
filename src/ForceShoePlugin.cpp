#include "ForceShoePlugin.h"

#include <mc_control/GlobalPluginMacros.h>

#include <boost/filesystem.hpp>
namespace bfs = boost::filesystem;

#include <bits/stdc++.h>

namespace mc_plugin
{

ForceShoePlugin::~ForceShoePlugin()
{
  if(th_running_ && th_.joinable())
  {
    th_running_ = false;
    th_.join();
  }
}

void ForceShoePlugin::init(mc_control::MCGlobalController & controller, const mc_rtc::Configuration & config)
{
  auto & ctl = controller.controller();
  mc_rtc::log::info("ForceShoePlugin::init called with configuration:\n{}", config.dump(true, true));
  // loading config port and baudrate
  auto comPort = config("comPort", std::string{"/dev/ttyUSB0"});
  auto baudrate = config("baudrate", 921600);
  cmt3_.reset(new xsens::Cmt3);

  config("calibFile", calibFile_);
  if(bfs::exists(calibFile_))
  {
    mc_rtc::log::info("[ForceShoes] Load calibration from {}", calibFile_);
    mc_rtc::Configuration calib(calibFile_);
    LFUnload = calib("LFUnload");
    LBUnload = calib("LBUnload");
    RFUnload = calib("RFUnload");
    RBUnload = calib("RBUnload");
    mode_ = Mode::Acquire;
  }

  // Putting mode in datastore (true is live, false is replay), true by default
  liveMode_ =
      ctl.config().find<bool>("ForceShoes", "liveMode").value_or(config.find<bool>("liveMode").value_or(liveMode_));
  ctl.datastore().make<bool>("ForceShoesMode", liveMode_);

  if(liveMode_)
  {
    doHardwareConnect(baudrate, comPort);
    doMtSettings();
    packet_.reset(new Packet((unsigned short)mtCount, cmt3_->isXm()));
    th_ = std::thread([this]() { dataThread(); });
  }
  reset(controller);
}

void ForceShoePlugin::reset(mc_control::MCGlobalController & controller)
{
  auto & ctl = controller.controller();
  if(liveMode_)
  {
    ctl.gui()->addElement(
        {"Plugin", "ForceShoes"},
        mc_rtc::gui::Label("Status", [this]() { return mode_ == Mode::Calibrate ? "Calibrating" : "Live"; }),
        mc_rtc::gui::Button("Calibrate",
                            [this]()
                            {
                              std::lock_guard<std::mutex> lck(mutex_);
                              if(mode_ == Mode::Calibrate)
                              {
                                mc_rtc::log::error("[ForceShoes] Already calibrating");
                              }
                              else
                              {
                                mode_ = Mode::Calibrate;
                              }
                            }));
    ctl.datastore().make<sva::ForceVecd>("ForceShoePlugin::LFForce", sva::ForceVecd::Zero());
    ctl.datastore().make<sva::ForceVecd>("ForceShoePlugin::LBForce", sva::ForceVecd::Zero());
    ctl.datastore().make<sva::ForceVecd>("ForceShoePlugin::RFForce", sva::ForceVecd::Zero());
    ctl.datastore().make<sva::ForceVecd>("ForceShoePlugin::RBForce", sva::ForceVecd::Zero());

    ctl.datastore().make<sva::ForceVecd>("ForceShoePlugin::LFFiltered", sva::ForceVecd::Zero());
    ctl.datastore().make<sva::ForceVecd>("ForceShoePlugin::LBFiltered", sva::ForceVecd::Zero());
    ctl.datastore().make<sva::ForceVecd>("ForceShoePlugin::RFFiltered", sva::ForceVecd::Zero());
    ctl.datastore().make<sva::ForceVecd>("ForceShoePlugin::RBFiltered", sva::ForceVecd::Zero());

    ctl.datastore().make_call("ForceShoePlugin::GetLFForce", [&ctl, this]()
                              { return ctl.datastore().get<sva::ForceVecd>("ForceShoePlugin::LFForce"); });
    ctl.datastore().make_call("ForceShoePlugin::GetLBForce", [&ctl, this]()
                              { return ctl.datastore().get<sva::ForceVecd>("ForceShoePlugin::LBForce"); });
    ctl.datastore().make_call("ForceShoePlugin::GetRFForce", [&ctl, this]()
                              { return ctl.datastore().get<sva::ForceVecd>("ForceShoePlugin::RFForce"); });
    ctl.datastore().make_call("ForceShoePlugin::GetRBForce", [&ctl, this]()
                              { return ctl.datastore().get<sva::ForceVecd>("ForceShoePlugin::RBForce"); });

    ctl.datastore().make_call("ForceShoePlugin::GetLFFiltered", [&ctl, this]()
                              { return ctl.datastore().get<sva::ForceVecd>("ForceShoePlugin::LFFiltered"); });
    ctl.datastore().make_call("ForceShoePlugin::GetLBFiltered", [&ctl, this]()
                              { return ctl.datastore().get<sva::ForceVecd>("ForceShoePlugin::LBFiltered"); });
    ctl.datastore().make_call("ForceShoePlugin::GetRFFiltered", [&ctl, this]()
                              { return ctl.datastore().get<sva::ForceVecd>("ForceShoePlugin::RFFiltered"); });
    ctl.datastore().make_call("ForceShoePlugin::GetRBFiltered", [&ctl, this]()
                              { return ctl.datastore().get<sva::ForceVecd>("ForceShoePlugin::RBFiltered"); });
  }
  else
  {
    ctl.datastore().make_call("ForceShoePlugin::GetLFForce",
                              [&ctl, this]() { return ctl.datastore().get<sva::ForceVecd>("ReplayPlugin::LFForce"); });
    ctl.datastore().make_call("ForceShoePlugin::GetLBForce",
                              [&ctl, this]() { return ctl.datastore().get<sva::ForceVecd>("ReplayPlugin::LBForce"); });
    ctl.datastore().make_call("ForceShoePlugin::GetRFForce",
                              [&ctl, this]() { return ctl.datastore().get<sva::ForceVecd>("ReplayPlugin::RFForce"); });
    ctl.datastore().make_call("ForceShoePlugin::GetRBForce",
                              [&ctl, this]() { return ctl.datastore().get<sva::ForceVecd>("ReplayPlugin::RBForce"); });

    ctl.datastore().make_call("ForceShoePlugin::GetLFFiltered",
                              [&ctl, this]() { return ctl.datastore().get<sva::ForceVecd>("ReplayPlugin::LFFiltered"); });
    ctl.datastore().make_call("ForceShoePlugin::GetLBFiltered",
                              [&ctl, this]() { return ctl.datastore().get<sva::ForceVecd>("ReplayPlugin::LBFiltered"); });
    ctl.datastore().make_call("ForceShoePlugin::GetRFFiltered",
                              [&ctl, this]() { return ctl.datastore().get<sva::ForceVecd>("ReplayPlugin::RFFiltered"); });
    ctl.datastore().make_call("ForceShoePlugin::GetRBFiltered",
                              [&ctl, this]() { return ctl.datastore().get<sva::ForceVecd>("ReplayPlugin::RBFiltered"); });
  }

  mc_rtc::log::info("ForceShoePlugin::reset called");
}

void ForceShoePlugin::before(mc_control::MCGlobalController & controller)
{
  if(liveMode_)
  {
    auto & ctl = controller.controller();
    auto & LF = ctl.datastore().get<sva::ForceVecd>("ForceShoePlugin::LFForce");
    auto & LB = ctl.datastore().get<sva::ForceVecd>("ForceShoePlugin::LBForce");
    auto & RF = ctl.datastore().get<sva::ForceVecd>("ForceShoePlugin::RFForce");
    auto & RB = ctl.datastore().get<sva::ForceVecd>("ForceShoePlugin::RBForce");

    auto & LFf = ctl.datastore().get<sva::ForceVecd>("ForceShoePlugin::LFFiltered");
    auto & LBf= ctl.datastore().get<sva::ForceVecd>("ForceShoePlugin::LBFiltered");
    auto & RFf = ctl.datastore().get<sva::ForceVecd>("ForceShoePlugin::RFFiltered");
    auto & RBf = ctl.datastore().get<sva::ForceVecd>("ForceShoePlugin::RBFiltered");

    std::lock_guard<std::mutex> lock(mutex_);
    computeAmpCalMat();
    computeUDiff();
    computeForceVec();

    LB = sva::ForceVecd(Eigen::Vector3d{LBforcevec[3], LBforcevec[4], LBforcevec[5]},
                        Eigen::Vector3d{LBforcevec[0]*coeffLBx, LBforcevec[1]*coeffLBy, LBforcevec[2]*coeffLBz});
    LF = sva::ForceVecd(Eigen::Vector3d{LFforcevec[3], LFforcevec[4], LFforcevec[5]},
                        Eigen::Vector3d{LFforcevec[0]*coeffLFx, LFforcevec[1]*coeffLFy, LFforcevec[2]*coeffLfz});
    RB = sva::ForceVecd(Eigen::Vector3d{RBforcevec[3], RBforcevec[4], RBforcevec[5]},
                        Eigen::Vector3d{RBforcevec[0], RBforcevec[1], RBforcevec[2]});
    RF = sva::ForceVecd(Eigen::Vector3d{RFforcevec[3], RFforcevec[4], RFforcevec[5]},
                        Eigen::Vector3d{RFforcevec[0], RFforcevec[1], RFforcevec[2]});



    // LBf = sva::ForceVecd(Eigen::Vector3d{LBFilteredvec[3], LBFilteredvec[4], LBFilteredvec[5]},
    //                     Eigen::Vector3d{LBFilteredvec[0]*coeffLBx, LBFilteredvec[1]*coeffLBy, LBFilteredvec[2]*coeffLBz});
    // LFf = sva::ForceVecd(Eigen::Vector3d{LFFilteredvec[3], LFFilteredvec[4], LFFilteredvec[5]},
    //                     Eigen::Vector3d{LFFilteredvec[0]*coeffLFx, LFFilteredvec[1]*coeffLFy, LFFilteredvec[2]*coeffLfz});
    // RBf = sva::ForceVecd(Eigen::Vector3d{RBFilteredvec[3], RBFilteredvec[4], RBFilteredvec[5]},
    //                     Eigen::Vector3d{RBforcevec[0], RBforcevec[1], RBforcevec[2]});
    // RFf = sva::ForceVecd(Eigen::Vector3d{RFFilteredvec[3], RFFilteredvec[4], RFFilteredvec[5]},
    //                      Eigen::Vector3d{RFFilteredvec[0], RFFilteredvec[1], RFFilteredvec[2]});
  }
}

void ForceShoePlugin::after(mc_control::MCGlobalController &) {}

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
