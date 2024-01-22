#include "ForceShoePlugin.h"

#include <mc_control/GlobalPluginMacros.h>
#include <filesystem>

namespace mc_force_shoe_plugin
{

void exit_on_error(XsensResultValue res, const std::string & comment)
{
  if(res != XRV_OK)
  {
    mc_rtc::log::error_and_throw("Error {} occurred in {}: {}\n", res, comment, xsensResultText(res));
  }
}

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
  if(std::filesystem::exists(calibFile_))
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

    ctl.datastore().make_call("ForceShoePlugin::GetLFForce", [&ctl, this]()
                              { return ctl.datastore().get<sva::ForceVecd>("ForceShoePlugin::LFForce"); });
    ctl.datastore().make_call("ForceShoePlugin::GetLBForce", [&ctl, this]()
                              { return ctl.datastore().get<sva::ForceVecd>("ForceShoePlugin::LBForce"); });
    ctl.datastore().make_call("ForceShoePlugin::GetRFForce", [&ctl, this]()
                              { return ctl.datastore().get<sva::ForceVecd>("ForceShoePlugin::RFForce"); });
    ctl.datastore().make_call("ForceShoePlugin::GetRBForce", [&ctl, this]()
                              { return ctl.datastore().get<sva::ForceVecd>("ForceShoePlugin::RBForce"); });
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

    std::lock_guard<std::mutex> lock(mutex_);
    computeAmpCalMat();
    computeUDiff();
    computeForceVec();

    LB = sva::ForceVecd(Eigen::Vector3d{LBforcevec[3], LBforcevec[4], LBforcevec[5]},
                        Eigen::Vector3d{LBforcevec[0], LBforcevec[1], LBforcevec[2]});
    LF = sva::ForceVecd(Eigen::Vector3d{LFforcevec[3], LFforcevec[4], LFforcevec[5]},
                        Eigen::Vector3d{LFforcevec[0], LFforcevec[1], LFforcevec[2]});
    RB = sva::ForceVecd(Eigen::Vector3d{RBforcevec[3], RBforcevec[4], RBforcevec[5]},
                        Eigen::Vector3d{RBforcevec[0], RBforcevec[1], RBforcevec[2]});
    RF = sva::ForceVecd(Eigen::Vector3d{RFforcevec[3], RFforcevec[4], RFforcevec[5]},
                        Eigen::Vector3d{RFforcevec[0], RFforcevec[1], RFforcevec[2]});
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

void ForceShoePlugin::doHardwareConnect(uint32_t baudrate, std::string portName)
{
  XsensResultValue res;
  List<CmtPortInfo> portInfo;

  xsens::cmtScanPorts(portInfo);

  CmtPortInfo current = {0, 0, 0, ""};
  current.m_baudrate = numericToRate(baudrate);
  sprintf(current.m_portName, portName.c_str());

  mc_rtc::log::info("Using COM port {} at {} baud", current.m_portName, current.m_baudrate);

  mc_rtc::log::info("Opening port...");

  // open the port which the device is connected to and connect at the device's baudrate.
  res = cmt3_->openPort(current.m_portName, current.m_baudrate);
  exit_on_error(res, "cmtOpenPort");

  mc_rtc::log::info("done");

  // set the measurement timeout to 100ms (default is 16ms)
  int timeOut = 100;
  res = cmt3_->setTimeoutMeasurement(timeOut);
  exit_on_error(res, "set measurement timeout");
  mc_rtc::log::info("Measurement timeout set to {} ms", timeOut);

  // get the Mt sensor count.
  mtCount = cmt3_->getMtCount();
  mtCount = mtCount;
  mc_rtc::log::info("MotionTracker count: {}", mtCount);

  // retrieve the device IDs
  mc_rtc::log::info("Retrieving MotionTrackers device ID(s)");
  for(unsigned int j = 0; j < mtCount; j++)
  {
    res = cmt3_->getDeviceId((unsigned char)(j + 1), deviceIds_[j]);
    exit_on_error(res, "getDeviceId");
    // long deviceIdVal = (long)deviceIds_[j];
    // mc_rtc::log::info("Device ID at busId {}: {}",j+1, deviceIdVal);
    // Done using a printf because device id is an unsigned int32 and mc rtc log does not seem to convert correctly
    printf("Device ID at busId %i: %08lx\n", j + 1, (long)deviceIds_[j]);
  }
}

void ForceShoePlugin::doMtSettings()
{
  XsensResultValue res;

  // set sensor to config sate
  res = cmt3_->gotoConfig();
  exit_on_error(res, "gotoConfig");

  unsigned short sampleFreq;
  sampleFreq = cmt3_->getSampleFrequency();

  // set the device output mode for the device(s)
  CmtDeviceMode deviceMode(mode, settings, sampleFreq);
  for(unsigned int i = 0; i < mtCount; i++)
  {
    res = cmt3_->setDeviceMode(deviceMode, true, deviceIds_[i]);
    exit_on_error(res, "setDeviceMode");
  }

  // start receiving data
  res = cmt3_->gotoMeasurement();
  exit_on_error(res, "gotoMeasurement");
}

void ForceShoePlugin::dataThread()
{
  Mode prevMode = mode_;
  Eigen::Vector6d LBCalib, LFCalib, RBCalib, RFCalib = Eigen::Vector6d::Zero();
  while(th_running_)
  {
    auto res = cmt3_->waitForDataMessage(packet_.get());
    if(res != XRV_OK)
    {
      // FIXME Display a warning on read error?
      continue;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    sdata_ = packet_->getSampleCounter();
    const auto & msg = packet_->m_msg;
    const auto & infoList = packet_->getInfoList(0);
    const auto & calAcc = infoList.m_calAcc;

    for(int i = 0; i < 8; i++)
    {

      LBraw[i] = shortToVolts(msg.getDataShort(calAcc + 1 * CALIB_DATA_OFFSET + 0 * RAWFORCE_OFFSET + 2 * i));
      LFraw[i] = shortToVolts(msg.getDataShort(calAcc + 2 * CALIB_DATA_OFFSET + 1 * RAWFORCE_OFFSET + 2 * i));
      RBraw[i] = shortToVolts(msg.getDataShort(calAcc + 3 * CALIB_DATA_OFFSET + 2 * RAWFORCE_OFFSET + 2 * i));
      RFraw[i] = shortToVolts(msg.getDataShort(calAcc + 4 * CALIB_DATA_OFFSET + 3 * RAWFORCE_OFFSET + 2 * i));
    }
    if(mode_ == Mode::Calibrate)
    {
      if(prevMode != mode_)
      {
        calibSamples_ = 0;
        LBCalib.setZero();
        LFCalib.setZero();
        RBCalib.setZero();
        RFCalib.setZero();
      }
      for(int i = 0; i < 6; ++i)
      {
        LBCalib[i] += LBraw[i];
        LFCalib[i] += LFraw[i];
        RBCalib[i] += RBraw[i];
        RFCalib[i] += RFraw[i];
      }
      calibSamples_ += 1;
      if(calibSamples_ == 100)
      {
        mode_ = Mode::Acquire;
        LBUnload = LBCalib / 100;
        LFUnload = LFCalib / 100;
        RBUnload = RBCalib / 100;
        RFUnload = RFCalib / 100;
        auto calib = mc_rtc::ConfigurationFile(calibFile_);
        calib.add("LBUnload", LBUnload);
        calib.add("LFUnload", LFUnload);
        calib.add("RBUnload", RBUnload);
        calib.add("RFUnload", RFUnload);
        calib.save();
      }
    }
    prevMode = mode_;
  }
}

} // namespace mc_force_shoe_plugin

EXPORT_MC_RTC_PLUGIN("ForceShoePlugin", mc_force_shoe_plugin::ForceShoePlugin)
