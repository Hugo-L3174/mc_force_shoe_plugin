#include "ForceShoePlugin.h"

#include <mc_control/GlobalPluginMacros.h>
#include <mc_rtc/io_utils.h>
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

  // Loading plugin config from schema
  c_.load(config);
  mc_rtc::log::warning("schema config is:\n{}", c_.dump(true, true));
  c_.addToGUI(*ctl.gui(), {"Plugins", "ForceShoePlugin", "Plugin Configuration"}, "Configure");

  cmt3_.reset(new xsens::Cmt3);

  const auto & calibFile = c_.calibFile;
  config("calibFile", calibFile);
  if(std::filesystem::exists(calibFile))
  {
    mc_rtc::log::info("[ForceShoes] Load calibration from {}", calibFile);
    mc_rtc::Configuration calib(calibFile);
    LFUnload = calib("LFUnload");
    LBUnload = calib("LBUnload");
    RFUnload = calib("RFUnload");
    RBUnload = calib("RBUnload");
    mode_ = Mode::Acquire;
  }

  // Putting mode in datastore (true is live, false is replay), true by default
  if(auto ctlLiveMode = ctl.config().find<bool>("ForceShoes", "liveMode"))
  {
    c_.liveMode = *ctlLiveMode;
  }
  ctl.datastore().make<bool>("ForceShoesMode", c_.liveMode);

  if(c_.liveMode)
  {
    doHardwareConnect(c_.baudRate, c_.comPort);
    doMtSettings();

    packet_.reset(new Packet((unsigned short)mtCount, cmt3_->isXm()));
    th_ = std::thread([this]() { dataThread(); });
  }
  reset(controller);
}

void ForceShoePlugin::reset(mc_control::MCGlobalController & controller)
{
  auto & ctl = controller.controller();
  if(c_.liveMode)
  {
    ctl.gui()->addElement(
        {"Plugins", "ForceShoePlugin"},
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
    // TODO: after connecting
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
  for(const auto & [_, forceSensor] : forceShoeSensorsById_)
  {
    forceSensor->addToCtl(controller.controller(), {"Plugins", "ForceShoePlugin", "Sensors"});
  }

  if(c_.liveMode)
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

    // mc_rtc::log::info("LB torque: {}, force: {}", LB.couple().transpose(), LB.force().transpose());
    // mc_rtc::log::info("LF torque: {}, force: {}", LF.couple().transpose(), LF.force().transpose());
    // mc_rtc::log::info("RB torque: {}, force: {}", RB.couple().transpose(), RB.force().transpose());
    // mc_rtc::log::info("RF torque: {}, force: {}", RF.couple().transpose(), RF.force().transpose());
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
  mc_rtc::log::info("MotionTracker count: {}", mtCount);

  // retrieve the device IDs
  mc_rtc::log::info("Retrieving MotionTrackers device ID(s)");
  for(unsigned int j = 0; j < mtCount; j++)
  {
    res = cmt3_->getDeviceId((unsigned char)(j + 1), deviceIds_[j]);
    exit_on_error(res, "getDeviceId");

    // formats the argument as a hexadecimal (`x`), with at least 8 digits (`08`), padded with zeros if necessary.
    // This gives us the IMU id in the same format as what is written on the device.
    std::string id_str = fmt::format("{:08x}", (long)deviceIds_[j]);
    mc_rtc::log::info("Device ID at busId {}: {}", j + 1, id_str);

    // Look for a matching configuration and create a ForceShoeSensor instance if found
    auto found = std::find_if(c_.forceShoeSensors.begin(), c_.forceShoeSensors.end(),
                              [&id_str](const auto & sensorConfig)
                              { return sensorConfig.motionTracker.serialNumber == id_str; });
    if(found != c_.forceShoeSensors.end())
    {
      mc_rtc::log::info("[ForceShoes] Associated sensor with ID {} to configuration:\n{}", id_str,
                        found->dump(true, true));
      auto [it, inserted] = forceShoeSensorsById_.emplace(
          id_str, std::make_unique<ForceShoeSensor>(found->forceSensor.serialNumber, *found));
    }
    else
    {
      mc_rtc::log::warning("[ForceShoes] No configuration found for sensor with ID {}, ignoring", id_str);
    }
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

    auto getDataShort = [&](int device, int i)
    { return msg.getDataShort(calAcc + (device + 1) * CALIB_DATA_OFFSET + device * RAWFORCE_OFFSET + 2 * i); };
    auto readDeviceVoltage = [this, getDataShort](int device)
    {
      std::array<double, 8> voltage;
      for(int i = 0; i < 8; ++i)
      {
        voltage[i] = shortToVolts(getDataShort(device, i));
      }
      return voltage;
    };

    bool calibrated = false;
    for(int i = 0; i < forceShoeSensorsById_.size(); ++i)
    {
      auto & sensor = *std::next(forceShoeSensorsById_.begin(), i)->second;
      if(mode_ == Mode::Calibrate)
      {
        if(prevMode != mode_)
        {
          sensor.startCalibration();
        }
        if(sensor.addCalibrationSample(readDeviceVoltage(i), c_.calibrationSamples))
        {
          calibrated = true;
        }
      }
      else
      {
        if(i == 0)
        {
          // mc_rtc::log::info("raw voltage new: {}", mc_rtc::io::to_string(readDeviceVoltage(i)));
        }
        sensor.setMeasuredVoltage(readDeviceVoltage(i));
      }
    }
    if(calibrated)
    {
      mode_ = Mode::Acquire;
      mc_rtc::log::info("[ForceShoes] Calibration completed, switching to Acquire mode");
    }

    for(int i = 0; i < 8; i++)
    {

      LBraw[i] = shortToVolts(msg.getDataShort(calAcc + 1 * CALIB_DATA_OFFSET + 0 * RAWFORCE_OFFSET + 2 * i));

      LFraw[i] = shortToVolts(msg.getDataShort(calAcc + 2 * CALIB_DATA_OFFSET + 1 * RAWFORCE_OFFSET + 2 * i));
      RBraw[i] = shortToVolts(msg.getDataShort(calAcc + 3 * CALIB_DATA_OFFSET + 2 * RAWFORCE_OFFSET + 2 * i));
      RFraw[i] = shortToVolts(msg.getDataShort(calAcc + 4 * CALIB_DATA_OFFSET + 3 * RAWFORCE_OFFSET + 2 * i));
      // mc_rtc::log::info(
      //   "LBraw[{}]: {} LFraw[{}]: {} RBraw[{}]: {} RFraw[{}]: {}",
      //   i,
      //   mc_rtc::io::to_string(std::array<double, 1>{LBraw[i]}),
      //   i,
      //   mc_rtc::io::to_string(std::array<double, 1>{LFraw[i]}),
      //   i,
      //   mc_rtc::io::to_string(std::array<double, 1>{RBraw[i]}),
      //   i,
      //   mc_rtc::io::to_string(std::array<double, 1>{RFraw[i]})
      // );
    }
    // mc_rtc::log::info("raw voltage ori: {}, {}, {}, {}, {}, {}, {}, {}", LBraw[0], LBraw[1], LBraw[2], LBraw[3],
    // LBraw[4], LBraw[5], LBraw[6], LBraw[7]);
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
      const auto Nsamples = c_.calibrationSamples;
      if(calibSamples_ == Nsamples)
      {
        mode_ = Mode::Acquire;
        LBUnload = LBCalib / Nsamples;
        LFUnload = LFCalib / Nsamples;
        RBUnload = RBCalib / Nsamples;
        RFUnload = RFCalib / Nsamples;
        auto calib = mc_rtc::ConfigurationFile(c_.calibFile);
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
