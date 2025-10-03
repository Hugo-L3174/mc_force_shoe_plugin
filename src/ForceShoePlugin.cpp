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
  c_.addToGUI(*ctl.gui(), {"Plugins", "ForceShoePlugin", "Plugin Configuration"}, "Configure");

  cmt3_.reset(new xsens::Cmt3);

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
        mc_rtc::gui::Label("All sensors calibrated?", [this]() { return checkAllCalibrated() ? "true" : "false"; }),
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
    // ctl.datastore().make<sva::ForceVecd>("ForceShoePlugin::LFForce", sva::ForceVecd::Zero());
    // ctl.datastore().make<sva::ForceVecd>("ForceShoePlugin::LBForce", sva::ForceVecd::Zero());
    // ctl.datastore().make<sva::ForceVecd>("ForceShoePlugin::RFForce", sva::ForceVecd::Zero());
    // ctl.datastore().make<sva::ForceVecd>("ForceShoePlugin::RBForce", sva::ForceVecd::Zero());
    //
    // ctl.datastore().make_call("ForceShoePlugin::GetLFForce", [&ctl, this]()
    //                           { return ctl.datastore().get<sva::ForceVecd>("ForceShoePlugin::LFForce"); });
    // ctl.datastore().make_call("ForceShoePlugin::GetLBForce", [&ctl, this]()
    //                           { return ctl.datastore().get<sva::ForceVecd>("ForceShoePlugin::LBForce"); });
    // ctl.datastore().make_call("ForceShoePlugin::GetRFForce", [&ctl, this]()
    //                           { return ctl.datastore().get<sva::ForceVecd>("ForceShoePlugin::RFForce"); });
    // ctl.datastore().make_call("ForceShoePlugin::GetRBForce", [&ctl, this]()
    //                           { return ctl.datastore().get<sva::ForceVecd>("ForceShoePlugin::RBForce"); });
  }
  else
  {
    // ctl.datastore().make_call("ForceShoePlugin::GetLFForce",
    //                           [&ctl, this]() { return ctl.datastore().get<sva::ForceVecd>("ReplayPlugin::LFForce");
    //                           });
    // ctl.datastore().make_call("ForceShoePlugin::GetLBForce",
    //                           [&ctl, this]() { return ctl.datastore().get<sva::ForceVecd>("ReplayPlugin::LBForce");
    //                           });
    // ctl.datastore().make_call("ForceShoePlugin::GetRFForce",
    //                           [&ctl, this]() { return ctl.datastore().get<sva::ForceVecd>("ReplayPlugin::RFForce");
    //                           });
    // ctl.datastore().make_call("ForceShoePlugin::GetRBForce",
    //                           [&ctl, this]() { return ctl.datastore().get<sva::ForceVecd>("ReplayPlugin::RBForce");
    //                           });
  }

  mc_rtc::log::info("ForceShoePlugin::reset called");
}

void ForceShoePlugin::before(mc_control::MCGlobalController & controller)
{
  for(const auto & [_, forceSensor] : forceShoeSensorsById_)
  {
    forceSensor->addToCtl(controller.controller(), {"Plugins", "ForceShoePlugin", "Sensors"});
    forceSensor->updateRobotSensor(controller.robot());
  }

  if(c_.liveMode)
  {
    // auto & ctl = controller.controller();
    // auto & LF = ctl.datastore().get<sva::ForceVecd>("ForceShoePlugin::LFForce");
    // auto & LB = ctl.datastore().get<sva::ForceVecd>("ForceShoePlugin::LBForce");
    // auto & RF = ctl.datastore().get<sva::ForceVecd>("ForceShoePlugin::RFForce");
    // auto & RB = ctl.datastore().get<sva::ForceVecd>("ForceShoePlugin::RBForce");
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

void ForceShoePlugin::saveCalibration()
{
  auto calib = mc_rtc::ConfigurationFile(c_.calibFile);
  for(const auto & [id, sensor] : forceShoeSensorsById_)
  {
    calib.add(id, sensor->unloadedVoltage());
  }
  mc_rtc::log::info("[ForceShoes] Saved calibration to {}", c_.calibFile);
  calib.save();
}

void ForceShoePlugin::loadCalibration()
{
  if(std::filesystem::exists(c_.calibFile))
  {
    mc_rtc::log::info("[Forchoes] Load calibration from {}", c_.calibFile);
    auto calib = mc_rtc::ConfigurationFile(c_.calibFile);
    for(const auto & [id, sensor] : forceShoeSensorsById_)
    {
      if(auto calibValue = calib.find(id))
      {
        mc_rtc::log::success("Found calibration for sensor with ID {}, setting unloaded voltage to {}", id,
                             calibValue->dump(true, true));
        sensor->setUnloadedVoltage(*calibValue);
      }
      else
      {
        mc_rtc::log::warning("No calibration found for sensor with ID {}, you should calibrate it", id);
      }
    }
    if(checkAllCalibrated())
    {
      mc_rtc::log::success("[ForceShoes] All sensors are calibrated, starting in acquisition mode");
      mode_ = Mode::Acquire;
    }
    else
    {
      mc_rtc::log::warning("[ForceShoes] Not all sensors are calibrated, starting in calibration mode");
      mode_ = Mode::Calibrate;
    }
  }
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
    auto found =
        std::find_if(c_.forceShoeSensors.begin(), c_.forceShoeSensors.end(), [&id_str](const auto & sensorConfig)
                     { return sensorConfig.motionTracker.serialNumber == id_str; });
    if(found != c_.forceShoeSensors.end())
    {
      mc_rtc::log::info("[ForceShoes] Associated sensor with ID {} to configuration:\n{}", id_str,
                        found->dump(true, true));
      // order in which the sensors were detected
      forceShoeSensorsIds_.push_back(id_str);
      // create a matching sensor implementation to handle calibration and force computation. This can be used to access
      // sensors by id WARNING: do not use the keys in this map for ordering, use forceShoeSensorsIds_ instead
      auto [it, inserted] = forceShoeSensorsById_.emplace(
          id_str, std::make_unique<ForceShoeSensor>(found->forceSensor.serialNumber, *found, c_));
    }
    else
    {
      mc_rtc::log::warning("[ForceShoes] No configuration found for sensor with ID {}, ignoring", id_str);
    }
  }
  loadCalibration();
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
    const auto itemCount = packet_->m_itemCount; // number of items in the packet
    const auto & infoList = packet_->getInfoList(0);
    const auto & calAcc = infoList.m_calAcc;

    auto getDataShort = [&](int device, int i)
    { return msg.getDataShort(calAcc + (device + 1) * CALIB_DATA_OFFSET + device * RAWFORCE_OFFSET + 2 * i); };
    std::array<double, 8> voltage;
    auto readDeviceVoltage = [this, voltage, getDataShort](int device) mutable
    {
      for(int i = 0; i < 8; ++i)
      {
        voltage[i] = shortToVolts(getDataShort(device, i));
      }
      return voltage;
    };

    bool calibrated = false;
    for(int i = 0; i < forceShoeSensorsIds_.size(); ++i)
    {
      auto & id = forceShoeSensorsIds_[i];
      auto & sensor = *forceShoeSensorsById_[id];
      if(mode_ == Mode::Calibrate)
      {
        if(prevMode != mode_)
        {
          sensor.startCalibration();
        }
        if(sensor.addCalibrationSample(readDeviceVoltage(i), c_.calibrationSamples))
        {
          saveCalibration();
          calibrated = true;
        }
      }
      else
      {
        sensor.setMeasuredVoltage(readDeviceVoltage(i));
      }
    }
    if(calibrated)
    {
      mode_ = Mode::Acquire;
      mc_rtc::log::info("[ForceShoes] Calibration completed, switching to Acquire mode");
    }
    prevMode = mode_;
  }
}

} // namespace mc_force_shoe_plugin

EXPORT_MC_RTC_PLUGIN("ForceShoePlugin", mc_force_shoe_plugin::ForceShoePlugin)
