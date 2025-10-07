/*
 * Copyright 2021 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_control/GlobalPlugin.h>
#include <mc_filter/LowPass.h>
#include <mc_rtc/io_utils.h>
#include <SpaceVecAlg/SpaceVecAlg>
#include "ForceShoePluginSchemas.h"
#include <cstring>
#include <thread>

#include "cmt3.h"
#include "cmtscan.h"
#include "xsens_list.h"
using namespace xsens;

namespace mc_force_shoe_plugin
{

/**
 * @brief tests for an error and exits the program with a message if there was one
 */
void exit_on_error(XsensResultValue res, const std::string & comment);

struct ForceShoeSensor
{
  struct UpdateRobotSensor
  {
    std::string robotName;
    RobotForceSensorEntrySchema fsEntrySchema;
  };

  // Raw data vectors (G0, G1, G2, G3, G4, G5, G6, ref)
  using RawMeasurementArray = std::array<double, 8>;
  using VoltageArray = std::array<double, 6>; // FIXME: should be 6 but segfaults

  ForceShoeSensor(const std::string & name, ForceShoeSensorSchema & config_, ForceShoePluginSchema & pluginConfig)
  : name_(name), config_(config_), pluginConfig_(pluginConfig), ampCalMat_(Eigen::MatrixXd::Zero(6, 6))
  {
  }

  ForceShoeSensor(const ForceShoeSensor &) = delete;
  ForceShoeSensor & operator=(const ForceShoeSensor &) = delete;

  std::string name_;
  ForceShoeSensorSchema & config_;
  ForceShoePluginSchema & pluginConfig_;

  void addToCtl(mc_control::MCController & ctl, std::vector<std::string> category)
  {
    if(inCtl_) return;
    inCtl_ = true;
    guiCategory_ = category;

    // Check if the controller has a force sensor defined in the plugin's configuration
    const auto & robotSensorsConfig = pluginConfig_.robotForceSensors;
    // For each robot in the configuration
    for(const auto & [robotName, robotForceSensors] : robotSensorsConfig)
    {
      mc_rtc::log::info("robot name is {}", ctl.robot().name());
      if(ctl.hasRobot(robotName))
      {
        mc_rtc::log::info("[ForceShoeSensor {}] Checking mapping to robot {}", name_, robotName);
        auto & robot = ctl.robot(robotName);
        for(const auto & sensor : robotForceSensors)
        {
          mc_rtc::log::info("  robot sensor name is {}", sensor.robotForceSensorName);
        }
        auto it = std::find_if(robotForceSensors.begin(), robotForceSensors.end(),
                               [this](const auto & entry)
                               {
                                 mc_rtc::log::info("Comparing {} with {}", entry.serialNumber,
                                                   config_.forceSensor.serialNumber);
                                 return entry.serialNumber == config_.forceSensor.serialNumber;
                               });
        if(it != robotForceSensors.end())
        {
          const auto & robotSensorName = it->robotForceSensorName;
          mc_rtc::log::info("[ForceShoeSensor {}] Found mapping to robot {} force sensor {}", name_, robot.name(),
                            robotSensorName);
          if(robot.hasForceSensor(robotSensorName))
          {
            updateRobotSensor_ = {robotName, *it};
            mc_rtc::log::info("[ForceShoeSensor {}] Mapping to robot {} force sensor {}", name_, robot.name(),
                              robotSensorName);
          }
        }
      }
    }

    updateForceFilterConfig(ctl.timeStep);

    using namespace mc_rtc::gui;
    category.push_back(name_);
    auto prefix = mc_rtc::io::to_string(category, "::");
    // if live mode
    ctl.datastore().make<sva::ForceVecd>(prefix + "::Force", Eigen::Vector6d::Zero());
    // else
    // ctl.datastore().make_call("ForceShoePlugin::GetLFForce",
    //                           [&ctl, this]() { return ctl.datastore().get<sva::ForceVecd>("ReplayPlugin::LFForce");
    //                           });

    ctl.gui()->addElement(this, category, Label("name", [this]() { return name_; }),
                          Label("Calibrated", [this]() { return calibrated_ ? "true" : "false"; }),
                          ArrayLabel("Unloaded Voltage", [this]() { return unloadedVoltage(); }),
                          ArrayLabel("Measured Voltage", [this]() { return measuredVoltage(); }),
                          ArrayLabel("Calibrated Voltage", [this]() { return calibratedVoltage(); }),
                          ArrayLabel("Measured Force Raw (FS frame)", [this]() { return measuredForce().vector(); }),
                          ArrayLabel("Filtered Force (FS frame)", [this]() { return filteredForce(); }),
                          NumberInput(
                              "Filter LowPass Period [s]", [this]() { return config_.forceSensor.lowPassPeriod; },
                              [this, &ctl](double period)
                              {
                                config_.forceSensor.lowPassPeriod = period;
                                updateForceFilterConfig(ctl.timeStep);
                              }));

    if(updateRobotSensor_)
    {
      const auto & robotName = updateRobotSensor_->robotName;
      const auto & sensorName = updateRobotSensor_->fsEntrySchema.robotForceSensorName;
      const auto & guiScale = updateRobotSensor_->fsEntrySchema.guiScale;
      category.push_back("Force Arrows");

      mc_rtc::gui::ForceConfig forceConfig;
      forceConfig.scale *= guiScale;
      ctl.gui()->addElement(this, category,
                            Force(
                                robotName + "::" + sensorName, forceConfig, [robotName, sensorName, &ctl]()
                                { return ctl.robot(robotName).forceSensor(sensorName).wrench(); },
                                [robotName, sensorName, &ctl]()
                                { return ctl.robot(robotName).frame(sensorName).position(); }));
    }
  }

  void updateForceFilterConfig(double dt)
  {
    forceFilter_ = std::make_unique<mc_filter::LowPass<sva::ForceVecd>>(dt, config_.forceSensor.lowPassPeriod);
    resetForceFilter_ = true;
  }

  void updateRobotSensor(mc_control::MCController & ctl)
  {
    if(updateRobotSensor_)
    {
      const auto & [robotName, sensorConfig] = *updateRobotSensor_;
      auto & robot = ctl.robot(robotName);
      auto & data = *robot.data();
      auto & fs = data.forceSensors[data.forceSensorsIndex.at(sensorConfig.robotForceSensorName)];

      auto measured = filteredForce();
      const auto & flipAxis = sensorConfig.flipMeasurementAxis;
      // Flip force along axis (left-handed sensor)
      if(flipAxis.x)
      {
        measured.force().x() *= -1.;
        measured.couple().x() *= -1.;
      }
      if(flipAxis.y)
      {
        mc_rtc::log::info("{} flipped y axis", sensorConfig.robotForceSensorName);
        measured.force().y() *= -1.;
        measured.couple().y() *= -1.;
      }
      if(flipAxis.z)
      {
        measured.force().z() *= -1.;
        measured.couple().z() *= -1.;
      }

      fs.wrench(measured);
    }

    if(guiCategory_)
    {
      auto category = *guiCategory_;
      category.push_back(name_);
      // publish to datastore
      ctl.datastore().get<sva::ForceVecd>(mc_rtc::io::to_string(category, "::") + "::Force") = measuredForce();
    }
  }

  // auto setForceShoeSensorValue =
  //     [this](const std::string & datastoreName, const std::string & robotName, const std::string & sensorName)
  // {
  //   auto & data = *robot(robotName).data();
  //   auto & fs = data.forceSensors[data.forceSensorsIndex.at(sensorName)];
  //   if(this->datastore().has(datastoreName))
  //   {
  //     auto & wrench = this->datastore().get<sva::ForceVecd>(datastoreName);
  //     fs.wrench(wrench);
  //   }
  //   else
  //   {
  //     fs.wrench(sva::ForceVecd::Zero());
  //   }
  // };

  void removeFromCtl(mc_control::MCController & ctl, std::vector<std::string> category)
  {
    inCtl_ = false;
    using namespace mc_rtc::gui;
    category.push_back(name_);
    auto prefix = mc_rtc::io::to_string(category, "::");
    ctl.datastore().remove(prefix + "::Force");
    ctl.gui()->removeElements(this);
  }

  /// Set raw measured voltage
  /// compute difference between current voltages and unloaded voltages
  void setMeasuredVoltage(const RawMeasurementArray & voltage)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    measuredVoltage_ = voltage;
    for(size_t i = 0; i < unloadedVoltage_.size(); ++i)
    {
      calibratedVoltage_[i] = measuredVoltage_[i] - unloadedVoltage_[i];
    }
    computeForceFromVoltage();
  }

  void startCalibration()
  {
    std::lock_guard<std::mutex> lock(calibMutex_);
    mc_rtc::log::info("[ForceShoeSensor {}] Starting calibration", name_);
    calibrationVoltageAccumulator_.fill(0.);
  }

  bool addCalibrationSample(const RawMeasurementArray & voltage, unsigned int Nsamples = 100)
  {
    // mc_rtc::log::info("[ForceShoeSensor {}] Adding calibration sample {}: {}", name_, samples_,
    // mc_rtc::io::to_string(voltage));
    std::lock_guard<std::mutex> lock(calibMutex_);
    for(size_t i = 0; i < calibrationVoltageAccumulator_.size(); ++i)
    {
      calibrationVoltageAccumulator_[i] += voltage[i];
    }

    if(++samples_ == Nsamples)
    {
      std::lock_guard<std::mutex> lock(mutex_);
      for(size_t i = 0; i < calibrationVoltageAccumulator_.size(); ++i)
      {
        unloadedVoltage_[i] = calibrationVoltageAccumulator_[i] / static_cast<double>(samples_);
      }
      calibrated_ = true;
      resetForceFilter_ = true;
      samples_ = 0;
      mc_rtc::log::info("[ForceShoeSensor {}] Calibration done, unloaded voltage: {}", name_,
                        mc_rtc::io::to_string(unloadedVoltage_));
      return true;
    }

    return false;
  }

  sva::ForceVecd measuredForce() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return measuredForce_;
  }

  sva::ForceVecd filteredForce() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if(forceFilter_)
    {
      return forceFilter_->eval();
    }
    else
    {
      return measuredForce_;
    }
  }

  VoltageArray unloadedVoltage() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return unloadedVoltage_;
  }

  void setUnloadedVoltage(const VoltageArray & unloadedVoltage)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    unloadedVoltage_ = unloadedVoltage;
    mc_rtc::log::info("set unloadedVoltage for sensor {}: {}", name_, mc_rtc::io::to_string(unloadedVoltage_));
    calibrated_ = true;
  }

  VoltageArray calibratedVoltage() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return calibratedVoltage_;
  }

  RawMeasurementArray measuredVoltage() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return measuredVoltage_;
  }

  inline bool calibrated() const noexcept
  {
    return calibrated_;
  }

private:
  void computeForceFromVoltage()
  {
    computeAmplifiedCalibrationMatrix();

    // calibratedVoltage_ is the voltage after removing the unloaded voltage
    // We convert its first 6 elements (force, couple) to a force using the calibration matrix
    Eigen::VectorXd voltVec = Eigen::Map<const Eigen::VectorXd>(calibratedVoltage_.data(), 6);
    Eigen::VectorXd forceVec = ampCalMat_ * voltVec;
    measuredForce_ = sva::ForceVecd(Eigen::Vector3d{forceVec[3], forceVec[4], forceVec[5]},
                                    Eigen::Vector3d{forceVec[0], forceVec[1], forceVec[2]});
    if(resetForceFilter_)
    {
      forceFilter_->reset(measuredForce_);
      resetForceFilter_ = false;
    }
    forceFilter_->update(measuredForce_);
  }

  /// compute amplified calibration matrixes (raw/amplifier gain/excitation)
  void computeAmplifiedCalibrationMatrix()
  {
    const auto & rawCalMat = config_.forceSensor.calibrationMatrix;
    // mc_rtc::log::info("rawCalMat:\n{}\nampGain: {}\nmeasuredVoltage[6]: {}", rawCalMat, config_.forceSensor.ampGain,
    // measuredVoltage_[6]);
    for(int i = 0; i < 6; i++)
    {
      for(int j = 0; j < 6; j++)
      {
        ampCalMat_(i, j) = rawCalMat(i, j) / config_.forceSensor.ampGain / measuredVoltage_[6];
      }
    }
  }

  VoltageArray calibrationVoltageAccumulator_ = {0.}; /// sum of all measurements since the calibration started
  unsigned samples_ = 0; /// number of samples accumulated for calibration

  /// reference voltage when no load is applied (calibration)
  VoltageArray unloadedVoltage_ = {0.};
  /// raw current voltage reading
  RawMeasurementArray measuredVoltage_ = {0.};
  /// Difference between measured voltage and unloaded
  VoltageArray calibratedVoltage_ = {0.};
  /// Amplified calibration matrixes: result of rawCalMat/amplifierGain/Excitation voltage
  Eigen::MatrixXd ampCalMat_;
  sva::ForceVecd measuredForce_ = sva::ForceVecd::Zero();
  std::unique_ptr<mc_filter::LowPass<sva::ForceVecd>> forceFilter_;
  bool resetForceFilter_ = true;

  // map of robot name to sensor name
  std::optional<UpdateRobotSensor> updateRobotSensor_ = std::nullopt;
  std::optional<std::vector<std::string>> guiCategory_ = std::nullopt;

  bool calibrated_ = false;
  bool inCtl_ = false;
  mutable std::mutex mutex_;
  mutable std::mutex calibMutex_;
};

struct ForceShoePlugin : public mc_control::GlobalPlugin
{
  void init(mc_control::MCGlobalController & controller, const mc_rtc::Configuration & config) override;

  void reset(mc_control::MCGlobalController & controller) override;

  void before(mc_control::MCGlobalController &) override;

  void after(mc_control::MCGlobalController & controller) override;

  mc_control::GlobalPlugin::GlobalPluginConfiguration configuration() override;

  ~ForceShoePlugin() override;

  void saveCalibration();
  void loadCalibration();
  bool checkAllCalibrated() const
  {
    return std::all_of(forceShoeSensorsById_.begin(), forceShoeSensorsById_.end(),
                       [](const auto & kv) { return kv.second->calibrated(); });
  }

  /// Connects to the desired port at the desired baudrate and checks for
  void doHardwareConnect(uint32_t baudrate, std::string portName);

  /// Set user settings in MTi/MTx
  /// Assumes initialized cmt3 class with open COM port
  void doMtSettings();

  /// Convert the short raw value to the voltage in float.
  inline double shortToVolts(const uint16_t raw) const
  {
    return static_cast<double>(raw) * 4.999924 / 65535;
  }

  /// Convert a numeric baudrate in bps to correct hardware values
  inline uint32_t numericToRate(int numeric)
  {
    switch(numeric)
    {
      case 9600:
        return B9600;
      case 19200:
        return B19200;
      case 38400:
        return B38400;
      case 57600:
        return B57600;
      case 115200:
        return B115200;
      case 230400:
        return B230400;
      case 460800:
        return B460800;
      case 921600:
        return B921600;
      default:
        return 0;
    }
  }

  /// Thread to wait for message while not blocking controller
  void dataThread();

private:
  ForceShoePluginSchema c_; /// plugin configuration from schemas

  enum class Mode
  {
    Calibrate,
    Acquire
  };
  Mode mode_ = Mode::Calibrate;

  bool th_running_ = true;
  std::thread th_;
  std::mutex mutex_;
  std::shared_ptr<Packet> packet_;

  CmtOutputMode mode = CMT_OUTPUTMODE_CALIB | CMT_OUTPUTMODE_AUXILIARY;
  CmtOutputSettings settings = CMT_OUTPUTSETTINGS_AUXILIARYMODE_FORCE | CMT_OUTPUTSETTINGS_TIMESTAMP_SAMPLECNT;
  unsigned long mtCount = 0;
  CmtDeviceId deviceIds_[256];
  std::shared_ptr<xsens::Cmt3> cmt3_;

  /// Order in which the force sensors were detected on the bus
  /// This matches the order of the data packets
  /// Use this order when associating with xsens data readings
  std::vector<std::string> forceShoeSensorsIds_;
  /// Actual force shoe sensors implementation
  /// WARNING: do NOT use for ordering, use forceShoeSensorsIds_ instead
  std::unordered_map<std::string, std::unique_ptr<ForceShoeSensor>> forceShoeSensorsById_;

  // sample counter
  unsigned short sdata_ = NULL;
};

} // namespace mc_force_shoe_plugin
