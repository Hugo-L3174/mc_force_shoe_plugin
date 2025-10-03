/*
 * Copyright 2021 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_control/GlobalPlugin.h>
#include <mc_rtc/Schema.h>
#include <mc_rtc/io_utils.h>
#include <SpaceVecAlg/EigenTypedef.h>
#include <SpaceVecAlg/SpaceVecAlg>
#include <cstring>
#include <iomanip>
#include <sstream>
#include <thread>

#include "cmt3.h"
#include "cmtscan.h"
#include "xsens_list.h"
using namespace xsens;

constexpr double DEFAULT_AMP_GAIN = 4.7;
constexpr int CALIB_DATA_OFFSET = 3 * 12; // 3*12 bytes
constexpr int RAWFORCE_OFFSET = 16; // 16 bytes

namespace mc_force_shoe_plugin
{

struct MotionTrackerSchema
{
  MC_RTC_NEW_SCHEMA(MotionTrackerSchema)
#define MEMBER(...) MC_RTC_SCHEMA_REQUIRED_DEFAULT_MEMBER(MotionTrackerSchema, __VA_ARGS__)
  MEMBER(std::string, serialNumber, "Motion Tracker's serial number")
#undef MEMBER
};

struct ForceSensorCalibrationSchema
{
  MC_RTC_NEW_SCHEMA(ForceSensorCalibrationSchema)
  MC_RTC_SCHEMA_MEMBER(ForceSensorCalibrationSchema,
                       Eigen::Vector6d,
                       calibrationVector,
                       "Gravity compensation offset",
                       mc_rtc::schema::Interactive,
                       Eigen::Vector6d::Zero())
};

struct ForceSensorSchema
{
  MC_RTC_NEW_SCHEMA(ForceSensorSchema)
#define MEMBER(...) MC_RTC_SCHEMA_REQUIRED_DEFAULT_MEMBER(ForceSensorSchema, __VA_ARGS__)
  MEMBER(std::string, serialNumber, "Force sensor's serial number")
#undef MEMBER
  // XXX: should be a MatrixXd
  MC_RTC_SCHEMA_MEMBER(ForceSensorSchema,
                       Eigen::MatrixXd,
                       calibrationMatrix,
                       "Calibration matrix represented as a vector",
                       mc_rtc::schema::None,
                       Eigen::MatrixXd::Zero(6, 6))
  MC_RTC_SCHEMA_MEMBER(ForceSensorSchema, double, ampGain, "Amplification gain", mc_rtc::schema::None, DEFAULT_AMP_GAIN)
};

struct ForceShoeSensorSchema
{
  MC_RTC_NEW_SCHEMA(ForceShoeSensorSchema)
#define MEMBER(...) MC_RTC_SCHEMA_REQUIRED_DEFAULT_MEMBER(ForceShoeSensorSchema, __VA_ARGS__)
  MEMBER(MotionTrackerSchema, motionTracker, "Motion tracker associated with this force sensor")
  MEMBER(ForceSensorSchema, forceSensor, "Force sensor associated with this motion tracker")
#undef MEMBER
};

/// Mapping of robot force sensors to force shoe sensors
struct RobotForceSensorEntrySchema
{
  MC_RTC_NEW_SCHEMA(RobotForceSensorEntrySchema)
  MC_RTC_SCHEMA_MEMBER(RobotForceSensorEntrySchema,
                       std::string,
                       serialNumber,
                       "Serial number of the force sensor",
                       mc_rtc::schema::None,
                       "");
};

struct ForceShoePluginSchema
{
  MC_RTC_NEW_SCHEMA(ForceShoePluginSchema)
  MC_RTC_SCHEMA_MEMBER(ForceShoePluginSchema,
                       std::string,
                       comPort,
                       "COM port to use to connect to the Xsens device",
                       mc_rtc::schema::None,
                       "/dev/ttyUSB0")
  MC_RTC_SCHEMA_MEMBER(ForceShoePluginSchema,
                       int,
                       baudRate,
                       "baudrate to use to connect to the Xsens device",
                       mc_rtc::schema::None,
                       921600)
  MC_RTC_SCHEMA_MEMBER(ForceShoePluginSchema,
                       std::string,
                       calibFile,
                       "calibration file",
                       mc_rtc::schema::None,
                       "/tmp/force-shoe-calib.yaml")
  MC_RTC_SCHEMA_MEMBER(ForceShoePluginSchema, bool, liveMode, "by default, live reading", mc_rtc::schema::None, true)
  MC_RTC_SCHEMA_MEMBER(ForceShoePluginSchema,
                       unsigned int,
                       calibrationSamples,
                       "Number of samples for gravity compensation calibration",
                       mc_rtc::schema::Interactive,
                       100)
  using VectorForceShoeSensor = std::vector<ForceShoeSensorSchema>;
  MC_RTC_SCHEMA_MEMBER(ForceShoePluginSchema,
                       VectorForceShoeSensor,
                       forceShoeSensors,
                       "List of force shoe sensors",
                       mc_rtc::schema::None,
                       VectorForceShoeSensor{})
  // Link force shoe sensors to robot force sensors
  using RobotForceSensorMap = std::map<std::string, RobotForceSensorEntrySchema>;
  MC_RTC_SCHEMA_MEMBER(ForceShoePluginSchema,
                       RobotForceSensorMap,
                       robotForceSensors,
                       "Optional mapping to robot force sensors",
                       mc_rtc::schema::None,
                       RobotForceSensorMap{})
};

/**
 * @brief tests for an error and exits the program with a message if there was one
 */
void exit_on_error(XsensResultValue res, const std::string & comment);

struct ForceShoeSensor
{
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

    using namespace mc_rtc::gui;
    category.push_back(name_);
    auto prefix = mc_rtc::io::to_string(category, "::");
    // if live mode
    ctl.datastore().make<sva::ForceVecd>(prefix + "::Force", Eigen::Vector6d::Zero());

    ctl.gui()->addElement(this, category, Label("name", [this]() { return name_; }),
                          Label("Calibrated", [this]() { return calibrated_ ? "true" : "false"; }),
                          ArrayLabel("Unloaded Voltage", [this]() { return unloadedVoltage(); }),
                          ArrayLabel("Measured Voltage", [this]() { return measuredVoltage(); }),
                          ArrayLabel("Calibrated Voltage", [this]() { return calibratedVoltage(); }),
                          ArrayLabel("Force", [this]() { return measuredForce().vector(); }));
    // else
    // ctl.datastore().make_call("ForceShoePlugin::GetLFForce",
    //                           [&ctl, this]() { return ctl.datastore().get<sva::ForceVecd>("ReplayPlugin::LFForce");
    //                           });

    // Check if the controller has a force sensor defined in the plugin's configuration
  }

  void updateRobotSensor(mc_rbdyn::Robot & robot)
  {
    // XXX should be mapped once and updated if the sensor exists
    const auto & robotSensorsConfig = pluginConfig_.robotForceSensors;
    auto it = std::find_if(robotSensorsConfig.begin(), robotSensorsConfig.end(), [this](const auto & entry)
                           { return entry.second.serialNumber == config_.forceSensor.serialNumber; });
    if(it != robotSensorsConfig.end())
    {
      const auto & sensorName = it->first;
      if(robot.hasForceSensor(sensorName))
      {
        auto & data = *robot.data();
        auto & fs = data.forceSensors[data.forceSensorsIndex.at(sensorName)];
        fs.wrench(measuredForce());
      }
      else
      {
        mc_rtc::log::warning("[ForceShoeSensor {}] Robot {} has no force sensor named {}", name_, robot.name(),
                             sensorName);
      }
    }
    else
    {
      // mc_rtc::log::info("[ForceShoeSensor {}] No mapping found for force sensor with serial number {}",
      //                   name_, config_.forceSensor.serialNumber);
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
