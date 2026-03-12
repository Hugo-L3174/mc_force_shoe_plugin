#include <mc_rtc/Schema.h>

constexpr double DEFAULT_AMP_GAIN = 4.7;
constexpr int CALIB_DATA_OFFSET = 3 * 12; // 3*12 bytes
constexpr int RAWFORCE_OFFSET = 16; // 16 bytes
constexpr double DEFAULT_LOWPASS_PERIOD = 0.1; // 16 bytes

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
  MC_RTC_SCHEMA_MEMBER(ForceSensorSchema,
                       double,
                       lowPassPeriod,
                       "Period for low pass filtering",
                       mc_rtc::schema::None,
                       DEFAULT_LOWPASS_PERIOD)
};

struct ForceShoeSensorSchema
{
  MC_RTC_NEW_SCHEMA(ForceShoeSensorSchema)
#define MEMBER(...) MC_RTC_SCHEMA_REQUIRED_DEFAULT_MEMBER(ForceShoeSensorSchema, __VA_ARGS__)
  MEMBER(MotionTrackerSchema, motionTracker, "Motion tracker associated with this force sensor")
  MEMBER(ForceSensorSchema, forceSensor, "Force sensor associated with this motion tracker")
#undef MEMBER
};

struct RobotForceSensorEntryFlipAxisSchema
{
  MC_RTC_NEW_SCHEMA(RobotForceSensorEntryFlipAxisSchema)
  MC_RTC_SCHEMA_MEMBER(RobotForceSensorEntryFlipAxisSchema,
                       bool,
                       x,
                       "flip X axis measurement",
                       mc_rtc::schema::None,
                       false)
  MC_RTC_SCHEMA_MEMBER(RobotForceSensorEntryFlipAxisSchema,
                       bool,
                       y,
                       "flip Y axis measurement",
                       mc_rtc::schema::None,
                       false)
  MC_RTC_SCHEMA_MEMBER(RobotForceSensorEntryFlipAxisSchema,
                       bool,
                       z,
                       "flip Z axis measurement",
                       mc_rtc::schema::None,
                       false)
};

/// Mapping of robot force sensors to force shoe sensors
struct RobotForceSensorEntrySchema
{
  MC_RTC_NEW_SCHEMA(RobotForceSensorEntrySchema)
  MC_RTC_SCHEMA_MEMBER(RobotForceSensorEntrySchema,
                       std::string,
                       robotForceSensorName,
                       "Name of the force sensor on the robot",
                       mc_rtc::schema::None,
                       "")
  MC_RTC_SCHEMA_MEMBER(RobotForceSensorEntrySchema,
                       std::string,
                       serialNumber,
                       "Serial number of the force sensor",
                       mc_rtc::schema::None,
                       "")
  MC_RTC_SCHEMA_MEMBER(RobotForceSensorEntrySchema,
                       double,
                       guiScale,
                       "Display scale of the force sensor arrow",
                       mc_rtc::schema::None,
                       1.0)
  MC_RTC_SCHEMA_MEMBER(RobotForceSensorEntrySchema,
                       RobotForceSensorEntryFlipAxisSchema,
                       flipMeasurementAxis,
                       "flip measurment along axis",
                       mc_rtc::schema::None,
                       RobotForceSensorEntryFlipAxisSchema{})
};

/**
 * Main configuration schema for the ForceShoePlugin
 */
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
  // Map of robot name to robot force sensor configs
  using RobotForceSensorMap = std::map<std::string, std::vector<RobotForceSensorEntrySchema>>;
  MC_RTC_SCHEMA_MEMBER(ForceShoePluginSchema,
                       RobotForceSensorMap,
                       robotForceSensors,
                       "Optional mapping to robot force sensors",
                       mc_rtc::schema::None,
                       RobotForceSensorMap{})
  MC_RTC_SCHEMA_MEMBER(ForceShoePluginSchema, bool, debug, "Additional debug information", mc_rtc::schema::None, false)
};
