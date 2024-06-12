/*
 * Copyright 2021 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_control/GlobalPlugin.h>
#include <SpaceVecAlg/SpaceVecAlg>
#include <cstring>
#include <thread>

#include <deque>

#include "cmt3.h"
#include "cmtscan.h"
#include "xsens_list.h"
using namespace xsens;

#define ampGain 4.7
#define CALIB_DATA_OFFSET 3 * 12 // 3*12 bytes
#define RAWFORCE_OFFSET 16 // 16 bytes
double coeffLFx=0.9251;
double coeffLFy= 0.9244;
double coeffLfz=0.9326;
double coeffLBx=1.6986;
double coeffLBy=0.93;
double coeffLBz=0.9679;

// this macro tests for an error and exits the program with a message if there was one
#define EXIT_ON_ERROR(res, comment)                                                                    \
  if(res != XRV_OK)                                                                                    \
  {                                                                                                    \
    mc_rtc::log::error_and_throw("Error {} occurred in " comment ": {}\n", res, xsensResultText(res)); \
    exit(1);                                                                                           \
  }


namespace mc_plugin
{ 

struct ForceShoePlugin : public mc_control::GlobalPlugin
{
  void init(mc_control::MCGlobalController & controller, const mc_rtc::Configuration & config) override;

  void reset(mc_control::MCGlobalController & controller) override;

  void before(mc_control::MCGlobalController &) override;

  void after(mc_control::MCGlobalController & controller) override;

  mc_control::GlobalPlugin::GlobalPluginConfiguration configuration() override;

  ~ForceShoePlugin() override;
 

  //////////////////////////////////////////////////////////////////////////
  // doHardwareConnect
  //
  // Connects to the desired port at the desired baudrate and checks for
  void doHardwareConnect(uint32_t baudrate, std::string portName)
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
    EXIT_ON_ERROR(res, "cmtOpenPort");

    mc_rtc::log::info("done");

    // set the measurement timeout to 100ms (default is 16ms)
    int timeOut = 100;
    res = cmt3_->setTimeoutMeasurement(timeOut);
    EXIT_ON_ERROR(res, "set measurement timeout");
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
      EXIT_ON_ERROR(res, "getDeviceId");
      // long deviceIdVal = (long)deviceIds_[j];
      // mc_rtc::log::info("Device ID at busId {}: {}",j+1, deviceIdVal);
      // Done using a printf because device id is an unsigned int32 and mc rtc log does not seem to convert correctly
      printf("Device ID at busId %i: %08lx\n", j + 1, (long)deviceIds_[j]);
    }
  }

  //////////////////////////////////////////////////////////////////////////
  // doMTSettings
  //
  // Set user settings in MTi/MTx
  // Assumes initialized cmt3 class with open COM port
  void doMtSettings()
  {
    XsensResultValue res;

    // set sensor to config sate
    res = cmt3_->gotoConfig();
    EXIT_ON_ERROR(res, "gotoConfig");

    unsigned short sampleFreq;
    sampleFreq = cmt3_->getSampleFrequency();

    // set the device output mode for the device(s)
    CmtDeviceMode deviceMode(mode, settings, sampleFreq);
    for(unsigned int i = 0; i < mtCount; i++)
    {
      res = cmt3_->setDeviceMode(deviceMode, true, deviceIds_[i]);
      EXIT_ON_ERROR(res, "setDeviceMode");
    }

    // start receiving data
    res = cmt3_->gotoMeasurement();
    EXIT_ON_ERROR(res, "gotoMeasurement");
  }

  //////////////////////////////////////////////////////////////////////////////////////////
  // Convert the short raw value to the voltage in float.
  double shortToVolts(const uint16_t raw)
  {
    double U = double(raw);
    U *= 4.999924 / 65535;
    return U;
  }

  //////////////////////////////////////////////////////////////////////////////////////////
  // Convert a numeric baudrate in bps to correct hardware values
  uint32_t numericToRate(int numeric)
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

  //////////////////////////////////////////////////////////////////////////////////////////
  // compute amplified calibration matrixes (raw/amplifier gain/excitation)
  void computeAmpCalMat()
  {
    for(int i = 0; i < 6; i++)
    {
      for(int j = 0; j < 6; j++)
      {
        ampCalMatLB[i][j] = rawCalMatLB[i][j] / ampGain / LBraw[6];
        ampCalMatLF[i][j] = rawCalMatLF[i][j] / ampGain / LFraw[6];
        ampCalMatRB[i][j] = rawCalMatRB[i][j] / ampGain / RBraw[6];
        ampCalMatRF[i][j] = rawCalMatRF[i][j] / ampGain / RFraw[6];
      }
    }

   

  }

  //////////////////////////////////////////////////////////////////////////////////////////
  // compute difference between current voltages and unloaded voltages
  void computeUDiff()
  {
    for(int i = 0; i < 6; i++)
    {
      LBdiff[i] = LBraw[i] - LBUnload[i];
      LFdiff[i] = LFraw[i] - LFUnload[i];
      RBdiff[i] = RBraw[i] - RBUnload[i];
      RFdiff[i] = RFraw[i] - RFUnload[i];
    }
  }

  //////////////////////////////////////////////////////////////////////////////////////////
  // process FT vectors from voltages (amplified calibration matrixes * diff voltage vectors)
  void computeForceVec()
  {
    
    // Reseting values
    for(int i = 0; i < 6; i++)
    {
      LBforcevec[i] = 0.;
      LFforcevec[i] = 0.;
      RBforcevec[i] = 0.;
      RFforcevec[i] = 0.;
    }

    for(int i = 0; i < 6; i++) // i: rows of cal mat
    {
      for(int j = 0; j < 6; j++) // j: column of voltage vect
      {
        LBforcevec[i] += ampCalMatLB[i][j] * LBdiff[j];
        LFforcevec[i] += ampCalMatLF[i][j] * LFdiff[j];
        RBforcevec[i] += ampCalMatRB[i][j] * RBdiff[j];
        RFforcevec[i] += ampCalMatRF[i][j] * RFdiff[j];
      }
    }

    
  }

  // void computeFilteredForceVec()
  // {
  //   constexpr int filterSize=10; 
  //   static std::deque<Eigen::Vector6d> LBForceHistory(filterSize, Eigen::Vector6d::Zero());
  //   static std::deque<Eigen::Vector6d> LFForceHistory(filterSize, Eigen::Vector6d::Zero());
  //   static std::deque<Eigen::Vector6d> RBForceHistory(filterSize, Eigen::Vector6d::Zero());
  //   static std::deque<Eigen::Vector6d> RFForceHistory(filterSize, Eigen::Vector6d::Zero());

  //    static std::deque<Eigen::Vector6d> LBFilteredvec(filterSize, Eigen::Vector6d::Zero());
  //   static std::deque<Eigen::Vector6d> LFFilteredvec(filterSize, Eigen::Vector6d::Zero());
  //   static std::deque<Eigen::Vector6d> RBFilteredvec(filterSize, Eigen::Vector6d::Zero());
  //   static std::deque<Eigen::Vector6d> RFFilteredvec(filterSize, Eigen::Vector6d::Zero());

  //   LBForceHistory.push_front(Eigen::Vector6d(LBforcevec));
  //   LFForceHistory.push_front(Eigen::Vector6d(LFforcevec));
  //   RBForceHistory.push_front(Eigen::Vector6d(RBforcevec));
  //   RFForceHistory.push_front(Eigen::Vector6d(RFforcevec));




  //   if (LBForceHistory.size()>filterSize)
  //     LBForceHistory.pop_back();

  //   if (LFForceHistory.size()>filterSize)
  //     LFForceHistory.pop_back();

  //   if (RBForceHistory.size()>filterSize)
  //     RAWFORCE_OFFSET.pop_back();

  //   if (RFForceHistory.size()>filterSize)
  //     RFForceHistory.pop_back();
    
  //   Eigen::Vector6d filteredLBForce = Eigen::Vector6d::Zero();
  //   Eigen::Vector6d filteredLFForce = Eigen::Vector6d::Zero();
  //   Eigen::Vector6d filteredRBForce = Eigen::Vector6d::Zero();
  //   Eigen::Vector6d filteredRFForce = Eigen::Vector6d::Zero();

  //   for (const auto& force : LBForceHistory)
  //     filteredLBForce/=LBForceHistory.size();

  //   for (const auto& force : LFForceHistory)
  //     filteredLFForce/=LFForceHistory.size();

  //   for (const auto& force : RBForceHistory)
  //     filteredRBForce/=RBForceHistory.size();

  //   for (const auto& force : RFForceHistory)
  //     filteredRFForce/=RFForceHistory.size();

  //   LBFilteredvec= filteredLBForce.data();
  //   LFFilteredvec= filteredLBForce.data();
  //   RBFilteredvec= filteredLBForce.data();
  //   RFFilteredvec= filteredLBForce.data();


  // }  

  //////////////////////////////////////////////////////////////////////////////////////////
  // Thread to wait for message while not blocking controller
  void dataThread()
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

private:
  bool liveMode_ = true; // by default, live reading
  std::string calibFile_ = "/tmp/force-shoe-calib.yaml";
   
  enum class Mode
  {
    Calibrate,
    Acquire
  };
  Mode mode_ = Mode::Calibrate;
  size_t calibSamples_ = 0;

  bool th_running_ = true;
  std::thread th_;
  std::mutex mutex_;
  std::shared_ptr<Packet> packet_;

  CmtOutputMode mode = CMT_OUTPUTMODE_CALIB | CMT_OUTPUTMODE_AUXILIARY;
  CmtOutputSettings settings = CMT_OUTPUTSETTINGS_AUXILIARYMODE_FORCE | CMT_OUTPUTSETTINGS_TIMESTAMP_SAMPLECNT;
  unsigned long mtCount = 0;
  CmtDeviceId deviceIds_[256];
  std::shared_ptr<xsens::Cmt3> cmt3_;

  // sample counter
  unsigned short sdata_ = NULL;

  // Unloaded voltage measure of force shoes
  Eigen::Vector6d LFUnload, LBUnload, RFUnload, RBUnload = Eigen::Vector6d::Zero();

  // Difference between measured voltage and unloaded
  double LFdiff[6], LBdiff[6], RFdiff[6], RBdiff[6] = {0., 0., 0., 0., 0., 0.};

  // Raw data vectors (G0, G1, G2, G3, G4, G5, G6, ref)
  double LFraw[8], LBraw[8], RFraw[8], RBraw[8] = {0., 0., 0., 0., 0., 0., 0., 0.};

  // Processed F/T vector after calculations: result of ampCalMat[6][6]*diffVoltages[6]
  double LFforcevec[6], LBforcevec[6], RFforcevec[6], RBforcevec[6] = {0., 0., 0., 0., 0., 0.};

  // Amplified calibration matrixes: result of rawCalMat/amplifierGain/Excitation voltage
  double ampCalMatLF[6][6], ampCalMatLB[6][6], ampCalMatRF[6][6], ampCalMatRB[6][6];

  // This is the raw calibration matrix for the force sensor FT15249 (Left Front)
  double rawCalMatLF[6][6] = {
      505.107982172812, 87.11955819031,  1367.21872816217, -21930.7381123915, -1652.27974488926,   22496.7931451793,
     -1108.37239160853, 25287.898316577,   1064.19237363973, -12625.5413902417,  623.98729881893,  -12993.542825778,
      33947.1671344626,  225.671123573245,  32492.4215740645,  231.449297408501,  33043.630266866,  163.85593685011,
      -6.08744074610149, 174.851866946243,  -530.014146023071, -91.4113603666753, 542.792525959471,  -85.3106024240965,
      619.552064828794,   4.1669049520933,   -321.129488202777, 149.553885176189,  -303.852391159778, -159.076992591019,
      14.6504114836628,  -321.66610662891,  18.8907964522682, -313.548857821211, 27.3241444486831, -328.214733354103};

  // This is the raw calibration matrix for the force sensor FT15250 (Left Back)
  double rawCalMatLB[6][6] = {
      606.911455888001,  179.249589261239,  2020.7573456855,  -21887.0052414965,  -2938.82688497076, 21901.0959107632,
      -2872.09191622726, 24954.0099148003,  1981.25741249244,  -12547.0432212112, 1336.39712740197,  -12911.7834064695,
      32235.0884572035,  1927.83631742394,  31839.2776582772,  16788.27955418321,  31458.3738880778,  1685.99760535694,
      -13.7691102600808, 171.076519313952,  -501.946621327464, -114.861383306443, 528.089109363728,  -57.9900102127892,
      582.271739445785,  34.3038116468284,  -314.765577429703, 133.292768127422,  -296.919258796827, -167.33195401529,
      28.5790955076564,   -307.035541947356, 32.3745736038554,  -315.59403475888,  49.4717847643117,  -320.234754744748};

  // This is the raw calibration matrix for the force sensor FT12068 (Right Front)
  double rawCalMatRF[6][6] = {
      343.630637281982,  105.105983793954,  -2953.26625126029, 22314.9264488265,  3260.3934704084,   -21719.9644384895,
      1623.0777459875,   -25552.3321523685, -1349.53867217226, 12844.2106735018,  -2356.21149902445, 12510.2165878903,
      -30142.1049892255, 562.224608578618,  -30329.979677345,  697.861864133106,  -30505.173520053,  336.661667405005,
      4.92731520408949,  -176.535546824105, 480.527538919159,  73.0865549273997,  -511.800932029994, 95.7465164537783,
      -575.310737786879, 14.7864098252503,  306.87524317224,   -161.111324281113, 261.864725287537,  144.440513232962,
      -18.7543249035664, 329.16211996994,   -43.6261968319026, 327.208881044527,  -45.4574714985197, 316.294118203176};

  // This is the raw calibration matrix for the force sensor FT14035 (Right Back)
  double rawCalMatRB[6][6] = {
      -909.765814075373, 49.2280913248435,  576.497629614237,  -25191.3869620449, 886.542005889738,  23543.7979396518,
      -2101.95157658302, 26046.7639260403,  559.39898126517,   -14501.4085513779, 32.7653763475444,  -13456.5142054579,
      32331.9894489576,  1814.74373437844,  31737.8786755207,  861.16789145453,   34068.4517175101,  2259.37787415044,
      -5.14066979214853, 181.879638964666,  -516.518430044674, -115.031396110224, 534.42712637739,   -57.0520829008726,
      612.232383890789,  33.2643570668898,  -314.503852561918, 167.218423612058,  -319.039352381607, -185.640176897279,
      15.4821990529192,  -326.427519836563, -12.5539291346415, -357.7553030025,   24.0083185975606,  -337.24859897945};
};

} // namespace mc_plugin
