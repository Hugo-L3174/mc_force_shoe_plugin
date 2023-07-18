#include "ForceShoePlugin.h"

#include <mc_control/GlobalPluginMacros.h>

namespace mc_plugin
{

ForceShoePlugin::~ForceShoePlugin() = default;

void ForceShoePlugin::init(mc_control::MCGlobalController & controller, const mc_rtc::Configuration & config)
{
  mc_rtc::log::info("ForceShoePlugin::init called with configuration:\n{}", config.dump(true, true));
}

void ForceShoePlugin::reset(mc_control::MCGlobalController & controller)
{
  mc_rtc::log::info("ForceShoePlugin::reset called");
}

void ForceShoePlugin::before(mc_control::MCGlobalController &) 
{
  mc_rtc::log::info("ForceShoePlugin::before");
}

void ForceShoePlugin::after(mc_control::MCGlobalController & controller)  
{
  mc_rtc::log::info("ForceShoePlugin::after");
}

mc_control::GlobalPlugin::GlobalPluginConfiguration ForceShoePlugin::configuration()
{
  mc_control::GlobalPlugin::GlobalPluginConfiguration out;
  out.should_run_before = true;
  out.should_run_after = true;
  out.should_always_run = true;
  return out;
}

} // namespace mc_plugin

EXPORT_MC_RTC_PLUGIN("ForceShoePlugin", mc_plugin::ForceShoePlugin)
