#include "NewPlugin.h"

#include <mc_control/GlobalPluginMacros.h>

namespace mc_plugin
{

NewPlugin::~NewPlugin() = default;

void NewPlugin::init(mc_control::MCGlobalController & controller, const mc_rtc::Configuration & config)
{
  mc_rtc::log::info("NewPlugin::init called with configuration:\n{}", config.dump(true, true));
}

void NewPlugin::reset(mc_control::MCGlobalController & controller)
{
  mc_rtc::log::info("NewPlugin::reset called");
}

void NewPlugin::before(mc_control::MCGlobalController &)
{
  mc_rtc::log::info("NewPlugin::before");
}

void NewPlugin::after(mc_control::MCGlobalController & controller)
{
  mc_rtc::log::info("NewPlugin::after");
}

mc_control::GlobalPlugin::GlobalPluginConfiguration NewPlugin::configuration()
{
  mc_control::GlobalPlugin::GlobalPluginConfiguration out;
  out.should_run_before = true;
  out.should_run_after = true;
  out.should_always_run = true;
  return out;
}

} // namespace mc_plugin

EXPORT_MC_RTC_PLUGIN("NewPlugin", mc_plugin::NewPlugin)
