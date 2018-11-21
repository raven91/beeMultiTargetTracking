#include "Tracking/MultiTargetTracker.hpp"

int main()
{
  MultiTargetTracker multitarget_tracker;
  std::string configuration_file_name("/Users/nikita/CLionProjects/beeMultiTargetTracking/Config.cfg");
  multitarget_tracker.PerformTrackingForOneExperiment(configuration_file_name);

  return 0;
}