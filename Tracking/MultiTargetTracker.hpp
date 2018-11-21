//
// Created by Nikita Kruk on 14.06.18.
//

#ifndef BEEMULTITARGETTRACKING_MULTITARGETTRACKER_HPP
#define BEEMULTITARGETTRACKING_MULTITARGETTRACKER_HPP

#include "../Definitions.hpp"
#include "../Parameters/ParameterHandler.hpp"

#include <vector>
#include <map>

#include <eigen3/Eigen/Dense>

class MultiTargetTracker
{
 public:

  MultiTargetTracker();
  ~MultiTargetTracker();

  void PerformTrackingForOneExperiment(const std::string &configuration_file_name);

 private:

  std::map<int, Eigen::VectorXd> targets_;    // i -> x_i y_i v_x_i v_y_i width_i height_i
  std::vector<Eigen::VectorXd> detections_;   // observations

};

#endif //BEEMULTITARGETTRACKING_MULTITARGETTRACKER_HPP
