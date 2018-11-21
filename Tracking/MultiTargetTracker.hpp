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

  void Start();

 private:

  ParameterHandler parameter_handler_;
  std::map<int, Eigen::VectorXf> targets_;    // i -> x_i y_i v_x_i v_y_i
  std::vector<Eigen::VectorXf> detections_;   // observations

};

#endif //BEEMULTITARGETTRACKING_MULTITARGETTRACKER_HPP
