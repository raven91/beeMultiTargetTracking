//
// Created by Nikita Kruk on 15.06.18.
//

#ifndef BEEMULTITARGETTRACKING_KALMANFILTER_HPP
#define BEEMULTITARGETTRACKING_KALMANFILTER_HPP

#include "../Definitions.hpp"
#include "../Parameters/ParameterHandler.hpp"
#include "../ImageProcessing/ImageProcessingEngine.hpp"

#include <vector>
#include <fstream>
#include <map>

#include <eigen3/Eigen/Dense>

class KalmanFilter
{
 public:

  explicit KalmanFilter(ParameterHandler &parameter_handler, ImageProcessingEngine &image_processing_engine);
  ~KalmanFilter();

  void InitializeTargets(std::map<int, Eigen::VectorXf> &targets, const std::vector<Eigen::VectorXf> &detections);
  void PerformEstimation(int image_idx,
                         std::map<int, Eigen::VectorXf> &targets,
                         const std::vector<Eigen::VectorXf> &detections);

 private:

  ParameterHandler &parameter_handler_;
  ImageProcessingEngine &image_processing_engine_;
  std::ofstream kalman_filter_output_file_;
  std::ofstream kalman_filter_matlab_output_file_;
  std::map<int, int> strikes_;
  int max_target_index_;
  Real cost_epsilon_;

  inline Real WrappingModulo(Real numerator, Real denominator)
  {
    return numerator - denominator * std::floorf(numerator / denominator);
  }

  void SaveTargets(std::ofstream &file, int image_idx, const std::map<int, Eigen::VectorXf> &targets);
  void SaveTargetsMatlab(std::ofstream &file, int image_idx, const std::map<int, Eigen::VectorXf> &targets);
  void SaveImages(int image_idx, const std::map<int, Eigen::VectorXf> &targets);

  CostInt InitializeCostMatrix(const std::map<int, Eigen::VectorXf> &targets,
                               const std::vector<Eigen::VectorXf> &detections,
                               std::vector<std::vector<CostInt>> &cost_matrix,
                               std::vector<int> &target_indexes);

};

#endif //BEEMULTITARGETTRACKING_KALMANFILTER_HPP
