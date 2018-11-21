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

  void CreateKalmanFilterOutputFiles();
  void CloseKalmanFilterOutputFiles();
  void InitializeTargets(std::map<int, Eigen::VectorXd> &targets, const std::vector<Eigen::VectorXd> &detections);
  void PerformEstimation(int image_idx,
                         std::map<int, Eigen::VectorXd> &targets,
                         const std::vector<Eigen::VectorXd> &detections);

 private:

  ParameterHandler &parameter_handler_;
  ImageProcessingEngine &image_processing_engine_;

  std::ofstream kalman_filter_output_file_;
  std::ofstream kalman_filter_matlab_output_file_;

  std::map<int, int> unmatched_;
  int max_prediction_time_;
  int max_target_index_;
  Real costs_order_of_magnitude_;
  Eigen::MatrixXd I_;
  Eigen::MatrixXd A_;
  Eigen::MatrixXd W_;
  Eigen::MatrixXd H_;
  Eigen::MatrixXd Q_;
  Eigen::MatrixXd P_;
  Eigen::MatrixXd K_;

  void ComputePriorEstimate(std::map<int, Eigen::VectorXd> &targets);
  void ComputeKalmanGainMatrix();
  void PerformDataAssociation(const std::map<int, Eigen::VectorXd> &targets,
                              const std::vector<Eigen::VectorXd> &detections,
                              int n_max_dim,
                              std::vector<int> &target_indexes,
                              std::vector<int> &assignments,
                              std::vector<CostInt> &costs);
  CostInt InitializeCostMatrix(const std::map<int, Eigen::VectorXd> &targets,
                               const std::vector<Eigen::VectorXd> &detections,
                               std::vector<std::vector<CostInt>> &cost_matrix,
                               std::vector<int> &target_indexes);
  void UnassignUnrealisticTargets(const std::map<int, Eigen::VectorXd> &targets,
                                  const std::vector<Eigen::VectorXd> &detections,
                                  int n_max_dim,
                                  std::vector<int> &assignments,
                                  std::vector<CostInt> &costs,
                                  const std::vector<int> &target_indexes);
  void ComputePosteriorEstimate(std::map<int, Eigen::VectorXd> &targets,
                                const std::vector<Eigen::VectorXd> &detections,
                                const std::vector<int> &assignments,
                                const std::vector<int> &target_indexes);
  void MarkLostTargetsAsUnmatched(std::map<int, Eigen::VectorXd> &targets,
                                  const std::vector<int> &assignments,
                                  const std::vector<int> &target_indexes);
  void MarkAllTargetsAsUnmatched(std::map<int, Eigen::VectorXd> &targets);
  void RemoveRecapturedTargetsFromUnmatched(std::map<int, Eigen::VectorXd> &targets,
                                            const std::vector<int> &assignments,
                                            const std::vector<int> &target_indexes);
  void AddNewTargets(int image_idx,
                     std::map<int, Eigen::VectorXd> &targets,
                     const std::vector<Eigen::VectorXd> &detections,
                     const std::vector<int> &assignments);
  void DeleteLongLostTargets(std::map<int, Eigen::VectorXd> &targets);

  void SaveTargets(std::ofstream &file, int image_idx, const std::map<int, Eigen::VectorXd> &targets);
  void SaveTargetsMatlab(std::ofstream &file, int image_idx, const std::map<int, Eigen::VectorXd> &targets);
  void SaveImages(int image_idx, const std::map<int, Eigen::VectorXd> &targets);

};

#endif //BEEMULTITARGETTRACKING_KALMANFILTER_HPP
