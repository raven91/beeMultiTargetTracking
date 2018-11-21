//
// Created by Nikita Kruk on 15.06.18.
//

#include "KalmanFilter.hpp"
#include "HungarianAlgorithm.hpp"

#include <iostream>
#include <sstream>
#include <algorithm> // std::copy, std::max, std::set_difference, std::for_each, std::sort
#include <iterator>  // std::back_inserter, std::prev
#include <cmath>
#include <set>
#include <numeric>   // std::iota
#include <random>
#include <iomanip>   // std::setfill, std::setw

KalmanFilter::KalmanFilter(ParameterHandler &parameter_handler, ImageProcessingEngine &image_processing_engine) :
    parameter_handler_(parameter_handler),
    image_processing_engine_(image_processing_engine),
    costs_order_of_magnitude_(1000000.0),
    unmatched_(),
    max_prediction_time_(1),
    max_target_index_(0)
{
  I_ = Eigen::MatrixXd::Identity(kNumOfStateVars, kNumOfStateVars);
  A_ = Eigen::MatrixXd::Zero(kNumOfStateVars, kNumOfStateVars);
  W_ = Eigen::MatrixXd::Zero(kNumOfStateVars, kNumOfStateVars);
  H_ = Eigen::MatrixXd::Zero(kNumOfDetectionVars, kNumOfStateVars);
  Q_ = Eigen::MatrixXd::Zero(kNumOfDetectionVars, kNumOfDetectionVars);
  P_ = Eigen::MatrixXd::Zero(kNumOfStateVars, kNumOfStateVars);
  K_ = Eigen::MatrixXd::Zero(kNumOfStateVars, kNumOfStateVars);

  Real dt = 1;// in ms==image
  A_(0, 0) = A_(1, 1) = A_(2, 2) = A_(3, 3) = 1.0;
  A_(0, 2) = A_(1, 3) = dt;
  H_(0, 0) = H_(1, 1) = 1.0;

  W_(0, 0) = W_(1, 1) = dt * dt * dt * dt / 4.0;
  W_(2, 2) = W_(3, 3) = dt * dt;
  W_(0, 2) = W_(1, 3) = W_(2, 0) = W_(3, 1) = dt * dt * dt / 2.0;
  W_ *= 2.5 * 2.5; // multiply by variance in acceleration
  Q_(0, 0) = Q_(1, 1) = 2.5 * 2.5;//dt

  P_ = W_;
}

KalmanFilter::~KalmanFilter()
{
  CloseKalmanFilterOutputFiles();
}

void KalmanFilter::CreateKalmanFilterOutputFiles()
{
  std::string kalman_filter_output_file_name =
      parameter_handler_.GetInputFolder() + parameter_handler_.GetDataAnalysisSubfolder()
          + parameter_handler_.GetKalmanFilterOutputFileName();
  kalman_filter_output_file_.open(kalman_filter_output_file_name, std::ios::out | std::ios::trunc);
  assert(kalman_filter_output_file_.is_open());

  std::string kalman_filter_matlab_output_file_name =
      parameter_handler_.GetInputFolder() + parameter_handler_.GetDataAnalysisSubfolder()
          + parameter_handler_.GetKalmanFilterMatlabOutputFileName();
  kalman_filter_matlab_output_file_.open(kalman_filter_matlab_output_file_name, std::ios::out | std::ios::trunc);
  assert(kalman_filter_matlab_output_file_.is_open());
}

void KalmanFilter::CloseKalmanFilterOutputFiles()
{
  if (kalman_filter_output_file_.is_open())
  {
    kalman_filter_output_file_.close();
  }
  if (kalman_filter_matlab_output_file_.is_open())
  {
    kalman_filter_matlab_output_file_.close();
  }
}

void KalmanFilter::InitializeTargets(std::map<int, Eigen::VectorXd> &targets,
                                     const std::vector<Eigen::VectorXd> &detections)
{
  targets.clear();

  int last_index = 0;
  Eigen::VectorXd new_target(kNumOfExtractedFeatures);
  for (const Eigen::VectorXd &detection : detections)
  {
    if (targets.empty())
    {
      last_index = -1;
    } else
    {
      last_index = std::prev(targets.end())->first;
//			last_index = targets.rbegin()->first;
    }
    new_target = detection;
    targets[++last_index] = new_target;
  }
  max_target_index_ = last_index;

  SaveTargets(kalman_filter_output_file_, parameter_handler_.GetFirstImage(), targets);
  SaveTargetsMatlab(kalman_filter_matlab_output_file_, parameter_handler_.GetFirstImage(), targets);
  SaveImages(parameter_handler_.GetFirstImage(), targets);
}

void KalmanFilter::PerformEstimation(int image_idx,
                                     std::map<int, Eigen::VectorXd> &targets,
                                     const std::vector<Eigen::VectorXd> &detections)
{
  std::cout << "kalman filter: image#" << image_idx << std::endl;

  int n_max_dim = 0; // max size between targets and detections

  ComputePriorEstimate(targets);
  ComputeKalmanGainMatrix();

  if (!(detections.empty()))
  {
    n_max_dim = (int) std::max(targets.size(), detections.size());
    std::vector<int> target_indexes;
    std::vector<int> assignments((unsigned long) n_max_dim, -1);
    std::vector<CostInt> costs((unsigned long) n_max_dim);

    PerformDataAssociation(targets, detections, n_max_dim, target_indexes, assignments, costs);
    UnassignUnrealisticTargets(targets, detections, n_max_dim, assignments, costs, target_indexes);
    ComputePosteriorEstimate(targets, detections, assignments, target_indexes);
    RemoveRecapturedTargetsFromUnmatched(targets, assignments, target_indexes);
    MarkLostTargetsAsUnmatched(targets, assignments, target_indexes);
    AddNewTargets(image_idx, targets, detections, assignments);
  } else
  {
    std::cout << "error in detector: no detections found" << std::endl;
    MarkAllTargetsAsUnmatched(targets);
  }
  DeleteLongLostTargets(targets);

  SaveTargets(kalman_filter_output_file_, image_idx, targets);
  SaveTargetsMatlab(kalman_filter_matlab_output_file_, image_idx, targets);
  SaveImages(image_idx, targets);

  std::cout << "number of overall targets taken part: " << max_target_index_ + 1 << "; number of current targets: "
            << targets.size() << std::endl;
}

void KalmanFilter::ComputePriorEstimate(std::map<int, Eigen::VectorXd> &targets)
{
  Eigen::VectorXd x_i_estimate(kNumOfStateVars);
  for (std::pair<const int, Eigen::VectorXd> &target : targets)
  {
    x_i_estimate = (target.second).head(kNumOfStateVars);
    x_i_estimate = A_ * x_i_estimate;
    (target.second).head(kNumOfStateVars) = x_i_estimate;
  }
  P_ = A_ * P_ * A_.transpose() + W_;
}

void KalmanFilter::ComputeKalmanGainMatrix()
{
  K_ = P_ * H_.transpose() * (H_ * P_ * H_.transpose() + Q_).inverse();
}

void KalmanFilter::PerformDataAssociation(const std::map<int, Eigen::VectorXd> &targets,
                                          const std::vector<Eigen::VectorXd> &detections,
                                          int n_max_dim,
                                          std::vector<int> &target_indexes,
                                          std::vector<int> &assignments,
                                          std::vector<CostInt> &costs)
{
  std::vector<std::vector<CostInt>>
      cost_matrix((unsigned long) n_max_dim, std::vector<CostInt>((unsigned long) n_max_dim, 0));
  CostInt max_cost = InitializeCostMatrix(targets, detections, cost_matrix, target_indexes);
  HungarianAlgorithm hungarian_algorithm(n_max_dim, cost_matrix);
  hungarian_algorithm.Start(assignments, costs);
  std::for_each(costs.begin(),
                costs.end(),
                [&](CostInt &c)
                {
                  c = CostInt((max_cost - c) / costs_order_of_magnitude_);
                });
}

CostInt KalmanFilter::InitializeCostMatrix(const std::map<int, Eigen::VectorXd> &targets,
                                           const std::vector<Eigen::VectorXd> &detections,
                                           std::vector<std::vector<CostInt>> &cost_matrix,
                                           std::vector<int> &target_indexes)
{
  target_indexes.clear();

  Eigen::VectorXd target(kNumOfExtractedFeatures);
  Eigen::VectorXd detection(kNumOfExtractedFeatures);
  Real cost = 0.0;
  Real max_cost = 0;
  Real dist = 0.0;
  Real max_dist = std::sqrt(parameter_handler_.GetSubimageXSize() * parameter_handler_.GetSubimageXSize()
                                + parameter_handler_.GetSubimageYSize() * parameter_handler_.GetSubimageYSize());
  // the trajectories are not guaranteed to have successive labels
  // row and column indexes are introduced to account for that
  int row = 0;
  for (std::map<int, Eigen::VectorXd>::const_iterator it = targets.begin(); it != targets.end(); ++it, ++row)
  {
    target_indexes.push_back(it->first);
    target = it->second;

    for (int column = 0; column < detections.size(); ++column)
    {
      detection = detections[column];
      Eigen::Vector2d dist_vec = target.head(2) - detection.head(2);
      dist = dist_vec.norm();
      // put only close assignment costs in the cost matrix
      if (dist > parameter_handler_.GetDataAssociationCost())
      {
        dist = max_dist;
      }

      cost = dist / max_dist;
      cost_matrix[row][column] = CostInt(cost * costs_order_of_magnitude_);
      if (max_cost < cost)
      {
        max_cost = cost;
      }
    } // column
  } // it

  // turn min cost problem into max cost problem
  for (int row = 0; row < targets.size(); ++row)
  {
    for (int column = 0; column < detections.size(); ++column)
    {
      cost_matrix[row][column] = CostInt(max_cost * costs_order_of_magnitude_) - cost_matrix[row][column];
    } // column
  } // row
  // the complementary values are left zero as needed for the max cost problem

  return CostInt(max_cost * costs_order_of_magnitude_);
}

void KalmanFilter::UnassignUnrealisticTargets(const std::map<int, Eigen::VectorXd> &targets,
                                              const std::vector<Eigen::VectorXd> &detections,
                                              int n_max_dim,
                                              std::vector<int> &assignments,
                                              std::vector<CostInt> &costs,
                                              const std::vector<int> &target_indexes)
{
  for (int i = 0; i < targets.size(); ++i)
  {
    if (assignments[i] >= detections.size()) // if the assignment is into an imaginary detection
    {
      assignments[i] = -1;
    } else // if a cost is too high
    {
      Eigen::VectorXd target = targets.at(target_indexes[i]);
      Eigen::VectorXd detection = detections[assignments[i]];
      Eigen::Vector2d dist_vec = target.head(2) - detection.head(2);
      Real dist = dist_vec.norm();
      if (dist > parameter_handler_.GetDataAssociationCost())
      {
        assignments[i] = -1;
      }
    }
  } // i
  // if the assignment is from an imaginary target
  for (int i = (int) targets.size(); i < n_max_dim; ++i)
  {
    assignments[i] = -1;
  }
}

void KalmanFilter::ComputePosteriorEstimate(std::map<int, Eigen::VectorXd> &targets,
                                            const std::vector<Eigen::VectorXd> &detections,
                                            const std::vector<int> &assignments,
                                            const std::vector<int> &target_indexes)
{
  Eigen::VectorXd z_i(kNumOfDetectionVars);
  Eigen::VectorXd x_i_estimate(kNumOfStateVars);
  for (int i = 0; i < targets.size(); ++i)
  {
    if (assignments[i] != -1)
    {
      x_i_estimate = targets[target_indexes[i]].head(kNumOfStateVars);
      z_i = detections[assignments[i]].head(2);
      x_i_estimate = x_i_estimate + K_ * (z_i - H_ * x_i_estimate);
      targets[target_indexes[i]].head(kNumOfStateVars) = x_i_estimate;

      targets[target_indexes[i]][4] = detections[assignments[i]][4];
      targets[target_indexes[i]][5] = detections[assignments[i]][5];
    }
  } // i
  Eigen::MatrixXd I = Eigen::MatrixXd::Identity(kNumOfStateVars, kNumOfStateVars);
  P_ = (I - K_ * H_) * P_;
}

/**
 * Must be called before AddNewTargets, which modifies the targets.size()
 * @param targets
 * @param assignments
 * @param target_indexes
 */
void KalmanFilter::MarkLostTargetsAsUnmatched(std::map<int, Eigen::VectorXd> &targets,
                                              const std::vector<int> &assignments,
                                              const std::vector<int> &target_indexes)
{
  // consider only the initial targets without appended undetected ones
  // and without appended artificial elements
  for (int i = 0; i < targets.size(); ++i)
  {
    if (assignments[i] == -1)
    {
      if (unmatched_.find(target_indexes[i]) != unmatched_.end())
      {
        ++unmatched_[target_indexes[i]];
      } else
      {
        unmatched_[target_indexes[i]] = 1;
      }
    }
  } // i
}

void KalmanFilter::RemoveRecapturedTargetsFromUnmatched(std::map<int, Eigen::VectorXd> &targets,
                                                        const std::vector<int> &assignments,
                                                        const std::vector<int> &target_indexes)
{
  for (int i = 0; i < targets.size(); ++i)
  {
    if (assignments[i] != -1)
    {
      if (unmatched_.find(target_indexes[i]) != unmatched_.end())
      {
        unmatched_.erase(target_indexes[i]); // stop suspecting a target if it has been recovered
      }
    }
  } // i
}

void KalmanFilter::MarkAllTargetsAsUnmatched(std::map<int, Eigen::VectorXd> &targets)
{
  // all the targets have been lost
  for (std::map<int, Eigen::VectorXd>::const_iterator it = targets.begin(); it != targets.end(); ++it)
  {
    if (unmatched_.find(it->first) != unmatched_.end())
    {
      ++unmatched_[it->first];
    } else
    {
      unmatched_[it->first] = 1;
    }
  } // it
}

void KalmanFilter::AddNewTargets(int image_idx,
                                 std::map<int, Eigen::VectorXd> &targets,
                                 const std::vector<Eigen::VectorXd> &detections,
                                 const std::vector<int> &assignments)
{
  std::vector<int> all_detection_indexes(detections.size());
  std::iota(all_detection_indexes.begin(),
            all_detection_indexes.end(),
            0); // construct detection indexes from 0 through d.size()-1
  std::vector<int> sorted_assignments(assignments.begin(), assignments.end());
  std::sort(sorted_assignments.begin(), sorted_assignments.end());
  std::vector<int> indexes_to_unassigned_detections;
  std::set_difference(all_detection_indexes.begin(),
                      all_detection_indexes.end(),
                      sorted_assignments.begin(),
                      sorted_assignments.end(),
                      std::back_inserter(indexes_to_unassigned_detections)); // set_difference requires pre-sorted containers
  for (int i = 0; i < indexes_to_unassigned_detections.size(); ++i)
  {
    Eigen::VectorXd new_detection = detections[indexes_to_unassigned_detections[i]];
    // if a new detection is not in the middle of the scene, then we add it as a new target
    targets[max_target_index_ + 1] = new_detection;
    ++max_target_index_;
  } // i
}

void KalmanFilter::DeleteLongLostTargets(std::map<int, Eigen::VectorXd> &targets)
{
  // if the target has been lost for too long -> remove it
  for (std::map<int, int>::iterator it = unmatched_.begin(); it != unmatched_.end();)
  {
    if (it->second > max_prediction_time_)
    {
      targets.erase(it->first);
      it = unmatched_.erase(it);
    } else
    {
      ++it;
    }
  } // it
}

void KalmanFilter::SaveTargets(std::ofstream &file, int image_idx, const std::map<int, Eigen::VectorXd> &targets)
{
  Eigen::VectorXd x_i;
  file << image_idx << " " << targets.size() << " ";
  for (std::map<int, Eigen::VectorXd>::const_iterator it = targets.begin(); it != targets.end(); ++it)
  {
    x_i = it->second;
    file << it->first << " " << x_i(0) << " " << x_i(1) << " " << x_i(2) << " " << x_i(3) << " "
         << x_i(4) << " " << x_i(5) << " ";
  }
  file << std::endl;
}

void KalmanFilter::SaveTargetsMatlab(std::ofstream &file, int image_idx, const std::map<int, Eigen::VectorXd> &targets)
{
  Eigen::VectorXd x_i;
  for (std::map<int, Eigen::VectorXd>::const_iterator it = targets.begin(); it != targets.end(); ++it)
  {
    x_i = it->second;
    file << image_idx << " " << it->first << " " << x_i(0) << " " << x_i(1) << " " << x_i(2) << " " << x_i(3) << " "
         << x_i(4) << " " << x_i(5) << std::endl;
  }
}

void KalmanFilter::SaveImages(int image_idx, const std::map<int, Eigen::VectorXd> &targets)
{
  cv::Mat image;
  image = image_processing_engine_.GetSourceImage();

  Eigen::VectorXd x_i;
  cv::Point2f center;
  cv::Point2f velocity;
  cv::Point2f rect_points[4];
  cv::Scalar color(0, 0, 255);
  Real width = 0.0f, height = 0.0f;
  Real speed = 0.0;

  for (std::map<int, Eigen::VectorXd>::const_iterator it = targets.begin(); it != targets.end(); ++it)
  {
    x_i = it->second;
    center = cv::Point2f(x_i(0), x_i(1));
    velocity = cv::Point2f(x_i(2), x_i(3));
    speed = cv::norm(center);
    velocity /= speed;
    width = 16.0;//x_i(4);
    height = 24.0;//x_i(5);

//    rect_points[0] = cv::Point2f(center.x + velocity.x * height / 2, center.y + velocity.y * width / 2);
//    rect_points[3] = cv::Point2f(center.x - velocity.x * height / 2, center.y + velocity.y * width / 2);
//    rect_points[2] = cv::Point2f(center.x - velocity.x * height / 2, center.y - velocity.y * width / 2);
//    rect_points[1] = cv::Point2f(center.x + velocity.x * height / 2, center.y - velocity.y * width / 2);
//    for (int j = 0; j < 4; ++j)
//    {
//      cv::line(image, rect_points[j], rect_points[(j + 1) % 4], color, 2, 8);
//    }
//    cv::circle(image, center, 3, color, -1, 8);
    cv::putText(image, std::to_string(it->first), center, cv::FONT_HERSHEY_DUPLEX, 0.6, color);
//    cv::line(image,
//             center,
//             center + velocity,
//             color);
  }

  std::ostringstream output_image_name_buf;
  output_image_name_buf << parameter_handler_.GetInputFolder() << parameter_handler_.GetKalmanFilterSubfolder()
                        << parameter_handler_.GetFileName0() << std::setfill('0') << std::setw(9) << image_idx
                        << parameter_handler_.GetFileName1();
  std::string output_image_name = output_image_name_buf.str();
  cv::imwrite(output_image_name, image);
}
