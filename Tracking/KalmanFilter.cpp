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
    cost_epsilon_(1.0e-0), // TODO: check whether needed
    strikes_(),
    max_target_index_(0)
{
  std::string kalman_filter_output_file_name =
      parameter_handler_.GetInputFolder() + parameter_handler.GetDataAnalysisSubfolder()
          + parameter_handler.GetKalmanFilterOutputFileName();
  kalman_filter_output_file_.open(kalman_filter_output_file_name, std::ios::out | std::ios::trunc);
  assert(kalman_filter_output_file_.is_open());

  std::string kalman_filter_matlab_output_file_name =
      parameter_handler_.GetInputFolder() + parameter_handler_.GetDataAnalysisSubfolder()
          + parameter_handler_.GetKalmanFilterMatlabOutputFileName();
  kalman_filter_matlab_output_file_.open(kalman_filter_matlab_output_file_name, std::ios::out | std::ios::trunc);
  assert(kalman_filter_matlab_output_file_.is_open());
}

KalmanFilter::~KalmanFilter()
{
  kalman_filter_output_file_.close();
  kalman_filter_matlab_output_file_.close();
}

void KalmanFilter::InitializeTargets(std::map<int, Eigen::VectorXf> &targets,
                                     const std::vector<Eigen::VectorXf> &detections)
{
  targets.clear();

  int last_index = 0;
  Eigen::VectorXf new_target(kNumOfExtractedFeatures);
  for (int b = 0; b < detections.size(); ++b)
  {
    if (targets.empty())
    {
      last_index = -1;
    } else
    {
      last_index = std::prev(targets.end())->first;
//			last_index = targets.rbegin()->first;
    }
    new_target = detections[b];
    targets[++last_index] = new_target;
  }
  max_target_index_ = last_index;

  SaveTargets(kalman_filter_output_file_, parameter_handler_.GetFirstImage(), targets);
  SaveTargetsMatlab(kalman_filter_matlab_output_file_, parameter_handler_.GetFirstImage(), targets);
  SaveImages(parameter_handler_.GetFirstImage(), targets);
}

void KalmanFilter::PerformEstimation(int image_idx,
                                     std::map<int, Eigen::VectorXf> &targets,
                                     const std::vector<Eigen::VectorXf> &detections)
{
  std::cout << "kalman filter: image#" << image_idx << std::endl;

  int n_max_dim = 0; // max size between targets and detections
  CostInt max_cost = 0;
  Real dt = 1;// in ms

  Eigen::VectorXf z_i(kNumOfDetectionVars);
  Eigen::VectorXf x_i_estimate(kNumOfStateVars);
  Eigen::MatrixXf I = Eigen::MatrixXf::Identity(kNumOfStateVars, kNumOfStateVars);
  Eigen::MatrixXf A = Eigen::MatrixXf::Zero(kNumOfStateVars, kNumOfStateVars);
  Eigen::MatrixXf W = Eigen::MatrixXf::Zero(kNumOfStateVars, kNumOfStateVars);
  Eigen::MatrixXf H = Eigen::MatrixXf::Zero(kNumOfDetectionVars, kNumOfStateVars);
  Eigen::MatrixXf Q = Eigen::MatrixXf::Zero(kNumOfDetectionVars, kNumOfDetectionVars);
  Eigen::MatrixXf P_estimate = Eigen::MatrixXf::Zero(kNumOfStateVars, kNumOfStateVars);
  Eigen::MatrixXf K = Eigen::MatrixXf::Zero(kNumOfStateVars, kNumOfStateVars);

  A(0, 0) = A(1, 1) = A(2, 2) = A(3, 3) = 1.0;
  A(0, 2) = A(1, 3) = dt;
  H(0, 0) = H(1, 1) = 1.0;

  W(0, 0) = W(1, 1) = dt * dt * dt * dt / 4.0f;
  W(2, 2) = W(3, 3) = dt * dt;
  W(0, 2) = W(1, 3) = W(2, 0) = W(3, 1) = dt * dt * dt / 2.0f;
  Q(0, 0) = Q(1, 1) = dt;
  P_estimate = W;

  // PRIOR ESTIMATE
  for (std::map<int, Eigen::VectorXf>::iterator it = targets.begin(); it != targets.end(); ++it)
  {
    x_i_estimate = (it->second).head(kNumOfStateVars);
    x_i_estimate = A * x_i_estimate;
    (it->second).head(kNumOfStateVars) = x_i_estimate;
  }
  P_estimate = A * P_estimate * A.transpose() + W;
  K = P_estimate * H.transpose() * (H * P_estimate * H.transpose() + Q).inverse();

  if (detections.size() > 0)
  {
    // DATA ASSOCIATION
    std::vector<int> target_indexes;
    n_max_dim = (int) std::max(targets.size(), detections.size());
    std::vector<std::vector<CostInt>> cost_matrix(n_max_dim, std::vector<CostInt>(n_max_dim, 0));
    std::vector<int> assignments(n_max_dim, -1);
    std::vector<CostInt> costs(n_max_dim);
    max_cost = InitializeCostMatrix(targets, detections, cost_matrix, target_indexes);

    HungarianAlgorithm hungarian_algorithm(n_max_dim, cost_matrix);
    hungarian_algorithm.Start(assignments, costs);
    std::for_each(costs.begin(),
                  costs.end(),
                  [&](CostInt &c)
                  {
                    c = CostInt((max_cost - c) * cost_epsilon_);
                  }); // TODO: Is conversion correct both ways?

// if a cost is too high or if the assignment is into an imaginary detection
    for (int i = 0; i < targets.size(); ++i)
    {
      if (costs[i] > parameter_handler_.GetDataAssociationCost() || assignments[i] >= detections.size()) // in pixels
      {
        assignments[i] = -1;
      }
    }
    // if the assignment is from an imaginary target
    for (int i = (int) targets.size(); i < n_max_dim; ++i)
    {
      assignments[i] = -1;
    }

    // POSTERIOR ESTIMATE OF DETECTED TARGETS
    for (int i = 0; i < targets.size(); ++i)
    {
      if (assignments[i] != -1)
      {
        x_i_estimate = targets[target_indexes[i]].head(kNumOfStateVars);
        z_i = detections[assignments[i]].head(2);
        x_i_estimate = x_i_estimate + K * (z_i - H * x_i_estimate);
        targets[target_indexes[i]].head(kNumOfStateVars) = x_i_estimate;

        if (strikes_.find(target_indexes[i]) != strikes_.end())
        {
          strikes_.erase(target_indexes[i]); // stop suspecting a target if it has been recovered
        }
      }
    }
    P_estimate = (I - K * H) * P_estimate;

// ADDITION OF NEW TRAJECTORIES
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

    // consider detections, left after segmentation, as new targets
    for (int i = 0; i < indexes_to_unassigned_detections.size(); ++i)
    {
      targets[max_target_index_ + 1] = detections[indexes_to_unassigned_detections[i]];
      ++max_target_index_;
    }

    // search for targets that have been lost
    // idx.size() == number of targets at the beginning of the iteration
    for (int i = 0; i < target_indexes.size(); ++i)
    {
      if (assignments[i] == -1)
      {
        if (strikes_.find(target_indexes[i]) != strikes_.end())
        {
          ++strikes_[target_indexes[i]];
        } else
        {
          strikes_[target_indexes[i]] = 1;
        }
      }
    }
  } else
  {
    // all the targets have been lost
    for (std::map<int, Eigen::VectorXf>::const_iterator it = targets.begin(); it != targets.end(); ++it)
    {
      if (strikes_.find(it->first) != strikes_.end())
      {
        ++strikes_[it->first];
      } else
      {
        strikes_[it->first] = 1;
      }
    }
  }

  // if the target has been lost for too long -> remove it
  for (std::map<int, int>::iterator it = strikes_.begin(); it != strikes_.end();)
  {
    if (it->second > 1)
    {
      targets.erase(it->first);
      it = strikes_.erase(it);
    } else
    {
      ++it;
    }
  }

  SaveTargets(kalman_filter_output_file_, image_idx, targets);
  SaveTargetsMatlab(kalman_filter_matlab_output_file_, image_idx, targets);
  SaveImages(image_idx, targets);

  std::cout << "number of overall targets taken part: " << max_target_index_ + 1 << "; number of current targets: "
            << targets.size() << std::endl;
}

void KalmanFilter::SaveTargets(std::ofstream &file, int image_idx, const std::map<int, Eigen::VectorXf> &targets)
{
  Eigen::VectorXf x_i;
  file << image_idx << " " << targets.size() << " ";
  for (std::map<int, Eigen::VectorXf>::const_iterator it = targets.begin(); it != targets.end(); ++it)
  {
    x_i = it->second;
    file << it->first << " " << x_i(0) << " " << x_i(1) << " " << x_i(2) << " " << x_i(3) << " "
         << x_i(4) << " " << x_i(5) << " ";
  }
  file << std::endl;
}

void KalmanFilter::SaveTargetsMatlab(std::ofstream &file, int image_idx, const std::map<int, Eigen::VectorXf> &targets)
{
  Eigen::VectorXf x_i;
  for (std::map<int, Eigen::VectorXf>::const_iterator it = targets.begin(); it != targets.end(); ++it)
  {
    x_i = it->second;
    file << image_idx << " " << it->first << " " << x_i(0) << " " << x_i(1) << " " << x_i(2) << " " << x_i(3) << " "
         << x_i(4) << " " << x_i(5) << std::endl;
  }
}

void KalmanFilter::SaveImages(int image_idx, const std::map<int, Eigen::VectorXf> &targets)
{
  cv::Mat image;
  image = image_processing_engine_.GetSourceImage();

  Eigen::VectorXf x_i;
  cv::Point2f center;
  cv::Point2f velocity;
  cv::Point2f rect_points[4];
  cv::Scalar color(0, 0, 255);
  Real width = 0.0f, height = 0.0f;
  Real speed = 0.0;

  for (std::map<int, Eigen::VectorXf>::const_iterator it = targets.begin(); it != targets.end(); ++it)
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

CostInt KalmanFilter::InitializeCostMatrix(const std::map<int, Eigen::VectorXf> &targets,
                                           const std::vector<Eigen::VectorXf> &detections,
                                           std::vector<std::vector<CostInt>> &cost_matrix,
                                           std::vector<int> &target_indexes)
{
  target_indexes.clear();

  Eigen::VectorXf target(kNumOfExtractedFeatures);
  Eigen::VectorXf detection(kNumOfExtractedFeatures);
  Real cost = 0.0;
  Real max_cost = 0;
  int i = 0;
  Real d_x = 0.0, d_y = 0.0;
  Real dist = 0.0;
  Real max_dist = std::sqrtf(parameter_handler_.GetSubimageXSize() * parameter_handler_.GetSubimageXSize()
                                 + parameter_handler_.GetSubimageYSize() * parameter_handler_.GetSubimageYSize());
  for (std::map<int, Eigen::VectorXf>::const_iterator it = targets.begin(); it != targets.end(); ++it, ++i)
  {
    target_indexes.push_back(it->first);
    target = it->second;

    for (int j = 0; j < detections.size(); ++j)
    {
      detection = detections[j];

      d_x = (target(0) - detection(0));
      d_y = (target(1) - detection(1));

      // put only close assignment costs in the cost matrix
      dist = std::sqrt(d_x * d_x + d_y * d_y);
      if (dist <= parameter_handler_.GetDataAssociationCost())
      {
        cost = dist; // Euclidean norm from a target to a detection
      } else
      {
        cost = max_dist;
      }

      cost /= cost_epsilon_;
      cost_matrix[i][j] = CostInt(cost);
      if (max_cost < cost)
      {
        max_cost = cost;
      }
    }
  }

  // turn min cost problem into max cost problem
  for (int i = 0; i < targets.size(); ++i)
  {
    for (int j = 0; j < detections.size(); ++j)
    {
      cost_matrix[i][j] = int(max_cost) - cost_matrix[i][j];
    }
  }

  return CostInt(max_cost);
}
