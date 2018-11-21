//
// Created by Nikita Kruk on 14.06.18.
//

#include "MultiTargetTracker.hpp"
#include "../ImageProcessing/ImageProcessingEngine.hpp"
#include "KalmanFilter.hpp"

#include <iostream>

MultiTargetTracker::MultiTargetTracker():
    parameter_handler_(),
    targets_(),
    detections_()
{
  std::cout << "multi-target tracking started" << std::endl;
}

MultiTargetTracker::~MultiTargetTracker()
{
  std::cout << "multi-target tracking ended" << std::endl;
}

void MultiTargetTracker::Start()
{
  ImageProcessingEngine image_processing_engine(parameter_handler_);
  KalmanFilter kalman_filter(parameter_handler_, image_processing_engine);

  image_processing_engine.RetrieveBacterialData(parameter_handler_.GetFirstImage(), detections_);
  kalman_filter.InitializeTargets(targets_, detections_);

  for (int i = parameter_handler_.GetFirstImage() + 1; i <= parameter_handler_.GetLastImage(); ++i)
  {
    image_processing_engine.RetrieveBacterialData(i, detections_);
    kalman_filter.PerformEstimation(i, targets_, detections_);
  }
}