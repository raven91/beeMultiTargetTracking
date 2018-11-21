//
// Created by Nikita Kruk on 14.06.18.
//

#include "MultiTargetTracker.hpp"
#include "../ImageProcessing/ImageProcessingEngine.hpp"
#include "KalmanFilter.hpp"

#include <iostream>

MultiTargetTracker::MultiTargetTracker():
    targets_(),
    detections_()
{
  std::cout << "multi-target tracking started" << std::endl;
}

MultiTargetTracker::~MultiTargetTracker()
{
  std::cout << "multi-target tracking ended" << std::endl;
}

void MultiTargetTracker::PerformTrackingForOneExperiment(const std::string &configuration_file_name)
{
  ParameterHandler parameter_handler(configuration_file_name);
  ImageProcessingEngine image_processing_engine(parameter_handler);
  image_processing_engine.CreateImageProcessingOutputFile();
  KalmanFilter kalman_filter(parameter_handler, image_processing_engine);
  kalman_filter.CreateKalmanFilterOutputFiles();

  image_processing_engine.RetrieveBacterialData(parameter_handler.GetFirstImage(), detections_);
  kalman_filter.InitializeTargets(targets_, detections_);

  for (int i = parameter_handler.GetFirstImage() + 1; i <= parameter_handler.GetLastImage(); ++i)
  {
    image_processing_engine.RetrieveBacterialData(i, detections_);
    kalman_filter.PerformEstimation(i, targets_, detections_);
  } // i
}