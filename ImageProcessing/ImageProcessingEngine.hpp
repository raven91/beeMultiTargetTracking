//
// Created by Nikita Kruk on 14.06.18.
//

#ifndef BEEMULTITARGETTRACKING_IMAGEPROCESSINGENGINE_HPP
#define BEEMULTITARGETTRACKING_IMAGEPROCESSINGENGINE_HPP

#include "../Definitions.hpp"
#include "../Parameters/ParameterHandler.hpp"

#include <fstream>
#include <vector>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <eigen3/Eigen/Dense>

class ImageProcessingEngine
{
 public:

  explicit ImageProcessingEngine(ParameterHandler &parameter_handler);
  ~ImageProcessingEngine();

  void RetrieveBacterialData(int image, std::vector<Eigen::VectorXf> &detections);

  const cv::Mat &GetSourceImage();
  const cv::Mat &GetSourceImage(int image);
  const std::vector<cv::Point>& GetContour(int idx);

 private:

  ParameterHandler &parameter_handler_;
  std::ofstream image_processing_output_file_;

  cv::Mat bgr_image_;
  cv::Mat saturated_image_;
  cv::Mat blur_image_;
  cv::Mat hsv_image_;
  cv::Mat morphology_image_;
  cv::Mat threshold_image_;
  cv::Mat edge_image_;
  cv::Mat contour_image_;
  cv::Mat gray_image_;

  std::vector<std::vector<cv::Point>> contours_;

  void RetrieveSourceImage(int image);
  void SaturateImage(const cv::Mat &I, cv::Mat &O);
  void ApplyBlurFilter(const cv::Mat &I, cv::Mat &O);
  void ConvertBgrToHsv(const cv::Mat &I, cv::Mat &O);
  void ConvertBgrToGray(const cv::Mat &I, cv::Mat &O);
  void ApplyMorphologicalTransform(const cv::Mat &I, cv::Mat &O);
  void ApplyThreshold(const cv::Mat &I, cv::Mat &O);
  void DetectEdges(const cv::Mat &I, cv::Mat &O);
  void FindContours(const cv::Mat &I);
  void DrawContours();

  void SaveImage(const cv::Mat &I, int image);
  void SaveDetectedObjects(int image, std::vector<Eigen::VectorXf> &detections);

};

#endif //BEEMULTITARGETTRACKING_IMAGEPROCESSINGENGINE_HPP
