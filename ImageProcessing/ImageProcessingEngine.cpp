//
// Created by Nikita Kruk on 14.06.18.
//

#include "ImageProcessingEngine.hpp"

#include <sstream>
#include <iostream>
#include <iomanip>
#include <cassert>

ImageProcessingEngine::ImageProcessingEngine(ParameterHandler &parameter_handler) :
    parameter_handler_(parameter_handler)
{
  bgr_image_ = cv::Mat::zeros(0, 0, CV_8UC3);
  hsv_image_ = cv::Mat::zeros(0, 0, CV_8UC3);
  threshold_image_ = cv::Mat::zeros(0, 0, CV_8UC1);
  edge_image_ = cv::Mat::zeros(0, 0, CV_8UC1);
  contour_image_ = cv::Mat::zeros(0, 0, CV_8UC3);

  std::string image_processing_output_file_name =
      parameter_handler_.GetInputFolder() + parameter_handler.GetDataAnalysisSubfolder()
          + parameter_handler.GetImageProcessingOutputFileName();
  image_processing_output_file_.open(image_processing_output_file_name, std::ios::out | std::ios::trunc);
  assert(image_processing_output_file_.is_open());
}

ImageProcessingEngine::~ImageProcessingEngine()
{
  image_processing_output_file_.close();
}

void ImageProcessingEngine::RetrieveBacterialData(int image, std::vector<Eigen::VectorXf> &detections)
{
  RetrieveSourceImage(image);
  SaturateImage(bgr_image_, saturated_image_);
  ApplyBlurFilter(saturated_image_, blur_image_);
//  ConvertBgrToHsv(blur_image_, hsv_image_);
  ConvertBgrToGray(blur_image_, gray_image_);
  ApplyMorphologicalTransform(gray_image_, morphology_image_);
  ApplyThreshold(morphology_image_, threshold_image_);
//  DetectEdges(threshold_image_, edge_image_);
  FindContours(threshold_image_);

  DrawContours();
  SaveImage(contour_image_, image);
  SaveDetectedObjects(image, detections);
}

void ImageProcessingEngine::RetrieveSourceImage(int image)
{
  std::ostringstream image_name_buf;
  image_name_buf << parameter_handler_.GetInputFolder() << parameter_handler_.GetOriginalImagesSubfolder()
                 << parameter_handler_.GetFileName0() << std::setfill('0')
                 << std::setw(9) << image << parameter_handler_.GetFileName1();
  std::string image_name = image_name_buf.str();

  std::cout << image_name << std::endl;

  bgr_image_ = cv::imread(image_name, CV_LOAD_IMAGE_COLOR);
  assert(bgr_image_.data != NULL);
  bgr_image_ = cv::Mat(bgr_image_,
                       cv::Rect(parameter_handler_.GetSubimageXPos(),
                                parameter_handler_.GetSubimageYPos(),
                                parameter_handler_.GetSubimageXSize(),
                                parameter_handler_.GetSubimageYSize()));
  cv::resize(bgr_image_, bgr_image_, cv::Size(), 1.0, 1.0, cv::INTER_LINEAR);
}

void ImageProcessingEngine::SaturateImage(const cv::Mat &I, cv::Mat &O)
{
  int alpha = 2;
  int beta = 0;
  O = I.clone();
  /// Do the operation new_image(i,j) = alpha*image(i,j) + beta
  for( int y = 0; y < I.rows; y++ )
  { for( int x = 0; x < I.cols; x++ )
    { for( int c = 0; c < 3; c++ )
      {
        O.at<cv::Vec3b>(y,x)[c] =
            cv::saturate_cast<uchar>( alpha*( I.at<cv::Vec3b>(y,x)[c] ) + beta );
      }
    }
  }
}

void ImageProcessingEngine::ApplyBlurFilter(const cv::Mat &I, cv::Mat &O)
{
  /*
   0: Normalized Block Filter
   1: Gaussian Filter
   2: Median Filter
   3: Bilateral Filter
   */
  int blur_type = parameter_handler_.GetBlurType();
  int blur_radius = parameter_handler_.GetBlurRadius();

  switch (blur_type)
  {
    case 0: // Normalized Block Filter
      cv::blur(I, O, cv::Size(2 * blur_radius + 1, 2 * blur_radius + 1), cv::Point(-1, -1));
      break;

    case 1: // Gaussian Filter
      cv::GaussianBlur(I, O, cv::Size(2 * blur_radius + 1, 2 * blur_radius + 1), 1, 1);
      break;

    case 2: // Median Filter
      cv::medianBlur(I, O, 2 * blur_radius + 1);
      break;

    case 3: // Bilateral Filter
      cv::bilateralFilter(I, O, 2 * blur_radius + 1, blur_radius * 2, blur_radius / 2);
      break;

    default:std::cerr << "wrong blur index" << std::endl;
      break;
  }
}

void ImageProcessingEngine::ConvertBgrToHsv(const cv::Mat &I, cv::Mat &O)
{
  cv::cvtColor(I, O, cv::COLOR_BGR2HSV);
}

void ImageProcessingEngine::ConvertBgrToGray(const cv::Mat &I, cv::Mat &O)
{
  cv::cvtColor(I, O, cv::COLOR_BGR2GRAY);
}

void ImageProcessingEngine::ApplyMorphologicalTransform(const cv::Mat &I, cv::Mat &O)
{
/* 0: Erosion
  1: Dilation
  2: Opening
  3: Closing
  4: Gradient
  5: Top Hat
  6: Black Hat
  */

  cv::Mat element = cv::getStructuringElement(parameter_handler_.GetMorphologicalElement(),
                                              cv::Size(2 * parameter_handler_.GetMorphologicalSize() + 1,
                                                       2 * parameter_handler_.GetMorphologicalSize() + 1),
                                              cv::Point(parameter_handler_.GetMorphologicalSize(),
                                                        parameter_handler_.GetMorphologicalSize()));
  cv::morphologyEx(I, O, parameter_handler_.GetMorphologicalOperator(), element);
}

void ImageProcessingEngine::ApplyThreshold(const cv::Mat &I, cv::Mat &O)
{
//  const int hue_lower = 0, saturation_lower = 0, value_lower = 0;
//  const int hue_upper = 180, saturation_upper = 255, value_upper = 255;
//  cv::inRange(hsv_image_,
//              cv::Scalar(hue_lower, saturation_lower, value_lower, 0),
//              cv::Scalar(hue_upper, saturation_upper, parameter_handler_.GetThresholdValue(), 0), threshold_image_);

  cv::threshold(I, O, parameter_handler_.GetThresholdValue(), 255, 1);
}

void ImageProcessingEngine::DetectEdges(const cv::Mat &I, cv::Mat &O)
{
  const int laplacial_kernel = 1;
  cv::Laplacian(I, O, CV_16S, 2 * laplacial_kernel + 1, 1, 0, cv::BORDER_DEFAULT);
  cv::convertScaleAbs(O, O);
}

void ImageProcessingEngine::FindContours(const cv::Mat &I)
{
  int offset = 2;
  cv::Mat extended_contour_image;
  contour_image_ = I.clone();
  cv::copyMakeBorder(contour_image_,
                     extended_contour_image,
                     offset,
                     offset,
                     offset,
                     offset,
                     cv::BORDER_CONSTANT,
                     cv::Scalar(0, 0, 0));

  contours_.clear();
  std::vector<cv::Vec4i> hierarchy;
  cv::findContours(extended_contour_image,
                   contours_,
                   hierarchy,
                   cv::RETR_EXTERNAL,
                   cv::CHAIN_APPROX_SIMPLE,
                   cv::Point(-offset, -offset));

  contours_.erase(std::remove_if(contours_.begin(),
                                 contours_.end(),
                                 [&](const std::vector<cv::Point> &contour)
                                 {
                                   double contour_area = cv::contourArea(contour);
                                   return (contour_area < 4 || contour_area > 500);
                                 }),
                  contours_.end());
}

void ImageProcessingEngine::DrawContours()
{
// draw colored contours
/*  cv::RNG rng(12345); // random color generator
  cv::Mat fitted_contours_image = cv::Mat::zeros(contour_image_.size(), CV_8UC3);
  for (int i = 0; i < contours_.size(); ++i)
  {
    cv::Scalar color = cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
    cv::drawContours(fitted_contours_image, contours_, i, color, -1, cv::LINE_AA, NULL, 0, cv::Point(0, 0));
//			cv::ellipse(fitted_contours_image, min_ellipse[i], color, 2, 8);
//			cv::Point2f rect_points[4];
//			min_rect[i].points(rect_points);
//			for (int j = 0; j < 4; ++j)
//			{
//				cv::line(fitted_contours_image, rect_points[j], rect_points[(j + 1) % 4], color, 2, 8);
//			}
//		}
  }
  contour_image_ = fitted_contours_image;
*/

  // draw contour centers
  cv::Mat indexed_contours_image = bgr_image_.clone();
  for (int i = 0; i < contours_.size(); ++i)
  {
    std::string index = std::to_string(i);
    cv::Moments mu = cv::moments(contours_[i], true);
    cv::Point center = cv::Point(mu.m10 / mu.m00, mu.m01 / mu.m00);
    cv::Scalar color = cv::Scalar(0, 0, 255);
    cv::circle(indexed_contours_image, center, 4, color, -1, 8);
    cv::putText(indexed_contours_image, index, center, cv::FONT_HERSHEY_DUPLEX, 0.6, color);
  }
  contour_image_ = indexed_contours_image;
}

void ImageProcessingEngine::SaveImage(const cv::Mat &I, int image)
{
  std::ostringstream output_image_name_buf;
  output_image_name_buf << parameter_handler_.GetInputFolder() << parameter_handler_.GetImageProcessingSubfolder()
                        << parameter_handler_.GetFileName0() << std::setfill('0') << std::setw(9) << image
                        << parameter_handler_.GetFileName1();
  std::string output_image_name = output_image_name_buf.str();
  cv::imwrite(output_image_name, I);
}

// according to the format
// i -> x_i y_i v_x_i v_y_i width length
void ImageProcessingEngine::SaveDetectedObjects(int image, std::vector<Eigen::VectorXf> &detections)
{
  detections.clear();

  image_processing_output_file_ << image << " " << contours_.size() << " ";

  cv::Vec4f fitted_line;
  cv::RotatedRect min_rect;
  Eigen::VectorXf new_detection(kNumOfExtractedFeatures);
  cv::Moments mu;

  for (int b = 0; b < (int) contours_.size(); ++b)
  {
    mu = cv::moments(contours_[b], true);
    new_detection(0) = Real(mu.m10 / mu.m00);
    new_detection(1) = Real(mu.m01 / mu.m00);

    new_detection(2) = 0.0;
    new_detection(3) = 0.0;

    min_rect = cv::minAreaRect(cv::Mat(contours_[b]));
    new_detection(4) = std::min(min_rect.size.width, min_rect.size.height);
    new_detection(5) = std::max(min_rect.size.width, min_rect.size.height);

    detections.push_back(new_detection);
    image_processing_output_file_ << b << " " << new_detection(0) << " " << new_detection(1) << " " <<
                                  new_detection(2) << " " << new_detection(3) << " " <<
                                  new_detection(4) << " " << new_detection(5) << " ";
  }
  image_processing_output_file_ << std::endl;
}

const cv::Mat &ImageProcessingEngine::GetSourceImage()
{
  return bgr_image_;
}

const std::vector<cv::Point> &ImageProcessingEngine::GetContour(int idx)
{
  return contours_[idx];
}
