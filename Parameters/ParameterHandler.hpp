//
// Created by Nikita Kruk on 14.06.18.
//

#ifndef BEEMULTITARGETTRACKING_PARAMETERHANDLER_HPP
#define BEEMULTITARGETTRACKING_PARAMETERHANDLER_HPP

#include "../Definitions.hpp"

#include <string>

class ParameterHandler
{
 public:

  explicit ParameterHandler(const std::string &file_name);
  ~ParameterHandler();

  const std::string &GetInputFolder();
  const std::string &GetFileName0();
  const std::string &GetFileName1();
  const std::string &GetOriginalImagesSubfolder();
  const std::string &GetImageProcessingSubfolder();
  const std::string &GetImageProcessingOutputFileName();
  const std::string &GetKalmanFilterSubfolder();
  const std::string &GetKalmanFilterOutputFileName();
  const std::string &GetKalmanFilterMatlabOutputFileName();
  const std::string &GetDataAnalysisSubfolder();

  int GetFirstImage();
  int GetLastImage();
  int GetSubimageXPos();
  int GetSubimageYPos();
  int GetSubimageXSize();
  int GetSubimageYSize();
  int GetBlurType();
  int GetBlurRadius();
  int GetThresholdValue();
  int GetMorphologicalOperator();
  int GetMorphologicalElement();
  int GetMorphologicalSize();
  Real GetDataAssociationCost();

 private:

  std::string input_folder_;
  std::string file_name_0_;
  std::string file_name_1_;
  std::string original_images_subfolder_;
  std::string image_processing_subfolder_;
  std::string image_processing_output_file_name_;
  std::string kalman_filter_subfolder_;
  std::string kalman_filter_output_file_name_;
  std::string kalman_filter_matlab_output_file_name_;
  std::string data_analysis_subfolder_;

  int first_image_;
  int last_image_;
  int subimage_x_pos_;
  int subimage_y_pos_;
  int subimage_x_size_;
  int subimage_y_size_;
  int blur_type_;
  int blur_radius_;
  int threshold_value_;
  int morphological_operator_;
  int morphological_element_;
  int morphological_size_;
  Real data_association_cost_;

};

#endif //BEEMULTITARGETTRACKING_PARAMETERHANDLER_HPP
