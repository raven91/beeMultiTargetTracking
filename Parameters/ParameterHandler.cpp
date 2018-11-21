//
// Created by Nikita Kruk on 14.06.18.
//

#include "ParameterHandler.hpp"

#include <fstream>
#include <map>

ParameterHandler::ParameterHandler(const std::string &file_name)
{
//  std::ifstream parameters_file("/Users/nikita/CLionProjects/beeMultiTargetTracking/Config.cfg", std::ios::in);
  std::ifstream parameters_file(file_name, std::ios::in);
  assert(parameters_file.is_open());

  // read string values
  parameters_file >> input_folder_ >> input_folder_;
  parameters_file >> file_name_0_ >> file_name_0_;
  parameters_file >> file_name_1_ >> file_name_1_;
  parameters_file >> original_images_subfolder_ >> original_images_subfolder_;
  parameters_file >> image_processing_subfolder_ >> image_processing_subfolder_;
  parameters_file >> image_processing_output_file_name_ >> image_processing_output_file_name_;
  parameters_file >> kalman_filter_subfolder_ >> kalman_filter_subfolder_;
  parameters_file >> kalman_filter_output_file_name_ >> kalman_filter_output_file_name_;
  parameters_file >> kalman_filter_matlab_output_file_name_ >> kalman_filter_matlab_output_file_name_;
  parameters_file >> data_analysis_subfolder_ >> data_analysis_subfolder_;

  // read real values
  std::map<std::string, Real> parameters_dictionary;
  std::string key;
  Real value;
  while (parameters_file >> key >> value)
  {
    parameters_dictionary[key] = value;
  }
  parameters_file.close();

  // initialize real-valued variables
  first_image_ = (int) parameters_dictionary["first_image"];
  last_image_ = (int) parameters_dictionary["last_image"];
  subimage_x_pos_ = (int) parameters_dictionary["subimage_x_pos"];
  subimage_y_pos_ = (int) parameters_dictionary["subimage_y_pos"];
  subimage_x_size_ = (int) parameters_dictionary["subimage_x_size"];
  subimage_y_size_ = (int) parameters_dictionary["subimage_y_size"];
  blur_type_ = (int) parameters_dictionary["blur_type"];
  blur_radius_ = (int) parameters_dictionary["blur_radius"];
  threshold_value_ = (int) parameters_dictionary["threshold_value"];
  morphological_operator_ = (int) parameters_dictionary["morphological_operator"];
  morphological_element_ = (int) parameters_dictionary["morphological_element"];
  morphological_size_ = (int) parameters_dictionary["morphological_size"];
  data_association_cost_ = parameters_dictionary["data_association_cost"];
}

ParameterHandler::~ParameterHandler() = default;

const std::string &ParameterHandler::GetInputFolder()
{
  return input_folder_;
}

const std::string &ParameterHandler::GetFileName0()
{
  return file_name_0_;
}

const std::string &ParameterHandler::GetFileName1()
{
  return file_name_1_;
}

const std::string& ParameterHandler::GetOriginalImagesSubfolder()
{
  return original_images_subfolder_;
}

const std::string &ParameterHandler::GetImageProcessingSubfolder()
{
  return image_processing_subfolder_;
}

const std::string &ParameterHandler::GetImageProcessingOutputFileName()
{
  return image_processing_output_file_name_;
}

const std::string &ParameterHandler::GetKalmanFilterSubfolder()
{
  return kalman_filter_subfolder_;
}

const std::string &ParameterHandler::GetKalmanFilterOutputFileName()
{
  return kalman_filter_output_file_name_;
}

const std::string &ParameterHandler::GetKalmanFilterMatlabOutputFileName()
{
  return kalman_filter_matlab_output_file_name_;
}

const std::string& ParameterHandler::GetDataAnalysisSubfolder()
{
  return data_analysis_subfolder_;
}

int ParameterHandler::GetFirstImage()
{
  return first_image_;
}

int ParameterHandler::GetLastImage()
{
  return last_image_;
}

int ParameterHandler::GetSubimageXPos()
{
  return subimage_x_pos_;
}

int ParameterHandler::GetSubimageYPos()
{
  return subimage_y_pos_;
}

int ParameterHandler::GetSubimageXSize()
{
  return subimage_x_size_;
}

int ParameterHandler::GetSubimageYSize()
{
  return subimage_y_size_;
}

int ParameterHandler::GetBlurType()
{
  return blur_type_;
}

int ParameterHandler::GetBlurRadius()
{
  return blur_radius_;
}

int ParameterHandler::GetThresholdValue()
{
  return threshold_value_;
}

int ParameterHandler::GetMorphologicalOperator()
{
  return morphological_operator_;
}

int ParameterHandler::GetMorphologicalElement()
{
  return morphological_element_;
}

int ParameterHandler::GetMorphologicalSize()
{
  return morphological_size_;
}

Real ParameterHandler::GetDataAssociationCost()
{
  return data_association_cost_;
}
