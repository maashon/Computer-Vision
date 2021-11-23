#pragma once
/*
void StereoEstimation_Naive(
  const int& window_size,
  const int& dmin,
  int height,
  int width,
  cv::Mat& image1, cv::Mat& image2, cv::Mat& naive_disparities, const double& scale);

void Disparity2PointCloud(
  const std::string& output_file,
  int height, int width, cv::Mat& disparities,
  const int& window_size,
  const int& dmin, const double& baseline, const double& focal_length);

*/

void Dynamic(
    const int& window_size,
    int height,
    int width,
    int weight,
    cv::Mat& image1, cv::Mat& image2,
    cv::Mat& dynamic_disparities, const double& scale);


void StereoEstimation_Naive(
    const int& window_size,
    const int& dmin,//we will use it later
    int height,
    int width,
    cv::Mat& image1, cv::Mat& image2, cv::Mat& naive_disparities, const double& scale/*later*/);


void Disparity2PointCloud(
    cv::Mat& image,
    const std::string& output_file,
    int height, int width, cv::Mat& disparities,
    const int& window_size,
    const int& dmin, const double& baseline, const double& focal_length, float xScale, float yScale, float zScale);
	
	
	
	double diparityDissimilaritySSD(
    cv::Mat& calculatedDisparity,
    cv::Mat& groundTruth
    ) ;
	
	

