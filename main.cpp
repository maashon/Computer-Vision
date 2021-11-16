#include <opencv2/opencv.hpp>
#include <iostream>
#include <string> 
#include <vector>
#include <time.h>
#include <fstream>
#include <sstream>
#include "main.h"

using namespace std;

using namespace cv;

int main(int argc, char** argv) {
    // Parameters //
    // camera setup parameters
    const double focal_length = 3740;//1247;
    const double baseline = 160;//213;

    // stereo estimation parameters
    const int dmin =  atoi(argv[3]);
    const int window_size = atoi(argv[4]);
    const double weight = 500;
    const double scale = 3;

    ///////////////////////////
    // Commandline arguments //
    ///////////////////////////

    if (argc < 4) {
        std::cerr << "Usage: " << argv[0] << " IMAGE1 IMAGE2 OUTPUT_FILE" << std::endl;
        return 1;
    }

    cv::Mat image1 = cv::imread(argv[1], cv::IMREAD_GRAYSCALE);
    cv::Mat image2 = cv::imread(argv[2], cv::IMREAD_GRAYSCALE);
    cv::Mat coloredImage = cv::imread(argv[1], cv::IMREAD_COLOR);


    const std::string output_file = "output-";

    if (!image1.data) {
        std::cerr << "No image1 data" << std::endl;
        return EXIT_FAILURE;
    }

    if (!image2.data) {
        std::cerr << "No image2 data" << std::endl;
        return EXIT_FAILURE;
    }

    std::cout << "------------------ Parameters -------------------" << std::endl;
    std::cout << "focal_length = " << focal_length << std::endl;
    std::cout << "baseline = " << baseline << std::endl;
    std::cout << "window_size = " << window_size << std::endl;
    std::cout << "occlusion weights = " << weight << std::endl;
    std::cout << "disparity added due to image cropping = " << dmin << std::endl;
    std::cout << "scaling of disparity images to show = " << scale << std::endl;
    std::cout << "output filename = " << argv[3] << std::endl;
    std::cout << "-------------------------------------------------" << std::endl;

    int height = image1.size().height;
    int width = image1.size().width;



    //naive Sterio reconstruction
   cv::Mat naive_disparities = cv::Mat::zeros(height, width, CV_8UC1);
   //disparity map
   StereoEstimation_Naive(window_size, dmin, height, width,image1, image2,naive_disparities, scale);
   //converting the disparity map tp 3D file
   Disparity2PointCloud(coloredImage, "output-naive 3D File", 0, 0, naive_disparities, 5, dmin, baseline, focal_length, 1.0, 1.0, 0.03);
   //writing the png file of the disparity map
   std::stringstream out1;
   out1 << output_file << "naive-disparity.png";
   cv::imwrite(out1.str(), naive_disparities);




   //estimating the disparity map using the Dynamic programmuing approach
   cv::Mat dp_disparities = cv::Mat::zeros(height, width, CV_8UC1);
   //disparity map
   StereoEstimation_Dynamic(window_size, height, width, 50000, image1, image2, dp_disparities, 10.0);
   Disparity2PointCloud(coloredImage, "output-dp 3D File", 0, 0, dp_disparities, 5, dmin, baseline, focal_length, 1.0, 1.0, 0.1);
   //writing the png file of the disparity map
   std::stringstream out2;
   out2 << output_file << "dp-disparity.png";
   cv::imwrite(out2.str(), dp_disparities);


  

   /*Ptr<StereoBM> sbm = StereoBM::create(48, 15);
   sbm->compute(image1, image2, naive_disparities);*/
   
   // cv::imshow("Naive", naive_disparities);
    
    cv::waitKey(0);

    return 0;
}

void StereoEstimation_Naive(
    const int& window_size,
    const int& dmin,//we will use it later
    int height,
    int width,
    cv::Mat& image1, cv::Mat& image2, cv::Mat& naive_disparities, const double& scale/*later*/)
{
    int half_window_size = window_size / 2;
    int progress = 0;// shred between threads
   

    auto t_begin = std::chrono::high_resolution_clock::now();

#pragma omp parallel for
    for (int i = half_window_size; i < height - half_window_size; ++i) {//for each row

#pragma omp critical
        {
            std::cout
                << "Calculating disparities for the naive approach... "
                << std::ceil(((progress - half_window_size + 1) / static_cast<double>(height - window_size + 1)) * 100) << "%\r"
                << std::flush;
            ++progress;
        }




        for (int j = half_window_size; j < width - half_window_size; ++j) {//for each column
            //min search 
            int min_ssd = INT_MAX;
            int disparity = 0;


            int mean_left = 0;

            for (int u = -half_window_size; u <= half_window_size; ++u) {
                for (int v = -half_window_size; v <= half_window_size; ++v) {
                    int value_left = image1.at<uchar>(i + u, j + v);
                    mean_left += value_left;

                }
            }
            mean_left /= window_size * window_size;
            
               

            for (int d = -j + half_window_size; d < width - j - half_window_size; ++d) {// for each possible disparity(for each location on the right image)
                int ssd = 0;



                int mean_right = 0;

                for (int u = -half_window_size; u <= half_window_size; ++u) {
                    for (int v = -half_window_size; v <= half_window_size; ++v) {
                        int value_right = image2.at<uchar>(i + u, j + v + d);
                        mean_right += value_right;

                    }
                }
                mean_right /= window_size * window_size;

                //normalization :center the data.we first compute the mean of left image then the mean for the
                // //right image then and substract it from the data before computing ssd
                // another thing is to apply scaling. when one image is brighter than the other one
                // so computing their difference is not sufficient
                // normalization can be applied to cross corr 
                //  cross correlation or negate cross co relation (cc or ncc)
                // if we want to focus into pixel location we could appy gaussian window
                // TODO: sum up matching cost (ssd) in a window*computing c(i,j,d)
                for (int u = -half_window_size; u <= half_window_size; ++u) {


                    for (int v = -half_window_size; v <= half_window_size; ++v) {
                        int value_left = image1.at<uchar>(i+u,j+v)-mean_left;
                        int value_right = image2.at<uchar>(i + u, j + v + d)- mean_right;
                        //int value_left = image1.at<uchar>(i + u, j + v);
                       // int value_right = image2.at<uchar>(i + u, j + v + d);
                        int spectural_distance = value_left - value_right;
                        ssd += spectural_distance * spectural_distance;
                    }

                }
                if (ssd < min_ssd) {
                    min_ssd = ssd;
                    disparity = d;
                }
            }

            naive_disparities.at<uchar>(i - half_window_size, j - half_window_size) = std::abs(disparity) * scale;
        }
    }
    auto t_end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::duration<double>>(t_end - t_begin).count();
    std::cout << "Computatoin of naive approach took " << duration << " seconds\n" << std::endl;
    std::cout << "Calculating disparities for the naive approach... Done.\r" << std::flush;
    std::cout << std::endl;
}


void Disparity2PointCloud(
    cv::Mat& image,
    const std::string& output_file,
    int height, int width, cv::Mat& disparities,
    const int& window_size,
    const int& dmin, const double& baseline, const double& focal_length, float xScale, float yScale, float zScale)

    {
        std::cout << "\nwriting the output 3D file";
        std::ofstream file;
        file.open(output_file+".txt");
        std::string line;
        double x, y, z;
        int R, G, B;
        //calculating the baseline*focal_length outside the for loop for the sake of avoiding
        //unnecessary calculations inside the loop
        double nominator = baseline * focal_length;
        cv::Mat channels[3];
        cv::split(image, channels);
        int h = disparities.size().height;
        int w = disparities.size().width;

for (int i = 0; i < h; i++) {
    std::cout << "Reconstructing 3D point cloud from disparities"<< std::ceil(((i) / static_cast<double>(height + 1.)) * 100) << "%\r" << std::flush;
    for (int j = 0; j < w ; j++) {
        if (disparities.at<uchar>(i, j) == 0) continue;

        z = nominator / (double(disparities.at<uchar>(i, j)) + dmin) *zScale;
        x = i*xScale;
        y = j*yScale;
        
        R = channels[2].at<uchar>(i, j);
        G=  channels[1].at<uchar>(i, j);
        B=  channels[0].at<uchar>(i, j);
        line = to_string(x) + " " + to_string(y) + " " + to_string(z) + " " + to_string(R) + " " + to_string(G) + " " + to_string(B) + "\n";
        file << line;

    }
}
file.close();
std::cout << "Reconstructing 3D point cloud from disparities... Done.\r" << std::flush;
std::cout << std::endl;
}




void StereoEstimation_Dynamic(
    const int& window_size,
    int height,
    int width,
    int weight,
    cv::Mat& image1, cv::Mat& image2,
    cv::Mat& dynamic_disparities, const double& scale)
{
    auto t_begin = std::chrono::high_resolution_clock::now();
    int half_window_size = window_size / 2;
    cout << "Calculating disparities for the dynamic approach... ";
#pragma parallel for
    for (int r = half_window_size; r < height - half_window_size; ++r) {

              /*  std::cout
            << "Calculating disparities for the dynamic approach... "
            << std::ceil(((r - half_window_size + 1) / static_cast<double>(height - window_size + 1)) * 100) << "%\r"
            << std::flush;*/

        cv::Mat C = cv::Mat::zeros(width, width, CV_32F);
        cv::Mat M = cv::Mat::zeros(width, width, CV_8UC1);

        for (int x = 1; x < width; ++x) {
            C.at<float>(x, 0) = x * weight;
            M.at<uchar>(x, 0) = 3;
        }

        for (int y = 1; y < width; ++y) {
            C.at<float>(0, y) = y * weight;
            M.at<uchar>(0, y) = 2;
        }

        for (int x = 1; x < width; ++x) {
            for (int y = 1; y < width; ++y) {

                double d = DisparitySpaceImage(image1, image2, half_window_size, r, x, y);
                double match_cost = C.at<float>(x - 1, y - 1) + d;
                double left_occl_cost = C.at<float>(x - 1, y) + static_cast<float>(weight);
                double right_occl_cost = C.at<float>(x, y - 1) + static_cast<float>(weight);

                if (match_cost < std::min(left_occl_cost, right_occl_cost)) {
                    C.at<float>(x, y) = match_cost;
                    M.at<uchar>(x, y) = 1;
                }
                else if (left_occl_cost < std::min(match_cost, right_occl_cost)) {
                    C.at<float>(x, y) = left_occl_cost;
                    M.at<uchar>(x, y) = 2;
                }
                else { // (right_occl_cost < std::min(match_cost, left_occl_cost))
                    C.at<float>(x, y) = right_occl_cost;
                    M.at<uchar>(x, y) = 3;
                }

            }
        }

        int x = width - 1;
        int y = width - 1;
        int c = width;
        int d = 0;
        while (x != 0 && y != 0) {
            switch (M.at<uchar>(x, y)) {
            case 1:
                d = abs(x - y) * 3;
                x--;
                y--;
                c--;
                break;
            case 2:
                x--;
                break;
            case 3:
                y--;
                break;
            }
            dynamic_disparities.at<uchar>(r - half_window_size, c) = d;
        }
    }
    auto t_end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::duration<double>>(t_end - t_begin).count();
    std::cout << "Computatoin of DP algorithm took " << duration << " seconds\n" << std::endl;
    std::cout << "Calculating disparities for the dynamic approach... Done.\r" << std::flush;
    std::cout << std::endl;
}


int DisparitySpaceImage(
    cv::Mat& image1, cv::Mat& image2,
    int half_window_size, int r, int x, int y)
{
    int ssd = 0;
    for (int u = -half_window_size; u <= half_window_size; ++u) {
        for (int v = -half_window_size; v <= half_window_size; ++v) {
            int val_left = image1.at<uchar>(r + u, x + v);
            int val_right = image2.at<uchar>(r + u, y + v);
            ssd += (val_left - val_right) * (val_left - val_right);
        }
    }
    return ssd;
}

