#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <time.h>

using namespace cv;
using namespace std;


void FitLineRANSAC(const vector<Point2d>& points,double threshold,int iteration_number,Mat &image,Mat &draw, int supportNumber); 

int main(int argc, char** argv)
{
    

    
    srand(time(NULL));
    int iteration;
    cout << "Enter the iteration number: ";
    cin >> iteration;
    int sn;
    cout << "Enter the supporting number of points: ";
    cin >> sn;
    
    // applying the canny edge detecting on the image
    
    cv::Mat mainImage = cv::imread("0097.png");    
    cv::Mat contours;
    cv::Mat gray_image;
    cv::cvtColor(mainImage, gray_image, CV_RGB2GRAY);
    //applying canny edge detection
    cv::Canny(gray_image, contours,100, 150);
   // extracting edge points from the canny output
    
    vector<Point2d> points;
    for (int i = 0; i < contours.rows; i++) {
        for (int j = 0; j < contours.cols; j++) {
            if (contours.at<uchar>(i, j)==255) {
                Point2d p;
                p.x = j;
                p.y = i;
                points.push_back(p);
            }  
        }
    }
    
   

  // running the ransac algorithm
    for (size_t i = 0; i < iteration;i++) {
        FitLineRANSAC(points, 2.0, 6, contours, mainImage, sn);
    }
    //writing the results
    cv::imwrite("resault.png",mainImage);
    cv::imwrite("contour resault.png", contours);
    
		waitKey(0);
}


void FitLineRANSAC(
    const vector<Point2d>& points_,double threshold_,int maximum_iteration_number_,Mat &image_, Mat& draw,int supportNumber)
{
    int iterationNumber = 0;
    int bestInlierNumber = 0;
    vector<int> bestInliers, inliers;
    

    Mat bestLine(3, 1, CV_64F);
    Point2d bestPt1, bestPt2;
    constexpr int kSampleSize = 2;
    std::vector<int> sample(kSampleSize);

    
    cv::Mat tmp_image;
    while (iterationNumber++ < maximum_iteration_number_)
    {

        //selecting 2 random points
        for (size_t sampleIdx = 0; sampleIdx < kSampleSize; ++sampleIdx)
        {
            do
            {
                sample[sampleIdx] = round((points_.size() - 1) * static_cast<double>(rand()) / static_cast<double>(RAND_MAX));
                if (sampleIdx == 0)
                    break;
                if (sampleIdx == 1 && sample[0] != sample[1])
                    break;
            } while (true);
        }

        //tmp_image = image_->clone();
        tmp_image = image_;


        const Point2d& p1 = points_[sample[0]];
        const Point2d& p2 = points_[sample[1]];
        
        Point2d v = p2 - p1; 
        v = v / cv::norm(v);
        Point2d n;
        n.x = -v.y;
        n.y = v.x;
        double a = n.x;
        double b = n.y;
        double c = -(a * p1.x + b * p1.y);

        inliers.clear();
        for (size_t pointIdx = 0; pointIdx < points_.size(); ++pointIdx)
        {
            const Point2d& point = points_[pointIdx];
            const double distance =abs(a * point.x + b * point.y + c);

            if (distance < threshold_)
            {
                inliers.emplace_back(pointIdx);
            }
        }



        if (inliers.size() > supportNumber &&  inliers.size()>bestInliers.size())
        {
            printf("the number of inliers is: %d \n", inliers.size());
            
            bestInliers.swap(inliers);
            inliers.clear();
            inliers.resize(0);
            bestLine.at<double>(0) = a;
            bestLine.at<double>(1) = b;
            bestLine.at<double>(2) = c;
            
        }
       
    }
    cv::line(draw,
        Point2d(0, -bestLine.at<double>(2) / bestLine.at<double>(1)),
        Point2d(tmp_image.cols, (-bestLine.at<double>(0) * tmp_image.cols - bestLine.at<double>(2)) / bestLine.at<double>(1)),
        cv::Scalar(0, 0, 255),
        2);

}



