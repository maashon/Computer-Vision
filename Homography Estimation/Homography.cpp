#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <fstream>
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>

using namespace cv;
using namespace std;

void readData(vector<pair<Point2f, Point2f> >& pointPairs, string adress);
Mat calcHomography(vector<pair<Point2f, Point2f> > pointPairs);
void transformImage(Mat origImg, Mat& newImage, Mat tr, bool isPerspective);
Mat homographyRANSAC(vector<pair<Point2f, Point2f> >& pointPairs, int maxIteration, float threshold);
int main(int argc, char** argv)
{
	vector<pair<Point2f, Point2f> >  points;
    readData(points, "match.txt");
    Mat H=homographyRANSAC(points,150,25.0);
   
    Mat image1;
    image1 = imread("input_0.orig.png");

    if (!image1.data)                              // Check for invalid input
    {
        cout << "Could not open or find the image" << argv[2] << std::endl;
        return -1;
    }

    Mat image2;
    image2 = imread("input_1.orig.png");   // Read the file

    if (!image2.data)                              // Check for invalid input
    {
        cout << "Could not open or find the image" << argv[3] << std::endl;
        return -1;
    }


    
    printf("Forming the panaromic Image ....");
    Mat transformedImage = Mat::zeros(1.5 * image1.size().height, 2.0 * image1.size().width, image1.type());
    transformImage(image2, transformedImage, Mat::eye(3, 3, CV_32F), true);

    //    transformImage(image2,transformedImage,tr2,false);
    transformImage(image1, transformedImage, H, true);

    imwrite("res.png", transformedImage);

    namedWindow("Display window", WINDOW_AUTOSIZE);// Create a window for display.
    imshow("Display window", transformedImage);                   // Show our image inside it.
    waitKey(0);
    



    return 0;
}




void readData(vector<pair<Point2f, Point2f> >& pointPairs, string adress) {
	ifstream MyReadFile(adress);
	string t;
	float x1, y1, x2, y2;
	cout << "Reading points from the match file file\n";
	while (getline(MyReadFile, t)) {
		x1 = stof(t.substr(0, t.find(" ")));
		t.erase(0, t.find(" ") + 2);
		y1 = stof(t.substr(0, t.find(" ")));
		t.erase(0, t.find(" ") + 2);
		x2 = stof(t.substr(0, t.find(" ")));
		t.erase(0, t.find(" ") + 2);
		y2 = stof(t.substr(0, t.length()-1));
		pair<Point2f, Point2f> currPts;
		currPts.first = Point2f(x1, y1);
		currPts.second = Point2f(x2, y2);
		pointPairs.emplace_back(currPts);
	}
	cout << "Point reading done!\n";
	MyReadFile.close();
}

Mat calcHomography(vector<pair<Point2f, Point2f> > pointPairs) {
    
    const int ptsNum = pointPairs.size();
    Mat A(2 * ptsNum, 9, CV_32F);
    for (int i = 0; i < ptsNum; i++) {
        float u1 = pointPairs[i].first.x;
        float v1 = pointPairs[i].first.y;

        float u2 = pointPairs[i].second.x;
        float v2 = pointPairs[i].second.y;

        A.at<float>(2 * i, 0) = u1;
        A.at<float>(2 * i, 1) = v1;
        A.at<float>(2 * i, 2) = 1.0f;
        A.at<float>(2 * i, 3) = 0.0f;
        A.at<float>(2 * i, 4) = 0.0f;
        A.at<float>(2 * i, 5) = 0.0f;
        A.at<float>(2 * i, 6) = -u2 * u1;
        A.at<float>(2 * i, 7) = -u2 * v1;
        A.at<float>(2 * i, 8) = -u2;

        A.at<float>(2 * i + 1, 0) = 0.0f;
        A.at<float>(2 * i + 1, 1) = 0.0f;
        A.at<float>(2 * i + 1, 2) = 0.0f;
        A.at<float>(2 * i + 1, 3) = u1;
        A.at<float>(2 * i + 1, 4) = v1;
        A.at<float>(2 * i + 1, 5) = 1.0f;
        A.at<float>(2 * i + 1, 6) = -v2 * u1;
        A.at<float>(2 * i + 1, 7) = -v2 * v1;
        A.at<float>(2 * i + 1, 8) = -v2;

    }

    Mat eVecs(9, 9, CV_32F), eVals(9, 9, CV_32F);
    //cout << A << endl;
    eigen(A.t() * A, eVals, eVecs);

    //cout << eVals << endl;
    //cout << eVecs << endl;


    Mat H(3, 3, CV_32F);
    for (int i = 0; i < 9; i++) H.at<float>(i / 3, i % 3) = eVecs.at<float>(8, i);

    //cout << H << endl;

    //Normalize:
    H = H * (1.0 / H.at<float>(2, 2));
    //printf("this is Homography martix \n");
    //cout << H << endl;
    
    return H;
}

void transformImage(Mat origImg, Mat& newImage, Mat tr, bool isPerspective) {
    Mat invTr = tr.inv();
    const int WIDTH = origImg.cols;
    const int HEIGHT = origImg.rows;

    const int newWIDTH = newImage.cols;
    const int newHEIGHT = newImage.rows;



    for (int x = 0; x < newWIDTH; x++) for (int y = 0; y < newHEIGHT; y++) {
        Mat pt(3, 1, CV_32F);
        pt.at<float>(0, 0) = x;
        pt.at<float>(1, 0) = y;
        pt.at<float>(2, 0) = 1.0;

        Mat ptTransformed = invTr * pt;
        
        if (isPerspective) ptTransformed = (1.0 / ptTransformed.at<float>(2, 0)) * ptTransformed;

        int newX = round(ptTransformed.at<float>(0, 0));
        int newY = round(ptTransformed.at<float>(1, 0));

        if ((newX >= 0) && (newX < WIDTH) && (newY >= 0) && (newY < HEIGHT)) newImage.at<Vec3b>(y, x) = origImg.at<Vec3b>(newY, newX);

        //        printf("x:%d y:%d newX:%d newY:%d\n",x,y,newY,newY);
    }
}



Mat homographyRANSAC(vector<pair<Point2f, Point2f> >& pointPairs,int maxIteration,float threshold) {

#define SQRT2 1.41
#define SQRT3 1.7321
	Mat bestH(3, 3, CV_32F);
    int bestInlierNumber = 0;
	constexpr int kSampleSize = 4;
	// The current sample
	std::vector<int> sample(kSampleSize);
	int iterationNumber = 0;
    printf("Performing the Ransac algorithm ..\n");
    vector<Point2f> image1Points;
    vector<Point2f> image2Points;

    vector<Point2f> normalizedPoints1;
    vector<Point2f> normalizedPoints2;
    for (size_t i=0; i < pointPairs.size();i++) {
        Point2f p1;
        Point2f p2;
        p1.x = pointPairs.at(i).first.x;
        p1.y = pointPairs.at(i).first.y;
        p2.x = pointPairs.at(i).second.x;
        p2.y = pointPairs.at(i).second.y;
        image1Points.emplace_back(p1);
        image2Points.emplace_back(p2);

    }
    int inlierNum = 0;

    //normalizing the datapoints
    int numOfPoints = image1Points.size();
    float mean1x = 0.0, mean1y = 0.0, mean2x = 0.0, mean2y = 0.0;
    for (size_t i = 0; i < numOfPoints; i++) {
        mean1x += image1Points[i].x;
        mean1y += image1Points[i].y;
        mean2x += image2Points[i].x;
        mean2y += image2Points[i].y;
    }
    mean1x /= numOfPoints;
    mean1y /= numOfPoints;
    mean2x /= numOfPoints;
    mean2y /= numOfPoints;
    
    float spread1x = 0.0, spread1y = 0.0, spread2x = 0.0, spread2y = 0.0;
    for (int i = 0; i < numOfPoints; i++) {
        spread1x += (image1Points[i].x - mean1x) * (image1Points[i].x - mean1x);
        spread1y += (image1Points[i].y - mean1y) * (image1Points[i].y - mean1y);
        spread2x += (image2Points[i].x - mean2x) * (image2Points[i].x - mean1x);
        spread2y += (image2Points[i].y - mean2y) * (image2Points[i].y - mean2y);
        
    }

    spread1x /= numOfPoints;
    spread1y /= numOfPoints;
    spread2x /= numOfPoints;
    spread2y /= numOfPoints;

    Mat offs1 = Mat::eye(3, 3, CV_32F);
    Mat offs2 = Mat::eye(3, 3, CV_32F);
    Mat scale1 = Mat::eye(3, 3, CV_32F);
    Mat scale2 = Mat::eye(3, 3, CV_32F);

    offs1.at<float>(0, 2) = -mean1x;
    offs1.at<float>(1, 2) = -mean1y;

    offs2.at<float>(0, 2) = -mean2x;
    offs2.at<float>(1, 2) = -mean2y;

    scale1.at<float>(0, 0) = SQRT2 / sqrt(spread1x);
    scale1.at<float>(1, 1) = SQRT2 / sqrt(spread1y);

    scale2.at<float>(0, 0) = SQRT2 / sqrt(spread2x);
    scale2.at<float>(1, 1) = SQRT2 / sqrt(spread2y);

    Mat T1=scale1 * offs1;
    Mat T2 = scale2 * offs2;
    for (size_t i = 0; i < numOfPoints; i++) {
        Mat pt(3, 1, CV_32F);
        pt.at<float>(0, 0) = image1Points.at(i).x;
        pt.at<float>(1, 0) = image1Points.at(i).y;
        pt.at<float>(2, 0) = 1.0;
        Mat ptTransformed = T1 * pt;
        Point2f tp;
        tp.x=ptTransformed.at<float>(0, 0);
        tp.y = ptTransformed.at<float>(1, 0);
        normalizedPoints1.emplace_back(tp);
    
    }

    for (size_t i = 0; i < numOfPoints; i++) {
        Mat pt(3, 1, CV_32F);
        pt.at<float>(0, 0) = image2Points.at(i).x;
        pt.at<float>(1, 0) = image2Points.at(i).y;
        pt.at<float>(2, 0) = 1.0;
        Mat ptTransformed = T2 * pt;
        Point2f tp;
        tp.x = ptTransformed.at<float>(0, 0);
        tp.y = ptTransformed.at<float>(1, 0);
        normalizedPoints2.emplace_back(tp);

    }
    vector<pair<Point2f, Point2f> > normalizedPairs;
    for (size_t i = 0; i < numOfPoints;i++) {
        pair<Point2f, Point2f> currPts;
        currPts.first = normalizedPoints1.at(i);
        currPts.second = normalizedPoints2.at(i);
        normalizedPairs.emplace_back(currPts);
    
    
    }

	while (iterationNumber< maxIteration){
		
		//selecting the 3 point samples to form a pane
		
		for (size_t sampleIdx = 0; sampleIdx < kSampleSize; sampleIdx++)
		{
			do
			{
				// Generate a random index between [0, pointNumber]
				sample[sampleIdx] =
					round((normalizedPairs.size() - 1) * static_cast<double>(rand()) / static_cast<double>(RAND_MAX));

				// If the first point is selected we don't have to check if
				// that particular index had already been selected.
				if (sampleIdx == 0)
					break;

				// If the second point is being generated,
				// it should be checked if the index had been selected beforehand. 
				if (sampleIdx == 1 &&
					sample[1] != sample[0])
					break;
				if (sampleIdx == 2 && sample[2] != sample[1] && sample[2] != sample[0])
					break;
				if (sampleIdx == 3 && sample[3] != sample[2] && sample[3] != sample[1] && sample[3] != sample[0])
					break;
			} while (true);
		}
		
        //calculating the H matrix with the selected random points
		vector<pair<Point2f, Point2f> > ranSample;
		ranSample.emplace_back(normalizedPairs.at(sample[0]));
		ranSample.emplace_back(normalizedPairs.at(sample[1]));
		ranSample.emplace_back(normalizedPairs.at(sample[2]));
		ranSample.emplace_back(normalizedPairs.at(sample[3]));
        
		Mat H = calcHomography(ranSample);
        if (iterationNumber == 1)bestH = H;
        //calculating the point in image 2 using H matrix

        
        vector<Point2f> estimatedPoints;
        for (size_t i = 0; i < normalizedPoints1.size(); i++) {
            Mat p(3, 1, CV_32F);
            p.at<float>(0, 0) = normalizedPoints1.at(0).x;
            p.at<float>(1, 0) = normalizedPoints1.at(0).y;
            p.at<float>(2, 0) = 1.0;

            Mat ptTransformed = H * p;
            Point2f pt;
            pt.x = ptTransformed.at<float>(0, 0);
            pt.y = ptTransformed.at<float>(1, 0);
            estimatedPoints.emplace_back(pt);
        
        }

    //calculating the error of estimation
        
        for (size_t i = 0; i < normalizedPoints2.size(); i++) {
            float dx = (estimatedPoints.at(i).x) - (normalizedPoints2.at(i).x);
            float dy = (estimatedPoints.at(i).y) - (normalizedPoints2.at(i).y);

            dx *= dx;
            dy *= dy;
            float d = sqrt(dx + dy);
            if (d<threshold) {
                inlierNum++;
            }
        }

        if (inlierNum > bestInlierNumber) {
            bestH = H;
            bestInlierNumber = inlierNum;
        }
        inlierNum = 0;
        iterationNumber++;
	}
    printf("The best inlier number is %d \n", bestInlierNumber);
    cout << "Optimal Homography calculated successfully\n";
	cout << "the RANSAC code ran successfully\n";
    Mat invT2 = T2.inv();
    Mat rescaledH = invT2 * bestH * T1;
    return rescaledH;
}

