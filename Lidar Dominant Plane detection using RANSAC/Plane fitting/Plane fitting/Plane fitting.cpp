
#include "stdafx.h"
#include <iostream>
#include <fstream>
#include <opencv\cv.hpp>
#include <opencv\highgui.h>
#include <vector>
#include <time.h>


using namespace cv;
using namespace std;
void PlaneRANSAC(const vector<Point3d>& points_,
	vector<int>& inliers_,
	Mat& plane_,
	double threshold_,
	double confidence_,
	int maximum_iteration_number_);
size_t GetIterationNumber(
	const double& inlierRatio_,
	const double& confidence_,
	const size_t& sampleSize_);
void readData(vector<Point3d>& points, string adress);

void FitPlaneLSQ(const vector<Point3d>* const points,
	vector<int>& inliers,
	Mat& plane);
int _tmain(int argc, _TCHAR* argv[])
{
	srand(time(NULL));
	string address = "street.xyz";
	vector<Point3d> points;
	readData(points, address);
	vector<int> inliers;
	//The parameters of the line
	Mat bestPlane(4, 1, CV_64F);
	double th = 0.04;
	PlaneRANSAC(points,inliers, bestPlane,th,0.95,20000);
	const double& a1 = bestPlane.at<double>(0);
	const double& b1 = bestPlane.at<double>(1);
	const double& c1 = bestPlane.at<double>(2);
	const double& d1 = bestPlane.at<double>(3);
	double averageError1 = 0.0;
	double den = sqrt((a1 * a1) + (b1 * b1) + (c1 * c1));
	for (const auto& inlierIdx : inliers)
	{
		double distance = abs(a1 * points[inlierIdx].x + b1 * points[inlierIdx].y + c1 * points[inlierIdx].z+d1);
		distance /= den;
		averageError1 += distance;
	}
	averageError1 /= inliers.size();
	cout << "the size of the inlier vector is :"<<inliers.size();
	cout << "\na is :" << bestPlane.at<double>(0) << " b is : " << bestPlane.at<double>(1) << " c is : " << bestPlane.at<double>(2) << " d is : " << bestPlane.at<double>(3);
	FitPlaneLSQ(&points, inliers, bestPlane);

	const double& a2 = bestPlane.at<double>(0);
	const double& b2 = bestPlane.at<double>(1);
	const double& c2 = bestPlane.at<double>(2);
	const double& d2 = bestPlane.at<double>(3);
	double averageError2 = 0.0;
	double den2 = sqrt((a2 * a2) + (b2 * b2) + (c2 * c2));
	for (const auto& inlierIdx : inliers)
	{
		double distance = abs(a2 * points[inlierIdx].x + b2 * points[inlierIdx].y + c2 * points[inlierIdx].z + d2);
		distance /= den2;
		averageError2 += distance;
	}
	averageError2 /= inliers.size();
	cout << "\nthe average error of RANSAC is :" << averageError1;
	cout << "\nthe average error of LO-RANSAC is :" << averageError2;
	
	/*
	inliers.clear();
	double a = bestPlane.at<double>(0);
	double b = bestPlane.at<double>(1);
	double c = bestPlane.at<double>(2);
	double d = bestPlane.at<double>(3);
	double distance;
	Point3d pt;
	double den = sqrt((a * a) + (b * b) + (c * c));
	for (int i = 0; i < points.size(); ++i) {
		pt = points.at(i);
		 distance = abs((a*pt.x)+ (b * pt.y)+ (c * pt.z)+d) / den;
		if (distance < th) {
			inliers.emplace_back(points.at(i));
		}

	
	}
	*/


	vector<bool> inlierVector(points.size());
	for (int i = 0; i < inlierVector.size();i++) {
		inlierVector.at(i) = false;
	}
	for (int i = 0; i < inliers.size(); i++) {
		inlierVector.at(inliers.at(i)) = true;
	}
	
	
	//writing the output to a file
	/**/
	cout << "\nwriting the output xyz file";
	ofstream file;
	file.open("outputlsq.txt");
	string line;
	for (int i = 0; i < points.size(); i++) {
		if (inlierVector.at(i) == true) {
			line = std::to_string(points.at(i).x) + " " + to_string(points.at(i).y) + " " + to_string(points.at(i).z) + " 255 255 0 \n";
		}
		else {
			line = std::to_string(points.at(i).x) + " " + to_string(points.at(i).y) + " " + to_string(points.at(i).z) + " 0 0 255 \n";
			
		}
		file << line;
	}
	file.close();
	cout << "\n the xyz file written successfully";
	}
		
		


void readData(vector<Point3d> &points,string adress) {
	ifstream MyReadFile(adress);
	string t;
	double x, y, z;
	cout << "reading points from the xyz file\n";
	while (getline(MyReadFile, t)) {
		if (adress == "garden.xyz"){
			t = t.substr(1, t.length());
		}
		x = stod(t.substr(0,t.find(" ")));
		t.erase(0, t.find(" ") +1);
		y = stod(t.substr(0, t.find(" ")));
		t.erase(0, t.find(" ") + 1);
		z = stod(t.substr(0, t.length() ));
		Point3d point;
		point.x = x;
		point.y = y;
		point.z = z;
		points.emplace_back (point);
	}
	cout << "Point reading done!\n";
	MyReadFile.close();
}


size_t GetIterationNumber(
	const double& inlierRatio_,
	const double& confidence_,
	const size_t& sampleSize_)
{
	double a =
		log(1.0 - confidence_);
	double b =
		log(1.0 - std::pow(inlierRatio_, sampleSize_));

	if (abs(b) < std::numeric_limits<double>::epsilon())
		return std::numeric_limits<size_t>::max();

	return a / b;
}
void PlaneRANSAC(const vector<Point3d>& points_,
    vector<int>& inliers_,
	Mat& plane_,
	double threshold_,
	double confidence_,
	int maximum_iteration_number_) {
	int iterationNumber = 0;
	int bestInlierNumber = 0;
	vector<int> bestInliers, inliers;
	//bestInliers.reserve(points_.size());
	//inliers.reserve(points_.size());
	
	Mat bestPlane(4, 1, CV_64F);
	// Helpers to draw the line if needed
	Point3d bestPt1, bestPt2, bestPt3;
	

	constexpr int kSampleSize = 3;
	// The current sample
	std::vector<int> sample(kSampleSize);
	
	int maximumIterations = maximum_iteration_number_;
	
	while (iterationNumber++ < maximumIterations)
	{
		

		cout << "doing the iteration number :" << iterationNumber<<"\n";
		//selecting the 3 point samples to form a pane
		
		for (size_t sampleIdx = 0; sampleIdx < kSampleSize; sampleIdx++)
		{
			do
			{
				// Generate a random index between [0, pointNumber]
				sample[sampleIdx] =
					round((points_.size() - 1) * static_cast<double>(rand()) / static_cast<double>(RAND_MAX));

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
			} while (true);
		}
		const Point3d& p1 = points_[sample[0]];
		const Point3d& p2 = points_[sample[1]]; 
		const Point3d& p3 = points_[sample[2]];

		double a = ((p2.y-p1.y) * (p3.z-p1.z)) - ((p3.y-p1.y) * (p2.z-p1.z));
		double b= ((p2.z - p1.z) * (p3.x - p1.x)) - ((p3.z - p1.z) * (p2.x - p1.x));
		double c= ((p2.x - p1.x) * (p3.y - p1.y)) - ((p3.x - p1.x) * (p2.y - p1.y));
		double d = -1*((a*p1.x)+(b*p1.y)+(c*p1.z));
		
		double den = sqrt((a*a)+(b*b)+(c*c));
		//calculating the distance of all points and decide the inliers
		inliers.clear();
		for (int pointIdx = 0; pointIdx < points_.size(); ++pointIdx)
		{
			const Point3d& point = points_[pointIdx];
			const double distance =abs((a * point.x) + (b * point.y) + (c*point.z)+d) / den;

			if (distance < threshold_)
			{
				inliers.emplace_back(pointIdx);
				
			}
		}


		//  Store the inlier number and the line parameters if it is better than the previous best.
		
		if (inliers.size() > bestInliers.size())
		{
			bestInliers.swap(inliers);
			inliers.clear();
			inliers.resize(0);

			bestPlane.at<double>(0) = a;
			bestPlane.at<double>(1) = b;
			bestPlane.at<double>(2) = c;
			bestPlane.at<double>(3) = d;

			// Update the maximum iteration number
			/*maximumIterations = GetIterationNumber(
				static_cast<double>(bestInliers.size()) / static_cast<double>(points_.size()),
				confidence_,
				kSampleSize);*/

			//printf("Inlier number = %d\tMax iterations = %d\n", bestInliers.size(), maximumIterations);
		}

		//cout << bestPlane<<endl;
		inliers_ = bestInliers;
		plane_ = bestPlane;
		//inliers.clear();

	}

	

	cout << "the RANSAC code ran successfully";


}

void FitPlaneLSQ(const vector<Point3d>* const points,
	vector<int>& inliers,
	Mat& plane)
{
	vector<Point3d> normalizedPoints;
	normalizedPoints.reserve(inliers.size());

	// Calculating the mass point of the points
	Point3d masspoint(0, 0,0);

	for (const auto& inlierIdx : inliers)
	{
		masspoint += points->at(inlierIdx);
		normalizedPoints.emplace_back(points->at(inlierIdx));
	}
	masspoint = masspoint * (1.0 / inliers.size());

	// Move the point cloud to have the origin in their mass point
	for (auto& point : normalizedPoints)
		point -= masspoint;

	// Calculating the average distance from the origin
	double averageDistance = 0.0;
	for (auto& point : normalizedPoints)
	{
		averageDistance += cv::norm(point);
		
	}

	averageDistance /= normalizedPoints.size();
	const double ratio = sqrt(3) / averageDistance;

	// Making the average distance to be sqrt(2)
	for (auto& point : normalizedPoints)
		point *= ratio;

	// Now, we should solve the equation.
	cv::Mat A(normalizedPoints.size(), 3, CV_64F);

	// Building the coefficient matrix
	for (size_t pointIdx = 0; pointIdx < normalizedPoints.size(); ++pointIdx)
	{
		const size_t& rowIdx = pointIdx;

		A.at<double>(rowIdx, 0) = normalizedPoints[pointIdx].x;
		A.at<double>(rowIdx, 1) = normalizedPoints[pointIdx].y;
		A.at<double>(rowIdx, 2) = normalizedPoints[pointIdx].z;
	}

	cv::Mat evals, evecs;
	cv::eigen(A.t() * A, evals, evecs);

	const cv::Mat& normal = evecs.row(2);
	const double& a = normal.at<double>(0),
		& b = normal.at<double>(1), & c = normal.at<double>(2);
	const double d = -(a * masspoint.x + b * masspoint.y+ c * masspoint.z);

	plane.at<double>(0) = a;
	plane.at<double>(1) = b;
	plane.at<double>(2) = c;
	plane.at<double>(3) = d;

}