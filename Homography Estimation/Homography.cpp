#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <fstream>
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>

using namespace cv;
using namespace std;

void readData(vector<pair<Point2f, Point2f> >& pointPairs, string adress);

int main(int argc, char** argv)
{
	vector<pair<Point2f, Point2f> >  points;
	readData(points,"match1.txt");
	


    

    return 0;
}




void readData(vector<pair<Point2f, Point2f> >& pointPairs, string adress) {
	ifstream MyReadFile(adress);
	string t;
	float x1, y1, x2, y2;
	cout << "reading points from the match file file\n";
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