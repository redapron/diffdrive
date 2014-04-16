#include <opencv2\highgui\highgui.hpp>
#include <iostream>
#include <string>

#define _USE_MATH_DEFINES
#include <math.h>

using namespace cv;
using namespace std;

#define BLUE Scalar(255,0,0)
#define GREEN Scalar(0,255,0)
#define RED Scalar(0,0,255)
#define BLACK Scalar(0,0,0)
#define WHITE Scalar(255,255,255)
#define GRAY Scalar(222,222,222)
double rightAngleInRad = 90 * M_PI / 180;

Point2d lineBegin, lineEnd;

void onMouse(int event, int x, int y, int flags, void* param)
{
	Mat map;
	map = *((Mat*)param);

	if (event == EVENT_LBUTTONDOWN) {
		lineBegin = Point(x,y);
		circle(map, lineBegin, 1, BLACK, 1);		
	}
	if (event == EVENT_LBUTTONUP) {
		lineEnd = Point(x,y);
		line(map, lineBegin, lineEnd, BLACK, 2);
	}
}


const int slider_max = 200;
int alpha_slider = 100, beta_slider = 100;

void on_trackbar( int, void* )
{
	// Do nothing!
}

int main(){
// <0> CREATE TRACKBAR FOR VELOCITY PARAMETER
	namedWindow("Velocity", 1);
	createTrackbar( "RED Wheel", "Velocity", &alpha_slider, slider_max, on_trackbar );
	createTrackbar( "BLUE Wheel", "Velocity", &beta_slider, slider_max, on_trackbar );
	
	bool isGoStraight = false;
	Point2d ICC;
	double x, y, theta, dTime;
	double vLeft, vRight, l;
	double w, R;
	Mat sol = (Mat_<double>(3,1) <<	250,
									250,
									90 * M_PI / 180	);

// <1.9> DRAW AN OBSTACLEMAP WHICH WILL BE COPIED TO THE ACTUAL MAP
	Mat obstacleMap(500,500, CV_8UC3, WHITE);
	int gridSize = 25;
	for (int k=0; k<obstacleMap.rows; k=k+gridSize) {
		line(obstacleMap, Point(k,0), Point(k, obstacleMap.cols), GRAY, 1);
	}
	for (int k=0; k<obstacleMap.cols; k=k+gridSize) {
		line(obstacleMap, Point(0,k), Point(obstacleMap.rows, k), GRAY, 1);
	}

// <1.01> DRAW OBSTACLE ON OBSTACLEMAP
	line(obstacleMap, Point(100,300), Point(300, 300), BLACK, 2);
	line(obstacleMap, Point(100,300), Point(100, 200), BLACK, 2);

// GET OBSTACLE FROM MOUSE
	Mat canvas(500,500, CV_8UC3, WHITE);
	imshow("canvas", canvas);
	setMouseCallback("canvas", onMouse, &obstacleMap);



	while(true) {		
		x = sol.at<double>(0);
		y = sol.at<double>(1);
		theta = sol.at<double>(2);
		dTime = 0.1;

		vLeft = alpha_slider-100;
		vRight =  beta_slider-100;
		l = 50.0;

// <1> FIND CAR NEW POSITION
		w = (vRight - vLeft)/l;
		if (vRight == vLeft) isGoStraight = true;
		else isGoStraight = false;
		if (!isGoStraight) {
			R = (l/2)*(vLeft+vRight)/(vRight-vLeft);

			ICC = Vec2d(	x - R*sin(theta),
							y +	R*cos(theta) );

			Mat term1 = (Mat_<double>(3,3) <<	cos(w*dTime),	-sin(w*dTime),	0, 
												sin(w*dTime),	cos(w*dTime),	0, 
												0,				0,				1  );
			Mat term2 = (Mat_<double>(3,1) <<	x - ICC.x,
												y - ICC.y,
												theta		);
			Mat term3 = (Mat_<double>(3,1) <<	ICC.x,
												ICC.y,
												w*dTime );

			sol = term1*term2+term3;
			//cout << sol << endl;
		} else {
			Mat term1 = (Mat_<double>(3,3) <<	cos(theta),	-sin(theta),	x, 
												sin(theta),	cos(theta),		y, 
												0,			0,				1  );
			Mat term2 = (Mat_<double>(3,3) <<	1,			0,				vRight*dTime, 
												0,			1,				0, 
												0,			0,				1  );
			Mat tempSol = term1*term2;
			sol = (Mat_<double>(3,1) << tempSol.at<double>(0,2),
										tempSol.at<double>(1,2),
										theta						);
		}

// DRAW THE MAP
		//Mat canvas = obstacleMap.clone();
		canvas = obstacleMap.clone();

// <1.1> CHECK FOR COLLISION??
		Point2d carPosition = Point2d(sol.at<double>(0), sol.at<double>(1));
		double sol_theta = sol.at<double>(2);

		Rect roi_carPosition( cv::Point( carPosition.x-l/2, carPosition.y-l/2 ), cv::Size( l,l ));
		Mat canvas_carPosition = canvas( roi_carPosition );
		imshow("canvas_carPosition", canvas_carPosition);
		int totalObstacle=0;
		for (int row=0; row<canvas_carPosition.rows; row++)
			for (int col=0; col<canvas_carPosition.cols; col++)
				if (canvas_carPosition.at<Vec3b>(row,col) == Vec3b(0,0,0))
					totalObstacle++;
		//cout << "totalObstacle: " << totalObstacle;
		circle(canvas_carPosition, Point(canvas_carPosition.rows/2, canvas_carPosition.cols/2), l/2, WHITE, -1);
		imshow("canvas_carPosition2", canvas_carPosition);

		int untouchObstacle=0;
		for (int row=0; row<canvas_carPosition.rows; row++)
			for (int col=0; col<canvas_carPosition.cols; col++)
				if (canvas_carPosition.at<Vec3b>(row,col) == Vec3b(0,0,0))
					untouchObstacle++;
		//cout << "\tuntouchObstacle: " << untouchObstacle << endl;

		bool isCollide = false;
		if (totalObstacle != untouchObstacle) isCollide = true;
		if (isCollide) {
			carPosition = Point2d(x,y);
			sol_theta = theta;

			sol = (Mat_<double>(3,1) << carPosition.x,
										carPosition.y,
										theta						);
		}
		if (isCollide) cout << "COLLISION DETECTED" << endl;


// <2> DRAW A CAR!
		Point2d leftWheel = Point2d(	carPosition.x - l/2*sin(sol_theta),
										carPosition.y + l/2*cos(sol_theta)	);
		Point2d rightWheel = Point2d(	carPosition.x + l/2*sin(sol_theta),
										carPosition.y - l/2*cos(sol_theta) );
		Point2d carHead = Point2d(	carPosition.x + l/2*cos(sol_theta),
									carPosition.y + l/2*sin(sol_theta) );

		circle(canvas, carPosition, 25, BLACK, 2);
		circle(canvas, leftWheel, 5, RED, -1);
		circle(canvas, rightWheel, 5, BLUE, -1);
		circle(canvas, carHead, 5, GREEN, -1);

		if (!isGoStraight) circle(canvas, ICC, 3, GRAY, -1);

		imshow("canvas", canvas);

		//cout << carPosition << endl;
		//cout << leftWheel << endl;
		//cout << rightWheel << endl;
		//cout << cos(theta) << endl;

// <3> DRAW WHAT THE CAR SEE

		char c = waitKey(100);
		if (c == 27) break;
		if (c == ' ') {
			while (true){
				cout << ">> ";
				string cmd;
				cin >> cmd;
				if (cmd.compare("resume") == 0) break;
				if (cmd.compare("exit") == 0) return 0;
				if (cmd.compare("turnright") == 0){
					sol = (Mat_<double>(3,1) << x,
												y,
												0*rightAngleInRad	);
					break;
					
				}
				if (cmd.compare("turnleft") == 0){
					sol = (Mat_<double>(3,1) << x,
												y,
												2*rightAngleInRad	);
					break;
				}
				if (cmd.compare("turnup") == 0){
					sol = (Mat_<double>(3,1) << x,
												y,
												3*rightAngleInRad	);
					break;
				}
				if (cmd.compare("turndown") == 0){
					sol = (Mat_<double>(3,1) << x,
												y,
												1*rightAngleInRad	);
					break;
				}
				if (cmd.compare("move") == 0){
					Mat term1 = (Mat_<double>(3,3) <<	cos(theta),	-sin(theta),	x, 
														sin(theta),	cos(theta),		y, 
														0,			0,				1  );
					Mat term2 = (Mat_<double>(3,3) <<	1,	0,	50, 
														0,	1,	0, 
														0,	0,	1  );
					Mat tempSol = term1*term2;
					sol = (Mat_<double>(3,1) << tempSol.at<double>(0,2),
												tempSol.at<double>(1,2),
												theta						);
					break;
				}
			}
		}

	}

	while (true) {
		char c = waitKey(100);
		if (c == 27) break;
	}
	return 0;
}