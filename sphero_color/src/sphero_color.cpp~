//sphero_color project, 2016
//Author : Boris Bidault

//Most of the Open CV part of the code was found online, but without any source of whom is the original writer

#include "ros/ros.h"
#include "std_msgs/ColorRGBA.h"
#include "std_msgs/Bool.h"
#include <sstream>

#include "opencv/cvaux.h"
#include "opencv/highgui.h"
#include "opencv/cxcore.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include "math.h"
#include <stdio.h>
#include <iostream>



using namespace cv;
using namespace std;

int main(int argc, char **argv){

	//Publisher initialisation 
	ros::init(argc, argv, "sphero_color");
	ros::NodeHandle n;
	ros::Publisher chatter_color = n.advertise<std_msgs::ColorRGBA>("set_color", 1000);		//the color publisher
	ros::Publisher chatter_stab = n.advertise<std_msgs::Bool>("disable_stabilization", 1000);	//the stability publisher, the automatic stability of sphero must be disabled to show the top of the robot to the camera efficiently


	//Webcam initialisation
	VideoCapture cap(1); 	//capture the video from web cam
	if (!cap.isOpened()){	// if not success, exit program
      		cout << "Cannot open the web cam" << endl;
      		return -1;
    	}


  	//The control window, not usefull here since the parameters are already rightly setted
  	namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"

  	//The different parameters are setted here 
  	int iLowH = 0;
  	int iHighH = 0;
  	int iLowS = 0; 
  	int iHighS = 0;
  	int iLowV = 255;
  	int iHighV = 255;

  	//Create trackbars in "Control" window
  	cvCreateTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
  	cvCreateTrackbar("HighH", "Control", &iHighH, 179);
  	cvCreateTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
  	cvCreateTrackbar("HighS", "Control", &iHighS, 255);
  	cvCreateTrackbar("LowV", "Control", &iLowV, 255); //Value (0 - 255)
  	cvCreateTrackbar("HighV", "Control", &iHighV, 255);


  	namedWindow("Original", CV_WINDOW_AUTOSIZE); //create a window called "Original"


  	while (ros::ok()){

    		Mat imgOriginal;	//The original image red from the video

    		bool bSuccess = cap.read(imgOriginal);	// read a new frame from video

      		if (!bSuccess){	//if not success, break loop
	  		cout << "Cannot read a frame from video stream" << endl;
	  		break;
        	}

	 	Mat imgCircle = Mat::zeros( imgOriginal.size(), CV_8UC3 );

      		Mat imgHSV; 	//HSV version of the original image

      		cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV); 	//Convert the captured frame from BGR to HSV

	 	Mat imgThresholded; 	//Thresholded version of the HSV image

		inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); 	//Threshold the image

		//morphological opening (remove small objects from the foreground)
      		erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
      		dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 

      		//morphological closing (fill small holes in the foreground)
      		dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(15,15)) ); 
      		erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(15,15)) );

		Moments oMoments = moments(imgThresholded);

      		double dM01 = oMoments.m01;
      		double dM10 = oMoments.m10;
      		double dArea = oMoments.m00;
	
		float posX, posY, size;


		if (dArea > 10000){
	
	  		//calculate the position of sphero
	  		posX = dM10 / dArea ;
	  		posY = dM01 / dArea;
	  		size = (float)sqrt( dArea/3.14)/15;
		}
	
    		float r = posX/640;
    		float g = posY/480;
    		float b = size/140;

		circle(imgCircle, Point(posX,posY), size, cvScalar(r*255, g*255, b*255),10,8,0);

		imgOriginal = imgOriginal + imgCircle;

      		imshow("Original", imgOriginal); 	//show the original image
      		imshow("Thresholded", imgThresholded);

   		std_msgs::Bool stab;

   		stab.data = 1;
   		chatter_stab.publish(stab);


		//Main part of the project, define the r, g and b parameter of sphero thanks to the position determined before
    		std_msgs::ColorRGBA msg;
	
    		if (r > g && r > b){
			msg.r = 1;
    			msg.g = g / r;
    			msg.b = b / r;
   			msg.a = 1;
		}
   		 if (g > r && g > b){
			msg.r = r / g;
    			msg.g = 1;
    			msg.b = b / g;
   			msg.a = 1;
		}
    		if (b > g && b > r){
			msg.r = r / b;
    			msg.g = g / b;
    			msg.b = 1;
   			msg.a = 1;
		}
    		if (r == 0 && g == 0 && b == 0){
			msg.r = 1;
    			msg.g = 1;
    			msg.b = 0;
   			msg.a = 1;
		}

    		chatter_color.publish(msg);

    		ros::spinOnce();


    		if (waitKey(30) >= 0){	 //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
	  		break; 
		}

  	}

	return 0;

}
