//sphero_move project, 2016
//Author : Boris Bidault

//arrayCallback function, Alex Sleat

#include "ros/ros.h"
#include "std_msgs/ColorRGBA.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int32MultiArray.h"
#include <sstream>

#include "opencv/cvaux.h"
#include "opencv/highgui.h"
#include "opencv/cxcore.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include "math.h"
#include <iostream>
#include <stdio.h>


using namespace std;

int Arr[3];
void arrayCallback(const std_msgs::Int32MultiArray::ConstPtr& array);


int main(int argc, char **argv){

	//Publisher initialisation
	ros::init(argc, argv, "text");
	ros::NodeHandle n;
	ros::Publisher pub_color = n.advertise<std_msgs::ColorRGBA>("set_color", 1000);
	ros::Publisher pub_move = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
	ros::Subscriber sub = n.subscribe("array", 100, arrayCallback);


    	std_msgs::ColorRGBA color_msg;
    	color_msg.r = 255;
    	color_msg.g = 255;
    	color_msg.b = 0;
    	color_msg.a = 1;
    	pub_color.publish(color_msg);

	
	//Part 1 : Calibration	

	//Save the starting position of sphero
	int pos[3] = {0};
	while(pos[2] == 0){
		ros::spinOnce();
		for(int i = 0 ; i < 3 ; i++){
			pos[i] = Arr[i];
		}
	}

	cout<< "pos[0] = " << pos[0] << " pos[1] = " << pos[1] << " pos[2] = " << pos[2] <<endl;

    	geometry_msgs::Twist move_msg;
    	move_msg.linear.x = 0;
    	move_msg.linear.y = 0;
    	move_msg.linear.z = 0;
    	move_msg.angular.x = 0;
    	move_msg.angular.y = 0;
    	move_msg.angular.z = 0;
    	pub_move.publish(move_msg);

    	for(int i = 0 ; i < 100000 ; i++){
		move_msg.linear.y = 50;
		pub_move.publish(move_msg);
    	}    

    	move_msg.linear.y = 0;
    	pub_move.publish(move_msg);


    	string test;
    	cout<<"Press any key to continue :";
    	cin>>test;

	//Save the intermediate position of sphero
	int pos1[3] = {0};
	while(pos1[2] == 0){
		ros::spinOnce();
		for(int i = 0 ; i < 3 ; i++){
			pos1[i] = Arr[i];
		}
	}

	cout<< "pos1[0] = " << pos1[0] << " pos1[1] = " << pos1[1] << " pos1[2] = " << pos1[2] <<endl;

    	for(int i = 0 ; i < 100000 ; i++){
		move_msg.linear.x = 50;
		pub_move.publish(move_msg);
    	}


    	move_msg.linear.x = 0;
    	pub_move.publish(move_msg);

    	cout<<"Press any key to continue :";
    	cin>>test;

	//Sae the final position of sphero
    	int pos2[3] = {0};
	while(pos2[2] == 0){
		ros::spinOnce();
		for(int i = 0 ; i < 3 ; i++){
			pos2[i] = Arr[i];
		}
	}

	cout<< "pos2[0] = " << pos2[0] << " pos2[1] = " << pos2[1] << " pos2[2] = " << pos2[2] <<endl;

    	int y1 = pos1[0] - pos[0];
    	int x1 = pos1[1] - pos[1];    
    	int y2 = pos2[0] - pos1[0];
    	int x2 = pos2[1] - pos1[1];

	cout<< "x1 = " << x1 << " y1 = " << y1 << " x2 = " << x2 << " y2 = " << y2 <<endl;
    
    	/*Calculation of the transformation matrix
		(alpha  beta)
		(gamma delta)	
    	//calculation of alpha and beta*/
    	int det = x1*y2 - x2*y1;
    	int alpha = -10000*y2/det;
    	int beta = 10000*x2/det;
    	//calculation of gamma and delta
    	int gamma = 10000*y1/det;
    	int delta = -10000*x1/det;

	cout<< "alpha = " << alpha << " beta = " << beta << " gamma = " << gamma << " delta = " << delta <<endl;

	/*Calcul de la invert matrix
		(new_alpha  new_beta)
		(new_gamma new_delta)	
    	//calculation of new_alpha and new_beta*/
	int den = alpha*delta - gamma*beta;
	int new_alpha = 10000*delta/den;
	int new_beta = - 10000*beta/den;
    	//calculation of new_gamma and new_delta
	int new_gamma = - 10000*gamma/den;
	int new_delta = 10000*alpha/den;
	
	cout<< "den = " << den << " new_alpha = " << new_alpha << " new_beta = " << new_beta << " new_gamma = " << new_gamma << " new_delta = " << new_delta <<endl;

    	int x, y, order_x, order_y, linear_x, linear_y;

	
	//Part 2 : movement orders

  	while (ros::ok()){   

		//The user is asked to enter a position wished for sphero to go to
    		cout<<"x coordinates :";
    		cin>>order_x;
    		cout<<"y coordinates :";
    		cin>>order_y;
    
		y = order_y - pos2[0];
		x = order_x - pos2[1];

		//Calculation of the command needed for sphero to move to this position
		linear_x = new_alpha*x + new_beta*y;
		linear_y = new_gamma*x + new_delta*y;

		cout<< "linear_x = " << linear_x << " linear_y = " << linear_y <<endl;

		//Normalisation of the command
		if(abs(linear_x) > abs(linear_y)){
			if(linear_x > 0){
				order_y = linear_y*40/linear_x;
				order_x = 40;
			}
			else{
				order_y = - linear_y*40/linear_x;
				order_x = - 40;
			}
		}
		else{
			if(linear_y > 0){
				order_x = linear_x*40/linear_y;
				order_y = 40;
			}
			else{
				order_x = - linear_x*40/linear_y;
				order_y = - 40;
			}
		}

		cout<< "order_x = " << order_x << " order_y = " << order_y <<endl;

		int pos[3] = {0};
		while(pos[2] == 0){
			ros::spinOnce();
			for(int i = 0 ; i < 3 ; i++){
				pos[i] = Arr[i];
			}
		}

		//If sphero is near the wished position, it stop, else, it go on
		while((pos[0] < x - 100 || pos[0] > x + 100) && (pos[1] < y - 100 || pos[1] > y + 100)){

    			move_msg.linear.x = order_x;
    			move_msg.linear.y = order_y;
    			pub_move.publish(move_msg);
			
			int pos[3] = {0};
			while(pos[2] == 0){
				ros::spinOnce();
				for(int i = 0 ; i < 3 ; i++){
					pos[i] = Arr[i];
				}
			}
		}

		move_msg.linear.x = 0;
    		move_msg.linear.y = 0;
    		pub_move.publish(move_msg);

  	}

  	return 0;

}

//Function to subscribe to an array (position of sphero), written by Alex Sleat
//http://alexsleat.co.uk/2011/07/02/ros-publishing-and-subscribing-to-arrays/
void arrayCallback(const std_msgs::Int32MultiArray::ConstPtr& array){

	int i = 0;
	for(std::vector<int>::const_iterator it = array->data.begin(); it != array->data.end(); ++it){
		Arr[i] = *it;
		i++;
	}

	return;
}
