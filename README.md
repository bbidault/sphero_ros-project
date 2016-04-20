# sphero_ros-project
Two small projects using sphero_ros 

You will need the sphero_ros driver to run each projects, 
https://github.com/mmwise/sphero_ros

Put each folder in your catkin_ws/src directory
In the console, go in your catkin_ws directory and run the following command

		catkin_make
 
Make sure sphero is detected through bluetooth, but not connected


To run the sphero_color project
	launch the roscore

		roscore

	run the sphero_node node

		rosrun sphero_node sphero.py

	run the sphero_color node

		rosrun sphero_color sphero_color


To run the sphero_move project
	launch the roscore

		roscore

	run the sphero_node node

		rosrun sphero_node sphero.py

	run the sphero_opencv node

		rosrun sphero_move sphero_opencv

	run the sphero_move node

		rosrun sphero_move sphero_move
