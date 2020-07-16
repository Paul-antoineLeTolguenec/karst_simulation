#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Imu.h"
#include "nav_msgs/Odometry.h"
#include "std_srvs/Trigger.h"
#include "tf/tf.h"
#include <cmath>
#include <vector>
/* Tubex include */
#include <ibex.h>
#include <tubex.h>
#include <tubex-rob.h>
#include <tubex-3rd.h>

using namespace std;
using namespace ibex;
using namespace tubex;

/* This node is created in order to make a link between ros and tubex.
In this node we will plot the robot's coordinates into the plan xy  */
//vector initialisation
/* Voir simulation.launch pour les coordonnées initiales du robot */
IntervalVector map_area(2, Interval(-10.,10.));
float dt=0.05;
Vector X{{33.},{80.},{0.},{0.}};
Vector U(2);
/* TUBE */
Interval tdomain(0,500);                           
TubeVector x(tdomain, 4);                     

tubex::ContractorNetwork cn;       
tubex::ContractorNetwork add_data;






/* Function that deals with the true estimation */
Vector f(Vector X, Vector U){
	Vector Xp(4);
	Xp[0]=X[3]*cos(X[2]);
	Xp[1]=X[3]*sin(X[2]);
	Xp[2]=U[0];
	Xp[3]=0.;
	return(Xp);
}

/* Function called each new raw data */
void chatterCallback(const nav_msgs::Odometry::ConstPtr& msg)
{	
	//first let's make the true trajectory with the raw data
	U[0]=float(msg->twist.twist.angular.z);
	X[3]=float(sqrt(pow(msg->twist.twist.linear.x,2)+pow(msg->twist.twist.linear.y,2)));
	for ( int i = 0; i < X.size(); i++)
	{
		X[i]=float(X[i]+dt*f(X,U)[i]);
	}


}




int main(int argc, char **argv){
	ros::init(argc, argv, "IA_node");
	ros::NodeHandle n;
	ros::Subscriber imu = n.subscribe("/rexrov/pose_gt", 1000, chatterCallback);
	

	/* =========== GRAPHICS =========== */

    
    // Map : into a general frame
	vibes::beginDrawing();
    VIBesFigMap fig_map("Map");
    fig_map.set_properties(10, 10, 600, 600);
	fig_map.draw_vehicle(X.subvector(0,2), 1);
	fig_map.axis_limits(map_area);
	fig_map.show();


	ros::Rate loop_rate(1/dt);
	while (ros::ok()){
		map_area=IntervalVector(2, Interval(-10.,10.));
		map_area[0]=map_area[0]+X[0];
		map_area[1]=map_area[1]+X[1];
		vibes::clearFigure("Map");
		fig_map.draw_vehicle(X.subvector(0,2), 1);
		fig_map.axis_limits(map_area);
		fig_map.show();
		

		ros::spinOnce();
		loop_rate.sleep();
		
	}
	vibes::endDrawing();
	return 0;
}