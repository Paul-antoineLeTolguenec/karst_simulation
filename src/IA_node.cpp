#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/LaserScan.h"
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
IntervalVector map_area(2, Interval(-30.,30.));
double dt=0.05;
double date=0.;
Vector X{{33.},{80.},{0.},{0.}};
Vector U(2);
/* TUBE */
Interval tdomain(0,20);    
IntervalVector Xv{{33.},{80.},{0.},{0.}};  
IntervalVector Uv(2);                
TubeVector x(tdomain,dt, 4);                     
tubex::ContractorNetwork cn;       
tubex::ContractorNetwork add_data;

/* OBSERVATIONS */
vector<vector<IntervalVector>> Map;
vector<IntervalVector> Obs_t;
pyibex::CtcPolar ctc_polar;




/* Function that deals with the true estimation */
Vector f(Vector X, Vector U){
	Vector Xp(4);
	Xp[2]=U[0];
	Xp[3]=0.;
	Xp[0]=X[3]*cos(X[2]);
	Xp[1]=X[3]*sin(X[2]);
	return(Xp);
}
IntervalVector fv(IntervalVector Xv, IntervalVector Uv){
	IntervalVector Xpv(4);
	Xpv[2]=Uv[0];
	Xpv[3]=0.;
	Xpv[0]=Xv[3]*cos(Xv[2]);
	Xpv[1]=Xv[3]*sin(Xv[2]);
	return(Xpv);
}

/* Function called each new imu data */
void chatterCallbackImu(const nav_msgs::Odometry::ConstPtr& msg)
{	date+=dt;
	//first let's make the true trajectory with the raw data
	U[0]=float(msg->twist.twist.angular.z);
	X[3]=float(sqrt(pow(msg->twist.twist.linear.x,2)+pow(msg->twist.twist.linear.y,2)));
	Uv[0]=float(msg->twist.twist.angular.z)/* +Interval(-M_PI/720.,M_PI/720.) */;
	Xv[3]=float(sqrt(pow(msg->twist.twist.linear.x,2)+pow(msg->twist.twist.linear.y,2)))/* +Interval(-0.0001,0.0001) */;
	for ( int i = 0; i < X.size(); i++)
	{
		X[i]=float(X[i]+dt*f(X,U)[i]);
		Xv[i]=Xv[i]+dt*fv(Xv,Uv)[i];
	}
	Xv[0].inflate(0.0002);
	Xv[1].inflate(0.0002);
	Xv[2].inflate(0.000001);
	/* cout<<date<<endl;
	cout<<x(max(0.,ibex::previous_float(date))).subvector(0,1)<<endl;
	cout<<"la"<<endl;
	cn.add_data(x,date,Xv);
	cout<<"ici"<<endl;
	cn.contract_during(dt);
	cout<<"pa"<<endl; */
}


void chatterCallbackSonar(const sensor_msgs::LaserScan::ConstPtr& msg)
{	vector<float> ranges;
	float angle_min;
	float angle_max ;
	float angle_increment;
	float range_min;
	float range_max ;
	angle_min=msg->angle_min;
	angle_max=msg->angle_max;
	angle_increment=msg->angle_increment;
	range_min=msg->range_min;
	range_max=msg->range_max;
	ranges=msg->ranges;
	vector<IntervalVector> v_b(ranges.size());
	Obs_t.clear();
	Interval theta;
	Interval rho;
	int i=0;
    for (auto& obs : v_b)
      { obs=IntervalVector(2);
        theta=Interval();
        theta=Xv[3]+Interval(float(angle_min+i*angle_increment));
		if (isinf(ranges[i]))
		{
			rho=Interval();
		}
		else
		{
			rho=Interval(ranges[i]);
		}
        ctc_polar.contract(obs[0], obs[1], rho, theta);
		obs[0]=obs[0]+Xv[0];
		obs[1]=obs[1]+Xv[1];
		Obs_t.push_back(obs);
        i++;
        }
	
	
	
	
}


int main(int argc, char **argv){
	ros::init(argc, argv, "IA_node");
	ros::NodeHandle n;
	ros::Subscriber imu = n.subscribe("/rexrov/pose_gt", 1000, chatterCallbackImu);
	ros::Subscriber sonar = n.subscribe("/rexrov/sonar", 1000, chatterCallbackSonar);
	

    // Map : into a general frame
	vibes::beginDrawing();
    VIBesFigMap fig_map("Map");
    fig_map.set_properties(10, 10, 600, 600);
	/* fig_map.add_tube(&x, "x", 0, 1); */
	/* fig_map.draw_vehicle(X.subvector(0,2), 1);
	fig_map.axis_limits(map_area);
	fig_map.show();
 */

	ros::Rate loop_rate(1/dt);
	while (ros::ok()){
		map_area=IntervalVector(2, Interval(-30.,30.));
		map_area[0]=map_area[0]+X[0];
		map_area[1]=map_area[1]+X[1];
		vibes::clearFigure("Map");
		/* fig_map.add_tube(&x, "x", 0, 1); */
		/* fig_map.draw_vehicle(X.subvector(0,2), 1); */
		for(auto& obs : Obs_t){
		fig_map.draw_box(obs,"red");} 

		fig_map.draw_box(Xv.subvector(0,1),"black");
		fig_map.axis_limits(map_area);
		fig_map.show();
		

		ros::spinOnce();
		loop_rate.sleep();
		
	}
	vibes::endDrawing();
	return 0;
}