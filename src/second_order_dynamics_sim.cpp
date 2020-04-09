#include "ros/ros.h"
#include "std_msgs/Float64.h"

using namespace std;
struct Dynamic_State {
  float last;
  float current;
};

int main(int argc, char **argv)
{
  ros::init(argc,argv,"second_order_dynamics_sim");
	ros::NodeHandle nh;

  // declare publisher
  ros::Publisher position_pub = nh.advertise<std_msgs::Float64>("/position",10);
  ros::Publisher velocity_pub = nh.advertise<std_msgs::Float64>("/velocity",10);
  ros::Publisher acc_pub = nh.advertise<std_msgs::Float64>("/acc",10);

  // Parameters initialization
  float M = 2;                              // mass
  float k = 1;                              // spring constant
  float b = 0.707;                          // damping coefficient
  float u = 1;                              // control input is unit step function
  float x1_dot = 0;                         // time derivative of x1 (velocity)
  float x2_dot = 0;                         // time derivative of x2 (acceleration)
  struct Dynamic_State x2;                  // velocity
  struct Dynamic_State x1;                  // position
  x1.current = 0;
  x1.last = 0;
  x2.current = 0;
  x2.last = 0;
	ros::Rate loop_rate(10);
	double past = ros::Time::now().toSec();
  double dt;                                //sampling time
	std_msgs::Float64 position;
  std_msgs::Float64 velocity;
  std_msgs::Float64 acc;

  // Discrete-Time Linear State-Space
	while(ros::ok()) {
		double now = ros::Time::now().toSec();

    // calculate time derivative of state
		dt = now - past;
    x1_dot = x2.current;
    x2_dot = (-k/M)*x1.current + (-b/M)*x2.current + u;

    // calculate new state by time derivative of state
    x1.current = x1.current + x1_dot*dt;
    x2.current = x2.current + x2_dot*dt;
    position.data = x1.current;
    velocity.data = x2.current;
    acc.data = x2_dot;

    // publish
    position_pub.publish(position);
    velocity_pub.publish(velocity);
    acc_pub.publish(acc);

    past = now;

		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
