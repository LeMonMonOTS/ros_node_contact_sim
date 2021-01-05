#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <vector>
#include "/home/lemonmon/catkin_ws/src/learning_parameter/src/Node.h"

#define RANDOM(a, b) (a + (b - a)*(double)std::rand()/RAND_MAX)


int main(int argc, char** argv){
  ros::init(argc, argv, "Node2");
  
  ros::NodeHandle n;

  ros::Publisher pub = n.advertise<std_msgs::String>("Node2_ad", 1000);

  int Node_id = 2;
  double x_vel, y_vel;
  std::vector<double> pos(2, 0), new_pos(2, 0);
  Node my_Node(Node_id, pos);


  ros::Rate loop_rate(2);
  while (ros::ok()){
    x_vel = RANDOM(-1, 1);
    y_vel = RANDOM(-1, 1);
    new_pos = my_Node.get_pos();
    new_pos[0] += x_vel;
    new_pos[1] += y_vel;
    my_Node.set_pos(new_pos);

    std_msgs::String msg;
    std::stringstream ss;
    ss << my_Node;
    msg.data = ss.str();

    pub.publish(msg);
    
    std::cout << msg.data << std::endl;
    
    loop_rate.sleep();
  }


  return 0;
}