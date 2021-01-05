#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "/home/lemonmon/catkin_ws/src/learning_parameter/src/Node.h"

#include "learning_parameter/chk.h"

#include <sstream>
#include <string>
#include <vector>


#define RANDOM(a, b) (a + (b - a)*(double)std::rand()/RAND_MAX)


int main(int argc, char** argv){
  if (argc != 2){
    std::cout << "please enter a node_name." << std::endl;
    return 1;
  }

  int Node_id = std::stoi(argv[1]);
  std::stringstream node_name, ad_name, srv_name;
  node_name << "Node" << Node_id;
  ad_name << "Node" << Node_id << "_tpc";
  srv_name << "Node" << Node_id << "_srv";
  
  ros::init(argc, argv, node_name.str());

  if (!ros::param::has("node_num")){
    ros::param::set("node_num", 1);
    std::cout << "set node_num=1" << std::endl;
  }else{
    int node_num;
    ros::param::get("node_num", node_num);
    ros::param::set("node_num", ++node_num);
    std::cout << "set node_num=" << node_num << std::endl;
  }

  ros::NodeHandle n;
  ros::ServiceClient clt = n.serviceClient<learning_parameter::chk>(srv_name.str());
  ros::Publisher pub = n.advertise<std_msgs::Float64MultiArray>(ad_name.str(), 1000);

  learning_parameter::chk srv;
  srv.request.node_id = Node_id;

  double x_vel, y_vel;
  Node my_Node(Node_id);

  ros::Rate loop_rate(1);
  while (ros::ok()){
    srand(Node_id*Node_id + signed(time(NULL)));
    x_vel = RANDOM(-1, 1);
    srand(Node_id*Node_id + (signed int)(time(NULL)/2));
    y_vel = RANDOM(-1, 1);
    my_Node.set_pos_x(my_Node.get_pos_x() + x_vel);
    my_Node.set_pos_y(my_Node.get_pos_y() + y_vel);
    std::vector<double> now_pos{(double)my_Node.get_Node_id(), my_Node.get_pos_x(), my_Node.get_pos_y()};

    std_msgs::Float64MultiArray pos_info;
    pos_info.data = now_pos;

    pub.publish(pos_info);
    

    std::cout << my_Node << std::endl;
    srv.response.neigh_table.clear();
    if (clt.call(srv)){
      my_Node.set_neigh_list(srv.response.neigh_table);
      std::cout << "successfully update neigh" << std::endl;
    }else{
      std::cout << "failed to update neigh" << std::endl;
    }

    // print neigh
    for (int i = 0; i < my_Node.get_neigh_list().size(); i++){
      std::cout << my_Node.get_neigh_list()[i] << ",";
    }
    std::cout << std::endl;


    loop_rate.sleep();
  }

  return 0;
}


  /* not working code...
  if (!ros::param::has("node_num")){
    ros::param::set("node_num", 1);
    std::cout << "set node_num=1" << std::endl;
  }else{
    ros::param::get("node_num", Node_id);
    ros::param::set("node_num", ++Node_id);
    std::cout << "set node_num=" << Node_id << std::endl;
  }
  */