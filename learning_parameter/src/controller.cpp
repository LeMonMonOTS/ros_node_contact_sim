// AUTHOR: LeMonMOn
// UPDATE: 2021/01/08
// UPDATE_NOTE: Add dynamic parameter.
// DESCRIPTION: This code is to simulating "nodes" movements in 2 dimensions spaces with
// random-walking. If there are 2 nodes distanced less than MAX_DIST, they will add each
// other to their own neigh_list. The nodes periodicly publish thier id and position to
// the controller, and by the way request a service to find out its neighbour. And the 
// controller collects information and judge the neighbours. This is the controller code.
// Note: Make sure this node running after ALL the nodes created. If you want to add new
// nodes, you should re-run all the programs.


#include "ros/ros.h"
// TO USE THIS, YOU SHOULD CONFIG THE INCLUDE PATH WITH YOUR SYSTEM
#include "/home/lemonmon/catkin_ws/src/learning_parameter/src/Node.h"

#include "learning_parameter/node_info.h"
#include "learning_parameter/chk.h"
#include "dynamic_reconfigure/server.h"
#include "learning_parameter/try_dyparamConfig.h"

#include <sstream>
#include <vector>


double MAX_DIST2 = 99999;


std::vector<int> id_table;
std::vector<double> pos_x_table;
std::vector<double> pos_y_table;


void sub_callback(const learning_parameter::node_info::ConstPtr& msg){
  std::vector<double> rec_data = msg->ad_info;
  int rec_id = (int)rec_data[0];
  double rec_x = rec_data[1], rec_y = rec_data[2];
  // std::cout << "received:" << "(" << rec_id << ", " << rec_x << ", " << rec_y << ")" << std::endl;
  // update pos_table
  bool is_in_vec = false;
  for (int i = 0; i < id_table.size(); i++){
    if (id_table[i] == rec_id){
      id_table[i] = rec_id;
      pos_x_table[i] = rec_x;
      pos_y_table[i] = rec_y;
      is_in_vec = true;
      break;
    }
  }
  // add pos_table
  if (!is_in_vec){
    id_table.push_back(rec_id);
    pos_x_table.push_back(rec_x);
    pos_y_table.push_back(rec_y);
  }
}


double cal_dist2(double x1, double y1, double x2, double y2){
  return (x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2);
}


bool check_callback(learning_parameter::chk::Request& req,
              learning_parameter::chk::Response& res){
  // find corresponding index
  std::cout << "Now client: " << req.node_id << std::endl;
  for (int i = 0; i < id_table.size(); i++){
    std::cout << id_table[i] << " " << pos_x_table[i] << " " << pos_y_table[i] << std::endl;
  } 
  std::cout << "-------------------" << std::endl;

  int cos_index = -1;
  for (int i = 0; i < id_table.size(); i++){
    if (id_table[i] == req.node_id){
      cos_index = i;
      break;
    }
  }
  if (cos_index == -1){
    std::cout << "ERROR: cos_index not found!" << std::endl;
    return false;
  }

  // calculate and judge distance^2, then add the id to the res vector
  for (int i = 0; i < id_table.size(); i++){
    double x1, y1, x2, y2;
    x1 = pos_x_table[cos_index];
    y1 = pos_y_table[cos_index];
    x2 = pos_x_table[i];
    y2 = pos_y_table[i];

    if (cal_dist2(x1, y1, x2, y2) < MAX_DIST2){
      res.neigh_table.push_back(id_table[i]);
    }
  }
  return true;
}


void para_callback(learning_parameter::try_dyparamConfig& config, uint32_t level){
  MAX_DIST2 = config.double_param;
  std::cout << "Now max_distance^2=" << config.double_param << std::endl; 
}


int main(int argc, char** argv){
  ros::init(argc, argv, "controller");

  ros::NodeHandle n;
  
  int node_num;
  ros::param::get("node_num", node_num);

  std::vector<ros::Subscriber> subs;
  std::vector<ros::ServiceServer> clts;

  for (int i = 1; i <= node_num; i++){
    std::stringstream tpc_name, srv_name;
    tpc_name << "Node" << i << "_tpc";
    srv_name << "Node" << i << "_srv";

    subs.push_back(n.subscribe(tpc_name.str(), 1000, sub_callback));
    clts.push_back(n.advertiseService(srv_name.str(), check_callback));
  }

  dynamic_reconfigure::Server<learning_parameter::try_dyparamConfig> para_ser;
  dynamic_reconfigure::Server<learning_parameter::try_dyparamConfig>::CallbackType para_f;

  para_f = boost::bind(&para_callback, _1, _2);
  para_ser.setCallback(para_f);

  ros::spin();

  return 0;
}
