#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "/home/lemonmon/catkin_ws/src/learning_parameter/src/Node.h"

#include "learning_parameter/chk.h"

#include <sstream>
#include <vector>


#define MAX_DIST2 4


std::vector<int> id_table;
std::vector<double> pos_x_table;
std::vector<double> pos_y_table;


void sub_callback(const std_msgs::Float64MultiArray::ConstPtr& msg){
  std::vector<double> rec_data = msg->data;
  int rec_id = (int)rec_data[0];
  double rec_x = rec_data[1], rec_y = rec_data[2];
  //std::cout << "received:" << "(" << rec_id << ", " << rec_x << ", " << rec_y << ")" << std::endl;
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
/*
  for (int i = 0; i < id_table.size(); i++){
    std::cout << id_table[i] << " " << pos_x_table[i] << " " << pos_y_table[i] << std::endl;
  } 
  std::cout << "-------------------" << std::endl;
*/
}


double cal_dist2(double x1, double y1, double x2, double y2){
  return (x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2);
}


bool check_callback(learning_parameter::chk::Request& req,
              learning_parameter::chk::Response& res){
  // find corresponding index
  std::cout << req.node_id << std::endl;
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

  ros::spin();

  return 0;
}
