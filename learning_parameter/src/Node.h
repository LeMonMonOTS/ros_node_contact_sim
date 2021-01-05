#ifndef __Node__H__
#define __Node__H__
#include <iostream>
#include <vector>


//<------------------------预声明--------------------------->
class Node;


//<------------------------声明类---------------------------->
// Node类
class Node{
  public: 
    Node();
    Node(int);
    Node(int, double, double);

    void set_pos_x(double);
    void set_pos_y(double);
    void set_neigh_list(std::vector<int>&);

    const int get_Node_id();
    const double get_pos_x();
    const double get_pos_y();

    const std::vector<int> get_neigh_list();

    friend std::ostream& operator<<(std::ostream&, Node&);
  private:
    int Node_id;
    double x;
    double y;
    std::vector<int> neigh_list;
};


//<------------------------定义类---------------------------->
//-------------------------构造函数---------------------------
Node::Node()
:Node_id(0)
,x(0)
,y(0)
{
  this->neigh_list.push_back(this->Node_id);
}


Node::Node(int id)
:Node_id(id)
,x(0)
,y(0)
{
  this->neigh_list.push_back(this->Node_id);
}


Node::Node(int id, double pos_x, double pos_y)
:Node_id(id)
,x(pos_x)
,y(pos_y)
{
  this->neigh_list.push_back(this->Node_id);
}


//-------------------------set类函数---------------------------
void Node::set_pos_x(double pos_x){
  this->x = pos_x;
}


void Node::set_pos_y(double pos_y){
  this->y = pos_y;
}


void Node::set_neigh_list(std::vector<int>& new_neigh){
  this->neigh_list = new_neigh;
}


//-------------------------get类函数---------------------------
const int Node::get_Node_id(){
  return this->Node_id;
}


const double Node::get_pos_x(){
  return this->x;
}


const double Node::get_pos_y(){
  return this->y;
}


const std::vector<int> Node::get_neigh_list(){
  return this->neigh_list;
}


//-------------------------运算符重载-----------------------------
std::ostream& operator<<(std::ostream& os, Node& n){
  os << "Node_id=" << n.Node_id << ", Node_pos=" << "(" << n.x << "," << n.y << ")";
  return os;
}

#endif
