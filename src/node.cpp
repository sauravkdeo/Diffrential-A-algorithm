/**
 *  MIT License
 *
 *  Copyright (c) 2018 Saimouli Katragadda, Saurav Kumar
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a
 *  copy of this software and associated documentation files (the "Software"),
 *  to deal in the Software without restriction, including without
 *  limitation the rights to use, copy, modify, merge, publish, distribute,
 *  sublicense, and/or sell copies of the Software, and to permit persons to
 *  whom the Software is furnished to do so, subject to the following
 *  conditions:
 *
 *  The above copyright notice and this permission notice shall be included
 *  in all copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 *  THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 *  FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 *  DEALINGS IN THE SOFTWARE.
 */

/**
 *@file         main.cpp
 *@author       Saurav Kumar
 *@copyright    MIT License
 *@brief        main function
 */

#include <ros/ros.h>
#include <../include/diffrential_turtlebot/node.h>
#include <map>
#include <utility>
#include <algorithm>
#include <string>
#include <queue>
#include <vector>
#include <iostream>


using std::cout;
using std::endl;
using std::cin;
using std::vector;
using std::priority_queue;

// completed
double Node::calculateastarcosttogo(Node* node, Coord goal) {
  return (sqrt(pow((node->position.get_x()-goal.get_x()), 2) +
               pow((node->position.get_y()-goal.get_y()), 2)));
}

struct comp {
  bool operator()(const Node* lhs, const Node* rhs) const {
    return (lhs->costtocome + lhs->costtogo) >
    (rhs->costtocome + rhs->costtogo);
  }
};

double Node::getstepcost(int leftwheelrpm, int rightwheelrpm, Map* map) {
  double stepcost = (rightwheelrpm+leftwheelrpm)*
      M_PI*map->getwheelradius()*map->getdelta_t()/60;
  return stepcost;
}

Node* Node::newnode(Nodeposition nodeposition, Node* parent, double stepCost,
              std::map<int, std::map<int,
              std::map<int, Node*> > >* ngridmap, Map* arraymap,
              priority_queue<Node*, std::vector<Node*>, comp>* pq) {
  if ((*ngridmap)[nodeposition.get_x()][nodeposition.get_y()]
                                        [nodeposition.get_theeta()]) {
    if (((*ngridmap)
        [nodeposition.get_x()][nodeposition.get_y()]
                               [nodeposition.get_theeta()]->costtocome) >
    (parent->costtocome+stepCost)) {
      (*ngridmap)
        [nodeposition.get_x()]
         [nodeposition.get_y()]
          [nodeposition.get_theeta()]->parent
          = parent;
      (*ngridmap)
        [nodeposition.get_x()]
         [nodeposition.get_y()]
          [nodeposition.get_theeta()]->costtocome =
              parent->costtocome+stepCost;
    }
  }
  if (!(*ngridmap)
      [nodeposition.get_x()]
       [nodeposition.get_y()][nodeposition.get_theeta()]) {
    Node* node = new Node;
    node->position = nodeposition;
    node->gridmap = arraymap;
    node->parent = parent;

    if (parent == NULL) {
      node->costtocome = stepCost;
    }
    if (parent != NULL) {
      node->costtocome = parent->costtocome+stepCost;
    }

    node->costtogo = sqrt
        (pow((nodeposition.get_x()-node->gridmap->getendcoord().get_x()), 2) +
         pow((nodeposition.get_y()-node->gridmap->getendcoord().get_y()), 2));
    if (node->gridmap->getgridmap
        (nodeposition.get_x(), nodeposition.get_y()) != 3) {
      node->gridmap->fillmap(nodeposition.get_x(), nodeposition.get_y(), 3);
    }
    (*pq).push(node);

    (*ngridmap)
        [nodeposition.get_x()]
         [nodeposition.get_y()]
          [nodeposition.get_theeta()] = node;
  }
  return (*ngridmap)
      [nodeposition.get_x()][nodeposition.get_y()][nodeposition.get_theeta()];
}




void Node::traceback(Node* trace) {
  if (trace->parent != NULL) {
    trace->parent->tracechild = trace;
    traceback(trace->parent);
  }
}

void Node::printtrace(Node* node, Node* end) {
  if (node != end) {
    node->gridmap->setchoice(0);
    node->gridmap->updatemap(node->position.get_x(),
                             node->position.get_y(),
                             node->tracechild->position.get_x(),
                             node->tracechild->position.get_y(), 5);
    printtrace(node->tracechild, end);
  }
}



double Node::getcurveradius(Map* map, int leftwheelrpm, int rightwheelrpm ) {
  if (rightwheelrpm != leftwheelrpm) {
    double curveradius =
        (map->getwheelgap()/2)*
        (leftwheelrpm+rightwheelrpm)/(rightwheelrpm- leftwheelrpm);
    return curveradius;
  } else {
    return 0;
  }
}


Nodeposition Node::getnewnodeposition
(Node* node, Map* map, int leftwheelrpm, int rightwheelrpm) {
  Nodeposition newnodeposition;
  int delta_theeta = map->getwheelradius()*map->getdelta_t()*
      (rightwheelrpm-leftwheelrpm)*6/(map->getwheelgap());
  int newtheeta = node->position.get_theeta()+delta_theeta;
  if (node->position.get_theeta()+delta_theeta >= 360) {
    newtheeta = newtheeta-360;
  }
  if (node->position.get_theeta()+delta_theeta < 0) {
    newtheeta = 360+newtheeta;
  }
  int delta_x = map->getwheelradius()*map->getdelta_t()*
      (leftwheelrpm+rightwheelrpm)*
      cos(newtheeta*M_PI/180)*M_PI/60;
  int delta_y = map->getwheelradius()*map->getdelta_t()*
      (leftwheelrpm+rightwheelrpm)*
      sin(newtheeta*M_PI/180)*M_PI/60;
  newnodeposition.set_x(node->position.get_x()+delta_x);
  newnodeposition.set_y(node->position.get_y()+delta_y);

  newnodeposition.set_theeta(newtheeta);
  return newnodeposition;
}

bool Node::move(Node* node, Map* arraymap, int leftrpm, int rightrpm) {
  Nodeposition np = getnewnodeposition(node, arraymap, leftrpm, rightrpm);
  if (np.get_x() >= 0 && np.get_x() < arraymap->getgridmapwidth() &&
      np.get_y() >= 0 && np.get_y() < arraymap->getgridmapheight()) {
    if (arraymap->getgridmap(np.get_x(), np.get_y()) == 1) {
      return true;
    } else {
      return false;
    }
  } else {
    return false;
  }
}
bool Node::nearvicinity(Node* node, Map* arraymap) {
  if ((pow((node->position.get_x()-arraymap->getendcoord().get_x()), 2)
      +pow((node->position.get_y()-arraymap->getendcoord().get_y()), 2))
      <= pow((arraymap->getwheelgap()/2), 2)) {
    return true;
  } else {
    return false;
  }
}

void Node::explorechild(Node* root,  priority_queue<Node*,
                  std::vector<Node*>, comp>* pq,
                  std::map<int, std::map<int, std::map<int, Node*> > >* ngmap,
                  Map* arraymap) {
  double stepcost;
  Nodeposition newlocation;
  if (move(root, arraymap, 0, arraymap->getrpm1())) {
    stepcost = getstepcost(0, arraymap->getrpm1(), arraymap);
    newlocation = getnewnodeposition(root, arraymap, 0, arraymap->getrpm1());
    root->child01 = newnode(newlocation, root, stepcost, ngmap, arraymap, pq);
  } else {
    root->child01 = NULL;
  }
  if (move(root, arraymap, 0, arraymap->getrpm2())) {
    stepcost = getstepcost(0, arraymap->getrpm2(), arraymap);
    newlocation = getnewnodeposition(root, arraymap, 0, arraymap->getrpm2());
    root->child02 =
        newnode(newlocation, root, stepcost, ngmap, arraymap, pq);
  } else {
    root->child02 = NULL;
  }
  if (move(root, arraymap, arraymap->getrpm1(), arraymap->getrpm2())) {
    stepcost = getstepcost(arraymap->getrpm1(), arraymap->getrpm2(), arraymap);
    newlocation = getnewnodeposition
        (root, arraymap, arraymap->getrpm1(), arraymap->getrpm2());
    root->child12 =
        newnode(newlocation, root, stepcost, ngmap, arraymap, pq);
  } else {
    root->child12 = NULL;
  }
  if (move(root, arraymap, arraymap->getrpm1(), arraymap->getrpm1())) {
    stepcost = getstepcost(arraymap->getrpm1(), arraymap->getrpm1(), arraymap);
    newlocation = getnewnodeposition
        (root, arraymap, arraymap->getrpm1(), arraymap->getrpm1());
    root->child11 =
        newnode(newlocation, root, stepcost, ngmap, arraymap, pq);
  } else {
    root->child11 = NULL;
  }
  if (move(root, arraymap, arraymap->getrpm2(), arraymap->getrpm2())) {
    stepcost = getstepcost(arraymap->getrpm2(), arraymap->getrpm2(), arraymap);
    newlocation = getnewnodeposition
        (root, arraymap, arraymap->getrpm2(), arraymap->getrpm2());
    root->child22 =
        newnode(newlocation, root, stepcost, ngmap, arraymap, pq);
  } else {
    root->child22 = NULL;
  }
  if (move(root, arraymap, arraymap->getrpm2(), arraymap->getrpm1())) {
    stepcost = getstepcost(arraymap->getrpm2(), arraymap->getrpm1(), arraymap);
    newlocation = getnewnodeposition
        (root, arraymap, arraymap->getrpm2(), arraymap->getrpm1());
    root->child21 =
        newnode(newlocation, root, stepcost, ngmap, arraymap, pq);
  } else {
    root->child21 = NULL;
  }
  if (move(root, arraymap, arraymap->getrpm1(), 0)) {
    stepcost = getstepcost(arraymap->getrpm1(), 0, arraymap);
    newlocation = getnewnodeposition
        (root, arraymap, arraymap->getrpm1(), 0);
    root->child10 =
        newnode(newlocation, root, stepcost, ngmap, arraymap, pq);
  } else {
    root->child10 = NULL;
  }
  if (move(root, arraymap, arraymap->getrpm2(), 0)) {
    stepcost = getstepcost(arraymap->getrpm2(), 0, arraymap);
    newlocation = getnewnodeposition
        (root, arraymap, arraymap->getrpm2(), 0);
    root->child20 =
        newnode(newlocation, root, stepcost, ngmap, arraymap, pq);
  } else {
    root->child20 = NULL;
  }
}

Node* Node::solve(Nodeposition position, Map* mp) {
  std::map<int, std::map<int, std::map<int, Node*> > > ngm;
  priority_queue<Node*, std::vector<Node*>, comp> pq;
  // create a root node and calculate its cost
  Node* root = newnode(position, NULL, 0.0, &ngm, mp, &pq);


  while (!pq.empty()) {
    // Find a live node with least estimated cost
    Node* min = pq.top();

    // The found node is deleted from the list of
    // live nodes
    pq.pop();

    // if min is an answer node
    if (nearvicinity(min, mp)) {
      min->tracechild = NULL;
      traceback(min);
      printtrace(root, min);
      return root;
    }
    explorechild(min, &pq, &ngm, mp);
  }
  if (pq.empty()) {
    cout << "No Solution !!";
    return NULL;
  }
  return NULL;
}

std::string Node::to_string(double x) {
  std::ostringstream ss;
  ss << x;
  return ss.str();
}

void Node::set_turtlebot_ingazebo(int x, int y) {
  double gazebo_x , gazebo_y;
  gazebo_x = static_cast<double>((x-555))/100;
  gazebo_y = static_cast<double>((y-505))/100;
  std::string g_x = to_string(gazebo_x);
  std::string g_y = to_string(gazebo_y);

  std::string first = "rosservice call /gazebo/set_model_state '{model_state: "
      "{ model_name: turtlebot3_waffle, "
      "pose: { position: { x: ";
  std::string middle = ", y: ";
  std::string last = " , z: 0.0 },"
      " orientation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0 } },"
      " twist: { linear: {x: 0.0 , y: 0.0 ,z: 0.0 } ,"
      " angular: { x: 0.0 , y: 0.0 , z: 0.0 } } ,"
      " reference_frame: world } }'";
  std::string cmd = first+g_x+middle+g_y+last;
  const char* command = cmd.c_str();
  ROS_INFO("Valid Starting and End points. "
      "Spawning turtlebot to user-defined location!!");
  system(command);
  cv::waitKey(2000);
}
