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
#include <frontier_exploration_turtlebot/PathPlanning.h>
#include <../include/frontier_exploration_turtlebot/map.h>
#include <../include/frontier_exploration_turtlebot/nodeposition.h>
#include <../include/frontier_exploration_turtlebot/node.h>
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

/**
 * @brief      main function
 * @param      argc  The argc
 * @param      argv  The argv
 * @return     int of value zero
 */
int main(int argc, char* argv[]) {
  ROS_INFO("Welcome to Project 3!!");
  ROS_INFO(" The coordinate axis is changed "
      "from the center of the map to the bottom left corner.");
  Coord startpoint, endpoint;
  int x, y, u, v, rpm1, rpm2, theeta, choice;
  cout << "Enter the x coordinate of starting point (cm integer): ";
  cin >> x;
  cout << "Enter the y coordinate of starting point (cm integer) : ";
  cin >> y;
  cout << "Enter the x coordinate of End point (cm integer) : ";
  cin >> u;
  cout << "Enter the y coordinate of End point (cm integer) : ";
  cin >> v;
  cout << "Enter 0 to visualize node exploration (Time consuming), "
      "else Enter 1 to see map after node exploration : ";
  cin >> choice;

  rpm1 = 20;
  rpm2 = 40;
  theeta = 0;

  startpoint.set_x(x);
  startpoint.set_y(y);
  endpoint.set_x(u);
  endpoint.set_y(v);
  Nodeposition startposition;
  startposition.set_x(x);
  startposition.set_y(y);
  startposition.set_theeta(theeta);

  ROS_INFO("Building Map....");
  Map map(rpm1, rpm2, choice);
  ROS_INFO("Map Build !!");
  if (!map.takeinput(startpoint, endpoint)) {
    ROS_ERROR("Invalid Entry!! Press Enter to Exit...");
    cin.ignore();
    cin.get();
    return 0;
  }


  ROS_INFO("Exploring Map...");
  Node initiate;

  Node* root = initiate.solve(startposition, &map);
  ROS_INFO(" Map Explored!!");
  ROS_INFO(" Please launch gazebo"
      " in new terminal using undermentioned command :");
  ROS_INFO(" roslaunch frontier_exploration_turtlebot gazebo.launch");
  ROS_INFO(" Press Enter if gazebo is properly launched");
  cin.ignore();
  cin.get();

  initiate.set_turtlebot_ingazebo(x, y);
  ros::init(argc, argv, "frontierExplorer");
  PathPlanning pathPlanning;
  Node* gazebo = root;

  while (ros::ok()) {
    if (gazebo->tracechild != NULL) {
      pathPlanning.setangularspeed
      (gazebo->position.get_theeta(),
       gazebo->tracechild->position.get_theeta(),
       gazebo->gridmap->getdelta_t());


      pathPlanning.setlinearspeed
      (gazebo->position.get_x(), gazebo->position.get_y(),
       gazebo->tracechild->position.get_x(),
       gazebo->tracechild->position.get_y(),
       gazebo->gridmap->getdelta_t());
      pathPlanning.PathGenerator();
      gazebo = gazebo->tracechild;
    } else {
      pathPlanning.setangularspeed(0, 0, 1);
      pathPlanning.setlinearspeed(0, 0, 0, 0, 1);
      pathPlanning.PathGenerator();
      ROS_INFO("Press Enter twice to Exit...");
      cin.ignore();
      cin.get();
      return 0;
    }
  }
}
