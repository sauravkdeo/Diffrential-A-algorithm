/**
 * @file      include/map.h
 * @brief     Header file for building map
 * @author    Saurav Kumar
 * @copyright 2019
 *
 **BSD 3-Clause License
 *
 *Copyright (c) 2018, Saurav Kumar
 *All rights reserved.
 *
 *Redistribution and use in source and binary forms, with or without
 *modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 *  and/or other materials provided with the distribution.
 *
 * Neither the name of the copyright holder nor the names of its
 *  contributors may be used to endorse or promote products derived from
 *  this software without specific prior written permission.
 *
 *THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef INCLUDE_DIFFRENTIAL_TURTLEBOT_NODE_H_
#define INCLUDE_DIFFRENTIAL_TURTLEBOT_NODE_H_

#include <ros/ros.h>
#include <../include/diffrential_turtlebot/PathPlanning.h>
#include <../include/diffrential_turtlebot/map.h>
#include <../include/diffrential_turtlebot/nodeposition.h>
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

class Node {
 public:
  // stores parent node of current node
  // helps in tracing path when the goal is achieved
  Nodeposition position;

  // stores the number of moves so far
  double costtocome;
  double costtogo;

  Node* parent;

  Node* child01;
  Node* child02;
  Node* child12;
  Node* child11;
  Node* child22;
  Node* child21;
  Node* child20;
  Node* child10;
  Node* tracechild;
  Map * gridmap;

  struct comp {
    bool operator()(const Node* lhs, const Node* rhs) const {
      return (lhs->costtocome + lhs->costtogo) >
      (rhs->costtocome + rhs->costtogo);
    }
  };
  double calculateastarcosttogo(Node*, Coord);
  double getstepcost(int, int, Map*);
  Node* newnode(Nodeposition, Node*, double,
                std::map<int, std::map<int,
                std::map<int, Node*> > >*, Map*,
                priority_queue<Node*, std::vector<Node*>, comp>*);
  void traceback(Node*);
  void printtrace(Node*, Node*);
  double getcurveradius(Map*, int, int);
  Nodeposition getnewnodeposition(Node*, Map*, int, int);
  bool move(Node*, Map*, int, int);
  bool nearvicinity(Node*, Map*);
  void explorechild(Node*,  priority_queue<Node*,
                    std::vector<Node*>, comp>*,
                    std::map<int, std::map<int, std::map<int, Node*> > >*,
                    Map*);
  Node* solve(Nodeposition, Map*);
  std::string to_string(double);
  void set_turtlebot_ingazebo(int, int);
};
#endif  // INCLUDE_DIFFRENTIAL_TURTLEBOT_NODE_H_

