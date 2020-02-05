
/**
 * @file      include/map.cpp
 * @brief     cpp file for building map
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



#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <../include/diffrential_turtlebot/map.h>
#include <math.h>
#include <algorithm>
#include <iostream>
#include <vector>


using std::ofstream;

Map::Map(int rpm_1, int rpm_2, int opt):
                rpm1(rpm_1), rpm2(rpm_2), choice(opt) {
  robotradius = 22;
  clearance = 10;
  wheelradius = 3.3;
  wheelegap = 28.7;
  delta_t = 1;
  gridmapheight = 1011;
  gridmapwidth = 1111;


  gridmap.resize(gridmapwidth);
  for (int i = 0; i < gridmapwidth; i++) {
    gridmap[i].resize(gridmapheight);
  }

  for (int i = 0; i < gridmapwidth; i++) {
    for (int j = 0; j < gridmapheight; j++) {
      gridmap[i][j]= 1;
    }
  }
  frame = cv::Mat(gridmapheight, gridmapwidth,
                  CV_8UC3, cv::Scalar(255, 255, 255));
  buildmap();
}
Map::~Map() {
}
void Map::definepolygonobstacle(int n, vector<vector<int>> A) {
  if (n > 2) {
    vector<vector<double>> B(n, std::vector<double>(3));
    double theeta, slope_theeta;
    int count;
    double c;
    for (int i = 0; i <= (n-1); i++) {
      if (i < (n-1)) {
        theeta = atan2((A[i+1][1]-A[i][1]), (A[i][0]-A[i+1][0]));
        slope_theeta =  atan2((A[i+1][1]-A[i][1]), (A[i+1][0]-A[i][0]));
        c = (sin(theeta)* A[i][0]) + (cos(theeta) * A[i][1]);
      }
      if (i == (n-1)) {
        theeta = atan2((A[0][1]-A[i][1]), (A[i][0]-A[0][0]));
        slope_theeta =  atan2((A[0][1]-A[i][1]), (A[0][0]-A[i][0]));
        c = (sin(theeta)* A[i][0]) + (cos(theeta) * A[i][1]);
      }
      B[i][0] = theeta;
      B[i][1] = c;
      B[i][2] = slope_theeta;
    }

    for (int j = 0; j < gridmapheight; j++) {
      for (int i = 0; i < gridmapwidth; i++) {
        count = 0;
        for (int k = 0; k <= n-1; k++) {
          if ((sin(B[k][0]) * i+cos(B[k][0]) * j)-B[k][1] <= 0) {
            count = count+1;
          }
          if (count == n) {
            gridmap[i][j]= -1;
          }
        }
      }
    }
    formobstacleboundary();
    joinconcaveshapes();
  }
}

void Map::definecircleobstacle(Coord center, int r) {
  for (int j = 0; j < gridmapheight; j++) {
    for (int i = 0; i < gridmapwidth; i++) {
      if (gridmap[i][j] > 0) {
        if (((i-center.get_x())*(i-center.get_x()))
            +((j-center.get_y())*(j-center.get_y()))-(r*r) <= 0) {
          gridmap[i][j] = -1;
        }
      }
    }
  }
  formobstacleboundary();
}

void Map::addobstaclebuffer() {
  int buffer = robotradius + clearance;
  for (int j = 0; j < gridmapheight; j++) {
    for (int i = 0; i < gridmapwidth; i++) {
      if (gridmap[i][j] == 0 || i == 0|| j == 0||
          i == gridmapwidth-1 || j == gridmapheight-1) {
        for (int k = 0; k < gridmapheight; k++) {
          for (int l = 0; l < gridmapwidth; l++) {
            if (((l-i)*(l-i)) + ((k-j)*(k-j))-(buffer*buffer) <= 0) {
              if (gridmap[l][k] > 0) {
                gridmap[l][k] = -1;
              }
            }
          }
        }
      }
    }
  }
}



int Map::getgridmapheight() {
  return gridmapheight;
}


int Map::getgridmapwidth() {
  return gridmapwidth;
}

void Map::setgridmap(int x, int y, int value) {
  gridmap[x][y] = value;
}
int Map::getgridmap(int x, int y) {
  return gridmap[x][y];
}

void Map::formobstacleboundary() {
  for (int j = 1; j < gridmapheight-1; j++) {
    for (int i = 1; i < gridmapwidth-1; i++) {
      if (gridmap[i][j] == -1) {
        if (gridmap[i][j+1]== 1 || gridmap[i+1][j+1]== 1 ||
            gridmap[i+1][j]== 1 || gridmap[i+1][j-1]== 1 ||
            gridmap[i][j-1]== 1 || gridmap[i-1][j-1]== 1 ||
            gridmap[i-1][j]== 1 || gridmap[i-1][j+1]== 1) {
          gridmap[i][j]= 0;
        }
      }
    }
  }
}

void Map::joinconcaveshapes() {
  for (int j = 0; j < gridmapheight; j++) {
    for (int i = 0; i < gridmapwidth; i++) {
      if (gridmap[i][j] == 0) {
        if (gridmap[i][j+1] <= 0 && gridmap[i+1][j+1] <= 0 &&
            gridmap[i+1][j] <= 0 && gridmap[i+1][j-1] <= 0 &&
            gridmap[i][j-1] <= 0 && gridmap[i-1][j-1] <= 0 &&
            gridmap[i-1][j] <= 0 && gridmap[i-1][j+1] <= 0) {
          gridmap[i][j]= -1;
        }
      }
    }
  }
}
void Map::buildmap() {
  vector<vector<int>> rectaobstacle1
  {{309, 910},
    {150, 910},
    {150, 750},
    {309, 750}};
  vector<vector<int>> rectaobstacle2
  {{529, 498},
    {438, 498},
    {438, 315},
    {529, 315}};
  vector<vector<int>> rectaobstacle3
  {{712, 341},
    {529, 341},
    {529, 265},
    {712, 265}};
  vector<vector<int>> rectaobstacle4
  {{748, 187},
    {474, 187},
    {474, 35},
    {748, 35}};
  vector<vector<int>> rectaobstacle5
  {{1110, 35},
    {685, 35},
    {685, 0},
    {1110, 0}};
  vector<vector<int>> rectaobstacle6
  {{896, 93},
    {779, 93},
    {779, 35},
    {896, 35}};
  vector<vector<int>> rectaobstacle7
  {{937, 384},
    {784, 384},
    {784, 267},
    {937, 267}};
  vector<vector<int>> rectaobstacle8
  {{1110, 111},
    {927, 111},
    {927, 35},
    {1110, 35}};
  vector<vector<int>> rectaobstacle9
  {{1110, 295},
    {1052, 295},
    {1052, 178},
    {1110, 178}};
  vector<vector<int>> rectaobstacle10
  {{1110, 449},
    {1019, 449},
    {1019, 362},
    {1110, 362}};
  vector<vector<int>> rectaobstacle11
  {{1110, 566},
    {1052, 566},
    {1052, 449},
    {1110, 449}};
  vector<vector<int>> rectaobstacle12
  {{1110, 697},
    {744, 697},
    {744, 621},
    {1110, 621}};
  vector<vector<int>> rectaobstacle13
  {{918, 1010},
    {832, 1010},
    {832, 827},
    {918, 827}};
  vector<vector<int>> rectaobstacle14
  {{1026, 1010},
    {983, 1010},
    {983, 919},
    {1026, 919}};


  Coord circle1, circle2, circle3, circle4, circle5, circle6;
  circle1.set_x(150);
  circle1.set_y(830);
  circle2.set_x(309);
  circle2.set_y(830);
  circle3.set_x(390);
  circle3.set_y(45);
  circle4.set_x(438);
  circle4.set_y(274);
  circle5.set_x(438);
  circle5.set_y(736);
  circle6.set_x(390);
  circle6.set_y(965);
  definecircleobstacle(circle1, 80);
  definecircleobstacle(circle2, 80);

  definecircleobstacle(circle3, 41);
  definecircleobstacle(circle4, 41);
  definecircleobstacle(circle5, 41);
  definecircleobstacle(circle6, 41);

  definepolygonobstacle(4, rectaobstacle1);
  definepolygonobstacle(4, rectaobstacle2);
  definepolygonobstacle(4, rectaobstacle3);
  definepolygonobstacle(4, rectaobstacle4);
  definepolygonobstacle(4, rectaobstacle5);
  definepolygonobstacle(4, rectaobstacle6);
  definepolygonobstacle(4, rectaobstacle7);
  definepolygonobstacle(4, rectaobstacle8);
  definepolygonobstacle(4, rectaobstacle9);
  definepolygonobstacle(4, rectaobstacle10);
  definepolygonobstacle(4, rectaobstacle11);
  definepolygonobstacle(4, rectaobstacle12);
  definepolygonobstacle(4, rectaobstacle13);
  definepolygonobstacle(4, rectaobstacle14);

  addobstaclebuffer();
  //
  seemap();
}

Coord Map::getendcoord() {
  return endCoord;
}
Coord Map::getstartingCoord() {
  return startingCoord;
}

void Map::seemap() {
  for (int i = 0; i < gridmapheight; i++) {
    for (int j = 0; j < gridmapwidth; j++) {
      if (gridmap[j][gridmapheight-i] == -1) {
        frame.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 0, 0);
      }
      if (gridmap[j][gridmapheight-i] == 0) {
        frame.at<cv::Vec3b>(i, j) = cv::Vec3b(128, 128, 128);
      }
      if (gridmap[j][gridmapheight-i] == 2) {
        frame.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 255,  0);
      }
      if (gridmap[j][gridmapheight-i] == 3) {
        frame.at<cv::Vec3b>(i, j) = cv::Vec3b(255, 0, 0);
      }
      if (gridmap[j][gridmapheight-i] == 4) {
        frame.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 0, 255);
      }
      if (gridmap[j][gridmapheight-i] == 5) {
        frame.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 0, 255);
      }
    }
  }
  cv:: Mat outImg;
  cv::resize(frame, outImg, cv::Size(), 1, 1);
  cv::namedWindow("Project3", cv::WINDOW_AUTOSIZE);
  cv::imshow("Project3", outImg);
  cv::waitKey(1000);
}

void Map::updatemap(int from_x, int from_y, int to_x, int to_y, int fillvalue) {
  Coord fillposition;
  double deltax = (to_x-from_x);
  double deltay = (to_y-from_y);
  double maximum = std::max(abs(deltax), abs(deltay));
  deltax = (to_x-from_x)/maximum;
  deltay = (to_y-from_y)/maximum;
  for (int i = 0; i < maximum; i++) {
    fillposition.set_x(from_x+ceil(i*deltax));
    fillposition.set_y(from_y+ceil(i*deltay));
    fillmap(fillposition.get_x(), fillposition.get_y(), fillvalue);
  }
}


void Map::fillmap(int x, int y, int value) {
  int i = gridmapheight-y-1;
  int j = x;
  gridmap[j][gridmapheight-i-1] = value;
  if (value == -1) {
    frame.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 0, 0);
  }
  if (value == 0) {
    frame.at<cv::Vec3b>(i, j) = cv::Vec3b(128, 128, 128);
  }
  if (value == 2) {
    frame.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 255,  0);
  }
  if (value == 3) {
    frame.at<cv::Vec3b>(i, j) = cv::Vec3b(255, 0, 0);
  }
  if (value == 4) {
    frame.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 0, 255);
  }
  if (value == 5) {
    frame.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 0, 255);
  }
  cv:: Mat outImg1;
  cv::resize(frame, outImg1, cv::Size(), 1, 1);
  cv::imshow("Project3", outImg1);
  if (choice == 0) {
  cv::waitKey(1);
  }
}


int Map::getrpm1() {
  return rpm1;
}

int Map::getrpm2() {
  return rpm2;
}
bool Map::checkboundary(Coord point) {
  return (point.get_x() >= 0 && point.get_x() < gridmapwidth &&
      point.get_y() >= 0 && point.get_y() < gridmapheight);
}
bool Map::checksanity(Coord point) {
  return(gridmap[point.get_x()][point.get_y()] == 1 && checkboundary(point));
}

bool Map::takeinput(Coord sp, Coord ep) {
  int sanity = 0;
  if (checksanity(sp)) {
    startingCoord = sp;
    circlepoint(startingCoord.get_x(), startingCoord.get_y(), 2);
    sanity = sanity+1;
  } else {
    std::cout << "Invalid Starting Point!!\n";
  }
  if (checksanity(ep)) {
    endCoord = ep;
    circlepoint(endCoord.get_x(), endCoord.get_y(), 4);
    sanity = sanity+1;
  } else {
    std::cout << "Invalid End Point!!\n";
  }
  return(sanity == 2);
}

void Map::setx(int value) {
  x = value;
}
int Map::getx() {
  return x;
}

void Map::sety(int value) {
  y = value;
}

int Map::gety() {
  return y;
}
double Map::getwheelradius() {
  return wheelradius;
}
double Map::getdelta_t() {
  return delta_t;
}
double Map::getwheelgap() {
  return wheelegap;
}
int Map::getbuffer() {
  return(robotradius+clearance);
}

void Map::circlepoint(int x, int y, int color_code) {
  int i = gridmapheight-y-1;
  int j = x;
  if (color_code == 2) {
    cv::circle(frame, cv::Point(j, i), 4.0, cv::Scalar(0, 255,  0), -1, 8);
  }
  if (color_code == 4) {
    cv::circle(frame, cv::Point(j, i), 4.0, cv::Scalar(0, 0, 255), -1, 8);
  }
  cv::imshow("Project3", frame);
  cv::waitKey(1);
}
void Map::destroy() {
  cv::destroyAllWindows();
}

void Map::setchoice(int option) {
  choice = option;
}
