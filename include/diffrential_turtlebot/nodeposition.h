/**
 * @file      include/SteeringControl.h
 * @brief     Header file for declaring the class SteeringControl,its attributes and its methods
 *            along with class SteeringControlOutput, class Point and their data members
 * @author    Saurav Kumar
 * @copyright 2019
 *
 **BSD 3-Clause License
 *
 *Copyright (c) 2019, Saurav Kumar
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

#ifndef INCLUDE_DIFFRENTIAL_TURTLEBOT_NODEPOSITION_H_
#define INCLUDE_DIFFRENTIAL_TURTLEBOT_NODEPOSITION_H_
#include<iostream>


/**
 * @brief   class Point is declared with its two data members namely X and Y
 *          and four methods : get_x
 *                             set_x
 *                             get_y
 *                             set_y
 */
class Nodeposition {
 private:
  int
  X,
  Y,
  theeta;

 public:
  /** constructor*
Point();
   * destructor*
~Point();
   */
  /**
   * @brief   get_x
   * @details returns the stored value of X coordinate of the the point
   * @param   void
   * @return  double
   */
  int get_x();

  /**
   * @brief   set_x
   * @details set the new value of X Coordinate in "X" datamember
   * @param   double
   * @return  void
   */
  void set_x(int);

  /**
   * @brief   get_y
   * @details returns the stored value of Y coordinate of the the point
   * @param   void
   * @return  double stored value of Y Coordinate from "Y" is returned
   */
  int get_y();

  /**
   * @brief   set_y
   * @details set the new value of Y Coordinate in "Y" datamember
   * @param   double
   * @return  void
   */
  void set_y(int);
  int get_theeta();

  /**
   * @brief   set_y
   * @details set the new value of Y Coordinate in "Y" datamember
   * @param   double
   * @return  void
   */
  void set_theeta(int);
};
#endif  // INCLUDE_DIFFRENTIAL_TURTLEBOT_NODEPOSITION_H_
