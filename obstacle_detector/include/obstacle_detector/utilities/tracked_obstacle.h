/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2017, Poznan University of Technology
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Poznan University of Technology nor the names
 *       of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Author: Mateusz Przybyla
 */

#pragma once

#include <obstacle_detector/Obstacles.h>
#include "obstacle_detector/utilities/kalman.h"

namespace obstacle_detector
{

class TrackedObstacle {
public:
  TrackedObstacle(const CircleObstacle& obstacle) : obstacle_(obstacle), kf_x_(0, 1, 2), kf_y_(0, 1, 2), kf_r_(0, 1, 2) {
    fade_counter_ = s_fade_counter_size_;
    initKF();
  }

  void predictState() {
    kf_x_.predictState();
    kf_y_.predictState();
    kf_r_.predictState();
    // kf_d_.predictState();//chiu

    obstacle_.center.x = kf_x_.q_pred(0);
    obstacle_.center.y = kf_y_.q_pred(0);

    obstacle_.velocity.x = kf_x_.q_pred(1);
    obstacle_.velocity.y = kf_y_.q_pred(1);

    obstacle_.radius = kf_r_.q_pred(0);

    // obstacle_.direction = kf_d_.q_pred(0);//chiu

    fade_counter_--;
  }

  void correctState(const CircleObstacle& new_obstacle) {
    kf_x_.y(0) = new_obstacle.center.x;
    kf_y_.y(0) = new_obstacle.center.y;
    kf_r_.y(0) = new_obstacle.radius;
    // kf_d_.y(0) = new_obstacle.direction;//chiu

    kf_x_.correctState();
    kf_y_.correctState();
    kf_r_.correctState();
    // kf_d_.correctState();//chiu

    obstacle_.center.x = kf_x_.q_est(0);
    obstacle_.center.y = kf_y_.q_est(0);

    obstacle_.velocity.x = kf_x_.q_est(1);
    obstacle_.velocity.y = kf_y_.q_est(1);

    obstacle_.radius = kf_r_.q_est(0);

    // obstacle_.direction = kf_d_.q_est(0);//chiu

    fade_counter_ = s_fade_counter_size_;
  }

  void updateState() {
    kf_x_.predictState();
    kf_y_.predictState();
    kf_r_.predictState();
    // kf_d_.predictState();//chiu

    kf_x_.correctState();
    kf_y_.correctState();
    kf_r_.correctState();
    // kf_d_.correctState();//chiu

    obstacle_.center.x = kf_x_.q_est(0);
    obstacle_.center.y = kf_y_.q_est(0);

    obstacle_.velocity.x = kf_x_.q_est(1);
    obstacle_.velocity.y = kf_y_.q_est(1);

    obstacle_.radius = kf_r_.q_est(0);

    // obstacle_.direction = kf_d_.q_est(0);//chiu

    fade_counter_--;
  }

  void calACC(const CircleObstacle& new_obstacle){
    // obstacle_.direction = obstacle_.velocity.x - new_obstacle.velocity.x; //chiu
    // double direction = atan2((new_obstacle.center.y-obstacle_.center.y), (new_obstacle.center.x-obstacle_.center.x));
    // double direction;
    // double x = new_obstacle.center.x-obstacle_.center.x;
    // double y = new_obstacle.center.y-obstacle_.center.y;
    // double distance = sqrt(pow(x, 2.0) + pow(y, 2.0));;
    
    double direction = atan2(obstacle_.velocity.y, obstacle_.velocity.x);
    double VelValue = 0.0;
    int sum_of_elems = 0;
    // direction = atan2((new_obstacle.center.y-obstacle_.center.y), (new_obstacle.center.x-obstacle_.center.x));
    direction = direction/3.14*180;
    if(direction<0)
      direction+=360;
    
    VelValue = sqrt(pow(obstacle_.velocity.x, 2) + pow(obstacle_.velocity.y, 2));
    if(VelValue<0.3){
      obstacle_.state = 0;
      obstacle_.state_list.push_back(-1);
    }
    else
    {
      obstacle_.state = 1;
      obstacle_.state_list.push_back(1);
    }
    if(obstacle_.state_list.size()==10)
    {
      for(auto& n : obstacle_.state_list)
        sum_of_elems +=n;
      std::cout<<"sum:"<<sum_of_elems<<std::endl;
      // if(sum_of_elems>0)
        // obstacle_.state = 1;
      // else
        // obstacle_.state = 0;
      obstacle_.state_list.clear();
    }  
    // if(0<new_obstacle.direction-direction<90)
    //   std::cout<<"turn left"<<std::endl;
    obstacle_.direction = transform_direction(direction);
    obstacle_.LCSS_state = LCSS_state(direction);
    obstacle_.direction_list.push_back(direction);

    if(obstacle_.direction_list.size()==51)
    {
      obstacle_.direction_list.clear();
    }  
  }

  int transform_direction(double x){
    if (337.5>=x & x>=292.5)
        return 315;
    else if (292.5>=x & x>=247.5)
        return 270;
    else if (247.5>=x & x>=202.5)
        return 225;
    else if (202.5>=x & x>=157.5)
        return 180;
    else if (157.5>=x & x>=112.5)
        return 135;
    else if (112.5>x & x>=67.5)
        return 90;
    else if (67.5>x & x>=22.5)
        return 45;
    else if (22.5>x & x>0)
        return 360;
    else if (360>=x & x>337.5)
        return 360;
  }

  int LCSS_state(double x){
    if (337.5>=x & x>=292.5)
        return 2;//315
    else if (292.5>=x & x>=247.5)
        return 3;//270
    else if (247.5>=x & x>=202.5)
        return 4;//225
    else if (202.5>=x & x>=157.5)
        return 5;//180
    else if (157.5>=x & x>=112.5)
        return 6;//135
    else if (112.5>x & x>=67.5)
        return 7;//90
    else if (67.5>x & x>=22.5)
        return 8;//45
    else if (22.5>x & x>0)
        return 9;//0
    else if (360>=x & x>337.5)
        return 9;//0
  }


  static void setSamplingTime(double tp) {
    s_sampling_time_ = tp;
  }

  static void setCounterSize(int size) {
    s_fade_counter_size_ = size;
  }

  static void setCovariances(double process_var, double process_rate_var, double measurement_var) {
    s_process_variance_ = process_var;
    s_process_rate_variance_ = process_rate_var;
    s_measurement_variance_ = measurement_var;
  }

  bool hasFaded() const { return ((fade_counter_ <= 0) ? true : false); }
  const CircleObstacle& getObstacle() const { return obstacle_; }
  const KalmanFilter& getKFx() const { return kf_x_; }
  const KalmanFilter& getKFy() const { return kf_y_; }
  const KalmanFilter& getKFr() const { return kf_r_; }
  // const KalmanFilter& getKFd() const { return kf_d_; }

private:
  void initKF() {
    kf_x_.A(0, 1) = s_sampling_time_;
    kf_y_.A(0, 1) = s_sampling_time_;
    kf_r_.A(0, 1) = s_sampling_time_;
    // kf_d_.A(0, 1) = s_sampling_time_;//chiu

    kf_x_.C(0, 0) = 1.0;
    kf_y_.C(0, 0) = 1.0;
    kf_r_.C(0, 0) = 1.0;
    // kf_d_.C(0, 0) = 1.0;//chiu

    kf_x_.R(0, 0) = s_measurement_variance_;
    kf_y_.R(0, 0) = s_measurement_variance_;
    kf_r_.R(0, 0) = s_measurement_variance_;
    // kf_d_.R(0, 0) = s_measurement_variance_;//chiu

    kf_x_.Q(0, 0) = s_process_variance_;
    kf_r_.Q(0, 0) = s_process_variance_;
    kf_y_.Q(0, 0) = s_process_variance_;
    // kf_d_.Q(0, 0) = s_process_variance_;//chiu

    kf_x_.Q(1, 1) = s_process_rate_variance_;
    kf_y_.Q(1, 1) = s_process_rate_variance_;
    kf_r_.Q(1, 1) = s_process_rate_variance_;
    // kf_d_.Q(1, 1) = s_process_rate_variance_;//chiu

    kf_x_.q_pred(0) = obstacle_.center.x;
    kf_r_.q_pred(0) = obstacle_.radius;
    kf_y_.q_pred(0) = obstacle_.center.y;
    // kf_d_.q_pred(0) = obstacle_.direction;//chiu
    // kf_d_.q_pred(0) = atan2(obstacle_.center.y, obstacle_.center.x);

    kf_x_.q_pred(1) = obstacle_.velocity.x;
    kf_y_.q_pred(1) = obstacle_.velocity.y;

    kf_x_.q_est(0) = obstacle_.center.x;
    kf_r_.q_est(0) = obstacle_.radius;
    kf_y_.q_est(0) = obstacle_.center.y;
    // kf_d_.q_est(0) = obstacle_.direction;//chiu
    // kf_d_.q_est(0) = atan2(obstacle_.center.y, obstacle_.center.x);

    kf_x_.q_est(1) = obstacle_.velocity.x;
    kf_y_.q_est(1) = obstacle_.velocity.y;

    // ROS_ERROR_STREAM( "Error "<<obstacle_.velocity.y);
  }

  CircleObstacle obstacle_;

  KalmanFilter kf_x_;
  KalmanFilter kf_y_;
  KalmanFilter kf_r_;
  // KalmanFilter kf_d_;//chiu

  int fade_counter_;

  // Common variables
  static int s_fade_counter_size_;
  static double s_sampling_time_;
  static double s_process_variance_;
  static double s_process_rate_variance_;
  static double s_measurement_variance_;
};

}
