// +-------------------------------------------------------------------------
// | motion_util.h
// | 
// | Author: Manmyung Kim
// +-------------------------------------------------------------------------
// | COPYRIGHT:
// |    Copyright Manmyung Kim 2013
// |    See the included COPYRIGHT.txt file for further details.
// |    
// |    This file is part of the TilingMotionPatch.
// |    TilingMotionPatch is free software: you can redistribute it and/or modify
// |    it under the terms of the MIT License.
// |
// |    You should have received a copy of the MIT License
// |    along with TilingMotionPatch.  If not, see <mit-license.org>.
// +-------------------------------------------------------------------------
#pragma once

#include "motion/ml.h"
#include <vector>


void set_color_motion(ml::Motion &m, const cml::vector3 &c);

double get_maximum_radius(const ml::Motion &m);
double get_ground_value(const ml::Motion& m);
double begin_time(const std::vector<ml::Motion> &motions);
double end_time(const std::vector<ml::Motion> &motions);

struct Feature
{
  Feature(): joint_num(0) { };
  Feature(const size_t pos, const ml::Motion &mot);

  size_t joint_num;
  cml::matrix3 orientation;
  // four properties will be considered to get feature of the posture.    
  std::vector<cml::vector3> positions;   // 1. Joint position
  std::vector<double> angles;			 // 2. Joint angles  
  std::vector<cml::vector3> velos;		 // 3. Joint velocity
  std::vector<double> angular_velos;	 // 4. Joint angular velocity

  void rotate_y(double radi);
  void rotate(cml::vector3 rv);
  void rotate(const cml::matrix3 rot);
  void transl(cml::vector3 offset);
};

std::vector<double> extract_joint_weight(const ml::Motion &mot);
std::vector<double> extract_pose_variance(const ml::Motion &mot);

double dist_pose_in_same_motion(const Feature &f1, const Feature &f2, const vector<double> &weights, const vector<double> &joint_variance);
double dist_pose_in_same_motion(const Feature &f1, const Feature &f2, const vector<double> &weights);
double dist_pose_in_same_motion(const size_t pos1, const size_t pos2, const ml::Motion &mot, const vector<double> &weights, const vector<double> &joint_variance);
double dist_pose_shape(const Feature &f1, const Feature& f2, const vector<double> &weights);

double activation(const ml::Motion &, const size_t, const vector<double> &, const vector<double> &, const size_t window_size = 1);

bool contact_pre_state(ml::Motion::Const_iterator &contact_end, const ml::Motion::Const_iterator &Bcondition);
bool contact_post_state(ml::Motion::Const_iterator &contact_end, const ml::Motion::Const_iterator &Bcondition);