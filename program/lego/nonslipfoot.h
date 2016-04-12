// +-------------------------------------------------------------------------
// | nonslipfoot.h
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

#include <vector>
#include "motion/ml.h"

class Patch;


class DontSlip
{
 public:
   // the Index of 
   // left_foot_joint = 18, left_toe_joint = 19 , left_toe_dummy_joint = 24, right_foot_joint = 3, right_toe_joint = 4,  right_toe_dummy_joint = 20
  enum WhichFoot {NON, LEFT, RIGHT, RIGHTLEFT};
  struct SECTION {
  	int begin_pos;
  	int end_pos;
  	WhichFoot left_or_right;
  };
  std::vector<SECTION> slippery_sections;

 private:
  ml::Motion mod_mot;
  size_t ltoedum_joint, ltoe_joint, lfoot_joint, rtoedum_joint, rtoe_joint, rfoot_joint;

 public:	
  DontSlip(size_t leftFoot = 18, size_t leftToe = 19, size_t leftToeDum = 24, size_t rightFoot = 3, size_t rightToe = 4, size_t rightToeDum = 20)
	: ltoedum_joint(leftToeDum), ltoe_joint(leftToe), lfoot_joint(leftFoot), rtoedum_joint(rightToeDum), rtoe_joint(rightToe), rfoot_joint(rightFoot) { }
  ~DontSlip(){ }

  void set_refmotion(const ml::Motion &m) { mod_mot = m;}
  void hold_foot(Patch&);
  void hold_foot(ml::Motion&);
	
  void classify_contact_section(const ml::Motion &ref_m);  
  void noSlipperyFoot(const SECTION &contact, const ml::Motion &ref);		// use inverse-kinematics
  void glue_foot(const int glueyFrame, const int s, const int e, const WhichFoot& which, const ml::Motion &ref);
  void smooth_forward(const int sP, const int mid, const int d, const WhichFoot& which, const ml::Motion &ref);
  void smooth_backward(const int eP, const int mid, const int d, const WhichFoot& which, const ml::Motion &ref);

 private:
   void correct_contact_section(const int corr_gap);
   void renew_pos_contact(ml::Motion&);
};



