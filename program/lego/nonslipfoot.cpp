#include "stdafx.h"
#include "nonslipfoot.h"
#include "util/ccml.h"
#include "motion/func.h"
#include "patch.h"
#include <iostream>



void DontSlip::hold_foot( ml::Motion &m )
{
  slippery_sections.clear();
  classify_contact_section(m);
  correct_contact_section(5);
  renew_pos_contact(m);
  set_refmotion(m);

  for (auto it=slippery_sections.begin(); it!=slippery_sections.end(); ++it) {
	noSlipperyFoot(*it, m);
  }

  m = mod_mot;
}

void DontSlip::hold_foot( Patch& pa )
{
  if (pa.motions.empty()) return;
  for (size_t i = 0; i < pa.motions.size(); ++i) {
	hold_foot(pa.motions[i]);
  }
}

void DontSlip::classify_contact_section(const ml::Motion &ref_m)
{
  bool trigL = false, trigR = false;
  int startL = 0, startR = 0;
  
  for (size_t i=0; i<ref_m.size(); ++i) {
    WhichFoot result;
    if (ref_m.posture(i).leftFootTouched) {
    	if (ref_m.posture(i).rightFootTouched) {
    	  result = RIGHTLEFT;
    	} else {
    	  result = LEFT;
    	}
    } else {
    	if (ref_m.posture(i).rightFootTouched) {
    	  result = RIGHT;
    	} else {
    	  result = NON;
    	}
    }
    if (i == ref_m.size() - 1) {
	  if (trigR) {
    	  SECTION temp = {startR, i, RIGHT};
    	  slippery_sections.push_back(temp);
	  } 
	  if (trigL) {
		SECTION temp = {startL, i, LEFT};
		slippery_sections.push_back(temp);
	  }
    } else {
	  if (result == RIGHTLEFT) {
	    if (!trigR) {
		  trigR = true;
		  startR = i;
	    } 
	    if (!trigL) {
		  trigL = true;
		  startL = i;
	    }
	  } else if (result == RIGHT) {
	    if (trigL) {
		  trigL = false;
		  SECTION temp = {startL, i, LEFT};
		  slippery_sections.push_back(temp);
	    } 
	    if (!trigR) {
		  trigR = true;
		  startR = i;
	    }			
	  } else if (result == LEFT) {
	    if (trigR) {
		  trigR = false;
		  SECTION temp = {startR, i, RIGHT};
		  slippery_sections.push_back(temp);
	    } 
	    if (!trigL) {
		  trigL = true;
		  startL = i;
	    }			
	  } else if (result == NON) {
	    if (trigR) {
		  trigR = false;
		  SECTION temp = {startR, i, RIGHT};
		  slippery_sections.push_back(temp);
	    } 
	    if (trigL) {
		  trigL = false;
		  SECTION temp = {startL, i, LEFT};
		  slippery_sections.push_back(temp);
	    }
	  }
    }
  }
}

void DontSlip::noSlipperyFoot(const SECTION &contact, const ml::Motion &ref)
{
  int sduration = 5;	
  int eduration = 6;	
  int mid = 0;
  int mot_size = static_cast<int>(ref.size());
  
  if (contact.begin_pos < sduration) {
  	mid = contact.begin_pos;
  	glue_foot(mid, 0, contact.end_pos + 1, contact.left_or_right, ref);
	if (eduration + contact.end_pos <= mot_size) {
  	  smooth_backward(contact.end_pos, mid, eduration, contact.left_or_right, ref);
	}
  } else if (contact.end_pos >= mot_size - eduration) {
  	mid = contact.end_pos;
  	glue_foot(mid, contact.begin_pos, mot_size, contact.left_or_right, ref);
	if (contact.begin_pos - sduration >= 0) {
  	  smooth_forward(contact.begin_pos, mid, sduration, contact.left_or_right, ref);
	}
  } else {
  	mid = (contact.begin_pos + contact.end_pos) / 2;
  	glue_foot(mid, contact.begin_pos, contact.end_pos + 1, contact.left_or_right, ref);
  	smooth_forward(contact.begin_pos, mid, sduration, contact.left_or_right, ref);
  	smooth_backward(contact.end_pos, mid, eduration, contact.left_or_right, ref);
  }
}

void DontSlip::glue_foot(const int center_f, const int s, const int e, const WhichFoot& lr, const ml::Motion &ref)
{
  size_t toe_j = 0, toedum_j = 0, foot_j = 0;
  if (lr == RIGHT) { 
    toe_j=rtoe_joint; 
    toedum_j=rtoedum_joint; 
    foot_j=rfoot_joint; 
  } else if (lr == LEFT) { 
    toe_j = ltoe_joint; 
    toedum_j = ltoedum_joint; 
    foot_j = lfoot_joint; 
  } else { 
    std::cout << "I don't know which foot is slipped!" << std::endl; 
    return; 
  }
  
  cml::vector3 mid_tpos(ref.posture(center_f).GetGlobalTranslation(toe_j));
  cml::vector3 mid_tdpos(ref.posture(center_f).GetGlobalTranslation(toedum_j));	
  
  for (size_t i=s; i<e ; ++i) {
  	mod_mot.posture(i).IkLimb(toe_j, mid_tpos);
  	mod_mot.posture(i).IkLimb(toedum_j, mid_tdpos);	
  }
}

void DontSlip::smooth_forward(const int sP, const int mid, const int d, const WhichFoot& lr, const ml::Motion &ref)
{
  size_t toe_j, toedum_j, foot_j;
  if (lr == RIGHT) { 
    toe_j = rtoe_joint; 
    toedum_j = rtoedum_joint; 
    foot_j = rfoot_joint; 
  } else if (lr == LEFT) { 
    toe_j = ltoe_joint; 
    toedum_j = ltoedum_joint; 
    foot_j = lfoot_joint; 
  } else { 
    std::cout << "I don't know which foot is slipped!" << std::endl; 
    return; 
  }
  
  for (size_t i=0; i<d; ++i) {
  	ml::Posture temp(ref.posture(sP-d+i));
  	temp.IkLimb(toe_j, mod_mot.posture(mid).GetGlobalTranslation(toe_j));
  	temp.IkLimb(toedum_j, mod_mot.posture(mid).GetGlobalTranslation(toedum_j));
  	ml::smoothing(mod_mot.posture(sP-d+i), temp, ref.posture(sP-d+i), i, d, false);
  }
}

void DontSlip::smooth_backward(const int eP, const int mid, const int d, const WhichFoot& lr, const ml::Motion &ref)
{
  size_t toe_j, toedum_j, foot_j;
  if (lr == RIGHT) { 
    toe_j = rtoe_joint; 
    toedum_j = rtoedum_joint; 
    foot_j = rfoot_joint; 
  } else if (lr == LEFT) { 
    toe_j = ltoe_joint; 
    toedum_j = ltoedum_joint; 
    foot_j = lfoot_joint; 
  }
  
  for (size_t i=0; i<d-1; ++i) {
  	ml::Posture temp(ref.posture(eP+i+1));
  	temp.IkLimb(toe_j, mod_mot.posture(mid).GetGlobalTranslation(toe_j));
  	temp.IkLimb(toedum_j, mod_mot.posture(mid).GetGlobalTranslation(toedum_j));
  	ml::smoothing(mod_mot.posture(eP+i+1), temp, ref.posture(eP+i+1), i, d-1, true);
  }
}

void DontSlip::correct_contact_section(const int Correction_gap)
{
  {
	vector<SECTION>::iterator prev_r = slippery_sections.begin();
	while (prev_r != slippery_sections.end()) {
	  if (prev_r->left_or_right == RIGHT) {
		break;
	  } else {
		++prev_r;
	  }
	}
	if ((prev_r != slippery_sections.end()) && (prev_r+1 != slippery_sections.end())) {
	  for (auto it = prev_r+1; it != slippery_sections.end(); ++it) {
		if (it->left_or_right == RIGHT) {
		  if (it->begin_pos - prev_r->end_pos < Correction_gap) {
			prev_r->end_pos = it->end_pos;
			it = slippery_sections.erase(it) - 1;
		  } else {
			prev_r = it;
		  }		
		}
	  }
	}
  }
  {
	vector<SECTION>::iterator prev_l = slippery_sections.begin();
	while (prev_l != slippery_sections.end()) {
	  if (prev_l->left_or_right == LEFT) {
		break;
	  } else {
		++prev_l;
	  }
	}
	if ((prev_l != slippery_sections.end()) && (prev_l+1 != slippery_sections.end())) {
	  for (auto it = prev_l+1; it != slippery_sections.end(); ++it) {
		if (it->left_or_right == LEFT) {
		  if (it->begin_pos - prev_l->end_pos < Correction_gap) {
			prev_l->end_pos = it->end_pos;
			it = slippery_sections.erase(it) - 1;
		  } else {
			prev_l = it;
		  }		
		}
	  }
	}
  }
  /*
  {
	vector<SECTION>::iterator prev_rl = slippery_sections.begin();
	while (prev_rl != slippery_sections.end()) {
	  if (prev_rl->left_or_right == RIGHTLEFT) {
		break;
	  } else {
		++prev_rl;
	  }
	}
	if ((prev_rl != slippery_sections.end()) && (prev_rl+1 != slippery_sections.end())) {
	  for (auto it = prev_rl+1; it != slippery_sections.end(); ++it) {
		if (it->left_or_right == LEFT || it->left_or_right == RIGHT) {
		  if (it->begin_pos - prev_rl->end_pos < 5) {
			it->left_or_right = RIGHTLEFT;
		  }
		} else if (it->left_or_right == RIGHTLEFT) {
		  prev_rl = it;
		}
	  }
	}
  }*/
}

void DontSlip::renew_pos_contact(ml::Motion &mot)
{
  for (auto it = slippery_sections.begin(); it != slippery_sections.end(); ++it) {
	if (it->left_or_right == LEFT) {
	  for (int p = it->begin_pos; p <= it->end_pos; ++p)
		mot[p].leftFootTouched = true;
	} else if (it->left_or_right == RIGHT) {
	  for (int p = it->begin_pos; p <= it->end_pos; ++p)
		mot[p].rightFootTouched = true;
	}
  }
}


