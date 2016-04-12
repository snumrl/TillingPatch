/*
 *  motion.cpp
 *  unagi
 *
 *  Created by normit on 09. 09. 09.
 *  Copyright 2009 __MyCompanyName__. All rights reserved.
 *
 */

#include "stdafx.h"
#include "ml.h"

#include <stack>
#include <fstream>
#include <cmath>

using namespace ml;

Motion::Motion(const Body *body) {
  m_body = body;
  color = cml::vector3(-1,-1,-1);
  connected_fore = nullptr;
  connected_back = nullptr;
}

void Motion::copy_common(const Motion &other) {
  m_body = other.m_body;
  color = other.color;
}

Motion::~Motion(){ }

void Motion::posture(int i, const Posture& p) {
  m_postures[i] = p;
}

void Motion::size( int size ) {
  m_postures.resize(size);
}


void Motion::AddPosture( const Posture& p ) {
  m_postures.push_back(p);
}

Motion *Motion::CropMotion(int from, int size, Motion *ret_motion) {
  if(ret_motion == 0) ret_motion = new Motion(m_body);
	
  ret_motion->body(m_body);
  ret_motion->size(size);
  for (int i=0; i<size; ++i) {
	ret_motion->posture(i, posture(from + i) );
  }
  return ret_motion;
}

Motion *Motion::Clone( Motion *ret_motion ) {
  return CropMotion(0, size(), ret_motion);
}

void Motion::LoadAMC( const char *amc_file, const char *asf_file, bool human_load /*= true*/, double scale /*= 1.0*/ ) {
  AMCReader reader;
  reader.LoadAMC(amc_file, asf_file, this, human_load, scale);
}

void Motion::LoadBVH(const char *file, bool human_load, double scale, int sample) {
  BVHReader reader;
  reader.LoadBVH(file, this, human_load, scale, sample);	
}

void Motion::LoadAMC_with_contactInfo(const char *amc_file, const char *asf_file, bool human_load /*= true*/, double scale /*= 1.0*/, double ground_truth /*= 0.0*/) {
  LoadAMC(amc_file, asf_file, human_load, scale);
  double ground = ground_truth;
  SetFootConstraint(24, 19, 20, 4, 0.0092, ground+.03, 0.012, ground+.04);
}

// set indexes of joint
// rfoot=3, rtoe=4,  rtoedummy=20, lfoot=18, ltoe=19 , ltoedummy=24
void Motion::SetFootConstraint( int ltoe_j, int lfoot_j, int rtoe_j, int rfoot_j, double toe_speed, double toe_height, double ankle_speed, double ankle_height ) {
  cml::vector3 fl0, fl1, fl2; 
  cml::vector3 tl0, tl1, tl2;
  cml::vector3 fr0, fr1, fr2;
  cml::vector3 tr0, tr1, tr2;
  cml::vector3 vfl, vtl;
  cml::vector3 vfr, vtr;

  for (int pos=0; pos<m_postures.size(); ++pos) 
  {
	fl0 = m_postures[pos].GetGlobalTranslation(lfoot_j);
	tl0 = m_postures[pos].GetGlobalTranslation(ltoe_j);
	fr0 = m_postures[pos].GetGlobalTranslation(rfoot_j);
	tr0 = m_postures[pos].GetGlobalTranslation(rtoe_j);		

	if (pos == 0) {
	  fl2 = m_postures[pos+1].GetGlobalTranslation(lfoot_j);
	  tl2 = m_postures[pos+1].GetGlobalTranslation(ltoe_j);
	  fr2 = m_postures[pos+1].GetGlobalTranslation(rfoot_j);
	  tr2 = m_postures[pos+1].GetGlobalTranslation(rtoe_j);
	  vfl = fl2 - fl0;
	  vtl = tl2 - tl0;
	  vfr = fr2 - fr0;		
	  vtr = tr2 - tr0;
	} else if (pos == m_postures.size() - 1) {
	  fl1 = m_postures[pos-1].GetGlobalTranslation(lfoot_j);
	  tl1 = m_postures[pos-1].GetGlobalTranslation(ltoe_j);
	  fr1 = m_postures[pos-1].GetGlobalTranslation(rfoot_j);
	  tr1 = m_postures[pos-1].GetGlobalTranslation(rtoe_j);
	  vfl = fl0 - fl1;
	  vtl = tl0 - tl1;
	  vfr = fr0 - fr1;
	  vtr = tr0 - tr1;
	} else {
	  fl1 = m_postures[pos-1].GetGlobalTranslation(lfoot_j);
	  tl1 = m_postures[pos-1].GetGlobalTranslation(ltoe_j);
	  fr1 = m_postures[pos-1].GetGlobalTranslation(rfoot_j);
	  tr1 = m_postures[pos-1].GetGlobalTranslation(rtoe_j);
	  fl2 = m_postures[pos+1].GetGlobalTranslation(lfoot_j);
	  tl2 = m_postures[pos+1].GetGlobalTranslation(ltoe_j);
	  fr2 = m_postures[pos+1].GetGlobalTranslation(rfoot_j);
	  tr2 = m_postures[pos+1].GetGlobalTranslation(rtoe_j);
	  vfl = (fl2 - fl1) / 2.0;
	  vtl = (tl2 - tl1) / 2.0;
	  vfr = (fr2 - fr1) / 2.0;
	  vtr = (tr2 - tr1) / 2.0;
	}
	double Vtl = vtl.length();
	double Vfl = vfl.length();
	double Vtr = vtr.length();
	double Vfr = vfr.length();

	m_postures[pos].leftFootTouched  = ((tl0[1] < toe_height && Vtl < toe_speed) || (fl0[1] < ankle_height && Vfl < ankle_speed));
	m_postures[pos].rightFootTouched = ((tr0[1] < toe_height && Vtr < toe_speed) || (fr0[1] < ankle_height && Vfr < ankle_speed));
  }
}


void Motion::SwapBody( const Body *toBody ) {
  for(size_t i = 0; i < size(); ++i) {
	m_postures[i].SwapBody(toBody);
  }
  m_body = toBody;
}

void Motion::Sample( double rate ) {
  int next_size = (int)(size() / rate);

  std::vector<Posture> postures;
  double cur_t = 0.;

  for(int i = 0; i < next_size; ++i) {
	int c = (int)cur_t;
	int c1 = c + 1;
	double a = c1 - cur_t;
	double b = 1 - a;
	ml::Posture p(m_postures[c].num_joint());
	p.body(m_body);

	p.trans( m_postures[c].trans() * a + m_postures[c1].trans() * b);
	for(size_t i = 0; i < p.num_joint(); ++i) {
	  p.rotate(i, cml::interpolate(m_postures[c].rotate(i), m_postures[c1].rotate(i), a));
	}
	p.time = i;
	postures.push_back(p);
	cur_t += rate;
  }
  m_postures = postures;
}

void ml::Motion::ApplyTransf( const cml::transf &t, double time /*= 0.0*/ ) {
  for (auto it = m_postures.begin(); it != m_postures.end(); ++it) {
	it->ApplyTransf(t);
	it->time += time;
  }
}

void ml::Motion::translate( const cml::vector3d &v ) {
  ApplyTransf(cml::trans_transf(v), 0.0);
}

void ml::Motion::rotate( double theta ) {
  ApplyTransf(cml::roty_transf(theta), 0.0);
}

void ml::Motion::translate_time( double time) {
  ApplyTransf(cml::identity_transf(), time);
}

void ml::Motion::translate_time_to_zero() {
  translate_time(-first_posture().time);
}

void ml::Motion::stitch( const Motion & const_add_m, bool forward /*= true*/ ) {
  Motion add_m = const_add_m;
  Posture ori_p, add_p;
  if (forward == true) {
	ori_p = last_posture();
	add_p = add_m.first_posture();
  } else {
	ori_p = first_posture();
	add_p = add_m.last_posture();
  }

  cml::matrix3 diff_rot = cml::PlaneProject_mat3(ori_p.rotate(0) * cml::inverse(add_p.rotate(0)));
  double diff_time = ori_p.time - add_p.time;

  for ( int i=0; i < add_m.size(); ++i ) {
	Posture &p = add_m.posture(i);
	double height = p.trans()[1];
	cml::vector3d diff_v = p.trans() - add_p.trans();
	cml::vector3d new_pos = ori_p.trans() + diff_rot * diff_v;
	new_pos[1] = height;

	p.trans(new_pos);
	p.rotate(0, diff_rot * p.rotate(0));
	p.time = p.time + diff_time; 
  }
  //warp
  ml::Motion *before_m, *after_m;
  if (forward == true) {
	before_m = this;
	after_m = &add_m;
  } else {
	before_m = &add_m;
	after_m = this;
  }
  warp(before_m, after_m);

  if (forward == true) {  
	this->m_postures.insert(m_postures.end(), add_m.m_postures.begin() + 1, add_m.m_postures.end());
  } else {
	this->m_postures.insert(m_postures.begin(), add_m.m_postures.begin(), add_m.m_postures.end()-1);
  }
}

void ml::Motion::transform_between_posture( ml::Posture to_posture, ml::Posture from_posture ) {
  double diff_time = to_posture.time - from_posture.time;
  cml::matrix3 diff_rot = cml::PlaneProject_mat3(to_posture.rotate(0) * cml::inverse(from_posture.rotate(0)));
  cml::transf diff_transf = cml::make_transf(diff_rot, cml::vector3(0,0,0));
  ApplyTransf(diff_transf);
  from_posture.ApplyTransf(diff_transf);

  translate_time(diff_time);
  translate(to_posture.trans() - from_posture.trans());
}

void ml::Motion::get_motion( const Motion &other, int first, int last ) {
  this->copy(other, other.begin()+first, other.begin()+last+1);
}

void ml::Motion::IkLimbSmoothly( const size_t frame, const int fduration, const int bduration, const size_t joint, const cml::vector3& pos ) {
  ml::Posture orig = m_postures[frame];
  m_postures[frame].IkLimb(joint, pos);
  ml::Posture fixed = m_postures[frame];

  int fmod_d = fduration;
  int bmod_d = bduration;
  if (frame+bduration>size()-1) {
	bmod_d = size()-1-frame;
  } else if (frame-fduration<0) {
	fmod_d = frame;
  }
  for (size_t i=0; i<fmod_d; ++i) {
	ml::smoothing(m_postures[frame-fmod_d+i], fixed, orig, i, fmod_d, false);
  }
  for (size_t i=0; i<bmod_d; ++i) {		
	ml::smoothing(m_postures[frame+i+1], fixed, orig, i, bmod_d, true);
  }
}

void ml::Motion::translateSmoothly( const size_t frame, const int fduration, const int bduration, const cml::vector3& pos ) {
  cml::vector3 orig = m_postures[frame].trans();
  m_postures[frame].trans(pos);
  cml::vector3 fixed = m_postures[frame].trans();
  int fmod_d = fduration;
  int bmod_d = bduration;
  if (frame+bduration>size()-1) {
	bmod_d = size()-1-frame;
  } else if (frame-fduration<0) {
	  fmod_d = frame;
  }	
  for (size_t i=0; i<fmod_d; ++i) {
	const cml::vector3 ori_pos = m_postures[frame-fmod_d+i].trans();
	m_postures[frame-fmod_d+i].trans( ori_pos + (fixed-orig) * (1.0-scalarTransitionFunc(static_cast<double>(i), static_cast<double>(fmod_d))));
  }
  for (size_t i=1; i<bmod_d+1; ++i) {		
	const cml::vector3 ori_pos = m_postures[frame+i].trans();
	m_postures[frame+i].trans( ori_pos + (fixed-orig) * scalarTransitionFunc(static_cast<double>(i), static_cast<double>(bmod_d)));
  }
}

