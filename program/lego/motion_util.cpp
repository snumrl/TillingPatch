#include "StdAfx.h"
#include "motion_util.h"


double get_maximum_radius(const ml::Motion &m) {
  vector<double> radius_all;
  for (auto it = m.begin(); it != m.end(); ++it) {
	const ml::Posture &p = *it;
	cml::vector3d root_pos = p.trans();

	for (int i = 1; i < (int)p.num_joint(); ++i) {
	  cml::vector3d offset = p.body()->offset(i);
	  cml::transf rot_se3 = cml::make_transf(cml::exp_mat3(cml::between_vector(cml::vector3d(1,0,0),offset)), cml::vector3d(0,0,0) );
	  cml::transf se3 = p.GetGlobalTransf(p.body()->parent(i));
	  double length = offset.length();
	  cml::transf end_se3 = se3 * rot_se3 * cml::trans_transf(cml::vector3d(length * 1.02, 0, 0));
	  cml::vector3d end_pos  = cml::trans(end_se3);

	  double radius = cml::length(cml::vec2(end_pos - root_pos)) + 0.05; // 상자의 끝 position 으로만 radius를 구했으므로 여유분을 더해줘야 안전하다.
	  radius_all.push_back(radius);
	}
  }
  return (*max_element(radius_all.begin(), radius_all.end())); 
}


double get_ground_value( const ml::Motion &m ) {
  return 0.0;
}


double begin_time( const vector<ml::Motion> &motions )
{
  vector<double> times;
  transform(motions.begin(), motions.end(), back_inserter(times), [](const ml::Motion &mot){return mot.first_posture().time;});

  return *min_element(times.begin(), times.end());
}

double end_time( const vector<ml::Motion> &motions )
{
  vector<double> times;
  transform(motions.begin(), motions.end(), back_inserter(times), [](const ml::Motion &mot){return mot.last_posture().time;});

  return *max_element(times.begin(), times.end());
}

double activation( const ml::Motion &m, const size_t p, const vector<double> &weight, const vector<double> &joint_var, const size_t window_size /*= 1*/ )
{
  double act_val = 0.0;  
  if (p == 0 || p == m.size() - 1) {
	if (p == 0) {
	  act_val = dist_pose_in_same_motion(0, 1, m, weight, joint_var);
	} else if (p == m.size() - 1) {
	  act_val = dist_pose_in_same_motion(m.size() - 2, m.size() - 1, m, weight, joint_var);
	}
	return act_val;
  } else {
	size_t local_window = window_size;
	if (p < window_size) {
	  local_window = p;	
	} else if (p > m.size() - window_size - 1) {
	  local_window = m.size() - p - 1;
	}
	for (size_t i = 0; i < local_window; ++i) {
	  for (size_t j = 0; j < local_window; ++j) {
		act_val += gaussian_function(static_cast<double>(i) - static_cast<double>(j), 0.0, 1.0) * dist_pose_in_same_motion(p-1-i, p+1+j, m, weight, joint_var);
	  }
	}
	act_val /= (static_cast<double>(local_window) * static_cast<double>(local_window));
	return act_val;
  }
}

double dist_pose_in_same_motion( const Feature &f1, const Feature &f2, const vector<double> &weights, const vector<double> &pos_variance )
{  
  if (f1.joint_num != f2.joint_num) {
	cerr << "error in making a feature of posture!" << endl;  return 10000.0;
  }
  double pos = 0.0;
  for (size_t i = 0; i < f1.joint_num; ++i) {
	pos += weights[i] * (f1.positions[i] - f2.positions[i]).length_squared() / std::sqrt(pos_variance[i]);
  }
  pos *= 2.0;
  double angle = 0.0; 
  for (size_t i = 0; i < f1.angles.size(); ++i) {
	angle += sqrt(pow(f1.angles[i] - f2.angles[i], 2)) / 1.0;
  }
  double vel = 0.0;
  for (size_t i = 0; i < f1.joint_num; ++i) {
	vel += weights[i] * (f1.velos[i] - f2.velos[i]).length_squared() / 1.0;
  }
  vel *= 15.0;
  double ang_vel = 0.0;
  for (size_t i = 0; i < f1.joint_num; ++i) {
	ang_vel += weights[i] * sqrt(pow(f1.angular_velos[i] - f2.angular_velos[i], 2)) / 1.0;
  }
  ang_vel *= 1.0;
  //cout << pos << ", " << angle << ", " << vel << ", " << ang_vel << endl;
  return pos + angle + vel + ang_vel;
}

double dist_pose_in_same_motion( const Feature &f1, const Feature &f2, const vector<double> &weights )
{
  return dist_pose_in_same_motion(f1, f2, weights, vector<double>(f1.joint_num, 1.0));
}

double dist_pose_in_same_motion( const size_t pos1, const size_t pos2, const ml::Motion &mot, const vector<double> &weights, const vector<double> &joint_variance )
{    
  Feature feature_1(pos1, mot);
  Feature feature_2(pos2, mot);
  return dist_pose_in_same_motion(feature_1, feature_2, weights, joint_variance);
}

double dist_pose_shape( const Feature &f1, const Feature& f2, const vector<double> &weights )
{
  if (f1.joint_num != f2.joint_num) {
	cerr << "error in making a feature of posture!" << endl;  return 10000.0;
  }
  Feature f = f2;
  f.transl(-1 * f.positions[0]);
  f.rotate(cml::PlaneProject_mat3(f1.orientation * cml::inverse(f.orientation)));
  f.transl(f1.positions[0]);
  double pos = 0.0;
  for (size_t i = 0; i < f1.joint_num; ++i) {
	pos += weights[i] * (f1.positions[i] - f.positions[i]).length_squared();
  }
  double angle = 0.0; 
  for (size_t i = 0; i < f1.angles.size(); ++i) {
	angle += pow(f1.angles[i] - f.angles[i], 2) / 1.0;
  }

  return pos + angle;
}

vector<double> extract_joint_weight( const ml::Motion &mot )
{
  vector<double> weights(mot.body()->num_joint(), 0.0);

  {
	int headjoint = 21;
	int curr = headjoint;
	double coeff = .9;	  
	for (size_t i = 0; i < 5; ++i) {
	  if (curr == 0) coeff = 1.;		// root
	  weights[curr] = coeff;
	  curr = mot.body()->parent(curr);
	}
  }
  {
	int rightjoint[2] = {20, 23};  
	int curr = 0;
	for (size_t j = 0; j < 2; ++j) {
	  curr = rightjoint[j];
	  double coeff = 1.0;  // 0.8
	  for (size_t i = 0; i < 5; ++i) {
		weights[curr] = coeff;
		if (i == 2) {
		  coeff = 0.9;
		}
		//coeff *= 1.08;	  
		curr = mot.body()->parent(curr);
	  }
	}
  }
  {
	int leftjoint[2] = {24, 22};  
	int curr = 0;
	for (size_t j = 0; j < 2; ++j) {
	  curr = leftjoint[j];
	  double coeff = 1.0;  
	  for (size_t i = 0; i < 5; ++i) {
		weights[curr] = coeff;
		if (i == 2) {
		  coeff = 0.9;
		}		
		curr = mot.body()->parent(curr);
	  }
	}
  }

  return weights;
}

vector<double> extract_pose_variance( const ml::Motion &mot )
{
  vector<cml::vector3> pos_mean(mot.body()->num_joint(), cml::vector3(0., 0., 0.));
  for (size_t j = 0; j < mot.size(); ++j) {
	for (size_t i = 0; i < mot.body()->num_joint(); ++i) {
	  pos_mean[i] += mot[j].GetGlobalTranslation(i);
	}
  }
  for (size_t j = 0; j < mot.body()->num_joint(); ++j) {
	pos_mean[j][0] /= static_cast<double>(mot.size());
	pos_mean[j][1] /= static_cast<double>(mot.size());
	pos_mean[j][2] /= static_cast<double>(mot.size());
  }
  vector<double> pos_var(mot.body()->num_joint(), 0.0);
  for (size_t j = 0; j < mot.body()->num_joint(); ++j) {
	for (size_t i = 0; i < mot.size(); ++i) {
	  pos_var[j] += (mot[i].GetGlobalTranslation(j) - pos_mean[j]).length_squared();
	}
	pos_var[j] /= static_cast<double>(mot.size());
  }
  return pos_var;
}


Feature::Feature( const size_t pos, const ml::Motion &mot )
{
  joint_num = mot.body()->num_joint();
  orientation = mot[pos].rotate(0);
  for (size_t i = 0; i < joint_num; ++i) {
	positions.push_back(mot[pos].GetGlobalTranslation(i));	
  }
  {
	int endjoint[5] = {23, 22, 20, 24, 21};
	for (size_t j = 0; j < 5; ++j) {
	  int prev = endjoint[j];
	  int curr = mot.body()->parent(prev);
	  double coeff =0.8;
	  while (curr != 0 && curr != 8 && curr != 12) {
		cml::vector3 v1 = mot[pos].GetGlobalTranslation(prev) - mot[pos].GetGlobalTranslation(curr);
		prev = curr;
		curr = mot.body()->parent(prev);
		cml::vector3 v2 = mot[pos].GetGlobalTranslation(prev) - mot[pos].GetGlobalTranslation(curr);
		angles.push_back(coeff * between_vector(v1, v2).length());
		coeff *= 1.08;
	  }
	}
  }
  size_t cpos = pos;
  if (pos == 0) cpos = 1;
  for (size_t i = 0; i < joint_num; ++i) {
	velos.push_back(mot[cpos].GetGlobalTranslation(i) - mot[cpos - 1].GetGlobalTranslation(i));	
  }
  for (size_t i = 0; i < joint_num; ++i) {
	angular_velos.push_back(angle_distance(mot[cpos].GetGlobalRoation(i), mot[cpos-1].GetGlobalRoation(i)));
  }
}

void Feature::rotate_y( double radi )
{
  cml::matrix3 roty = cml::roty_mat(radi);
  for (auto it = positions.begin(); it != positions.end(); ++it) {
	*it = roty * (*it);
  }
  for (auto it = velos.begin(); it != velos.end(); ++it) {
	*it = roty * (*it);
  }
}

void Feature::rotate( cml::vector3 rotation_vec )
{
  cml::matrix3 rot = cml::exp_mat3(rotation_vec);
  for (auto it = positions.begin(); it != positions.end(); ++it) {
	*it = rot * (*it);
  }
  for (auto it = velos.begin(); it != velos.end(); ++it) {
	*it = rot * (*it);
  }
}

void Feature::rotate( const cml::matrix3 rot )
{
  for (auto it = positions.begin(); it != positions.end(); ++it) {
	*it = rot * (*it);
  }
  for (auto it = velos.begin(); it != velos.end(); ++it) {
	*it = rot * (*it);
  }
}

void Feature::transl( cml::vector3 offset )
{
  for (auto it = positions.begin(); it != positions.end(); ++it) {
	*it += offset;
  }
}

bool dec_it(ml::Motion::Const_iterator &it_, const ml::Motion::Const_iterator &b_) {
  if (it_ == b_) { 
	return false; 
  } else {
	--it_;
	return true;
  }
}

size_t pre_toggling_state_length( bool isleft, bool isturnon, const ml::Motion::Const_iterator& endit, const ml::Motion::Const_iterator &boundCondi );
size_t post_toggling_state_length( bool isleft, bool isturnon, const ml::Motion::Const_iterator& endit, const ml::Motion::Const_iterator &boundCondi );

// if neither of feet contact the floor, then return false, 
bool contact_pre_state( ml::Motion::Const_iterator &endit, const ml::Motion::Const_iterator &boundCondi )
{ 
  int lstate = (endit->leftFootTouched) ? 0 : 1;
  int rstate = (endit->rightFootTouched) ? 0 : 1;

  if (endit == boundCondi) {
	endit = boundCondi;
	return (lstate * rstate == 1) ? false : true;
  }

  if (lstate * rstate == 1) {	
	ml::Motion::Const_iterator ret_it = endit;
	while (!ret_it->leftFootTouched && !ret_it->rightFootTouched) {
	  if (!dec_it(ret_it, boundCondi)) {
		endit = boundCondi;
		return (!endit->leftFootTouched && !endit->rightFootTouched) ? false : true;
	  }
	}
	endit = ret_it;
	return (!endit->leftFootTouched && !endit->rightFootTouched) ? false : true;
  } else {	
	vector<size_t> toggle_length(4, 100000);
	toggle_length[0] = pre_toggling_state_length(true, true, endit, boundCondi);
	toggle_length[1] = pre_toggling_state_length(true, false, endit, boundCondi);
	toggle_length[2] = pre_toggling_state_length(false, true, endit, boundCondi);
	toggle_length[3] = pre_toggling_state_length(false, false, endit, boundCondi);	
	endit -= (*min_element(toggle_length.begin(), toggle_length.end()));
	return (!endit->leftFootTouched && !endit->rightFootTouched) ? false : true;
  }  
}

bool inc_it(ml::Motion::Const_iterator &it_, const ml::Motion::Const_iterator &b_) {
  if (it_ == b_) { 
	return false; 
  } else {
	++it_;
	return true;
  }
}

bool contact_post_state( ml::Motion::Const_iterator &endit, const ml::Motion::Const_iterator &boundCondi )
{ 
  int lstate = (endit->leftFootTouched) ? 0 : 1;
  int rstate = (endit->rightFootTouched) ? 0 : 1;

  if (endit == boundCondi) {
	endit = boundCondi;
	return (lstate * rstate == 1) ? false : true;
  }

  if (lstate * rstate == 1) {	
	ml::Motion::Const_iterator ret_it = endit;
	while (!ret_it->leftFootTouched && !ret_it->rightFootTouched) {
	  if (!inc_it(ret_it, boundCondi)) {
		endit = boundCondi;
		return (!endit->leftFootTouched && !endit->rightFootTouched) ? false : true;
	  }
	}
	endit = ret_it;	
  } else {
	vector<size_t> toggle_length(4, 100000);
	toggle_length[0] = post_toggling_state_length(true, true, endit, boundCondi);
	toggle_length[1] = post_toggling_state_length(true, false, endit, boundCondi);
	toggle_length[2] = post_toggling_state_length(false, true, endit, boundCondi);
	toggle_length[3] = post_toggling_state_length(false, false, endit, boundCondi);
	endit += (*min_element(toggle_length.begin(), toggle_length.end()));
  }

  return (!endit->leftFootTouched && !endit->rightFootTouched) ? false : true;
}

size_t pre_toggling_state_length( bool isleft, bool isturnon, const ml::Motion::Const_iterator& endit, const ml::Motion::Const_iterator &boundCondi )
{
  ml::Motion::Const_iterator l_it = endit;
  if (isleft && isturnon && l_it->leftFootTouched) {
	while (l_it->leftFootTouched) {
	  if (!dec_it(l_it, boundCondi)) {
		break;
	  }
	} 	
  } else if (isleft && !isturnon && !l_it->leftFootTouched) {
	while (!l_it->leftFootTouched) {
	  if (!dec_it(l_it, boundCondi)) {
		break;
	  }
	} 	
  } else if (!isleft && isturnon && l_it->rightFootTouched) {
	while (l_it->rightFootTouched) {
	  if (!dec_it(l_it, boundCondi)) {
		break;
	  }
	} 	
  } else if (!isleft && !isturnon && !l_it->rightFootTouched) {
	while (!l_it->rightFootTouched) {
	  if (!dec_it(l_it, boundCondi)) {
		break;
	  }
	} 
  } else {
	return 100000;
  }
  return endit - l_it;  
}

size_t post_toggling_state_length( bool isleft, bool isturnon, const ml::Motion::Const_iterator& endit, const ml::Motion::Const_iterator &boundCondi )
{
  ml::Motion::Const_iterator l_it = endit;
  if (isleft && isturnon && l_it->leftFootTouched) {
	while (l_it->leftFootTouched) {
	  if (!inc_it(l_it, boundCondi)) {
		break;
	  }
	} 	
  } else if (isleft && !isturnon && !l_it->leftFootTouched) {
	while (!l_it->leftFootTouched) {
	  if (!inc_it(l_it, boundCondi)) {
		break;
	  }
	} 	
  } else if (!isleft && isturnon && l_it->rightFootTouched) {
	while (l_it->rightFootTouched) {
	  if (!inc_it(l_it, boundCondi)) {
		break;
	  }
	} 	
  } else if (!isleft && !isturnon && !l_it->rightFootTouched) {
	while (!l_it->rightFootTouched) {
	  if (!inc_it(l_it, boundCondi)) {
		break;
	  }
	} 
  } else {
	return 100000;
  }
  return l_it - endit;  
}

void set_color_motion( ml::Motion &m, const cml::vector3 &c )
{
  m.color = c;
}

