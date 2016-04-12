#include "stdafx.h"
#include "ml.h"
#include <vector>

using namespace ml;

double ml::DiffPosture( const Posture& p1, const Posture& p2 )
{
	std::vector<int> leaf_nodes;
	double ret_error = 0.;
	for(size_t i = 0; i < p1.body()->num_joint(); ++i) {
		ret_error += (p1.GetGlobalTranslation(i) - p2.GetGlobalTranslation(i)).length_squared();
	}

	return ret_error;
}

// d = p2 - p1
std::pair<cml::vector3d, std::vector<cml::vector3d>> ml::difference(const Posture &p2, const Posture &p1)
{
	cml::vector3d diff_trans = p2.trans() - p1.trans();
	std::vector<cml::vector3d> diff_oris;
	for (int k = 0; k < p2.body()->num_joint(); ++k)
	{
		diff_oris.push_back(cml::log_mat3(cml::inverse(p1.rotate(k)) * p2.rotate(k)));
	}
	return std::make_pair(diff_trans, diff_oris);
}

std::pair<cml::vector3d, cml::vector3d> ml::difference_root(const Posture &p2, const Posture &p1)
{
	cml::vector3d diff_trans = p2.trans() - p1.trans();
	int k = 0;
	return std::make_pair(diff_trans, cml::log_mat3(cml::inverse(p1.rotate(k)) * p2.rotate(k)));
}

// p + diff
void ml::add_difference(Posture &p, std::pair<cml::vector3d, std::vector<cml::vector3d>> &diff, double ratio)
{
	p.trans(p.trans() + diff.first * ratio);
	for (int k = 0; k < p.body()->num_joint(); ++k)
	{
		p.rotate(k, p.rotate(k) * cml::exp_mat3(diff.second[k] * ratio));
	}
}

void ml::add_difference_root_direction_height(Posture &p, std::pair<cml::vector3d, cml::vector3d> &diff, double ratio)
{
	//p.trans(p.trans() + diff.first * ratio);
	p.m_trans[1] = p.m_trans[1] + (diff.first)[1] * ratio;
	for (int k = 0; k < 1; ++k)
	{
		p.rotate(k, p.rotate(k) * cml::exp_mat3(diff.second * ratio));
	}
}


Motion& ml::stitch(Motion m1, Motion m2, int warp_width /*= 20 */)
{
	//translate and rotate ( diff_rot * p2_rot = p1_rot )
	Posture p1 = m1[m1.size()-1];
	Posture p2 = m2[0];
	cml::matrix3 diff_rot = cml::PlaneProject_mat3(p1.rotate(0) * cml::inverse(p2.rotate(0)));
	for ( int i=0; i < m2.size(); ++i )
	{
		Posture &p = m2[i];
		double height = p.trans()[1];
		cml::vector3d diff_v = p.trans() - p2.trans();
		cml::vector3d new_pos = p1.trans() + diff_rot * diff_v;
		new_pos[1] = height;

		p.trans(new_pos);
		p.rotate(0, diff_rot * p.rotate(0));
	}

	//warp
	p1 = m1[m1.size()-1];
	p2 = m2[0];
	Posture mid = p1;
	add_difference(mid, difference(p2, p1), 0.5);
	std::pair<cml::vector3d, std::vector<cml::vector3d>> diff_mid_p1 = difference(mid, p1);
	std::pair<cml::vector3d, std::vector<cml::vector3d>> diff_mid_p2 = difference(mid, p2);

	int half_width = warp_width / 2;
	for (int i = m1.size()-1- half_width; i <= m1.size()-1; ++i)
	{
		double x = (i - ((int)(m1.size())-1))/ double(half_width+1);
		double ratio = cml::blend_weight(x);
		Posture &p = m1[i];
		add_difference(p, diff_mid_p1, ratio);
	}

	for (int i = 0; i <= half_width; ++i)
	{
		double x = i / double(half_width+1);
		double ratio = cml::blend_weight(x);
		Posture &p = m2[i];
		add_difference(p, diff_mid_p2, ratio);
	}

	//stitch
	Motion &m = *(new Motion(m1));
	m.m_postures.insert(m.m_postures.end(), m2.m_postures.begin() +1, m2.m_postures.end());
	return m;
}

double ml::scalarTransitionFunc(const double t, const double range)
{
	return 0.5*std::cos(cml::pi()*t/range)+0.5;
}


void ml::warp( ml::Motion * before_m, ml::Motion * after_m ) {
  Posture before_p = before_m->last_posture();
  Posture after_p = after_m->first_posture();

  int warp_width = 20;
  int before_width = (before_m->size() < warp_width) ? before_m->size() : warp_width;
  int after_width = (after_m->size() < warp_width) ? after_m->size() : warp_width;

  Posture mid = before_p;
  add_difference(mid, difference(after_p, before_p), (double)before_width / (double)(before_width + after_width));

  for (int i = before_m->size()-1 - (before_width-1); i <= before_m->size()-1; ++i) {
	double x = (i - ((int)(before_m->size())-1))/ double(before_width);
	double ratio = cml::blend_weight(x);
	Posture &p = before_m->posture(i);
	add_difference(p, difference(mid, before_p), ratio);
  }
  for (int i = 0; i <= (after_width - 1); ++i) {
	double x = i / double(after_width);
	double ratio = cml::blend_weight(x);
	Posture &p = after_m->posture(i);
	add_difference(p, difference(mid, after_p), ratio);
  }
}

void ml::warp_root_direction_height( ml::Motion * before_m, ml::Motion * after_m, int before_warp_width, int after_warp_width ) {
  Posture before_p = before_m->last_posture();
  Posture after_p = after_m->first_posture();

  int before_width = (before_m->size() < before_warp_width) ? before_m->size() : before_warp_width;
  int after_width = (after_m->size() < after_warp_width) ? after_m->size() : after_warp_width;

  Posture mid = before_p;
  add_difference_root_direction_height(mid, difference_root(after_p, before_p), (double)before_width / (double)(before_width + after_width));

  for (int i = before_m->size()-1 - (before_width-1); i <= before_m->size()-1; ++i) {
	double x = (i - ((int)(before_m->size())-1))/ double(before_width);
	double ratio = cml::blend_weight(x);
	Posture &p = before_m->posture(i);
	add_difference_root_direction_height(p, difference_root(mid, before_p), ratio);
  }
  for (int i = 0; i <= (after_width - 1); ++i) {
	double x = i / double(after_width);
	double ratio = cml::blend_weight(x);
	Posture &p = after_m->posture(i);
	add_difference_root_direction_height(p, difference_root(mid, after_p), ratio);
  }
}

std::string ml::write_motions( const vector<ml::Motion> &motions, const string &name )
{
  string file_name;
  file_name = string("./data/save/") + name + ".mot";

  //cout << "save file name : " << file_name << endl;
  std::ofstream file(file_name, std::ios::out);

  //motions
  file << motions.size() << endl;
  for (auto it_m = motions.begin(); it_m != motions.end(); ++it_m) {
	file<< it_m->color[0] << ' ' 
	  << it_m->color[1] << ' ' 
	  << it_m->color[2] << ' ' 
	  << it_m->size() << ' '
	  << endl;

	for (auto it_p = it_m->m_postures.begin(); it_p != it_m->m_postures.end(); ++it_p) {
	  const ml::Posture &p = *it_p;
	  file << p.time << ' ' << p.leftFootTouched << ' ' << p.rightFootTouched << ' ' << p.type_ << ' ' << p.object << endl;
	  const cml::vector3 &trans = p.m_trans;
	  file << trans[0] << ' ' << trans[1] << ' ' << trans[2] << ' ';
	  for (auto it = p.m_rotates.begin(); it != p.m_rotates.end(); ++it) {
		const cml::matrix3 &rot = *it;
		file<<	safe_double(rot(0,0)) << ' ' << safe_double(rot(0,1)) << ' ' << safe_double(rot(0,2)) << ' '
		  <<	safe_double(rot(1,0)) << ' ' << safe_double(rot(1,1)) << ' ' << safe_double(rot(1,2)) << ' '
		  <<	safe_double(rot(2,0)) << ' ' << safe_double(rot(2,1)) << ' ' << safe_double(rot(2,2)) << ' ';
	  }
	}
  }
  file.close();

  return file_name;
}

void ml::read_motions( vector<ml::Motion> &motions, const string &name ) 
{
  ml::Motion base_m;
  base_m.LoadAMC_with_contactInfo("./data/test_motion.amc", "./data/wd2.asf", true, 0.027);
  motions.clear();

  ifstream file("./data/save/" + name + ".mot", std::ios::in);
  if ( file.fail() ) {
	cout << "no mot file: " << name << endl;
	return;
  }

  int motion_size;
  file >> motion_size;
  for (int i_m = 0; i_m < motion_size; ++i_m) {
	ml::Motion m(base_m[0].body());
	file >> m.color[0] >> m.color[1] >> m.color[2];

	int posture_size;
	file >> posture_size;
	for (int i_p = 0; i_p < posture_size; ++i_p) {
	  ml::Posture p(base_m[0]);
	  file >> p.time >> p.leftFootTouched >> p.rightFootTouched >> p.type_ >> p.object;
	  p.type_ = -1;	  // boundary of patch type: 강제로 -1로 초기화 한다.
	  cml::vector3 &trans = p.m_trans;
	  file >> trans[0] >> trans[1] >> trans[2];
	  for (auto it = p.m_rotates.begin(); it != p.m_rotates.end(); ++it) {
		cml::matrix3 &rot = *it;
		file>>	rot(0,0) >> rot(0,1) >> rot(0,2) 
		  >>	rot(1,0) >> rot(1,1) >> rot(1,2)
		  >>	rot(2,0) >> rot(2,1) >> rot(2,2);
	  }
	  m.AddPosture(p);
	}
	motions.push_back(m);
  }
  file.close();
}