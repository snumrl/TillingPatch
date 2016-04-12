#include "stdafx.h"
#include "motion_edit.h"

static double radian(cml::vector2d v)
{
	if (v[1] >= 0)
		return acos(v[0] / length(v));
	else
		return cml::constants<double>::two_pi() - acos(v[0] / length(v));
}

static double get_radian(const std::vector<cml::vector3d> &vec3_v, int index, int width)
{
	int last_index = (int)vec3_v.size() -1;

	int p_index = index - width;
	int n_index = index + width;

	if (p_index < 0)
		p_index = 0;
	if (n_index > last_index)
		n_index = last_index;

	cml::vector2d p_vec2;
	p_vec2[0] = vec3_v[p_index][2];
	p_vec2[1] = vec3_v[p_index][0];

	cml::vector2d n_vec2;
	n_vec2[0] = vec3_v[n_index][2];
	n_vec2[1] = vec3_v[n_index][0];

	return radian(n_vec2-p_vec2);
}

static double get_radian(const std::vector<cml::vector2d> &vec2_v, int index, int width)
{
	int last_index = (int)vec2_v.size() -1;

	int p_index = index - width;
	int n_index = index + width;

	if (p_index < 0)
		p_index = 0;
	if (n_index > last_index)
		n_index = last_index;

	return radian(vec2_v[n_index]-vec2_v[p_index]);
}

static double get_diff_angle(double to, double from)
{
	//angle -> matrix
	double a[4] = {cos(to), -sin(to), sin(to), cos(to)};
	double b[4] = {cos(from), -sin(from), sin(from), cos(from)};
	// c = inverse b
	double c[4] = {b[3], -b[1], -b[2], b[0]}; 

	// z = a * inverse b
	double z[4] = {
		a[0] * c[0] + a[1] * c[2],
		a[0] * c[1] + a[1] * c[3],
		a[2] * c[0] + a[3] * c[2],
		a[2] * c[1] + a[3] * c[3]
	};

	//error handling
	if (z[0] > 1)
		z[0] = 1.;
	else if(z[0] < -1)
		z[0] = -1.;

	//matrix -> angle
	double theta;
	if (z[2] >= 0)
		theta = acos(z[0]);
	else
		theta = -acos(z[0]);

	return theta;
}

static std::vector<bool> get_defineds_by_angular_velocity(std::vector<double> &rads, double threshhold)
{
	std::vector<bool> defineds_by_angular_vel(rads.size(), true);
	int last_index = rads.size() - 1;
	for (int i = 0; i < rads.size()-1; ++i)
	{
		double prev_rad = rads[i];
		double next_rad = rads[i+1];

		if (fabs(next_rad - prev_rad) > threshhold)
		{
			int width = 4;
			for (int w = 0; w < width; ++w)
			{
				int p_index = i - w;
				int n_index = i+1 + w;

				if (p_index < 0)
					p_index = 0;
				if (n_index > last_index)
					n_index = last_index;
				defineds_by_angular_vel[p_index] = false;
				defineds_by_angular_vel[n_index] = false;
			}
		}
	}

	return defineds_by_angular_vel;
}

static void radian_restore_by_blending(std::vector<double> &rads, std::vector<bool> &is_defineds)
{
	//assert(rads.size() == is_defineds.size());

	std::vector<int> boundary;
	for(int i = 1; i < is_defineds.size(); ++i)
	{
		if (is_defineds[i] == false)
		{
			boundary.push_back(i-1);
			while(true) {
				++i;
				if (is_defineds[i] == true || i == (is_defineds.size() - 1))
					break;
			}
			boundary.push_back(i);
		}
	}

	for(int i = 0; i < boundary.size(); i += 2)
	{
		int before_i = boundary[i];
		int after_i = boundary[i+1];

		double before_val = rads[before_i];
		double after_val = rads[after_i];

		if (is_defineds[before_i] == true && is_defineds[after_i] == true)
		{
			for(int j = before_i+1; j < after_i; ++j)
			{
				double diff = get_diff_angle(after_val, before_val);
				double j_val = before_val + diff * float(j - before_i) / float(after_i - before_i);

				rads[j] = j_val;
			}
		}
		else if (is_defineds[before_i] == false && is_defineds[after_i] == true)
		{
			for(int j = before_i; j < after_i; ++j)
			{
				double j_val = after_val;

				rads[j] = j_val;
			}
		}
		else if (is_defineds[before_i] == true && is_defineds[after_i] == false)
		{
			for(int j = before_i+1; j <= after_i; ++j)
			{
				double j_val = before_val;

				rads[j] = j_val;
			}
		}
		else
		{
			//		assert(false);
		}
	}
}

double motion_edit( ml::Motion &motion, std::map<int, double> cons_time, std::map<int, cml::vector3d> &cons_pos, std::map<int, cml::vector3d> &cons_ori, double rigid_thres /*= 0.027*/ )
{
	std::vector<cml::vector3d> points;
	for ( int i=0; i < motion.size(); i++ )
	{
		points.push_back(motion.posture(i).trans());
	}

	int size = motion.size();
	std::vector<cml::vector3d> original_path = points;

	//////////////////////////////////////////// edit points////
	std::vector<cml::vector2d> points_2d;
	for (int i = 0; i < points.size(); ++i)
	{
		points_2d.push_back(cml::vector2d(points[i][2], points[i][0]));
	}
	std::map<int, cml::vector2d> cons_pos_2d;
	for (auto it=cons_pos.begin() ; it != cons_pos.end(); ++it)
	{	
		int i = (*it).first;
		cml::vector3d &p = (*it).second;
		cons_pos_2d[i] = cml::vector2d(p[2], p[0]);
	}
	std::map<int, cml::vector2d> cons_ori_2d;
	for (auto it=cons_ori.begin() ; it != cons_ori.end(); ++it)
	{	
		int i = (*it).first;
		cml::vector3d &ori = (*it).second;
		cons_ori_2d[i] = cml::vector2d(ori[2], ori[0]);
	}
	double error = edit2D(points_2d, cons_pos_2d, cons_ori_2d, get_cons_rigid_2d(points_2d, rigid_thres));
	for (int i = 0; i < points.size(); ++i)
	{
		points[i][0] = points_2d[i][1];
		points[i][2] = points_2d[i][0];
	}
	//////////////////////////////////////////////////////////////

	for(int i=0; i < size; ++i) 
	{
		motion.posture(i).trans(points[i]);
	}

	std::vector<double> diff_rads;
	for(int i=0; i < size; ++i)
	{
		double diff_rad = get_diff_angle(get_radian(original_path, i, 1), get_radian(points, i, 1) );
		diff_rads.push_back(diff_rad);
	}
	
	/// tangent flipping 
	std::vector<bool> defineds_by_angular_vel = get_defineds_by_angular_velocity(diff_rads, 0.12); // threshold direction
	radian_restore_by_blending(diff_rads, defineds_by_angular_vel);
	///
	
	for(int i=0; i < size; ++i) 
	{
		cml::matrix3 cur_mat = motion.posture(i).rotate(0);
		cml::matrix3 rot_mat = cml::roty_mat(-diff_rads[i]);
		motion.posture(i).rotate(0, rot_mat * cur_mat);
	}

	if (cons_time.size() == 2)
	{
		auto it = cons_time.begin();
		int index1 = it->first;
		double time1 = it->second;
		it++;
		int index2 = it->first;
		double time2 = it->second;
		if (index2 < index1)
		{
			swap(index1, index2);
			swap(time1, time2);
		}

		double time_diff = time2 - time1;
		for (int i = index1; i <= index2; ++i)
		{
			double ratio = (i - index1) / (double)(index2 - index1);
			motion.posture(i).time = time1 + time_diff * ratio;
		}

	}
	return error;
}

double multi_motion_edit( vector<ml::Motion*> &motions, const vector<Constraint> &constraints, double rigid_thres /*= 0.027*/ )
{
  vector<vector<cml::vector2d>> multi_points;
  for (auto it = motions.begin(); it != motions.end(); ++it) {
	vector<cml::vector2d> single_points;
	transform((*it)->begin(), (*it)->end(), back_inserter(single_points), [](ml::Posture& p){return cml::vec2(p.trans());});
	multi_points.push_back(single_points);
  }
  vector<vector<cml::vector2d>> multi_points_ori(multi_points);

  double error = multi_edit2D(multi_points, constraints, rigid_thres);

  for (int group = 0; group < multi_points.size(); ++group) {
	vector<cml::vector2d> &points = multi_points[group];
	vector<cml::vector2d> &points_ori = multi_points_ori[group];
	ml::Motion& mot = *motions[group];

	std::vector<double> diff_rads;
	for (int i=0; i < points.size(); ++i) diff_rads.push_back(get_diff_angle(get_radian(points_ori, i, 1), get_radian(points, i, 1) ));

	// tangent flipping 
	radian_restore_by_blending(diff_rads, get_defineds_by_angular_velocity(diff_rads, 0.12));

	// set rotate
	for(int i = 0; i < mot.size(); ++i) {
	  cml::matrix3 cur_mat = mot.posture(i).rotate(0);
	  cml::matrix3 rot_mat = cml::roty_mat(-diff_rads[i]);
	  mot.posture(i).rotate(0, rot_mat * cur_mat);
	}

	//set trans
	for (int i = 0; i < mot.size(); ++i) {
	  mot[i].m_trans[0] = points[i][1];
	  mot[i].m_trans[2] = points[i][0];
	}
  }

  //time
  vector<vector<cml::vector2d>> multi_points_time;
  for (auto it = motions.begin(); it != motions.end(); ++it) {
	vector<cml::vector2d> single_points_time;
	transform((*it)->begin(), (*it)->end(), back_inserter(single_points_time), [](ml::Posture& p){return cml::vector2d(p.time, 0.0);});
	multi_points_time.push_back(single_points_time);
  }
	
  double error_time = multi_edit2D_time(multi_points_time, constraints, rigid_thres);

  for (int group = 0; group < multi_points_time.size(); ++group) {
	vector<cml::vector2d> &points_time = multi_points_time[group];
	ml::Motion& mot = *motions[group];

	//set trans
	for (int i = 0; i < mot.size(); ++i) {
	  mot[i].time = points_time[i][0];
	}
  }

  return error + error_time;
}

double multi_motion_edit( vector<ml::Motion> &motions, const vector<Constraint> &constraints, double rigid_thres /*= 0.027*/ )
{
	vector<ml::Motion*> p_motions;
	for (auto it = motions.begin(); it != motions.end(); ++it)
		p_motions.push_back(&(*it));
	return multi_motion_edit(p_motions, constraints, rigid_thres);
}

void add_cons_pos( vector<Constraint> &cons, int group, int index, cml::vector3d v )
{
	Constraint con;
	con.type = "pos";
	con.m_int[0] = group;
	con.m_int[1] = index;
	con.m_vector2d[0] = cml::vec2(v);
	cons.push_back(con);
}

void add_cons_dir( vector<Constraint> &cons, int group, int index, cml::vector3d v )
{
	Constraint con;
	con.type = "dir";
	con.m_int[0] = group;
	con.m_int[1] = index;
	con.m_vector2d[0] = cml::vec2(v);
	cons.push_back(con);
}

void add_cons_time( vector<Constraint> &cons, int group, int index, double t )
{
	Constraint con;
	con.type = "time";
	con.m_int[0] = group;
	con.m_int[1] = index;
	con.m_double[0] = t;
	cons.push_back(con);
}

void add_cons_pin( vector<Constraint> &cons, int group, const ml::Motion & motion, bool last )
{
	if (last == true) {
		add_cons_pos(cons, group, motion.size()-1,  (motion.end()-1)->trans());
		add_cons_pos(cons, group, motion.size()-2,  (motion.end()-2)->trans());
		add_cons_dir(cons, group, motion.size()-1,  (motion.end()-1)->trans() - (motion.end()-2)->trans());
		add_cons_time(cons, group, motion.size()-1,  (motion.end()-1)->time);
	} else {
		add_cons_pos(cons, group, 0,  (motion.begin())->trans());
		add_cons_pos(cons, group, 1,  (motion.begin()+1)->trans());
		add_cons_dir(cons, group, 0,  (motion.begin()+1)->trans() - (motion.begin())->trans());
		add_cons_time(cons, group, 0,  (motion.begin())->time);
	}
}

void add_cons_pin( vector<Constraint> &cons, int group, const ml::Motion & motion, int index )
{
	add_cons_pos(cons, group, index, (motion.begin()+index)->trans());
	add_cons_pos(cons, group, index+1, (motion.begin()+index+1)->trans());
	add_cons_time(cons, group, index,  (motion.begin()+index)->time);
}

void add_cons_rel_pos_oneside( vector<Constraint> &cons, const vector<ml::Motion> &motions, int group1, int index1, int group2, int index2 )
{
	Constraint con;
	con.type = "rel_pos";

	cml::vector2 p1_plus1 = cml::vec2(motions[group1].posture(index1+1).trans());
	cml::vector2 p1_minus1 = cml::vec2(motions[group1].posture(index1-1).trans());
	cml::vector2 p1 = cml::vec2(motions[group1].posture(index1).trans());
	cml::vector2 p2 = cml::vec2(motions[group2].posture(index2).trans());

	cml::vector2 global_v =  p2 - p1;
	cml::vector2 x_axis = p1_plus1 - p1_minus1;
	cml::vector2 y_axis(-x_axis[1], x_axis[0]);

	double x = dot(global_v, x_axis) / x_axis.length() / x_axis.length();
	double y = dot(global_v, y_axis) / y_axis.length() / y_axis.length();

	con.m_int[0] = group1;
	con.m_int[1] = index1;
	con.m_int[2] = group2;
	con.m_int[3] = index2;
	con.m_vector2d[0] = cml::vector2(x, y);
	cons.push_back(con);
}

void add_cons_rel_pos( vector<Constraint> &cons, const vector<ml::Motion> &motions, int group1, int index1, int group2, int index2 )
{
	add_cons_rel_pos_oneside(cons, motions, group1, index1, group2, index2);
	add_cons_rel_pos_oneside(cons, motions, group2, index2, group1, index1);
}

void add_cons_rel_pos_same_time( vector<Constraint> &cons, const vector<ml::Motion> &motions, int group1, int index1, int group2, int index2 )
{
	add_cons_rel_pos(cons, motions, group1, index1, group2, index2);
	add_cons_same_time(cons, group1, index1, group2, index2);
}

void add_cons_same_pos( vector<Constraint> &cons, int group1, int index1, int group2, int index2 )
{
	Constraint con;
	con.type = "same_pos";
	con.m_int[0] = group1;
	con.m_int[1] = index1;
	con.m_int[2] = group2;
	con.m_int[3] = index2;
	cons.push_back(con);
}

void add_cons_same_dir( vector<Constraint> &cons, int group1, int index1, int group2, int index2 )
{
	Constraint con;
	con.type = "same_dir";
	con.m_int[0] = group1;
	con.m_int[1] = index1;
	con.m_int[2] = group2;
	con.m_int[3] = index2;
	cons.push_back(con);
}

void add_cons_same_time( vector<Constraint> &cons, int group1, int index1, int group2, int index2 )
{
	Constraint con;
	con.type = "same_time";
	con.m_int[0] = group1;
	con.m_int[1] = index1;
	con.m_int[2] = group2;
	con.m_int[3] = index2;
	cons.push_back(con);
}

void add_cons_same_pos_dir_time( vector<Constraint> &cons, int group1, int index1, int group2, int index2 )
{
	add_cons_same_pos(cons, group1, index1, group2, index2);
	add_cons_same_dir(cons, group1, index1, group2, index2);
	add_cons_same_time(cons, group1, index1, group2, index2);
}

void add_phase1_info( vector<Constraint> &cons)
{
	Constraint con;
	con.type = "phase1";
	cons.push_back(con);
}


