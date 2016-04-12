#include "stdafx.h"
#include "patch.h"

struct Patch_black;


ml::Posture& Boundary::posture()
{
  if (forward) {
	return p_motion->last_posture();
  } else {
	return p_motion->first_posture();
  }
}

const ml::Posture& Boundary::posture() const
{
  if (forward) {
	return p_motion->last_posture();
  } else {	
	return p_motion->first_posture();
  }
}

Patch::Patch( const Patch &other, bool only_boundary )
	: name(other.name)
{
	for (auto it = other.motions.begin(); it != other.motions.end(); ++it)
		add_motion_only_boundary(*it);
	set_boundaries();	
}

ml::Motion * Boundary::connected_motion() const
{
  if (forward) {
	return p_motion->connected_fore;
  } else {
	return p_motion->connected_back;
  }
}  


Patch::Patch( const Patch &other )
  : inner_cons(other.inner_cons), name(other.name), patch_object(other.patch_object), rel_pos_time(other.rel_pos_time) {
  for (auto it = other.motions.begin(); it != other.motions.end(); ++it) add_motion(*it);
  set_boundaries();
}

double Patch::get_begin_time() const {
  vector<double> times;
  transform(motions.begin(), motions.end(), back_inserter(times), [](const ml::Motion &m){return m.first_posture().time;});

  return *min_element(times.begin(), times.end());
}

double Patch::get_last_time() const {
  vector<double> times;
  transform(motions.begin(), motions.end(), back_inserter(times), [](const ml::Motion &m){return m.last_posture().time;});
  
  return *max_element(times.begin(), times.end());
}

void Patch::apply_transf( const cml::transf &t, double time /*= 0.0*/ ) {
  for_each(motions.begin(), motions.end(), [&t, time](ml::Motion &m){m.ApplyTransf(t, time);});
}

void Patch::translate( const cml::vector3 &v ) {
  apply_transf(cml::trans_transf(v));	
  std::ostringstream oss;
  oss << "translate " << v << endl;
  log += oss.str();
}

void Patch::translate_time( double time) {
  apply_transf(cml::identity_transf(), time);
  std::ostringstream oss;
  oss << "translate_time " << time << endl;
  log += oss.str();
}

void Patch::translate_origin() {
  vector<cml::vector2> trans2_v;
  vector<double> time_v;

  for (auto it = motions.begin(); it != motions.end(); ++it) {
	  auto it_p = it->begin();
	  trans2_v.push_back(cml::vec2(it_p->trans()));
	  time_v.push_back(it_p->time);

	  it_p = it->end()-1;
	  trans2_v.push_back(cml::vec2(it_p->trans()));
	  time_v.push_back(it_p->time);
  }

  cml::vector2 mean_trans2 = accumulate(trans2_v.begin(), trans2_v.end(), cml::vector2(0,0)) / trans2_v.size();
  cml::vector3 mean_trans3 = cml::vec3(mean_trans2);
  double mean_time = accumulate(time_v.begin(), time_v.end(), 0.0) / time_v.size();
	
  apply_transf(cml::trans_transf(-mean_trans3), -mean_time);
}

void Patch::rotate( double theta ) {
  apply_transf(cml::roty_transf(theta), 0.0);

  std::ostringstream oss;
  oss << "rotate " << theta << endl;
  log += oss.str();
}

void Patch::add_motion( const ml::Motion &m ) {
  motions.push_back(m);
}

void Patch::add_motion_only_boundary( const ml::Motion &m )
{
	ml::Motion m_bound;
	m_bound.copy_common(m);
	m_bound.AddPosture(m.first_posture());
	m_bound.AddPosture(m.last_posture());
	motions.push_back(move(m_bound));
}

void Patch::set_boundaries() {
  boundaries.clear();
  for (int i = 0; i < motions.size(); ++i) {
	Boundary b1, b2;
	b1.motion_index = i;
	b2.motion_index = i;
	b1.p_motion = &motions[i];
	b2.p_motion = &motions[i];
	b1.motion_index = b2.motion_index = i;
	b1.forward = false;
	b2.forward = true;

	boundaries.push_back(move(b1));
	boundaries.push_back(move(b2));
  }
}

void Patch::transform_between_posture( const ml::Posture & to_posture, const ml::Posture & from_posture ) 
{
  auto inv_from = cml::inverse(from_posture.rotate(0));
  cml::matrix3 diff_rot = cml::PlaneProject_mat3(to_posture.rotate(0) * inv_from);	  // !!!!!
  
  cml::transf diff_rot_transf = cml::make_transf(diff_rot, cml::vector3(0,0,0));
  apply_transf(diff_rot_transf);
  translate(cml::vec3(cml::vec2(to_posture.trans() - from_posture.trans())));

  double diff_time = to_posture.time - from_posture.time;
  translate_time(diff_time);
}

int Patch::get_boundary_type( size_t idx )
{
  return (boundaries[idx].forward ? motions[boundaries[idx].motion_index].last_posture().type_ : motions[boundaries[idx].motion_index].first_posture().type_);
}

double begin_time( const vector<shared_ptr<Patch>> &patches ) {
  vector<double> times;
  transform(patches.begin(), patches.end(), back_inserter(times), [](const shared_ptr<Patch> &p_patch){return p_patch->get_begin_time();});

  return *min_element(times.begin(), times.end());
}

double end_time( const vector<shared_ptr<Patch>> &patches ) {
  vector<double> times;
  transform(patches.begin(), patches.end(), back_inserter(times), [](const shared_ptr<Patch> &p_patch){return p_patch->get_last_time();});
	
  return *max_element(times.begin(), times.end());
}

map<string, int> get_end_num(const vector<shared_ptr<Patch>> &patches, double t_min, double t_max) {
  map<string, int> end_num;
  end_num["connect"] = 0;
  end_num["dead"] = 0;
  end_num["boundary"] = 0;

  for (auto it = patches.begin(); it != patches.end(); ++it) {
	map<string, int> end_num_pa = get_end_num(*it, t_min, t_max);
	end_num["connect"] += end_num_pa["connect"];
	end_num["dead"] += end_num_pa["dead"];
	end_num["boundary"] += end_num_pa["boundary"];
  }
  return end_num;
}

map<string, int> get_end_num(const shared_ptr<Patch> &patch, double t_min, double t_max) {
  map<string, int> end_num;
  end_num["connect"] = 0;
  end_num["dead"] = 0;
  end_num["boundary"] = 0;

  for (auto it_b = patch->boundaries.begin(); it_b != patch->boundaries.end(); ++it_b) ++end_num[get_end_type(*it_b, t_min, t_max)];

  return end_num;
}

string get_end_type(Boundary & b, double t_min, double t_max) {
  string end_type;
  if (b.posture().time < t_min || b.posture().time > t_max) {
	end_type = "boundary";
  } else if (b.is_connected()) {
	end_type = "connect";
  } else {
	end_type = "dead";
  }

  return end_type;
}

double color_set[9][3] = 
{
  {202. / 255., 84. / 255., 110. / 255.},
  {0., 0.4 ,1.0},
  {0.6, 0.6 ,0.0},
  {0.2, 0.5 ,0.2},
  {0. / 255., 136. / 255., 124. / 255.},
  {178./ 255., 122. / 255., 180. / 255.},
  {246. / 255., 139. / 255., 51. / 255.},
  {130. / 255., 0. / 255., 80. / 255.},
  {24. / 255., 148. / 255., 184. / 255.}
};

double color_set_twopart[8][3] = 
{
  {1, 0.2, 0.2},
  {1-0.1, 0.2+0.1, 0.2},
  {1-0.1, 0.2, 0.2+0.1},
  {1, 0.2-0.1, 0.2},
  {0.2, 0.2, 1},
  {0.2, 0.2+0.1, 1-0.1},
  {0.2+0.1, 0.2, 1.-0.1},
  {0.2, 0.2-0.1, 1},
};

cml::vector3 get_random_color() {
  double * col = color_set[random(0,8)];
  return cml::vector3(col[0], col[1], col[2]);
}

void get_next_color( double &r, double &g, double &b ) {
  static int cur_i = 0;
  while(1) {
	cur_i += 904;
	r = (cur_i % 11) * .063 + .1;
	g = ((cur_i / 11) % 13) * .054 + .1;
	b = ((cur_i / 143) % 11) * .063 + .1;

	if(r + g + b < 0.3 || r + g + b > 1.8) continue;
	break;
  }
}

void get_next_color_twopart( double &r, double &g, double &b ) {
  static int cur_i_twopart = 0;
  cur_i_twopart++;
  if (cur_i_twopart == 8)
    cur_i_twopart = 0;
  
  r = color_set_twopart[cur_i_twopart][0];  
  g = color_set_twopart[cur_i_twopart][1];  
  b = color_set_twopart[cur_i_twopart][2];  
}

cml::vector3 get_next_color() {
  double r, g, b;
  get_next_color(r,g,b);
  return cml::vector3(r,g,b);
}

cml::vector3 get_next_color_twopart() {
  double r, g, b;
  get_next_color_twopart(r,g,b);
  return cml::vector3(r,g,b);
}

cml::vector3 get_next_color_one() {
  return cml::vector3(color_set[0][0],color_set[0][1],color_set[0][2]);
}

void propagate_color(ml::Motion * p_motion, cml::vector3 &color) {
  if (p_motion->color[0] < 0) {
	p_motion->color = color;
	if (p_motion->connected_fore != nullptr) propagate_color(p_motion->connected_fore, color);
	if (p_motion->connected_back != nullptr) propagate_color(p_motion->connected_back, color);
  }
}

pair<int, int> connected_patch_motion( const ml::Motion * p_motion, const vector<shared_ptr<Patch>> &patches ) {
  if (p_motion == nullptr) return pair<int,int>(-1,-1);

  for (int i = 0; i < patches.size(); ++i) {
	for (int j = 0; j < patches[i]->motions.size(); ++j) {
	  if (p_motion == &(patches[i]->motions[j])) return pair<int,int>(i,j);
	}
  }
  ASSERT(false);
}

pair<int, int> connected_patch_boundary( const Boundary &b1, const vector<shared_ptr<Patch>> &patches )
{
  for (int i = 0; i < patches.size(); ++i) {
	for (int j = 0; j < patches[i]->boundaries.size(); ++j) {
	  const Boundary & b2 = patches[i]->boundaries[j];
	  if (b1.forward != b2.forward && b1.connected_motion() == b2.p_motion) return pair<int,int>(i,j);
	}
  }	
  ASSERT(false);
}

void set_color( vector<shared_ptr<Patch>> &patches ) {
	for (auto it = patches.begin(); it != patches.end(); ++it) {
	  for (auto it_m = (*it)->motions.begin(); it_m != (*it)->motions.end(); ++it_m) {
		it_m->color = cml::vector3(-1,-1,-1);
	  }
	}

	for (auto it = patches.begin(); it != patches.end(); ++it) {
	  for (auto it_m = (*it)->motions.begin(); it_m != (*it)->motions.end(); ++it_m)
		propagate_color(&(*it_m), get_next_color());
	}
}

void set_color_twopart( vector<shared_ptr<Patch>> &patches ) {
  for (auto it = patches.begin(); it != patches.end(); ++it) {
    for (auto it_m = (*it)->motions.begin(); it_m != (*it)->motions.end(); ++it_m) {
      it_m->color = cml::vector3(-1,-1,-1);
    }
  }

  for (auto it = patches.begin(); it != patches.end(); ++it) {
    for (auto it_m = (*it)->motions.begin(); it_m != (*it)->motions.end(); ++it_m)
      propagate_color(&(*it_m), get_next_color_twopart());
  }
}

void set_color_from_back( vector<shared_ptr<Patch>> &patches ) {
  for (auto it = patches.begin(); it != patches.end(); ++it) {
    for (auto it_m = (*it)->motions.begin(); it_m != (*it)->motions.end(); ++it_m)
    {
      if (it_m->color[0] < 0) {
        it_m->color = it_m->connected_back->color;
      }
    }
  }
}

string write_patches( const vector<shared_ptr<Patch>> & patches, const string &name) 
{
  char Date[10];
  char Time[10];
  _strdate_s(Date);
  _strtime_s(Time);
  string Date_;
  remove_copy(Date, Date+5, back_inserter(Date_), '/');
  string Time_;
  remove_copy(Time, Time+8, back_inserter(Time_), ':');

  string file_name;
  if (name.size() == 0) {
	file_name = string("./data/save/") + Date_ + '_' + Time_ + ".pat";
  } else {
	file_name = string("./data/save/") + name + ".pat";
  }
  //cout << "save file name : " << file_name << endl;
  std::ofstream file(file_name, std::ios::out);  
  file << patches.size() << endl;
	
  //patch
  for (auto it_p = patches.begin(); it_p != patches.end(); ++it_p) {
	const Patch &pa = **it_p;
	file << pa.name << endl;

	//inner_cons
	const vector<Constraint> & inner_cons = pa.inner_cons;
	file << inner_cons.size() << endl;
	for (auto it_c = inner_cons.begin(); it_c != inner_cons.end(); ++it_c) {
	  const Constraint &con = *it_c;
	  file << con.type << ' ' << safe_int(con.m_int[0]) << ' ' << safe_int(con.m_int[1]) << ' ' << safe_int(con.m_int[2]) << ' ' << safe_int(con.m_int[3]) << ' ' << safe_double(con.m_double[0]) << ' ' << safe_double(con.m_double[1]) << ' ' << safe_double(con.m_vector2d[0][0]) << ' ' << safe_double(con.m_vector2d[0][1]) << ' ' << safe_double(con.m_vector2d[1][0]) << ' ' << safe_double(con.m_vector2d[1][1]) << endl;
	}
	//motions
	file << pa.motions.size() << endl;
	for (auto it_m = pa.motions.begin(); it_m != pa.motions.end(); ++it_m) {
	  file << it_m->color[0] << ' ' 
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
  } file << endl;

  //patch connection
  for (int i_pa = 0; i_pa < patches.size(); ++i_pa) {
	for (int i_m = 0; i_m < patches[i_pa]->motions.size(); ++i_m) {
	  file << i_pa << ' ' << i_m << ' ';
	  const ml::Motion &m = patches[i_pa]->motions[i_m];
	  {
		pair<int, int> connected = connected_patch_motion(m.connected_back, patches);
		file << connected.first << ' ' << connected.second << ' ';
	  }
	  {
		pair<int, int> connected = connected_patch_motion(m.connected_fore, patches);
		file << connected.first << ' ' << connected.second << ' ';
	  }
	}
  }
  file.close();

  return file_name;
}

void read_patches( vector<shared_ptr<Patch>> & patches, const string &name ) 
{
  ml::Motion base_m;
  base_m.LoadAMC_with_contactInfo("./data/1/test_motion.amc", "./data/wd2.asf", true, 0.027);

  patches.clear();

  ifstream file("./data/save/" + name + ".pat", std::ios::in);
  if ( file.fail() ) {
	cout << "no pat file: " << name << endl;
	return;
  }

  //ÀÐÀÚ
  int patch_size;
  file >> patch_size;
  Patch base_pa;

  for(int i_pa = 0; i_pa < patch_size; ++i_pa) {
	patches.push_back(shared_ptr<Patch>(new Patch(base_pa)));
	Patch &pa = **(patches.end()-1);
	file >> pa.name;

	//inner_cons
	int cons_size;
	file >> cons_size;
	for (int i_c = 0; i_c < cons_size; ++i_c) {
	  Constraint con;
	  file >> con.type >> con.m_int[0] >> con.m_int[1] >> con.m_int[2] >> con.m_int[3] >> con.m_double[0] >> con.m_double[1] >> con.m_vector2d[0][0] >> con.m_vector2d[0][1] >> con.m_vector2d[1][0] >> con.m_vector2d[1][1];
	  pa.inner_cons.push_back(con);
	}

	//motions
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
	  pa.motions.push_back(m);
	}		
	//boundary
	pa.set_boundaries();
  }

  //patch connection
  for (int i_pa = 0; i_pa < patches.size(); ++i_pa) {
	for (int i_m = 0; i_m < patches[i_pa]->motions.size(); ++i_m) {
	  int pa, m, pa_back, m_back, pa_fore, m_fore;
	  file >> pa >> m >> pa_back >> m_back >> pa_fore >> m_fore;
	  if (pa != i_pa || m != i_m) ASSERT(false);
			
	  ml::Motion &mot = patches[pa]->motions[m];
	  if (pa_back == -1) mot.connected_back = nullptr;
	  else mot.connected_back = &(patches[pa_back]->motions[m_back]);
			
	  if (pa_fore == -1) mot.connected_fore = nullptr;
	  else mot.connected_fore = &(patches[pa_fore]->motions[m_fore]);
	}
  }
  //patch connection
  /*vector<vector<int>> connect_info;
  for (int i_pa = 0; i_pa < patches.size(); ++i_pa)
  {
	  for (int i_m = 0; i_m < patches[i_pa]->motions.size(); ++i_m)
	  {
		  vector<int> connect;
		  for (int i = 0; i < 6; ++i)
		  {
			  int integer;
			  file >> integer;
			  connect.push_back(integer);
		  }
		  connect_info.push_back(connect);
	  }
  }*/
  //
  file.close();
}

void remove_dangling_patch( vector<shared_ptr<Patch> > & cooked_patches, const vector<string> &deathNote ) 
{
  for (auto it = cooked_patches.begin(); it != cooked_patches.end(); ) {    
	if (find(deathNote.begin(), deathNote.end(), (*it)->name) != deathNote.end()) {
	  it = cooked_patches.erase(it);
	} else {
		bool isBreak = false;
		for (auto bt = (*it)->boundaries.begin(); bt != (*it)->boundaries.end(); ++bt) {
			if (bt->posture_type() == -1) {
				it = cooked_patches.erase(it);
				isBreak = true;
				break;
			}	  
		} if (!isBreak) ++it;
	}    
  }
}

void copy_patches(vector<shared_ptr<Patch>> &to, const vector<shared_ptr<Patch>> &from)
{
  for (auto it = from.begin(); it != from.end(); ++it) {
	const Patch &pa = *(*it);
	shared_ptr<Patch> pa_new(new Patch(pa));
	to.push_back(pa_new);
  }
}

vector<double> edit_degree( const Patch &pa, const Patch &pa_ori )
{
  vector<double> diffs;

  vector<double> linear_vel_diff;
  vector<double> angular_vel_diff;
  vector<double> time_diff;

  for( size_t i=0; i<pa.motions.size(); ++i) {
	const ml::Motion &m = pa.motions[i];
	const ml::Motion &m_ori = pa_ori.motions[i];

	for( size_t f=1; f<m.size(); ++f) {
	  double linear_vel = (m[f].trans() - m[f-1].trans()).length();
	  double linear_vel_ori = (m_ori[f].trans() - m_ori[f-1].trans()).length();

	  double linear_diff = fabs(linear_vel - linear_vel_ori);
	  linear_vel_diff.push_back(linear_diff);

	  double angular_vel = cml::log_mat3(m[f].rotate(0) * cml::inverse(m[f-1].rotate(0))).length();
	  double angular_vel_ori = cml::log_mat3(m_ori[f].rotate(0) * cml::inverse(m_ori[f-1].rotate(0))).length();
	  double angular_diff = fabs(angular_vel - angular_vel_ori);
	  angular_vel_diff.push_back(angular_diff); 

	  double time = fabs(m[f].time - m[f-1].time);
	  double time_ori = fabs(m_ori[f].time - m_ori[f-1].time);
			
	  time_diff.push_back(fabs(time_ori - time));
	}
  }

  diffs.push_back(*max_element(linear_vel_diff.begin(), linear_vel_diff.end()));
  diffs.push_back(*max_element(angular_vel_diff.begin(), angular_vel_diff.end()));
  diffs.push_back(*max_element(time_diff.begin(), time_diff.end()));

  return diffs;
}

bool edit_ok( const Patch &pa, const map<string, Patch> * patch_type )
{
  auto degree = edit_degree(pa, (patch_type->find(pa.name))->second);
  if (degree[0] > 0.031 - 0.001 || degree[1] > 0.129 + 0.001 - 0.005|| degree[2] > 0.57 - 0.02) {
  //if (degree[0] > 0.031 - 0.001 + 0.003|| degree[1] > 0.129 + 0.001 - 0.005 + 0.01|| degree[2] > 0.57 - 0.02 + 0.05) {
	return false;
  } else {
	return true;
  }
}

ml::Motion * get_first_motion(ml::Motion * cur_m)
{
	if (cur_m->connected_back != nullptr) {
		return get_first_motion(cur_m->connected_back);
	} else {
	  return cur_m;
	}
}

void make_long_motion(ml::Motion * p_motion, ml::Motion & merge_motion, map<pair<int,int>, pair<int,int>> &trans_map, const vector<shared_ptr<Patch>> & patches, int long_motion_index)
{
  //insert trans_map
  pair<int,int> patch_motion = connected_patch_motion(p_motion, patches);
  int transform_frame = merge_motion.size()-1;
  if (merge_motion.size() == 0) transform_frame = 0;
  trans_map[patch_motion] = make_pair(long_motion_index, transform_frame);

  //merge_motion
  if (merge_motion.size() == 0) {
	merge_motion = *p_motion;
  } else {
	merge_motion.m_postures.insert(merge_motion.m_postures.end(), p_motion->begin() + 1, p_motion->end());
  }
  if (p_motion->connected_fore != nullptr) make_long_motion(p_motion->connected_fore, merge_motion, trans_map, patches, long_motion_index);
}

Constraint convert_con(const Constraint &con, const vector<shared_ptr<Patch>> &patches, map<pair<int, int>, pair<int,int>> &trans_map, const Patch &cur_patch)
{
  int group1 = con.m_int[0];
  int index1 = con.m_int[1];
  int group2 = con.m_int[2];
  int index2 = con.m_int[3];

  const ml::Motion *m1 = &(cur_patch.motions[group1]);
  const ml::Motion *m2 = &(cur_patch.motions[group2]);
  pair<int,int> patch_motion1 = connected_patch_motion(m1, patches);
  pair<int,int> patch_motion2 = connected_patch_motion(m2, patches);

  if (trans_map.find(patch_motion1) == trans_map.end()) cout << "error1" << endl;
  if (trans_map.find(patch_motion2) == trans_map.end()) cout << "error2" << endl;

  pair<int,int> trans_result1 = trans_map[patch_motion1];
  pair<int,int> trans_result2 = trans_map[patch_motion2];

  Constraint con_converted = con;
  con_converted.m_int[0] = trans_result1.first;
  con_converted.m_int[1] += trans_result1.second;
  con_converted.m_int[2] = trans_result2.first;
  con_converted.m_int[3] += trans_result2.second;

  return con_converted;
}

void convert_connected_motions( const vector<shared_ptr<Patch>> & patches, vector<ml::Motion> & motions, vector<Constraint> &inner_cons )
{
  set<ml::Motion *> first_motions;
  for (auto it = patches.begin(); it != patches.end() ; ++it) {
	for (auto it_m = (*it)->motions.begin(); it_m != (*it)->motions.end(); ++it_m) {
	  ml::Motion * first_motion = get_first_motion(&(*it_m));
	  first_motions.insert(first_motion);
	}
  }

  map<pair<int, int>, pair<int,int>> trans_map;
  for (auto it = first_motions.begin(); it != first_motions.end(); ++it) {
	ml::Motion merge_motion;
	make_long_motion(*it, merge_motion, trans_map, patches, motions.size());
	motions.push_back(merge_motion);
  }

  //convert inner_cons
  inner_cons.clear();
  for (auto it = patches.begin(); it != patches.end(); ++it) {
	for (auto it_c = (*it)->inner_cons.begin(); it_c != (*it)->inner_cons.end(); ++it_c) {
	  if (it_c->type == "rel_pos") {
		Constraint &con_converted = convert_con(*it_c, patches, trans_map, **it);
		inner_cons.push_back(con_converted);
	  }
	  if (it_c->type == "same_time") {
		Constraint &con_converted = convert_con(*it_c, patches, trans_map, **it);
		inner_cons.push_back(con_converted);
	  }
	}
  }
}


size_t find_boundary_idx(const shared_ptr<Patch> patch, const int motion_index, const bool forw_dir)
{
  for (size_t i=0; i<patch->boundaries.size(); ++i) {
	if ((patch->boundaries[i].p_motion == &patch->motions[motion_index]) && (patch->boundaries[i].forward==forw_dir))
	  return i;
  }
  ASSERT(false);
}

struct Patch_black 
{
  int blacked_motion[12];
  int real_m;
  Patch_black(const int k): real_m(k) { init(real_m); }
  void init(const int k) {
	for (size_t i=0; i<k; ++i)	
	  blacked_motion[i] = 0;	// false;
	for (size_t i=k; i<12; ++i)
	  blacked_motion[i] = 1;	// true;
  }
  bool is_blacked(int k) const {
	if (blacked_motion[k] == 0) {
	  return false;
	} else {
	  return true;
	}
  }
  void blacking(int k) {
	blacked_motion[k] = 1;
  }
  bool is_blacked() const {
	for( int i=0; i<real_m; i++)
	  if( blacked_motion[i]==0)
		return false;
	return true;
  }
};

void convert_patch_to_motion(vector<ml::Motion> &motions, const vector< shared_ptr<Patch> > &patches)
{
  // initialize Patch_black struct
  vector<Patch_black> blacked;
  blacked.reserve(patches.size());
  for (size_t i=0; i<patches.size(); ++i) {
	Patch_black temp(patches[i]->motions.size());
	blacked.push_back(temp);
  }

  vector< list<int> > patches_reordered_on_time;	// new_motion_id = 12*patch_idx+motion_idx
  for (auto itb = blacked.begin(); itb != blacked.end(); ++itb) {
	if (!itb->is_blacked()) {
	  for (int idx_m=0; idx_m<itb->real_m; ++idx_m) {
		if (!itb->is_blacked(idx_m)) {					
		  list<int> motion_line;
		  const int pat_id = static_cast<int>(itb - blacked.begin());
		  motion_line.push_back(pat_id*12 + idx_m);
		  blacked[pat_id].blacking(idx_m);

		  // traverse forward
		  int fit_patch = pat_id;
		  int idx_b = find_boundary_idx(patches[fit_patch], idx_m, false);

		  while (patches[fit_patch]->boundaries[idx_b].is_connected()) {
			pair<int,int> pat_bound = connected_patch_boundary(patches[fit_patch]->boundaries[idx_b], patches);	// finds forward directional boundary!
			fit_patch = pat_bound.first;
			int it_motion = patches[fit_patch]->boundaries[pat_bound.second].motion_index;
			motion_line.push_front( fit_patch*12 + it_motion);
			blacked[fit_patch].blacking(it_motion);
			idx_b = find_boundary_idx( patches[fit_patch], it_motion, false);
		  }

		  // traverse backward
		  int bit_patch = pat_id;
		  idx_b = find_boundary_idx( patches[bit_patch], idx_m, true);

		  while (patches[bit_patch]->boundaries[idx_b].is_connected()) {
			pair<int,int> pat_bound = connected_patch_boundary( patches[bit_patch]->boundaries[idx_b], patches );
			bit_patch = pat_bound.first;
			int it_motion = patches[bit_patch]->boundaries[pat_bound.second].motion_index;
			motion_line.push_back( bit_patch*12 + it_motion);
			blacked[bit_patch].blacking(it_motion);
			idx_b = find_boundary_idx( patches[bit_patch], it_motion, true);
		  }
		  patches_reordered_on_time.push_back(motion_line);
		}
	  }
	}
  }

  motions.clear();
  motions.reserve(patches_reordered_on_time.size());
  for (auto it=patches_reordered_on_time.begin(); it!=patches_reordered_on_time.end(); ++it) {
	auto mit=it->begin();
	ml::Motion copy_motion(patches[(*mit)/12]->motions[(*mit)%12]);
	if (++mit==it->end()) {
	  motions.push_back(copy_motion);
	  continue;
	}
	for ( ; mit!=it->end(); ++mit) {
	  const ml::Motion &patch_mot = patches[(*mit)/12]->motions[(*mit)%12];
	  size_t add_resize = patch_mot.size();
	  copy_motion.m_postures.resize(copy_motion.size() + add_resize);
	  copy_backward(patch_mot.begin(), patch_mot.begin()+add_resize, copy_motion.end());
	}
	motions.push_back(copy_motion);
  }
}

