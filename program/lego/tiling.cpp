#include "StdAfx.h"
#include "tiling.h"
#include "get_patch.h"

#include <fstream>


map<string, int> get_boundary_num( const vector<shared_ptr<Patch>> &patches ) 
{
	map<string, int> boundary_num;
	boundary_num["stitch"] = 4;

	return boundary_num;
}

ConnectInfo::ConnectInfo( int b1, int pa2, int b2, double distance)
{
	this->b1 = b1;
	this->pa2 = pa2;
	this->b2 = b2;
	this->distance = distance;
	this->trans_pos = cml::vector3(0,0,0);
}

ConnectInfo::ConnectInfo( int b1, int pa2, int b2, double distance, cml::vector3 trans_pos )
{
	this->b1 = b1;
	this->pa2 = pa2;
	this->b2 = b2;
	this->distance = distance;
	this->trans_pos = trans_pos;
}

double get_distance_rot(const cml::matrix3 &m1, const cml::matrix3 &m2)
{
	return cml::length(cml::log_mat3(cml::PlaneProject_mat3(m2 * cml::inverse(m1))));
}
double get_distance_rot_fast(const cml::matrix3 &m1, const cml::matrix3 &m2)
{
	return cml::length(cml::log_mat3(m2 * cml::inverse(m1)));
}

pair<double, cml::vector3> diff_trans(const cml::vector3 &p1, const cml::vector3 &p2, const Env &env)
{
	double min_len = 10000000.0;
	cml::vector3 min_trans;

	int x_diff = 0;
	if (env.is_cyclic.find("x")->second)
		x_diff = 1;
	int z_diff = 0;
	if (env.is_cyclic.find("z")->second)
		z_diff = 1;

	for (int x_i = -x_diff; x_i <= x_diff; ++x_i)
	{
		for (int z_i = -z_diff; z_i <= z_diff; ++z_i)
		{
			double trans_x = (double)x_i*(env.range.find("x_max")->second - env.range.find("x_min")->second);
			double trans_z = (double)z_i*(env.range.find("z_max")->second - env.range.find("z_min")->second);
			cml::vector3 trans(trans_x, 0, trans_z);
			
			double len = cml::length(p1 - (p2 + trans));
			if (len < min_len) {
				min_len = len;
				min_trans = trans;
			}
		}
	}
	return pair<double,cml::vector3>(min_len, min_trans);
}

pair<double,cml::vector3> get_distance(ml::Posture& p1, ml::Posture& p2, const Env &env)
{
	double type_penalty = 0.0;
	double rotate_dist = get_distance_rot(p1.rotate(0), p2.rotate(0));
	pair<double,cml::vector3> trans_dist = diff_trans(p1.trans(), p2.trans(), env);
	double total_distance = trans_dist.first + 4.0 * rotate_dist + 0.07 * fabs(p1.time - p2.time) + type_penalty;

	return pair<double, cml::vector3>(total_distance, trans_dist.second);
}

void set_inner_cons( const vector<shared_ptr<Patch>> & patches, const map<int,int> &patch2motion_offset, vector<Constraint> &cons )
{
	for (auto it_ = patch2motion_offset.begin(); it_ != patch2motion_offset.end(); ++it_)
	{
		int patch_i = it_->first;
		int motion_offset = it_->second;

		vector<Constraint> inner_cons_trans = patches[patch_i]->inner_cons;
		for (auto it = inner_cons_trans.begin(); it != inner_cons_trans.end(); ++it) {
			if (it->type == "rel_pos" || it->type == "same_time") {
				it->m_int[0] += motion_offset;
				it->m_int[2] += motion_offset;
			}
			if (it->type == "pos" || it->type == "time" || it->type == "dir") {
				it->m_int[0] += motion_offset;
			}
		}
		copy(inner_cons_trans.begin(), inner_cons_trans.end(), back_inserter(cons));
	}
}

void Pinning( vector<Constraint> &cons, vector<shared_ptr<Patch>> & patches, const set<int> &edit_patches, map<int,int> patch2motion_offset )
{
  int pin_num = 0;
  for (auto it = edit_patches.begin(); it != edit_patches.end(); ++it) {
	for (auto it_b = patches[*it]->boundaries.begin(); it_b != patches[*it]->boundaries.end(); ++it_b) {
	  if (it_b->is_connected() == true) {
		add_cons_pin(cons, patch2motion_offset[*it]+it_b->motion_index, *(it_b->p_motion), it_b->forward);
		++pin_num;
	  }
	}
  }
  if (pin_num == 0) {
	int oldest_patch = *(edit_patches.begin());
	int group = patch2motion_offset[oldest_patch] + 0;
	const ml::Motion &mot = patches[oldest_patch]->motions[0];
	add_cons_pin(cons, group, mot, true);
  }
}

bool edit_ok_col_ok( const vector<shared_ptr<Patch>> &patches, const set<int> &edit_patches, const map<string, Patch> *patch_type, const Env &env, bool ignore_collision /*= false*/ )
{
	for (auto it = edit_patches.begin(); it != edit_patches.end(); ++it) {
	  const Patch &pa = *patches[*it];
	  if (edit_ok(pa, patch_type) == false)
		return false;
	}
    if (ignore_collision == false)
    {
      for (auto it = edit_patches.begin(); it != edit_patches.end(); ++it) {
        set<int> solo_patch;
        solo_patch.insert(*it);
        if ((collision_patches_env_obb(patches, solo_patch, env) > -1000) == true) return false;
      }
    }
	return true;
}

void patches_edit( vector<shared_ptr<Patch>> &patches, int center_patch_i, const vector<ConnectInfo> &connect_infos, const set<int> &edit_patches, const map<string, Patch> * patch_type, const Env &env, bool is_jitter /*= false*/, const ml::Posture& jitter_posture /*= NULL*/ )
{
  vector<ml::Motion*> edit_motions;
  map<int,int> patch2motion_offset;
  int offset = 0;
  for (auto it = edit_patches.begin(); it != edit_patches.end(); ++it) {
	patch2motion_offset[*it] = offset;
	for (auto it_m = patches[*it]->motions.begin(); it_m != patches[*it]->motions.end(); ++it_m) {
	  edit_motions.push_back(&(*it_m));
	  offset++;
	}
  }

  //cyclic을 위해 translate하자.
  map<int, cml::vector3> patch_trans;
  for (auto it = connect_infos.begin(); it != connect_infos.end(); ++it) {
	const ConnectInfo &ci = *it;
	patch_trans[ci.pa2] = ci.trans_pos;
  }
  for (auto it = patch_trans.begin(); it != patch_trans.end(); ++it) {
	patches[it->first]->translate(it->second);
  }
			
  vector<Constraint> cons;
  set_inner_cons(patches, patch2motion_offset, cons);
  for (auto it = connect_infos.begin(); it != connect_infos.end(); ++it) {
	const ConnectInfo &ci = *it;
	const Boundary &boundary1 = patches[center_patch_i]->boundaries[ci.b1];
	const Boundary &boundary2 = patches[ci.pa2]->boundaries[ci.b2];
	int group1 = patch2motion_offset[center_patch_i] + boundary1.motion_index;
	int group2 = patch2motion_offset[ci.pa2] + boundary2.motion_index;
	int index1, index2;
	double dir_ratio;
	if ( boundary1.forward == true) {
	  index1 = boundary1.p_motion->size()-1;
	  index2 = 0;
	} else {
	  index1 = 0;
	  index2 = boundary2.p_motion->size()-1;
	}
	add_cons_same_pos_dir_time(cons, group1, index1, group2, index2);
  }

  //jitter
  if (is_jitter == true) {
	  Patch & new_patch = *(patches[center_patch_i]);
	  ml::Motion &m = new_patch.motions[0];
	  add_cons_pos(cons, patch2motion_offset[center_patch_i], m.size()/2, jitter_posture.trans());
  }

  //boundary
  Pinning(cons, patches, edit_patches, patch2motion_offset);
	
  //여기서 원래 모션으로 바꾼 다음 에디팅 하면 에러누적이 없을 듯 하다.
  for (auto it = edit_patches.begin(); it != edit_patches.end(); ++it) {
	string pa_name = patches[*it]->name;
	const Patch &pa_type = (patch_type->find(pa_name))->second;
	Patch &pa_cur = *patches[*it];
	for (size_t m_i = 0; m_i < pa_cur.motions.size(); ++m_i) {
	  //에러 체크
	  if (pa_type.motions[m_i].size() != pa_cur.motions[m_i].size()) {
		cout << "Error !!! " << pa_type.name << ' ' << pa_cur.name << endl;
		exit(0);
	  }
	  //원래 모션으로 바꿈
	  for (size_t p_i = 0; p_i < pa_cur.motions[m_i].size(); ++p_i) {
		ml::Posture &posture_cur = pa_cur.motions[m_i][p_i];
		const ml::Posture &posture_type = pa_type.motions[m_i][p_i];
		posture_cur.trans(posture_type.trans());
		posture_cur.rotate(0, posture_type.rotate(0));
	  }
	}
  }
	
  //edit
  multi_motion_edit(edit_motions, cons, 0.028);

  //warp: 지금 연결된 것들
  for (auto it = connect_infos.begin(); it != connect_infos.end(); ++it) {
	const ConnectInfo &ci = *it;
	const Boundary &boundary1 = patches[center_patch_i]->boundaries[ci.b1];
	const Boundary &boundary2 = patches[ci.pa2]->boundaries[ci.b2];
	if (boundary1.forward) {
	  ml::warp(boundary1.p_motion, boundary2.p_motion);
	} else {
	  ml::warp(boundary2.p_motion, boundary1.p_motion);
	}
  }

  //cyclic때문에 한 translate 원상복귀
  for (auto it = patch_trans.begin(); it != patch_trans.end(); ++it) {
	patches[it->first]->translate(-it->second);
  }
				
  //warp: 이전에 연결되어 있던 것들. 원래 모션으로 바꾼 다음 에디팅 하므로 조금 틀어지는 경우가 생겨서 필요하다.
  for (auto it = edit_patches.begin(); it != edit_patches.end(); ++it) {
	warp_boundary_half_direction_height(*(patches[*it]));
  }

  //cyclic 때문에 범위 밖에 있는 애들은 다시 안으로 넣어주자. 
  {
	auto it = patches[center_patch_i];
	vector<cml::vector3> trans3_v;
	for (auto b = it->boundaries.begin(); b != it->boundaries.end(); ++b) {
	  trans3_v.push_back(b->posture().trans());
	}

	cml::vector3 mean_trans3 = accumulate(trans3_v.begin(), trans3_v.end(), cml::vector3(0,0,0)) / trans3_v.size();
	if( env.is_cyclic.find("x")->second) {
	  if (mean_trans3[0] < env.range.find("x_min")->second) {
		it->translate(cml::vector3((env.range.find("x_max")->second - env.range.find("x_min")->second),0,0));
	  } else if (mean_trans3[0] > env.range.find("x_max")->second) {
		it->translate(cml::vector3(-(env.range.find("x_max")->second - env.range.find("x_min")->second),0,0));
	  }
	}
	if( env.is_cyclic.find("z")->second) {
	  if (mean_trans3[2] < env.range.find("z_min")->second) {
		it->translate(cml::vector3(0, 0, (env.range.find("z_max")->second - env.range.find("z_min")->second)));
	  } else if (mean_trans3[2] > env.range.find("z_max")->second) {
		it->translate(cml::vector3(0, 0, -(env.range.find("z_max")->second - env.range.find("z_min")->second)));
	  }
	}
  }
}

void warp_boundary(Patch &pa)
{
	for (auto it = pa.boundaries.begin(); it != pa.boundaries.end(); ++it)
	{
		if (it->is_connected())
		{
			if (it->forward == true)
			{
				ml::warp(it->p_motion, it->connected_motion());
			}
			else
			{
				ml::warp(it->connected_motion(), it->p_motion);
			}
		}
	}
}

void warp_boundary_patches( vector<shared_ptr<Patch>> &patches ) 
{
	for (int i = 0 ; i < patches.size(); ++i)
	{
		warp_boundary(*patches[i]);
	}
}

void warp_boundary_half_direction_height(Patch &pa)
{
	for (auto it = pa.boundaries.begin(); it != pa.boundaries.end(); ++it)
	{
		if (it->is_connected())
		{
			if (it->forward == true)
			{
				ml::warp_root_direction_height(it->p_motion, it->connected_motion(), 20, 0);
			}
			else
			{
				ml::warp_root_direction_height(it->connected_motion(), it->p_motion, 0, 20);
			}
		}
	}
}

void backupPatches( std::map<int,Patch> &backup_patches, const vector<shared_ptr<Patch> > &patches, const std::set<int> &edit_patches)
{
	int new_pa_i = patches.size();
	for(auto it=edit_patches.begin(); it!=edit_patches.end(); ++it) {
	  if (*it!=new_pa_i)
	    backup_patches[*it] = *patches[*it];
	}
}

void fetchPatches( vector<shared_ptr<Patch>> &patches, const map<int, Patch> &backup_patches )
{
	for (auto it = backup_patches.begin(); it != backup_patches.end(); ++it) {
	  const Patch &pa_from = it->second;
	  Patch &pa_to = *patches[it->first];

	  for (size_t m_i=0; m_i<pa_from.motions.size(); ++m_i) {
		for (size_t p_i=0; p_i<pa_from.motions[m_i].size(); ++p_i) {
		  const ml::Posture &p_from = pa_from.motions[m_i][p_i];
		  ml::Posture &p_to = pa_to.motions[m_i][p_i];
		  p_to.m_rotates[0] = p_from.m_rotates[0];
		  p_to.m_trans = p_from.m_trans;
		  p_to.time = p_from.time;
	    }
	  }
	}
}

set<int> get_edit_patches( const vector<ConnectInfo> & connect_infos, const int center_patch_i )
{
	set<int> edit_patches;
	edit_patches.insert(center_patch_i);
	for (auto it = connect_infos.begin(); it != connect_infos.end(); ++it)
	  edit_patches.insert(it->pa2);

	return edit_patches;
}

bool is_stitchable(vector<shared_ptr<Patch> > &patches, const shared_ptr<Patch> &pa, const map<string,Patch> *patch_type, const vector<ConnectInfo> &connect_infos, const Env &env /*= Env()*/)
{
	patches.push_back(pa);
	int new_pa_idx = patches.size()-1;
	const set<int> edit_patches = get_edit_patches(connect_infos, new_pa_idx);

	// Back up original patches
	map<int, Patch> backup_patches;
	backupPatches(backup_patches, patches, edit_patches);

	patches_edit(patches, new_pa_idx, connect_infos, edit_patches, patch_type, env);
	bool is_ok = edit_ok_col_ok(patches, edit_patches, patch_type, env);		// verify the editable-ness and the collision of edited-patches.
	
	fetchPatches(patches, backup_patches);
	patches.pop_back();	

	if (!is_ok) {		
		return false;
	} else {
		return true;
	}
}

void do_stitch(vector<shared_ptr<Patch> > &patches, const shared_ptr<Patch> &pa, const map<string,Patch> *patch_type, const vector<ConnectInfo> & connect_infos, const Env &env, const bool is_compensation /*= true*/)
{
	patches.push_back(pa);
	int new_pa_idx = patches.size()-1;
	const set<int> edit_patches = get_edit_patches(connect_infos, new_pa_idx);

	patches_edit(patches, new_pa_idx, connect_infos, edit_patches, patch_type, env);

	// update connection-infos
	for (auto it = connect_infos.begin(); it != connect_infos.end(); ++it) {
		Boundary &b1 = patches[new_pa_idx]->boundaries[it->b1];		
		Boundary &b2 = patches[it->pa2]->boundaries[it->b2];

		if (b1.forward) {
		  b1.p_motion->connected_fore = b2.p_motion;
		} else {
		  b1.p_motion->connected_back = b2.p_motion;
		}	  
		if (b2.forward)	{
		  b2.p_motion->connected_fore = b1.p_motion;
		} else {
		  b2.p_motion->connected_back = b1.p_motion;
		}
	}
	if (is_compensation) {
	  bool comp = compensation(patches, new_pa_idx, patch_type, 4.5, env);
	    if (comp == true)
	      cout <<  "   compensation!" << endl;
	}
}

bool stitch( vector<shared_ptr<Patch> > &patches, const shared_ptr<Patch> &pa, const map<string,Patch> *patch_type, const vector<ConnectInfo> &connect_infos, const Env &env /*= Env()*/, bool is_compensation /*= false*/, bool jittering /*= false*/, int jitter_start_i/*=1*/, int jitter_end_i/*=24*/, bool ignore_collision /*= false*/ )
{
	patches.push_back(pa);
	int new_pa_idx = patches.size()-1;
	const set<int> edit_patches = get_edit_patches(connect_infos, new_pa_idx);
	
	// Back up original patches
	map<int, Patch> backup_patches;
	backupPatches(backup_patches, patches, edit_patches);
	
	patches_edit(patches, new_pa_idx, connect_infos, edit_patches, patch_type, env);
	bool is_ok = false;
	if (jittering == false) {
	  is_ok = edit_ok_col_ok(patches, edit_patches, patch_type, env, ignore_collision);		// verify the editable-ness and the collision of edited-patches.
	} else {
	  Patch &new_patch = **(patches.end()-1);
	  ml::Motion &m = new_patch.motions[0];
	  const ml::Posture &p = m[m.size()/2];
	  
	  vector<cml::vector2d> diff2s;
	  get_diff2s(diff2s);
	  
	  for (auto it = diff2s.begin()+jitter_start_i; it != diff2s.begin() + jitter_end_i + 1; ++it) { //0,0제외하기 위해 begin()+1	  
	  	ml::Posture new_p = p;
	  	cml::vector2d &diff2 = *it;
	  	new_p.ApplyTransf(trans_transf(cml::vector3d(0.3*diff2[0], 0, 0.3*diff2[1])));
	  
	  	patches_edit(patches, new_pa_idx, connect_infos, edit_patches, patch_type, env, true, new_p);
	  	is_ok = edit_ok_col_ok(patches, edit_patches, patch_type, env);
	  	if (is_ok) {
	  		cout << "jitter diff: " << *it << endl;
	  		break;
	  	}
	  }
	}
			
	if (!is_ok) {
	  fetchPatches(patches, backup_patches);
	  patches.pop_back();	  
	  return false;
	} else {
	  // update connection-infos
	  for (auto it = connect_infos.begin(); it != connect_infos.end(); ++it) {
	    Boundary &b1 = patches[new_pa_idx]->boundaries[it->b1];		
	    Boundary &b2 = patches[it->pa2]->boundaries[it->b2];
	  
        if (b1.forward) {
	  	  b1.p_motion->connected_fore = b2.p_motion;
		} else {
	  		b1.p_motion->connected_back = b2.p_motion;
		}	  
	  	if (b2.forward)	{
	  		b2.p_motion->connected_fore = b1.p_motion;
		} else {
	  		b2.p_motion->connected_back = b1.p_motion;
		}
       
	  }
	  if (is_compensation) {
	  	/*bool comp = compensation(patches, new_pa_idx, patch_type, 4.5, env);
	  	if (comp == true)
	  	  cout <<  "   compensation!" << endl;*/
	  }
	  return true;
	}	
}

string read_word(string file_name)
{
	ifstream file(file_name, ifstream::in);
	string word;
	file >> word;
	file.close();

	return word;
}

bool is_stop()
{
	bool stop = read_word("control.txt") == "stop";
	if (stop) {
	  ofstream file ("control.txt");
	  file << "-stop";
	  file.close();
	}
	return stop;
}

void get_connect_infos( vector<ConnectInfo> &connect_infos, const shared_ptr<Patch> &pa_new, const vector<shared_ptr<Patch>> &patches, double thres, const Env &env )
{
	for (size_t b1_i=0; b1_i<pa_new->boundaries.size(); ++b1_i) {
	  for (size_t pa2_i=0; pa2_i<patches.size(); ++pa2_i) {
	    const shared_ptr<Patch> &pa_result = patches[pa2_i];

	    for (int b2_i = 0; b2_i < pa_result->boundaries.size(); ++b2_i) {
	      Boundary &b1 = pa_new->boundaries[b1_i];
	      Boundary &b2 = pa_result->boundaries[b2_i];
	      
		  if (b1.forward!=b2.forward && !b2.is_connected() && b1.posture_type()==b2.posture_type()) {
	      	pair<double,cml::vector3> distance_trans = get_distance(b1.posture(), b2.posture(), env);
	      	if (distance_trans.first < thres) {
	      	  ConnectInfo con_info(b1_i, pa2_i, b2_i, distance_trans.first, distance_trans.second);
	      	  
	      	  //같은 곳에 연결되면 안된다.
	      	  auto it_con = find_if(connect_infos.begin(), connect_infos.end(), 
	      	  					  [&con_info](ConnectInfo &c) { return con_info.b1 != c.b1 && con_info.pa2 == c.pa2 && con_info.b2 == c.b2;});
	      	  if (it_con != connect_infos.end()) continue;
	      	  
	      	  //한 패치와는 다른 translate로 연결되면 안된다. 
	      	  auto it_con_ = find_if(connect_infos.begin(), connect_infos.end(), 
	      	  					   [&con_info](ConnectInfo &c) { return con_info.b1 != c.b1 && con_info.pa2 == c.pa2 && length(con_info.trans_pos - c.trans_pos) > 0.1;});
	      	  if (it_con_ != connect_infos.end()) continue;
	      	  
	      	  auto it_bi = find_if(connect_infos.begin(), connect_infos.end(),
	      	  					 [&con_info](ConnectInfo &c) { return con_info.b1 == c.b1;});
	      	  if (it_bi == connect_infos.end()) {
	      	  	connect_infos.push_back(con_info);
	      	  } else if (con_info.distance<it_bi->distance) {
	      	  	connect_infos.erase(it_bi);
	      	  	connect_infos.push_back(con_info);
	      	  }
	      	  //cout << b2.connected() << endl;
	      	  //cout << ci.b1 << ' ' << ci.pa2 << ' ' << ci.b2 << ' ' << ' ' << ci.distance << endl;
	      	  //cout << pa_new->log << endl;
	      	}
	      }
	    }
	  }
	}
}

bool compensation( vector<shared_ptr<Patch>> & patches, int pa_i, const map<string, Patch> * patch_type, double thres /*= 3.9*/, const Env& env /*= Env() */ )
{
  vector<ConnectInfo> connect_infos;

  Patch &pa1 = *(patches[pa_i]);
  for (int b1_i = 0; b1_i < pa1.boundaries.size(); ++b1_i) {
	Boundary &b1 = pa1.boundaries[b1_i];
	if (b1.is_connected()) continue;

	for (int pa2_i = 0; pa2_i < patches.size(); ++pa2_i) {
	  if (pa_i == pa2_i) continue;
	  Patch &pa2 = *(patches[pa2_i]);
	  for (int b2_i = 0; b2_i < pa2.boundaries.size(); ++b2_i) {
		Boundary &b2 = pa2.boundaries[b2_i];
		if (b1.forward != b2.forward && !b2.is_connected() && b1.posture_type() == b2.posture_type()) {
		  pair<double,cml::vector3> distance_trans = get_distance(b1.posture(), b2.posture(), env);
		  if (distance_trans.first < thres) {
			ConnectInfo ci(b1_i, pa2_i, b2_i, distance_trans.first, distance_trans.second);

			//같은 곳에 연결되면 안된다.
			auto it_con = find_if(connect_infos.begin(), connect_infos.end(), 
							  [&ci](ConnectInfo &c){return ci.b1 != c.b1 &&	ci.pa2 == c.pa2 && ci.b2 == c.b2;});
						
			if (it_con != connect_infos.end()) continue;

			//한 패치와는 다른 translate로 연결되면 안된다. 
			auto it_con_ = find_if(connect_infos.begin(), connect_infos.end(), 
							  [&ci](ConnectInfo &c){return ci.b1 != c.b1 && ci.pa2 == c.pa2 && length(ci.trans_pos - c.trans_pos) > 0.1;});
						
			if (it_con_ != connect_infos.end()) continue;

			auto it_bi = find_if(connect_infos.begin(), connect_infos.end(), [&ci](ConnectInfo &c){return ci.b1 == c.b1;});
			if (it_bi == connect_infos.end()) {
			  connect_infos.push_back(ci);
			} else if (ci.distance < it_bi->distance) {
			  connect_infos.erase(it_bi);
			  connect_infos.push_back(ci);
			}
		  }
		}
	  }
	}
  }

	if (connect_infos.empty())
		return false;

	for (int b1_i = 0; b1_i < patches[pa_i]->boundaries.size(); ++b1_i)
	{
		Boundary & b1 = patches[pa_i]->boundaries[b1_i];
		if (b1.is_connected())
		{
			auto connect_pair = connected_patch_boundary(b1, patches);
			ConnectInfo ci(b1_i, connect_pair.first, connect_pair.second);
			connect_infos.push_back(ci);
		}
	}

	const set<int> edit_patches = get_edit_patches(connect_infos, pa_i);
	/*for (auto it = edit_patches.begin(); it != edit_patches.end(); ++it)
		cout << *it << endl;*/
	
	//backup
	map<int, Patch> backup_patches;
	for(auto it = edit_patches.begin(); it != edit_patches.end(); ++it)
	{
		backup_patches[*it] = *patches[*it];
	}

	bool ok_col_edit;
	patches_edit(patches, pa_i, connect_infos, edit_patches, patch_type, env); 

	ok_col_edit = edit_ok_col_ok(patches, edit_patches, patch_type, env);
	
	if (ok_col_edit == false) {
		//restore from backup
		for (auto it = backup_patches.begin(); it != backup_patches.end(); ++it)
		{
			const Patch &pa_from = it->second;
			Patch &pa_to = *patches[it->first];

			for (int m_i = 0; m_i < pa_from.motions.size(); ++m_i)
			{
				for(int p_i = 0; p_i < pa_from.motions[m_i].size(); ++p_i)
				{
					const ml::Posture &p_from = pa_from.motions[m_i][p_i];
					ml::Posture &p_to = pa_to.motions[m_i][p_i];
					p_to.m_rotates[0] = p_from.m_rotates[0];
					p_to.m_trans = p_from.m_trans;
					p_to.time = p_from.time;
				}
			}
		}
		
		return false;
	}

	for (auto it = connect_infos.begin(); it != connect_infos.end(); ++it)
	{
		Boundary &b1 = patches[pa_i]->boundaries[it->b1];
		Boundary &b2 = patches[it->pa2]->boundaries[it->b2];
		if (b1.forward == true)
			b1.p_motion->connected_fore = b2.p_motion;
		else
			b1.p_motion->connected_back = b2.p_motion;

		if (b2.forward == true)
			b2.p_motion->connected_fore = b1.p_motion;
		else
			b2.p_motion->connected_back = b1.p_motion;
	}

	return true;
}

double get_energy(double connect, double boundary, double dead, double alpha ) 
{
	double total = connect + boundary + dead;
	double energy;
	if (total < 0.01)
		energy = 100000.;
	else
		//energy = dead / (dead + connect);
		//energy = dead / (dead + connect) - 0.05 / 400. * (dead + connect);
		//energy = dead / (dead + connect) - 0.2 / 400. * (dead + connect);
		//energy = dead / (dead + connect) - 0.15 / 400. * (dead + connect);
		energy = dead / (dead + connect) - alpha / 400. * (dead + connect);
	//energy = dead / (dead + connect) - 0.25 / 400. * (dead + connect);
	//energy = - 0.25 / 400. * (dead + connect);
	//energy = energy = dead / total;// - 0.3 / 600. * total;

	/*if (total > 20 * 4)
	energy = dead / (dead + connect);*/
	//cout << "daf " << dead << ' ' << connect << ' ' << dead / (dead + connect) << ' ' << alpha / 400. * (dead + connect) << endl;
	return energy;
}

void print_info(vector<shared_ptr<Patch>> &patches, Env &env, ostream & out2, time_t start_time, int attempt_num, double cur_energy = 0.0);

map<string, int> get_end_num( const vector<shared_ptr<Patch>> &patches, const Env &env )
{
	return get_end_num(patches, env.range.find("t_min")->second, env.range.find("t_max")->second);
}

void metropolis( vector<shared_ptr<Patch>> &patches, const map<string, Patch> * patch_type, double max_time, int last_patch_id, int max_attempt, vector<string> &sampling_patches, Env &env /*= Env()*/, double k /*= 0.0008*/, double alpha /*= 0.0001*/, ostream& out /*= std::cout*/, time_t start_time /*= time(NULL)*/, ostream &out2 /*= ofstream() */ )
{
  time_t current_time;
	
  while (true) {
	int attempt_num = 0;
	int collision_num = 0;

	while(true)	{
	  random_shuffle(sampling_patches.begin(), sampling_patches.end());
	  for (auto it_p = sampling_patches.begin(); it_p != sampling_patches.end(); ++it_p) {
		auto cur_end_num = get_end_num(patches, env.range["t_min"], env.range["t_max"]);
		int cur_connect = cur_end_num["connect"];
		int cur_boundary = cur_end_num["boundary"];
		int cur_dead = cur_end_num["dead"];
				
		double cur_energy = get_energy(cur_connect, cur_boundary, cur_dead, alpha);

		shared_ptr<Patch> pa1(new Patch((patch_type->find(*it_p))->second));
		env.rand_scatter(*pa1);

		vector<ConnectInfo> connect_infos;
		if (pa1->motions.size() == 1) {
		  get_connect_infos(connect_infos, pa1, patches, 4.6, env);
		} else {
		  get_connect_infos(connect_infos, pa1, patches, 4.8, env);
		}

		set<int> connect_set;
		for (auto it = connect_infos.begin(); it != connect_infos.end(); ++it) {
		  connect_set.insert(it->b1);
		}

		int pa1_connect = 0;
		int pa1_boundary = 0;
		int pa1_dead = 0;
		for (int i = 0; i < pa1->boundaries.size(); ++i) {
		  ml::Posture &p = pa1->boundaries[i].posture();
		  if (p.time < env.range["t_min"] || p.time > env.range["t_max"]) {
			++pa1_boundary;
		  } else if (connect_set.count(i) > 0) {
			++pa1_connect;
		  } else {
			++pa1_dead;
		  }
		}
		int new_connect = cur_connect + 2 * pa1_connect;
		int new_boundary = cur_boundary + pa1_boundary;
		int new_dead = cur_dead - pa1_connect + pa1_dead;
		double new_energy = get_energy(new_connect, new_boundary, new_dead, alpha);

		++attempt_num;

		double T = 1. - (1.0 / 200.0) * patches.size();
		if (T < 0.0) T = 0.0;
		double probability = exp(-(new_energy - cur_energy) / (k * T));
		double random_0_1 = random(0, 30000) / 30000.;
		bool is_accept = probability > random_0_1;
				
		if (is_accept) {
		  if (patches.size() > 1 && new_connect == 0)	continue;				
		  if (pa1->motions.size() == 1 && pa1_connect < 2) continue;   // 2개인 패치
			
		  if (attempt_num > max_attempt) goto out_tiling;	// max_attempt
		  if (is_stop()) goto out_tiling;

		  bool is_stitch = stitch(patches, pa1, patch_type, connect_infos, env);
		  if (is_stitch) {
			add_phase1_info(pa1->inner_cons);	  //1st phase라는 걸 표시하자.

			time(&current_time);
			out << "time " << difftime(current_time, start_time) << endl;
			out << "patches ID: " << patches.size()-1 << ' ' << *it_p << ' ' << "Connect ##### " << connect_infos.size() << endl;
			out << "Attempt Num: " << attempt_num << " Collision Num: " << collision_num << endl;
			out << "old(exact)  " << cur_energy << " t:" << cur_connect+cur_boundary+cur_dead << " c:" << cur_connect << " b:" << cur_boundary << " d:" << cur_dead <<  endl;
			out << "new(pseudo) " << new_energy << " t:" << new_connect+new_boundary+new_dead << " c:" << new_connect << " b:" << new_boundary << " d:" << new_dead <<  endl << endl;

			//print2
			print_info(patches, env, out2, start_time, attempt_num, cur_energy);			
			goto out_pa1;
		  } else {
			++collision_num;
		  }
		}
	  }
	}
out_pa1:;
	if (difftime(current_time, start_time) > max_time || patches.size()-1 >= last_patch_id) goto out_tiling;
  }
out_tiling:;
	
  //phase1이 끝났음을 알리자.
  out2 << -1 << endl; 
}

void remove_dangling( vector<shared_ptr<Patch>> &patches, const vector<Patch> &patch_type_unary, const map<string, Patch> * patch_type, Env &env /*= Env()*/, ostream& out /*= std::cout*/, time_t start_time /*= time(NULL)*/, ostream &out2 /*= ofstream() */ )
{
  while (true) {
	bool is_remove = remove_dangling_oneturn(patches, patch_type_unary, patch_type, env, out, start_time, out2);
	out << "########## one turn ###########" << endl;

	if (is_remove == false) return ;
	if (is_stop()) return;
  }
}

set<int> dangling_stitch( vector<shared_ptr<Patch>> &patches, const vector<Patch> &patch_type_unary, const map<string, Patch> * patch_type, const vector<pair<int,int>> &traverse_vec, Env& env, ostream& out, time_t start_time, ostream &out2 )
{
  vector<pair<int,int>> traverse_vec_cannot;

  //dead end가 있으면 순회하면서 처리하기
  for (auto it = traverse_vec.begin(); it != traverse_vec.end(); ++it) {
	const auto it_b2 = &(patches[it->first]->boundaries[it->second]);
	const auto &p2 = it_b2->posture();

	for (int connect_num = 2; connect_num >= 1; --connect_num) {
	  auto type_random = get_random_vec(patch_type_unary.size());
	  for (int type_i = 0; type_i < type_random.size(); ++type_i)	{
		//patch를 만들기도 전에 넘겨버리자.
		if (connect_num == 1 && it_b2->stitched_unary.count(patch_type_unary[type_random[type_i]].name) > 0) continue;

		shared_ptr<Patch> pa1(new Patch(patch_type_unary[type_random[type_i]]));

		for (auto it_b1 = pa1->boundaries.begin(); it_b1 != pa1->boundaries.end(); ++it_b1)	{
		  if (it_b1->forward == it_b2->forward || it_b1->posture_type() != it_b2->posture_type())	continue;
					
		  const auto &p1 = it_b1->posture();
		  pa1->transform_between_posture(p2, p1);

		  vector<ConnectInfo> connect_infos;
		  if (connect_num == 2) {
			get_connect_infos(connect_infos, pa1, patches, 4.8, env);
		  } else if (connect_num == 1) {
			get_connect_infos(connect_infos, pa1, patches, 1.2, env);
		  }
					
		  if (connect_infos.size() >= connect_num) {
			if (stitch( patches, pa1, patch_type, connect_infos, env, false) == true) {
			  auto cur_end_num = get_end_num(patches, env.range["t_min"], env.range["t_max"]);
			  out << "dead num: " << cur_end_num["dead"] << endl;
			  out << "patches ID: " << patches.size()-1 << " where: " << it->first << ' ' << it->second << " Connect ##### " << connect_infos.size() << endl;
			  out << endl;
							
			  //연결했다는 기록 남기자.
			  if (connect_num == 1) // && patches[it->first]->motions.size() == 1)
				  it_b2->stitched_unary.insert(pa1->name);
			  //print_info
			  print_info(patches, env, out2, start_time, 0);
			  goto stitch_dead;
			}
		  }
		}
	  }
	}
	traverse_vec_cannot.push_back(pair<int,int>(it->first, it->second));
	//cout << "!!!! Can not " <<  it->first << ' ' << it->second << endl << endl;
	stitch_dead:;
  }

  // 진짜 못 해결한 순회 목록 만들기
  vector<pair<int,int>> traverse_vec_cannot_real;
  for (auto it = traverse_vec_cannot.begin(); it != traverse_vec_cannot.end(); ++it) {
	string end_type = get_end_type(patches[it->first]->boundaries[it->second], env.range["t_min"], env.range["t_max"]);
	if (end_type == "dead") {
	  out << "can not dead real: " << it->first << ' ' << it->second << endl;
	  traverse_vec_cannot_real.push_back(*it);
	}
  }

  //진짜로 못 해결한 것 없앨 것 없애고, 지터링 할 건 지터링 하자.
  set<int> delete_list;
  int dead_end_num = get_end_num(patches, env.range["t_min"], env.range["t_max"])["dead"];
  for (auto it = traverse_vec_cannot_real.begin(); it != traverse_vec_cannot_real.end(); ++it) {
	//지워야 할 패치에는 jittering할 필요없다.
	if (delete_list.count(it->first) > 0) continue;
	out << "can not dead real " << it->first << ' ' << it->second << endl;

	int finish_num = 0;
	////2인 아이들은 무조건 jittering 해보자.dead가 5 이하인 ?嚥珥?2개 연결 잘 안되므로 패스
	if (dead_end_num > finish_num - 3) {
	  if (jitter(patch_type_unary, patches, patch_type, env, out, *it, 2) == true) {
		print_info(patches, env, out2, start_time, 0);
		continue;
	  }
	}

	//1인 아이들은 jittering 필요없다. finish_num 범위인 경우 jittering한다.
	if (patches[it->first]->motions.size() == 1) {
	  if (dead_end_num <= finish_num) {
		if (patches[it->first]->boundaries[it->second].jittering_stitched_unary.size() >= patch_type_unary.size()) {
		  delete_list.insert(it->first);
		  out << "wow ^^ 1 motion jittering_stitch delete " << it->first << endl;
		  continue;
		} else {
		  if (jitter(patch_type_unary, patches, patch_type, env, out, *it, 1)) { 
			auto b = patches[it->first]->boundaries[it->second];
			string connected_name = patches[connected_patch_motion(b.connected_motion(), patches).first]->name;
			b.jittering_stitched_unary.insert(connected_name);

			//print_info
			print_info(patches, env, out2, start_time, 0);
			continue;
		  } else {
			delete_list.insert(it->first);
			out << "====== 1 motion size ====== delete_list insert : " << it->first << endl << endl;
			continue;
		  }
		}
	  }
	  delete_list.insert(it->first);
	  out << "==== 1 motion size ==== delete_list insert : " << it->first << endl << endl;
	  //그 윗단이 정말 문제라면 그 다음턴에 연결할 아이가 없으므로 그 윗단이 없어진다.
	} else {
	  if (patches[it->first]->boundaries[it->second].jittering_stitched_unary.size() >= patch_type_unary.size()) {
		delete_list.insert(it->first);
		out << "wow ^^ 2 motion jittering_stitch delete" << it->first << endl;
	  } else {
		if (jitter(patch_type_unary, patches, patch_type, env, out, *it, 1) == true) {		  

		  auto b = patches[it->first]->boundaries[it->second];
		  string connected_name = patches[connected_patch_motion(b.connected_motion(), patches).first]->name;
		  b.jittering_stitched_unary.insert(connected_name);
		  //나중에 잘 안되면 여기서 제거하자. 하지만 jitter가 랜덤하게 되므로 그런 일은 거의 없을 것 같다.

		  //print_info
		  print_info(patches, env, out2, start_time, 0);
		  continue;
		} else {
		  delete_list.insert(it->first);
		  out << "====== 2 motion size ====== delete_list insert : " << it->first << endl << endl;
		  continue;
		}
	  }
	}
  }
  return delete_list;
}

bool jitter( const vector<Patch> &patch_type_unary, vector<shared_ptr<Patch>> &patches, const map<string, Patch> * patch_type, Env& env, ostream& out, const pair<int, int> &it, int connect_num )
{
	const auto it_b2 = &(patches[it.first]->boundaries[it.second]);
	const auto &p2 = it_b2->posture();

	auto type_random = get_random_vec(patch_type_unary.size());
	for (int type_i = 0; type_i < type_random.size(); ++type_i)
	{
		//patch를 만들기도 전에 넘겨버리자.
		if (it_b2->jittering_stitched_unary.count(patch_type_unary[type_random[type_i]].name) > 0)
			continue;

		shared_ptr<Patch> pa1(new Patch(patch_type_unary[type_random[type_i]]));

		for (auto it_b1 = pa1->boundaries.begin(); it_b1 != pa1->boundaries.end(); ++it_b1)
		{
			if (it_b1->forward == it_b2->forward || it_b1->posture_type() != it_b2->posture_type())
				continue;

			const auto &p1 = it_b1->posture();
			pa1->transform_between_posture(p2, p1);

			vector<ConnectInfo> connect_infos;

			if (connect_num == 2)
				get_connect_infos(connect_infos, pa1, patches, 4.8, env);
			else if (connect_num == 1)
				get_connect_infos(connect_infos, pa1, patches, 1.2, env);

			if (connect_infos.size() >= connect_num)
			{
				if (stitch(patches, pa1, patch_type, connect_infos, env, false, true, 1, 24) == true) {
					auto cur_end_num = get_end_num(patches, env.range["t_min"], env.range["t_max"]);
					out << "Resolve with jittering!!!!!!! connect_num: " << connect_num << endl;
					out << "dead num: " << cur_end_num["dead"] << endl;
					out << "patches ID: " << patches.size()-1 << " where: " << it.first << ' ' << it.second << " Connect ##### " << connect_infos.size() << endl;
					out << endl;
					return true;
				}
			}
		}
	}

	//for (int type_i = 0; type_i < type_random.size(); ++type_i)
	//{
	//	//patch를 만들기도 전에 넘겨버리자.
	//	if (it_b2->jittering_stitched_unary.count(patch_type_unary[type_random[type_i]].name) > 0)
	//		continue;

	//	shared_ptr<Patch> pa1(new Patch(patch_type_unary[type_random[type_i]]));

	//	for (auto it_b1 = pa1->boundaries.begin(); it_b1 != pa1->boundaries.end(); ++it_b1)
	//	{
	//		if (it_b1->forward == it_b2->forward || it_b1->posture_type() != it_b2->posture_type())
	//			continue;

	//		const auto &p1 = it_b1->posture();
	//		pa1->transform_between_posture(p2, p1);

	//		vector<ConnectInfo> connect_infos;

	//		if (connect_num == 2)
	//			get_connect_infos(connect_infos, pa1, patches, 4.8, env);
	//		else if (connect_num == 1)
	//			get_connect_infos(connect_infos, pa1, patches, 1.2, env);

	//		if (connect_infos.size() >= connect_num)
	//		{
	////			cout << "curious " << 24 << ' ' << 48 << endl;
	//			if (stitch(patches, pa1, patch_type, connect_infos, env, false, true, 24, 48) == true) {
	//				auto cur_end_num = get_end_num(patches, env.range["t_min"], env.range["t_max"]);
	//				out << "Resolve with jittering!!!!!!! connect_num: " << connect_num << endl;
	//				out << "dead num: " << cur_end_num["dead"] << endl;
	//				out << "patches ID: " << patches.size()-1 << " where: " << it.first << ' ' << it.second << " Connect ##### " << connect_infos.size() << endl;
	//				out << endl;
	//				return true;
	//			}
	//		}
	//	}
	//}

	//for (int type_i = 0; type_i < type_random.size(); ++type_i)
	//{
	//	//patch를 만들기도 전에 넘겨버리자.
	//	if (it_b2->jittering_stitched_unary.count(patch_type_unary[type_random[type_i]].name) > 0)
	//		continue;

	//	shared_ptr<Patch> pa1(new Patch(patch_type_unary[type_random[type_i]]));

	//	for (auto it_b1 = pa1->boundaries.begin(); it_b1 != pa1->boundaries.end(); ++it_b1)
	//	{
	//		if (it_b1->forward == it_b2->forward || it_b1->posture_type() != it_b2->posture_type())
	//			continue;

	//		const auto &p1 = it_b1->posture();
	//		pa1->transform_between_posture(p2, p1);

	//		vector<ConnectInfo> connect_infos;

	//		if (connect_num == 2)
	//			get_connect_infos(connect_infos, pa1, patches, 4.8, env);
	//		else if (connect_num == 1)
	//			get_connect_infos(connect_infos, pa1, patches, 1.2, env);

	//		if (connect_infos.size() >= connect_num)
	//		{
	//			//			cout << "curious " << 24 << ' ' << 48 << endl;
	//			if (stitch(patches, pa1, patch_type, connect_infos, env, false, true, 49, 72) == true) {
	//				auto cur_end_num = get_end_num(patches, env.range["t_min"], env.range["t_max"]);
	//				out << "Resolve with jittering!!!!!!! connect_num: " << connect_num << endl;
	//				out << "dead num: " << cur_end_num["dead"] << endl;
	//				out << "patches ID: " << patches.size()-1 << " where: " << it.first << ' ' << it.second << " Connect ##### " << connect_infos.size() << endl;
	//				out << endl;
	//				return true;
	//			}
	//		}
	//	}
	//}

	//	cout << "no jitter " << endl;
	return false;
}

vector<pair<int,int>> get_traverser_vec( const vector<shared_ptr<Patch>> &patches, Env &env )
{
  vector<pair<int,int>> traverse_vec;
  for (int pa2_i = 0; pa2_i < patches.size(); ++pa2_i) {
	for (int b2_i = 0; b2_i < patches[pa2_i]->boundaries.size(); ++b2_i) {
	  string end_type = get_end_type(patches[pa2_i]->boundaries[b2_i], env.range["t_min"], env.range["t_max"]);
	  if (end_type == "dead") {
		traverse_vec.push_back(pair<int,int>(pa2_i, b2_i));
		//cout << "dead " << pa2_i << ' ' << b2_i << endl;
	  }
	}
  }
  return traverse_vec;
}

void dangling_delete( vector<shared_ptr<Patch>> &patches, const set<int> &delete_list, ostream& out )
{
  out << "---delete start " << endl;
  for (auto it = delete_list.rbegin(); it != delete_list.rend(); ++it) {
	out << *it << ' ';
	vector<ml::Motion> &motions_del = patches[*it]->motions;
	for (auto it_p = patches.begin(); it_p != patches.end(); ++it_p) {
	  for (auto it_b = (*it_p)->boundaries.begin(); it_b != (*it_p)->boundaries.end(); ++it_b) {
		for (int i = 0; i < motions_del.size(); ++i) {
		  if (it_b->p_motion->connected_fore == &(motions_del[i])) {
			it_b->p_motion->connected_fore = nullptr;
		  }
		  if (it_b->p_motion->connected_back == &(motions_del[i])) {
			it_b->p_motion->connected_back = nullptr;
		  }
		}
	  }
	}
	patches.erase(patches.begin() + *it);
  }
  out << endl << "---delete end" << endl;
}

void delete_patch( vector<shared_ptr<Patch>> &patches, int patch_i, ostream& out )
{
	out << "delete patch_i: " << patch_i << endl;
	
	vector<ml::Motion> &motions_del = patches[patch_i]->motions;
	for (auto it_p = patches.begin(); it_p != patches.end(); ++it_p)
	{
		for (auto it_b = (*it_p)->boundaries.begin(); it_b != (*it_p)->boundaries.end(); ++it_b)
		{
			for (int i = 0; i < motions_del.size(); ++i)
			{
				if (it_b->p_motion->connected_fore == &(motions_del[i])) {
					it_b->p_motion->connected_fore = nullptr;
				}
				if (it_b->p_motion->connected_back == &(motions_del[i])) {
					it_b->p_motion->connected_back = nullptr;
				}
			}
		}
	}
	patches.erase(patches.begin() + patch_i);
}

bool remove_dangling_oneturn( vector<shared_ptr<Patch>> & patches, const vector<Patch> & patch_type_unary, const map<string, Patch> * patch_type, Env & env, ostream& out, time_t start_time, ostream &out2 )
{
  //순회 목록 만들기
  vector<pair<int,int>> traverse_vec = get_traverser_vec(patches, env);
	
  //dead end가 없으면 멈춘다.
  if (traverse_vec.empty()) return false;
	
  set<int> delete_list = dangling_stitch(patches, patch_type_unary, patch_type, traverse_vec, env, out, start_time, out2);
	
  dangling_delete(patches, delete_list, out);

  return true;
}

void print_info( vector<shared_ptr<Patch>> &patches, Env &env, ostream & out2, time_t start_time, int attempt_num, double cur_energy /*= 0.0*/ )
{
  time_t current_time;
  time(&current_time);
  auto cur_end_num = get_end_num(patches, env.range["t_min"], env.range["t_max"]);
  int cur_connect = cur_end_num["connect"];
  int cur_boundary = cur_end_num["boundary"];
  int cur_dead = cur_end_num["dead"];
  out2 << difftime(current_time, start_time) << ' ' << cur_dead << ' ' << cur_connect << ' ' << patches.size() << ' ' << attempt_num << ' ' << cur_energy << endl;
}


void Danglings::erase( size_t pat_idx, size_t b_idx )
{
  auto it = find_if(danglings.begin(), danglings.end(), 
	[pat_idx, b_idx](const Dangling &d){ return (d.patches_idx == pat_idx) && (d.boundary_idx == b_idx);});

  if (it != danglings.end()) {
	danglings.erase(it);
  }
}

void get_naive_dangling_table(Danglings &dangles, const vector<shared_ptr<Patch> > &patches, const Env &env)
{
  dangles.clear();
  for (auto it=patches.begin(); it!=patches.end(); ++it) {
	for (auto jt = (*it)->boundaries.begin(); jt != (*it)->boundaries.end(); ++jt) {			
	  if (is_stitchable_dangling(env, *jt)) {
		Dangling dangling;
		dangling.patches_idx = it - patches.begin();
		dangling.boundary_idx = jt - (*it)->boundaries.begin();
		dangling.pos = jt->posture().trans();
		dangling.is_begin = !jt->forward;
		dangles.push(dangling);
		jt->posture().set_status_in_viewer(true, cml::vector3(1.0, 0.25, 0.25));
	  } else {
		jt->posture().status.is_highlight = false;
	  }
	}
  }
}

bool is_stitchable_dangling(const Env &env_, const Boundary& b )
{
  if (b.is_connected() || 
    (b.posture().time < env_.get_t_min()) || 
  	(b.posture().time > env_.get_t_max()) || 
  	(b.posture().trans()[0] < env_.get_x_min()) || 
  	(b.posture().trans()[0] > env_.get_x_max()) || 
  	(b.posture().trans()[2] < env_.get_z_min()) || 
  	(b.posture().trans()[2] > env_.get_z_max())) {
    return false;
  } else {
  	return true;
  }
}

void renew_dangles( Danglings &danglings, const vector<shared_ptr<Patch> > &patches, const int pat_idx, const vector<ConnectInfo> &coninfos, const Env &env )
{
  vector<int> black(patches[pat_idx]->boundaries.size(), 0);
  for (auto it = coninfos.begin(); it != coninfos.end(); ++it) {
	black[it->b1] = 1;	
	danglings.erase(it->pa2, it->b2);
	patches[it->pa2]->boundaries[it->b2].posture().status.is_highlight = false;	
  }

  for (auto it=black.begin(); it!=black.end(); ++it) {
	if (*it == 0) {
	  Dangling dangling;
	  dangling.patches_idx = pat_idx;
	  dangling.boundary_idx = it - black.begin();
	  auto &jt = patches[dangling.patches_idx]->boundaries[dangling.boundary_idx];
	  if (!is_stitchable_dangling(env, jt)) continue;
	  dangling.pos = jt.posture().trans();
	  dangling.is_begin = !jt.forward;
	  danglings.push(dangling);
	  jt.posture().set_status_in_viewer(true, cml::vector3(1.0, 0.25, 0.25));
	}
  }  
}


bool select( shared_ptr<Patch> &out, const vector<shared_ptr<Patch>> &patches, const map<string, Patch> &patch_type ) 
{	
  return false;
}

int select( Dangling &d, const Danglings &ds )
{
  if (ds.dead()) return -1;
  for (size_t i=0; i<ds.size(); ++i) {
	if (!ds[i].is_dead) {
	  d = ds[i];
	  return i;
	}
  }
  return -1;
}

int select( const Danglings &ds )
{
  if (ds.dead()) return -1;
  for (size_t i=0; i<ds.size(); ++i) {
	if (!ds[i].is_dead) {
	  return i;
	}
  }
  return -1;
}

bool stitch_a_patch(vector<shared_ptr<Patch> > &patches, Danglings &dtable, const int didx, const map<string, Patch> &patch_type, const char *in_pat, const Env &env)
{
  Dangling dangle = dtable[didx];
  const Boundary &it_d = patches[dangle.patches_idx]->boundaries[dangle.boundary_idx];
  const auto &pos2 = it_d.posture();  
  if (pos2.type_ == -1) return false;

  shared_ptr<Patch> t_patch(new Patch(patch_type.find(in_pat)->second));

  map< size_t,vector<ConnectInfo> > stitch_table;
  for (auto it_b=t_patch->boundaries.begin(); it_b!=t_patch->boundaries.end(); ++it_b) {	
	if ((it_b->forward == it_d.forward) || (it_b->posture_type() != it_d.posture_type())) continue;

	const auto &pos1 = it_b->posture();
	t_patch->transform_between_posture(pos2, pos1);

	vector<ConnectInfo> connect_infos;
	get_connect_infos(connect_infos, t_patch, patches, 4.8, env);

	if (!connect_infos.empty()) {
	  if (is_stitchable(patches, t_patch, &patch_type, connect_infos, env)) {
		stitch_table[it_b-t_patch->boundaries.begin()] = connect_infos;
	  }
	}
  }

  if (stitch_table.empty()) return false;

  auto max_it = max_element(stitch_table.begin(), stitch_table.end(), 
	[](pair< size_t,vector<ConnectInfo> > p1, pair<size_t,vector<ConnectInfo> > p2) {return p1.second.size() < p2.second.size();});
  do_stitch(patches, t_patch, &patch_type, max_it->second, env);
  renew_dangles(dtable, patches, patches.size()-1, max_it->second, env);

  return true;
}

void stitch_all_patches( vector<shared_ptr<Patch>> &patches, const map<string, Patch> &patch_type, const Env &env )
{
  Danglings dtable;
  get_naive_dangling_table(dtable, patches, env);
  int didx = select(dtable);
  while (didx != -1) {
	if (!stitch_a_patch(patches, dtable, didx, patch_type, "jump", env)) {
	  patches[dtable[didx].patches_idx]->boundaries[dtable[didx].boundary_idx].posture().set_status_in_viewer(true, cml::vector3(0.3, 0.3, 0.3));
	  dtable[didx].is_dead = true;
	}
	didx = select(dtable);
  }
}

string get_random_type(const map<string, Patch> &patch_types)
{
  size_t size = patch_types.size();
  size_t trav = cml::random_integer(1, size) - 1;
  auto it = patch_types.begin();
  for (size_t i=0; i<trav; ++i) { ++it;}
  return it->first;
}

vector<string> shake_patch_types(const map<string,Patch> &patch_type)
{
  vector<string> ret;
  for (auto it=patch_type.begin(); it!=patch_type.end(); ++it) {
	ret.push_back(it->first);
  }
  random_shuffle(ret.begin(), ret.end());
  return ret;
}

bool stitch_patches( vector< shared_ptr<Patch> > &patches, const size_t num_pat, const map<string,Patch> &patch_type, const Env &env )
{
  Danglings dtable;
  get_naive_dangling_table(dtable, patches, env);
  int didx = select(dtable);
  if (didx == -1) {
	cerr << "There are no danglings of these patches!!" << endl;
	return false;
  }

  size_t count = 0;
  while (count++ < num_pat) {
	//string patch_name = get_random_type(patch_type);
	vector<string> mixed_type = shake_patch_types(patch_type);
	auto mit = mixed_type.begin();
	while (!stitch_a_patch(patches, dtable, didx, patch_type, mit->c_str(), env)) {
	  if (++mit == mixed_type.end()) {
		patches[dtable[didx].patches_idx]->boundaries[dtable[didx].boundary_idx].posture().set_status_in_viewer(true, cml::vector3(0.3, 0.3, 0.3));
		dtable[didx].is_dead = true;
		--count;  break;
	  }
	}
	didx = select(dtable);
	if (didx == -1) return false;
  }
  return true;
}


#include <queue>
#include "QPerformanceTimer.h"

void get_energies_path( vector<string> &sampling_patches, map<string, Patch> * patch_type, const int pa2, const int b2, vector<shared_ptr<Patch>> & patches, Env & env, vector<Stitch_energy> &energies ) 
{
	const auto it_b2 = &(patches[pa2]->boundaries[b2]);
	const auto &p2 = it_b2->posture();

	random_shuffle(sampling_patches.begin(), sampling_patches.end());
	for (auto it_p = sampling_patches.begin(); it_p != sampling_patches.end(); ++it_p)
	{
		//boundary만 넣어서 빠르게 함
		shared_ptr<Patch> pa1(new Patch((patch_type->find(*it_p))->second, true));

		for (auto it_b1 = pa1->boundaries.begin(); it_b1 != pa1->boundaries.end(); ++it_b1)
		{
			int it_b1_i = (int)(it_b1 - pa1->boundaries.begin());
			bool is_stitch_history = it_b2->stitch_history.count(pair<string, int>(pa1->name, it_b1_i)) > 0;
			if (is_stitch_history == true)
				cout << "history determined" << endl;

			if (it_b1->forward == it_b2->forward || it_b1->posture_type() != it_b2->posture_type() || is_stitch_history == true)
				continue;

			const auto &p1 = it_b1->posture();
			pa1->transform_between_posture(p2, p1);

			vector<ConnectInfo> connect_infos;
			get_connect_infos(connect_infos, pa1, patches, 5.0, env);

			//energy
			double energy = 100.0;
			int connect_n = connect_infos.size();
			int new_n = pa1->boundaries.size();

			energy += new_n;
			if (2 * connect_n  < new_n)
				energy -= 1000.0;

			if (connect_n >= 2)
				energy += 10.0;

			//5,-5
			vector<pair<cml::vector3, double>> dead_pos;
			for (int pa2_i = 0; pa2_i < patches.size(); ++pa2_i)
			{
				for (int b2_i = 0; b2_i < patches[pa2_i]->boundaries.size(); ++b2_i)
				{
					bool is_dead = true;
					if (get_end_type(patches[pa2_i]->boundaries[b2_i], env.range["t_min"], env.range["t_max"]) != "dead") {
						is_dead = false;
					}
					for (auto it_c = connect_infos.begin(); it_c != connect_infos.end(); ++it_c)
					{
						if (it_c->pa2 == pa2_i && it_c->b2 == b2_i)
							is_dead = false;
					}

					/*if (is_dead == true)
						dead_pos.push_back(pair<cml::vector3, int>(patches[pa2_i]->boundaries[b2_i].posture().trans(), patches[pa2_i]->boundaries[b2_i].posture().time));*/
				}
			}
			for (int b1_i = 0; b1_i < pa1->boundaries.size(); ++b1_i)
			{
				bool is_dead = true;
				if (get_end_type(pa1->boundaries[b1_i], env.range["t_min"], env.range["t_max"]) != "dead") {
					is_dead = false;
				}
				for (auto it_c = connect_infos.begin(); it_c != connect_infos.end(); ++it_c)
				{
					if (it_c->b1 == b1_i)
						is_dead = false;
				}
				if (is_dead == true)
					dead_pos.push_back(pair<cml::vector3, int>(pa1->boundaries[b1_i].posture().trans(),pa1->boundaries[b1_i].posture().time) );
			}
			
			if (dead_pos.size() != 0)
			{
				//goal
				cml::vector3 sum_pos(0,0,0);
				for (auto it_dp = dead_pos.begin(); it_dp != dead_pos.end(); ++it_dp)
				{
					sum_pos += it_dp->first;
				}
				cml::vector3 average_pos = sum_pos / (double)(dead_pos.size());
				cml::vector3 goal_pos(0, 0, -5);
				cml::vector3 diff_pos = average_pos - goal_pos;
				diff_pos[1] = 0.0;
				double diff = diff_pos.length();
				energy -= 2.5 * diff;

				//coherence
				if (dead_pos.size() >= 2)
				{
					//cout << "coherence " << endl;
					for (int i = 0; i < dead_pos.size(); ++i)
					{
						//cout << dead_pos[i] << endl;
						for (int j = i + 1; j < dead_pos.size(); ++j)
						{
							double len = cml::length(dead_pos[i].first - dead_pos[j].first);
						//	cout << len << endl;
							double time_len = fabs(dead_pos[i].second - dead_pos[j].second);
							//if (time_len < 300 && len > 15.)	energy -= 1000.;
						}
					}
					//cout << "===" << endl;
				}
			}

			//cout << "energy " << energy << endl;
				
			Stitch_energy se;
			se.name = pa1->name;
			se.b = it_b1_i;
			se.energy = energy;

			energies.push_back(se);

		}
	}
}

void get_energies_path_target( vector<string> &sampling_patches, map<string, Patch> * patch_type, const int pa2, const int b2, vector<shared_ptr<Patch>> & patches, Env & env, vector<Stitch_energy> &energies, cml::vector3 target, cml::vector3 target1 )
{
	const auto it_b2 = &(patches[pa2]->boundaries[b2]);
	const auto &p2 = it_b2->posture();

    //target 색깔
    bool is_target_red = false;
    if (it_b2->p_motion->color[0] > 0.5)
      is_target_red = true;

	random_shuffle(sampling_patches.begin(), sampling_patches.end());
	for (auto it_p = sampling_patches.begin(); it_p != sampling_patches.end(); ++it_p)
	{
		//boundary만 넣어서 빠르게 함
		shared_ptr<Patch> pa1(new Patch((patch_type->find(*it_p))->second, true));

		for (auto it_b1 = pa1->boundaries.begin(); it_b1 != pa1->boundaries.end(); ++it_b1)
		{
			int it_b1_i = (int)(it_b1 - pa1->boundaries.begin());
		    if (it_b1->forward == it_b2->forward || it_b1->posture_type() != it_b2->posture_type())
				continue;

			const auto &p1 = it_b1->posture();
			pa1->transform_between_posture(p2, p1);

			vector<ConnectInfo> connect_infos;
			get_connect_infos(connect_infos, pa1, patches, 4.0, env);

			//energy
			double energy = 100.0;
			int connect_n = connect_infos.size();
			int new_n = pa1->boundaries.size();

            //end가 변하지 않도록 함
			if (2 * connect_n  != new_n)
				energy -= 1000.0;

            //같은 편끼리만 싸우도록
            if (connect_infos.size() == 2)
            {
              bool is_red0 = patches[connect_infos[0].pa2]->boundaries[connect_infos[0].b2].p_motion->color[0] > 0.5;
              bool is_red1 = patches[connect_infos[1].pa2]->boundaries[connect_infos[1].b2].p_motion->color[0] > 0.5;
              if (is_red0 == is_red1)
                energy -= 1000.0;
            }

            //5,-5
            vector<pair<cml::vector3, double>> dead_pos;
			for (int b1_i = 0; b1_i < pa1->boundaries.size(); ++b1_i)
			{
				bool is_dead = true;
				if (get_end_type(pa1->boundaries[b1_i], env.range["t_min"], env.range["t_max"]) != "dead") {
					is_dead = false;
				}
				for (auto it_c = connect_infos.begin(); it_c != connect_infos.end(); ++it_c)
				{
					if (it_c->b1 == b1_i)
						is_dead = false;
				}
				if (is_dead == true)
					dead_pos.push_back(pair<cml::vector3, int>(pa1->boundaries[b1_i].posture().trans(),pa1->boundaries[b1_i].posture().time) );
			}
			
			if (dead_pos.size() != 0)
			{
				//goal
				cml::vector3 sum_pos(0,0,0);
				for (auto it_dp = dead_pos.begin(); it_dp != dead_pos.end(); ++it_dp)
				{
					sum_pos += it_dp->first;
				}
				cml::vector3 average_pos = sum_pos / (double)(dead_pos.size());
      			cml::vector3 diff_pos;
                if (is_target_red == true)
                  diff_pos = average_pos - target;
                else
                  diff_pos = average_pos - target1;

				diff_pos[1] = 0.0;
				double diff = diff_pos.length();
				energy -= 2.5 * diff;

				//coherence
				if (dead_pos.size() >= 2)
				{
					//cout << "coherence " << endl;
					for (int i = 0; i < dead_pos.size(); ++i)
					{
						//cout << dead_pos[i] << endl;
						for (int j = i + 1; j < dead_pos.size(); ++j)
						{
							double len = cml::length(dead_pos[i].first - dead_pos[j].first);
						//	cout << len << endl;
							double time_len = fabs(dead_pos[i].second - dead_pos[j].second);
							//if (time_len < 300 && len > 15.)	energy -= 1000.;
						}
					}
					//cout << "===" << endl;
				}
			}

			//cout << "energy " << energy << endl;
				
			Stitch_energy se;
			se.name = pa1->name;
			se.b = it_b1_i;
			se.energy = energy;

			energies.push_back(se);
		}
	}
}

void get_energies_path_target_fight( vector<string> &sampling_patches, map<string, Patch> * patch_type, const int pa2, const int b2, vector<shared_ptr<Patch>> & patches, Env & env, vector<Stitch_energy> &energies, cml::vector3 target, cml::vector3 target1 )
{
  const auto it_b2 = &(patches[pa2]->boundaries[b2]);
  const auto &p2 = it_b2->posture();

  //target 색깔
  bool is_target_red = false;
  if (it_b2->p_motion->color[0] > 0.5)
    is_target_red = true;

  random_shuffle(sampling_patches.begin(), sampling_patches.end());
  for (auto it_p = sampling_patches.begin(); it_p != sampling_patches.end(); ++it_p)
  {
    //boundary만 넣어서 빠르게 함
    shared_ptr<Patch> pa1(new Patch((patch_type->find(*it_p))->second, true));

    for (auto it_b1 = pa1->boundaries.begin(); it_b1 != pa1->boundaries.end(); ++it_b1)
    {
      int it_b1_i = (int)(it_b1 - pa1->boundaries.begin());
      if (it_b1->forward == it_b2->forward || it_b1->posture_type() != it_b2->posture_type())
        continue;

      const auto &p1 = it_b1->posture();
      pa1->transform_between_posture(p2, p1);

      vector<ConnectInfo> connect_infos;
      get_connect_infos(connect_infos, pa1, patches, 4.0, env);

      //energy
      double energy = 500.0;
      int connect_n = connect_infos.size();
      int new_n = pa1->boundaries.size();

      //end가 변하지 않도록 함
      if (2 * connect_n  != new_n)
        energy -= 1000.0;

      //같은 편끼리만 싸우도록
      if (connect_infos.size() == 2)
      {
        bool is_red0 = patches[connect_infos[0].pa2]->boundaries[connect_infos[0].b2].p_motion->color[0] > 0.5;
        bool is_red1 = patches[connect_infos[1].pa2]->boundaries[connect_infos[1].b2].p_motion->color[0] > 0.5;
        if (is_red0 == is_red1)
          energy -= 1000.0;
      }

      //충돌 더 많게
      energy += 0*connect_n;

      //5,-5
      vector<pair<cml::vector3, double>> dead_pos;
      for (int b1_i = 0; b1_i < pa1->boundaries.size(); ++b1_i)
      {
        bool is_dead = true;
        if (get_end_type(pa1->boundaries[b1_i], env.range["t_min"], env.range["t_max"]) != "dead") {
          is_dead = false;
        }
        for (auto it_c = connect_infos.begin(); it_c != connect_infos.end(); ++it_c)
        {
          if (it_c->b1 == b1_i)
            is_dead = false;
        }
        if (is_dead == true)
          dead_pos.push_back(pair<cml::vector3, int>(pa1->boundaries[b1_i].posture().trans(),pa1->boundaries[b1_i].posture().time) );
      }

      if (dead_pos.size() != 0)
      {
        //goal
        cml::vector3 sum_pos(0,0,0);
        for (auto it_dp = dead_pos.begin(); it_dp != dead_pos.end(); ++it_dp)
        {
          sum_pos += it_dp->first;
        }
        cml::vector3 average_pos = sum_pos / (double)(dead_pos.size());
        cml::vector3 diff_pos;
        if (is_target_red == true)
          diff_pos = average_pos - target;
        else
          diff_pos = average_pos - target1;

        diff_pos[1] = 0.0;
        double diff = diff_pos.length();
        energy -= 2.5 * diff;
       
        //coherence
        if (dead_pos.size() >= 2)
        {
          //cout << "coherence " << endl;
          for (int i = 0; i < dead_pos.size(); ++i)
          {
            //cout << dead_pos[i] << endl;
            for (int j = i + 1; j < dead_pos.size(); ++j)
            {
              double len = cml::length(dead_pos[i].first - dead_pos[j].first);
              //	cout << len << endl;
              double time_len = fabs(dead_pos[i].second - dead_pos[j].second);
              //if (time_len < 300 && len > 15.)	energy -= 1000.;
            }
          }
          //cout << "===" << endl;
        }
      }

      //cout << "energy " << energy << endl;

      Stitch_energy se;
      se.name = pa1->name;
      se.b = it_b1_i;
      se.energy = energy;

      energies.push_back(se);
    }
  }
}

void SkeletonPatch( vector<shared_ptr<Patch>> &patches, Env &env, map<string, Patch> * patch_type, vector<Patch> * patch_type_unary )
{
	srand(79797+5+1);

	//phase1에서 patches sampling 정도 조절
	vector<string> sampling_patches;
	for (auto it = patch_type->begin(); it != patch_type->end(); ++it) 	{
		sampling_patches.push_back(it->first);
	}

	for (int i = 0; i < 1; ++i)
	{
		random_shuffle(sampling_patches.begin(), sampling_patches.end());
		for (auto it_p = sampling_patches.begin(); it_p != sampling_patches.end(); ++it_p)
		{
			shared_ptr<Patch> pa1(new Patch((patch_type->find(*it_p))->second));
			if (pa1->motions.size() == 1)
				continue;
			env.rand_scatter(*pa1);
			vector<ConnectInfo> connect_infos;
			get_connect_infos(connect_infos, pa1, patches, 4.8, env);

			if (stitch(patches, pa1, patch_type, connect_infos, env, false) == true) {
				break;
			}
		}
	}

	for (int ii = 0; ii < 10000; ++ii)
	{
		vector<pair<int,int>> traverse_vec = get_traverser_vec(patches, env);
		for (auto it = traverse_vec.begin(); it != traverse_vec.end(); ++it)
		{
			vector<Stitch_energy> energies;

			get_energies_path(sampling_patches, patch_type, it->first, it->second, patches, env, energies);

			sort(energies.begin(), energies.end(), [](const Stitch_energy &a, const Stitch_energy &b){return a.energy > b.energy;});

			/*for (auto it = energies.begin(); it != energies.end(); ++it)
			cout << it->name << ' ' << it->b << ' ' << it->energy << endl;*/

			const auto it_b2 = &(patches[it->first]->boundaries[it->second]);
			const auto &p2 = it_b2->posture();
			for (auto it = energies.begin(); it != energies.end(); ++it)
			{
				if (it->energy < 0.)
					continue;

				shared_ptr<Patch> pa1(new Patch((patch_type->find(it->name))->second));
				const auto &p1 = pa1->boundaries[it->b].posture();
				pa1->transform_between_posture(p2, p1);
				vector<ConnectInfo> connect_infos;
				get_connect_infos(connect_infos, pa1, patches, 5.0, env);

				if (stitch(patches, pa1, patch_type, connect_infos, env, false) == true) {
					cout << "stitch1 " << endl;
					if (pa1->motions.size() >= 2)
						cout << "interaction!" << endl;

					//어쩌다 연결된 dangling말고 지금 연결하려고 하는 dangling에만 history만 기록하자.
					it_b2->stitch_history.insert(pair<string,int>(it->name, it->b));
					goto resolve_dead;
				}
			}

			//for (auto it = energies.begin(); it != energies.end(); ++it)
			//	cout << it->name << ' ' << it->b << ' ' << it->energy << endl;
			//delete the patch containing it.
			delete_patch(patches, it->first, cout);
			goto resolve_dead;
		}
resolve_dead:;
		cout << endl << "ii " << ii << endl;

		int dead_num = get_end_num(patches, env)["dead"];
		cout << "dead num: " << dead_num << endl;

		if ( dead_num == 0) {
			break;
		}
	}
}

void path( vector<shared_ptr<Patch>> &patches, Env &env, map<string, Patch> * patch_type, vector<Patch> * patch_type_unary )
{
	srand(79797+5+2);

	vector<string> sampling_patches;
	for (auto it = patch_type->begin(); it != patch_type->end(); ++it) 	{
		sampling_patches.push_back(it->first);
	}

	//for (int i = 0; i < 2; ++i)
	//{
	//	random_shuffle(sampling_patches.begin(), sampling_patches.end());
	//	while(true)
	//	{
	//		for (auto it_p = sampling_patches.begin(); it_p != sampling_patches.end(); ++it_p)
	//		{
	//			shared_ptr<Patch> pa1(new Patch((patch_type->find(*it_p))->second));
	//			if (pa1->motions.size() == 1)
	//				continue;

	//			env.scatter_range(*pa1, -1, 1, -1, 1, 80, 120);

	//			vector<ConnectInfo> connect_infos;
	//			get_connect_infos(connect_infos, pa1, patches, 4.8, env);

	//			if (stitch(patches, pa1, patch_type, connect_infos, env, false) == true) {
	//				goto one_tile;
	//			}
	//		}
	//	}
	//	one_tile:;
	//}

	{
		shared_ptr<Patch> pa(new Patch((patch_type->find("bow_4"))->second));
		pa->translate_time(100);
		patches.push_back(pa);
	}
	

	for (int ii = 0; ii < 10000; ++ii)
	{
		vector<pair<int,int>> traverse_vec = get_traverser_vec(patches, env);
		for (auto it = traverse_vec.begin(); it != traverse_vec.end(); ++it)
		{
			vector<Stitch_energy> energies;

			get_energies_path(sampling_patches, patch_type, it->first, it->second, patches, env, energies);
		
			sort(energies.begin(), energies.end(), [](const Stitch_energy &a, const Stitch_energy &b){return a.energy > b.energy;});

			/*for (auto it = energies.begin(); it != energies.end(); ++it)
				cout << it->name << ' ' << it->b << ' ' << it->energy << endl;*/
		
			const auto it_b2 = &(patches[it->first]->boundaries[it->second]);
			const auto &p2 = it_b2->posture();
			for (auto it = energies.begin(); it != energies.end(); ++it)
			{
				if (it->energy < 0.)
					continue;

				shared_ptr<Patch> pa1(new Patch((patch_type->find(it->name))->second));
				const auto &p1 = pa1->boundaries[it->b].posture();
				pa1->transform_between_posture(p2, p1);
				vector<ConnectInfo> connect_infos;
				get_connect_infos(connect_infos, pa1, patches, 5.5, env);

				if (stitch(patches, pa1, patch_type, connect_infos, env, false) == true) {
					cout << "stitch1 " << endl;
					if (pa1->motions.size() >= 2)
						cout << "interaction!" << endl;

					//어쩌다 연결된 dangling말고 지금 연결하려고 하는 dangling에만 history만 기록하자.
					it_b2->stitch_history.insert(pair<string,int>(it->name, it->b));
					goto resolve_dead;
				}
			}
				
			//for (auto it = energies.begin(); it != energies.end(); ++it)
			//	cout << it->name << ' ' << it->b << ' ' << it->energy << endl;
			//delete the patch containing it.
			delete_patch(patches, it->first, cout);
			goto resolve_dead;
		}
		resolve_dead:;
		cout << endl << "ii " << ii << endl;

		int dead_num = get_end_num(patches, env)["dead"];
		cout << "dead num: " << dead_num << endl;

		if ( dead_num == 0) {
			break;
		}
	}
}

void get_energies( vector<string> &sampling_patches, map<string, Patch> * patch_type, const int pa2, const int b2, vector<shared_ptr<Patch>> & patches, Env & env, vector<Stitch_energy> &energies ) 
{
  const auto it_b2 = &(patches[pa2]->boundaries[b2]);
  const auto &p2 = it_b2->posture();

  random_shuffle(sampling_patches.begin(), sampling_patches.end());
  for (auto it_p = sampling_patches.begin(); it_p != sampling_patches.end(); ++it_p)
  {
    //boundary만 넣어서 빠르게 함
    shared_ptr<Patch> pa1(new Patch((patch_type->find(*it_p))->second, true));

    for (auto it_b1 = pa1->boundaries.begin(); it_b1 != pa1->boundaries.end(); ++it_b1)
    {
      int it_b1_i = (int)(it_b1 - pa1->boundaries.begin());
      bool is_stitch_history = it_b2->stitch_history.count(pair<string, int>(pa1->name, it_b1_i)) > 0;
      if (is_stitch_history == true)
        cout << "history determined" << endl;

      if (it_b1->forward == it_b2->forward || it_b1->posture_type() != it_b2->posture_type() || is_stitch_history == true)
        continue;

      const auto &p1 = it_b1->posture();
      pa1->transform_between_posture(p2, p1);

      vector<ConnectInfo> connect_infos;
      get_connect_infos(connect_infos, pa1, patches, 5.0, env);

      //energy
      double energy = 100.0;
      int connect_n = connect_infos.size();
      int new_n = pa1->boundaries.size();

      energy += new_n;
      if (2 * connect_n  < new_n)
        energy -= 1000.0;

      if (connect_n >= 2)
        energy += 10.0;

      //5,-5
      vector<pair<cml::vector3, double>> dead_pos;
      for (int pa2_i = 0; pa2_i < patches.size(); ++pa2_i)
      {
        for (int b2_i = 0; b2_i < patches[pa2_i]->boundaries.size(); ++b2_i)
        {
          bool is_dead = true;
          if (get_end_type(patches[pa2_i]->boundaries[b2_i], env.range["t_min"], env.range["t_max"]) != "dead") {
            is_dead = false;
          }
          for (auto it_c = connect_infos.begin(); it_c != connect_infos.end(); ++it_c)
          {
            if (it_c->pa2 == pa2_i && it_c->b2 == b2_i)
              is_dead = false;
          }

          /*if (is_dead == true)
          dead_pos.push_back(pair<cml::vector3, int>(patches[pa2_i]->boundaries[b2_i].posture().trans(), patches[pa2_i]->boundaries[b2_i].posture().time));*/
        }
      }
      for (int b1_i = 0; b1_i < pa1->boundaries.size(); ++b1_i)
      {
        bool is_dead = true;
        if (get_end_type(pa1->boundaries[b1_i], env.range["t_min"], env.range["t_max"]) != "dead") {
          is_dead = false;
        }
        for (auto it_c = connect_infos.begin(); it_c != connect_infos.end(); ++it_c)
        {
          if (it_c->b1 == b1_i)
            is_dead = false;
        }
        if (is_dead == true)
          dead_pos.push_back(pair<cml::vector3, int>(pa1->boundaries[b1_i].posture().trans(),pa1->boundaries[b1_i].posture().time) );
      }

      if (dead_pos.size() != 0)
      {
        //goal
        cml::vector3 sum_pos(0,0,0);
        for (auto it_dp = dead_pos.begin(); it_dp != dead_pos.end(); ++it_dp)
        {
          sum_pos += it_dp->first;
        }
        cml::vector3 average_pos = sum_pos / (double)(dead_pos.size());
        //cml::vector3 goal_pos(0, 0, -5);
        cml::vector3 goal_pos(5, 0, 5);
        cml::vector3 diff_pos = average_pos - goal_pos;
        diff_pos[1] = 0.0;
        double diff = diff_pos.length();
        energy -= 5*2.5 * diff;
        ////coherence
        //if (dead_pos.size() >= 2)
        //{
        //  //cout << "coherence " << endl;
        //  for (int i = 0; i < dead_pos.size(); ++i)
        //  {
        //    //cout << dead_pos[i] << endl;
        //    for (int j = i + 1; j < dead_pos.size(); ++j)
        //    {
        //      double len = cml::length(dead_pos[i].first - dead_pos[j].first);
        //      //	cout << len << endl;
        //      double time_len = fabs(dead_pos[i].second - dead_pos[j].second);
        //      //if (time_len < 300 && len > 15.)	energy -= 1000.;
        //    }
        //  }
        //  //cout << "===" << endl;
        //}
      }

      
      //cout << "energy " << energy << endl;

      Stitch_energy se;
      se.name = pa1->name;
      se.b = it_b1_i;
      se.energy = energy;

      energies.push_back(se);
    }
  }
}

void get_energies_general( vector<string> &sampling_patches, map<string, Patch> * patch_type, const int pa2, const int b2, vector<shared_ptr<Patch>> & patches, Env & env, vector<Stitch_energy> &energies ) 
{
  const auto it_b2 = &(patches[pa2]->boundaries[b2]);
  const auto &p2 = it_b2->posture();

  random_shuffle(sampling_patches.begin(), sampling_patches.end());
  for (auto it_p = sampling_patches.begin(); it_p != sampling_patches.end(); ++it_p)
  {
    //boundary만 넣어서 빠르게 함
    shared_ptr<Patch> pa1(new Patch((patch_type->find(*it_p))->second, true));

    for (auto it_b1 = pa1->boundaries.begin(); it_b1 != pa1->boundaries.end(); ++it_b1)
    {
      int it_b1_i = (int)(it_b1 - pa1->boundaries.begin());
      bool is_stitch_history = it_b2->stitch_history.count(pair<string, int>(pa1->name, it_b1_i)) > 0;
      if (is_stitch_history == true)
        cout << "history determined" << endl;

      if (it_b1->forward == it_b2->forward || it_b1->posture_type() != it_b2->posture_type() || is_stitch_history == true)
        continue;

      const auto &p1 = it_b1->posture();
      pa1->transform_between_posture(p2, p1);

      vector<ConnectInfo> connect_infos;
      get_connect_infos(connect_infos, pa1, patches, 5.0, env);

      //energy
      double energy = 100.0;
      int connect_n = connect_infos.size();
      int new_n = pa1->boundaries.size();

      energy += new_n;
      if (2 * connect_n  < new_n)
        energy -= 1000.0;

      if (connect_n >= 2)
        energy += 10.0;

      //cout << "energy " << energy << endl;

      Stitch_energy se;
      se.name = pa1->name;
      se.b = it_b1_i;
      se.energy = energy;

      energies.push_back(se);
    }
  }
}

void deterministic( vector<shared_ptr<Patch>> &patches, Env &env, map<string, Patch> * patch_type, void (*energy_func)( vector<string> &sampling_patches, map<string, Patch> * patch_type, const int pa2, const int b2, vector<shared_ptr<Patch>> & patches, Env & env, vector<Stitch_energy> &energies ) )
{
  vector<string> sampling_patches;
  for (auto it = patch_type->begin(); it != patch_type->end(); ++it) 	{
    sampling_patches.push_back(it->first);
  }

  for (int ii = 0; ii < 1000; ++ii)
  {
    vector<pair<int,int>> traverse_vec = get_traverser_vec(patches, env);
    for (auto it = traverse_vec.begin(); it != traverse_vec.end(); ++it)
    {
      vector<Stitch_energy> energies;
      //get_energies_general(sampling_patches, patch_type, it->first, it->second, patches, env, energies);
      energy_func(sampling_patches, patch_type, it->first, it->second, patches, env, energies);
      sort(energies.begin(), energies.end(), [](const Stitch_energy &a, const Stitch_energy &b){return a.energy > b.energy;});

      const auto it_b2 = &(patches[it->first]->boundaries[it->second]);
      const auto &p2 = it_b2->posture();
      for (auto it = energies.begin(); it != energies.end(); ++it)
      {
        if (it->energy < 0.)
          continue;

        shared_ptr<Patch> pa1(new Patch((patch_type->find(it->name))->second));
        const auto &p1 = pa1->boundaries[it->b].posture();
        pa1->transform_between_posture(p2, p1);
        vector<ConnectInfo> connect_infos;
        get_connect_infos(connect_infos, pa1, patches, 5.5, env);

        if (stitch(patches, pa1, patch_type, connect_infos, env, false) == true) {
          cout << "stitch1 " << endl;
          if (pa1->motions.size() >= 2)
            cout << "interaction!" << endl;

          //어쩌다 연결된 dangling말고 지금 연결하려고 하는 dangling에만 history만 기록하자.
          it_b2->stitch_history.insert(pair<string,int>(it->name, it->b));
          goto resolve_dead;
        }
      }

      //for (auto it = energies.begin(); it != energies.end(); ++it)
      //	cout << it->name << ' ' << it->b << ' ' << it->energy << endl;
      //delete the patch containing it.
      delete_patch(patches, it->first, cout);
      goto resolve_dead;
    }
resolve_dead:;
    cout << endl << "ii " << ii << endl;

    int dead_num = get_end_num(patches, env)["dead"];
    cout << "dead num: " << dead_num << endl;

    if ( dead_num == 0) {
      break;
    }
  }
}
