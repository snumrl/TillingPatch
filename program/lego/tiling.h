// +-------------------------------------------------------------------------
// | tiling.h
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

#include "patch.h"
#include "environment.h"
#include "collision.h"


struct ConnectInfo
{
	ConnectInfo() {}
	ConnectInfo( int b1, int pa2, int b2, double distance = 1.0);
	ConnectInfo( int b1, int pa2, int b2, double distance, cml::vector3 trans_pos );

	int b1;
	int pa2;
	int b2;
	double distance;
	cml::vector3 trans_pos;
};


bool stitch(vector<shared_ptr<Patch> > &patches, const shared_ptr<Patch> &pa, const map<string,Patch> *patch_type, const vector<ConnectInfo> &connect_infos, const Env &env = Env(), bool is_compensation = false, bool jittering = false, int jitter_start_i=1, int jitter_end_i=24, bool ignore_collision = false);

void get_connect_infos(vector<ConnectInfo> &connect_infos, const shared_ptr<Patch> &pa_new, const vector<shared_ptr<Patch>> &patches, double thres, const Env &env);

bool is_stop();

set<int> get_edit_patches(const vector<ConnectInfo> & connect_infos, const int center_patch_i );

void patches_edit( vector<shared_ptr<Patch>> &patches, int center_patch_i, const vector<ConnectInfo> &connect_infos, const set<int> &edit_patches, const map<string, Patch> *patch_type, const Env &env, bool is_jitter = false, const ml::Posture& jitter_posture = NULL);

void get_diff2s(vector<cml::vector2d> &diff2s);

bool compensation( vector<shared_ptr<Patch>> & patches, int pa_i, const map<string, Patch> * patch_type, double thres = 3.9, const Env& env = Env() );

void set_inner_cons( const vector<shared_ptr<Patch>> & patches, const map<int,int> &patch2motion_offset, vector<Constraint> &cons );

void Pinning( vector<Constraint> &cons, vector<shared_ptr<Patch>> & patches, const set<int> &edit_patches, map<int,int> patch2motion_offset );

void metropolis( vector<shared_ptr<Patch>> &patches, const map<string, Patch> * patch_type, double max_time, int last_patch_id, int max_attempt, vector<string> &sampling_patches, Env &env = Env(), double k = 0.0008, double alpha = 0.0001, ostream& out = std::cout, time_t start_time = time(NULL), ostream &out2 = ofstream() );

set<int> dangling_stitch( vector<shared_ptr<Patch>> &patches, const vector<Patch> &patch_type_unary, const map<string, Patch> * patch_type, const vector<pair<int,int>> &traverse_vec, Env& env, ostream& out, time_t start_time, ostream &out2  );

vector<pair<int,int>> get_traverser_vec(const vector<shared_ptr<Patch>> &patches, Env &env);

void dangling_delete(vector<shared_ptr<Patch>> &patches, const set<int> &delete_list, ostream& out );

void remove_dangling( vector<shared_ptr<Patch>> &patches, const vector<Patch> &patch_type_unary, const map<string, Patch> * patch_type, Env &env = Env(), ostream& out = std::cout, time_t start_time = time(NULL), ostream &out2 = ofstream()  );

bool remove_dangling_oneturn( vector<shared_ptr<Patch>> & patches, const vector<Patch> & patch_type_unary, const map<string, Patch> * patch_type, Env & env, ostream& out, time_t start_time, ostream &out2);

void warp_boundary(Patch &pa);
void warp_boundary_half_direction_height(Patch &pa);
void warp_boundary_patches( vector<shared_ptr<Patch>> &patches );

bool edit_ok_col_ok(const vector<shared_ptr<Patch>> &patches, const set<int> &edit_patches, const map<string, Patch> *patch_type, const Env &env, bool ignore_collision = false);




struct Dangling
{
  Dangling() { init(); }

  size_t patches_idx;
  size_t boundary_idx;
  cml::vector3 pos;
  bool is_begin;
  bool is_dead;

  void init() { patches_idx = 0, boundary_idx = 0, pos = cml::vector3(0.,0.,0.), is_begin = true, is_dead = false; }
};

struct Danglings
{	
  typedef vector<Dangling>::iterator Iterator;
  typedef vector<Dangling>::const_iterator Const_iterator;

  vector<Dangling> danglings;

  Dangling& operator[] (size_t idx) { return danglings[idx];}
  const Dangling& operator[] (size_t idx) const { return danglings[idx];}

  size_t size() const			{return danglings.size();}
  void clear()					{danglings.clear();}
  void erase(size_t pat_idx, size_t b_idx);
  void push(const Dangling& d)	{danglings.push_back(d);}
  bool empty() const			{return danglings.empty();}
  bool dead()  const			{for (auto it=danglings.begin(); it!=danglings.end(); ++it) if (!it->is_dead) return false; return true; }

  Iterator begin()				{return danglings.begin();}
  Iterator end()				{return danglings.end();}
  Const_iterator begin()const	{return danglings.begin();}
  Const_iterator end() const	{return danglings.end();}
};

bool is_stitchable(vector<shared_ptr<Patch> > &, const shared_ptr<Patch> &, const map<string, Patch> *, const vector<ConnectInfo> &, const Env &env = Env());
void do_stitch(vector<shared_ptr<Patch> > &, const shared_ptr<Patch> &, const map<string, Patch> *, const vector<ConnectInfo> &, const Env &env = Env(), const bool is_compensation = true);

bool select(shared_ptr<Patch> &, const vector< shared_ptr<Patch> > &, const map<string, Patch> &);
int select(Dangling &, const Danglings &);
int select(const Danglings &);

void get_naive_dangling_table(Danglings &, const vector<shared_ptr<Patch> > &, const Env &);
bool is_stitchable_dangling( const Env &, const Boundary& );
void renew_dangles(Danglings &, const vector<shared_ptr<Patch> > &, const int pat_idx, const vector<ConnectInfo> &, const Env &);

bool stitch_a_patch(vector< shared_ptr<Patch> > &, Danglings &, const int d_idx, const map<string, Patch> &, const char *, const Env &);
bool stitch_patches(vector< shared_ptr<Patch> > &, const size_t pat_n, const map<string,Patch> &, const Env &);
// 모든 Danglings 주변에 붙일 수 있는 한 모든 Patch를 스티칭한다
void stitch_all_patches( vector<shared_ptr<Patch> > &patches, const map<string, Patch> &patch_type, const Env &env );


pair<double, cml::vector3> diff_trans(const cml::vector3 &p1, const cml::vector3 &p2, const Env &env);

double get_distance_rot(const cml::matrix3 &m1, const cml::matrix3 &m2);
double get_distance_rot_fast(const cml::matrix3 &m1, const cml::matrix3 &m2);
map<string, int> get_end_num( const vector<shared_ptr<Patch>> &patches, const Env &env );

void SkeletonPatch(vector<shared_ptr<Patch>> &patches, Env &env, map<string, Patch> * patch_type, vector<Patch> * patch_type_unary );
void delete_patch( vector<shared_ptr<Patch>> &patches, int patch_i, ostream& out );
void path( vector<shared_ptr<Patch>> &patches, Env &env, map<string, Patch> * patch_type, vector<Patch> * patch_type_unary );

struct Stitch_energy
{
  string name;
  int b;
  double energy;
};

//void deterministic( vector<shared_ptr<Patch>> &patches, Env &env, map<string, Patch> * patch_type);
//void deterministic( vector<shared_ptr<Patch>> &patches, Env &env, map<string, Patch> * patch_type, void *energy_func( vector<string> &sampling_patches, map<string, Patch> * patch_type, const int pa2, const int b2, vector<shared_ptr<Patch>> & patches, Env & env, vector<Stitch_energy> &energies ) );
void get_energies_general( vector<string> &sampling_patches, map<string, Patch> * patch_type, const int pa2, const int b2, vector<shared_ptr<Patch>> & patches, Env & env, vector<Stitch_energy> &energies );
void get_energies_path( vector<string> &sampling_patches, map<string, Patch> * patch_type, const int pa2, const int b2, vector<shared_ptr<Patch>> & patches, Env & env, vector<Stitch_energy> &energies );
void deterministic( vector<shared_ptr<Patch>> &patches, Env &env, map<string, Patch> * patch_type, void (*energy_func)( vector<string> &sampling_patches, map<string, Patch> * patch_type, const int pa2, const int b2, vector<shared_ptr<Patch>> & patches, Env & env, vector<Stitch_energy> &energies ) );

void get_energies_path_target( vector<string> &sampling_patches, map<string, Patch> * patch_type, const int pa2, const int b2, vector<shared_ptr<Patch>> & patches, Env & env, vector<Stitch_energy> &energies, cml::vector3 target, cml::vector3 target1 );

bool jitter( const vector<Patch> &patch_type_unary, vector<shared_ptr<Patch>> &patches, const map<string, Patch> * patch_type, Env& env, ostream& out, const pair<int, int> &it, int connect_num);
void get_energies_path_target_fight( vector<string> &sampling_patches, map<string, Patch> * patch_type, const int pa2, const int b2, vector<shared_ptr<Patch>> & patches, Env & env, vector<Stitch_energy> &energies, cml::vector3 target, cml::vector3 target1 );