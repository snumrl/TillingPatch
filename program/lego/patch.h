// +-------------------------------------------------------------------------
// | patch.h
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

#include "util/ccml.h"
//#include "motion_util.h"
#include "motion/ml.h"
#include "motion_edit.h"
#include "interaction_util.h"


struct PatchObject 
{
  PatchObject() { objectType = kNone; range = 5; }
  int begin_motion;
  int begin_frame;

  int end_motion;
  int end_frame;
  int range;

  enum { kNone, kPatchBox, kBoxBlending, kPrevBox} objectType;
};


struct Boundary
{
  bool forward;
  int motion_index;
  ml::Motion* p_motion;
  size_t patch_idx;
  set<string> stitched_unary;
  set<string> jittering_stitched_unary;
  set<pair<string, int>> stitch_history;
  set<pair<string, int>> stitch_history_jitter;
  
  ml::Posture& posture();
  const ml::Posture& posture() const;
  ml::Motion *connected_motion() const; 
  bool is_connected() const {return (connected_motion() != nullptr);}
  int posture_type() const  {return posture().type_;}
};


class Patch
{
 public:
  Patch(){ }
  Patch(const Patch &other);
  Patch( const Patch &other, bool only_boundary );

  virtual ~Patch(){ }
  double get_begin_time() const;
  double get_last_time() const;
  
  void translate(const cml::vector3 &v);
  void translate_origin();
  void rotate(double theta);
  void translate_time(double time);

  void add_motion(const ml::Motion &m);
  void set_boundaries();
  int get_boundary_type(size_t idx);

  void transform_between_posture(const ml::Posture &to_posture, const ml::Posture &from_posture);
  void add_motion_only_boundary( const ml::Motion &m );
  bool checked;
  string name;
  string log;
  
  vector<ml::Motion> motions;
  vector<Boundary> boundaries;
  vector<Constraint> inner_cons;
  
  PatchObject patch_object;	
  vector<pair_section> rel_pos_time;

 protected:
  void apply_transf(const cml::transf &t, double time = 0.0);
};


string write_patches( const vector<shared_ptr<Patch>> & patches, const string &name = string());
void read_patches( vector<shared_ptr<Patch>> & patches, const string &name );
void remove_dangling_patch( vector<shared_ptr<Patch> > & cooked_patches, const vector<string> &deathNote = vector<string>() );

double begin_time(const vector< shared_ptr<Patch> > &patches);
double end_time(const vector< shared_ptr<Patch> > &patches);
map<string, int> get_end_num(const vector<shared_ptr<Patch>> &patches, double t_min = 100, double t_max = 700);
map<string, int> get_end_num(const shared_ptr<Patch> &patch, double t_min = 100, double t_max = 700);

std::string get_end_type( Boundary & b, double t_min, double t_max );
void set_color(vector< shared_ptr<Patch> > &patches);

void copy_patches(vector< shared_ptr<Patch> > &to, const vector< shared_ptr<Patch> > &from);
pair<int, int> connected_patch_motion(const ml::Motion * p_motion, const vector< shared_ptr<Patch> > &patches);
pair<int, int> connected_patch_boundary(const Boundary &b1, const vector< shared_ptr<Patch> > &patches);
vector<double> edit_degree(const Patch &pa, const Patch &pa_ori);

bool edit_ok(const Patch &pa, const map<string, Patch> * patch_type);
void convert_connected_motions(const vector<shared_ptr<Patch>> & patches, vector<ml::Motion> & motions, vector<Constraint> &inner_cons);

size_t find_boundary_idx(const shared_ptr<Patch> patch, const int motion_index, const bool forw_dir);
void convert_patch_to_motion(vector<ml::Motion> &motions, const vector< shared_ptr<Patch> > &patches);
void set_color_twopart( vector<shared_ptr<Patch>> &patches );
void set_color_from_back( vector<shared_ptr<Patch>> &patches );