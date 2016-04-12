// +-------------------------------------------------------------------------
// | cook_util.h
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

#include "motion_util.h"
#include <vector>
#include "ingredient.h"

using std::vector;

const double Pose_Similarity = 1.8;


struct PosturePoints
{
  PosturePoints(){}
  PosturePoints(const ml::Motion &m, const size_t pos);
  vector<cml::vector3> joints;
  vector<cml::vector3> velocitys;
  cml::matrix3 orient;

  bool contact_foot[2];
  
  bool is_forwarding;

  cml::vector3& operator [] (size_t i) {return joints[i];}
  const cml::vector3 &operator [] (size_t i) const {return joints[i];}
  PosturePoints& operator*= (const int ratio) {for_each(joints.begin(), joints.end(), [ratio](cml::vector3 &j){j *= ratio;}); return *this;}
  
  size_t size() const {return joints.size();}
  void transl(const cml::vector3 &t) {for_each(joints.begin(), joints.end(), [t](cml::vector3 &j){j += t;});}
  void rotate(const cml::vector3 &r) {cml::matrix3 rot = cml::exp_mat3(r); for_each(joints.begin(), joints.end(), [rot](cml::vector3 &j){j = rot * j;}); orient = rot * orient;}
  void rotate(const cml::matrix3 &r) {for_each(joints.begin(), joints.end(), [r](cml::vector3 &j){j = r * j;}); orient = r * orient;}
};

PosturePoints operator* (const int r, PosturePoints & pc); 
PosturePoints operator* (PosturePoints & pc, const int r); 

double pose_difference(const PosturePoints &a, const PosturePoints &b);
double pose_weighted_difference(const PosturePoints &a, const PosturePoints &b, const vector<double> &weights);
double weighted_distance(const PosturePoints &a, const PosturePoints &b, const vector<double> &weights);
void align_points_to_other( PosturePoints & from, const PosturePoints & to );

bool is_same_contact_state(const PosturePoints& a, const PosturePoints& b);


struct PointsCloud
{
  PointsCloud(const ml::Motion &, const size_t, const size_t );
  
  PosturePoints& operator[] (size_t idx) {return points_cloud[idx];}
  const PosturePoints& operator[] (size_t idx) const {return points_cloud[idx];}
  
  size_t size_cloud() const {return points_cloud.size();}  

  vector<PosturePoints> points_cloud;
  vector<double> weights;
};

double distance_Kovar_metric(const PointsCloud &a, const PointsCloud &b);


class PatchEntryExit
{
  PatchEntryExit(){}

 public:
  PatchEntryExit(size_t m, size_t p, size_t dist, vector<ml::Motion> *);
  PatchEntryExit(size_t b, size_t m, size_t p, size_t dist, Ingredient *);

  const ml::Motion &get_motion() const;
  ml::Motion::Const_iterator it_posture() const;
  ml::Motion::Iterator it_posture();
  
  bool is_included_in_ref_mots;

  size_t get_mot_idx() const {if (ref_m != nullptr) return mot_pos_idx.first; else return ref_ingre->map_bundle_to_mot(pair<size_t,size_t>(bundle_mot_pos_idx.first,bundle_mot_pos_idx.second.first)); };
  size_t get_bundle_idx() const {if (ref_ingre != nullptr) return bundle_mot_pos_idx.first; else return 0;}
  size_t get_bundle_mot_idx() const {if (ref_ingre != nullptr) return bundle_mot_pos_idx.second.first; else return 0;}
  size_t get_pos_idx() const {if (ref_m != nullptr) return mot_pos_idx.second; else return bundle_mot_pos_idx.second.second;};    

  size_t distance_from_interact;
  struct feature {
	int cluster_group_idx;		// the range is -1 ~ n.
	double pose_distance;
  } f_info;

  PosturePoints get_origin_pcloud() const { return PosturePoints(get_motion(), get_pos_idx());}  
  PosturePoints& get_pcloud() { return pcloud;}
  const PosturePoints& get_pcloud() const { return pcloud;}

 private:
   vector<ml::Motion> *ref_m;
   Ingredient * ref_ingre;
   PosturePoints pcloud;

   pair<size_t,size_t> mot_pos_idx;  
   pair< size_t,pair<size_t,size_t> > bundle_mot_pos_idx;
};

void transl_to_origin(PatchEntryExit & pee);
void align_points_to_other(PatchEntryExit & from, const PatchEntryExit & to);
double shape_difference(const PatchEntryExit & e1, const PatchEntryExit & e2);
double shape_difference(const PosturePoints &p, const PatchEntryExit & pee);


struct EEGroup
{
  enum Type {Entry, Exit};

  vector<PatchEntryExit> entry_exits;
  Type type;  
  size_t interaction_idx;
  size_t leg_idx;

  size_t get_i_idx() const {return interaction_idx;}
  Type	 get_type() const {return type;}
  size_t get_mot_idx() const {if (entry_exits.empty()) return 0; else return entry_exits[0].get_mot_idx();}
  size_t size() const {return entry_exits.size();}
  bool is_containing(size_t midx, pair<size_t,size_t> sect);

  vector<PatchEntryExit>::iterator begin() {return entry_exits.begin();}
  vector<PatchEntryExit>::iterator end() {return entry_exits.end();}
  vector<PatchEntryExit>::const_iterator begin() const {return entry_exits.begin();}
  vector<PatchEntryExit>::const_iterator end() const {return entry_exits.end();}

  void push(const PatchEntryExit &pee) {entry_exits.push_back(pee);}
};

void view_similar_frame(EEGroup &eeg, const PosturePoints &p);


struct Node 
{
  Node(Node * p = nullptr): parent(p), size(0){}
  virtual ~Node(){}

  Node *parent;
  size_t size;
  virtual PosturePoints get_representative() = 0;
  virtual vector<size_t> get_generations_idx() const = 0;
  virtual vector<Node *> get_siblings() const = 0;
  virtual double get_value() const = 0;
};

Node* get_root(Node* t); 
void get_group_member(vector<size_t> &g, Node *root);

struct Terminal : public Node 
{
  Terminal(size_t g, size_t e, PatchEntryExit * pp, Node * p = nullptr): Node(p), ptr_pee(pp), ee_idx(e), group_idx(g) {size = 1;}
  ~Terminal() {}
  PosturePoints get_representative() {return ptr_pee->get_origin_pcloud();}
  vector<size_t> get_generations_idx() const {return vector<size_t>(1, n_th);}
  double get_value() const {return 100000.0;}
  vector<Node *> get_siblings() const {return vector<Node*>(1, nullptr);}

  size_t group_idx;
  size_t ee_idx;
  size_t n_th;
  PatchEntryExit * ptr_pee;
};

struct Clustering : public Node 
{
  ~Clustering() { for(auto it = siblings.begin(); it != siblings.end(); ++it) delete *it;}

  vector<Node *> siblings;  
  double max_dist;  
  PosturePoints mean;  

  PosturePoints get_representative() {return mean;}
  vector<size_t> get_generations_idx() const {
	vector<size_t> ret; 
	for (auto it = siblings.begin(); it != siblings.end(); ++it) {
	  auto siblings_idx = (*it)->get_generations_idx();
	  copy(siblings_idx.begin(), siblings_idx.end(), back_inserter(ret));
	} return ret;
  }
  vector<Node *> get_siblings() const { return siblings;}
  double get_value() const {return max_dist;}
};


struct EntryExitTable 
{
  EntryExitTable(size_t size_): root_of_clustering(nullptr), mot_size(size_) {}
  ~EntryExitTable() {if (root_of_clustering != nullptr) delete root_of_clustering;}

  size_t size_mot() const {return mot_size;}
  size_t size_candidates() const;
  size_t size_clusters() const {return clustered_tree_leaf_indexes.size();}
  size_t size_cluster_group(size_t idx) const {return clustered_tree_leaf_indexes[idx].size();}

  void clear() {boundary_group.clear(); if (root_of_clustering != nullptr) delete root_of_clustering;}
  void reserve_mem(size_t size_inters) {boundary_group.reserve(2 * size_inters + 1);}

  void evaluate_all_legs_by_cluster();
  
  void store_infos_in_pees(const vector< vector<size_t> > &, const size_t group_idx);
  void store_highlight_in_pees( const vector< vector<size_t> > &clustered_leaf_nodes_by_mot_idx );
  void store_cluster_group_in_pees(const vector< vector<size_t> > &, const int);
  void store_shape_dist_in_pees(const vector< vector<size_t> > &, const size_t);

  void agglomerative_tree_construction();
  void print_tree();
  void group_cluster_tree_by_dist(const double distance, const size_t boundary_size);
  void get_cluster_group_distribution(vector< vector<size_t> > &, const size_t g_idx);  

  PatchEntryExit& get_boundary(size_t idx);
  const PatchEntryExit& get_boundary(size_t idx) const;
  void set_boundary_group(const EEGroup &eeg) {boundary_group.push_back(eeg);}
  EEGroup* get_last_ptr_boundary_group() {return &boundary_group.back();}
  Terminal* get_cluster_tree_leaf_node(size_t idx) {return cluster_tree_leaf_node[idx];}
  const Terminal* get_cluster_tree_leaf_node(size_t idx) const {return cluster_tree_leaf_node[idx];}
  PatchEntryExit* get_cluster_tree_leaf_pee(size_t idx) {return cluster_tree_leaf_node[idx]->ptr_pee;}
  const PatchEntryExit* get_cluster_tree_leaf_pee(size_t idx) const {return cluster_tree_leaf_node[idx]->ptr_pee;}

  void test_pee_init();

  // to be erased //////////////////////////////////////////////////////////////////////////////////
  double evaluate_cross_sect_given_pose(const size_t p_idx, const PosturePoints &p, const size_t interaction_num);
  void print_cross_sect(const PosturePoints &pcloud);
  //////////////////////////////////////////////////////////////////////////////////////////////////

  vector<EEGroup> boundary_group;

  vector< vector<size_t> > clustered_tree_leaf_indexes;

  double evaluate_distance_in_group(const PatchEntryExit &, const size_t &);

private:
  size_t mot_size;
  Clustering * root_of_clustering;
  
  vector<Terminal*> cluster_tree_leaf_node;
  
  void transl_to_origin(const size_t b, const size_t pee);
  void transl_to_origin(const size_t);

  Clustering * build_parent_tree(pair<size_t,size_t> idx, vector< Terminal* > &ctree, vector< vector<double> > &distM);
  double max_distance_cluster(const vector<size_t> &group1, const vector<size_t> &group2, const vector< vector<double> > &distM);
  void update_distance_mat( vector< vector<double> > &distM, const vector<size_t> &g, vector< Terminal* > & ctree );
  void replace_distance_mat(vector< vector<double> > &dM, const vector<size_t> &g1, const vector<size_t> &g2, const double val);
  void preorder(vector< vector<size_t> > &, const Node *, const double thres);

  pair<size_t,double> get_proximately_group(const PatchEntryExit &);
  void print_node(Node * node);
  void print_cluster_tree(const vector< vector<size_t> > &groups, const size_t boundary_size);
  void print_minor_tree(const vector< vector<size_t> > &clustered_tree_leaf_indexes );  
};

