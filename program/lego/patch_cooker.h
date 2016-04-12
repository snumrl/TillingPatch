// +-------------------------------------------------------------------------
// | patch_cooker.h
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

#include "patch_constructor.h"
#include "interaction_util.h"
#include "ingredient.h"


struct Boundary_group
{
  size_t idx;
  section sect; 
  vector<size_t> pee_idxes;
};

bool operator== (const Boundary_group &a, const Boundary_group &b);
bool operator!= (const Boundary_group &a, const Boundary_group &b);


class PatchCooker
{
 public:  
  PatchCooker(vector<ml::Motion> *motions);
  PatchCooker(Ingredient *ingre);
  ~PatchCooker();

  vector< shared_ptr<Patch> > get_all_patches(); 
  vector<pair_section> relation_pairs();
  
  size_t size_interactions();
  size_t size_mots();

  vector<pair_section> init_interactions(); 

  bool init_EETable(); 

  void agglomerative_cluster(double sim_thres = 3.0, size_t bSize = 6);
  
  void mold_motion_patches(size_t b_size = 6);    
  void mold_interact_patch();
  void auto_covering_interacts(const size_t, vector<Interact_section> &, vector<int> &);
  void mold_non_interact_patch(const vector< vector<Boundary_group> > &);  
  void cook_motion_patches();

  void cook_interact_patches();
  void cook_unary_patches(const vector< vector<Boundary_group> > &);
  void tcook_motion_patches();

  void comb_cook_motion_patches(size_t s= 0);

  void evaluate_this_system();

  ///.................... TEST ............................///
  void Test_molding(size_t num_boundary, size_t num_candi = 10);
  void test_adj_interaction_idx();   
  ///////////////////////////////////////////////////////////

  // to be erased //////////////////////////////////////////////////////////////
  void set_cross_sects(const size_t pos, const ml::Motion &m);
  void evaluate_cross_sects_given_pose();
  void print_evaluation()	{ for (auto it = feature_grades.begin(); it != feature_grades.end(); ++it) cout << it->first << ": " << it->second << endl;}
  void print_max_cross_sec(){ eeTable->print_cross_sect(cross_sect_features[feature_grades[0].first]);}

  pair_section direct_covering_pair_section(const pair_section &pair_sect, const size_t contact_range_befr, const size_t contact_range_aftr);
  vector<section> direct_covering_interacts(const Interaction &inter, const size_t contact_range_befr, const size_t contact_range_aftr);  
  section covering_section(const section &s, const size_t contact_range_befr, const size_t contact_range_aftr);
  void mold_patch_edges(const vector<pair_section> &pair_sects);

  void sort_candi_poses() {sort(feature_grades.begin(), feature_grades.end(), [](pair<size_t,double> a, pair<size_t,double> b)->bool{ return (a.second > b.second) ? true : false;});}  

  vector<PosturePoints> cross_sect_features;	  // to be erased!
  vector< pair<size_t,double> > feature_grades;  

  double eval_molding_boundary_given_poses(size_t f_idx, const PatchEntryExit &bt);
  ////////////////////////////////////////////////////////////////////////////// 

 public:
  InteractionFinder *interaction_prober;
  vector<string> sampling_patches;
  map<string, Patch> *patches;
  vector<Patch> *unary_patches;
  EntryExitTable *eeTable;

  vector<ml::Motion> *ref_mots;
  Ingredient * ingredients;

 private:
  size_t adj_interaction_idx(const EEGroup *);
  Interact_section auto_covering_section(const EEGroup *, const EEGroup *);
  int max_feature_pos( section& bound, const EEGroup *leg, bool isentry );
  double evaluate_boundary_by_cluster(const PatchEntryExit& bpee);  

  void select_most_distributed_groups(vector< vector< vector<size_t> > > &, vector< pair<double,size_t> > &, size_t size_); 
  
  void select_best_combinations_of_groups(vector< vector< vector<size_t> > > &out_groups, vector<int> &out_combi, vector<double> &eval, size_t candi_size);

  double evaluate_distribution(const vector< vector<size_t> > &);
  void store_pose_info_by_selected_group(const vector< vector< vector<size_t> > > &, const vector<size_t> &);  
   
  vector< vector<Boundary_group> > most_distributed_select_poses();
  vector< vector<Boundary_group> > combi_distribution_of_select_poses(size_t sel);

  void cook_a_patch(const vector<section> &, const string &);   
  void cook_a_patch(const vector<Interact_section> &, const string &);
  void cook_a_patch(const Interact_section &, const string &);
  vector< vector<Boundary_group> > convert_sorting_criteria_from_groupidx_to_motionidx(const vector< vector< vector<size_t> > > &, const vector<size_t> &);

  void remove_similar_non_interact_patch( vector<Interact_section> & );
  void remove_timid_patch( vector<Interact_section> & );
  void set_boundary_type_size( size_t btype );  
  void set_boundary_type( const vector<size_t> &eval_selected_group_distri ); 
  
  void get_optimal_combinations_of_group( vector<int> &out_combi, vector<double> &eval, const vector< vector< vector<size_t> > > &tree_idx_groups, size_t candi_size );
  double get_optimal_combination( vector<int> &comb, size_t n, size_t r, const vector< vector< vector<size_t> > > &out_groups );
  double evaluate_combinational_boundaries( const vector< vector <vector<size_t> > > &comb_groups );
  
 private:       
  vector< vector<Interact_section> > raw_interact_patch;
  vector<Interact_section> raw_non_interact_patch;
  size_t boundaryTypeSize;
  vector<size_t> boundary_type;

  vector<size_t> original_candidate_group_idxes;


  // to be erased
  vector<pair_section> molding_bounds;
  ///////////////////////////////////////////////
};
