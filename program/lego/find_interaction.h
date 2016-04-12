// +-------------------------------------------------------------------------
// | find_interaction.h
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
#include "motion_util.h"
#include "interaction_util.h"
#include "ingredient.h"


class InteractionFinder
{
  vector<ml::Motion> *ref_mots;
  Ingredient *ingre;
  
  vector<pair_section> process_bunch(const vector<ml::Motion> *, const size_t trim = 10);
  vector<pair_section> process_bundle(Ingredient *);

 public:

   vector<Interaction> interactions;

   bool mutual_exclusive_test() const;
   size_t size() const {return interactions.size();}
   const Interaction &get(size_t i) const;
   Interaction &get(size_t i);

   InteractionFinder(vector<ml::Motion> *ref_) :ref_mots(ref_), ingre(nullptr) { }
   InteractionFinder(Ingredient * ingre_): ingre(ingre_), ref_mots(nullptr) { }
   ~InteractionFinder(){}
   
   vector<pair_section> process();
   
};


void find_interactions(vector<pair_section> &inters, const vector<ml::Motion> &motions, const size_t trim_edge);

void check_interactions(vector<ml::Motion> *motions, const vector<pair_section> &relate_pairs);
void check_interactions(vector<ml::Motion> *motions, const vector<Interaction> &inters);
void check_interactions(Ingredient *, const vector<Interaction> &);

vector<pair_section> likely_collisions(const vector<ml::Motion> *);
void likely_collisions(vector<pair_section> &, const size_t mot1, const size_t mot2, const vector<ml::Motion> *);
bool collision_likely_posture(const ml::Posture &pos1, const ml::Posture &pos2);

vector<pair_section> exact_collisions(const vector<ml::Motion> *);
void exact_collisions(vector<pair_section> &secs, size_t i, size_t j, const vector<ml::Motion> *);
void unite_sections_within_contact(vector<pair_section> &, const size_t, const vector<ml::Motion> *);

vector<pair_section> intimate_characters(const vector<ml::Motion> *);
void intimate_characters(vector<pair_section> &, const size_t mot1, const size_t mot2, const vector<ml::Motion> *);
bool is_spatio_intimate(const ml::Posture &pos1, const ml::Posture &pos2);
double spatio_intimate_val(const ml::Posture &pos1, const ml::Posture &pos2);
double all_comb_joints_distance(const ml::Posture &, const ml::Posture &);

void strong_interaction(vector<pair_section> &sec, const vector<ml::Motion> *mots);

void synchronized_interaction(vector<pair_section> &, const vector<ml::Motion> *);
bool synchronized_action(vector<pair_section> &, const size_t, const size_t, const vector<ml::Motion> *, const vector< vector<double> > &);
bool is_sync_interaction(const ml::Posture &, const ml::Posture &, const double , const double );

void erase_timid_interaction(vector<pair_section> &secs, const size_t length = 3);

bool gather_boundary(vector<Interaction> &, EntryExitTable&, vector<ml::Motion> &, const size_t size_sect = 30);
bool gather_boundary(vector<Interaction> &, EntryExitTable &, Ingredient&, const size_t sect = 30);
