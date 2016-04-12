// +-------------------------------------------------------------------------
// | interaction_util.h
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

#include "cook_util.h"


struct ABB {
  ABB();

  cml::vector3 origin;
  double length[3]; 
};

bool intersect_abb(const ABB &a, const ABB &b);
ABB bound_abb_posture(const ml::Posture &);
cml::vector3 virtual_geo_center(const ml::Posture &pos1);
double face_diff(const ml::Posture &, const ml::Posture &);

int length_apart(const pair<size_t,size_t> &a, const pair<size_t,size_t> &b);


struct section
{
  section(): pos_sec(pair<size_t,size_t>(0,0)), mot_idx(0), is_extend(pair<bool,bool>(false, false)){}
  section(size_t m, pair<size_t,size_t> s): mot_idx(m), pos_sec(s), is_extend(pair<bool,bool>(false, false)){}
  
  size_t begin_pos() const {return pos_sec.first;}
  size_t end_pos()	const {return pos_sec.second;}
  size_t motion_idx() const {return mot_idx;}

  pair<size_t,size_t> pos_sec;
  size_t mot_idx;
  
  pair<bool,bool> is_extend;
};

size_t length_section(const pair<size_t,size_t> &sect);
int length_apart(const section &, const section &);
pair<size_t,size_t> overlap(const pair<size_t,size_t> a, const pair<size_t,size_t> b);
bool overlap_secs(section& ret, const section a, const section b);


struct Interact_section : public section
{
  Interact_section(): section(), boundary_type(pair<size_t,size_t>(0, 0)){}
  Interact_section(section s): section(s), boundary_type(pair<size_t,size_t>(0, 0)){}

  int entry_type() const {return boundary_type.first;}
  int exit_type() const {return boundary_type.second;}
  void set(size_t ent, size_t ext) { boundary_type.first = ent; boundary_type.second = ext; }

  pair<int, int> boundary_type;
};

void fill_interact_section_gap(vector<Interact_section> &, const int);
int length_apart(const Interact_section&, const Interact_section&);
bool overlap_interact_secs(Interact_section&, const Interact_section &, const Interact_section &);
bool is_similar(const Interact_section &a, const Interact_section &b);


struct pair_section
{  
  enum Types{Default, Contact, Intimate, Sync};

  pair_section(): type(Default), bunch_idx(-1){ }
  pair_section(size_t m1, size_t m2, pair<size_t,size_t> s1, pair<size_t,size_t> s2); 

  void set_motion_idx(size_t i, size_t idx) {psec[i].mot_idx = idx;}
  void set_pos_sec(size_t i, pair<size_t,size_t> s) {psec[i].pos_sec = s;}
  void set_pos_sec_beg(size_t i, size_t b) {psec[i].pos_sec.first = b;}
  void set_pos_sec_end(size_t i, size_t e) {psec[i].pos_sec.second= e;}

  size_t get_motion_idx(size_t i) const	  {return psec[i].motion_idx();}
  pair<size_t,size_t> get_section(size_t i) const {return psec[i].pos_sec;}
  size_t get_section_beg(size_t i) const  {return psec[i].begin_pos();}
  size_t get_section_end(size_t i) const  {return psec[i].end_pos();}

  Types type;
  section psec[2];
  int bunch_idx;
};

int length_apart(const pair_section &, const pair_section &);
bool is_same_sections(const pair_section &, const pair_section &);
void divide_overlapping_sect_half(pair_section &, pair_section &);
bool merge(pair_section &sec1, const pair_section &sec2, const size_t thres_length);
bool overlap_both_pairsects(pair_section &, const pair_section &);
void fill_pair_section_gap(vector<pair_section> &secs, const int length);
void insert_sections(vector<pair_section> &msect, const vector<pair_section> &sec1, const vector<pair_section> &sec2);
void check_section(vector<ml::Motion> *mots, const vector<pair_section> &col_sec, const cml::vector3 &color=cml::vector3(0.1, 0.1, 0.6));


class Interaction 
{
public:
  Interaction(){}
  ~Interaction(){}

  size_t size() const {return interacts.size();}
  vector<section>::iterator begin() {return interacts.begin();}
  vector<section>::iterator end() {return interacts.end();}
  vector<section>::const_iterator begin() const {return interacts.begin();}
  vector<section>::const_iterator end() const {return interacts.end();}

  void set_relation(const pair_section &ms) {relations.push_back(ms);}
  void set_single_relation(const section &s) {interacts.push_back(s);}
  void set_leg(EEGroup *eeg) {leg_ptrs.push_back(eeg);}
  EEGroup* get_leg(size_t i, bool is_entry) const {return (is_entry) ? leg_ptrs[2 * i] : leg_ptrs[2 * i + 1];}
  void normalize();

  vector<pair_section> relations;
  vector<EEGroup*> leg_ptrs;

private:   
  vector<section> interacts;  
};


bool merge(Interaction &int1, const Interaction &int2, const size_t thres_length);
size_t size(const vector<Interaction> &inters);

double min_distance(const ml::Posture &pos1, const ml::Posture &pos2);

bool gather_pre_neighborhood_of_mots_interact(Interaction &, EntryExitTable&, const size_t inter_idx, const size_t inner_idx, const size_t sect_size, vector<ml::Motion> &);
bool gather_post_neighborhood_of_mots_interact(Interaction &, EntryExitTable&, const size_t inter_idx, const size_t inner_idx, const size_t sect_size, vector<ml::Motion> &);
bool gather_pre_neighborhood_of_ingrient_interact(Interaction &, EntryExitTable&, const size_t inter_idx, const size_t inner_idx, const size_t sect_size, Ingredient &);
bool gather_post_neighborhood_of_ingrient_interact(Interaction &, EntryExitTable&, const size_t inter_idx, const size_t inner_idx, const size_t sect_size, Ingredient &);