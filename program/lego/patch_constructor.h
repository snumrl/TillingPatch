// +-------------------------------------------------------------------------
// | patch_constructor.h
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
#include "find_interaction.h"


class Patch_constructor
{
  Patch_constructor() {}

 public:
  Patch_constructor(vector<ml::Motion> *mots): ref_mots(mots), ref_ingre(nullptr), is_faulty(false), patch(new Patch){}
  Patch_constructor(Ingredient *ingre): ref_ingre(ingre), ref_mots(nullptr), is_faulty(false), patch(new Patch){}
  ~Patch_constructor() { delete patch; }

  vector<section> characters_mots;

  Patch * get_patch() {return patch;}
  bool realize_motion_data();
  
  void set_name(const string& n) {patch->name = n;}
  void set_interact_mot(const Interact_section &);
  void set_relation(const section &sec) {characters_mots.push_back(sec);}
  void set_constraint();  
  
  void find_interact_sects( vector<section> & );
  void mark_relate_interacts( int** , const vector<section> & );
  bool is_related_and_set_cons( const section &, const section & );

 private:
  bool is_faulty;
  vector<ml::Motion> *ref_mots;
  Ingredient * ref_ingre;
  Patch * patch;

  ml::Motion & get_motion(size_t i); 
};


