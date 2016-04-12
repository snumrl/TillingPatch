// +-------------------------------------------------------------------------
// | ingredient.h
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

#include "motion/ml.h"
#include <vector>


class Ingredient
{ 
  vector<ml::Motion> raw_motions;

 public:
  typedef std::vector<ml::Motion> in_motions; 
  vector<in_motions> raw_before_cook;
  
  map< size_t, pair<size_t,size_t> > interact_bunch_to_bunble;
  map< pair<size_t,size_t>, size_t > interact_bunble_to_bunch;

  void push_in_basket(const in_motions & im) {raw_before_cook.push_back(im); copy(im.begin(), im.end(), back_inserter(raw_motions));}  

  size_t size_mot() const;
  size_t size_pos() const;
  size_t bunch_size() const {return raw_before_cook.size();}
  pair<size_t,size_t> map_mot_to_bundle( size_t seq_mot_idx ) const;
  size_t map_bundle_to_mot(pair<size_t,size_t> bundle) const;
  vector<ml::Motion> * get_ptr_total_motions() {return &raw_motions;}
};

void mix_synthesized_data(Ingredient &ingre);
void mix_raw_data(Ingredient &ingre);
