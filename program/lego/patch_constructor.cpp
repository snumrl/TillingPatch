#include "StdAfx.h"
#include "patch_constructor.h"



bool Patch_constructor::realize_motion_data()
{
  if (characters_mots.empty()) {
	cerr << "Error: there is no information of character!" << endl;
	return false;
  } else {
	vector<section> patch_including_legs;
	if (characters_mots.size() == 1) {
	  patch_including_legs.push_back(characters_mots[0]);
	  
	  auto pt = patch_including_legs.begin();
	  const auto &ref = get_motion(pt->motion_idx());
	  ml::Motion mot;
	  mot.copy(ref, ref.begin() + pt->begin_pos(), ref.begin() + pt->end_pos() + 1);	  
	  patch->motions.push_back(mot);
	} else {
	  sort(characters_mots.begin(), characters_mots.end(), 
		[](section &a, section &b)->bool{if (a.motion_idx() < b.motion_idx()) return true; else return false;});

	  auto pre_it = characters_mots.begin();
	  size_t pre_idx = characters_mots.begin()->motion_idx();
	  auto jt = characters_mots.begin() + 1;
	  while (jt != characters_mots.end()) {
		if (jt->motion_idx() != pre_idx) {
		  size_t min_p = 100000, max_p = 0;
		  for (auto kt = pre_it; kt != jt; ++kt) {
			if (kt->begin_pos() < min_p) min_p = kt->begin_pos();
			if (kt->end_pos() > max_p) max_p = kt->end_pos();
		  }
		  patch_including_legs.push_back(section(pre_it->motion_idx(), pair<size_t, size_t>(min_p, max_p)));
		  pre_idx = jt->motion_idx();
		  pre_it = jt;
		} 		
		++jt;
	  } { // push last legs. 
		size_t min_p = 100000, max_p = 0;
		for (auto kt = pre_it; kt != jt; ++kt) {
		  if (kt->begin_pos() < min_p) min_p = kt->begin_pos();
		  if (kt->end_pos() > max_p) max_p = kt->end_pos();
		}
		patch_including_legs.push_back(section(pre_it->motion_idx(), pair<size_t, size_t>(min_p, max_p)));
	  }
	  for (auto pt = patch_including_legs.begin(); pt != patch_including_legs.end(); ++pt) {
		const auto &ref = get_motion(pt->motion_idx());
		ml::Motion mot;
		mot.copy(ref, ref.begin() + pt->begin_pos(), ref.begin() + pt->end_pos() + 1);
		patch->motions.push_back(mot);
	  }	  
	}	
	patch->translate_time(-1 * patch->get_begin_time());
	patch->translate_origin();

	return true;
  }  
}

void Patch_constructor::set_interact_mot( const Interact_section &isec )
{
  /*cout << "Listed entry type is " << isec.entry_type() << " VS " << get_motion(isec.motion_idx()).posture(isec.begin_pos()).type_ << endl;
  cout << "Listed exit type is " << isec.exit_type() << " VS " << get_motion(isec.motion_idx()).posture(isec.end_pos()).type_ << endl;*/
  set_relation(section(isec.motion_idx(), isec.pos_sec));
}

void Patch_constructor::set_constraint()
{
  if (characters_mots.size() < 2) return;

  vector<section> interact_sects;
  find_interact_sects(interact_sects);

  int** relate_table = new int* [interact_sects.size()]; // row
  for (size_t i = 0; i <interact_sects.size(); ++i) {
	relate_table[i] = new int [interact_sects.size()];
  }
  for (size_t r = 0; r < interact_sects.size(); ++r) {
	for (size_t c = 0; c < interact_sects.size(); ++c) {
	  relate_table[r][c] = 0;
	}
  }

  mark_relate_interacts(relate_table, interact_sects);

  for (size_t i = 0; i < interact_sects.size(); ++i) {
	delete [] relate_table[i];
  } delete [] relate_table;
}

void Patch_constructor::find_interact_sects( vector<section> &interact_sects )
{
  for (auto jt = patch->motions.begin(); jt != patch->motions.end(); ++jt) {
	size_t s = 0, e = 0;
	size_t p = 0;
	bool pre_state = jt->posture(p++).is_interact;
	if (pre_state) s = p;

	for (; p < jt->size(); ++p) {
	  if (jt->posture(p).is_interact != pre_state) {
		if (pre_state == false) {
		  s = p;
		  pre_state = true;
		} else {
		  e = p - 1;
		  section temp(jt - patch->motions.begin(), pair<size_t,size_t>(s, e));
		  interact_sects.push_back(temp);
		  pre_state = false;
		}		 
	  }
	}
  }  
}

void Patch_constructor::mark_relate_interacts( int** relate_table, const vector<section> &interact_sects )
{
  for (size_t i = 0; i < (interact_sects.size() - 1); ++i) {
	for (size_t j = i + 1; j < interact_sects.size(); ++j) {
	  if (is_related_and_set_cons(interact_sects[i], interact_sects[j])) {
		relate_table[i][j] = relate_table[j][i] = 1;
	  }
	}
  }
}

bool Patch_constructor::is_related_and_set_cons( const section &a, const section &b )
{
  double a_s_time = patch->motions[a.motion_idx()][a.begin_pos()].time; 
  double b_s_time =	patch->motions[b.motion_idx()][b.begin_pos()].time;  
  double a_e_time =	patch->motions[a.motion_idx()][a.end_pos()].time;   
  double b_e_time =	patch->motions[b.motion_idx()][b.end_pos()].time;   

  if (a_s_time > b_e_time || a_e_time < b_s_time) return false;
  
  double f = (a_s_time > b_s_time) ? a_s_time : b_s_time;
  double e = (a_e_time < b_e_time) ? a_e_time : b_e_time;
  
  auto a_f_it =	patch->motions[a.motion_idx()].iterator_at(f); 
  auto b_f_it =	patch->motions[b.motion_idx()].iterator_at(f); 
  auto a_e_it =	patch->motions[a.motion_idx()].iterator_at(e); 
  auto b_e_it =	patch->motions[b.motion_idx()].iterator_at(e); 

  size_t min_d = (a_e_it - a_f_it < b_e_it - b_f_it) ? (a_e_it - a_f_it) : (b_e_it - b_f_it);
  double max_dist = 0.0;
  size_t max_i = 0;
  for (size_t i = 0; i < min_d; ++i) {
	double dist = spatio_intimate_val(*(a_f_it + i), *(b_f_it + i));
	if (dist > max_dist) {
	  max_dist = dist;
	  max_i = i;
	}
  }
  
  size_t a_s_cons = a_f_it - patch->motions[a.motion_idx()].begin();
	size_t a_e_cons = a_e_it - patch->motions[a.motion_idx()].begin();
	size_t b_s_cons = b_f_it - patch->motions[b.motion_idx()].begin();
	size_t b_e_cons = b_e_it - patch->motions[b.motion_idx()].begin();
	size_t a_max_cons = a_s_cons + max_i;
	size_t b_max_cons = b_s_cons + max_i;

	add_cons_rel_pos_same_time(patch->inner_cons, patch->motions, a.motion_idx(), a_max_cons, b.motion_idx(), b_max_cons);	

	size_t a_min_i = a_s_cons, b_min_i = b_s_cons;	
	if (a_max_cons == a_min_i) {
	  a_min_i =  (a_e_cons + a_s_cons) / 2;
	}
	if (b_max_cons == b_min_i) {
	  b_min_i = (b_e_cons + b_s_cons) / 2;
	}	 
	add_cons_rel_pos_same_time(patch->inner_cons, patch->motions, a.motion_idx(), a_min_i, b.motion_idx(), b_min_i);

	add_cons_rel_pos_same_time(patch->inner_cons, patch->motions, a.motion_idx(), a_e_cons, b.motion_idx(), b_e_cons);

	// check for visualizing when drawing Patch-Viewer
	patch->rel_pos_time.push_back(pair_section(a.motion_idx(), b.motion_idx(), pair<size_t, size_t>(a_max_cons, a_max_cons), pair<size_t,size_t>(b_max_cons, b_max_cons)));
	patch->rel_pos_time.push_back(pair_section(a.motion_idx(), b.motion_idx(), pair<size_t, size_t>(a_min_i, a_min_i), pair<size_t,size_t>(b_min_i, b_min_i)));
	patch->rel_pos_time.push_back(pair_section(a.motion_idx(), b.motion_idx(), pair<size_t, size_t>(a_e_cons, a_e_cons), pair<size_t,size_t>(b_e_cons, b_e_cons)));

	return true;
}

ml::Motion & Patch_constructor::get_motion( size_t i ) 
{
  if (ref_mots != nullptr) {
	return ref_mots->at(i);
  } else {	
	return ref_ingre->raw_before_cook[ref_ingre->map_mot_to_bundle(i).first][ref_ingre->map_mot_to_bundle(i).second];
  }
}

