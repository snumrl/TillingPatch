#include "StdAfx.h"
#include "interaction_util.h"


ABB::ABB()
{
  length[0] = length[1] = length[2] = 0.0;
}

bool intersect_abb(const ABB &a, const ABB &b)
{
  double x[2] = {0., 0.};
  double l = 0.0;

  for (size_t i=0; i<3; ++i) {
	if (a.origin[i] == b.origin[i]) {
	  x[0] = x[1] = b.origin[i];		
	  l = (a.length[i] > b.length[i]) ? b.length[i] : a.length[i];
	} else {
	  if (a.origin[i] < b.origin[i]) {
		x[0] = a.origin[i];
		l = a.length[i];
		x[1] = b.origin[i];
	  } else if (a.origin[i] > b.origin[i]) {
		x[0] = b.origin[i];
		l = b.length[i];
		x[1] = a.origin[i];
	  }
	  if ((x[1] - x[0]) > l) {
		return false;
	  }		
	}
  } 
  return true;
}

ABB bound_abb_posture( const ml::Posture &pos )
{
  ABB ret_box;
  cml::vector3 rad[6];
  rad[0] = pos.GetGlobalTranslation(21);  // head_dummy
  rad[1] = pos.GetGlobalTranslation(22);  // Left hand dummy
  rad[2] = pos.GetGlobalTranslation(24);  // Left toe dummy
  rad[3] = pos.GetGlobalTranslation(23);  // right hand dummy
  rad[4] = pos.GetGlobalTranslation(20);  // right toe dummy
  rad[5] = pos.GetGlobalTranslation(0);
  double min_x= 100000.0, min_y= 100000.0, min_z= 100000.0;
  double max_x=-100000.0, max_y=-100000.0, max_z=-100000.0;
  for (size_t i=0; i<6; ++i) {
	if (rad[i][0] > max_x) {
	  max_x = rad[i][0];
	} if (rad[i][0] < min_x) {
	  min_x = rad[i][0];
	}
	if (rad[i][1] > max_y) {
	  max_y = rad[i][1];
	} if (rad[i][1] < min_y) {
	  min_y = rad[i][1];
	}
	if (rad[i][2] > max_z) {
	  max_z = rad[i][2];
	} if (rad[i][2] < min_z) {
	  min_z = rad[i][2];
	}
  }
  ret_box.origin = cml::vector3(min_x, min_y, min_z);
  ret_box.length[0] = abs(max_x - min_x);
  ret_box.length[1] = abs(max_y - min_y);
  ret_box.length[2] = abs(max_z - min_z);
  return ret_box;
}

cml::vector3 virtual_geo_center(const ml::Posture &pos1)
{
  ABB abb = bound_abb_posture(pos1);
  return abb.origin + cml::vector3(abb.length[0]/2.0, abb.length[1]/2.0, abb.length[2]/2.0);
}

double min_distance(const ml::Posture &pos1, const ml::Posture &pos2)
{
  double min_d = 1000000.0;
  int indexes[4] = {0, 21, 22, 23};
  for (size_t i = 0; i < 4; ++i) {
	auto dist_vec = (pos1.GetGlobalTranslation(indexes[i]) - pos2.GetGlobalTranslation(indexes[i]));
	if (min_d > length(dist_vec)) min_d = length(dist_vec);
  }
  return min_d;
}

// 겹치는 구간이면 하나로 합친다.
bool merge( pair_section &sec1, const pair_section &sec2, const size_t thres_length )
{  
  int length = length_apart(sec1, sec2);
  if (length > static_cast<int>(thres_length)) {	// not overlap.
	return false;
  } else {
	overlap_both_pairsects(sec1, sec2);
	return true;
  }
}

// 인터렉션이 있는 연속적인 동작의 경우, 하나의 인터렉션으로 합친다.
bool merge( Interaction &inter1, const Interaction &inter2, const size_t thres_length )
{
  
  return false;
}

bool overlap_both_pairsects( pair_section &a, const pair_section &b )
{
  bool same_mot_0 = true, same_mot_1 = true;
  if (a.get_motion_idx(0) == b.get_motion_idx(0)) {
	a.set_pos_sec(0, overlap(a.get_section(0), b.get_section(0)));	
  } else if (a.get_motion_idx(0) == b.get_motion_idx(1)) {
	a.set_pos_sec(0, overlap(a.get_section(0), b.get_section(1)));	
  } else {
	same_mot_0 = false;
  }
  if (a.get_motion_idx(1) == b.get_motion_idx(0)) {
	a.set_pos_sec(1, overlap(a.get_section(1), b.get_section(0)));	
  } else if (a.get_motion_idx(1) == b.get_motion_idx(1)) {
	a.set_pos_sec(1, overlap(a.get_section(1), b.get_section(1)));	
  } else {
	same_mot_1 = false;
  }

  if (same_mot_0 && same_mot_1) {
	return true;
  } else if (!same_mot_0 && !same_mot_1) {
	cerr << "At least a motion index is same each other to overlap pair_sections!" << endl;
  } return false;
}

// 서로 붙어있거나 threshold 이하의 거리만큼 떨어져 있는 두 구간을 하나로 합친다.
void fill_pair_section_gap( vector<pair_section> &sects, const int length )
{
  if (sects.empty()) return;
  auto temp = sects;  
  for (size_t i = 0; i < sects.size() - 1; ++i) {
	for (size_t j = i + 1; j < sects.size(); ++j) {	  
	  int apartness = length_apart(sects[i], temp[j]); 
	  if (apartness < length) {
		overlap_both_pairsects(temp[j], sects[i]);
	  }
	}
	sects[i] = temp[i];
  } 
}

pair<size_t,size_t> overlap( const pair<size_t,size_t> a, const pair<size_t,size_t> b )
{
  pair<size_t,size_t> ret;
  ret.first = (a.first < b.first) ? a.first: b.first;
  ret.second = (a.second > b.second) ? a.second : b.second;
  return ret;
}

void divide_overlapping_sect_half( pair_section &a, pair_section &b )
{
  typedef pair<size_t,size_t> Duration;
  Duration *mss[2] = {&Duration(0,0), &Duration(0,0)};
  bool tog = true;

  if (a.get_motion_idx(0) == b.get_motion_idx(0)) {
	mss[0] = &a.psec[0].pos_sec;
	mss[1] = &b.psec[0].pos_sec;
  } else if (a.get_motion_idx(0) == b.get_motion_idx(1)) {
	mss[0] = &a.psec[0].pos_sec;
	mss[1] = &b.psec[1].pos_sec;
  } else {
	tog = false;
  }
  if (tog) {
	Duration overlap_ = overlap(*mss[0], *mss[1]);
	int beg = (mss[0]->first < mss[1]->first) ? 0 : 1;
	int fin = (beg == 0) ? 1 : 0;
	mss[beg]->second = (overlap_.first + overlap_.second) / 2;
	mss[fin]->first = (overlap_.first + overlap_.second) / 2 + 1;
  }  

  tog = true;
  if (a.get_motion_idx(1) == b.get_motion_idx(0)) {
	mss[0] = &a.psec[1].pos_sec;
	mss[1] = &b.psec[0].pos_sec;
  } else if (a.get_motion_idx(1) == b.get_motion_idx(1)) {
	mss[0] = &a.psec[1].pos_sec;
	mss[1] = &b.psec[1].pos_sec;
  } else {
	tog = false;
  }
  if (tog) {
	Duration overlap_ = overlap(*mss[0], *mss[1]);
	int beg = (mss[0]->first < mss[1]->first) ? 0 : 1;
	int fin = (beg == 0) ? 1 : 0;
	mss[beg]->second = (overlap_.first + overlap_.second) / 2;
	mss[fin]->first = (overlap_.first + overlap_.second) / 2 + 1;
  }  
}

int length_apart( const pair<size_t,size_t> &a, const pair<size_t,size_t> &b )
{
  size_t f1 = (a.first > b.first) ? a.first : b.first;
  size_t e1 = (a.second < b.second) ? a.second : b.second;
  return static_cast<int>(f1) - static_cast<int>(e1);
}

int length_apart( const pair_section &a, const pair_section &b )
{
  if (a.get_motion_idx(0) == b.get_motion_idx(0)) {	
	int l1 = length_apart(a.get_section(0), b.get_section(0));
	if (a.get_motion_idx(1) == b.get_motion_idx(1)) {	
	  int l2 = length_apart(a.get_section(1), b.get_section(1));
	  return (l2 < l1) ? l2 : l1;
	} else {
	  return l1;
	}
  } else if (a.get_motion_idx(1) == b.get_motion_idx(1)) {
	return length_apart(a.get_section(1), b.get_section(1));
  } else if (a.get_motion_idx(0) == b.get_motion_idx(1)) {
	int l1 = length_apart(a.get_section(0), b.get_section(1));
	if (a.get_motion_idx(1) == b.get_motion_idx(0)) {		  
	  int l2 = length_apart(a.get_section(1), b.get_section(0));
	  return (l2 < l1) ? l2 : l1;
	} else {
	  return l1;
	}
  } else if (a.get_motion_idx(1) == b.get_motion_idx(0)) {
	return length_apart(a.get_section(1), b.get_section(0));	
  } else {
	return 100000;
  }
}

int length_apart( const section &a, const section &b )
{
  if (a.motion_idx() != b.motion_idx()) { 
	return 100000; 
  } 
  return length_apart(a.pos_sec, b.pos_sec);
}

int length_apart( const Interact_section &a, const Interact_section &b )
{
  if (a.motion_idx() != b.motion_idx()) { 
	return 100000; 
  } 
  return length_apart(a.pos_sec, b.pos_sec);
}

void check_section( vector<ml::Motion> *mots, const vector<pair_section> &col_sec, const cml::vector3 &color)
{
  for (auto it=col_sec.begin(); it!=col_sec.end(); ++it) {	
	for (size_t i=it->get_section_beg(0); i<=it->get_section_end(0); ++i) {
	  mots->at(it->get_motion_idx(0)).posture(i).set_status_in_viewer(true, color);
	}	
	for (size_t i=it->get_section_beg(1); i<=it->get_section_end(1); ++i) {
	  mots->at(it->get_motion_idx(1)).posture(i).set_status_in_viewer(true, color);
	}
  }
}

bool overlap_secs( section &ret, const section a, const section b )
{
  if (a.motion_idx() != b.motion_idx()) {
	return false;
  } else {
	ret.mot_idx = a.motion_idx();
	ret.pos_sec = overlap(a.pos_sec, b.pos_sec);
	return true;
  }
}

void Interaction::normalize()
{
  if (relations.empty()) return;
  if (relations.size() == 1) {
	interacts.push_back(section(relations[0].get_motion_idx(0), relations[0].get_section(0)));
	interacts.push_back(section(relations[0].get_motion_idx(1), relations[0].get_section(1)));
  } else {
	vector<int> mark(2 * relations.size(), 0);
	auto mt = find(mark.begin(), mark.end(), 0);
	while (mt != mark.end()) {
	  int x = static_cast<int>(mt - mark.begin());
	  pair<size_t, size_t> sect = relations[x/2].get_section(x%2);
	  mark[mt - mark.begin()] = 1;
	  for (auto it = mark.begin(); it != mark.end(); ++it) {
		if (*it == 1) continue;
		int y = static_cast<int>(it - mark.begin());
		if (relations[x/2].get_motion_idx(x%2) == relations[y/2].get_motion_idx(y%2)) {
		  int l1 = length_apart(sect, relations[y/2].get_section(y%2));
		  if (l1 < 6) {
		   sect = overlap(sect, relations[y/2].get_section(y%2));
		   mark[it - mark.begin()] = 1;
		  }
		}
	  }
	  interacts.push_back(section(relations[x/2].get_motion_idx(x%2), sect));	  
	  mt = find(mark.begin(), mark.end(), 0);
	}	
  }
  for (auto it = interacts.begin(); it != interacts.end(); ++it) {
	auto ft = find_if(it + 1, interacts.end(), [it](section &s){return (s.motion_idx() == it->motion_idx()) ? true : false;});
	if (ft != interacts.end()) {
	  pair<size_t, size_t> sect = overlap(it->pos_sec, ft->pos_sec);
	  it->pos_sec = sect;
	  interacts.erase(ft);
	}
  }
}

bool is_same_sections( const pair_section &a, const pair_section &b )
{
  if ((a.get_motion_idx(0) == b.get_motion_idx(0) && a.get_motion_idx(1) == b.get_motion_idx(1)) || (a.get_motion_idx(1) == b.get_motion_idx(0) && a.get_motion_idx(0) == b.get_motion_idx(1))) {
	return true;
  } else {
	return false;
  }
}

size_t size( const vector<Interaction> &inters )
{
  size_t ret = 0;
  for (auto it = inters.begin(); it != inters.end(); ++it) {
	ret += it->size();
  } return ret;
}

double face_diff( const ml::Posture &a, const ml::Posture &b )
{
  return length(cml::log_mat3(cml::PlaneProject_mat3(a.GetGlobalRoation(7) * cml::inverse(b.GetGlobalRoation(7)))));
}

void fill_interact_section_gap( vector<Interact_section> &sects, const int length )
{
  if (sects.empty()) return;

  auto temp(sects);
  vector<Interact_section> rv;
  while (temp.size() > 1) {
	auto s = temp.begin();
	auto ft = find_if(temp.begin() + 1, temp.end(), [s, length](Interact_section& a)->bool{if (length_apart(*s, a) < length) return true; else return false;});
	if (ft == temp.end()) {
	  rv.push_back(*s);
	  temp.erase(s);
	} else {
	  overlap_interact_secs(*ft, *s, *ft);
	  temp.erase(s);
	}
  }
  rv.push_back(temp[0]);
  sects.swap(rv);
}

size_t length_section( const pair<size_t,size_t> &sect )
{
  if (sect.first > sect.second) return 0;
  return (sect.second - sect.first);
}

bool is_similar( const Interact_section &a, const Interact_section &b )
{
  if (a.entry_type() == b.entry_type() && a.exit_type() == b.exit_type() 
	&& std::abs(static_cast<double>(length_section(a.pos_sec)) - static_cast<double>(length_section(b.pos_sec))) < 8.0) {
	return true;
  } else {
	return false;
  }  
}

bool gather_pre_neighborhood_of_mots_interact( Interaction &inter, EntryExitTable& eeTable, const size_t inter_idx, const size_t inner_idx, const size_t Gather_section, vector<ml::Motion> &m )
{
  const auto &jt = inter.begin() + inner_idx;
  const ml::Motion &mot = m.at(jt->motion_idx());  

  if (jt->begin_pos() == 0) return false;

  EEGroup eeg;
  ml::Motion::Const_iterator end_cut = mot.begin() + (jt->begin_pos() - 1);

  if (end_cut == mot.begin()) {	
	//(m.at(jt->motion_idx()).begin() + (end_cut - mot.begin()))->set_status_in_viewer(true, cml::vector3(0.8, 0.8, 0.8));
	PatchEntryExit entext(jt->motion_idx(), end_cut - mot.begin(), 1,  &m);
	eeg.push(entext);	
  } else {
	bool is_iact = false;
	ml::Motion::Const_iterator mit = end_cut;
	for (size_t j = 0; j < Gather_section; ++j) {
	  contact_pre_state(mit, mot.begin());
	  if (mit == mot.begin() || mit->is_interact) break;
	}
	for (auto pt = end_cut; pt != mit; --pt) {
	  if (pt->is_interact) break;
	  //(m.at(jt->motion_idx()).begin() + (pt - mot.begin()))->set_status_in_viewer(true, cml::vector3(0.8, 0.8, 0.8));
	  PatchEntryExit entext(jt->motion_idx(), pt - mot.begin(), (mot.begin() + jt->begin_pos()) - pt,  &m);
	  eeg.push(entext);
	}	
  }

  if (eeg.size() == 0) {
	cerr << "there is none of leg" << endl; 
	return false;
  } else {
	eeg.leg_idx = 2 * inner_idx;
	eeg.interaction_idx = inter_idx;
	eeg.type = EEGroup::Entry;
	eeTable.set_boundary_group(eeg);
	inter.set_leg(eeTable.get_last_ptr_boundary_group());
	return true;
  }  
}

bool gather_post_neighborhood_of_mots_interact( Interaction &inter, EntryExitTable& eeTable, const size_t inter_idx, const size_t inner_idx, const size_t Gather_section, vector<ml::Motion> &m )
{
  const auto &jt = inter.begin() + inner_idx;
  const ml::Motion &mot = m.at(jt->motion_idx());  

  ml::Motion::Const_iterator end_cut = mot.begin() + jt->end_pos() + 1;
  if (end_cut == mot.end()) return false;

  EEGroup egg;

  if (end_cut == mot.end() - 1) {	
	//(m.at(jt->motion_idx()).begin() + (end_cut - mot.begin()))->set_status_in_viewer(true, cml::vector3(0.5, 0.8, 0.5));
	PatchEntryExit entext(jt->motion_idx(), end_cut - mot.begin(), end_cut - (mot.begin() + jt->end_pos()), &m);
	egg.push(entext);	
  } else {
	ml::Motion::Const_iterator mit = end_cut;
	for (size_t j = 0; j < Gather_section; ++j) {
	  contact_post_state(mit, mot.end() - 1);
	  if (mit == mot.end() - 1 || mit->is_interact) break;
	}
	for (auto pt = end_cut; pt != mit + 1; ++pt) {
	  if (pt->is_interact) break;
	  //(m.at(jt->motion_idx()).begin() + (pt - mot.begin()))->set_status_in_viewer(true, cml::vector3(0.5, 0.8, 0.5));
	  PatchEntryExit entext(jt->motion_idx(), pt - mot.begin(), pt - (mot.begin() + jt->end_pos()), &m);
	  egg.push(entext);
	}
  }

  if (egg.size() == 0) {
	cerr << "there is none of leg" << endl; 
	return false;
  } else {
	egg.leg_idx = 2 * inner_idx + 1;
	egg.interaction_idx = inter_idx;
	egg.type = EEGroup::Exit;
	eeTable.set_boundary_group(egg);
	inter.set_leg(eeTable.get_last_ptr_boundary_group());
	return true;
  } 
}

bool gather_pre_neighborhood_of_ingrient_interact( Interaction &inter, EntryExitTable& eeTable, const size_t inter_idx, const size_t inner_idx, const size_t Gather_section, Ingredient &ingre)
{
  const auto &jt = inter.begin() + inner_idx;
  auto &m = ingre.raw_before_cook[ingre.interact_bunch_to_bunble[inter_idx].first];
  const ml::Motion &mot = m.at(jt->motion_idx());

  if (jt->begin_pos() == 0) return false;

  EEGroup eeg;
  ml::Motion::Const_iterator end_cut = mot.begin() + (jt->begin_pos() - 1);

  if (end_cut == mot.begin()) {	
	//(m.at(jt->motion_idx()).begin() + (end_cut - mot.begin()))->set_status_in_viewer(true, cml::vector3(0.5, 0.5, 0.8));
	PatchEntryExit entext(ingre.interact_bunch_to_bunble[inter_idx].first, jt->motion_idx(), end_cut - mot.begin(), 1,  &ingre);
	eeg.push(entext);	
  } else {
	bool is_iact = false;
	ml::Motion::Const_iterator mit = end_cut;
	for (size_t j = 0; j < Gather_section; ++j) {
	  contact_pre_state(mit, mot.begin());
	  if (mit == mot.begin() || mit->is_interact) break;
	}
	for (auto pt = end_cut; pt != mit; --pt) {
	  if (pt->is_interact) break;
	  //(m.at(jt->motion_idx()).begin() + (pt - mot.begin()))->set_status_in_viewer(true, cml::vector3(0.5, 0.5, 0.8));
	  PatchEntryExit entext(ingre.interact_bunch_to_bunble[inter_idx].first, jt->motion_idx(), pt - mot.begin(), (mot.begin() + jt->begin_pos()) - pt,  &ingre);
	  eeg.push(entext);
	}	
  }

  if (eeg.size() == 0) {
	cerr << "there is none of leg" << endl; 
	return false;
  } else {
	eeg.leg_idx = 2 * inner_idx;
	eeg.interaction_idx = inter_idx;
	eeg.type = EEGroup::Entry;
	eeTable.set_boundary_group(eeg);
	inter.set_leg(eeTable.get_last_ptr_boundary_group());
	return true;
  }  
}


bool gather_post_neighborhood_of_ingrient_interact( Interaction &inter, EntryExitTable& eeTable, const size_t inter_idx, const size_t inner_idx, const size_t Gather_section, Ingredient &ingre )
{
  const auto &jt = inter.begin() + inner_idx;
  auto &m = ingre.raw_before_cook[ingre.interact_bunch_to_bunble[inter_idx].first];
  const ml::Motion &mot = m.at(jt->motion_idx());

  ml::Motion::Const_iterator end_cut = mot.begin() + jt->end_pos() + 1;
  if (end_cut == mot.end()) return false;

  EEGroup egg;

  if (end_cut == mot.end() - 1) {	
	//(m.at(jt->motion_idx()).begin() + (end_cut - mot.begin()))->set_status_in_viewer(true, cml::vector3(0.5, 0.8, 0.5));
	PatchEntryExit entext(ingre.interact_bunch_to_bunble[inter_idx].first, jt->motion_idx(), end_cut - mot.begin(), end_cut - (mot.begin() + jt->end_pos()), &ingre);
	egg.push(entext);	
  } else {
	ml::Motion::Const_iterator mit = end_cut;
	for (size_t j = 0; j < Gather_section; ++j) {
	  contact_post_state(mit, mot.end() - 1);
	  if (mit == mot.end() - 1 || mit->is_interact) break;
	}
	for (auto pt = end_cut; pt != mit + 1; ++pt) {
	  if (pt->is_interact) break;
	  //(m.at(jt->motion_idx()).begin() + (pt - mot.begin()))->set_status_in_viewer(true, cml::vector3(0.5, 0.8, 0.5));
	  PatchEntryExit entext(ingre.interact_bunch_to_bunble[inter_idx].first, jt->motion_idx(), pt - mot.begin(), pt - (mot.begin() + jt->end_pos()), &ingre);
	  egg.push(entext);
	}
  }

  if (egg.size() == 0) {
	cerr << "there is none of leg" << endl; 
	return false;
  } else {
	egg.leg_idx = 2 * inner_idx + 1;
	egg.interaction_idx = inter_idx;
	egg.type = EEGroup::Exit;
	eeTable.set_boundary_group(egg);
	inter.set_leg(eeTable.get_last_ptr_boundary_group());
	return true;
  } 
}

bool overlap_interact_secs( Interact_section &ret, const Interact_section &a, const Interact_section &b )
{
  if (a.motion_idx() != b.motion_idx()) {
	return false;
  } else {
	ret.mot_idx = a.motion_idx();
	ret.pos_sec = overlap(a.pos_sec, b.pos_sec);
	ret.boundary_type.first = (a.begin_pos() < b.begin_pos()) ? a.boundary_type.first : b.boundary_type.first;
	ret.boundary_type.second= (a.end_pos() < b.end_pos()) ? b.boundary_type.second: a.boundary_type.second;
	return true;
  }
}

pair_section::pair_section( size_t m1, size_t m2, pair<size_t,size_t> s1, pair<size_t,size_t> s2 )
  : bunch_idx(-1)
{
  type = Default;
  psec[0].mot_idx = m1;	
  psec[1].mot_idx = m2;
  psec[0].pos_sec = s1;	
  psec[1].pos_sec = s2;
}
