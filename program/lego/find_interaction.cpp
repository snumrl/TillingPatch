#include "StdAfx.h"
#include "find_interaction.h"
#include "intersect_obj.h"
#include "collision.h"



int collision_motions_at_frame( int time, const ml::Motion & m1, const ml::Motion & m2 );


vector<pair_section> InteractionFinder::process_bunch(const vector<ml::Motion> * ref_m, const size_t trim )
{
  vector<pair_section> mss;
  find_interactions(mss, *ref_m, trim);
  
  vector<size_t> mark(mss.size(), 0);
  while (true) {
	vector<size_t> group;
	auto beg = find(mark.begin(), mark.end(), 0);
	if (beg == mark.end()) break;
	group.push_back(beg - mark.begin());

	Interaction interact;
	interact.set_relation(mss[group[0]]);	
	mark[group[0]] = 1;

	size_t count = 0;
	while (count < group.size()) {		
	  for (size_t i = 0; i < mark.size(); ++i) {
		if (mark[i] == 1) continue;	  	  
		if (length_apart(mss[group[count]], mss[i]) < 6) {
		  interact.set_relation(mss[i]);
		  group.push_back(i);
		  mark[i] = 1;		  
		}	  
	  }
	  ++count;
	}
	interact.normalize();
	interactions.push_back(interact);	
  } 
  return mss;
}

vector<pair_section> InteractionFinder::process_bundle( Ingredient *ingre )
{
  vector<pair_section> ret;

  for (auto it = ingre->raw_before_cook.begin(); it != ingre->raw_before_cook.end(); ++it) {
	size_t bunch_idx = it - ingre->raw_before_cook.begin();
	size_t last_i_idx = interactions.size();
	auto bunch_ret = process_bunch(&(*it), 0);
	for_each(bunch_ret.begin(), bunch_ret.end(), [bunch_idx](pair_section & ps){ps.bunch_idx = static_cast<int>(bunch_idx);});
	copy(bunch_ret.begin(), bunch_ret.end(), back_inserter(ret));	
	size_t cur_i_idx = interactions.size();

	for (size_t i = last_i_idx; i < cur_i_idx; ++i) {
	  ingre->interact_bunch_to_bunble[i] = pair<size_t,size_t>(bunch_idx, i - last_i_idx);
	  ingre->interact_bunble_to_bunch[pair<size_t,size_t>(bunch_idx, i - last_i_idx)] = i;
	}
  }
  return ret;
}

vector<pair_section> InteractionFinder::process()
{
  vector<pair_section> ret;

  if (!mutual_exclusive_test()) {
	cerr << "Critical error during initialization!" << endl; 
	return ret;
  }
  
  if (ref_mots != nullptr) {
	ret = process_bunch(ref_mots);
	check_interactions(ref_mots, interactions);
	return ret;
  } else {
	ret = process_bundle(ingre);
	check_interactions(ingre, interactions);
	return ret;
  }   
}

const Interaction& InteractionFinder::get( size_t i ) const
{
  if (!mutual_exclusive_test()) {
	cerr << "Critical error during initialization!" << endl; 
	return interactions[0];
  }
 return interactions[i]; 
}

Interaction & InteractionFinder::get( size_t i )
{
  if (!mutual_exclusive_test()) {
	cerr << "Critical error during initialization!" << endl; 
	return interactions[0];
  }
  return interactions[i];  
}

bool InteractionFinder::mutual_exclusive_test() const
{
  if ((ref_mots == nullptr && ingre == nullptr) || ref_mots != nullptr && ingre != nullptr) {
	return false; 
  } else return true;
}

vector<pair_section> likely_collisions( const vector<ml::Motion> *mots )
{
  vector<pair_section> likely_col_section;
  if (mots->empty()) return likely_col_section;

  for (size_t i=0; i<mots->size()-1; ++i) {
	for (size_t j=i+1; j<mots->size(); ++j) {
	  likely_collisions(likely_col_section, i, j, mots);
	}
  }
  return likely_col_section;
}

void likely_collisions( vector<pair_section> &likely_col_sec, const size_t mot1, const size_t mot2, const vector<ml::Motion> *mots )
{
  const ml::Motion &m1 = mots->at(mot1);
  const ml::Motion &m2 = mots->at(mot2);

  if (m1.begin()->time > (m2.end()-1)->time || m2.begin()->time > (m1.end()-1)->time) return;

  int begin_time = (m1.begin()->time > m2.begin()->time) ? m1.begin()->time : m2.begin()->time;
  int last_time = ((m1.end()-1)->time < (m2.end()-1)->time) ? (m1.end()-1)->time : (m2.end()-1)->time;

  size_t bt1 = static_cast<size_t>(m1.begin()->time);
  size_t bt2 = static_cast<size_t>(m2.begin()->time);

  //모션의 가장자리에서는 충돌체크 안한다.  
  for (size_t time = begin_time+1; time < last_time; ++time) {	
	const ml::Posture &p1 = m1[time - bt1];
	const ml::Posture &p2 = m2[time - bt2];
	if (collision_likely_posture(p1, p2)) {
	  pair_section col_sec(mot1, mot2, pair<size_t,size_t>(time - bt1, time - bt1), pair<size_t,size_t>(time - bt2, time - bt2));
	  while (collision_likely_posture(m1[time - bt1], m2[time - bt2])) {
		if (++time >= last_time) {
		  likely_col_sec.push_back(col_sec);
		  break;
		}		
	  }
	  col_sec.set_pos_sec(0, pair<size_t,size_t>(col_sec.get_section_beg(0), --time - bt1));
	  col_sec.set_pos_sec(1, pair<size_t,size_t>(col_sec.get_section_beg(1), time - bt2));
	  likely_col_sec.push_back(col_sec);
	}
  }
}

bool collision_likely_posture( const ml::Posture &pos1, const ml::Posture &pos2 )
{
  ABB abb1 = bound_abb_posture(pos1);
  ABB abb2 = bound_abb_posture(pos2);
  cml::vector3 abb1_ori = abb1.origin + cml::vector3(abb1.length[0]/2.0, abb1.length[1]/2.0, abb1.length[2]/2.0);
  cml::vector3 abb2_ori = abb2.origin + cml::vector3(abb2.length[0]/2.0, abb2.length[1]/2.0, abb2.length[2]/2.0);
  cml::vector3 ro1 = pos1.GetGlobalRoation(0) * cml::vector3(1.0, 0., 0.);
  cml::vector3 ro2 = pos2.GetGlobalRoation(0) * cml::vector3(1.0, 0., 0.);
  ro1[1] = ro2[1] = 0.0;
  ro1 /= ro1.length();
  ro2 /= ro2.length();

  return (intersect_abb(abb1, abb2) && abs(cml::dot(ro1, ro2)) < 0.7);  
}


vector<pair_section> exact_collisions( const vector<ml::Motion> *mots )
{
  vector<pair_section> secs;
  for (size_t i = 0; i < mots->size() - 1; ++i) {
	for (size_t j = i + 1; j < mots->size(); ++j) {
	  exact_collisions(secs, i, j, mots);
	}
  }
  return secs;
}

void exact_collisions( vector<pair_section> &secs, size_t i, size_t j, const vector<ml::Motion> *mots )
{
  const ml::Motion &m1 = mots->at(i);
  const ml::Motion &m2 = mots->at(j);
  if (m1.begin()->time > (m2.end()-1)->time || m2.begin()->time > (m1.end()-1)->time) return;

  int begin_time = m1.begin()->time;
  int last_time = (m1.end()-1)->time;  
  for (int time = begin_time; time <= last_time; ++time) {
	if (collision_motions_at_frame(time, m1, m2) >= 1) {
	  pair_section sec;
	  sec.type = pair_section::Contact;
	  sec.set_motion_idx(0, i);	  
	  sec.set_motion_idx(1, j);
	  size_t s0 = m1.iterator_at(time) - m1.begin();
	  size_t s1 = m2.iterator_at(time) - m2.begin();
	  while (collision_motions_at_frame(++time, m1, m2) >= 1) {
		if (time > last_time) break;
	  }
	  sec.set_pos_sec(0, pair<size_t,size_t>(s0, m1.iterator_at(--time) - m1.begin()));
	  sec.set_pos_sec(1, pair<size_t,size_t>(s1, m2.iterator_at(time) - m2.begin()));	  
	  secs.push_back(sec);
	}	
  }
}

vector<pair_section> intimate_characters( const vector<ml::Motion> *mots )
{
  vector<pair_section> intimate_section;
  if (mots->empty()) return intimate_section;

  for (size_t i=0; i<mots->size()-1; ++i) {
	for (size_t j=i+1; j<mots->size(); ++j) {
	  intimate_characters(intimate_section, i, j, mots);
	}
  }
  return intimate_section;
}

void intimate_characters( vector<pair_section> &inti_sec, const size_t mot1, const size_t mot2, const vector<ml::Motion> *mots )
{
  size_t midx[2] = {mot1, mot2};
  const ml::Motion * m[2] = {&mots->at(mot1),  &mots->at(mot2)};
  if (m[0]->begin()->time > (m[1]->end()-1)->time || m[1]->begin()->time > (m[0]->end()-1)->time) return;

  double bt1 = m[0]->begin()->time, bt2 = m[1]->begin()->time;
  double begin_time = (bt1 > bt2) ? bt1: bt2;
  size_t begin_frame[2] = {m[0]->iterator_at(begin_time) - m[0]->begin(), m[1]->iterator_at(begin_time) - m[1]->begin()};

  double last_time = ((m[0]->end()-1)->time < (m[1]->end()-1)->time) ? (m[0]->end()-1)->time : (m[1]->end()-1)->time;
  size_t last_frame[2] = {m[0]->iterator_at(last_time) - m[0]->begin(), m[1]->iterator_at(last_time) - m[1]->begin()};  

  int longer = (last_frame[0] - begin_frame[0] > last_frame[1] - begin_frame[1]) ? 0 : 1;
  int shorter= (longer == 0) ? 1 : 0;
  size_t duration =  last_frame[longer] - begin_frame[longer];

  for (size_t i = 0; i < duration; ++i) {
	const ml::Posture &p1 = (*m[longer])[begin_frame[longer] + i];
	double time = p1.time;
	const ml::Posture &p2 = (*m[shorter]).posture_at_time(time);
	if (abs(p1.time - p2.time) < 1.0 && is_spatio_intimate(p1, p2)) {	  
	  pair_section i_sec;	  
	  i_sec.set_motion_idx(0, midx[longer]);
	  i_sec.set_motion_idx(1, midx[shorter]);
	  i_sec.set_pos_sec_beg(0, begin_frame[longer] + i);
	  i_sec.set_pos_sec_beg(1, m[shorter]->iterator_at(time) - m[shorter]->begin());
	  i_sec.type = pair_section::Intimate;
	  while (++i < duration && is_spatio_intimate((*m[longer])[begin_frame[longer] + i], (*m[shorter]).posture_at_time((*m[longer])[begin_frame[longer] + i].time)))
		;
	  i_sec.set_pos_sec_end(0, begin_frame[longer] + (--i));
	  i_sec.set_pos_sec_end(1, m[shorter]->iterator_at((*m[longer])[begin_frame[longer] + i].time) - m[shorter]->begin());
	  inti_sec.push_back(i_sec);
	}
  }
}

bool is_spatio_intimate( const ml::Posture &pos1, const ml::Posture &pos2 )
{ 
  const double thresh = 0.8;
  double delta_inti = spatio_intimate_val(pos1, pos2);  

  if (delta_inti > thresh) { 
	return true; 
  } else {
	return false;
  }
}

double spatio_intimate_val( const ml::Posture &pos1, const ml::Posture &pos2 )
{
  //cml::vector3 abb_center1 = virtual_geo_center(pos1);
  //cml::vector3 abb_center2 = virtual_geo_center(pos2);  
  //return 1.0 / (length(abb_center1 - abb_center2) + 1.0) + 1.0 / (min_distance(pos1, pos2) + 1.0);	  // ( > 1.9 ||  < 0.17)

  return 1.0 / (all_comb_joints_distance(pos1, pos2) + 1.0);  
}

double all_comb_joints_distance( const ml::Posture & pos1, const ml::Posture & pos2 ) 
{
  double min_dist = 10000000.0;
  for (size_t i = 0; i < pos1.num_joint(); ++i) {
	for (size_t j = 0; j < pos2.num_joint(); ++j) {
	  double loc_dist = (pos1.GetGlobalTranslation(i) - pos2.GetGlobalTranslation(j)).length_squared();
	  if (min_dist > loc_dist) min_dist = loc_dist;
	}
  }
  return std::sqrt(min_dist);
}

void unite_sections_within_contact( vector<pair_section> &col, const size_t contact_length, const vector<ml::Motion> *mots )
{
  //vector<pair_section> temp(col.begin(), col.end());  
  vector<int> erasing_mark(col.size(), 0);

  for (size_t i = 0; i < col.size() - 1; ++i) {
	bool is_united = false;
	for (size_t j = i + 1; j < col.size(); ++j) {
	  if (is_same_sections(col[i], col[j])) {
		if (length_apart(col[i], col[j]) > 60)	continue;
		if (length_apart(col[i], col[j]) < 1) {
		  int j_same_idx = (col[i].get_motion_idx(0) == col[j].get_motion_idx(0)) ? 0 : 1;
		  pair_section new_sec;
		  new_sec.set_motion_idx(0, col[i].get_motion_idx(0));
		  new_sec.set_motion_idx(1, col[i].get_motion_idx(1));
		  new_sec.set_pos_sec(0, overlap(col[i].get_section(0), col[j].get_section(j_same_idx)));
		  int inv_j_same_idx = (j_same_idx == 0) ? 1 : 0;
		  new_sec.set_pos_sec(1, overlap(col[i].get_section(1), col[j].get_section(inv_j_same_idx)));
		  col[j] = new_sec;
		  erasing_mark[i] = 1;

		  continue;
		}

		int j_same_idx = (col[i].get_motion_idx(0) == col[j].get_motion_idx(0)) ? 0 : 1;
		ml::Motion::Const_iterator edge = mots->at(col[i].get_motion_idx(0)).begin() + (col[i].get_section_beg(0) + col[i].get_section_end(0)) / 2;

		if (col[i].get_section_beg(0) > col[j].get_section_end(j_same_idx)) { 
		  for (size_t t = 0; t < contact_length; ++t) {
			contact_pre_state(edge, mots->at(col[i].get_motion_idx(0)).begin());
			if (edge == mots->at(col[i].get_motion_idx(0)).begin()) break;
		  }		  
		  if ((edge - mots->at(col[i].get_motion_idx(0)).begin()) < col[j].get_section_end(j_same_idx)) {
			pair_section new_sec;
			new_sec.set_motion_idx(0, col[i].get_motion_idx(0));
			new_sec.set_motion_idx(1, col[i].get_motion_idx(1));
			new_sec.set_pos_sec(0, overlap(col[i].get_section(0), col[j].get_section(j_same_idx)));
			int inv_j_same_idx = (j_same_idx == 0) ? 1 : 0;
			new_sec.set_pos_sec(1, overlap(col[i].get_section(1), col[j].get_section(inv_j_same_idx)));
			col[j] = new_sec;
			is_united = true;
		  }
		} else if (col[i].get_section_end(0) < col[j].get_section_beg(j_same_idx)) { 
		  for (size_t t = 0; t < contact_length; ++t) {
			contact_post_state(edge, mots->at(col[i].get_motion_idx(0)).end() - 1);
			if (edge == mots->at(col[i].get_motion_idx(0)).end() - 1) break;
		  }
		  if (edge - mots->at(col[i].get_motion_idx(0)).begin() > col[j].get_section_beg(j_same_idx)) {
			pair_section new_sec;
			new_sec.set_motion_idx(0, col[i].get_motion_idx(0));
			new_sec.set_motion_idx(1, col[i].get_motion_idx(1));
			new_sec.set_pos_sec(0, overlap(col[i].get_section(0), col[j].get_section(j_same_idx)));
			int inv_j_same_idx = (j_same_idx == 0) ? 1 : 0;
			new_sec.set_pos_sec(1, overlap(col[i].get_section(1), col[j].get_section(inv_j_same_idx)));
			col[j] = new_sec;
			is_united = true;
		  }		  
		}
	  }
	}
	if (is_united) erasing_mark[i] = 1;
  }

  for (int i = erasing_mark.size() - 1; i >= 0; --i) {
	if (erasing_mark[i] == 1) {
	  col.erase(col.begin() + i);
	}  
  }
}

void strong_interaction( vector<pair_section> &sec, const vector<ml::Motion> *mots )
{
  vector<pair_section> intimate = intimate_characters(mots);
  erase_timid_interaction(intimate, 4);
  unite_sections_within_contact(intimate, 4, mots);

  vector<pair_section> collision = exact_collisions(mots);
  unite_sections_within_contact(collision, 5, mots);
  
  insert_sections(sec, intimate, collision);
}

void insert_sections( vector<pair_section> &new_sec, const vector<pair_section> &sec1, const vector<pair_section> &sec2 )
{
  new_sec.clear();
  copy(sec1.begin(), sec1.end(), back_inserter(new_sec));  

  for (size_t i = 0; i < sec2.size(); ++i) {	
	for (size_t j = 0; j < sec1.size(); ++j) {
	  merge(new_sec[j], sec2[i], 9); 
	}
  }
  copy(sec2.begin(), sec2.end(), back_inserter(new_sec)); 
}


int collision_motions_at_frame( int time, const ml::Motion & m1, const ml::Motion & m2 )
{
  VCollide vc;
  int id[2];

  vc.NewObject(&id[0]);
  add_collision_motion(time, m1, vc);
  vc.EndObject();

  vc.NewObject(&id[1]);
  add_collision_motion(time, m2, vc);
  vc.EndObject();

  VCReport report;
  vc.Collide(&report);

  return report.numObjPairs();
}


void erase_timid_interaction( vector<pair_section> &secs, const size_t length)
{
  for (auto it = secs.begin(); it != secs.end(); ) {
	if (it->get_section_end(0) - it->get_section_beg(0) < length && it->get_section_end(1) - it->get_section_beg(1) < length) {
	  it = secs.erase(it);
	} else {
	  ++it;
	}
  }
}

void find_interactions( vector<pair_section> & inters, const vector<ml::Motion> &motions, const size_t trim_edge )
{
  vector<pair_section> si;
  strong_interaction(si, &motions);
  vector<pair_section> wi;
  synchronized_interaction(wi, &motions);
  insert_sections(inters, si, wi);

  // 모션 양 끝에 존재하는 인터렉션은 제거한다.  
  for (auto it = inters.begin(); it != inters.end();) {	
	if (it->get_section_beg(0) < trim_edge|| 
		it->get_section_beg(1) < trim_edge|| 
		it->get_section_end(0) > motions[it->get_motion_idx(0)].size() - trim_edge ||
		it->get_section_end(1) > motions[it->get_motion_idx(1)].size() - trim_edge) 
	{ 
	  it = inters.erase(it);
	} else { 
	  ++it;
	}
  }
}



void check_interactions( vector<ml::Motion> *motions, const vector<Interaction> &inters )
{ 
  double r = 0.2, g = 0.3, b = 0.25; 
  double ratio = 1.34;
  for (auto it = inters.begin(); it != inters.end(); ++it) {		
	for (auto jt = it->begin(); jt != it->end(); ++jt) {	  
	  r *= (ratio * ratio); g *= ratio; b *= ratio;
	  for (size_t j = jt->begin_pos(); j <= jt->end_pos(); ++j) {
		motions->at(jt->motion_idx()).posture(j).is_interact = true;
		//double intpart;
		//motions->at(jt->motion_idx()).posture(j).set_status_in_viewer(true, cml::vector3(modf(r, &intpart), modf(g, &intpart), modf(b, &intpart)));
	  }	  
	}	
  }
}

void check_interactions( vector<ml::Motion> *motions, const vector<pair_section> &relation_pairs )
{
  for (auto it = relation_pairs.begin(); it != relation_pairs.end(); ++it) {
	for (size_t idx = 0; idx < 2; ++idx) {
	  for (size_t j = it->get_section_beg(idx); j <= it->get_section_end(idx); ++j) {
		motions->at(it->get_motion_idx(idx)).posture(j).is_interact = true;
	  }
	}
  }
}

void check_interactions( Ingredient *ingre, const vector<Interaction> &inters )
{
  for (size_t i = 0; i < inters.size(); ++i) {
	auto mapped_idx = ingre->interact_bunch_to_bunble[i];
	for (auto jt = inters[i].begin(); jt != inters[i].end(); ++jt) {
	  for (size_t k = jt->begin_pos(); k <= jt->end_pos(); ++k) {
		ingre->raw_before_cook[mapped_idx.first][jt->motion_idx()].posture(k).is_interact = true;
		//ingre->raw_before_cook[mapped_idx.first][jt->motion_idx()].posture(k).set_status_in_viewer(true, cml::vector3(1.0, 0.2, 0.2));
	  }		
	}	
  }
}

void synchronized_interaction( vector<pair_section> &sects, const vector<ml::Motion> *mots )
{
  typedef vector<double> activation_vals;
  vector<activation_vals> mots_activations(mots->size());

  for (size_t i = 0; i < mots->size(); ++i) {
	const auto &mot = mots->at(i);
	vector<double> weights = extract_joint_weight(mot);
	vector<double> joint_variance(mot.body()->num_joint(), 1.0);
	activation_vals act_val(mot.size(), 50.0);

	for (size_t p = 1; p < mot.size()-1; ++p) {
	  act_val[p] = activation(mot, p, weights, joint_variance, 3);
	}
	mots_activations[i] = act_val;
  }

  for (size_t i = 0; i < mots->size() - 1; ++i) {
	for (size_t j = i +	1; j < mots->size(); ++j) {
	  vector<pair_section> temp_secs;	  
	  if (synchronized_action(temp_secs, i, j, mots, mots_activations)) {
		copy(temp_secs.begin(), temp_secs.end(), back_inserter(sects));
	  }
	}  
  }  

  erase_timid_interaction(sects);
  unite_sections_within_contact(sects, 4, mots);
}

bool synchronized_action( vector<pair_section> &secs, const size_t i, const size_t j, const vector<ml::Motion> *mots, const vector< vector<double> > &activations )
{
  const ml::Motion *m[2] = {&mots->at(i),  &mots->at(j)};
  bool has_sync_interaction = false;
  if (m[0]->begin()->time > (m[1]->end()-1)->time || m[1]->begin()->time > (m[0]->end()-1)->time) return has_sync_interaction;

  size_t midx[2] = {i, j};  
  const vector<double> acts[2] = {activations[i], activations[j]};

  double begin_time = (m[0]->begin()->time > m[1]->begin()->time) ? m[0]->begin()->time: m[1]->begin()->time;
  size_t begin_frame[2] = {m[0]->iterator_at(begin_time) - m[0]->begin(), m[1]->iterator_at(begin_time) - m[1]->begin()};
  double last_time = ((m[0]->end()-1)->time < (m[1]->end()-1)->time) ? (m[0]->end()-1)->time : (m[1]->end()-1)->time;
  size_t last_frame[2] = {m[0]->iterator_at(last_time) - m[0]->begin(), m[1]->iterator_at(last_time) - m[1]->begin()};  

  int longer = (last_frame[0] - begin_frame[0] > last_frame[1] - begin_frame[1]) ? 0 : 1;
  int shorter= (longer == 0) ? 1 : 0;
  size_t duration =  last_frame[longer] - begin_frame[longer];

  for (size_t k = 0; k < duration; ++k) {
	const auto &p1 = (*m[longer])[begin_frame[longer] + k];
	double time = p1.time;
	size_t idx2 = (*m[shorter]).iterator_at(time) - m[shorter]->begin();
	const auto &p2 = m[shorter]->posture(idx2);

	if (abs(p1.time - p2.time) < 1.0 && is_sync_interaction(p1, p2, acts[longer][begin_frame[longer] + k], acts[shorter][idx2])) {
	  pair_section i_sec(midx[longer], midx[shorter], pair<size_t,size_t>(begin_frame[longer] + k, 0), pair<size_t,size_t>(idx2, 0));
	  i_sec.type = pair_section::Sync;
	  while (++k < duration) {
		const auto & lpos = (*m[longer])[begin_frame[longer] + k];
		size_t sidx = m[shorter]->iterator_at(lpos.time) - m[shorter]->begin();
		if (!is_sync_interaction(lpos, (*m[shorter])[sidx], acts[longer][begin_frame[longer] + k], acts[shorter][sidx])) break;	
	  }		
	  i_sec.set_pos_sec_end(0, begin_frame[longer] + (--k));
	  i_sec.set_pos_sec_end(1, m[shorter]->iterator_at((*m[longer])[begin_frame[longer] + k].time) - m[shorter]->begin());
	  secs.push_back(i_sec);
	  has_sync_interaction = true;
	}
  }  
  return has_sync_interaction;
}

bool is_sync_interaction( const ml::Posture &p1, const ml::Posture &p2, const double act1, const double act2 )
{
  cml::vector3 &a = virtual_geo_center(p1);
  cml::vector3 &b = virtual_geo_center(p2);
  //cout << 1.0 / length(a - b) << ", " << act1 << ", " << act2 << ", " << dot(shoulder_orientation(p1), shoulder_orientation(p2)) << endl;
  bool ab = ((1.0 / length(a - b)) > 0.97), bb = (act1 * act2 > 20.0), cb = abs(dot(shoulder_orientation(p1), shoulder_orientation(p2))) > 0.4;
  return (ab && bb && cb);
}


bool gather_boundary( vector<Interaction>& inters, EntryExitTable& eeTable, vector<ml::Motion> &m, const size_t gather_sect /*= 15*/ )
{
  eeTable.clear();
  eeTable.reserve_mem(size(inters));
  for (auto it = inters.begin(); it != inters.end(); ++it) {
	for (size_t j = 0; j < it->size(); ++j) {
	  if (!gather_pre_neighborhood_of_mots_interact(*it, eeTable, it - inters.begin(), j, gather_sect, m)) return false;
	  if (!gather_post_neighborhood_of_mots_interact(*it, eeTable, it - inters.begin(), j, gather_sect, m)) return false;
	}
  }
  return true;
}

bool gather_boundary( vector<Interaction> &inters, EntryExitTable &eeTable, Ingredient &ingre, const size_t sect /*= 15*/ )
{
  eeTable.clear();
  eeTable.reserve_mem(size(inters));

  for (size_t i = 0; i < inters.size(); ++i) {	
	for (size_t j = 0; j < inters[i].size(); ++j) {
	  if (!gather_pre_neighborhood_of_ingrient_interact(inters[i], eeTable, i, j, sect, ingre)) return false;
	  if (!gather_post_neighborhood_of_ingrient_interact(inters[i], eeTable, i, j, sect, ingre)) return false;
	}
  }
  return true;
}

