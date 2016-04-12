#include "StdAfx.h"
#include "patch_cooker.h"
#include "../combination/combination.h"


bool operator==( const Boundary_group &a, const Boundary_group &b )
{
  if (a.idx == b.idx && 
	a.sect.motion_idx() == b.sect.motion_idx() && 
	a.sect.begin_pos() == b.sect.begin_pos() && 
	a.sect.end_pos() == b.sect.end_pos()) return true;
  else return false;
}

bool operator!=( const Boundary_group &a, const Boundary_group &b )
{
  return !(a == b);
}


PatchCooker::PatchCooker( vector<ml::Motion> *motions )
  : ref_mots(motions), interaction_prober(new InteractionFinder(motions)), eeTable(new EntryExitTable(motions->size())), boundaryTypeSize(4)
{
  ingredients = nullptr;
  patches = new map<string, Patch>();
  unary_patches = new vector<Patch>();
}

PatchCooker::PatchCooker( Ingredient *ingre )
  : ingredients(ingre), boundaryTypeSize(4), eeTable(new EntryExitTable(ingre->size_mot())), interaction_prober(new InteractionFinder(ingre))
{
  ref_mots = nullptr;
  patches = new map<string, Patch>();
  unary_patches = new vector<Patch>();
}

PatchCooker::~PatchCooker()
{
  delete interaction_prober;
  delete eeTable;
  delete patches;
  delete unary_patches;
}

vector<pair_section> PatchCooker::init_interactions()
{
  vector<pair_section> ret;
  
  time_t start_time;
  time(&start_time);
  
  ret = interaction_prober->process(); 
  
  time_t cur_time;
  time(&cur_time);
  cout << "time " << difftime(cur_time, start_time) << endl;  
  cout << "interaction size: " << interaction_prober->size() << endl;
  return ret;
}

bool PatchCooker::init_EETable()
{
  if (ref_mots != nullptr) {
	if (!gather_boundary(interaction_prober->interactions, *eeTable, *ref_mots)) {
	  return false;
	} else {
	  return true;
	}	
  } else {
	if (!gather_boundary(interaction_prober->interactions, *eeTable, *ingredients)) {
	  return false;
	} else {
	  return true;
	}
  }  
}

void PatchCooker::cook_motion_patches()
{
  if (raw_interact_patch.empty()) return;

  for (auto it = raw_interact_patch.begin(); it != raw_interact_patch.end(); ++it) {
	fill_interact_section_gap(*it, 1);	
	std::ostringstream oss;
	oss << it - raw_interact_patch.begin();
	string name(oss.str());	
	cook_a_patch(*it, name);	
	sampling_patches.push_back(name);
  }

  if (raw_non_interact_patch.empty()) return;
  remove_timid_patch(raw_non_interact_patch);  
  remove_similar_non_interact_patch(raw_non_interact_patch);  

  for (auto nit = raw_non_interact_patch.begin(); nit != raw_non_interact_patch.end(); ++nit) {
	std::ostringstream oss;
	oss << "u" << nit - raw_non_interact_patch.begin();
	string name(oss.str());
	cook_a_patch(*nit, name);
	sampling_patches.push_back(name);
  }
}

void PatchCooker::cook_a_patch( const vector<section> &inter, const string &name )
{
  if (inter.empty()) return;

  Patch_constructor * pcons;
  if (ref_mots != nullptr) {
	pcons = new Patch_constructor(ref_mots);
  } else {
	pcons = new Patch_constructor(ingredients);
  }
  
  for (auto it = inter.begin(); it != inter.end(); ++it) {	
	pcons->set_relation(*it);
  }
  if (pcons->realize_motion_data()) {
	(*patches)[name] = *pcons->get_patch();
	patches->at(name).set_boundaries();
  } 
  delete pcons;
}

void PatchCooker::cook_a_patch( const vector<Interact_section> &inters, const string &name )
{
  if (inters.empty()) return;
  
  Patch_constructor * pcons;
  if (ref_mots != nullptr) {
	pcons = new Patch_constructor(ref_mots);
  } else {
	pcons = new Patch_constructor(ingredients);
  }
  pcons->set_name(name);
  for (auto it = inters.begin(); it != inters.end(); ++it) {
	pcons->set_interact_mot(*it);
  }
  if (pcons->realize_motion_data()) {
	pcons->set_constraint();	
	(*patches)[name] = *pcons->get_patch();	
	patches->at(name).set_boundaries();
  }  
  delete pcons;
}

void PatchCooker::cook_a_patch( const Interact_section &inter, const string &name )
{
  Patch_constructor * pcons;
  if (ref_mots != nullptr) {
	pcons = new Patch_constructor(ref_mots);
  } else {
	pcons = new Patch_constructor(ingredients);
  }
  pcons->set_name(name);
  pcons->set_interact_mot(inter);
  
  if (pcons->realize_motion_data()) {	
	unary_patches->push_back(*pcons->get_patch());
	(*patches)[name] = unary_patches->back();
	patches->at(name).set_boundaries();
  }  
  delete pcons;
}


void PatchCooker::evaluate_cross_sects_given_pose()
{
  if (!feature_grades.empty()) feature_grades.clear();

  if (cross_sect_features.empty()) {
	cerr << "please, set cross section posture first!" << endl; return;
  }
  for (auto it = cross_sect_features.begin(); it != cross_sect_features.end(); ++it) {
	size_t can_f_idx = it - cross_sect_features.begin();
	double eval = eeTable->evaluate_cross_sect_given_pose(can_f_idx, *it, size_interactions());
	if (eval < 0.0) {
	  cerr << "interruption occurs during evaluating cross sections!" << endl;
	  feature_grades.clear();
	  return;
	} else {
	  feature_grades.push_back(pair<size_t,double>(can_f_idx, eval));
	}
  }  
  sort_candi_poses();  
}

section PatchCooker::covering_section( const section &sec, const size_t bef, const size_t aft )
{
  section ret;
  ret.mot_idx = sec.motion_idx();
  auto &mot1 = ref_mots->at(sec.motion_idx());
  ml::Motion::Iterator mit = mot1.begin() + sec.begin_pos();
  for (size_t i = 0; i < bef; ++i) {
	contact_pre_state(mit, mot1.begin());
	if (mit == mot1.begin()) break;
  }
  ret.pos_sec.first = mit - mot1.begin() + 1;
  
  mit = mot1.begin() + sec.end_pos();
  for (size_t i = 0; i < aft; ++i) {
	contact_post_state(mit, mot1.end() - 1);
	if (mit == (mot1.end() - 1)) break;
  }
  ret.pos_sec.second = mit - mot1.begin();

  return ret;
}

pair_section PatchCooker::direct_covering_pair_section( const pair_section &pair_sect, const size_t bef, const size_t aft )
{
  pair_section ret(pair_sect.get_motion_idx(0), pair_sect.get_motion_idx(1), pair<size_t,size_t>(0, 0), pair<size_t,size_t>(0, 0));
  ret.set_pos_sec(0, covering_section(pair_sect.psec[0], bef, aft).pos_sec);	
  ret.set_pos_sec(1, covering_section(pair_sect.psec[1], bef, aft).pos_sec);
  return ret;
}

vector<section> PatchCooker::direct_covering_interacts( const Interaction &inter, const size_t bef, const size_t aft )
{
  vector<section> ret;
  ret.reserve(inter.size());
  for (auto it = inter.begin(); it != inter.end(); ++it) {
	ret.push_back(covering_section(*it, bef, aft));
  }
  return ret;
}

void PatchCooker::mold_motion_patches(size_t b_size)
{
  set_boundary_type_size(b_size);
  auto distri = most_distributed_select_poses();

  mold_interact_patch();
  mold_non_interact_patch(distri);  
}

void PatchCooker::select_most_distributed_groups( vector< vector< vector<size_t> > > &selected_tree_idx_group, vector< pair<double,size_t> > &eval_selected_groups, size_t size_ )
{
  selected_tree_idx_group.clear();
  eval_selected_groups.clear();

  typedef vector< vector<size_t> > distribution;
  vector<distribution> group_distribution;
  for (size_t i = 0; i < eeTable->size_clusters(); ++i) {
	distribution temp;
	eeTable->get_cluster_group_distribution(temp, i);
	group_distribution.push_back(temp);
  }

  vector< pair<double, size_t> > evals(group_distribution.size(), pair<double, size_t>(0.0, 0));
  for (size_t i = 0; i < group_distribution.size(); ++i) {
	evals[i] = pair<double, size_t>(evaluate_distribution(group_distribution[i]), i);
  }
  sort(evals.begin(), evals.end(), [](pair<double,size_t> &i, pair<double,size_t> &j)->bool{return (i.first > j.first);});  

  for (size_t i = 0; i < size_; ++i) {
	selected_tree_idx_group.push_back(group_distribution[evals[i].second]);
  }  
  eval_selected_groups = evals;

  // print out for debugging
  // for (size_t i = 0; i < 2 * size_; ++i) {	
  //	cout << "distribution: " << evals[i].first << endl;
  // }
  /*double r = 0.14, g = 0.18, b = 0.22; 
  double ratio = 1.26;
  for (auto it = selected_group.begin(); it != selected_group.end(); ++it) {
  for (auto jt = it->begin(); jt != it->end(); ++jt) {
  for (auto kt = jt->begin(); kt != jt->end(); ++kt) {
  eeTable->get_cluster_tree_leaf_pee(*kt)->it_posture()->set_status_in_viewer(true, cml::vector3(r, g, b));
  }	  
  }
  r *= ratio; g *= ratio; b *= ratio;
  }*/
}

void PatchCooker::select_best_combinations_of_groups( vector< vector< vector<size_t> > > &candi_tree_idx_groups, vector<int> &out_combi, vector<double> &eval, size_t candi_size )
{
  typedef vector< vector<size_t> > distribution;
  vector<distribution> loc_tree_idx_distribution;
  for (size_t i = 0; i < eeTable->size_clusters(); ++i) {
	distribution temp;
	eeTable->get_cluster_group_distribution(temp, i);
	loc_tree_idx_distribution.push_back(temp);
  }

  vector< pair<double, size_t> > evals(loc_tree_idx_distribution.size(), pair<double, size_t>(0.0, 0));
  for (size_t i = 0; i < loc_tree_idx_distribution.size(); ++i) {
	evals[i] = pair<double, size_t>(evaluate_distribution(loc_tree_idx_distribution[i]), i);
  }
  sort(evals.begin(), evals.end(), [](pair<double,size_t> &i, pair<double,size_t> &j)->bool{return (i.first > j.first);});    

  candi_tree_idx_groups.clear();
  original_candidate_group_idxes.clear();

  auto last_it = find_if(evals.begin(), evals.end(), [&evals, candi_size](pair<double,size_t> &a)->bool {return (a.first == evals[candi_size - 1].first);});
  for (auto it = evals.begin(); it != last_it; ++it) {
	candi_tree_idx_groups.push_back(loc_tree_idx_distribution[it->second]);
	original_candidate_group_idxes.push_back(it->second);
  } 

  auto same_elem_boundary_it = find_end(evals.begin(), evals.end(), last_it, last_it + 1, [](pair<double,size_t> &a, pair<double,size_t> &b)->bool {return (a.first == b.first);});
  vector< pair<double,size_t> > rand_last_elem (last_it, same_elem_boundary_it + 1);
  random_shuffle(rand_last_elem.begin(), rand_last_elem.end());
  
  for (auto it = rand_last_elem.begin(); it != rand_last_elem.begin() + (candi_size - (last_it - evals.begin())); ++it) {
	candi_tree_idx_groups.push_back(loc_tree_idx_distribution[it->second]);
	original_candidate_group_idxes.push_back(it->second);
  }  

  get_optimal_combinations_of_group(out_combi, eval, candi_tree_idx_groups, candi_size);

}

double PatchCooker::evaluate_distribution( const vector< vector<size_t> > &cluster_tree_idxes_distri_in_a_mot )
{
  if (cluster_tree_idxes_distri_in_a_mot.empty()) return 0.0;

  const double Coff_Number = 1.0;
  const double Coff_Distri = 2.2;

  size_t cnumber = 0;
  size_t cdistri = 0;
  double eval = 0.0;

  for (auto it = cluster_tree_idxes_distri_in_a_mot.begin(); it != cluster_tree_idxes_distri_in_a_mot.end(); ++it) {
	if (it->empty()) continue;

	size_t pre_pos = eeTable->get_cluster_tree_leaf_pee(*it->begin())->get_pos_idx();
	pair<size_t, size_t> pre_legs;	// 하나의 posture에 해당하는 PatchEntryExit가 두 개 존재할 수 있기 때문에 두 개의 PEE를 포함하는 leg indexes를 저장한다.

	if (it->size() == 1 || pre_pos != eeTable->get_cluster_tree_leaf_pee(*(it->begin() + 1))->get_pos_idx()) {
	  pre_legs.first = pre_legs.second = eeTable->get_cluster_tree_leaf_node(*it->begin())->group_idx;
	} else {
	  pre_legs.first = eeTable->get_cluster_tree_leaf_node(*it->begin())->group_idx;
	  pre_legs.second= eeTable->get_cluster_tree_leaf_node(*(it->begin() + 1))->group_idx;
	}
	
	eval += Coff_Number + Coff_Distri;
	cnumber += 1;	  cdistri += 1;
	for (auto jt = it->begin(); jt != it->end(); ++jt) {
	  size_t cur_pos = eeTable->get_cluster_tree_leaf_pee(*jt)->get_pos_idx();

	  if (cur_pos - pre_pos > 1) {
		eval += (Coff_Number);	cnumber += 1;
		size_t cur_leg = eeTable->get_cluster_tree_leaf_node(*jt)->group_idx;
		
		if (pre_legs.first != cur_leg && pre_legs.second != cur_leg) {
		  eval += (Coff_Distri);  
		  cdistri += 1;
		  
		  if ((jt + 1) != it->end()) {			
			if (cur_pos != eeTable->get_cluster_tree_leaf_pee(*(jt + 1))->get_pos_idx()) {
			  pre_legs.first = pre_legs.second = cur_leg;
			} else {
			  pre_legs.first = cur_leg;
			  pre_legs.second= eeTable->get_cluster_tree_leaf_node(*(jt + 1))->group_idx;
			}
		  }	  
		}
	  }
	  pre_pos = cur_pos;
	}
  }
  if (false) 
  {
	size_t t = 1;
	for (auto it = cluster_tree_idxes_distri_in_a_mot.begin(); it != cluster_tree_idxes_distri_in_a_mot.end(); ++it) {
	  t *= it->size();
	}
	eval += static_cast<double>(t);
  }  

  // cout << "Num: " << cnumber << ", Distri: " << cdistri << endl;
  return eval;
}

void PatchCooker::store_pose_info_by_selected_group( const vector< vector< vector<size_t> > > &sel_group, const vector<size_t> &boundary_groups )
{
  for (size_t i = 0; i < sel_group.size(); ++i) {
	eeTable->store_infos_in_pees(sel_group[i], boundary_groups[i]);
  }
}

void PatchCooker::mold_interact_patch()
{
  if (interaction_prober->interactions.empty()) return;

  vector<int> mark(interaction_prober->interactions.size(), 0);
  auto it = mark.begin();
  while (it != mark.end()) {
	vector<Interact_section> beginends;
	auto_covering_interacts(it - mark.begin(), beginends, mark);
	raw_interact_patch.push_back(beginends);
	it = find(mark.begin(), mark.end(), 0);
  }
}

void PatchCooker::auto_covering_interacts( const size_t inter_idx, vector<Interact_section> &interacts_in_a_patch, vector<int> &mark )
{
  if (mark[inter_idx] == 1) return;
  const Interaction &inter = interaction_prober->interactions[inter_idx];

  for (size_t i = 0; i < inter.size(); ++i) {
	const EEGroup *entee = inter.get_leg(i, true);
	const EEGroup *extee = inter.get_leg(i, false);
	if (entee->size() == 0 || extee->size() == 0) {
	  cerr << "error: this leg is empty!" << endl;
	} else {
	  Interact_section interact_shell = auto_covering_section(entee, extee);
	  interacts_in_a_patch.push_back(interact_shell);
	  mark[inter_idx] = 1;
	  if (interact_shell.is_extend.first) {
	    auto_covering_interacts(adj_interaction_idx(entee), interacts_in_a_patch, mark);
	  } 
	  if (interact_shell.is_extend.second) {
	    auto_covering_interacts(adj_interaction_idx(extee), interacts_in_a_patch, mark);
	  } 
	}
  }
}

Interact_section PatchCooker::auto_covering_section( const EEGroup * entry_leg, const EEGroup * exit_leg )
{
  section sec;  
  sec.mot_idx = entry_leg->get_mot_idx();  
  int entry_type_ = max_feature_pos(sec, entry_leg, true);    
  int exit_type_ = max_feature_pos(sec, exit_leg, false);

  Interact_section ret(sec);
  ret.set(entry_type_, exit_type_);
  return ret;
}

int PatchCooker::max_feature_pos( section& bounds_, const EEGroup *leg, bool isentry )
{
  if (leg->size() == 0) {
	cerr << "Critical Error: Empty leg! this interaction " << leg->interaction_idx << " must be merged into adjacent interaction." << endl;
	(isentry ? bounds_.pos_sec.first  : bounds_.pos_sec.second) = 0;
	return -2;
  }
  auto max_bt = leg->begin();
  double max_eval = -1.0, max_sim = 0.0;
  size_t max_dist = 0;  
  for (auto bt = leg->begin(); bt != leg->end(); ++bt) {
	int f_idx = bt->f_info.cluster_group_idx;
	if (f_idx > -1) {  
	  double eval = evaluate_boundary_by_cluster(*bt);
	  if (eval > max_eval) {
		max_eval = eval;
		max_bt = bt;
		max_sim = bt->f_info.pose_distance;
		max_dist = bt->distance_from_interact;
	  }
	}
  }
  if (max_eval < 0.0) {
	cout << "Fitting edge isn't here!" << endl;	
	(isentry ? bounds_.is_extend.first : bounds_.is_extend.second) = true;
	(isentry ? bounds_.pos_sec.first : bounds_.pos_sec.second) = (leg->end() - 1)->get_pos_idx();
	return -1;
  } else {
	cout << max_eval << ": cluster group idx: " << max_bt->f_info.cluster_group_idx <<", max: " << max_sim << ", dist: " << max_dist << endl;
	(isentry ? bounds_.pos_sec.first : bounds_.pos_sec.second) = max_bt->get_pos_idx();
	return max_bt->f_info.cluster_group_idx;
  }
}

double PatchCooker::evaluate_boundary_by_cluster( const PatchEntryExit& bpee )
{
  return (4.0 / (bpee.f_info.pose_distance + 1.0)) + (25.0 / (1.0 + exp(0.05 * static_cast<double>(bpee.distance_from_interact) - 3.5))); 
}

double PatchCooker::eval_molding_boundary_given_poses( size_t f_idx, const PatchEntryExit &bt )
{
  return find_if(feature_grades.begin(), feature_grades.end(), [f_idx](pair<size_t,double> &f)->bool{if (f_idx == f.first) return true; else return false;})->second
		  + 1.0 / bt.f_info.pose_distance + (30.0 / static_cast<double>(bt.distance_from_interact));
}

void PatchCooker::mold_non_interact_patch(const vector< vector<Boundary_group> > &boundary_info)
{
  vector< vector< pair<size_t,size_t> > > all_possible_unary_patch_classified_by_mot(boundary_info.size(), vector< pair<size_t,size_t> >());  
  for (auto it = boundary_info.begin(); it != boundary_info.end(); ++it) {
	if (it->size() < 2) continue;

	vector< pair<size_t,size_t> > all_possible_unary_patch;
	for (size_t j = 0; j < (it->size() - 1); ++j) {
	  size_t post_j = (j + 1);
	  all_possible_unary_patch.push_back(pair<size_t, size_t>(j, post_j));
	}
	all_possible_unary_patch_classified_by_mot[it - boundary_info.begin()] = all_possible_unary_patch;
  }

  if (raw_interact_patch.empty()) {cerr << "Error: mold interact_patch at first." << endl;	return;}

  // remove all unary motion patches included in interaction patches.
  for (auto it = raw_interact_patch.begin(); it != raw_interact_patch.end(); ++it) {
	for (auto jt = it->begin(); jt != it->end(); ++jt) {	  
	  auto &possible_unary_patch = all_possible_unary_patch_classified_by_mot[jt->motion_idx()];
	  if (possible_unary_patch.empty()) continue;
	  
	  for (auto kt = possible_unary_patch.begin(); kt != possible_unary_patch.end(); ) {
		pair<size_t,size_t> shortest_sect(boundary_info[jt->motion_idx()][kt->first].sect.end_pos(), boundary_info[jt->motion_idx()][kt->second].sect.begin_pos());
		
		if (length_apart(jt->pos_sec, shortest_sect) == -1 * static_cast<int>(length_section(shortest_sect))) {
		  //cout << "motion ID: " << jt->motion_idx() << ", (" << jt->pos_sec.first << ", " << jt->pos_sec.second << ")  >  (" << shortest_sect.first << ", " << shortest_sect.second << ")" << endl;
		  kt = possible_unary_patch.erase(kt);
		} else {
		  ++kt;
		}	  
	  }
	}
  }

  for (auto mt = all_possible_unary_patch_classified_by_mot.begin(); mt != all_possible_unary_patch_classified_by_mot.end(); ++mt) {
	size_t mot_idx = mt - all_possible_unary_patch_classified_by_mot.begin();
	for (size_t i = 0; i < all_possible_unary_patch_classified_by_mot[mot_idx].size(); ++i) {
	  Interact_section new_obj;
	  new_obj.mot_idx = mot_idx;
	  new_obj.boundary_type = pair<int, int>(boundary_info[mot_idx][mt->at(i).first].idx, boundary_info[mot_idx][mt->at(i).second].idx);
	  
	  double min_dist = 10000.0;
	  size_t min_i = (boundary_info[mot_idx][mt->at(i).first].sect.begin_pos() + boundary_info[mot_idx][mt->at(i).first].sect.end_pos()) / 2;
	  const auto &pees = boundary_info[mot_idx][mt->at(i).first].pee_idxes;
	  for (auto it = pees.begin(); it != pees.end(); ++it) {
		double gdist = eeTable->evaluate_distance_in_group(*eeTable->get_cluster_tree_leaf_pee(*it), new_obj.boundary_type.first);
		if (min_dist > gdist) {
		  min_dist = gdist;
		  min_i = eeTable->get_cluster_tree_leaf_pee(*it)->get_pos_idx();
		}
	  }
	  new_obj.pos_sec.first = min_i;

	  min_dist = 10000.0;
	  min_i = (boundary_info[mot_idx][mt->at(i).second].sect.begin_pos() + boundary_info[mot_idx][mt->at(i).second].sect.end_pos()) / 2;
	  const auto &peess = boundary_info[mot_idx][mt->at(i).second].pee_idxes;
	  for (auto it = peess.begin(); it != peess.end(); ++it) {
		double gdist = eeTable->evaluate_distance_in_group(*eeTable->get_cluster_tree_leaf_pee(*it), new_obj.boundary_type.second);
		if (min_dist > gdist) {
		  min_dist = gdist;
		  min_i = eeTable->get_cluster_tree_leaf_pee(*it)->get_pos_idx();
		}
	  }
	  new_obj.pos_sec.second = min_i;

	  raw_non_interact_patch.push_back(new_obj);
	}
  }  
}

vector< vector<Boundary_group> > 
  PatchCooker::convert_sorting_criteria_from_groupidx_to_motionidx( const vector< vector< vector<size_t> > > &group_distri, const vector<size_t> &group_idx )
{
  vector< vector<Boundary_group> > ret;
  if (group_distri.empty()) return ret;

  const size_t MotSize = size_mots();

  for (size_t m = 0; m < MotSize; ++m) {
	vector<Boundary_group> boundary_group_in_same_motion;

	for (auto it = group_distri.begin(); it != group_distri.end(); ++it) {
	  auto boundary_bunble = it->at(m);
	  if (boundary_bunble.empty()) continue;

	  Boundary_group boundary_group_repo;
	  boundary_group_repo.idx = group_idx[it - group_distri.begin()];
	  boundary_group_repo.sect.mot_idx = m;
	  auto bt = boundary_bunble.begin();
	  boundary_group_repo.pee_idxes.push_back(*bt);
	  int pre_idx = static_cast<int>(eeTable->get_cluster_tree_leaf_pee(*(bt++))->get_pos_idx());

	  boundary_group_repo.sect.pos_sec.first = pre_idx;	  	  
	  for (; bt != boundary_bunble.end(); ++bt) {
		int cur_idx = static_cast<int>(eeTable->get_cluster_tree_leaf_pee(*bt)->get_pos_idx());
		boundary_group_repo.pee_idxes.push_back(*bt);
		if ((cur_idx - pre_idx) < 0) cerr << "Error! group distribution was not sorted!" << endl;
		if ((cur_idx - pre_idx) > 1) {
		  boundary_group_repo.sect.pos_sec.second = pre_idx;
		  boundary_group_repo.pee_idxes.pop_back();
		  boundary_group_in_same_motion.push_back(boundary_group_repo);
		  boundary_group_repo.pee_idxes.clear();

		  boundary_group_repo.sect.pos_sec.first = cur_idx;
		  boundary_group_repo.pee_idxes.push_back(*bt);
		} 
		pre_idx = cur_idx;
	  } 
	  boundary_group_repo.sect.pos_sec.second = pre_idx;

	  if (boundary_group_in_same_motion.empty() || (boundary_group_in_same_motion.back() != boundary_group_repo)) {
		boundary_group_in_same_motion.push_back(boundary_group_repo);
	  }
	}
	ret.push_back(boundary_group_in_same_motion);
  }
  for (auto it = ret.begin(); it != ret.end(); ++it) {
	sort(it->begin(), it->end(), [](Boundary_group &a, Boundary_group &b)->bool{ if (a.sect.begin_pos() < b.sect.begin_pos()) return true; else return false;});
  }

  return ret;
}

void PatchCooker::mold_patch_edges( const vector<pair_section> &pair_sects )
{
  for (auto it = pair_sects.begin(); it != pair_sects.end(); ++it) {
	molding_bounds.push_back(direct_covering_pair_section(*it, 3, 3));
  }  
}

void PatchCooker::set_cross_sects( const size_t pos, const ml::Motion &m )
{
  PosturePoints f(m, pos);
  cross_sect_features.push_back(f);
}

vector<pair_section> PatchCooker::relation_pairs()
{
  vector<pair_section> ret;
  auto inters = interaction_prober->interactions;
  for (auto it = inters.begin(); it != inters.end(); ++it) {
	copy(it->relations.begin(), it->relations.end(), back_inserter(ret));
  }
  return ret;
}

size_t PatchCooker::size_interactions()
{
  size_t count = 0;
  for (auto it = interaction_prober->interactions.begin(); it != interaction_prober->interactions.end(); ++it) {
	count += it->size();
  }
  return count;
}

size_t PatchCooker::adj_interaction_idx( const EEGroup *eeg )
{
  size_t mot_idx = eeg->get_mot_idx();
  pair<size_t,size_t> eeg_pos_section;	  // double timing = (eeg->begin() + eeg->size() / 2)->it_posture()->time;
  eeg_pos_section.first = (eeg->begin()->get_pos_idx() > (eeg->end() - 1)->get_pos_idx()) ? (eeg->end() - 1)->get_pos_idx() : eeg->begin()->get_pos_idx();
  eeg_pos_section.second= (eeg->begin()->get_pos_idx() > (eeg->end() - 1)->get_pos_idx()) ? eeg->begin()->get_pos_idx() : (eeg->end() - 1)->get_pos_idx();
  bool adj_type;
  if (eeg->type == EEGroup::Entry) {
	adj_type = false;
  } else if (eeg->type == EEGroup::Exit) {
	adj_type = true;
  } 
 
  if (eeg->begin()->is_included_in_ref_mots) {
	for (auto it = interaction_prober->interactions.begin(); it != interaction_prober->interactions.end(); ++it) {
	  if ((it - interaction_prober->interactions.begin()) != eeg->get_i_idx()) {
		for (size_t j = 0; j < it->size(); ++j) {
		  auto leg = it->get_leg(j, adj_type);
		  if (leg->is_containing(mot_idx, eeg_pos_section)) {
			return leg->get_i_idx();
		  }
		}
	  }
	}
  } else {
	size_t bundleidx = ingredients->map_mot_to_bundle(eeg->get_mot_idx()).first;
	for (auto it = interaction_prober->interactions.begin(); it != interaction_prober->interactions.end(); ++it) {
	  if (ingredients->interact_bunch_to_bunble[it - interaction_prober->interactions.begin()].first == bundleidx) {
		if ((it - interaction_prober->interactions.begin()) != eeg->get_i_idx()) {
		  for (size_t j = 0; j < it->size(); ++j) {
			auto leg = it->get_leg(j, adj_type);
			if (leg->is_containing(mot_idx, eeg_pos_section)) {
			  return leg->get_i_idx();
			}
		  } 
		}
	  }
	}	
  }

  // if there is no adjacent interaction, then return your interaction index
  return eeg->get_i_idx();
}

void PatchCooker::test_adj_interaction_idx()
{
  size_t tmp_idx = 2;
  size_t bg = eeTable->boundary_group[tmp_idx].get_i_idx();  
  size_t adjidx = adj_interaction_idx(&eeTable->boundary_group[tmp_idx]);
  cout << bg << ", "<< adjidx << endl;

  for (size_t i = 0; i < interaction_prober->interactions[adjidx].size(); ++i) {
	cout << adj_interaction_idx(interaction_prober->interactions[adjidx].get_leg(i, true)) << " ";
	cout << adj_interaction_idx(interaction_prober->interactions[adjidx].get_leg(i, false)) << " ";
  }
}

void PatchCooker::remove_similar_non_interact_patch( vector<Interact_section> &raw_non_interact_patch )
{  
  if (raw_non_interact_patch.empty()) return;

  vector<Interact_section> swap_temp;  
  while (raw_non_interact_patch.size() > 1) {
	vector<int> mark(raw_non_interact_patch.size(), 0);
	
	auto it = raw_non_interact_patch.begin();	
	for (auto jt = (it + 1); jt != raw_non_interact_patch.end(); ++jt) {
	  if (is_similar(*it, *jt)) mark[jt - raw_non_interact_patch.begin()] = 1; 
	}
	for (int inv = mark.size() - 1; inv > 0; --inv) {
	  if (mark[inv] == 1) {
		raw_non_interact_patch.erase(raw_non_interact_patch.begin() + inv);
	  }
	} 
	swap_temp.push_back(*raw_non_interact_patch.begin());
	raw_non_interact_patch.erase(raw_non_interact_patch.begin());
  }

  if (!raw_non_interact_patch.empty()) swap_temp.push_back(*raw_non_interact_patch.begin());
  raw_non_interact_patch = swap_temp;  
}

vector< shared_ptr<Patch> > PatchCooker::get_all_patches()
{
  vector< shared_ptr<Patch> > ret;
  double t = 0.0;
  for (auto it = patches->begin(); it != patches->end(); ++it) {
	ret.push_back(shared_ptr<Patch>(new Patch(it->second)));
	/*ret.back()->translate(cml::vector3(5.0 * t, 0.0, 0.0));
	t += 1.0;*/
  }  
  set_color(ret);
  return ret;
}

void PatchCooker::evaluate_this_system()
{
  cout << endl;
  cout << "interaction Patch: " << patches->size() - unary_patches->size() << endl;
  cout << "non-interaction Patch: " << unary_patches->size() << endl;
  cout << "the # of boundary types: " << boundaryTypeSize << " and the types are";
  for (auto it = boundary_type.begin(); it != boundary_type.end(); ++it) cout << " " << *it;  cout << endl;
  cout << endl;

  // 패치 별로 경계 종류 
  for (auto it = patches->begin(); it != patches->end(); ++it) {
	if (it->second.motions.size() > 1) {
	  cout << "name: " << it->second.name << " --> ";
	  for (size_t j = 0; j < it->second.boundaries.size(); ++j) {
		cout << it->second.boundaries[j].posture_type() << " "; 
	  } cout << endl;	  
	}
  }
  // 생존 패치 개수
  size_t count_ = 0;
  for (auto it = patches->begin(); it != patches->end(); ++it) {
	if (it->second.motions.size() > 1) {
	  bool check = false;
	  for (auto jt = it->second.boundaries.begin(); jt != it->second.boundaries.end(); ++jt) {
		if (jt->posture_type() == -1) check = true;
	  }
	  if (!check) ++count_;
	}
  } cout << "survived patch #: " << count_ << endl << endl;
  
  // 개별 패치로 연결할 수 있는 패치 수
  count_ = 0;
  for (auto it = unary_patches->begin(); it != unary_patches->end(); ++it) {
	cout << it->name << " : ";
	cout << it->boundaries[0].posture_type() << " --> ";
	cout << it->boundaries[1].posture_type();
	++count_;
	cout << endl;
  } cout << "Unary patch #: " << count_ << endl << endl;
}

void PatchCooker::set_boundary_type_size( size_t btype )
{
  boundaryTypeSize = btype;
}

void PatchCooker::set_boundary_type( const vector<size_t> &selected_groups )
{
  boundary_type.clear();
  boundary_type = selected_groups;
}

size_t PatchCooker::size_mots()
{
  if (ref_mots != nullptr) {
	return ref_mots->size();
  } else {
	return ingredients->size_mot();
  }
}

void PatchCooker::remove_timid_patch( vector<Interact_section> & upatch)
{
  for (auto it = upatch.begin(); it != upatch.end(); ) {
	if (length_section(it->pos_sec) < 10) {
	  it = upatch.erase(it);
	} else {
	  ++it;
	}
  }
}

void PatchCooker::get_optimal_combinations_of_group( vector<int> &out_combi, vector<double> &eval, const vector< vector< vector<size_t> > > &tree_idx_groups, size_t candi_size )
{
  if (tree_idx_groups.size() != candi_size) {cerr << "Error in arg 2: the # of out_groups!" << endl; return;}

  out_combi.clear();  
  
  vector<double> best_eval(candi_size, 0.0);
  for (size_t i = 1; i <= candi_size; ++i) {
	vector<int> comb(i, -1);
	best_eval[i - 1] = get_optimal_combination(comb, candi_size, i, tree_idx_groups);
	if (best_eval[i - 1] == 0.0) { cout << "There are no boundaries that the size of candidates is " << i << endl;}
	copy(comb.begin(), comb.end(), back_inserter(out_combi));
  }
  eval = best_eval;
}

double PatchCooker::get_optimal_combination( vector<int> &comb, size_t n, size_t r, const vector< vector< vector<size_t> > > &out_groups )
{
  vector<int> local_comb_n, local_comb_r;
  for (int i = 0; i < n; ++i) local_comb_n.push_back(i);  
  for (int i = 0; i < r; ++i) local_comb_r.push_back(i);
  
  double optimal_val = 0.0;
  vector<int> optimal_comb(comb.size(), -1);
  do {
	vector< vector < vector<size_t> > > comb_groups;
	for (auto it = local_comb_r.begin(); it != local_comb_r.end(); ++it) {
	  auto distri = out_groups[*it];
	  comb_groups.push_back(distri);
	}
	double local_val = evaluate_combinational_boundaries(comb_groups);
	if (optimal_val < local_val) {
	  optimal_val = local_val;	  optimal_comb = local_comb_r;
	}
  } 
  while (stdcomb::next_combination(local_comb_n.begin(), local_comb_n.end(), local_comb_r.begin(), local_comb_r.end()));

  comb = optimal_comb;
  return optimal_val;
}

double PatchCooker::evaluate_combinational_boundaries( const vector< vector < vector<size_t> > > &comb_groups )
{
  // check postures
  for (auto it = comb_groups.begin(); it != comb_groups.end(); ++it) {
	eeTable->store_cluster_group_in_pees(*it, 1);
  }
  double total_eval = 0.0;
  
  for (size_t i = 0; i < interaction_prober->interactions.size(); ++i) {
	const Interaction &inter = interaction_prober->interactions[i];
	bool cut_interact_patch = true;
	double compactness = 0.0;

	for (size_t j = 0; j < inter.size(); ++j) {	  
	  const EEGroup *entee = inter.get_leg(j, true);
	  const EEGroup *extee = inter.get_leg(j, false);
	  
	  auto it = entee->begin();
	  for (; it != entee->end(); ++it) {
		if (it->f_info.cluster_group_idx == 1) {
		  compactness += (1.0 / (1.0 + exp(0.2 * static_cast<double>(it - entee->begin()) - 6.0)));
		  break;
		}
	  }
	  if (it == entee->end()) {
		cut_interact_patch = false;
		break;
	  }

	  auto jt = extee->begin();
	  for (; jt != extee->end(); ++jt) {
		if (jt->f_info.cluster_group_idx == 1) {
		  compactness += (1.0 / (1.0 + exp(0.2 * static_cast<double>(jt - extee->begin()) - 6.0)));
		  break;
		}
	  }
	  if (jt == extee->end()) { 
		cut_interact_patch = false;
		break;
	  }
	}
	if (cut_interact_patch) {
	  total_eval += 100.0;
	  total_eval += compactness / static_cast<double>(inter.size());
	}
  }

  // de-check postures
  for (auto it = comb_groups.begin(); it != comb_groups.end(); ++it) {
	eeTable->store_cluster_group_in_pees(*it, -1);
  }
  return total_eval;
}


void PatchCooker::agglomerative_cluster( double sim_thres /*= 3.0*/, size_t bSize /*= 6*/ )
{
   eeTable->agglomerative_tree_construction(); 
   //set_boundary_type_size(bSize); 
   eeTable->group_cluster_tree_by_dist(sim_thres, bSize);
}

void PatchCooker::cook_interact_patches()
{
  mold_interact_patch();

  if (raw_interact_patch.empty()) return;

  for (auto it = raw_interact_patch.begin(); it != raw_interact_patch.end(); ++it) {
	fill_interact_section_gap(*it, 1);	
	std::ostringstream oss;
	oss << it - raw_interact_patch.begin();
	string name(oss.str());	
	cook_a_patch(*it, name);	
	sampling_patches.push_back(name);
  }
}

void PatchCooker::cook_unary_patches(const vector< vector<Boundary_group> > & distri)
{
  mold_non_interact_patch(distri);

  if (raw_non_interact_patch.empty()) return;

  remove_timid_patch(raw_non_interact_patch);  
  remove_similar_non_interact_patch(raw_non_interact_patch);  

  for (auto nit = raw_non_interact_patch.begin(); nit != raw_non_interact_patch.end(); ++nit) {
	std::ostringstream oss;
	oss << "u" << nit - raw_non_interact_patch.begin();
	string name(oss.str());
	cook_a_patch(*nit, name);
	sampling_patches.push_back(name);
  }
}

void PatchCooker::tcook_motion_patches()
{
  set_boundary_type_size(4);
  auto selected_pose_distribution = most_distributed_select_poses(); 
  cook_interact_patches();
  //cook_unary_patches(selected_pose_distribution);
}


vector< vector<Boundary_group> > PatchCooker::most_distributed_select_poses()
{
  vector< vector< vector<size_t> > > selected_group_distri_cluster_tree_idx;
  vector< pair<double,size_t> > eval_selected_group_distri;  
  select_most_distributed_groups(selected_group_distri_cluster_tree_idx, eval_selected_group_distri, boundaryTypeSize);
  
  vector<size_t> selected_groups;
  for_each(eval_selected_group_distri.begin(), eval_selected_group_distri.end(), [&selected_groups](pair<double,size_t> &a){ selected_groups.push_back(a.second); });
  set_boundary_type(selected_groups);
  store_pose_info_by_selected_group(selected_group_distri_cluster_tree_idx, boundary_type);

  return convert_sorting_criteria_from_groupidx_to_motionidx(selected_group_distri_cluster_tree_idx, selected_groups);
}

void PatchCooker::Test_molding( size_t num_boundary, size_t num_candi )
{
  vector< vector< vector<size_t> > > selected_cluster_tree_idx;
  vector<int> seq_combi_groups;
  vector<double> comb_eval;

  set_boundary_type_size(num_boundary);
  select_best_combinations_of_groups(selected_cluster_tree_idx, seq_combi_groups, comb_eval, num_candi);
}


void PatchCooker::comb_cook_motion_patches(size_t sel)
{
  auto selected_pose_distribution = combi_distribution_of_select_poses(sel); 
  cook_interact_patches();
  cook_unary_patches(selected_pose_distribution);
}

vector< vector<Boundary_group> > PatchCooker::combi_distribution_of_select_poses(size_t sel)
{
  vector< vector< vector<size_t> > > candi_group_distri_cluster_tree_idx;
  vector<int> best_comb_accum;
  vector<double> comb_eval;

  select_best_combinations_of_groups(candi_group_distri_cluster_tree_idx, best_comb_accum, comb_eval, 10);
  
  size_t comb_size = sel;
  if (comb_size == 0) {
	auto mit = max_element(comb_eval.begin(), comb_eval.end());
	for (size_t i = 0; i < comb_eval.size(); ++i) {
	  cout << comb_eval[i] << " ";
	} cout << endl;
	comb_size = (mit - comb_eval.begin()) + 1;
  }

  set_boundary_type_size(comb_size);
  cout << "Combination size is " << comb_size << endl;

  size_t jump = 0;
  for (size_t i = 1; i < comb_size; ++i) jump += i;

  vector<size_t> selected_groups;
  vector<size_t> selected_original_groups;
  for (size_t i = 0; i < comb_size; ++i) {
	selected_groups.push_back(best_comb_accum[jump + i]);
	selected_original_groups.push_back(original_candidate_group_idxes[best_comb_accum[jump + i]]);
  }

  set_boundary_type(selected_original_groups);
  vector< vector< vector<size_t> > > selected_group_distri_cluster_tree_idx;
  for (auto it = selected_groups.begin(); it != selected_groups.end(); ++it) {
	selected_group_distri_cluster_tree_idx.push_back(candi_group_distri_cluster_tree_idx[*it]);
  }
  
  //eeTable->test_pee_init();
  store_pose_info_by_selected_group(selected_group_distri_cluster_tree_idx, boundary_type);

  return convert_sorting_criteria_from_groupidx_to_motionidx(selected_group_distri_cluster_tree_idx, boundary_type);
}

