#include "StdAfx.h"
#include "cook_util.h"


PosturePoints::PosturePoints( const ml::Motion &m, const size_t pos )
  :orient(m[pos].rotate(0))
{
  joints.reserve(m.size());
  for (size_t i = 0; i < m.body()->num_joint(); ++i) {
	joints.push_back(m[pos].GetGlobalTranslation(i));
  }  
  contact_foot[0] = m[pos].leftFootTouched;
  contact_foot[1] = m[pos].rightFootTouched;
  
  size_t cur = pos;
  if (pos == 0) cur = 1;
  
  for (size_t i = 0; i < m.body()->num_joint(); ++i) {
	velocitys.push_back(m[cur].GetGlobalTranslation(i) - m[cur - 1].GetGlobalTranslation(i));
  }

  is_forwarding = (dot(shoulder_orientation(m[cur]), m[cur].trans() - m[cur - 1].trans()) > 0) ? true : false;  
}

double weighted_distance( const PosturePoints &a, const PosturePoints &b, const vector<double> &weights )
{
  if (a.size() != b.size()) {
	cerr << "error in point_cloud!" << endl;
	return -1.0;
  } else {	
	double dist = 0.0;
	if (a.is_forwarding != b.is_forwarding) dist += 5000.0;

	if (!is_same_contact_state(a, b)) dist += 5000.0;

	for (size_t i = 0; i < a.size(); ++i) {
	  dist += weights[i] * (a[i] - b[i]).length();
	}
	
	//if (dot(a.velocitys[0], b.velocitys[0]) < 0.0) dist += 5000.0;

	double v_dist = 0.0;
	for (size_t i = 0; i < a.size(); ++i) {
	  v_dist += weights[i] * (a.velocitys[i] - b.velocitys[i]).length();
	}
	v_dist *= 2.0;
	//cout << "dist : " << dist << ", velocity : " << v_dist << endl;
	return (dist + std::abs(v_dist));
  }  
}

double pose_difference( const PosturePoints &a, const PosturePoints &b )
{
  vector<double> w(a.size(), 1.0);
  return pose_weighted_difference(a, b, w);
}

double pose_weighted_difference( const PosturePoints &a, const PosturePoints &b, const vector<double> &weights )
{
  PosturePoints t = a;
  align_points_to_other(t, b);
  return weighted_distance(t, b, weights);
}


PatchEntryExit::PatchEntryExit( size_t m, size_t p, size_t dist, vector<ml::Motion> *ms )
  : ref_m(ms), distance_from_interact(dist), mot_pos_idx(pair<size_t,size_t>(m, p)), ref_ingre(nullptr), is_included_in_ref_mots(true)
{
  f_info.cluster_group_idx = -1;
  f_info.pose_distance = -1.0;
  pcloud = get_origin_pcloud();
}

PatchEntryExit::PatchEntryExit( size_t bundle_idx, size_t m, size_t p, size_t dist, Ingredient *ingre )
  : ref_m(nullptr), distance_from_interact(dist), bundle_mot_pos_idx(pair< size_t, pair<size_t,size_t> >(bundle_idx, pair<size_t,size_t>(m, p))), ref_ingre(ingre)
  , is_included_in_ref_mots(false)
{
  f_info.cluster_group_idx = -1;
  f_info.pose_distance = -1.0;
  pcloud = get_origin_pcloud();
}

const ml::Motion & PatchEntryExit::get_motion() const
{
  if (ref_m != nullptr) { 
	return ref_m->at(mot_pos_idx.first);
  } else {
	return ref_ingre->raw_before_cook[bundle_mot_pos_idx.first][bundle_mot_pos_idx.second.first];
  }
}

ml::Motion::Const_iterator PatchEntryExit::it_posture() const
{
   if (ref_m != nullptr) { 
	 return ((*ref_m)[mot_pos_idx.first].begin() + mot_pos_idx.second);
   } else {
	 return ref_ingre->raw_before_cook[bundle_mot_pos_idx.first][bundle_mot_pos_idx.second.first].begin() + bundle_mot_pos_idx.second.second;
   }
}

ml::Motion::Iterator PatchEntryExit::it_posture()
{
  if (ref_m != nullptr) { 
	return ((*ref_m)[mot_pos_idx.first].begin() + mot_pos_idx.second);
  } else {
	return ref_ingre->raw_before_cook[bundle_mot_pos_idx.first][bundle_mot_pos_idx.second.first].begin() + bundle_mot_pos_idx.second.second;
  }
}

double shape_difference( const PatchEntryExit & e2, const PatchEntryExit & e1 )
{
  return pose_weighted_difference(e1.get_pcloud(), e2.get_pcloud(), extract_joint_weight(e1.get_motion()));
}

double shape_difference( const PosturePoints &p, const PatchEntryExit & pee )
{
  return pose_weighted_difference(p, pee.get_pcloud(), extract_joint_weight(pee.get_motion()));
}

void transl_to_origin( PatchEntryExit & pee )
{  
  pee.get_pcloud().transl(-1 * plane_vec3(pee.it_posture()->GetGlobalTranslation(0)));
}

void view_similar_frame( EEGroup &eeg, const PosturePoints &p )
{
  for (auto it = eeg.begin(); it != eeg.end(); ++it) {
	if (shape_difference(p, *it) < Pose_Similarity) {
	  it->it_posture()->set_status_in_viewer(true, cml::vector3(0.0, 1.0, 0.));
	  //cout << "( " << eeg.get_mot_idx() << ", " <<  it->it_posture()->time << ")" << endl;
	}
  }
}

void align_points_to_other( PatchEntryExit & from, const PatchEntryExit & to )
{
  transl_to_origin(from);
  from.get_pcloud().rotate(cml::PlaneProject_mat3(to.it_posture()->rotate(0) * cml::inverse(from.it_posture()->rotate(0))));
  from.get_pcloud().transl(plane_vec3(to.it_posture()->trans()));
}

void align_points_to_other( PosturePoints & from, const PosturePoints & to )
{
  cml::vector3 transl_ = from[0];
  transl_[1] = 0.0;
  from.transl(-1 * transl_);
  from.rotate(cml::PlaneProject_mat3(to.orient * cml::inverse(from.orient))); 
  from.transl(plane_vec3(to[0]));
}

PosturePoints operator*( const int r, PosturePoints & pc )
{
  return pc.operator *=(r);
}

PosturePoints operator*( PosturePoints & pc, const int r )
{
  return pc.operator *=(r);
}

PointsCloud::PointsCloud( const ml::Motion &mot, const size_t s, const size_t e )
  : weights(extract_joint_weight(mot))
{
  for (size_t i = s; i <= e; ++i) {
	PosturePoints pp(mot, i);
	points_cloud.reserve(e - s + 1);
	points_cloud.push_back(pp);
  }
}

double distance_Kovar_metric( const PointsCloud &a, const PointsCloud &b )
{
  size_t cloud_num = a.size_cloud();
  size_t pos_num = a[0].size();
  size_t n = cloud_num * pos_num;  
  ASSERT(b.size_cloud() * b[0].size() == n);
  
  // Refer SnapTogetherMotion.pdf by H.J.Shin
  double theta = 0.0, x0 = 0.0, z0 = 0.0;

  // Calculate theta
  double xbar = 0.0, x2bar = 0.0, zbar = 0.0, z2bar = 0.0;
  enum {X = 0, Y = 1, Z = 2};  
  double sum_weights = 0.0;
  for_each(a.weights.begin(), a.weights.end(), [&sum_weights](double w){sum_weights += w;});
  sum_weights *= static_cast<double>(a.size_cloud());
  vector<double> weights = a.weights;  
  for_each(weights.begin(), weights.end(), [sum_weights](double &w){w /= sum_weights;});
  
  for (size_t i = 0; i < cloud_num; ++i) {
	for (size_t j = 0; j < pos_num; ++j) {
	  xbar += weights[j] * a[i][j][X];
	  x2bar+= weights[j] * b[i][j][X];
	  zbar += weights[j] * a[i][j][Z];
	  z2bar+= weights[j] * b[i][j][Z];
	}	
  }

  double sumA = 0.0, sumB = 0.0;
  for (size_t i = 0; i < cloud_num; ++i) {
	for (size_t j = 0; j < pos_num; ++j) {
	  double xi = a[i][j][X];
	  double zi = a[i][j][Z];
	  double xi2 = b[i][j][X];
	  double zi2 = b[i][j][Z];
	  sumA += weights[j] * (xi * zi2 - xi2 * zi);
	  sumB += weights[j] * (xi * xi2 + zi * zi2);
	}
  }  
  sumA -= (xbar * z2bar - x2bar * zbar);
  sumB -= (xbar * x2bar + zbar * z2bar);
  theta = atan2(sumA, sumB);

  // calculate x0 and z0
  x0 = xbar - x2bar * cos(theta) - z2bar * sin(theta);
  z0 = zbar + x2bar * sin(theta) - z2bar * cos(theta);
  cout << "theta " << theta << ", x0 " << x0 << ", z0 " << z0 << endl;
  
  // calculate matching distance
  PointsCloud transformed_b(b);
  for (size_t i = 0; i < transformed_b.size_cloud(); ++i) {
	transformed_b[i].rotate(cml::vector3(0., theta, 0.));
	transformed_b[i].transl(cml::vector3(x0, 0., z0));
  }
  double distance = 0.0;
  for (size_t i = 0; i < cloud_num; i++) {
	double temp = weighted_distance(a[i], transformed_b[i], a.weights);
	distance += (temp * temp);
  }

  return sqrt(distance);
}


Node* get_root( Node* t )
{
  Node *trav = t; 
  while (trav->parent != nullptr) {
	trav = trav->parent;
  }
  return trav;
}

void EntryExitTable::transl_to_origin( const size_t eidx )
{
  PatchEntryExit &edge = get_boundary(eidx);
  ::transl_to_origin(edge);
}

void EntryExitTable::transl_to_origin( size_t b, size_t pee )
{
  PatchEntryExit &edge = boundary_group[b].entry_exits[pee];
  ::transl_to_origin(edge);
}

pair<size_t,size_t> min_value_idx(const vector< vector<double> > & distmat)
{
  pair<size_t, size_t> ret(0, 0);
  double min_v = 100000.0;
  for (size_t i = 0; i < distmat.size() - 1; ++i) {
	for (size_t j = i + 1; j < distmat[i].size(); ++j) {
	  if (distmat[i][j] < min_v) {
		min_v = distmat[i][j];
		ret.first = i;	ret.second = j;
	  }
	}
  }
  return ret;
}

void get_group_member( vector<size_t> &g, Node *root )
{
  g = root->get_generations_idx();
}

void EntryExitTable::agglomerative_tree_construction()
{
  size_t size_candi = size_candidates();
  cout << "cluster node # is " << size_candi << endl;
  cluster_tree_leaf_node.reserve(size_candi);
  
  size_t count = 0;
  for (auto it = boundary_group.begin(); it != boundary_group.end(); ++it) {
	for (auto jt = it->begin(); jt != it->end(); ++jt) {
	  Terminal term(it - boundary_group.begin(), jt - it->begin(), &(*jt));
	  term.n_th = count;
	  cluster_tree_leaf_node.push_back(new Terminal(term));
	  ++count;
	}
  }
  
  vector< vector<double> > distMat;
  vector<double> row(size_candi, 0);  
  distMat.resize(size_candi, row);
  for (size_t i = 0; i < size_candi - 1; ++i) {
	for (size_t j = i + 1; j < size_candi; ++j) {
	  distMat[i][j] = distMat[j][i] = shape_difference(*cluster_tree_leaf_node[i]->ptr_pee, *cluster_tree_leaf_node[j]->ptr_pee);
	}
  }

  root_of_clustering = nullptr;
  while (root_of_clustering == nullptr || root_of_clustering->size != size_candi) {
	auto min_idx = min_value_idx(distMat);
	root_of_clustering = build_parent_tree(min_idx, cluster_tree_leaf_node, distMat);
  }  
}

void check(vector<int> &mark, const vector<size_t> group)
{
  for (auto it = group.begin(); it != group.end(); ++it) {
	mark[*it] = 1;
  }
}

bool is_same_contact_state( const PosturePoints& a, const PosturePoints& b )
{
  if ((a.contact_foot[0] && a.contact_foot[1]) || (b.contact_foot[0] && b.contact_foot[1])) {
	return false;
  } else if (a.contact_foot[0] == b.contact_foot[0] && a.contact_foot[1] == b.contact_foot[1]) {
	return true; 
  } else {
	return false;
  }

  // for chicken example
  /*if (a.contact_foot[1] == true && a.contact_foot[1] == b.contact_foot[1]) {
	return true; 
  } else {
	return false;
  }*/
}

Clustering * EntryExitTable::build_parent_tree( pair<size_t,size_t> idx, vector< Terminal* > &ctree, vector< vector<double> > &distM )
{
  Clustering* cnode = new Clustering;
  Node* trav1 = get_root(&(*ctree[idx.first]));
  Node* trav2 = get_root(&(*ctree[idx.second]));  

  trav1->parent = cnode;
  trav2->parent = cnode;

  vector<size_t> g1, g2;
  get_group_member(g1, trav1); 
  get_group_member(g2, trav2);
  
  cnode->size = trav1->size + trav2->size;
  cnode->siblings.push_back(trav1);	
  cnode->siblings.push_back(trav2);
  cnode->max_dist = max_distance_cluster(g1, g2, distM);  
  
  replace_distance_mat(distM, g1, g2, 100000.0);  
  vector<size_t> merged(g1.begin(), g1.end());
  copy(g2.begin(), g2.end(), back_inserter(merged));
  update_distance_mat(distM, merged, ctree); 

  return cnode;
}

double EntryExitTable::max_distance_cluster( const vector<size_t> &group1, const vector<size_t> &group2, const vector< vector<double> > &distM )
{
  double max_dist = 0.0;
  pair<size_t,size_t> max_pair(0, 0);
  for (size_t i = 0; i < group1.size(); ++i) {
	for (size_t j = 0; j < group2.size(); ++j) {
	  if (distM[group1[i]][group2[j]] > max_dist) {
		max_dist = distM[group1[i]][group2[j]];
		max_pair.first = group1[i];	max_pair.second = group2[j];
	  }
	}
  } return max_dist;
}

void EntryExitTable::replace_distance_mat( vector< vector<double> > &dM, const vector<size_t> &g1, const vector<size_t> &g2, const double val )
{
  for (auto it = g1.begin(); it != g1.end(); ++it) {
	for (auto jt = g2.begin(); jt != g2.end(); ++jt) {
	  dM[*it][*jt] = dM[*jt][*it] = val;	   
	}
  }  
}

void EntryExitTable::update_distance_mat( vector< vector<double> > &distM, const vector<size_t> &group, vector< Terminal* > & ctree )
{
  vector<int> mark(distM.size(), 0);
  for (size_t i = 0; i < group.size(); ++i) {mark[group[i]] = 1;} 

  for (auto mt = mark.begin(); mt != mark.end(); ++mt) {
	if (*mt == 0) {
	  vector<size_t> ng;
	  Node * nr = get_root(&(*ctree[mt - mark.begin()]));
	  get_group_member(ng, nr);
	  double ndist = max_distance_cluster(group, ng, distM);
	  replace_distance_mat(distM, group, ng, ndist);
	  check(mark, ng);
	}
  }
}

double EntryExitTable::evaluate_cross_sect_given_pose( const size_t p_idx, const PosturePoints &pcloud, const size_t inter_num )
{  
  if (boundary_group.empty()) {
	cerr << "please, initialize EETable first!" << endl;	
	return -1.0;
  } else {
	vector<int> entries(inter_num, 0);
	vector<int> exits(inter_num, 0);
	double many = 0.0;
	double distribution = 0.0;
	for (auto it = boundary_group.begin(); it != boundary_group.end(); ++it) {
	  for (auto jt = it->begin(); jt != it->end(); ++jt) {
		double unlikeness = shape_difference(pcloud, *jt);		
		if (Pose_Similarity > unlikeness) {
		  if (it->get_type() == EEGroup::Entry) {
			entries[it->get_i_idx()] = 1;
		  } else if (it->get_type() == EEGroup::Exit) {
			exits[it->get_i_idx()] = 1;
		  } else {
			cerr << "error: The type of leg is None!" << endl;	continue;
		  }
		  jt->f_info.cluster_group_idx = it->get_i_idx();
		  jt->f_info.pose_distance = 1.0 - (unlikeness / Pose_Similarity);
		  int farness = jt->distance_from_interact;
		  if (farness == 0) {
			cerr << "error: wrong initialization of EETable at (" << it - boundary_group.begin() << ", " << jt - it->begin() << ")" << endl; continue;
		  } else {
			many += (jt->f_info.pose_distance) * (15.0 / static_cast<double>(farness));
		  }
		}
	  }
	}
	distribution = static_cast<double>(std::count_if(entries.begin(), entries.end(), [](int check_)->bool {if (check_ == 1) return true; else return false;})) 
				 * static_cast<double>(std::count_if(exits.begin(), exits.end(), [](int check_)->bool {if (check_ == 1) return true; else return false;}));
	cout << p_idx << " Many: " << many << " , Distribute: " << distribution << endl;
	return many + distribution;
  }  
}

void EntryExitTable::print_cross_sect(const PosturePoints &pcloud)
{
  for (auto bt = boundary_group.begin(); bt != boundary_group.end(); ++bt) {
	view_similar_frame(*bt, pcloud);
  }  
}

PatchEntryExit& EntryExitTable::get_boundary( size_t idx )
{
  size_t count_ = idx;
  size_t trav = 0;
  while (count_ > 0) {
	count_ -= boundary_group[trav++].size();
  }
  count_ += boundary_group[--trav].size();
  return boundary_group[trav].entry_exits[count_];
}

const PatchEntryExit& EntryExitTable::get_boundary( size_t idx ) const
{
  size_t count_ = idx;
  size_t trav = 0;
  while (count_ > 0) {
	count_ -= boundary_group[trav++].size();
  }
  count_ += boundary_group[--trav].size();
  return boundary_group[trav].entry_exits[count_];
}

size_t EntryExitTable::size_candidates() const
{
  size_t ret = 0;
  for (auto it = boundary_group.begin(); it != boundary_group.end(); ++it) {
	ret += it->size();
  }
  return ret;
}

void EntryExitTable::print_node( Node * node )
{
  const auto & idxes = node->get_generations_idx();
  cout << node->size << ": ";
  for (size_t i = 0; i < idxes.size(); ++i) {
	cout << idxes[i] << ",";
  } cout << endl;
}

void EntryExitTable::print_tree()
{
  print_node(root_of_clustering);
}

void EntryExitTable::group_cluster_tree_by_dist( const double dist, const size_t boundary_size )
{
  clustered_tree_leaf_indexes.clear();
  preorder(clustered_tree_leaf_indexes, root_of_clustering, dist);
  sort(clustered_tree_leaf_indexes.begin(), clustered_tree_leaf_indexes.end(), [](vector<size_t> &a, vector<size_t> &b){return (a.size() > b.size());});
  print_cluster_tree(clustered_tree_leaf_indexes, boundary_size);
  //print_minor_tree(clustered_tree_leaf_indexes);
}

void EntryExitTable::preorder( vector< vector<size_t> > &ret, const Node * node, const double thres )
{
  if (node->get_value() < thres) {
	ret.push_back(node->get_generations_idx());
	return;
  }
  if (node->get_siblings()[0] == nullptr) return;
  for (size_t i = 0; i < node->get_siblings().size(); ++i) {
	preorder(ret, node->get_siblings()[i], thres);
  }  
}

void EntryExitTable::evaluate_all_legs_by_cluster()
{
  if (boundary_group.empty()) {
	cerr << "please, initialize EETable first!" << endl;
  } else {	
	for (auto it = boundary_group.begin(); it != boundary_group.end(); ++it) {
	  for (auto jt = it->begin(); jt != it->end(); ++jt) {
		pair<size_t, double> min_dist = get_proximately_group(*jt);
		jt->f_info.cluster_group_idx = static_cast<int>(min_dist.first);
		jt->f_info.pose_distance = min_dist.second;
	  }
	}	
  }
}

void EntryExitTable::store_infos_in_pees( const vector< vector<size_t> > &clustered_leaf_node_idxes, const size_t group_idx )
{
  store_cluster_group_in_pees(clustered_leaf_node_idxes, group_idx);
  store_shape_dist_in_pees(clustered_leaf_node_idxes, group_idx);
  //store_highlight_in_pees(clustered_leaf_node_idxes);
}

pair<size_t,double> EntryExitTable::get_proximately_group( const PatchEntryExit & pee )
{
  size_t g_idx = 0;
  double min_eval = 100000.0;
  for (auto it = clustered_tree_leaf_indexes.begin(); it != clustered_tree_leaf_indexes.end(); ++it) {
	double eval = evaluate_distance_in_group(pee, it - clustered_tree_leaf_indexes.begin());
	if (eval < min_eval) {
	  min_eval = eval;
	  g_idx = it - clustered_tree_leaf_indexes.begin();
	}
  }
  return pair<size_t, double>(g_idx, min_eval);
}

double EntryExitTable::evaluate_distance_in_group( const PatchEntryExit &pee, const size_t &g_idx )
{  
  double eval = 0.0;
  const auto &clustered_group = clustered_tree_leaf_indexes[g_idx];
  for (size_t j = 0; j < clustered_group.size(); ++j) {
	eval += shape_difference(pee, *(cluster_tree_leaf_node[clustered_group[j]]->ptr_pee));
  }
  eval /= static_cast<double>(clustered_group.size());

  return eval;
}

double boundary_set[9][3] = 
{
  {202. / 255., 84. / 255., 110. / 255.},
  {0., 0.4 ,1.0},
  {0.6, 0.6 ,0.0},
  {0.2, 0.5 ,0.2},
  {0. / 255., 136. / 255., 124. / 255.},
  {178./ 255., 122. / 255., 180. / 255.},
  {246. / 255., 139. / 255., 51. / 255.},
  {130. / 255., 0. / 255., 80. / 255.},
  {24. / 255., 148. / 255., 184. / 255.}
};

cml::vector3 get_random_bcolor() {
  double * col = boundary_set[random(0,8)];
  return cml::vector3(col[0], col[1], col[2]);
}

void EntryExitTable::print_cluster_tree( const vector< vector<size_t> > &groups, const size_t boundary_size )
{
  cout << "total group number: " << groups.size() << endl;
  for (auto it = groups.begin(); it != groups.begin() + boundary_size; ++it) {
	cout << endl << "the size is " << it->size() << endl;
	auto scolor = get_random_bcolor();
	for (auto gt = it->begin(); gt != it->end(); ++gt) {
	  cout << ", (" << get_cluster_tree_leaf_pee(*gt)->get_mot_idx() << ", "
					<< get_cluster_tree_leaf_pee(*gt)->it_posture()->time 
			  << ")";
	  //get_cluster_tree_leaf_pee(*gt)->it_posture()->set_status_in_viewer(true, scolor);
	} cout << endl;
  }
}

void EntryExitTable::print_minor_tree( const vector< vector<size_t> > &cluster_groups )
{
  size_t count_ = 0;
  for (auto it = cluster_groups.begin(); it != cluster_groups.end(); ++it) {
	if (it->size() < 10) {
	  ++count_;
	  for (auto gt = it->begin(); gt != it->end(); ++gt) {
		cout << "(" << get_cluster_tree_leaf_pee(*gt)->get_mot_idx() << ", "
		  << get_cluster_tree_leaf_pee(*gt)->it_posture()->time << "), ";
	  } cout << endl;
	}
  }
  cout << "Minor posture # is " << count_ << endl;
}


void EntryExitTable::get_cluster_group_distribution( vector< vector<size_t> > &tree_idx_distribution, const size_t g_idx )
{
  vector<size_t> the_group = clustered_tree_leaf_indexes[g_idx];
  vector< vector<size_t> > mark(size_mot(), vector<size_t>());
  
  for (size_t i = 0; i < the_group.size(); ++i) {	
	mark[get_cluster_tree_leaf_pee(the_group[i])->get_mot_idx()].push_back(the_group[i]);
  }  
  tree_idx_distribution = mark;

  for (auto it = tree_idx_distribution.begin(); it != tree_idx_distribution.end(); ++it) {
	sort(it->begin(), it->end(), [this](size_t i, size_t j)->bool {return (get_cluster_tree_leaf_pee(i)->get_pos_idx() < get_cluster_tree_leaf_pee(j)->get_pos_idx());});
  }  
}

void EntryExitTable::store_cluster_group_in_pees( const vector< vector<size_t> > &clustered_leaf_nodes_idx, const int group_idx )
{
  for (auto jt = clustered_leaf_nodes_idx.begin(); jt != clustered_leaf_nodes_idx.end(); ++jt) {
	for (auto kt = jt->begin(); kt != jt->end(); ++kt) {
	  get_cluster_tree_leaf_pee(*kt)->f_info.cluster_group_idx = get_cluster_tree_leaf_pee(*kt)->it_posture()->type_ = group_idx;
	}
  }
}

void EntryExitTable::store_shape_dist_in_pees( const vector< vector<size_t> > &clustered_leaf_nodes_idx, const size_t group_idx )
{
  for (auto jt = clustered_leaf_nodes_idx.begin(); jt != clustered_leaf_nodes_idx.end(); ++jt) {
	for (auto kt = jt->begin(); kt != jt->end(); ++kt) {	  
	  get_cluster_tree_leaf_pee(*kt)->f_info.pose_distance = evaluate_distance_in_group(*get_cluster_tree_leaf_pee(*kt), group_idx);	  
	}
  }
}

void EntryExitTable::store_highlight_in_pees( const vector< vector<size_t> > &clustered_leaf_nodes_idx )
{
  auto scolor = get_random_bcolor();
  for (auto jt = clustered_leaf_nodes_idx.begin(); jt != clustered_leaf_nodes_idx.end(); ++jt) {
	for (auto kt = jt->begin(); kt != jt->end(); ++kt) {	  	  
	  get_cluster_tree_leaf_pee(*kt)->it_posture()->set_status_in_viewer(true, scolor);
	}
  }
}

void EntryExitTable::test_pee_init()
{
   for (auto it = cluster_tree_leaf_node.begin(); it != cluster_tree_leaf_node.end(); ++it) { 
	 if ((*it)->ptr_pee->f_info.cluster_group_idx != -1 || (*it)->ptr_pee->it_posture()->type_ != -1) {
	   cout << "Error!" << endl;
	 }
   }
}

bool EEGroup::is_containing( size_t midx, pair<size_t,size_t> pos_section )
{
  if (midx != get_mot_idx()) return false;

  pair<size_t,size_t> this_sect;	  
  this_sect.first = (begin()->get_pos_idx() > (end() - 1)->get_pos_idx()) ? (end() - 1)->get_pos_idx() : begin()->get_pos_idx();
  this_sect.second= (begin()->get_pos_idx()> (end() - 1)->get_pos_idx()) ? begin()->get_pos_idx() : (end() - 1)->get_pos_idx();

  int f1 = (this_sect.first > pos_section.first) ? this_sect.first : pos_section.first;
  int e1 = (this_sect.second < pos_section.second) ? this_sect.second : pos_section.second;
  if (f1 - e1 < 2) {
	return true;
  } else {
	return false;
  }  
}
