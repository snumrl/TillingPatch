#include "StdAfx.h"
#include "patch_cooker.h"
#include "find_interaction.h"
#include "get_patch.h"
#include "nonslipfoot.h"
#include "tiling.h"
#include "motions_viewer.h"
#include "patches_viewer.h"


class InteractionTest : public testing::Test
{
protected:
  static vector<ml::Motion> *motions;  
  
  static void SetUpTestCase() {
	motions = new vector<ml::Motion>();
	string file_name;
	if (true) {
	  file_name = string("light_example_");
	  char patch_size [256] = "25";
	  file_name += patch_size;	  
	} else {
	  file_name = string("large_scale_example");
	}
	read_motions(*motions, file_name);	
  }

  static void TearDownTestCase() {
	delete motions;	
  }
};

vector<ml::Motion> *InteractionTest::motions = NULL;


TEST(Data_IO, DISABLED_get_small_patch_file)
{
  Env env;
  env.set_scatter_range(-9, 9, -9, 9, 100, 800);

  vector< shared_ptr<Patch> > patches;  
  map<string, Patch> patch_type;

  get_patch(&patch_type);
  shared_ptr<Patch> seed = get_a_patch("jump", patch_type);
  env.rand_scatter(*seed);
  patches.push_back(seed);

  char patch_size [128] = "5";
  if (!stitch_patches(patches, atoi(patch_size), patch_type, env)) {
	cout << "Dead End Scene" << endl;
  }
  set_color(patches);
  
  vector<ml::Motion> motions;
  convert_patch_to_motion(motions, patches);
  double t_time = begin_time(motions);
  for_each(motions.begin(), motions.end(), [t_time](ml::Motion &mot){ mot.translate_time(-t_time);});

  string file_name = string("light_example_");
  write_motions(motions, file_name + string(patch_size));

  vector<ml::Motion> re_read_mots;
  read_motions(re_read_mots, file_name + string(patch_size));

  DontSlip noslip(18, 19, 24, 3, 4, 20);
  for (auto it = re_read_mots.begin(); it != re_read_mots.end(); ++it) {
	noslip.hold_foot(*it);
  }
  MotionsViewer *mv = new MotionsViewer(re_read_mots);
  mv->get_camera().distance += 2;
}

TEST(Data_IO, DISABLED_get_large_patch_file)
{
  vector<shared_ptr<Patch>> read_pat;
  read_patches(read_pat, "first_example_36010_10000_150000_0.2_0.0015_phase2");
  vector<ml::Motion> read_mots;
  convert_patch_to_motion(read_mots, read_pat);
  write_motions(read_mots, "large_scale_example");

  vector<ml::Motion> re_read_mots;
  read_motions(re_read_mots, "large_scale_example");

  MotionsViewer *mv = new MotionsViewer(re_read_mots);
  mv->get_camera().distance += 2;
}

TEST_F(InteractionTest, DISABLED_gtest_buildup)
{
  MotionsViewer *mv = new MotionsViewer(*motions);
  mv->get_camera().distance += 2; 
}

TEST(Collision, DISABLED_bound_abb)
{
  map<string, Patch> patch_type;
  get_patch(&patch_type);  

  vector< shared_ptr<Patch> > patches;
  patches.push_back(get_a_patch("beat", patch_type));
  /*for (auto it=patch_type.begin(); it!=patch_type.end(); ++it) {
	patches.push_back(get_a_patch((it->first).c_str(), patch_type));
  }*/
  PatchesViewer *pv = new PatchesViewer(patches);
  pv->get_camera().distance += 2;
}

TEST_F(InteractionTest, DISABLED_collision_of_character)
{
  vector<pair_section> collisions = likely_collisions(motions);
  check_section(motions, collisions);

  MotionsViewer *mv = new MotionsViewer(*motions);
  mv->get_camera().distance += 2;
}

TEST_F(InteractionTest, DISABLED_extract_contact)
{
  DontSlip noslip(18, 19, 24, 3, 4, 20);
  for (auto it = motions->begin(); it != motions->end(); ++it) {
	noslip.hold_foot(*it);
  }
  MotionsViewer *mv = new MotionsViewer(*motions);
  mv->get_camera().distance += 2;

  /*find_contact(motions);
  MotionsViewer *mv = new MotionsViewer(motions);
  mv->get_camera().distance += 2; */
}

TEST(ContactTest, DISABLED_turn_right)
{
  map<string, Patch> patch_type;
  add_patch(&patch_type, "3/avoid", 622, 2, 702, 2, -1, -1, 0.018);
  {
	Patch pa;
	make_patch_kick(pa);
	patch_type[pa.name] = move(pa);
  }

  Env env;
  env.set_scatter_range(-9, 9, -9, 9, 100, 800);
  vector< shared_ptr<Patch> > patches;  
  shared_ptr<Patch> seed = get_a_patch("kick", patch_type);
  env.rand_scatter(*seed);
  patches.push_back(seed);
  if (!stitch_patches(patches, 6, patch_type, env)) {
	cout << "Dead End Scene" << endl;
  }
  
  vector<ml::Motion> motions;
  convert_patch_to_motion(motions, patches);
  double t_time = begin_time(motions);
  for_each(motions.begin(), motions.end(), [t_time](ml::Motion &mot){ mot.translate_time(-t_time);});

  DontSlip noslip(18, 19, 24, 3, 4, 20);
  noslip.hold_foot(motions[0]);
  
  MotionsViewer *mv = new MotionsViewer(motions);
  mv->get_camera().distance += 2;
}

TEST(PatchEvent, DISABLED_posture_distance)
{
  shared_ptr<Patch> seed = get_a_patch("jump");
  seed->translate_origin();
  seed->translate_time(-1 * seed->get_begin_time());
  const ml::Motion mot1(seed->motions[1]);
  vector<double> weights = extract_joint_weight(mot1);
  vector<double> var = extract_pose_variance(mot1);
  for (size_t j=2; j < mot1.size(); ++j) {
	cout << j << " ";
	dist_pose_in_same_motion(1, j, mot1, weights, var);
  }
  vector<ml::Motion> motions;
  motions.push_back(mot1);
  MotionsViewer *mv = new MotionsViewer(motions);
  mv->get_camera().distance += 2;
}

TEST(PatchEvent, DISABLED_root_orientation)
{
  
}

TEST(PatchEvent, DISABLED_activation)
{
  shared_ptr<Patch> seed = get_a_patch("accident");
  seed->translate_origin();
  seed->translate_time(-1 * seed->get_begin_time());

  const ml::Motion mot1(seed->motions[1]);
  vector<double> weights = extract_joint_weight(mot1);
  vector<double> joint_variance = extract_pose_variance(mot1);
  for (size_t j = 0; j < mot1.size(); ++j) {
	cout << j << " : " << activation(mot1, j, weights, joint_variance, 3) << endl;
  }

  /*const ml::Motion mot2(seed->motions[1]);
  weights = extract_joint_weight(mot2);
  joint_variance = extract_pose_variance(mot2);
  for (size_t j=0; j < mot2.size(); ++j) {
	cout << j << " : " << new_activation(mot2, j, weights, joint_variance, 7) << endl;
  }*/

  MotionsViewer *mv = new MotionsViewer(seed->motions);
  mv->get_camera().distance += 2;
}

TEST(PatchEvent, DISABLED_an_exact_collision)
{
  shared_ptr<Patch> seed = get_a_patch("beat");
  seed->translate_origin();
  seed->translate_time(-1 * seed->get_begin_time());
  
  auto secs = exact_collisions(&seed->motions);  
  check_section(&seed->motions, secs);

  for (size_t i = 0; i < seed->motions[0].size(); ++i) {
	cout << setprecision(6) << face_diff(seed->motions[0][i], seed->motions[1][i]) << ", ";
  }

  MotionsViewer *mv = new MotionsViewer(seed->motions);
  mv->interact_pair = secs;
  mv->get_camera().distance += 2;
}

TEST_F(InteractionTest, DISABLED_exact_collision)
{
  auto secs = exact_collisions(motions);
  unite_sections_within_contact(secs, 5, motions);
  check_section(motions, secs);

  MotionsViewer *mv = new MotionsViewer(*motions);
  mv->interact_pair = secs;
  mv->get_camera().distance += 2;
}

TEST(PatchEvent, DISABLED_an_intimate)
{
  shared_ptr<Patch> seed = get_a_patch("accident");
  seed->translate_origin();
  seed->translate_time(-1 * seed->get_begin_time());

  auto secs = intimate_characters(&seed->motions);  
  check_section(&seed->motions, secs);

  MotionsViewer *mv = new MotionsViewer(seed->motions);
  mv->interact_pair = secs;
  mv->get_camera().distance += 2;
}

TEST_F(InteractionTest, DISABLED_intimates)
{
  vector<pair_section> intimate = intimate_characters(motions);
  check_section(motions, intimate);

  MotionsViewer *mv = new MotionsViewer(*motions);
  mv->interact_pair = intimate;
  mv->get_camera().distance += 2;
}

TEST_F(InteractionTest, DISABLED_strong_interaction)
{
  vector<pair_section> secs;
  strong_interaction(secs, motions);
  check_section(motions, secs);

  MotionsViewer *mv = new MotionsViewer(*motions);
  mv->interact_pair = secs;
  mv->get_camera().distance += 2;
}

TEST_F(InteractionTest, DISABLED_sync_interactions)
{
  vector<pair_section> secs;
  synchronized_interaction(secs, motions);	  
  check_section(motions, secs);

  MotionsViewer *mv = new MotionsViewer(*motions);
  mv->interact_pair = secs;
  mv->get_camera().distance += 2;
}

TEST(PatchEvent, DISABLED_a_sync_interaction)
{
  shared_ptr<Patch> seed = get_a_patch("ballet");
  seed->translate_origin();
  seed->translate_time(-1 * seed->get_begin_time());

  vector<pair_section> secs;
  synchronized_interaction(secs, &seed->motions);
  check_section(&seed->motions, secs);

  MotionsViewer *mv = new MotionsViewer(seed->motions);
  mv->interact_pair = secs;
  mv->get_camera().distance += 2;
}

TEST(Basic, DISABLED_interpolate_interacts) 
{
  pair_section m1(0, 1, pair<size_t,size_t>(0,10), pair<size_t,size_t>(13,15));
  pair_section m2(0, 1, pair<size_t,size_t>(13,15), pair<size_t,size_t>(23,25));
  divide_overlapping_sect_half(m1, m2);

  pair_section m3(0, 1, pair<size_t,size_t>(0,10), pair<size_t,size_t>(13,15));
  pair_section m4(1, 2, pair<size_t,size_t>(9,11), pair<size_t,size_t>(23,25));
  divide_overlapping_sect_half(m3, m4);

  pair_section m5(0, 1, pair<size_t,size_t>(0,10), pair<size_t,size_t>(13,15));
  pair_section m6(2, 0, pair<size_t,size_t>(9,11), pair<size_t,size_t>(23,25));
  divide_overlapping_sect_half(m5, m6);

  pair_section m7(0, 1, pair<size_t,size_t>(0,10), pair<size_t,size_t>(13,15));
  pair_section m8(2, 3, pair<size_t,size_t>(9,11), pair<size_t,size_t>(23,25));
  divide_overlapping_sect_half(m7, m8);
}

TEST(Basic, DISABLED_merge_sects)
{
  pair_section a(0, 1, pair<size_t,size_t>(1, 11), pair<size_t,size_t>(2, 13)), b(1, 0, pair<size_t,size_t>(14, 33), pair<size_t,size_t>(12, 21));
  if (merge(a, b, 0)) {
	cout << a.get_motion_idx(0) << ", " << a.get_motion_idx(1) << " : (" << a.get_section(0).first << ", " << a.get_section(0).second << ")"
	  << "(" << a.get_section(1).first << ", " << a.get_section(1).second << ")" << endl;
  } else {
	cout << "not merge!" << endl;
  }
}

TEST_F(InteractionTest, DISABLED_rectify_interactions) 
{
  if (false) {
	vector<pair_section> secs;
	strong_interaction(secs, motions);
	fill_pair_section_gap(secs, 10);
	check_section(motions, secs);

	MotionsViewer *mv = new MotionsViewer(*motions);
	mv->interact_pair = secs;
	mv->get_camera().distance += 2;
  } else {
	vector<pair_section> secs;
	synchronized_interaction(secs, motions);
	fill_pair_section_gap(secs, 6);
	check_section(motions, secs);

	MotionsViewer *mv = new MotionsViewer(*motions);
	mv->interact_pair = secs;
	mv->get_camera().distance += 2;
  }  
}

TEST_F(InteractionTest, DISABLED_find_raw_interactions)
{
  vector<pair_section> inters;  
  vector<pair_section> intimate = intimate_characters(motions);
  vector<pair_section> collision = exact_collisions(motions);
  vector<pair_section> wi;
  synchronized_interaction(wi, motions);
  copy(intimate.begin(), intimate.end(), back_inserter(inters));
  copy(collision.begin(), collision.end(), back_inserter(inters));
  copy(wi.begin(), wi.end(), back_inserter(inters));

  MotionsViewer *mv = new MotionsViewer(*motions);
  mv->interact_pair = inters;
  mv->get_camera().distance += 2;
}

TEST_F(InteractionTest, DISABLED_input_analysis)
{
  cout << motions->size() << endl;
  size_t max_ = 0, min_ = 1000000;
  for (auto it = motions->begin(); it != motions->end(); ++it) {
	if (it->size() > max_) max_ = it->size();
	if (it->size() < min_) min_ = it->size();
  } cout << max_ << ", " << min_ << endl;
}

TEST_F(InteractionTest, DISABLED_find_interactions)
{
  vector<pair_section> inters;
  find_interactions(inters, *motions, 10);
  check_interactions(motions, inters);

  MotionsViewer *mv = new MotionsViewer(*motions);
  mv->interact_pair = inters;
  mv->get_camera().distance += 2;
}

TEST(Library, DISABLED_erase)
{
  vector<size_t> myvector;

  // set some values (from 1 to 10)
  for (size_t i=1; i<=10; i++) myvector.push_back(i);

  for (auto it = myvector.begin(); it != myvector.end(); ) {
	if (*it == 3) {
	  it = myvector.erase(it);
	} else {
	  ++it;
	}
  }
}

TEST(Library, DISABLED_begin)
{
  vector<int> vec;
  bool check = false;
  for (auto it = vec.begin(); it != vec.end(); ++it) {
	check = true;
  }
  EXPECT_EQ(false, check);
}

TEST_F(InteractionTest, DISABLED_contact_bef) 
{
  DontSlip noslip(18, 19, 24, 3, 4, 20);
  for (auto it = motions->begin(); it != motions->end(); ++it) {
	noslip.hold_foot(*it);
  }
  
  ml::Motion::Iterator mt = motions->at(0).end() - 1;
  while (mt != motions->at(0).begin()) {
	contact_pre_state(mt, motions->at(0).begin());
	mt->set_status_in_viewer(true, cml::vector3(1., 0., 0.));
  }

  MotionsViewer *mv = new MotionsViewer(*motions);  
  mv->get_camera().distance += 2;
}

TEST_F(InteractionTest, DISABLED_contact_aft) 
{
  DontSlip noslip(18, 19, 24, 3, 4, 20);
  for (auto it = motions->begin(); it != motions->end(); ++it) {
	noslip.hold_foot(*it);
  }    
  ml::Motion::Iterator mt = motions->at(0).begin();
  while (mt != motions->at(0).end() - 1) {
	contact_post_state(mt, motions->at(0).end() - 1);
	mt->set_status_in_viewer(true, cml::vector3(0., 0., 1.));
  } 

  MotionsViewer *mv = new MotionsViewer(*motions);  
  mv->get_camera().distance += 2;
}

TEST_F(InteractionTest, DISABLED_find_edge)
{
  PatchCooker cooking(motions);
  pair_section mat(0, 1, pair<size_t,size_t>(25, 31), pair<size_t,size_t>(34, 41));
  pair_section ret = cooking.direct_covering_pair_section(mat, 2, 2);

  MotionsViewer *mv = new MotionsViewer(*motions);  
  mv->interact_pair.push_back(ret);
  mv->get_camera().distance += 2;
}

//TEST_F(InteractionTest, cut_edges)
//{
//  NoSlippery noslip(18, 19, 24, 3, 4, 20);
//  for (auto it = motions->begin(); it != motions->end(); ++it) {
//	noslip.hold_foot(*it);
//  }
//
//  vector<pair_section> pair_sects;
//  find_interactions(pair_sects, *motions);
//  MotionsViewer *mv = new MotionsViewer(*motions);
//  mv->interact_pair = pair_sects;
//  mv->get_camera().distance += 2;
//  
//  PatchCooker cooking(motions);
//  cooking.mold_patch_edges(pair_sects);
//  if (!cooking.cook_pairs()) {
//	cout << "false!!" << endl;
//  }
//  PatchesViewer * v = new PatchesViewer(cooking.patches);
//  v->get_camera().distance += 2;  
//}

TEST(Interaction, DISABLED_normalize_interaction)
{
  shared_ptr<Patch> pa = get_a_patch("beat4");
  pa->translate_time(-1 * pa->get_begin_time());
  vector<ml::Motion> mots;
  for (size_t i = 0; i < pa->motions.size(); ++i) {
	mots.push_back(pa->motions[i]);
  }  
  PatchCooker cooker(&mots);
  cooker.init_interactions();  
  MotionsViewer * v = new MotionsViewer(mots);
  v->get_camera().distance += 2;
}


TEST_F(InteractionTest, DISABLED_cut_interaction_edges)
{
  DontSlip noslip(18, 19, 24, 3, 4, 20);
  for (auto it = motions->begin(); it != motions->end(); ++it) {
	noslip.hold_foot(*it);
  }  
  PatchCooker cooking(motions);
  cooking.init_interactions();
  cooking.init_EETable();
  cooking.mold_interact_patch();
  cooking.cook_motion_patches();
  if (!cooking.patches->empty()) {
	PatchesViewer * v = new PatchesViewer(cooking.get_all_patches());
	v->get_camera().distance += 2;	
  } else {
	cout << "false!!" << endl;
  }  
}
