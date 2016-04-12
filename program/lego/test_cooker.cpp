#include "StdAfx.h"
#include "patch_cooker.h"
#include "get_patch.h"
#include "nonslipfoot.h"
#include "motions_viewer.h"
#include "patches_viewer.h"
#include "execute.h"
#include "tiling.h"

#include "../combination/combination.h"


class PCooker : public testing::Test
{
protected:
  static vector<ml::Motion> *motions;  

  static void SetUpTestCase() {
	motions = new vector<ml::Motion>();
	string file_name;
	if (true) {
	  file_name = string("light_example_");
	  char patch_size [256] = "5";
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

vector<ml::Motion> *PCooker::motions = NULL;


TEST_F(PCooker, DISABLED_gtest_buildup)
{
  MotionsViewer *mv = new MotionsViewer(*motions);
  mv->get_camera().distance += 2; 
}

TEST_F(PCooker, DISABLED_gather_posture)
{
  PatchCooker cooker(motions);
  cooker.init_interactions();
  cooker.init_EETable();

  MotionsViewer *mv = new MotionsViewer(*motions);
  mv->interact_pair = cooker.relation_pairs();
  mv->get_camera().distance += 2; 
}

TEST(Motion, DISABLED_rotate)
{
  shared_ptr<Patch> pat = get_a_patch("jump");
  pat->translate_time(-1 * pat->get_begin_time());
  ml::Motion &mot = pat->motions[1];
  cml::vector3 rot = cml::between_vector(plane_vec3(log_mat3(mot[0].GetGlobalRoation(0))), cml::vector3(1., 0., 1.));

  Feature f1(0, mot);
  f1.rotate(rot);
  
  mot.ApplyTransf(cml::make_transf(exp_mat3(rot), cml::vector3(0., 0., 0.)));
  Feature f2(0, mot);
  
  EXPECT_EQ(0.0, dist_pose_in_same_motion(f1 , f2, extract_joint_weight(mot), extract_pose_variance(mot)));

  MotionsViewer *mv = new MotionsViewer(mot);
  mv->get_camera().distance += 2; 
}

TEST(Motion, DISABLED_dist_pose_in_same_motion)
{
  shared_ptr<Patch> pat = get_a_patch("ballet");
  pat->translate_time(-1 * pat->get_begin_time());
  ml::Motion &mot = pat->motions[1];
  vector<Feature> fs(mot.size() - 1);
  for (size_t i = 1; i < mot.size(); ++i) {
	Feature f(i, mot);
	cml::vector3 rot = cml::between_vector(plane_vec3(log_mat3(mot[i].GetGlobalRoation(0))), cml::vector3(1., 0., 0.));
	f.transl(-1 * cml::vector3(mot[i].trans()[0], 0., mot[i].trans()[2]));
	f.rotate(rot);	
	fs[i - 1] = f;
  }

  for (size_t i = 1; i < mot.size() - 1; ++i) {
	cout << i << " : " << dist_pose_in_same_motion(fs[0], fs[i], extract_joint_weight(mot)) << endl;
  }

  MotionsViewer *mv = new MotionsViewer(mot);
  mv->get_camera().distance += 2; 
}

TEST(Motion, DISABLED_select_pos)
{
  shared_ptr<Patch> pat = get_a_patch("shake_hands");
  pat->translate_time(-1 * pat->get_begin_time());
  vector<ml::Motion> mots;
  for (size_t i = 0; i < pat->motions.size(); ++i) {
	mots.push_back(pat->motions[i]);
  }

  MotionsViewer *mv = new MotionsViewer(mots);  
  mv->get_camera().distance += 2;
}

TEST(Motion, DISABLED_evaluate_edge)
{
  shared_ptr<Patch> pat = get_a_patch("beat");
  pat->translate_time(-1 * pat->get_begin_time());
  vector<ml::Motion> mots;
  for (size_t i = 0; i < pat->motions.size(); ++i) mots.push_back(pat->motions[i]);

  PatchCooker cooker_(&mots);
  cooker_.init_interactions();
  cooker_.init_EETable();

  ml::Motion mot;
  double ground = 0.03;
  mot.LoadAMC_with_contactInfo("./data/1/walk_straight.amc", "./data/wd2.asf", true, 0.027, ground);  
  cooker_.set_cross_sects(123, mot);
  cooker_.set_cross_sects(130, mot);
  cooker_.set_cross_sects(140, mot);
  cooker_.set_cross_sects(147, mot);

  cooker_.evaluate_cross_sects_given_pose();
  cooker_.print_evaluation();

  MotionsViewer *mv = new MotionsViewer(mots);  
  mv->get_camera().distance += 2;
}

TEST(Motion, DISABLED_shape_distance)
{
  shared_ptr<Patch> pat = get_a_patch("beat4");
  pat->translate_time(-1 * pat->get_begin_time());
  vector<ml::Motion> mots;
  for (size_t i = 0; i < pat->motions.size(); ++i) mots.push_back(pat->motions[i]);

  PosturePoints * f[4];
  for (size_t i = 0; i < 4; ++i) {
	f[i] = new PosturePoints(mots[i], 3);
  }
  EXPECT_LE(pose_weighted_difference(*f[0], *f[1], extract_joint_weight(mots[0])), 0.00001);
  EXPECT_LE(pose_weighted_difference(*f[0], *f[2], extract_joint_weight(mots[0])), 0.00001);
  EXPECT_LE(pose_weighted_difference(*f[0], *f[3], extract_joint_weight(mots[0])), 0.00001);
}

TEST(RawData, DISABLED_multiview)
{
  vector<ml::Motion> ms;

  if (false) 
  {
	ml::Motion m21, m22;
	clip_copy_motion(m21, "multi/hys_push", 0, 1769);
	clip_copy_motion(m22, "multi/mmk_push", 0, 1769);
	ms.push_back(m21);
	ms.push_back(m22);
  }
  if (false) 
  {
	ml::Motion m21, m22;
	clip_copy_motion(m21, "multi/hys_hello", 0, 2774);
	clip_copy_motion(m22, "multi/mmk_hello", 0, 2774);
	ms.push_back(m21);
	ms.push_back(m22);
  }
  if (false) 
  {
	ml::Motion m21, m22;
	clip_copy_motion(m21, "multi/hys_hifive", 0, 3548);
	clip_copy_motion(m22, "multi/mmk_hifive", 0, 3548);
	ms.push_back(m21);
	ms.push_back(m22);
  }
  if (false) 
  {
	ml::Motion m21, m22;
	clip_copy_motion(m21, "multi/hys_synch", 0, 3534);
	clip_copy_motion(m22, "multi/mmk_synch", 0, 3534);
	ms.push_back(m21);
	ms.push_back(m22);
  }

  DontSlip noslip;
  for (auto it = ms.begin(); it != ms.end(); ++it) {
	noslip.hold_foot(*it);
  }
  MotionsViewer * t = new MotionsViewer(ms);
}


TEST_F(PCooker, DISABLED_shape_distance)
{
  vector<ml::Motion> m(1, motions->at(3));  
  
  EEGroup egg;
  for (size_t i = 0; i < m[0].size(); ++i) {
	egg.push(PatchEntryExit(0, i, 1, &m));
  }

  EntryExitTable eet(m.size());
  eet.set_boundary_group(egg);
  eet.agglomerative_tree_construction();
  eet.group_cluster_tree_by_dist(2.2, 4);

  DontSlip noslip;
  for (auto it = m.begin(); it != m.end(); ++it) {
	noslip.hold_foot(*it);
  }
  MotionsViewer *mv = new MotionsViewer(m);
  mv->get_camera().distance += 2;

  if (false) {
	// blending
	ml::Motion m1(m[0], m[0].begin(), m[0].begin() + 100);
	ml::Motion m2(m[0], m[0].begin() + 354, m[0].end());
	m1.stitch(m2);

	noslip.hold_foot(m1);
	MotionsViewer *mvv = new MotionsViewer(m1);
	mvv->get_camera().distance += 2;
  }  
}


TEST_F(PCooker, DISABLED_evaluate_edges)
{
  PatchCooker cooker_(motions);
  cooker_.init_interactions();
  if (cooker_.init_EETable()) {
	ml::Motion mot;
	double ground = 0.03;
	mot.LoadAMC_with_contactInfo("./data/1/walk_straight.amc", "./data/wd2.asf", true, 0.027, ground);  
	cooker_.set_cross_sects(123, mot);
	cooker_.set_cross_sects(130, mot);
	cooker_.set_cross_sects(140, mot);
	cooker_.set_cross_sects(147, mot);

	cooker_.evaluate_cross_sects_given_pose();
	cooker_.print_evaluation();
	cooker_.print_max_cross_sec();
  }
  MotionsViewer *mv = new MotionsViewer(*motions);
  mv->get_camera().distance += 2;
}

TEST(Motion, DISABLED_auto_covering)
{
  shared_ptr<Patch> pat = get_a_patch("beat4");
  pat->translate_time(-1 * pat->get_begin_time());
  vector<ml::Motion> mots;
  for (size_t i = 0; i < pat->motions.size(); ++i) mots.push_back(pat->motions[i]);

  PatchCooker cooker_(&mots);
  cooker_.init_interactions();
  cooker_.init_EETable();

  ml::Motion mot;
  double ground = 0.03;
  mot.LoadAMC_with_contactInfo("./data/1/walk_straight.amc", "./data/wd2.asf", true, 0.027, ground);  
  cooker_.set_cross_sects(123, mot);
  cooker_.set_cross_sects(130, mot);
  cooker_.set_cross_sects(140, mot);
  cooker_.set_cross_sects(147, mot);

  cooker_.evaluate_cross_sects_given_pose();
  for (size_t i = 0; i < cooker_.interaction_prober->interactions.size(); ++i) {
	//cooker_.auto_covering_interacts(i);
  }  
}

TEST_F(PCooker, DISABLED_view)
{
  DontSlip noslip(18, 19, 24, 3, 4, 20);
  for (auto it = motions->begin(); it != motions->end(); ++it) {
	noslip.hold_foot(*it);
  }

  MotionsViewer *mv = new MotionsViewer(*motions);
  mv->get_camera().distance += 2;
}

TEST_F(PCooker, DISABLED_dofinder)
{
  cout << motions->size() << endl;
  size_t max_ = 0, min_ = 1000000;
  for (auto it = motions->begin(); it != motions->end(); ++it) {
	if (it->size() > max_) max_ = it->size();
	if (it->size() < min_) min_ = it->size();
  } cout << max_ << ", " << min_ << endl;
  
  PatchCooker cooker_(motions);
  vector<pair_section> psec = cooker_.init_interactions();
  
  DontSlip noslip(18, 19, 24, 3, 4, 20);
  for (auto it = motions->begin(); it != motions->end(); ++it) {
	noslip.hold_foot(*it);
  }

  MotionsViewer * v = new MotionsViewer(*motions);  
  v->interact_pair = psec;
  v->get_camera().distance += 2;
}


TEST(Algorithm, DISABLED_agglomerative_tree)
{
  shared_ptr<Patch> pat = get_a_patch("push");
  pat->translate_time(-1 * pat->get_begin_time());
  vector<ml::Motion> mots;
  for (size_t i = 0; i < pat->motions.size(); ++i) mots.push_back(pat->motions[i]);
  
  PatchCooker cooker_(&mots);
  cooker_.init_interactions();
  cooker_.init_EETable();
  cooker_.agglomerative_cluster();

  MotionsViewer *mv = new MotionsViewer(mots);
  mv->get_camera().distance += 2;
}

TEST_F(PCooker, DISABLED_agglomerative_tree)
{
  PatchCooker cooker_(motions);
  cooker_.init_interactions(); 
  size_t count_ = 0;  
  time_t start_time;
  time(&start_time);

  if (cooker_.init_EETable()) {
	cooker_.agglomerative_cluster();
  }

  time_t cur_time;
  time(&cur_time);
  cout << "time is" << difftime(cur_time, start_time) << endl;  

  MotionsViewer *mv = new MotionsViewer(*motions);
  mv->get_camera().distance += 2;
}

TEST_F(PCooker, DISABLED_get_combi)
{
  PatchCooker cooker_(motions);
  cooker_.init_interactions(); 
  size_t count_ = 0;  
  time_t start_time;
  time(&start_time);

  if (cooker_.init_EETable()) {
	cooker_.agglomerative_cluster();
	cooker_.Test_molding(5, 10);
  }

  time_t cur_time;
  time(&cur_time);
  cout << "time is" << difftime(cur_time, start_time) << endl;  

  MotionsViewer *mv = new MotionsViewer(*motions);
  mv->get_camera().distance += 2;
}

TEST(Algorithm, DISABLED_eval_distribution)
{
  vector < vector<size_t> > input;
  vector<size_t> temp;
  temp.push_back(1); 
  temp.push_back(2);
  temp.push_back(3);
  temp.push_back(5);
  temp.push_back(8);
  temp.push_back(9);
  temp.push_back(10);
  temp.push_back(13);
  input.push_back(temp);
  input.push_back(temp);
  
  double eval = 0.0;
  for (auto it = input.begin(); it != input.end(); ++it) {
	eval += 1.0;
	size_t pre = *(it->begin());
	for (auto jt = it->begin(); jt != it->end(); ++jt) {
	  if (*jt - pre > 1) eval += 1.0;
	  pre = *jt;
	}
  }
  EXPECT_EQ(8.0, eval);
}


TEST_F(PCooker, DISABLED_mold_patches)
{
  PatchCooker cooker_(motions);
  cooker_.init_interactions(); 
  if (cooker_.init_EETable()) {
	cooker_.agglomerative_cluster();
	cooker_.mold_motion_patches();	
  }

  MotionsViewer *mv = new MotionsViewer(*motions);
  mv->get_camera().distance += 2;
}

TEST(Algorithm, DISABLED_Kovar_distance)
{
  shared_ptr<Patch> pat = get_a_patch("beat");
  pat->translate_time(-1 * pat->get_begin_time());
  vector<ml::Motion> mots;
  for (size_t i = 0; i < pat->motions.size(); ++i) mots.push_back(pat->motions[i]);

  double dist = distance_Kovar_metric(PointsCloud(mots[0], 1, 25), PointsCloud(mots[1], 2, 26)); 
  EXPECT_LT(dist, 0.00001);

  if (false) {
	MotionsViewer *mv = new MotionsViewer(mots);
	mv->get_camera().distance += 2;
  }  
}

TEST_F(PCooker, DISABLED_adjcent_inter_idx)
{
  PatchCooker cooker_(motions);
  cooker_.init_interactions();  
  if (cooker_.init_EETable()) {
	cooker_.test_adj_interaction_idx();
  }
  DontSlip noslip(18, 19, 24, 3, 4, 20);
  for (auto it = motions->begin(); it != motions->end(); ++it) {
	noslip.hold_foot(*it);
  }
  MotionsViewer *mv = new MotionsViewer(*motions);
  mv->get_camera().distance += 2;  
}

TEST_F(PCooker, DISABLED_cook_patches)
{
  PatchCooker cooker_(motions);

  time_t start_time;
  time(&start_time);
  cooker_.init_interactions();    
  time_t cur_time;
  time(&cur_time);
  cout << "Interactions Detecting time is " << difftime(cur_time, start_time) << endl; 

  if (cooker_.init_EETable()) {	
	time_t mid_time;
	time(&mid_time);
	cooker_.agglomerative_cluster();	
	time(&cur_time);
	cout << "Cluster time is " << difftime(cur_time, mid_time) << endl; 

	cooker_.mold_motion_patches();
	cooker_.cook_motion_patches();
	time(&cur_time);
	cout << "Total time is " << difftime(cur_time, start_time) << endl; 

	if (!cooker_.patches->empty()) {
	  cooker_.evaluate_this_system();
	  PatchesViewer * v = new PatchesViewer(cooker_.get_all_patches());
	  v->get_camera().distance += 2;	
	}
  }
  DontSlip noslip(18, 19, 24, 3, 4, 20);
  for (auto it = motions->begin(); it != motions->end(); ++it) {
	noslip.hold_foot(*it);
  }
  MotionsViewer *mv = new MotionsViewer(*motions);
  mv->get_camera().distance += 2;  
}

TEST_F(PCooker, DISABLED_comb_cook)
{
  PatchCooker cooker_(motions);

  time_t start_time;
  time(&start_time);
  vector<pair_section> psec = cooker_.init_interactions();
  time_t cur_time;
  time(&cur_time);
  cout << "Interactions Detecting time is " << difftime(cur_time, start_time) << endl; 

  cooker_.init_EETable();

  time_t mid_time;
  time(&mid_time);
  cooker_.agglomerative_cluster();
  time(&cur_time);
  cout << "Clustering time is " << difftime(cur_time, mid_time) << endl; 

  cooker_.comb_cook_motion_patches();
  time(&cur_time);
  cout << "Total time is " << difftime(cur_time, start_time) << endl; 

  if (!cooker_.patches->empty()) {
	cooker_.evaluate_this_system();	
	PatchesViewer * v = new PatchesViewer(cooker_.get_all_patches());
	//v->set_origin_motions(cooker_);
	v->get_camera().distance += 2;	
  }
}

TEST_F(PCooker, DISABLED_save_patches) 
{
  PatchCooker cooker_(motions);

  time_t start_time;
  time(&start_time);

  cooker_.init_interactions();  
  if (cooker_.init_EETable()) {	
	cooker_.agglomerative_cluster();
	cooker_.mold_motion_patches();
	cooker_.cook_motion_patches();	

	time_t cur_time;
	time(&cur_time);
	cout << "Total elapsed time is " << difftime(cur_time, start_time) << endl; 

	if (!cooker_.patches->empty()) {
	  cooker_.evaluate_this_system();

	  string save_file_str = "cooker_patches_25";
	  write_patches(cooker_.get_all_patches(), save_file_str);

	  PatchesViewer * v = new PatchesViewer(cooker_.get_all_patches());
	  v->get_camera().distance += 2;	
	}
  }
}

TEST(Tiling, DISABLED_read_patches)
{
  vector< shared_ptr<Patch> > cooked_patches;
  read_patches(cooked_patches, "cooker_patches_100");
  set_color(cooked_patches);
    
  for (auto it = cooked_patches.begin() + 6; it != cooked_patches.begin() + 11; ++it) {
	/*if ((*it)->motions.size() > 4) {
	  patches.push_back(*it);
	  break;
	}*/
	//if ((*it)->motions.size() < 2) {
	  vector< shared_ptr<Patch> > patches;
	  patches.push_back(*it);
	  PatchesViewer * v = new PatchesViewer(patches);
	  v->get_camera().distance += 2;
	//}	
  } 
 // int count_ = 0;
 // for (auto it = cooked_patches.begin(); it != cooked_patches.end(); ++it) {
	///*if ((*it)->motions.size() > 4) {
	//  patches.push_back(*it);
	//  break;
	//}*/
	//if ((*it)->motions.size() < 2) {
	//  if (count_++ > 4) break;
	//  vector< shared_ptr<Patch> > patches;
	//  patches.push_back(*it);
	//  PatchesViewer * v = new PatchesViewer(patches);
	//  v->get_camera().distance += 2;
	//}	
 // }  
  PatchesViewer * v = new PatchesViewer(cooked_patches);
  v->get_camera().distance += 2;
}

TEST(CTiling, DISABLED_stitching) 
{ 
  vector< shared_ptr<Patch> > cooked_patches;
  read_patches(cooked_patches, "cooker_patches_25");
  map<string,Patch> patch_types;
  for (auto it = cooked_patches.begin(); it != cooked_patches.end(); ++it) {
	patch_types[(*it)->name] = (**it);
  }
  Env env;
  env.set_scatter_range(-6, 6, -6, 6, 100, 700);  
  Patch * seed = new Patch(*cooked_patches[5]);
  env.put_center(*seed);
  vector< shared_ptr<Patch> > patches;
  patches.push_back(shared_ptr<Patch>(seed));

  char patch_size [128] = "15";
  if (!stitch_patches(patches, atoi(patch_size), patch_types, env)) {
	cout << "Dead End Scene" << endl;
  }
  set_color(patches);
  
  DontSlip noslipfoot(18, 19, 24, 3, 4, 20);
  for (auto it = patches.begin(); it != patches.end(); ++it) noslipfoot.hold_foot(**it);

  PatchesViewer * v = new PatchesViewer(patches, env);
  v->get_camera().distance += 2;	
}

//TEST(CTiling, tiling)
TEST(CTiling, DISABLED_tiling)
{
  vector<shared_ptr<Patch> > cooked_patches;
  //read_patches(cooked_patches, "hand_chicken");
  //read_patches(cooked_patches, "cooked_rawData_chicken_combi_3");  
  //read_patches(cooked_patches, "cooked_rawData_dance_combi_4");  
  //read_patches(cooked_patches, "cooked_rawData_Hello_combi_3");
  read_patches(cooked_patches, "cooked_rawData_dance_combi_3");
  //read_patches(cooked_patches, "cooked_rawData_catchMe_combi_7");
  vector<string> dNote;
  /*
  dNote.push_back("u3");
  dNote.push_back("2");*/
  remove_dangling_patch(cooked_patches, dNote);
  
  vector< shared_ptr<Patch> > patches;
  Env env;
  tiling_by_example(patches, env, cooked_patches, "general", 20);
  
  //if (false)
  {
	PatchesViewer * v = new PatchesViewer(patches, env);
	v->get_camera().distance += 2;

	vector<ml::Motion> view_m;
	convert_patch_to_motion(view_m, patches);
    DontSlip noslipfoot;
    for (auto it = view_m.begin(); it != view_m.end(); ++it) {
	  noslipfoot.hold_foot(*it);
	}
    MotionsViewer * mv = new MotionsViewer(view_m);
	mv->get_camera().distance += 2;
  }


  if (false)
  {
    map<string, Patch> *patch_type = new map<string, Patch>();
    get_patch(patch_type);
    vector< shared_ptr<Patch> > patch_type_vector;
    for (auto it = patch_type->begin(); it != patch_type->end(); ++it) {
	  patch_type_vector.push_back(shared_ptr<Patch>(new Patch(it->second)));
	}

    vector< shared_ptr<Patch> > result_patches;
    Env env;
    tiling_by_example(result_patches, env, patch_type_vector, "dance", 2);
    
    /*DontSlip noslipfoot(18, 19, 24, 3, 4, 20);
    for (auto it = patches.begin(); it != patches.end(); ++it) noslipfoot.hold_foot(**it);*/
    PatchesViewer * v = new PatchesViewer(result_patches, env);
    v->get_camera().distance += 2;
  }

  
  if (false)
  {
    double offset = 0.0;
    for (auto it = cooked_patches.begin(); it != cooked_patches.end(); ) {
      /*if ((*it)->motions.size() == 1) {
      it = cooked_patches.erase(it);
      } else */
      {
        (*it)->translate(cml::vector3(offset, 0., 0.));
        ++it;
        offset += 7.0;
      }
    }
    PatchesViewer * v = new PatchesViewer(cooked_patches);
    v->get_camera().distance += 2;
  }
}

TEST(CTiling, DISABLED_patchView)
{
  vector< shared_ptr<Patch> > cooked_patches;
  //read_patches(cooked_patches, "hand_chicken");
  read_patches(cooked_patches, "cooked_rawData_Hello_combi_3");

  double offset = 0.0;
  for (auto it = cooked_patches.begin(); it != cooked_patches.end(); ) {
    /*if ((*it)->motions.size() == 1) {
      it = cooked_patches.erase(it);
    } else */
    {
      (*it)->translate(cml::vector3(offset, 0., 0.));
      ++it;
      offset += 7.0;
    }
  }
  DontSlip noslipfoot;
  for (auto it = cooked_patches.begin(); it != cooked_patches.end(); ++it) noslipfoot.hold_foot(**it);
  PatchesViewer * v = new PatchesViewer(cooked_patches);
  v->get_camera().distance += 2;	
}

TEST(DISABLED_tiling_mm)
{
  //if (false)
  {
    vector< shared_ptr<Patch> > cooked_patches;
    read_patches(cooked_patches, "cooked_rawData_chicken_combi_6");
    
    // remove the patch that has "-1" boundary type.
    remove_dangling_patch(cooked_patches);
    
    map<string,Patch> patch_types;
    vector<Patch> unary_patch;
    vector<string> sampling_patches;
    for (auto it = cooked_patches.begin(); it != cooked_patches.end(); ++it) {
      patch_types[(*it)->name] = (**it);
      if ((*it)->motions.size() == 1) {
        unary_patch.push_back(**it);
      }
      bool is_connectable = true; 
      for (auto jt = (*it)->boundaries.begin(); jt != (*it)->boundaries.end(); ++jt) {
        if (jt->posture_type() == -1) is_connectable = false;	  
      }
      if (is_connectable) sampling_patches.push_back((*it)->name);
    }    

    //phase1이든 phase2이든 중간에 멈추려면 bin/control.txt를 -stop에서 stop으로 바꾼다. 그 때의 패치들을 파일로 저장하고 끝남.
    //phase1 parameter
    srand(79797+16+3+2);
    //environment
    Env env;
    env.set_scatter_range(-5,5,-5,5, 100, 800);		
    env.static_boxes.push_back(Box(trans_transf(cml::vector3(0, 0.625, -6.5)), cml::vector3(14, 1.25, 1)));
    env.static_boxes.push_back(Box(trans_transf(cml::vector3(0, 0.625, 6.5)), cml::vector3(14, 1.25, 1)));
    env.static_boxes.push_back(Box(trans_transf(cml::vector3(-6.5, 0.625, 0)), cml::vector3(1, 1.25, 14)));
    env.static_boxes.push_back(Box(trans_transf(cml::vector3(6.5, 0.625, 0)), cml::vector3(1, 1.25, 14)));		

    std::ostringstream oss;
    time_t start_time;
    time(&start_time);
    oss << start_time;
    string save_file_str(oss.str());

    srand(79797+5+2);
    vector< shared_ptr<Patch> > patches;

    //random scat
    for (int i = 0; i < 3; ++i)
    {
      random_shuffle(sampling_patches.begin(), sampling_patches.end());
      while(true)
      {
        for (auto it_p = sampling_patches.begin(); it_p != sampling_patches.end(); ++it_p)
        {
          shared_ptr<Patch> pa1(new Patch((patch_types.find(*it_p))->second));
          if (pa1->motions.size() == 1)
            continue;
          env.rand_scatter(*pa1);

          vector<ConnectInfo> connect_infos;
          get_connect_infos(connect_infos, pa1, patches, 4.0, env);

          if (stitch(patches, pa1, &patch_types, connect_infos, env) == true) {
            goto one_tile;
          }
        }
      }
      one_tile:;
    }

    //2nd Phase
    //deterministic(patches, env, &patch_types);
    remove_dangling(patches, unary_patch, &patch_types, env, cout);
    set_color(patches);

    //vector< shared_ptr<Patch> > patches;
    //all_phase_tiling(patches, env, &patch_types, &unary_patch, max_time, max_num, max_attempt, sampling_patches, k, alpha, save_file_str);
    //shared_ptr<Patch> pa1(new Patch((patch_type->find(*it_p))->second));
    //env.rand_scatter(*pa1);

  /*  DontSlip noslipfoot(18, 19, 24, 3, 4, 20);
    for (auto it = patches.begin(); it != patches.end(); ++it) noslipfoot.hold_foot(**it);*/

    PatchesViewer * v = new PatchesViewer(patches, env);
    v->get_camera().distance += 2;
  }

  if (false)
  {
    vector<shared_ptr<Patch>> patches;
    Env env;
    read_patches_env(patches, env, "240_10_15000_0.2_0.0015_phase2");

    //DontSlip noslipfoot(18, 19 , 24, 3, 4, 20);
    //for (auto it = patches.begin(); it != patches.end(); ++it) noslipfoot.hold_foot(**it);

    PatchesViewer * v = new PatchesViewer(patches, env);
    v->get_camera().distance += 2;
  }
}

TEST(Ingredient, DISABLED_view)
{
  Ingredient * ingre = new Ingredient();
  //mix_by_itself(*ingre);
  mix_raw_data(*ingre);

  for (size_t i = 0; i < ingre->bunch_size(); ++i) {
	MotionsViewer * v = new MotionsViewer(ingre->raw_before_cook[i]);
	v->get_camera().distance += 2;
  }
}

TEST(Ingredient, DISABLED_dofinder)
{
  Ingredient * ingre = new Ingredient();
  //mix_by_itself(*ingre);
  mix_raw_data(*ingre);

  PatchCooker cooker_(ingre);
  vector<pair_section> psec = cooker_.init_interactions();

  for (size_t i = 0; i < ingre->bunch_size(); ++i) {
	vector<pair_section> bunch_sect;
	for (auto it = psec.begin(); it != psec.end(); ++it) {
	  if (it->bunch_idx == i) {
		bunch_sect.push_back(*it);
	  }
	}	
	MotionsViewer * v = new MotionsViewer(ingre->raw_before_cook[i]);
	v->interact_pair = bunch_sect;
	v->get_camera().distance += 2;
  }  
}

TEST(Ingredient, DISABLED_cluster_tree)
{
  Ingredient * ingre = new Ingredient();
  //mix_by_itself(*ingre);
  mix_raw_data(*ingre);

  PatchCooker cooker_(ingre);
  vector<pair_section> psec = cooker_.init_interactions();
  cooker_.init_EETable();
  cooker_.agglomerative_cluster();
  
  for (size_t i = 0; i < ingre->bunch_size(); ++i) {
	vector<pair_section> bunch_sect;
	for (auto it = psec.begin(); it != psec.end(); ++it) {
	  if (it->bunch_idx == i) {
		bunch_sect.push_back(*it);
	  }
	}
	MotionsViewer * v = new MotionsViewer(ingre->raw_before_cook[i]);
	v->interact_pair = bunch_sect;
	v->get_camera().distance += 2;
  }    
}

TEST(Ingredient, DISABLED_mold_patches)
{
  Ingredient * ingre = new Ingredient();
  //mix_by_itself(*ingre);
  mix_raw_data(*ingre);

  PatchCooker cooker_(ingre);
  vector<pair_section> psec = cooker_.init_interactions();
  cooker_.init_EETable();
  cooker_.agglomerative_cluster();
  cooker_.mold_motion_patches();

  for (size_t i = 0; i < ingre->bunch_size(); ++i) {
	vector<pair_section> bunch_sect;
	for (auto it = psec.begin(); it != psec.end(); ++it) {
	  if (it->bunch_idx == i) {
		bunch_sect.push_back(*it);
	  }
	}
	MotionsViewer * v = new MotionsViewer(ingre->raw_before_cook[i]);
	v->interact_pair = bunch_sect;
	v->get_camera().distance += 2;
  }    
}

TEST(Ingredient, DISABLED_cook_patches)
{
  Ingredient * ingre = new Ingredient();
  //mix_by_itself(*ingre);
  mix_raw_data(*ingre);
  cout << "motion size: " << ingre->size_mot() << " & " << "posture size: " << ingre->size_pos() << endl;
  PatchCooker cooker_(ingre);

  time_t start_time;
  time(&start_time);
  vector<pair_section> psec = cooker_.init_interactions();
  time_t cur_time;
  time(&cur_time);
  cout << "Interactions Detecting time is " << difftime(cur_time, start_time) << endl; 

  cooker_.init_EETable();

  time_t mid_time;
  time(&mid_time);
  cooker_.agglomerative_cluster();
  time(&cur_time);
  cout << "Clustering time is " << difftime(cur_time, mid_time) << endl; 

  /*size_t boundary_pose_size = 6;
  cooker_.mold_motion_patches(boundary_pose_size);
  cooker_.cook_motion_patches();*/
  cooker_.tcook_motion_patches();
  time(&cur_time);
  cout << "Total time is " << difftime(cur_time, start_time) << endl; 

  if (!cooker_.patches->empty()) {
	cooker_.evaluate_this_system();
	string save_file_str = "cooked_rawData_chicken_6";
	//write_patches(cooker_.get_all_patches(), save_file_str);
	PatchesViewer * v = new PatchesViewer(cooker_.get_all_patches());
	//v->set_origin_motions(cooker_);
	v->get_camera().distance += 2;		
  }

  for (size_t i = 0; i < ingre->bunch_size(); ++i) {
	vector<pair_section> bunch_sect;
	for (auto it = psec.begin(); it != psec.end(); ++it) {
	  if (it->bunch_idx == i) {
		bunch_sect.push_back(*it);
	  }
	}
	MotionsViewer * v = new MotionsViewer(ingre->raw_before_cook[i]);
	v->interact_pair = bunch_sect;
	v->get_camera().distance += 2;
  } 
}


void display(char* begin,char* end)
{
  cout << begin << endl;
}
  
TEST(Algorithm, DISABLED_recursiveComb)
{
  char ca[]="0123456789";
  char cb[]="01234";

  stdcomb::recursive_combination(ca, ca + 10, 0,
	cb, cb + 5, 0, 10 - 5, display);
}

template<class BidIt>
void display(BidIt begin,BidIt end)
{
  for (BidIt it = begin; it != end; ++it)
	cout << *it << " ";
  cout << endl;
}

TEST(Algorithm, DISABLED_nonRecursiveComb)
{
  vector<int> ca;
  ca.push_back (1);
  ca.push_back (2);
  ca.push_back (3);
  ca.push_back (4);
  ca.push_back (5);
  ca.push_back (6);

  vector<int> cb;
  cb.push_back (1);
  cb.push_back (2);
  cb.push_back (3);
  cb.push_back (4);

  do {
	display(cb.begin(),cb.end());
  }
  while (stdcomb::next_combination(ca.begin(), ca.end(), cb.begin(), cb.end()));

  cout<<"Complete!"<<endl;  
}

TEST(Ingredient, DISABLED_comb_determine)
{
  Ingredient * ingre = new Ingredient();
  //mix_by_itself(*ingre);
  mix_raw_data(*ingre);
  cout << "motion size: " << ingre->size_mot() << " & " << "posture size: " << ingre->size_pos() << endl;
  PatchCooker cooker_(ingre);

  time_t start_time;
  time(&start_time);
  vector<pair_section> psec = cooker_.init_interactions();
  time_t cur_time;
  time(&cur_time);
  cout << "Interactions Detecting time is " << difftime(cur_time, start_time) << endl; 

  cooker_.init_EETable();

  time_t mid_time;
  time(&mid_time);
  cooker_.agglomerative_cluster(3.5);
  time(&cur_time);
  cout << "Clustering time is " << difftime(cur_time, mid_time) << endl; 

  cooker_.comb_cook_motion_patches(4);
  time(&cur_time);
  cout << "Total time is " << difftime(cur_time, start_time) << endl; 
  
  vector<pair_section> bunch_sect;
  for (size_t i = 0; i < ingre->bunch_size(); ++i) {	
	for (auto it = psec.begin(); it != psec.end(); ++it) {
	  if (it->bunch_idx == i) {
		bunch_sect.push_back(*it);
	  }
	}	
	MotionsViewer * v = new MotionsViewer(ingre->raw_before_cook[i]);
	v->interact_pair = bunch_sect;
	v->get_camera().distance += 2;	
  }
  if (!cooker_.patches->empty()) {
	cooker_.evaluate_this_system();	
	PatchesViewer * pv = new PatchesViewer(cooker_.get_all_patches());
	pv->interact_pair = bunch_sect;
	pv->set_origin_motions(cooker_);
	pv->get_camera().distance += 2;	
  }
}

TEST(Ingredient, DISABLED_combi_save_patches)
{
  Ingredient * ingre = new Ingredient();
  //mix_by_itself(*ingre);
  mix_raw_data(*ingre);
  cout << "motion size: " << ingre->size_mot() << " & " << "posture size: " << ingre->size_pos() << endl;
  PatchCooker cooker_(ingre);

  time_t start_time;
  time(&start_time);
  vector<pair_section> psec = cooker_.init_interactions();
  time_t cur_time;
  time(&cur_time);
  cout << "Interactions Detecting time is " << difftime(cur_time, start_time) << endl; 

  cooker_.init_EETable();

  time_t mid_time;
  time(&mid_time);
  cooker_.agglomerative_cluster();
  time(&cur_time);
  cout << "Clustering time is " << difftime(cur_time, mid_time) << endl; 

  cooker_.comb_cook_motion_patches();
  time(&cur_time);
  cout << "Total time is " << difftime(cur_time, start_time) << endl; 

  if (!cooker_.patches->empty()) {
	cooker_.evaluate_this_system();	
	
	string save_file_str = "cooked_rawData_chicken_combi_6";
	write_patches(cooker_.get_all_patches(), save_file_str);

	PatchesViewer * v = new PatchesViewer(cooker_.get_all_patches());
	//v->set_origin_motions(cooker_);
	v->get_camera().distance += 2;	
  }
}


