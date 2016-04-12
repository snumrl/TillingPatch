#include "StdAfx.h"
#include "patch_cooker.h"
#include "get_patch.h"
#include "nonslipfoot.h"
#include "motions_viewer.h"
#include "patches_viewer.h"
#include "execute.h"
#include "tiling.h"

#include "../combination/combination.h"


TEST(Example, DISABLED_makeChicken)
{
  vector< shared_ptr<Patch> > ms;
  make_patch_chicken(ms);
  write_patches(ms, "hand_chicken");
}

TEST(Example, DISABLED_chicken1)
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
  cooker_.agglomerative_cluster(6.5, 4);
  time(&cur_time);
  cout << "Clustering time is " << difftime(cur_time, mid_time) << endl;   

 // for (size_t i = 0; i < ingre->bunch_size(); ++i) {
	//vector<pair_section> bunch_sect;
	//for (auto it = psec.begin(); it != psec.end(); ++it) {
	//  if (it->bunch_idx == i) {
	//	bunch_sect.push_back(*it);
	//  }
	//}
	//MotionsViewer * v = new MotionsViewer(ingre->raw_before_cook[i]);
	//v->interact_pair = bunch_sect;
	//v->get_camera().distance += 2;
 // }

  cooker_.comb_cook_motion_patches(2);
  time(&cur_time);
  cout << "Total time is " << difftime(cur_time, start_time) << endl; 

  if (!cooker_.patches->empty()) {
	cooker_.evaluate_this_system();	

	string save_file_str = "cooked_rawData_chicken_combi_2";
	write_patches(cooker_.get_all_patches(), save_file_str);

	PatchesViewer * v = new PatchesViewer(cooker_.get_all_patches());
	//v->set_origin_motions(cooker_);
	v->get_camera().distance += 2;	
  }
}

TEST(Example, DISABLED_dance)
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
  cooker_.agglomerative_cluster(7.7);
  time(&cur_time);
  cout << "Clustering time is " << difftime(cur_time, mid_time) << endl;   

 // for (size_t i = 0; i < ingre->bunch_size(); ++i) {
 // vector<pair_section> bunch_sect;
	//for (auto it = psec.begin(); it != psec.end(); ++it) {
	//  if (it->bunch_idx == i) {
	//	bunch_sect.push_back(*it);
	//  }
	//}
	//MotionsViewer * v = new MotionsViewer(ingre->raw_before_cook[i]);
	//v->interact_pair = bunch_sect;
	//v->get_camera().distance += 2;
 // }

  cooker_.comb_cook_motion_patches(3);
  time(&cur_time);
  cout << "Total time is " << difftime(cur_time, start_time) << endl; 

  if (!cooker_.patches->empty()) {
	cooker_.evaluate_this_system();	

	string save_file_str = "cooked_rawData_dance_combi_3";
	write_patches(cooker_.get_all_patches(), save_file_str);

	PatchesViewer * v = new PatchesViewer(cooker_.get_all_patches());
	//v->set_origin_motions(cooker_);
	v->get_camera().distance += 2;	
  }
}


TEST(Example, DISABLED_catchMeIfYouCan)
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
  cooker_.agglomerative_cluster(7.8);
  time(&cur_time);
  cout << "Clustering time is " << difftime(cur_time, mid_time) << endl;   

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

  cooker_.comb_cook_motion_patches(7);
  time(&cur_time);
  cout << "Total time is " << difftime(cur_time, start_time) << endl; 

  if (!cooker_.patches->empty()) {
	cooker_.evaluate_this_system();	

    string save_file_str = "cooked_rawData_catchMe_combi_7";
	write_patches(cooker_.get_all_patches(), save_file_str);

	PatchesViewer * v = new PatchesViewer(cooker_.get_all_patches());
	//v->set_origin_motions(cooker_);
	v->get_camera().distance += 2;	
  }
}



TEST(Example, DISABLED_streetFight)
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
  cooker_.agglomerative_cluster(4.0);
  time(&cur_time);
  cout << "Clustering time is " << difftime(cur_time, mid_time) << endl;   

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

  cooker_.comb_cook_motion_patches(4);
  time(&cur_time);
  cout << "Total time is " << difftime(cur_time, start_time) << endl; 

  if (!cooker_.patches->empty()) {
	cooker_.evaluate_this_system();	

	string save_file_str = "cooked_rawData_streetFight_combi_4";
	//write_patches(cooker_.get_all_patches(), save_file_str);

	PatchesViewer * v = new PatchesViewer(cooker_.get_all_patches());
	//v->set_origin_motions(cooker_);
	v->get_camera().distance += 2;	
  }
}


TEST(Example, DISABLED_hello)
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
  cooker_.agglomerative_cluster(6.5);
  time(&cur_time);
  cout << "Clustering time is " << difftime(cur_time, mid_time) << endl;   

  //for (size_t i = 0; i < ingre->bunch_size(); ++i) {
  //  vector<pair_section> bunch_sect;
  //  for (auto it = psec.begin(); it != psec.end(); ++it) {
  //    if (it->bunch_idx == i) {
  //      bunch_sect.push_back(*it);
  //    }
  //  }

  //  MotionsViewer * v = new MotionsViewer(ingre->raw_before_cook[i]);
  //  v->interact_pair = bunch_sect;
  //  v->get_camera().distance += 2;
  //}

  cooker_.comb_cook_motion_patches(3);
  time(&cur_time);
  cout << "Total time is " << difftime(cur_time, start_time) << endl; 

  if (!cooker_.patches->empty()) {
    cooker_.evaluate_this_system();	

    string save_file_str = "cooked_rawData_Hello_combi_3";
    write_patches(cooker_.get_all_patches(), save_file_str);

    PatchesViewer * v = new PatchesViewer(cooker_.get_all_patches());
    //v->set_origin_motions(cooker_);
    v->get_camera().distance += 2;	
  }
}
