#include "StdAfx.h"
#include "patch_constructor.h"
#include "patches_viewer.h"


class PatchConsTest : public testing::Test
{
protected:
  static Patch_constructor * pconstructor;
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

	pconstructor = new Patch_constructor(motions);
  }

  static void TearDownTestCase() {
	delete pconstructor;	
	delete motions;
  }
};

Patch_constructor * PatchConsTest::pconstructor= NULL;
vector<ml::Motion> *PatchConsTest::motions = NULL;


TEST_F(PatchConsTest, DISABLED_make_real_pat)
{
  section intm;
  intm.mot_idx = 0;
  intm.pos_sec = pair<size_t,size_t>(5 , 25);
  pconstructor->set_relation(intm);
  intm.mot_idx = 1;
  intm.pos_sec = pair<size_t,size_t>(5 , 25);  
  pconstructor->set_relation(intm);

  pconstructor->realize_motion_data();

  vector< shared_ptr<Patch> > patches;
  patches.push_back(shared_ptr<Patch>(new Patch(*pconstructor->get_patch())));
  PatchesViewer * v = new PatchesViewer(patches);
  v->get_camera().distance += 2;  
}


