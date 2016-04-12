#include "StdAfx.h"
#include "motions_viewer.h"
#include "patches_viewer.h"
#include "motion_edit.h"
#include "tiling.h"
#include "get_patch.h"
#include "nonslipfoot.h"


class LocalTilingTest : public testing::Test 
{
 protected:

  static map<string, Patch> *patch_type;
  static vector<Patch> *patch_type_unary;	

  static void SetUpTestCase() {
	patch_type = new map<string, Patch>();
	patch_type_unary = new vector<Patch>();
	get_patch(patch_type);		
	//get_patch_unary(patch_type_unary, patch_type);
  }

  static void TearDownTestCase() {
	delete patch_type;
	delete patch_type_unary;
	patch_type = NULL;
	patch_type_unary = NULL;  
  } 

};

map<string, Patch> *LocalTilingTest::patch_type = NULL;
vector<Patch> *LocalTilingTest::patch_type_unary= NULL;


TEST_F( LocalTilingTest, DISABLED_a_patch)
{
	vector<shared_ptr<Patch>> patches;
	patches.push_back( get_a_patch( "jump", *patch_type));
	
	PatchesViewer *v = new PatchesViewer( patches);
	v->get_camera().distance += 2;
}


TEST_F( LocalTilingTest, DISABLED_dangling_table)
{
	Env env_;
	env_.set_scatter_range( -5, 5, -5, 5, 100, 200);

	vector< shared_ptr< Patch>> patches;
	shared_ptr<Patch> temp = get_a_patch( "jump", *patch_type);
	env_.rand_scatter( *temp);
	patches.push_back( temp);

	Danglings dtable;
	get_naive_dangling_table(dtable, patches, env_);

	set_color(patches);
	PatchesViewer *v = new PatchesViewer( patches);
	v->get_camera().distance += 2;
}

TEST_F( LocalTilingTest, DISABLED_stitch_a_patch)
{
	Env env_;
	env_.set_scatter_range( -5, 5, -5, 5, 100, 200);

	vector<shared_ptr<Patch> > patches;
	shared_ptr<Patch> temp = get_a_patch("jump", *patch_type);
	env_.rand_scatter(*temp);
	patches.push_back(temp);

	Danglings dtable;
	get_naive_dangling_table(dtable, patches, env_);
	int didx = select(dtable);
	if (didx > -1)
	  stitch_a_patch(patches, dtable, didx, *patch_type, "jump", env_);

	set_color(patches);
	PatchesViewer *v = new PatchesViewer(patches, env_);
	v->get_camera().distance += 2;
}

TEST_F( LocalTilingTest, DISABLED_stitch_all_patches)
{
	Env env_;
	env_.set_scatter_range( -8, 8, -8, 8, 200, 800);

	vector< shared_ptr< Patch>> patches;
	shared_ptr<Patch> temp = get_a_patch("jump", *patch_type);
	env_.rand_scatter(*temp);
	patches.push_back(temp);

	stitch_all_patches(patches, *patch_type, env_);

	set_color(patches);
	PatchesViewer *v = new PatchesViewer( patches, env_);
	v->get_camera().distance += 2;
}

//TEST_F( LocalTilingTest, fill_stage)
//{
//	Env env_;
//	env_.set_scatter_range( -10, 10, -10, 10, 200, 1000);
//
//	vector< shared_ptr<Patch> > patches;
//	shared_ptr<Patch> temp = get_a_patch("jump", *patch_type);
//	env_.scatter(*temp);
//	patches.push_back(temp);
//	
//	Danglings dtable;	
//	get_naive_dangling_table( dtable, patches, env_);
//	size_t limit = 0;
//	while (!dtable.empty()) {
//		get_naive_dangling_table(dtable, patches, env_);	
//		stitch_patches(patches, dtable, *patch_type, env_);		
//		if (++limit > 9)
//			break;
//	}
//	set_color(patches);
//	PatchesViewer *v = new PatchesViewer(patches, env_);
//	v->get_camera().distance += 2;
//
//	for( auto it=patch_type->begin(); it!=patch_type->end(); ++it)
//		v->patch_type->operator[](it->first) = it->second;
//}

TEST_F(LocalTilingTest, DISABLED_procedural_viewer)
{
  Env env_;
  env_.set_scatter_range(-10, 10, -10, 10, 100, 600);
  vector< shared_ptr<Patch> > patches;
  shared_ptr<Patch> temp = get_a_patch( "jump", *patch_type);
  env_.rand_scatter( *temp);
  patches.push_back( temp);	
  
  // must initialize viewer's patch_type variable
  PatchesViewer *v = new PatchesViewer(patches, patch_type, env_);
  v->get_camera().distance += 2;
}

TEST_F(LocalTilingTest, DISABLED_PatchesToMotions)
{
  Env env_;
  env_.set_scatter_range(-9, 9, -9, 9, 100, 600);
  vector< shared_ptr<Patch> > patches;
  shared_ptr<Patch> seed = get_a_patch("jump", *patch_type);
  env_.rand_scatter(*seed);
  patches.push_back(seed);

  stitch_patches(patches, 6, *patch_type, env_);  
  PatchesViewer *v = new PatchesViewer(patches, env_);
  v->get_camera().distance += 2;

  vector<ml::Motion> converted_motions;
  convert_patch_to_motion(converted_motions, patches);
  double t_time = begin_time(converted_motions);
  for_each(converted_motions.begin(), converted_motions.end(), [t_time](ml::Motion &mot){ mot.translate_time(-t_time);});

  MotionsViewer *mv = new MotionsViewer(converted_motions);
  mv->get_camera().distance += 2;
}

TEST_F(LocalTilingTest, DISABLED_interative_user_interface)
{
  // viewer에 ctrl키를 누르면서 마우스 드래그를 하면 원이 그려지고 그 원 안에 들어가는 posture들이 선택되는 interface 구현 ??
  //
}

