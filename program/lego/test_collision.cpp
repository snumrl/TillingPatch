#include "StdAfx.h"
#include "collision.h"
#include "tiling.h"
#include "motions_viewer.h"
#include "patches_viewer.h"
#include "QPerformanceTimer.h"

TEST(collision_posture_obb) {
	ml::Motion m;
	m.LoadAMC_with_contactInfo("./data/1/test_motion.amc", "./data/wd2.asf", true, 0.027);

	ml::Posture p1 = m[0];
	EXPECT_TRUE(collision_posture_obb(p1, p1));
	
	ml::Posture p2 = p1;
	p2.ApplyTransf(cml::trans_transf(cml::vector3(-0.2,0,0.5)));
	EXPECT_TRUE(collision_posture_obb(p1, p2));

	ml::Posture p3 = p1;
	p3.ApplyTransf(cml::trans_transf(cml::vector3(0.2*1.3,0,-0.5*1.3)));
	EXPECT_FALSE(collision_posture_obb(p1, p3));

	////1000번 실험하고 profiling
	//BEGIN_TIMER(collision_1000);
	//for (int i = 0; i < 1000; ++i)
	//	collision_posture(p1, p3);
	//END_TIMER(collision_1000);
	
	if (false)
	{
		vector<ml::Posture> ps;
		ps.push_back(p1);
		ps.push_back(p2);
		ps.push_back(p3);
		MotionsViewer * v = new MotionsViewer(ps);
		v->get_camera().rotateY += 1.1;
	}
}

TEST(collision_posture_box_obb) {
	ml::Motion m;
	m.LoadAMC_with_contactInfo("./data/3/pass_mid_to_mid.amc", "./data/wd2.asf", true, 0.027);

	ml::Posture p1 = m[624+63];
	p1.object = 1;

	ml::Posture p2 = p1;
	p2.ApplyTransf(cml::trans_transf(cml::vector3(-0.7,0,0.0)));
	EXPECT_TRUE(collision_posture_obb(p1, p2));

	ml::Posture p3 = p1;
	p3.ApplyTransf(cml::trans_transf(cml::vector3(-0.8,0,0.0)));
	EXPECT_TRUE(collision_posture_obb(p1, p3));

	ml::Posture p4 = p1;
	p4.ApplyTransf(cml::trans_transf(cml::vector3(-0.91,0,0.0)));
	EXPECT_FALSE(collision_posture_obb(p1, p4));

	ml::Posture p5 = p1;
	p5.ApplyTransf(cml::trans_transf(cml::vector3(-1.0,0,0.0)));
	EXPECT_FALSE(collision_posture_obb(p1, p5));

	if (false)
	{
		vector<ml::Posture> ps;
		ps.push_back(p1);
		ps.push_back(p2);
		ps.push_back(p3);
		ps.push_back(p4);
		ps.push_back(p5);
		MotionsViewer * v = new MotionsViewer(ps);
	}
}

TEST(collision_posture_exact) {
	ml::Motion m;
	m.LoadAMC_with_contactInfo("./data/1/test_motion.amc", "./data/wd2.asf", true, 0.027);
	ml::Posture p1 = m[0];
	ml::Posture p2 = p1;

	vector<ml::Posture> ps;
	ps.push_back(p1);

	for (int i = 0; i < 50; ++i)
	{
		p2.ApplyTransf(cml::trans_transf(cml::vector3(0.02 * 0.4, 0, -0.02)));
		ps.push_back(p2);

		if (i <= 29) {
			EXPECT_EQ(true, collision_posture_obb(p1, p2)); 
		}
		else
		{
			EXPECT_EQ(false, collision_posture_obb(p1, p2));
		}
	}

	if (false)
	{
		MotionsViewer * v = new MotionsViewer(ps);
		v->get_camera().rotateY += 1.1;
	}
}

TEST(collision_postures_obb) {
	ml::Motion m;
	m.LoadAMC_with_contactInfo("./data/1/test_motion.amc", "./data/wd2.asf", true, 0.027);
	ml::Posture posture = m[0];
	double maximum_radius = get_maximum_radius(m);
			
	vector<const ml::Posture *> ps1;
	for (int i = 0; i < 70; ++i) {
		ml::Posture * p = new ml::Posture(posture);
		p->ApplyTransf(cml::trans_transf(cml::vector3(0, 0, i * -0.5 + 1.25)));
		ps1.push_back(p);
	}

	vector<const ml::Posture *> ps2;
	for (int i = 0; i < 6; ++i) {
		ml::Posture * p = new ml::Posture(posture);
		p->ApplyTransf(cml::trans_transf(cml::vector3(3 + i * 0.5 - 1.25,0,3)));
		ps2.push_back(p);
	}

	vector<const ml::Posture *> ps3;
	for (int i = 0; i < 6; ++i) {
		ml::Posture * p = new ml::Posture(posture);
		p->ApplyTransf(cml::trans_transf(cml::vector3(i * 0.5 - 1.4,0,0)));
		ps3.push_back(p);
	}

	//BEGIN_TIMER(postures_test);
	EXPECT_EQ(false, collision_postures_obb(ps1, ps2, maximum_radius));
	//END_TIMER(postures_test);
		
	EXPECT_EQ(true, collision_postures_obb(ps1, ps3, maximum_radius));

	EXPECT_EQ(false, collision_postures_obb_nocull(ps1, ps2, maximum_radius));
	EXPECT_EQ(true, collision_postures_obb_nocull(ps1, ps3, maximum_radius));
	
	if (false)
	{
		vector<const ml::Posture *> ps = ps1;
		copy(ps3.begin(), ps3.end(), back_inserter(ps));
		copy(ps2.begin(), ps2.end(), back_inserter(ps));
		MotionsViewer * v = new MotionsViewer(ps);
		v->get_camera().rotateX -= 0.2;
		v->get_camera().distance += 7;
	}
}

TEST(collision_motion) {
	ml::Motion m1;
	m1.LoadAMC_with_contactInfo("./data/1/test_motion.amc", "./data/wd2.asf", true, 0.027);
	EXPECT_TRUE(collision_motion(m1, m1) == 1);

	ml::Motion m2 = m1;
	m2.rotate(cml::pi());
	m2.translate(cml::vector3(-0.19,0,1.07));
	EXPECT_TRUE(collision_motion(m1, m2) == 7);

	ml::Motion m3 = m2;
	m3.translate(cml::vector3(0,0,-1));
	EXPECT_TRUE(collision_motion(m1, m3) == -1);

	if (false)
	{
		vector<ml::Motion> ms;
		ms.push_back(m1);
		ms.push_back(m2);
		ms.push_back(m3);
		MotionsViewer * v = new MotionsViewer(move(ms));
	}
}

TEST(collision_motion_obb) {
	ml::Motion m1;
	m1.LoadAMC_with_contactInfo("./data/1/test_motion.amc", "./data/wd2.asf", true, 0.027);
	EXPECT_EQ(1, collision_motion_obb(m1, m1));

	ml::Motion m2 = m1;
	m2.rotate(cml::pi());
	m2.translate(cml::vector3(-0.19,0,1.07));
	EXPECT_EQ(7, collision_motion_obb(m1, m2));

	ml::Motion m3 = m2;
	m3.translate(cml::vector3(0,0,-1));
	EXPECT_EQ(-1, collision_motion_obb(m1, m3));

	if (false)
	{
		//100번 실험하고 profiling
		BEGIN_TIMER(motion_100);
		for (int i = 0; i < 100; ++i)
			collision_motion(m1, m2);
		END_TIMER(motion_100);

		BEGIN_TIMER(motion_100_obb);
		for (int i = 0; i < 100; ++i)
			collision_motion_obb(m1, m2);
		END_TIMER(motion_100_obb);

		vector<ml::Motion> ms;
		ms.push_back(m1);
		ms.push_back(m2);
		ms.push_back(m3);
		MotionsViewer * v = new MotionsViewer(move(ms));
	}
}



class CollisionTest : public testing::Test {
protected:  
	static void SetUpTestCase() {
		pa = new Patch();

		ml::Motion base_motion;
		base_motion.LoadAMC_with_contactInfo("./data/3/pass_mid_to_mid.amc", "./data/wd2.asf", true, 0.027);

		ml::Motion m1;
		ml::Motion m2;
		m1.copy(base_motion, base_motion.begin()+452, base_motion.begin()+585);
		m1.translate(cml::vector3d(-2-0.1,0,-0.17));
		m1.translate_time_to_zero();
		m1.translate_time(13);

		m2.copy(base_motion, base_motion.begin()+624, base_motion.begin()+762);
		m2.translate(cml::vector3d(2.56+0.04-0.1,0,0.035));
		m2.translate_time_to_zero();

		pa->add_motion(m1);
		pa->add_motion(m2);
		pa->set_boundaries();
		add_cons_rel_pos(pa->inner_cons, pa->motions, 0, 65-13, 1, 65);
		add_cons_same_time(pa->inner_cons, 0, 65-13, 1, 65);
	}
	static void TearDownTestCase() {
		delete pa;
		pa = NULL;
	}

	static Patch *pa;
};

Patch * CollisionTest::pa = NULL;

TEST_F(CollisionTest, patch) {
	vector<shared_ptr<Patch>> patches;	
	for (int i = 0; i < 3; ++i) {
		shared_ptr<Patch> pa_(new Patch(*pa));
		patches.push_back(pa_);
	}
	patches[1]->translate(cml::vector3(3.8,0,1.0));
	patches[2]->translate(cml::vector3(0,0,1));

	EXPECT_TRUE(collision_patch(*patches[0], *patches[0]) == 1);
	EXPECT_TRUE(collision_patch(*patches[0], *patches[1]) == 128);
	EXPECT_TRUE(collision_patch(*patches[0], *patches[2]) == -1);

	if (false) 
	{
		PatchesViewer * v = new PatchesViewer(patches);
	}
}

TEST_F(CollisionTest, patch_obb) {
	vector<shared_ptr<Patch>> patches;	
	for (int i = 0; i < 3; ++i) {
		shared_ptr<Patch> pa_(new Patch(*pa));
		patches.push_back(pa_);
	}
	patches[1]->translate(cml::vector3(3.8,0,1.0));
	patches[2]->translate(cml::vector3(0,0,1));

	EXPECT_TRUE(collision_patch_obb(*patches[0], *patches[0]) == 1);
	EXPECT_TRUE(collision_patch_obb(*patches[0], *patches[1]) == 128);
	EXPECT_TRUE(collision_patch_obb(*patches[0], *patches[2]) == -1);

	if (false) 
	{
		BEGIN_TIMER(patch_10);
		for (int i = 0; i < 10; ++i)
			collision_patch(*patches[0], *patches[1]);
		END_TIMER(patch_10);

		BEGIN_TIMER(patch_10_obb);
		for (int i = 0; i < 10; ++i)
			collision_patch_obb(*patches[0], *patches[1]);
		END_TIMER(patch_10_obb);

		PatchesViewer * v = new PatchesViewer(patches);
	}
}

set<int> array_to_set(int size, int a[])
{
	set<int> s(a, a+size);
	return s;
}

TEST_F(CollisionTest, DISABLED_patches_nopreprocess) {
	vector<shared_ptr<Patch>> patches;	
	for (int i = 0; i < 3; ++i) {
		shared_ptr<Patch> pa_(new Patch(*pa));
		patches.push_back(pa_);
	}
	patches[1]->translate(cml::vector3(3.8,0,1.0));
	patches[2]->translate(cml::vector3(0,0,1.1));
	patches[2]->translate_time(10);

	{
		int c[] = {0};
		EXPECT_TRUE(collision_patches_nopreprocess(patches, array_to_set(1, c)));
	}
	{
		int c[] = {1};
		EXPECT_TRUE(collision_patches_nopreprocess(patches, array_to_set(1, c)));
	}
	{
		int c[] = {2};
		EXPECT_FALSE(collision_patches_nopreprocess(patches, array_to_set(1, c)));
	}
	{
		int c[] = {0,1};
		EXPECT_TRUE(collision_patches_nopreprocess(patches, array_to_set(2, c)));
	}
	{
		int c[] = {1,2};
		EXPECT_TRUE(collision_patches_nopreprocess(patches, array_to_set(2, c)));
	}

	if (false) 
	{
		PatchesViewer * v = new PatchesViewer(patches);
	}
}

TEST_F(CollisionTest, DISABLED_patches_nopreprocess_twopart) {
	vector<shared_ptr<Patch>> patches;	
	for (int i = 0; i < 3; ++i) {
		shared_ptr<Patch> pa_(new Patch(*pa));
		patches.push_back(pa_);
	}
	patches[1]->translate(cml::vector3(3.8,0,1.0));
	patches[2]->translate(cml::vector3(0,0,1.1));
	patches[2]->translate_time(10);

	{
		int c[] = {0};
		EXPECT_TRUE(collision_patches_nopreprocess_twopart(patches, array_to_set(1, c)));
	}
	{
		int c[] = {1};
		EXPECT_TRUE(collision_patches_nopreprocess_twopart(patches, array_to_set(1, c)));
	}
	{
		int c[] = {2};
		EXPECT_FALSE(collision_patches_nopreprocess_twopart(patches, array_to_set(1, c)));
	}
	{
		int c[] = {0,1};
		EXPECT_FALSE(collision_patches_nopreprocess_twopart(patches, array_to_set(2, c)));
	}
	{
		int c[] = {1,2};
		EXPECT_TRUE(collision_patches_nopreprocess_twopart(patches, array_to_set(2, c)));
	}

	if (false) 
	{
		PatchesViewer * v = new PatchesViewer(patches);
	}
}

TEST_F(CollisionTest, patches_obb) {
	vector<shared_ptr<Patch>> patches;	
	for (int i = 0; i < 3; ++i) {
		shared_ptr<Patch> pa_(new Patch(*pa));
		patches.push_back(pa_);
	}
	patches[1]->translate(cml::vector3(3.8,0,1.0));
	patches[2]->translate(cml::vector3(0,0,1.1));
	patches[2]->translate_time(10);

	{
		int c[] = {0};
		EXPECT_LE(0, collision_patches_obb(patches, array_to_set(1, c)));
	}
	{
		int c[] = {1};
		EXPECT_LE(0, collision_patches_obb(patches, array_to_set(1, c)));
	}
	{
		int c[] = {2};
		EXPECT_GT(0, collision_patches_obb(patches, array_to_set(1, c)));
	}
	{
		int c[] = {0,1};
		EXPECT_GT(0, collision_patches_obb(patches, array_to_set(2, c)));
	}
	{
		int c[] = {1,2};
		EXPECT_LE(0, collision_patches_obb(patches, array_to_set(2, c)));
	}

	if (false) 
	{
		PatchesViewer * v = new PatchesViewer(patches);
	}
}

TEST_F(CollisionTest, DISABLED_incremental_small) {
	srand(79797+3);
	Env env;
	vector<shared_ptr<Patch>> patches;

	int total_num = 7;
	for (int i = 0; i <= total_num; ++i) {
		shared_ptr<Patch> pa_(new Patch(*pa));
		env.rand_scatter(*pa_);
		patches.push_back(pa_);
		{
			set<int> check_set;
			check_set.insert(patches.size()-1);
			bool is_col_nopre = collision_patches_nopreprocess(patches, check_set);
			bool is_col_obb = collision_patches_obb(patches, check_set) > -1000;
			bool is_col_obb_nocull = collision_patches_obb_nocull(patches, check_set) > -1;
			EXPECT_TRUE(is_col_nopre == is_col_obb);
			EXPECT_TRUE(is_col_obb == is_col_obb_nocull);
		}
	}
	
	if (false) 
	{
		set_color(patches);
		PatchesViewer * v = new PatchesViewer(patches);
	}
}

TEST_F(CollisionTest, DISABLED_incremental) {
//TEST_F(CollisionTest, incremental) {
	srand(79797+3);
	Env env;

	vector<shared_ptr<Patch>> patches;

	int total_num = 400;

	for (int i = 0; i <= total_num; ++i) {
		shared_ptr<Patch> pa_(new Patch(*pa));
		env.rand_scatter(*pa_);
		patches.push_back(pa_);

		if (i >= total_num - 30)
		{
			set<int> check_set;
			check_set.insert(patches.size()-1);

			BEGIN_TIMER(no_pre);
			collision_patches_nopreprocess(patches, check_set);
			END_TIMER(no_pre);

			BEGIN_TIMER(obb);
			int col_time = collision_patches_obb(patches, check_set);
			END_TIMER(obb);

			BEGIN_TIMER(obb_nocull);
			int col_time_nocull = collision_patches_obb_nocull(patches, check_set);
			END_TIMER(obb_nocull);

			EXPECT_EQ(col_time, col_time_nocull);
			
			cout << endl;
		}
	}

	//if (false) 
	{
		set_color(patches);
		PatchesViewer * v = new PatchesViewer(patches);
	}
}
