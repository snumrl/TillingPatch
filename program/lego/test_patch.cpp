#include "StdAfx.h"
#include "motion_edit.h"
#include "patch.h"
#include "tiling.h"
#include "motions_viewer.h"
#include "patches_viewer.h"
#include "nonslipfoot.h"

class PatchTest : public testing::Test {
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
		pa->name = "box";

		patch_type = new map<string, Patch>();
		(*patch_type)["box"] = *pa;
	}
	static void TearDownTestCase() {
		delete pa;
		pa = NULL;
		delete patch_type;
		patch_type = NULL;
	}

	static Patch *pa;
	static map<string, Patch> *patch_type;
};

Patch * PatchTest::pa = NULL;
map<string, Patch> * PatchTest::patch_type = NULL;

TEST_F(PatchTest, init) {
	EXPECT_TRUE(cml::length(pa->motions[1][0].trans() - cml::vector3(1.5064-0.1, 1.07182, -0.0575708)) < 0.001);
}

TEST_F(PatchTest, translate) {
	Patch pa_(*pa);
	pa_.translate(cml::vector3(0,0,-1));
	
	EXPECT_TRUE(cml::length(pa_.motions[1][0].trans() - cml::vector3(1.5064-0.1, 1.07182, -0.0575708-1)) < 0.001);
}

TEST_F(PatchTest, only_boundary) {
	vector<shared_ptr<Patch>> patches;
	patches.push_back(shared_ptr<Patch>(new Patch(*pa, true)));
	patches[0]->translate(cml::vector3(0,0,-1));
	patches.push_back(shared_ptr<Patch>(new Patch(*pa)));
	
	EXPECT_EQ(2, patches[0]->motions[0].size());
	EXPECT_LT(2, patches[1]->motions[0].size());

	if (false) {
		PatchesViewer * v = new PatchesViewer(patches, Env());
	}
}

TEST_F(PatchTest, patch_edit) {
	Patch pa1(*pa);
	Patch pa2(*pa);
	Patch pa3(*pa);
	pa2.translate(cml::vector3(5.5,0,1));
	pa2.translate_time(137-13);
	pa3.rotate(cml::pi()*0.8);
	pa3.translate(cml::vector3(1.5,0,4+1));
	pa3.translate_time(145);

	vector<Constraint> cons;
	add_cons_pos(cons, 1, 137,  pa2.motions[0].posture(0).trans());
	add_cons_dir(cons, 1, 137-1,  (pa2.motions[0].posture(2).trans()-pa2.motions[0].posture(0).trans()));

	add_cons_pos(cons, 0, 145-13,  pa3.motions[1].posture(0).trans());
	add_cons_dir(cons, 0, 145-13-1,  (pa3.motions[1].posture(2).trans()-pa3.motions[1].posture(0).trans()));
	
	//add_cons_rel_pos(cons, pa1.motions, 0, 44-13, 1, 18);
	add_cons_rel_pos(cons, pa1.motions, 0, 65-13, 1, 65);
	multi_motion_edit(pa1.motions, cons);

	EXPECT_TRUE(cml::length(pa1.motions[1].posture(65).trans() - cml::vector3(1.91695, 1.08757, 1.73485)) < 0.001);
	
	if (false)
	{
		MotionsViewer * v = new MotionsViewer(4, &pa1.motions, &pa2.motions, &pa3.motions, &pa->motions);
		v->get_camera().distance += 3.;
		v->get_camera().centerX += 3.;
		v->get_camera().centerZ += 3.;
	}
}

TEST_F(PatchTest, scatter) {
	srand(79797+3);

	Env env;
	vector<ml::Motion> view_motions;
	for (int i = 0; i < 10; ++i)
	{
		Patch pa_(*pa);
		EXPECT_TRUE(env.rand_scatter(pa_));
		copy(pa_.motions.begin(), pa_.motions.end(), back_inserter(view_motions));
	}

	if (false)
	{
		MotionsViewer * v = new MotionsViewer(move(view_motions));
		v->get_camera().distance += 8.;
	}
}

TEST_F(PatchTest, get_end_num) {
	shared_ptr<Patch> pa_new(new Patch(*pa));
	pa_new->translate_time(50);
	auto end_num = get_end_num(pa_new, 100, 700);
	EXPECT_EQ(0, end_num["connect"]);
	EXPECT_EQ(2, end_num["dead"]);
	EXPECT_EQ(2, end_num["boundary"]);

	vector<shared_ptr<Patch>> patches;
	patches.push_back(pa_new);

	{
		shared_ptr<Patch> pa_new(new Patch(*pa));
		vector<ConnectInfo> connect_infos;
		connect_infos.push_back(ConnectInfo(0,0,1));
		stitch(patches, pa_new, patch_type, connect_infos);
	}
	
	auto end_num_ = get_end_num(patches, 100, 700);
	EXPECT_EQ(2, end_num_["connect"]);
	EXPECT_EQ(4, end_num_["dead"]);
	EXPECT_EQ(2, end_num_["boundary"]);
	
	if (false)
	{
		PatchesViewer * v = new PatchesViewer(patches);
	}
}

TEST_F(PatchTest, stitch_sametime) {
	vector<shared_ptr<Patch>> patches;
	patches.push_back(shared_ptr<Patch>(new Patch(*pa)));
	patches.push_back(shared_ptr<Patch>(new Patch(*pa)));

	patches[0]->translate(cml::vector3(5.5,0,1));
	patches[0]->translate_time(137-13+10);
	patches[1]->rotate(cml::pi()*0.8);
	patches[1]->translate(cml::vector3(1.5,0,4+1));
	patches[1]->translate_time(145);

	{
		shared_ptr<Patch> pa_new(new Patch(*pa));
		vector<ConnectInfo> connect_infos;
		connect_infos.push_back(ConnectInfo(1,0,2,2.37737));
		stitch(patches, pa_new, patch_type, connect_infos);
	}

	{
		shared_ptr<Patch> pa_new(new Patch(*pa));
		vector<ConnectInfo> connect_infos;
		connect_infos.push_back(ConnectInfo(3,1,0,2.37737));
		stitch(patches, pa_new, patch_type, connect_infos);
	}

	//스티칭후 비교를 위해 모션들 미리 저장
	vector<ml::Motion> view_motions;
	for (auto it = patches.begin(); it != patches.end(); ++it)
		for (auto it_m = (*it)->motions.begin(); it_m != (*it)->motions.end(); ++it_m)
			view_motions.push_back(move(*it_m));

	{
		shared_ptr<Patch> pa_new(new Patch(*pa));
		vector<ConnectInfo> connect_infos;
		connect_infos.push_back(ConnectInfo(3,0,0,2.37737));
		connect_infos.push_back(ConnectInfo(1,1,3,2.37737));
		stitch(patches, pa_new, patch_type, connect_infos);
	}

	EXPECT_TRUE(cml::length((*(patches.end()-1))->motions[0].first_posture().trans() - cml::vector3(1.32118, 1.04135, 3.52851)) < 0.25);
	
	if (false)
	{
		PatchesViewer * v = new PatchesViewer(patches);
		v->get_camera().distance += 7.;
	}	
}

TEST_F(PatchTest, stitch2) {
	vector<shared_ptr<Patch>> patches;
	patches.push_back(shared_ptr<Patch>(new Patch(*pa)));
	shared_ptr<Patch> pa_new(new Patch(*pa));
	pa_new->rotate(2.70177);
	pa_new->translate(cml::vector3(-3.36, 0, -1.1));
	pa_new->translate_time(149);

	vector<ConnectInfo> connect_infos;
	connect_infos.push_back(ConnectInfo(0,0,1));
	bool is_stitch = stitch(patches, pa_new, patch_type, connect_infos);
	
	EXPECT_TRUE(is_stitch == true);
	EXPECT_TRUE(patches.size() == 2);
	EXPECT_TRUE(cml::length(cml::vec2(patches[0]->motions[0].last_posture().trans() - patches[1]->motions[0].first_posture().trans())) < 0.01);
	EXPECT_TRUE(cml::length(cml::vec2(patches[0]->motions[0].first_posture().trans() - pa->motions[0].first_posture().trans())) < 0.01);
		
	if (false)
		PatchesViewer * v = new PatchesViewer(patches);
}

TEST_F(PatchTest, stitch2_cyclic) {
	vector<shared_ptr<Patch>> patches;
	patches.push_back(shared_ptr<Patch>(new Patch(*pa)));
	patches[0]->translate(cml::vector3(6,0,0));
	
	shared_ptr<Patch> pa_new(new Patch(*pa));
	pa_new->translate(cml::vector3(0.2, 0, 0));
	pa_new->translate_time(128);
	
	vector<ConnectInfo> connect_infos;
	get_connect_infos(connect_infos, pa_new, patches, 2.5, Env());
	/*for (auto it = connect_infos.begin(); it != connect_infos.end(); ++it)
		cout << it->b1 << ' ' << it->b2 << ' ' << it->pa2 << ' ' << it->distance << endl;*/
		
	bool is_stitch = stitch(patches, pa_new, patch_type, connect_infos, Env(), false);

	EXPECT_EQ(true, is_stitch);
	EXPECT_EQ(2, patches.size());
	
	if (false)
	{
		set_color(patches);
		PatchesViewer * v = new PatchesViewer(patches);
	}
}

TEST_F(PatchTest, stitch2_cyclic2) {
	vector<shared_ptr<Patch>> patches;
	patches.push_back(shared_ptr<Patch>(new Patch(*pa)));
	patches[0]->translate(cml::vector3(0.2,0,0));

	shared_ptr<Patch> pa_new(new Patch(*pa));
	pa_new->translate(cml::vector3(6, 0, 0));
	pa_new->translate_time(128);

	vector<ConnectInfo> connect_infos;
	get_connect_infos(connect_infos, pa_new, patches, 2.5, Env());
	/*for (auto it = connect_infos.begin(); it != connect_infos.end(); ++it)
		cout << it->b1 << ' ' << it->b2 << ' ' << it->pa2 << ' ' << it->distance << ' ' << it->trans_x << endl;*/

	bool is_stitch = stitch(patches, pa_new, patch_type, connect_infos, Env(), false);

	EXPECT_EQ(true, is_stitch);
	EXPECT_EQ(2, patches.size());

	if (false)
	{
		set_color(patches);
		PatchesViewer * v = new PatchesViewer(patches);
	}
}

TEST_F(PatchTest, stitch2_cyclic3) {
	vector<shared_ptr<Patch>> patches;
	patches.push_back(shared_ptr<Patch>(new Patch(*pa)));
	patches[0]->translate(cml::vector3(6,0,0));

	patches.push_back(shared_ptr<Patch>(new Patch(*pa)));
	patches[1]->translate(cml::vector3(6,0,0));
	patches[1]->translate_time(270);

	shared_ptr<Patch> pa_new(new Patch(*pa));
	pa_new->translate(cml::vector3(0.2, 0, 0));
	pa_new->translate_time(128);

	vector<ConnectInfo> connect_infos;
	get_connect_infos(connect_infos, pa_new, patches, 2.5, Env());
	//for (auto it = connect_infos.begin(); it != connect_infos.end(); ++it)
	//	cout << it->b1 << ' ' << it->b2 << ' ' << it->pa2 << ' ' << it->distance << ' ' << it->trans_x << endl;

	bool is_stitch = stitch(patches, pa_new, patch_type, connect_infos, Env(), false);
	EXPECT_EQ(true, is_stitch);
	EXPECT_EQ(3, patches.size());
	
	if (false)
	{
		set_color(patches);
		PatchesViewer * v = new PatchesViewer(patches);
	}
}

TEST_F(PatchTest, stitch3) {
	vector<shared_ptr<Patch>> patches;
	patches.push_back(shared_ptr<Patch>(new Patch(*pa)));
	{
		shared_ptr<Patch> pa_new(new Patch(*pa));
		pa_new->rotate(2.70177);
		pa_new->translate(cml::vector3(-3.36, 0, -1.1));
		pa_new->translate_time(149);

		vector<ConnectInfo> connect_infos;
		connect_infos.push_back(ConnectInfo(0,0,1));
		stitch(patches, pa_new, patch_type, connect_infos);
	}

	{
		shared_ptr<Patch> pa_new(new Patch(*pa));
		pa_new->rotate(5.84336);
		pa_new->translate(cml::vector3(2.62,0, 0.86));
		pa_new->translate_time(112);

		vector<ConnectInfo> connect_infos;
		connect_infos.push_back(ConnectInfo(0,0,3));
		stitch(patches, pa_new, patch_type, connect_infos);
	}

	EXPECT_TRUE(patches.size() == 3);
	EXPECT_TRUE(cml::length(cml::vec2(patches[0]->motions[0].last_posture().trans() - patches[1]->motions[0].first_posture().trans())) < 0.01);
	EXPECT_TRUE(cml::length(cml::vec2(patches[0]->motions[0].first_posture().trans() - pa->motions[0].first_posture().trans())) < 0.01);
	EXPECT_TRUE(cml::length(cml::vec2(patches[0]->motions[1].last_posture().trans() - patches[2]->motions[0].first_posture().trans())) < 0.01);

	if (false)
		PatchesViewer * v = new PatchesViewer(patches);
}

TEST_F(PatchTest, end_num) {
	vector<shared_ptr<Patch>> patches;
	shared_ptr<Patch> pa_new_(new Patch(*pa));
	pa_new_->translate_time(50);
	patches.push_back(pa_new_);

	{
		shared_ptr<Patch> pa_new(new Patch(*pa));
		vector<ConnectInfo> connect_infos;
		connect_infos.push_back(ConnectInfo(0,0,1));
		stitch(patches, pa_new, patch_type, connect_infos);
	}
	{
		shared_ptr<Patch> pa_new(new Patch(*pa));
		vector<ConnectInfo> connect_infos;
		connect_infos.push_back(ConnectInfo(0,0,3));
		stitch(patches, pa_new, patch_type, connect_infos);
	}

	EXPECT_TRUE(patches.size() == 3);
	EXPECT_TRUE(cml::length(cml::vec2(patches[0]->motions[0].last_posture().trans() - patches[1]->motions[0].first_posture().trans())) < 0.01);
	EXPECT_TRUE(cml::length(cml::vec2(patches[0]->motions[0].first_posture().trans() - pa->motions[0].first_posture().trans())) < 0.01);
	EXPECT_TRUE(cml::length(cml::vec2(patches[0]->motions[1].last_posture().trans() - patches[2]->motions[0].first_posture().trans())) < 0.01);

	map<string, int> end_num = get_end_num(patches, 100, 700);
	EXPECT_TRUE(end_num["connect"] == 4);
	EXPECT_TRUE(end_num["dead"] == 6);
	EXPECT_TRUE(end_num["boundary"] == 2);

	if (false)
		PatchesViewer * v = new PatchesViewer(patches);
}

TEST(flash) {
	ml::Motion m;
	m.LoadAMC_with_contactInfo("./data/1/test_motion.amc", "./data/wd2.asf", true, 0.027);

	Patch pa;
	pa.add_motion(m);
	
	vector<shared_ptr<Patch>> patches;
	patches.push_back(shared_ptr<Patch>(new Patch(pa)));

	ml::Motion &m_ = patches[0]->motions[0];
	
	vector<Constraint> cons;
	add_cons_pin(cons, 0, m_, false);
	add_cons_time(cons, 0, m_.size()-1, 30);

	multi_motion_edit(patches[0]->motions, cons);
	
	if (false)
		PatchesViewer * v = new PatchesViewer(patches);
}

TEST_F(PatchTest, DISABLED_write_read) {
	vector<shared_ptr<Patch>> patches_;
	patches_.push_back(shared_ptr<Patch>(new Patch(*pa)));
	string file_name = write_patches(patches_, "test");

	vector<shared_ptr<Patch>> patches;
	read_patches(patches, "test");

	EXPECT_EQ(138, patches[0]->motions[1].size());
	
	if (false)
		PatchesViewer * v = new PatchesViewer(patches);
}

TEST_F(PatchTest, DISABLED_write_read_connection) {
	vector<shared_ptr<Patch>> patches;

	{
		shared_ptr<Patch> pa1(new Patch(*pa));
		pa1->translate_time(400);
		patches.push_back(pa1);
	}

	{
		shared_ptr<Patch> pa1(new Patch(*pa));
		pa1->translate_time(400);

		vector<ConnectInfo> connect_infos;
		connect_infos.push_back(ConnectInfo(1,0,0));
		stitch(patches, pa1, patch_type, connect_infos);
	}

	string file_name = write_patches(patches, "test_connect");

	vector<shared_ptr<Patch>> patches_;
	read_patches(patches_, "test_connect");
	set_color(patches_);
		
	EXPECT_EQ(patches[0]->inner_cons.size(), patches_[0]->inner_cons.size());
	EXPECT_EQ(patches[0]->inner_cons[0].type, patches_[0]->inner_cons[0].type);
	EXPECT_EQ(patches_[0]->motions[0].color[0], patches_[1]->motions[0].color[0]);
	
	if (false)
		PatchesViewer * v = new PatchesViewer(patches_);
}

TEST_F(PatchTest, DISABLED_write_read_restart) {
	vector<shared_ptr<Patch>> patches;
	patches.push_back(shared_ptr<Patch>(new Patch(*pa)));
	patches[0]->translate_time(400);
	{
		shared_ptr<Patch> pa_new(new Patch(*pa));
	
		vector<ConnectInfo> connect_infos;
		connect_infos.push_back(ConnectInfo(0,0,1));
		stitch(patches, pa_new, patch_type, connect_infos);
	}

	string file_name = write_patches(patches, "test_restart");
	vector<shared_ptr<Patch>> patches_;
	read_patches(patches_, "test_restart");

	{
		shared_ptr<Patch> pa_new(new Patch(*pa));
		vector<ConnectInfo> connect_infos;
		connect_infos.push_back(ConnectInfo(0,0,3));
		stitch(patches, pa_new, patch_type, connect_infos, Env(), true);
	}

	{
		shared_ptr<Patch> pa_new(new Patch(*pa));
		vector<ConnectInfo> connect_infos;
		connect_infos.push_back(ConnectInfo(0,0,3));
		stitch(patches_, pa_new, patch_type, connect_infos, Env(), true);
	}
	
	EXPECT_TRUE(patches_.size() == 3);
	EXPECT_TRUE(cml::length(cml::vec2(patches[2]->motions[1].last_posture().trans() - patches_[2]->motions[1].last_posture().trans())) < 0.001);
	EXPECT_TRUE(cml::length(cml::vec2(patches[1]->motions[1].first_posture().trans() - patches_[1]->motions[1].first_posture().trans())) < 0.001);
	
	if (false) 
	{
		copy_patches(patches, patches_);
		PatchesViewer * v = new PatchesViewer(patches);
	}
}


TEST_F(PatchTest, DISABLED_retarget_patch)
{
	vector<shared_ptr<Patch>> patches;
	read_patches(patches, "41");

	vector<shared_ptr<Patch>> patches_ori;
	read_patches(patches_ori, patches[0]->name);

	for (auto it = patches_ori[0]->motions.begin(); it != patches_ori[0]->motions.end(); ++it) 
	{
		double dy = get_ground_value(*it);
		it->SetFootConstraint( 24, 19, 20, 4, 0.004, 0.05+dy, 0.0045, 0.055+dy );
	}

	for (int i = 0; i < patches_ori[0]->motions.size(); ++i)
	{
		for (int j = 0; j < patches_ori[0]->motions[i].size(); ++j)
		{
			const ml::Posture &p_ori = patches_ori[0]->motions[i][j];
			ml::Posture &p = patches[0]->motions[i][j];
			p.leftFootTouched = p_ori.leftFootTouched;
			p.rightFootTouched = p_ori.rightFootTouched;
		}
	}

	DontSlip noslipfoot(18, 19 , 24, 3, 4, 20);
	for (auto it = patches.begin(); it != patches.end(); ++it)
		noslipfoot.hold_foot(**it);

	//if (false)
	{
		PatchesViewer * v = new PatchesViewer(patches);
		v->get_camera().distance += 2;
	}
}

