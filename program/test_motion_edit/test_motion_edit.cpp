#include "StdAfx.h"
#include "motions_viewer.h"
#include "motion_edit.h"

class MotionEditTest : public testing::Test {
protected:  
	static void SetUpTestCase() {
		motions = new vector<ml::Motion>;

		ml::Motion base_motion;
		base_motion.LoadAMC_with_contactInfo("./data/3/pass_mid_to_mid.amc", "./data/wd2.asf", true, 0.027);

		ml::Motion m1, m2;
		m1.copy(base_motion, base_motion.begin()+452, base_motion.begin()+585);
		m1.translate(cml::vector3d(-2,0,-0.17));
		m1.translate_time_to_zero();
		m1.translate_time(13);

		m2.copy(base_motion, base_motion.begin()+624, base_motion.begin()+762);
		m2.translate(cml::vector3d(2.56+0.04,0,0.035));
		m2.translate_time_to_zero();
		
		motions->push_back(m1);
		motions->push_back(m2);
	}
	static void TearDownTestCase() {
		delete motions;
		motions = NULL;
	}
	static vector<ml::Motion> *motions;
};

vector<ml::Motion> * MotionEditTest::motions = NULL;

TEST_F(MotionEditTest, motion_setting) {
	EXPECT_TRUE(cml::length((*motions)[1][0].trans() - cml::vector3(1.5064, 1.07182, -0.0575708)) < 0.001);

	//MotionsViewer * t = new MotionsViewer(*motions);
}

TEST_F(MotionEditTest, position_constraints) {
	vector<ml::Motion> motions_(*motions);
	
	vector<Constraint> cons;
	add_cons_pos(cons, 0, 0,  motions_[0][0].trans());
	//add_cons(1, 1, motions_[1][1]);
			
	add_cons_pos(cons, 1, 0, motions_[1][0].trans());
	add_cons_pos(cons, 1, 1, motions_[1][1].trans());
	multi_motion_edit(motions_, cons);

	cml::vector3 pos_ori = (*motions)[1][80].trans();
	cml::vector3 pos_edited = motions_[1][80].trans();
	EXPECT_TRUE(cml::length(pos_ori - pos_edited) < 0.001);

	//MotionsViewer * t = new MotionsViewer(2, motions, &motions_);
}

TEST_F(MotionEditTest, time_constraints) {
	vector<ml::Motion> motions_(*motions);

	vector<Constraint> cons;
	add_cons_pos(cons, 0, 0,  motions_[0][0].trans());
	add_cons_pos(cons, 0, 1,  motions_[0][1].trans());
	add_cons_rel_pos(cons, motions_, 0, 65-13, 1, 65);

	add_cons_time(cons, 0, 0, motions_[0][0].time+10);
	add_cons_same_time(cons, 0, 65-13, 1, 65);
		
	multi_motion_edit(motions_, cons);

	cml::vector3 pos_ori = (*motions)[1][80].trans();
	cml::vector3 pos_edited = motions_[1][80].trans();
	EXPECT_TRUE(cml::length(pos_ori - pos_edited) < 0.001);

	EXPECT_TRUE(fabs(((*motions)[0][0].time + 10) - motions_[0][0].time) < 0.001);
	EXPECT_TRUE(fabs(((*motions)[1][0].time + 10) - motions_[1][0].time) < 0.001);

	//MotionsViewer * t = new MotionsViewer(2, motions, &motions_);
}

TEST_F(MotionEditTest, relative_position_constraints) {
	vector<ml::Motion> motions_(*motions);
	
	vector<Constraint> cons;
	add_cons_pos(cons, 0, 0,  motions_[0][0].trans());
	add_cons_pos(cons, 0, 1,  motions_[0][1].trans());
	
	//add_cons_rel_pos(motions_, 0, 44-13, 1, 18);
	
	add_cons_rel_pos(cons, motions_, 0, 65-13, 1, 65);
	multi_motion_edit(motions_, cons);

	cml::vector3 pos_ori = (*motions)[1][80].trans();
	cml::vector3 pos_edited = motions_[1][80].trans();
	EXPECT_TRUE(cml::length(pos_ori - pos_edited) < 0.001);

	//MotionsViewer * t = new MotionsViewer(2, motions, &motions_);
}

TEST_F(MotionEditTest, direction_constraints) {
	vector<ml::Motion> motions_(*motions);
	
	vector<Constraint> cons;
	add_cons_pos(cons, 0, 0,  motions_[0][0].trans());
	add_cons_dir(cons, 0, 0,  cml::roty_mat(cml::pi() * 0.0) * (motions_[0][1].trans()-motions_[0][0].trans()));

	add_cons_rel_pos(cons, motions_, 0, 65-13, 1, 65);
	multi_motion_edit(motions_, cons);

	cml::vector3 pos_ori = (*motions)[1][80].trans();
	cml::vector3 pos_edited = motions_[1][80].trans();
	EXPECT_TRUE(cml::length(pos_ori - pos_edited) < 0.001);

	//MotionsViewer * t = new MotionsViewer(2, motions, &motions_);
}

TEST_F(MotionEditTest, posture_direction) {
	vector<ml::Motion> motions_(*motions);

	vector<Constraint> cons;
	add_cons_pos(cons, 0, 0,  motions_[0][0].trans());
	add_cons_dir(cons, 0, 0,  cml::roty_mat(cml::pi() * 0.25) * (motions_[0][1].trans()-motions_[0][0].trans()));
	add_cons_rel_pos(cons, motions_, 0, 65-13, 1, 65);
	multi_motion_edit(motions_, cons);

	double dir_ori = cml::length(cml::log_mat3(cml::roty_mat(cml::pi() * 0.25) * (*motions)[1][105].rotate(0)));
	double dir_edited= cml::length(cml::log_mat3(motions_[1][105].rotate(0)));
	EXPECT_TRUE(abs(dir_ori-dir_edited) < 0.001);

	//MotionsViewer * t = new MotionsViewer(2, motions, &motions_);
}


