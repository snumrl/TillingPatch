#include "StdAfx.h"
#include "motions_viewer.h"
#include "motion_edit.h"

TEST(Motion, DISABLED_view)
{
	vector<ml::Motion> motions;
	ml::read_motions(motions, "small");
	//ml::read_motions(motions, "large");

	MotionsViewer * m = new MotionsViewer(motions);
	m->get_camera().distance += 10;
}

class MotionTest : public testing::Test {
protected:  
	virtual void SetUp() {
		base_m.LoadAMC_with_contactInfo("./data/1/test_motion.amc", "./data/wd2.asf", true, 0.027);
	}
	ml::Motion base_m;
};

TEST_F(MotionTest, Load) {
	EXPECT_EQ(10, base_m.size());
}

TEST_F(MotionTest, Constructor) {
	ml::Motion m(base_m);
	EXPECT_EQ(10, m.size());
}

TEST_F(MotionTest, iter) {
	for (auto it = base_m.begin(); it != base_m.end(); ++it)
		EXPECT_EQ(it->num_joint() , base_m[0].num_joint());
}

TEST_F(MotionTest, iter_construct) {
	ml::Motion m(base_m, base_m.begin(), base_m.begin()+5);
	EXPECT_EQ(m.size(), 5);
	for (auto it = m.begin(); it != m.end(); ++it)
		EXPECT_EQ(it->num_joint() , 25);
}

TEST_F(MotionTest, copy_operator) {
	ml::Motion m;
	m = base_m;
	EXPECT_EQ(10, m.size());
}

TEST_F(MotionTest, copy_operator2) {
	ml::Motion m;
	m.copy(base_m, base_m.begin(), base_m.begin()+5);
	EXPECT_EQ(5, m.size());
}

TEST_F(MotionTest, ik_limb) {
	vector<ml::Motion> ms;
	ml::Motion m;
	m.copy(base_m, base_m.begin(), base_m.begin()+1);
	ms.push_back(m);

	ml::Posture &p = m[0];
	cml::vector3 pos = p.GetGlobalTranslation(3);
	
	p.IkLimb(3, pos+ cml::vector3(0.3, 0, 0));
	EXPECT_TRUE(cml::length(p.GetGlobalTranslation(3) - (pos+ cml::vector3(0.3, 0, 0))) < 0.001);

	ms.push_back(m);
	//MotionsViewer * t = new MotionsViewer(ms);
}

TEST_F(MotionTest, DISABLED_get_angle)
{
  size_t pos = 1;
  vector<double> y_vec;
  int endjoint[5] = {23, 22, 20, 24, 21};
  for (size_t j = 0; j < 5; ++j) {
	int prev = endjoint[j];
	int curr = base_m.body()->parent(prev);
	double coeff = 0.8;
	while (curr != 0 && curr != 8 && curr != 12) {
	  cml::vector3 v1 = base_m[pos].GetGlobalTranslation(prev) - base_m[pos].GetGlobalTranslation(curr);
	  v1.normalize();
	  prev = curr;
	  curr = base_m.body()->parent(prev);
	  cml::vector3 v2 = base_m[pos].GetGlobalTranslation(prev) - base_m[pos].GetGlobalTranslation(curr);
	  v2.normalize();
	  y_vec.push_back(coeff * between_vector(v1, v2).length());
	  coeff *= 1.08;
	}
  }

  for (size_t i=0; i<y_vec.size(); ++i) {
	cout << y_vec[i] <<" ";
  } cout << endl;
}

TEST(Motion, DISABLED_Multiview)
{
  vector<ml::Motion> ms;
  ml::Motion m, m1;

  //m.LoadAMC_with_contactInfo("./data/multi/hys_hello.amc", "./data/multi/hys.asf", true, 0.027); 
  m.LoadAMC_with_contactInfo("./data/multi/hys_march_hand_2_x2d.amc", "./data/multi/hys.asf", true, 0.027); 
  ms.push_back(m);

  //m1.LoadAMC_with_contactInfo("./data/multi/mmk_synch.amc", "./data/multi/mmk.asf", true, 0.027);  
  //ms.push_back(m1);  

  MotionsViewer * t = new MotionsViewer(ms);
}


