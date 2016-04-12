#include "StdAfx.h"
#include "get_patch.h"
#include "nonslipfoot.h"

map<string, ml::Motion> motions_type;

void auto_get_motion( ml::Motion &m, const char * fullname, int first, int last, int first_type = 1, int last_type = 1, double ground = 0.0);
void set_posture_type( ml::Motion &m, int first, int last, int type);
void add_bent( map<string, Patch> * patch_type, const string &name, double angle );
//void add_patch(map<string, Patch> * patch_type , const char * fullname, int first, int first_type, int last, int last_type, int object_first = -1, int object_last = -1, double ground_truth = 0.0);
void add_patch_box(map<string, Patch> * patch_type , const char * fullname, int first, int first_type, int last, int last_type);

void make_patch_jump(Patch &pa);
void make_patch_push(Patch &pa);
void make_patch_beat(Patch &pa);
void make_patch_beat_4(Patch &pa);
void make_patch_bow(Patch &pa);
void make_patch_bow_10(Patch &pa);
void make_patch_bow_4(Patch &pa);
void make_patch_shake_hands(Patch &pa);
void make_patch_step_on(Patch &pa);
void make_patch_assist_man(Patch &pa);
void make_patch_poke_3(Patch &pa);
void make_patch_highfive_6(Patch &pa);
void make_patch_ballet_3(Patch &pa);
void make_patch_accicdent(Patch &pa);
void make_patch_wheel_poke_1(Patch &pa);
void make_patch_wheel_poke_2(Patch &pa);
void make_patch_wheel_2_oneway(Patch &pa);
void make_patch_highfive(Patch &pa);
void make_patch_highfive_2(Patch &pa);

void make_patch_chicken(vector< shared_ptr<Patch> > &);



void get_patch(map<string, Patch> *patch_type)
{
  {
	Patch pa;
	make_patch_jump(pa);
	(*patch_type)[pa.name] = move(pa);
  } {
	Patch pa;
	make_patch_push(pa);
	(*patch_type)[pa.name] = move(pa);
  } {
	Patch pa;
	make_patch_beat(pa);
	(*patch_type)[pa.name] = move(pa);
  } {
	Patch pa;
	make_patch_highfive(pa);
	(*patch_type)[pa.name] = move(pa);
  } {
	Patch pa;
	make_patch_bow(pa);
	(*patch_type)[pa.name] = move(pa);
  } {
	Patch pa;
	make_patch_shake_hands(pa);
	(*patch_type)[pa.name] = move(pa);
  } {
	Patch pa;
	make_patch_ballet_3(pa);
	(*patch_type)[pa.name] = move(pa);
  } {
	Patch pa;
	make_patch_kick(pa);
	(*patch_type)[pa.name] = move(pa);
  } {
	Patch pa;
	make_patch_bow_4(pa);	
	(*patch_type)[pa.name] = move(pa);
  } {
	Patch pa;
	make_patch_step_on(pa);
	(*patch_type)[pa.name] = move(pa);
  } {
	Patch pa;
	make_patch_poke_3(pa);
	(*patch_type)[pa.name] = move(pa);
  } {
	Patch pa;
	make_patch_accicdent(pa);
	(*patch_type)[pa.name] = move(pa);
  }{
	Patch pa;
	make_patch_wheel_poke_1(pa);
	(*patch_type)[pa.name] = move(pa);
  } {
	Patch pa;
	make_patch_wheel_poke_2(pa);
	(*patch_type)[pa.name] = move(pa);
  } {
	Patch pa;
	make_patch_wheel_2_oneway(pa);
	(*patch_type)[pa.name] = move(pa);
  } {
	Patch pa;
	make_patch_beat_4(pa);
	(*patch_type)[pa.name] = move(pa);
  } {
	Patch pa;
	make_patch_highfive_2(pa);
	(*patch_type)[pa.name] = move(pa);
  } /*{
	Patch pa;
	make_patch_bow_10(pa);
	(*patch_type)[pa.name] = move(pa);
	} {
	Patch pa;
	make_patch_assist_man(pa);
	(*patch_type)[pa.name] = move(pa);
	}*/

  //make_patch_highfive_6(patch_type);	
  add_patch(patch_type, "3/avoid", 129, 1, 180, 1); //앉음
  add_patch(patch_type, "3/avoid", 108, 2, 180, 1); 
  add_patch(patch_type, "1/walk_clockwise", 96, 1, 115, 2, -1, -1, 0.03);
  add_patch(patch_type, "1/walk_clockwise", 115, 2, 131, 1, -1, -1, 0.03);
  add_patch(patch_type, "1/walk_clockwise", 96, 1, 131, 1, -1, -1, 0.03);
  add_patch(patch_type, "1/walk_clockwise", 80, 2, 115, 2, -1, -1, 0.03);
  add_patch(patch_type, "1/walk_counterclockwise", 88, 2, 106, 1, -1, -1, 0.03);
  add_patch(patch_type, "1/walk_counterclockwise", 106, 1, 124, 2, -1, -1, 0.03);
  add_patch(patch_type, "1/walk_counterclockwise", 88, 2, 124, 2, -1, -1, 0.03);
  add_patch(patch_type, "1/walk_counterclockwise", 77, 1, 106, 1, -1, -1, 0.03);
  add_patch(patch_type, "1/walk_left_45degree", 67, 2, 84, 1, -1, -1, 0.03);
  add_patch(patch_type, "1/walk_left_45degree_1", 85, 1, 102, 2, -1, -1, 0.03);
  add_patch(patch_type, "1/walk_right_45degree", 62, 2, 77, 1, -1, -1, 0.03); 
  add_patch(patch_type, "3/shot", 523, 2, 653, 1);
  add_patch(patch_type, "3/walk", 573, 2, 610, 2, -1, -1, 0.01);
  add_patch(patch_type, "3/walk", 591, 1, 629, 1, -1, -1, 0.01);
  add_patch(patch_type, "3/walk_side", 279, 2, 351, 1, -1, -1, 0.01);
  add_patch(patch_type, "3/walk_side", 479, 1, 548, 1, -1, -1, 0.01);
  add_patch(patch_type, "3/walk_side", 93, 1, 137, 2, -1, -1, 0.01);
  add_patch(patch_type, "3/walk_stop_walk", 146, 1, 234, 1, -1, -1, 0.01);
  add_patch(patch_type, "3/walk_stop_walk", 382, 2, 462, 1, -1, -1, 0.01);
  add_patch(patch_type, "3/walk_stop_walk", 700, 2, 760, 1, -1, -1, 0.01);
  //add_patch(patch_type, "3/avoid", 622, 2, 702, 2, -1, -1, 0.019);	// turn right ?????? 
  add_patch(patch_type, "3/get_hit", 236, 2, 299, 1, -1, -1, 0.018); // turn left
  add_patch(patch_type, "3/get_hit", 651, 1, 707, 2, -1, -1, 0.018); // turn left
  add_patch(patch_type, "3/pass_mid_to_mid", 413, 1, 452, 1);
  add_patch(patch_type, "3/pass_mid_to_mid", 413, 1, 433, 2);
  add_patch(patch_type, "3/pass_mid_to_mid", 433, 2, 452, 1);

  //add_patch(patch_type, "3/get_hit", 1018, 1, 1106, 1);
  //add_patch(patch_type, "3/jump", 102, 1, 205, 1);
  //add_patch(patch_type, "3/jump", 292, 2, 393, 1);
  //add_patch(patch_type, "3/pass_duck", 1021, 2, 1081, 1);
  //add_patch(patch_type, "3/pass_duck", 1021, 2, 1103, 2);
  //add_patch(patch_type, "3/pushed", 529, 1, 599, 2);
  //add_patch(patch_type, "3/pushed", 529, 1, 578, 1);
  //add_patch(patch_type, "3/pushed", 929, 2, 1009, 1);
  //add_patch(patch_type, "3/pushed", 929, 2, 992, 2);
  //add_patch(patch_type, "3/pushed", 1241, 2, 1260, 1);
  //add_patch(patch_type, "3/pushed", 1260, 1, 1279, 2);  
  //add_patch(patch_type, "1/flying_kick", 57, 1, 134, 1);
  ////add_patch(patch_type, "1/flying_kick_1", 45, 2, 123, 2);
  //add_patch(patch_type, "1/jump", 61, 1, 95, 1);
  //add_patch(patch_type, "1/jump_high_2", 49, 1, 110, 1);	
  //add_patch(patch_type, "1/move_cup_1", 635, 2, 696, 1);

}


void get_patch_object(map<string, Patch> *patch_type)
{
  //box관련 모션 
  {
	ml::Motion m1 = auto_motion("3/pass_mid_to_mid", 469, 549, 2, 5, 502+5, 549);
	m1.translate(cml::vector3d(-2-0.1,0,-0.17));
	m1.translate_time_to_zero();
	m1.translate_time(13+7+10);
	ml::Motion m2 = auto_motion("3/pass_mid_to_mid", 624, 744-19, 5, 1, 624, 624+63-5);
	m2.translate(cml::vector3d(2.56+0.04-0.1 + 0.01008742,0,0.035-0.018401295));
	m2.translate_time_to_zero();

	for(int i = 63 - 20; i <= 63; ++i) {
	  double ratio = 1. - (63 - i) / 20.;
	  m2[i].object_height_correction = -0.060955 * ratio;
	}
	for(int i = 33; i <= 33 + 20; ++i) {
	  double ratio = 1. - (i - 33) / 20.;
	  m1[i].object_height_correction = 0.060955 * ratio;
	}

	Patch pa;
	pa.add_motion(m1);
	pa.add_motion(m2);
	pa.translate_origin();
	pa.set_boundaries();

	pa.patch_object.objectType = PatchObject::kBoxBlending;
	pa.patch_object.range = 5;
	pa.patch_object.begin_motion = 1;
	pa.patch_object.begin_frame = 63;
	pa.patch_object.end_motion = 0;
	pa.patch_object.end_frame = 33;

	add_cons_rel_pos(pa.inner_cons, pa.motions, 0, 63-30, 1, 63);
	add_cons_same_time(pa.inner_cons, 0, 63-30, 1, 63);
	pa.name = "box";
	(*patch_type)[pa.name] = move(pa);
  }

  //
  {
	ml::Motion m1 = auto_motion("3/pass_ground_to_low", 244, 319, 1, 6, 282, 319);
	ml::Posture &p1 = m1.posture(38);
	m1.IkLimbSmoothly(38, 8, 8, 11, p1.GetGlobalTranslation(11) + cml::vector3d(0, 0.1, 0));
	m1.IkLimbSmoothly(38, 8, 8, 15, p1.GetGlobalTranslation(15) + cml::vector3d(0, 0.1, 0));
	m1.translate_time(-60);
	m1.rotate(-1.57);
	m1.translate(cml::vector3d(-2, 0, 2));

	ml::Motion m2 = auto_motion("3/pass_mid_to_mid", 55, 148, 1, 5, 91, 148);
	ml::Posture &p2 = m2.posture(36);
	m2.IkLimbSmoothly(36, 15, 20, 11, p2.GetGlobalTranslation(11) + cml::vector3d(0, -0.3, 0));
	m2.IkLimbSmoothly(36, 15, 20, 15, p2.GetGlobalTranslation(15) + cml::vector3d(0, -0.3, 0));
	m2.translate_time(80);
	m2.rotate(3.14);
	m2.translate(cml::vector3d(-4.2, 0, 0));

	ml::Motion m3 = auto_motion("3/pass_mid_to_mid", 55, 148, 1, 5, 91, 148);
	ml::Posture &p3 = m3.posture(36);
	m3.IkLimbSmoothly(36, 10, 12, 11, p3.GetGlobalTranslation(11) + cml::vector3d(0, 0.2, 0));
	m3.IkLimbSmoothly(36, 10, 12, 15, p3.GetGlobalTranslation(15) + cml::vector3d(0, 0.2, 0));
	m3.translate_time(40);
	m3.rotate(1.57);
	m3.translate(cml::vector3d(-2, 0, -2.1));

	ml::Motion m4 = auto_motion("3/pass_mid_to_mid", 55, 148, 1, 5, 91, 148);
	ml::Posture &p4 = m4.posture(36);
	m4.IkLimbSmoothly(36, 20, 20, 11, p4.GetGlobalTranslation(11) + cml::vector3d(0, 0.7, 0));
	m4.IkLimbSmoothly(36, 20, 20, 15, p4.GetGlobalTranslation(15) + cml::vector3d(0, 0.7, 0));
	m4.translate(cml::vector3d(0, 0, 0.2));
	
	Patch pa;
	pa.add_motion(m1);
	pa.add_motion(m2);
	pa.add_motion(m3);
	pa.add_motion(m4);
	pa.translate_origin();
	pa.set_boundaries();

	add_cons_rel_pos_same_time(pa.inner_cons, pa.motions, 0, 40, 1, 89);
	add_cons_rel_pos_same_time(pa.inner_cons, pa.motions, 1, 26, 2, 66);
	add_cons_rel_pos_same_time(pa.inner_cons, pa.motions, 2, 31, 3, 71);

	pa.name = "box_from";
	(*patch_type)[pa.name] = move(pa);
  }

  {
	ml::Motion m1 = auto_motion("1/chair", 326-247, 496-247, 2, 2);

	ml::Motion m2 = auto_motion("3/pass_mid_to_low", 493, 605, 1, 5, 534, 605);
	ml::Motion m3 = auto_motion("3/pass_mid_to_low", 269, 400, 5, 1, 269, 329);

	for(int i = 329 - 20; i <= 329; ++i) {
		double ratio = 1. - (329 - i) / 20.;
		m3[i -269].object_height_correction = -0.03029897 * ratio;
	}
	for(int i = 534; i <= 534 + 20; ++i) {
		double ratio = 1. - (i - 534) / 20.;
		m2[i - 493].object_height_correction = 0.03029897 * ratio;
	}

	m1.rotate(-1.55);
	m1.translate(cml::vector3(2.65548, 0, -1.86856));
	m1.translate_time_to_zero();
	m1.translate_time(41);

	//ml::Posture &p2 = m2.posture(534-473);
	//m2.IkLimbSmoothly(534-473, 10, 20, 11, p2.GetGlobalTranslation(11) + cml::vector3d(0, -0.2, 0));
	//m2.IkLimbSmoothly(534-473, 10, 20, 15, p2.GetGlobalTranslation(15) + cml::vector3d(0, -0.2, 0));
	m2.rotate(-3.05);
	m2.translate(cml::vector3(4.68860048, 0, -0.3400243));
	m2.translate_time_to_zero();
	m2.translate_time(160);

	//ml::Posture &p3 = m3.posture(400-269);
	//m3.IkLimbSmoothly(400-269, 20, 0, 11, p3.GetGlobalTranslation(11) + cml::vector3d(0, -0.2, 0));
	//m3.IkLimbSmoothly(400-269, 20, 0, 15, p3.GetGlobalTranslation(15) + cml::vector3d(0, -0.2, 0));

	m3.translate(cml::vector3(0.602292, 0, -0.171791));
	m3.translate_time_to_zero();

	Patch pa;
	pa.add_motion(m1);
	pa.add_motion(m2);
	pa.add_motion(m3);
	pa.translate_origin();
	pa.set_boundaries();
		
	add_cons_rel_pos_same_time(pa.inner_cons, pa.motions, 2, 60, 0, 19);
	add_cons_rel_pos_same_time(pa.inner_cons, pa.motions, 1, 30, 0, 155);

	pa.patch_object.objectType = PatchObject::kPatchBox;
	pa.patch_object.begin_motion = 2;
	pa.patch_object.begin_frame = 50;
	pa.patch_object.end_motion = 1;
	pa.patch_object.end_frame = 35;
		
	pa.name = "chair";
	(*patch_type)[pa.name] = move(pa);		
  }
  add_patch_box(patch_type, "3/pass_duck", 239, 5, 264, 6);
  add_patch_box(patch_type, "3/pass_duck", 264, 6, 290, 5);
  add_patch_box(patch_type, "3/pass_duck", 239, 5, 290, 5);
  add_patch_box(patch_type, "3/pass_mid_to_mid", 167, 6, 207, 6);
  add_patch_box(patch_type, "3/pass_mid_to_high_1", 671, 5, 695, 6);
  add_patch_box(patch_type, "3/pass_mid_to_high_1", 671, 5, 714, 5);
  add_patch_box(patch_type, "3/pass_ground_to_low", 1039, 6, 1059, 5); //고개
  add_patch_box(patch_type, "3/pass_ground_to_low", 1039, 6, 1077, 6); //고개
  add_patch_box(patch_type, "3/pass_ground_to_high_1", 183, 6, 203, 5); 
  add_patch_box(patch_type, "3/pass_ground_to_high_1", 183, 6, 221, 6); 
  add_patch_box(patch_type, "3/pass_ground_to_high_1", 572, 5, 592, 6); 
  add_patch_box(patch_type, "3/pass_ground_to_high_1", 572, 5, 609, 5); 
}

void get_patch_unary( vector<Patch> *patch_type_unary, map<string, Patch> * patch_type )
{
  for (auto it = patch_type->begin(); it != patch_type->end(); ++it) {
	const Patch &pa = it->second;
	if (pa.motions.size() == 1) { // && it->first == "pass_mid_to_mid_413_452") {
	  patch_type_unary->push_back(pa);
	}
  }
}

void add_bent( map<string, Patch> * patch_type, const string &name, double angle )
{
  std::ostringstream strs;
  strs << angle;
  std::string str = strs.str();
  (*patch_type)[name + str] = (*patch_type)[name];
  {
	ml::Motion &m = (*patch_type)[name+str].motions[0];
	vector<ml::Motion *> motions;
	motions.push_back(&m);
	vector<Constraint> cons;
	add_cons_pin(cons, 0, m, false);
	add_cons_dir(cons, 0, m.size()-1, cml::roty_mat(cml::pi() * angle) * (m[m.size()-1].trans()-m[m.size()-2].trans()));
	multi_motion_edit(motions, cons);
  }
}

void auto_get_motion( ml::Motion &m, const char * fullname, int first, int last, int first_type /*= 1*/, int last_type /*= 1*/, double ground )
{
  string fullname_(fullname);
  if (motions_type.find(fullname_) == motions_type.end()) { 
	motions_type[fullname_] = ml::Motion();
	motions_type[fullname_].LoadAMC_with_contactInfo(("./data/"+ fullname_ + ".amc").c_str(), "./data/wd2.asf", true, 0.027, ground);
  }

  m.get_motion(motions_type[fullname_], first, last);
  m.first_posture().type_ = first_type;
  m.last_posture().type_ = last_type;
}

ml::Motion auto_motion( const char * fullname, int first, int last, int first_type, int last_type, int object_first, int object_last, double ground )
{
  ml::Motion m;
  string fullname_(fullname);
  if (motions_type.find(fullname_) == motions_type.end()) { 
	motions_type[fullname_] = ml::Motion();
	motions_type[fullname_].LoadAMC_with_contactInfo(("./data/"+ fullname_ + ".amc").c_str(), "./data/wd2.asf", true, 0.027, ground);
  }

  m.get_motion(motions_type[fullname_], first, last);
  m.first_posture().type_ = first_type;
  m.last_posture().type_ = last_type;

  if (object_first != -1 ) {
	for (int i = object_first; i <= object_last; ++i)
	  m[i-first].object = 1;
  }

  return m;
}

void set_posture_type( ml::Motion &m, int first, int last, int type)
{
  for (int i = first; i <= last; ++i) {
    m[i].object = type;
  }
}

void add_patch( map<string, Patch> * patch_type , const char * fullname, int first, int first_type, int last, int last_type, int object_first /*= -1*/, int object_last /*= -1*/, double ground_truth /*= 0.0*/ )
{
  ml::Motion m = auto_motion(fullname, first, last, first_type, last_type, object_first, object_last, ground_truth);

  Patch pa;
  pa.add_motion(m);
  pa.translate_origin();
  pa.set_boundaries();

  stringstream first_s;
  first_s << first;
  stringstream last_s;
  last_s << last;

  string fullname_(fullname);
  string filename = fullname_.substr(fullname_.find_last_of("/")+1);
  pa.name = filename + '_'+ first_s.str() + '_' + last_s.str();
  (*patch_type)[pa.name] = move(pa);
}

void add_patch_box( map<string, Patch> * patch_type , const char * fullname, int first, int first_type, int last, int last_type)
{
	add_patch(patch_type, fullname, first, first_type, last, last_type, first, last);
}

shared_ptr<Patch> get_a_patch( const char* name, const map<string,Patch> &patch_type )
{
	return shared_ptr<Patch>( new Patch((patch_type.find(name))->second));
}

shared_ptr<Patch> get_a_patch( const char* name )
{
  Patch pa;
  if (strcmp(name, "jump") == 0) {
	make_patch_jump(pa);
  } else if (strcmp(name, "bow") == 0) {
	make_patch_bow(pa);
  } else if (strcmp(name, "highfive") == 0) {
	make_patch_highfive(pa);
  } else if (strcmp(name, "accident") == 0) {
	make_patch_accicdent(pa);
  } else if (strcmp(name, "shake_hands") == 0) {
	make_patch_shake_hands(pa);
  } else if (strcmp(name, "beat") == 0) {
	make_patch_beat(pa);
  } else if (strcmp(name, "beat4") == 0) {
	make_patch_beat_4(pa);
  } else if (strcmp(name, "ballet") == 0) {
	make_patch_ballet_3(pa);
  } else if (strcmp(name, "kick") == 0) {
	make_patch_kick(pa);
  } else if (strcmp(name, "push") == 0) {
	make_patch_push(pa);
  } else { pa.name = "none"; }

  return shared_ptr<Patch>(new Patch(pa));
}

void make_patch_jump( Patch &patch )
{	
  patch.name = "jump";

  ml::Motion m21 = auto_motion("3/jump", 270+22, 393, 2, 1);
  m21.translate_time_to_zero();
  m21.translate(cml::vector3(-0.1, 0, 0));
  m21.translate_time(22);

  ml::Motion m22 = auto_motion("2/itonic3_wd2_walktoduck", 27, 86, 1, 1);
  ml::Motion m23 = auto_motion("2/itonic3_wd2_ducktowalk", 26, 100, 1, 2);
  m22.stitch(m23);
  m22.translate_time_to_zero();
  m22.rotate(cml::pi() * 0.5);
  m22.translate(cml::vector3(0,0,0.73));

  patch.add_motion(m21);
  patch.add_motion(m22);			
  patch.translate_origin();
  patch.set_boundaries();

  add_cons_rel_pos_same_time(patch.inner_cons, patch.motions, 0, 46, 1, 68);	
}

void make_patch_push( Patch & pa )
{
  pa.name = "push";	

  ml::Motion m21 = auto_motion("3/pushed", 803-75, 910-75-41, 1, 1);
  m21.translate_time_to_zero();
  m21.rotate(1.58319);
  m21.translate(cml::vector3(-1.61513, 0, 1.01225));
  m21.translate_time(33-38);

  ml::Motion m22 = auto_motion("3/push_person", 93, 160, 1, 3);
  m22.translate_time_to_zero();
  m22.translate(cml::vector3(-0.309585, 0, 1.91209));

  ml::Motion m22_ = auto_motion("3/push_person", 231, 270, 3, 1);
  m22.stitch(m22_);

  pa.add_motion(m22);
  pa.add_motion(m21);
  pa.translate_origin();
  pa.set_boundaries();

  add_cons_rel_pos_same_time(pa.inner_cons, pa.motions, 0, 28, 1, 24);
}

void make_patch_beat( Patch & pa )
{
  pa.name = "beat";

  ml::Motion m21 = auto_motion("3/beat", 84, 84+101, 1, 4, -1, -1, 0.02);
  m21.translate_time_to_zero();
  m21.rotate(3.14);
  m21.translate(cml::vector3(-4.28558, 0, 0.237186));
  ml::Motion m21_ = auto_motion("3/pass_jump", 1712, 1751, 4, 2, -1, -1, 0.02);
  m21.stitch(m21_);

  ml::Motion m22 = auto_motion("3/beat", 84, 84+101, 1, 4, -1, -1, 0.02);
  m22.translate_time_to_zero();
  ml::Motion m22_ = auto_motion("3/throw", 444, 503, 4, 1, -1, -1, 0.02);
  m22.stitch(m22_);

  pa.add_motion(m21);
  pa.add_motion(m22);
  pa.translate_origin();
  pa.set_boundaries();

  add_cons_rel_pos_same_time(pa.inner_cons, pa.motions, 0, 40, 1, 40);
  add_cons_rel_pos_same_time(pa.inner_cons, pa.motions, 0, 45, 1, 45);		
}

void make_patch_beat_4( Patch & pa )
{
  pa.name = "beat_4";

  ml::Motion m21 = auto_motion("3/beat", 84, 84+101, 1, 4, -1, -1, 0.02);
  m21.translate_time_to_zero();
  m21.rotate(3.14);
  m21.translate(cml::vector3(-4.28558, 0, 0.237186));
  ml::Motion m21_ = auto_motion("3/throw", 444, 503, 4, 1, -1, -1, 0.02);
  m21.stitch(m21_);

  ml::Motion m22 = auto_motion("3/beat", 84, 84+101, 1, 4, -1, -1, 0.02);
  m22.translate_time_to_zero();
  m22.rotate(1.5);
  m22.translate(cml::vector3(-2.09239, 0, -2.15418));
  ml::Motion m22_ = auto_motion("3/throw_1", 630, 679, 4, 2, -1, -1, 0.02);
  m22.stitch(m22_);

  ml::Motion m23 = auto_motion("3/beat", 84, 84+101, 1, 4, -1, -1, 0.02);
  m23.translate_time_to_zero();
  m23.rotate(-1.5);
  m23.translate(cml::vector3(-1.93742, 0, 2.24613));
  ml::Motion m23_ = auto_motion("3/throw", 444, 503, 4, 1, -1, -1, 0.02);
  m23.stitch(m23_);

  ml::Motion m24 = auto_motion("3/beat", 84, 84+101, 1, 4, -1, -1, 0.02);
  m24.translate_time_to_zero();
  ml::Motion m24_ = auto_motion("1/move_3", 238, 281, 4, 2, -1, -1, 0.02);
  m24.stitch(m24_);

  pa.add_motion(m21);
  pa.add_motion(m22);
  pa.add_motion(m23);
  pa.add_motion(m24);
  pa.translate_origin();
  pa.set_boundaries();

  add_cons_rel_pos_same_time(pa.inner_cons, pa.motions, 0, 42, 1, 42);
  add_cons_rel_pos_same_time(pa.inner_cons, pa.motions, 0, 42, 2, 42);
  add_cons_rel_pos_same_time(pa.inner_cons, pa.motions, 0, 42, 3, 42);
  add_cons_rel_pos_same_time(pa.inner_cons, pa.motions, 1, 42, 2, 42);
  add_cons_rel_pos_same_time(pa.inner_cons, pa.motions, 1, 42, 3, 42);
  add_cons_rel_pos_same_time(pa.inner_cons, pa.motions, 2, 42, 3, 42);	
}

void make_patch_bow( Patch & pa )
{
  pa.name = "bow";

  ml::Motion m21 = auto_motion("3/bow", 97+21, 230, 2, 1);
  m21.translate_time_to_zero();
  m21.rotate(3.1);
  m21.translate(cml::vector3(-1.59561, 0, -0.974999 + 0.3));

  ml::Motion m22 = auto_motion("3/bow", 97+21, 230, 2, 1);
  m22.translate_time_to_zero();

  pa.add_motion(m21);
  pa.add_motion(m22);
  pa.translate_origin();
  pa.set_boundaries();

  add_cons_rel_pos_same_time(pa.inner_cons, pa.motions, 0, 53, 1, 53);
}

void make_patch_bow_10( Patch & pa )
{
  pa.name = "bow_10";

  ml::Motion m21 = auto_motion("3/bow", 97+21, 230, 2, 1);
  m21.rotate(3.1);
  m21.translate(cml::vector3(-1.59561, 0, -0.974999 + 0.3));
  ml::Motion m22 = auto_motion("3/bow", 97+21, 230, 2, 1);

  for (int i = 0; i < 5; ++i) {
	ml::Motion m21_ = m21;
	ml::Motion m22_ = m22;
	m21_.translate(cml::vector3(i*1.5, 0, 0));
	m22_.translate(cml::vector3(i*1.5, 0, 0));
	pa.add_motion(m21_);
	pa.add_motion(m22_);
  }
  pa.translate_origin();
  pa.set_boundaries();

  for (int i = 0; i < 5; ++i) {
	add_cons_rel_pos_same_time(pa.inner_cons, pa.motions, 2*i, 53, 2*i+1, 53);
  }
  for (int i = 0; i < 4; ++i) {
	add_cons_rel_pos_same_time(pa.inner_cons, pa.motions, 2*i, 55, 2*(i+1)+1, 55);
  }	
}

void make_patch_bow_4( Patch & pa )
{
  pa.name = "bow_4";

  ml::Motion m21 = auto_motion("3/bow", 97+21, 230, 2, 1);
  m21.rotate(3.1415-0.05);
  m21.translate(cml::vector3(-1.59561+0.03, 0, -0.974999 + 0.3-0.05));

  ml::Motion m22 = auto_motion("3/bow", 97+21, 230, 2, 1);
  m22.rotate(-0.05);
  m22.translate(cml::vector3(-0.01,0,0.04));

  ml::Motion m23 = auto_motion("3/bow", 97+21, 230, 2, 1);
  m23.rotate(1.5708-0.05);
  m23.translate(cml::vector3(-1.55+1.3-0.16, 0, 0.4-0.5-1-0.03));

  ml::Motion m24 = auto_motion("3/bow", 97+21, 230, 2, 1);
  m24.rotate(-1.5708-0.05);
  m24.translate(cml::vector3(-1.3+0.12, 0, 0.4+0.04));

  pa.add_motion(m21);
  pa.add_motion(m22);
  pa.add_motion(m23);
  pa.add_motion(m24);
  pa.translate_origin();
  pa.set_boundaries();

  add_cons_rel_pos_same_time(pa.inner_cons, pa.motions, 0, 53, 1, 53);
  add_cons_rel_pos_same_time(pa.inner_cons, pa.motions, 0, 53, 2, 53);
  add_cons_rel_pos_same_time(pa.inner_cons, pa.motions, 0, 53, 3, 53);
  add_cons_rel_pos_same_time(pa.inner_cons, pa.motions, 1, 53, 2, 53);
  add_cons_rel_pos_same_time(pa.inner_cons, pa.motions, 1, 53, 3, 53);
  add_cons_rel_pos_same_time(pa.inner_cons, pa.motions, 2, 53, 3, 53);	
}

void make_patch_kick( Patch & pa )
{
  pa.name = "kick";

  ml::Motion m21 = auto_motion("3/walk_side", 256+23, 351, 2, 1);
  m21.translate_time_to_zero();
  m21.rotate(-1.5);
  m21.translate(cml::vector3(-1.55, 0, -0.27));

  ml::Motion m22 = auto_motion("1/flying_kick", 282-225, 391-225 - 32, 1, 1, -1, -1, 0.02);
  m22.translate_time_to_zero();
  m22.translate_time(26-23);

  pa.add_motion(m21);
  pa.add_motion(m22);
  pa.translate_origin();
  pa.set_boundaries();

  add_cons_rel_pos_same_time(pa.inner_cons, pa.motions, 0, 32, 1, 32-3);	
}

void make_patch_shake_hands( Patch & pa )
{
  pa.name = "shake_hands";

  ml::Motion m21 = auto_motion("3/shake_hand", 686, 787, 1, 1, -1, -1, 0.015);
  m21.IkLimbSmoothly(721-686, 7, 4, 15, m21.posture(738-686).GetGlobalTranslation(15));
  m21.translate_time_to_zero();

  ml::Motion m22 = auto_motion("3/shake_hand", 686, 787, 1, 1, -1, -1, 0.015);
  m22.IkLimbSmoothly(721-686, 7, 4, 15, m22.posture(738-686).GetGlobalTranslation(15));
  m22.translate_time_to_zero();
  m22.rotate(cml::pi());
  m22.translate(cml::vector3(-2.75, 0.0, -1.4));		

  pa.add_motion(m21);
  pa.add_motion(m22);
  pa.translate_origin();
  pa.set_boundaries();

  add_cons_rel_pos_same_time(pa.inner_cons, pa.motions, 0, 35, 1, 35);
  add_cons_rel_pos_same_time(pa.inner_cons, pa.motions, 0, 50, 1, 50);
  add_cons_rel_pos_same_time(pa.inner_cons, pa.motions, 0, 65, 1, 65);	
}

void make_patch_step_on( Patch & pa )
{
  pa.name = "step_on";

  ml::Motion m21 = auto_motion("2/itonic3_wd2_walktocrawl_1", 11, 54, 2, 2);
  ml::Motion m21_ = auto_motion("2/itonic3_wd2_crawltowalk_1", 44, 112, 1, 2);
  m21.stitch(m21_);
  m21.translate_time_to_zero();
  m21.translate(cml::vector3(0.55, 0, 0.04));

  ml::Motion m22 = auto_motion("1/box", 57, 133, 2, 2, -1, -1, 0.025);
  size_t step_frame = (83-57+102-57)/2;
  cml::vector3 footvias (cml::vector3(m22.posture(step_frame).GetGlobalTranslation(3))+cml::vector3(0., .30, 0.));
  cml::vector3 toevias (m22.posture(step_frame).GetGlobalTranslation(4)+cml::vector3(0., .30, 0.));
  cml::vector3 toedumvias (m22.posture(step_frame).GetGlobalTranslation(20)+cml::vector3(0., .30, 0.));
  m22.IkLimbSmoothly(step_frame, 35, 0, 3, footvias);
  m22.IkLimbSmoothly(step_frame, 35, 0, 4, toevias);
  m22.IkLimbSmoothly(step_frame, 35, 0, 20, toedumvias);

  for (size_t i=0; i<1; ++i) {
	m22.posture(step_frame+i+1).IkLimb(3, footvias);
	m22.posture(step_frame+i+1).IkLimb(4, toevias);
	m22.posture(step_frame+i+1).IkLimb(20, toedumvias);
  }
  m22.IkLimbSmoothly(step_frame+2, 0, 15, 3, footvias);
  m22.IkLimbSmoothly(step_frame+2, 0, 15, 4, toevias);
  m22.IkLimbSmoothly(step_frame+2, 0, 15, 20, toedumvias);

  m22.translate_time_to_zero();
  m22.rotate(-1.0*cml::pi()/2.0);
  m22.translate_time(16.0);		

  pa.add_motion(m21);
  pa.add_motion(m22);
  pa.translate_origin();
  pa.set_boundaries();

  add_cons_rel_pos_same_time(pa.inner_cons, pa.motions, 0, 41-0, 1, 41-16);
  add_cons_rel_pos_same_time(pa.inner_cons, pa.motions, 0, 65-0, 1, 65-16);	
}

void make_patch_assist_man( Patch & pa )
{
  pa.name = "assist_man";

  ml::Motion m31 = auto_motion("1/walk_left_90degree", 98, 115, 2, 1);
  ml::Motion m31_ = auto_motion("1/walk_left_90degree", 150, 156, 1, 2);
  m31.stitch(m31_);
  ml::Motion m31__ = auto_motion("1/throw", 248, 278, 2, 2);
  m31.stitch(m31__);
  m31.translate_time_to_zero();
  m31.rotate(-1.0*cml::pi()/2.0);

  ml::Motion m32 = auto_motion("3/walk_stop_walk", 145, 188, 1, 1);
  ml::Motion m32_ = auto_motion("1/jump_high_1", 54, 98, 1, 2);
  m32.stitch(m32_);
  m32.translate_time_to_zero();
  m32.translate(cml::vector3(-0.19, 0., -0.9));
  m32.translate_time(-11.0);

  ml::Motion m33 = auto_motion("2/itonic3_wd2_walktorabbit_1", 32, 87, 1, 2);
  ml::Motion m33_= auto_motion("2/itonic3_wd2_rabbittowalk_1", 54, 109, 2, 1);
  m33.stitch(m33_);
  m33.translate_time_to_zero();
  m33.rotate(cml::pi()/2.0);
  m33.translate(cml::vector3(-1.63, 0., -0.7));
  m33.translate_time(5.);

  pa.add_motion(m31);
  pa.add_motion(m32);
  pa.add_motion(m33);
  pa.translate_origin();
  pa.set_boundaries();

  add_cons_rel_pos_same_time(pa.inner_cons, pa.motions, 0, 40-0, 1, 40+11);
  add_cons_rel_pos_same_time(pa.inner_cons, pa.motions, 1, 56+11, 2, 56-5);
}

void make_patch_poke_3( Patch & pa )
{
  pa.name = "poke_3";

  ml::Motion m31 = auto_motion("3/light", 83, 179, 1, 2);
  m31.translate_time_to_zero();
  m31.translate(cml::vector3(0., 0., 1.25));

  ml::Motion m32 = auto_motion("3/light", 83, 179, 1, 2);
  m32.translate_time_to_zero();
  m32.rotate(-2.0*cml::pi()/3.0);
  m32.translate(cml::vector3(-1.15, 0., -0.2));

  ml::Motion m33 = auto_motion("3/light", 83, 179, 1, 2);
  m33.translate_time_to_zero();
  m33.rotate(2.0*cml::pi()/3.0);
  m33.translate(cml::vector3(0.7, 0., -0.5));

  pa.add_motion(m31);
  pa.add_motion(m32);
  pa.add_motion(m33);
  pa.translate_origin();
  pa.set_boundaries();

  add_cons_rel_pos_same_time(pa.inner_cons, pa.motions, 0, 49-0, 1, 49-0);
  add_cons_rel_pos_same_time(pa.inner_cons, pa.motions, 0, 49-0, 2, 49-0);
  add_cons_rel_pos_same_time(pa.inner_cons, pa.motions, 1, 49-0, 2, 49-0);
}

void make_patch_highfive_6( Patch & pa )
{
  pa.name = "highfive_6";

  ml::Motion m61 = auto_motion("1/high_five", 86, 137, 1, 2);
  m61.translate_time_to_zero();
  ml::Motion m62 = auto_motion("1/high_five", 86, 137, 1, 2);
  m62.translate_time_to_zero();
  m62.rotate(cml::pi());
  m62.translate(cml::vector3(-1.45, 0., -0.4));

  ml::Motion m63 = auto_motion("1/high_five", 86, 137, 1, 2);
  m63.translate_time_to_zero();
  m63.translate(cml::vector3(0., 0., 1.5));
  ml::Motion m64 = auto_motion("1/high_five", 86, 137, 1, 2);
  m64.translate_time_to_zero();
  m64.rotate(cml::pi());
  m64.translate(cml::vector3(-1.45, 0., -0.4+1.5));

  ml::Motion m65 = auto_motion("1/high_five", 86, 137, 1, 2);
  m65.translate_time_to_zero();
  m65.translate(cml::vector3(0., 0., -1.5));
  ml::Motion m66 = auto_motion("1/high_five", 86, 137, 1, 2);
  m66.translate_time_to_zero();
  m66.rotate(cml::pi());
  m66.translate(cml::vector3(-1.45, 0., -0.4-1.5));

  pa.add_motion(m61);
  pa.add_motion(m62);
  pa.add_motion(m63);
  pa.add_motion(m64);
  pa.add_motion(m65);
  pa.add_motion(m66);
  pa.translate_origin();
  pa.set_boundaries();

  add_cons_rel_pos_same_time(pa.inner_cons, pa.motions, 0, 5, 1, 5);
  add_cons_rel_pos_same_time(pa.inner_cons, pa.motions, 0, 5, 3, 5);
  add_cons_rel_pos_same_time(pa.inner_cons, pa.motions, 0, 5, 2, 5);
  add_cons_rel_pos_same_time(pa.inner_cons, pa.motions, 0, 5, 4, 5);
  add_cons_rel_pos_same_time(pa.inner_cons, pa.motions, 1, 5, 3, 5);
  add_cons_rel_pos_same_time(pa.inner_cons, pa.motions, 1, 5, 4, 5);
  add_cons_rel_pos_same_time(pa.inner_cons, pa.motions, 1, 5, 5, 5);
  add_cons_rel_pos_same_time(pa.inner_cons, pa.motions, 2, 5, 3, 5);
  add_cons_rel_pos_same_time(pa.inner_cons, pa.motions, 4, 5, 5, 5);
  add_cons_rel_pos_same_time(pa.inner_cons, pa.motions, 0, 18, 1, 18);
  add_cons_rel_pos_same_time(pa.inner_cons, pa.motions, 0, 18, 3, 18);
  add_cons_rel_pos_same_time(pa.inner_cons, pa.motions, 0, 18, 2, 18);
  add_cons_rel_pos_same_time(pa.inner_cons, pa.motions, 0, 18, 4, 18);
  add_cons_rel_pos_same_time(pa.inner_cons, pa.motions, 1, 18, 3, 18);
  add_cons_rel_pos_same_time(pa.inner_cons, pa.motions, 1, 18, 4, 18);
  add_cons_rel_pos_same_time(pa.inner_cons, pa.motions, 1, 18, 5, 18);
  add_cons_rel_pos_same_time(pa.inner_cons, pa.motions, 2, 18, 3, 18);
  add_cons_rel_pos_same_time(pa.inner_cons, pa.motions, 4, 18, 5, 18);	
}

void make_patch_ballet_3( Patch & pa )
{
  pa.name = "ballet_3";

  ml::Motion m31 = auto_motion("2/itonic3_wd2_vallet_3", 7, 62, 1, 2);
  m31.translate_time_to_zero();
  ml::Motion m32 = auto_motion("2/itonic3_wd2_vallet_3", 7, 62, 1, 2);
  m32.translate_time_to_zero();
  ml::Motion m33 = auto_motion("2/itonic3_wd2_vallet_3", 7, 62, 1, 2);
  m33.translate_time_to_zero();
  m31.rotate(cml::pi());
  m31.translate(cml::vector3(-0.6, 0., 0.));
  m32.translate(cml::vector3(0., 0., 1.0));
  m33.translate(cml::vector3(0., 0., -0.5));

  pa.add_motion(m31);
  pa.add_motion(m32);
  pa.add_motion(m33);
  pa.translate_origin();
  pa.set_boundaries();

  add_cons_rel_pos_same_time(pa.inner_cons, pa.motions, 0, 27, 1, 27);
  add_cons_rel_pos_same_time(pa.inner_cons, pa.motions, 0, 27, 2, 27);
  add_cons_rel_pos_same_time(pa.inner_cons, pa.motions, 1, 27, 2, 27);	
}

void make_patch_accicdent( Patch & pa )
{	
  pa.name = "accident";

  ml::Motion m21 = auto_motion("2/itonic3_wd2_walktorun", 15, 38, 2, 2);
  ml::Motion m21_ = auto_motion("2/itonic3_wd2_runtowalk_2", 0, 30, 2, 2);
  m21.stitch(m21_);
  m21.translate_time_to_zero();

  ml::Motion m22 = auto_motion("2/itonic3_wd2_slip", 35, 116, 1, 1);
  m22.translate_time_to_zero();
  m22.rotate(cml::pi()/2.0);
  m22.translate(cml::vector3(0.8, 0.0, 0.5));
  m22.translate_time(13.0);

  pa.add_motion(m21);
  pa.add_motion(m22);
  pa.translate_origin();
  pa.set_boundaries();

  add_cons_rel_pos_same_time(pa.inner_cons, pa.motions, 0, 24-0, 1, 24-13);		
}

void make_patch_wheel_poke_1( Patch & pa )
{	
  pa.name = "wheel_poke_1";

  ml::Motion m21 = auto_motion("3/light", 83, 179, 1, 2);
  m21.translate_time_to_zero();
  m21.rotate(cml::pi());
  m21.translate(cml::vector3(-0.3, 0., -2.2));
  ml::Motion m22 = auto_motion("2/itonic3_wd2_wheel",19, 86, 2, 1);
  m22.translate_time_to_zero();
  m22.translate(cml::vector3(-0.2, 0., -1.2));
  m22.translate_time(18);

  m22.translateSmoothly(46-18, 15, 10, m22.posture(46-18).GetGlobalTranslation(0)+cml::vector3(0., -.14, 0.));

  pa.add_motion(m21);
  pa.add_motion(m22);
  pa.translate_origin();
  pa.set_boundaries();

  add_cons_rel_pos_same_time(pa.inner_cons, pa.motions, 0, 46-0, 1, 46-18);
}

void make_patch_wheel_poke_2( Patch & pa )
{
  pa.name = "wheel_poke_2";

  ml::Motion m31 = auto_motion("3/light", 83, 179, 1, 2);
  m31.translate_time_to_zero();
  ml::Motion m32 = auto_motion("3/light", 83, 179, 1, 2);
  m32.translate_time_to_zero();
  m32.rotate(cml::pi());
  m32.translate(cml::vector3(-0.3, 0., -2.2));
  ml::Motion m33 = auto_motion("2/itonic3_wd2_wheel",19, 86, 2, 1);
  m33.translate_time_to_zero();
  m33.translate(cml::vector3(-0.2, 0., -1.2));
  m33.translate_time(18);
  m33.translateSmoothly(45-18, 15, 10, m33.posture(45-18).GetGlobalTranslation(0)+cml::vector3(0., -.14, 0.));

  pa.add_motion(m31);
  pa.add_motion(m32);
  pa.add_motion(m33);
  pa.translate_origin();
  pa.set_boundaries();

  add_cons_rel_pos_same_time(pa.inner_cons, pa.motions, 0, 49-0, 1, 49-0);
  add_cons_rel_pos_same_time(pa.inner_cons, pa.motions, 0, 49-0, 2, 49-18);
  add_cons_rel_pos_same_time(pa.inner_cons, pa.motions, 1, 49-0, 2, 49-18);
}

void make_patch_wheel_2_oneway( Patch & pa )
{
  pa.name = "wheel_2_oneway";

  ml::Motion m21 = auto_motion("2/itonic3_wd2_wheel",19, 86, 2, 1);
  m21.translate_time_to_zero();
  ml::Motion m22 = auto_motion("2/itonic3_wd2_wheel",19, 86, 2, 1);
  m22.translate_time_to_zero();
  m22.translate(cml::vector3(0., 0., 0.8));
  m21.translateSmoothly(28, 15, 10, m21.posture(28).GetGlobalTranslation(0)+cml::vector3(0., -.14, 0.));
  m22.translateSmoothly(28, 15, 10, m22.posture(28).GetGlobalTranslation(0)+cml::vector3(0., -.14, 0.));

  pa.add_motion(m21);
  pa.add_motion(m22);
  pa.translate_origin();
  pa.set_boundaries();

  add_cons_rel_pos_same_time(pa.inner_cons, pa.motions, 0, 10, 1, 10);
  add_cons_rel_pos_same_time(pa.inner_cons, pa.motions, 0, 20, 1, 20);
  add_cons_rel_pos_same_time(pa.inner_cons, pa.motions, 0, 28, 1, 28);
  add_cons_rel_pos_same_time(pa.inner_cons, pa.motions, 0, 43, 1, 43);	
}

void make_patch_highfive( Patch & pa )
{
  pa.name = "high_five";

  ml::Motion m21 = auto_motion("1/high_five", 86, 137, 1, 2, -1, -1, 0.02);
  m21.translate_time_to_zero();
  ml::Motion m22 = auto_motion("1/high_five", 86, 137, 1, 2, -1, -1, 0.02);
  m22.translate_time_to_zero();
  m22.rotate(cml::pi());
  m22.translate(cml::vector3(-1.5+0.02, 0.0, -0.4+0.03));

  pa.add_motion(m21);
  pa.add_motion(m22);
  pa.translate_origin();
  pa.set_boundaries();

  add_cons_rel_pos_same_time(pa.inner_cons, pa.motions, 0, 5-0, 1, 5-0);
  add_cons_rel_pos_same_time(pa.inner_cons, pa.motions, 0, 16-0, 1, 16-0);
  add_cons_rel_pos_same_time(pa.inner_cons, pa.motions, 0, 21-0, 1, 21-0);
}

void make_patch_highfive_2( Patch & pa )
{
  pa.name = "high_five_2";

  ml::Motion m21 = auto_motion("4/mgmg_highfive1_x2d", 384, 457, 1, 2);
  m21.translate_time_to_zero();
  m21.translate(cml::vector3(0., 0.15, 0.));
  ml::Motion m22 = auto_motion("4/mgmg_highfive1_x2d", 384, 457, 1, 2);
  m22.translate_time_to_zero();
  m22.rotate(cml::pi());
  m22.translate(cml::vector3(-.67, 0.15, -.92));

  pa.add_motion(m21);
  pa.add_motion(m22);
  pa.translate_origin();
  pa.set_boundaries();

  add_cons_rel_pos_same_time(pa.inner_cons, pa.motions, 1, 29-0, 0, 29-0);
}

void clip_copy_motion( ml::Motion& cliped_mot, const char * fullname, int first, int last, double ground )
{
  string fullname_(fullname);
  if (motions_type.find(fullname_) == motions_type.end()) { 
	motions_type[fullname_] = ml::Motion();
	motions_type[fullname_].LoadAMC_with_contactInfo(("./data/"+ fullname_ + ".amc").c_str(), "./data/wd2.asf", true, 0.027, ground);
  }
  cliped_mot.get_motion(motions_type[fullname_], first, last); 
}

void make_patch_chicken( vector< shared_ptr<Patch> > &cooked_patches )
{
  string name = "chicken";  
  {
	Patch pa;
	pa.name = name + "1";
	ml::Motion m21 = auto_motion("multi/hys_chicken_2", 252, 311);		
	ml::Motion m22 = auto_motion("multi/mmk_chicken_2", 253, 323);
	pa.add_motion(m21);
	pa.add_motion(m22);

	pa.translate_origin();
	pa.set_boundaries();

	add_cons_rel_pos_same_time(pa.inner_cons, pa.motions, 0, 278-252, 1, 278-253);

	cooked_patches.push_back(shared_ptr<Patch>(new Patch(pa)));
  }
  {
	Patch pa;
	pa.name = name + "2";
	ml::Motion m21 = auto_motion("multi/hys_chicken_2", 414, 486);
	ml::Motion m22 = auto_motion("multi/mmk_chicken_2", 409, 542);	
	pa.add_motion(m21);
	pa.add_motion(m22);

	pa.translate_origin();
	pa.set_boundaries();

	add_cons_rel_pos_same_time(pa.inner_cons, pa.motions, 0, 452-414, 1, 452-409);

	cooked_patches.push_back(shared_ptr<Patch>(new Patch(pa)));
  }
  {
	Patch pa;
	pa.name = name + "3";
	ml::Motion m21 = auto_motion("multi/hys_chicken_2", 570, 651);
	ml::Motion m22 = auto_motion("multi/mmk_chicken_2", 570, 655);	
	pa.add_motion(m21);
	pa.add_motion(m22);

	pa.translate_origin();
	pa.set_boundaries();

	add_cons_rel_pos_same_time(pa.inner_cons, pa.motions, 0, 605-570, 1, 605-570);

	cooked_patches.push_back(shared_ptr<Patch>(new Patch(pa)));
  }
  {
	Patch pa;
	pa.name = name + "4";
	ml::Motion m21 = auto_motion("multi/hys_chicken_2", 738, 808);
	ml::Motion m22 = auto_motion("multi/mmk_chicken_2", 724, 820);	
	pa.add_motion(m21);
	pa.add_motion(m22);

	pa.translate_origin();
	pa.set_boundaries();

	add_cons_rel_pos_same_time(pa.inner_cons, pa.motions, 0, 764-738, 1, 724-738);

	cooked_patches.push_back(shared_ptr<Patch>(new Patch(pa)));
  }
  {
	Patch pa;
	pa.name = name + "5";
	ml::Motion m21, m22; 
	clip_copy_motion(m21, "multi/hys_chicken_2", 1245, 1316, 0.05);
	clip_copy_motion(m22, "multi/mmk_chicken_2", 1255, 1367);	
	pa.add_motion(m21);
	pa.add_motion(m22);

	pa.translate_origin();
	pa.set_boundaries();

	add_cons_rel_pos_same_time(pa.inner_cons, pa.motions, 0, 1286-1245, 1, 1286-1255);

	cooked_patches.push_back(shared_ptr<Patch>(new Patch(pa)));
  }
  {
	Patch pa;
	pa.name = name + "6";
	ml::Motion m21 = auto_motion("multi/hys_chicken_2", 1376, 1453, 0.05);
	ml::Motion m22 = auto_motion("multi/mmk_chicken_2", 1381, 1465);	
	pa.add_motion(m21);
	pa.add_motion(m22);

	pa.translate_origin();
	pa.set_boundaries();

	add_cons_rel_pos_same_time(pa.inner_cons, pa.motions, 0, 1409-1376, 1, 1409-1381);

	cooked_patches.push_back(shared_ptr<Patch>(new Patch(pa)));
  }


  name = "chicken_U";
  {
	Patch pa;
	pa.name = name + "1";	// turn left
	ml::Motion m21 = auto_motion("multi/hys_chicken_2", 312, 341);	
	pa.add_motion(m21);
	
	pa.translate_origin();
	pa.set_boundaries();

	cooked_patches.push_back(shared_ptr<Patch>(new Patch(pa)));
  }
  {
	Patch pa;
	pa.name = name + "2";	// turn left
	ml::Motion m21 = auto_motion("multi/hys_chicken_2", 326, 355);	
	pa.add_motion(m21);

	pa.translate_origin();
	pa.set_boundaries();

	cooked_patches.push_back(shared_ptr<Patch>(new Patch(pa)));
  }
  {
	Patch pa;
	pa.name = name + "3";	// turn left
	ml::Motion m21 = auto_motion("multi/hys_chicken_2", 341, 385);	
	pa.add_motion(m21);

	pa.translate_origin();
	pa.set_boundaries();

	cooked_patches.push_back(shared_ptr<Patch>(new Patch(pa)));
  }
  {
	Patch pa;
	pa.name = name + "4";	// turn right
	ml::Motion m21 = auto_motion("multi/hys_chicken_2", 500, 528);	
	pa.add_motion(m21);

	pa.translate_origin();
	pa.set_boundaries();

	cooked_patches.push_back(shared_ptr<Patch>(new Patch(pa)));
  }
  {
	Patch pa;
	pa.name = name + "5";	// turn left
	ml::Motion m21 = auto_motion("multi/mmk_chicken_2", 322, 394);	
	pa.add_motion(m21);

	pa.translate_origin();
	pa.set_boundaries();

	cooked_patches.push_back(shared_ptr<Patch>(new Patch(pa)));
  }
  {
	Patch pa;
	pa.name = name + "6";	// straight
	ml::Motion m21 = auto_motion("multi/hys_chicken_2", 651, 679);	
	pa.add_motion(m21);

	pa.translate_origin();
	pa.set_boundaries();

	cooked_patches.push_back(shared_ptr<Patch>(new Patch(pa)));
  }
  {
	Patch pa;
	pa.name = name + "7";	// straight
	ml::Motion m21 = auto_motion("multi/mmk_chicken_2", 669, 710);	
	pa.add_motion(m21);

	pa.translate_origin();
	pa.set_boundaries();

	cooked_patches.push_back(shared_ptr<Patch>(new Patch(pa)));
  }
  {
	Patch pa;
	pa.name = name + "8";	// straight
	ml::Motion m21 = auto_motion("multi/mmk_chicken_2", 806, 821);	
	pa.add_motion(m21);

	pa.translate_origin();
	pa.set_boundaries();

	cooked_patches.push_back(shared_ptr<Patch>(new Patch(pa)));
  }
  {
	Patch pa;
	pa.name = name + "9";	// straight
	ml::Motion m21 = auto_motion("multi/mmk_chicken_2", 996, 1026);	
	pa.add_motion(m21);

	pa.translate_origin();
	pa.set_boundaries();

	cooked_patches.push_back(shared_ptr<Patch>(new Patch(pa)));
  }
  {
	Patch pa;
	pa.name = name + "10";	// straight
	ml::Motion m21 = auto_motion("multi/mmk_chicken_2", 176, 237);	
	pa.add_motion(m21);

	pa.translate_origin();
	pa.set_boundaries();

	cooked_patches.push_back(shared_ptr<Patch>(new Patch(pa)));
  }
  {
	Patch pa;
	pa.name = name + "11";	// straight
	ml::Motion m21 = auto_motion("multi/mmk_chicken_2", 496, 541);	
	pa.add_motion(m21);

	pa.translate_origin();
	pa.set_boundaries();

	cooked_patches.push_back(shared_ptr<Patch>(new Patch(pa)));
  }
  {
	Patch pa;
	pa.name = name + "12";	// turn right
	ml::Motion m21 = auto_motion("multi/hys_chicken_2", 484, 513);	
	pa.add_motion(m21);

	pa.translate_origin();
	pa.set_boundaries();

	cooked_patches.push_back(shared_ptr<Patch>(new Patch(pa)));
  }
  {
	Patch pa;
	pa.name = name + "13";	// turn right
	ml::Motion m21 = auto_motion("multi/hys_chicken_2", 541, 569);	
	pa.add_motion(m21);

	pa.translate_origin();
	pa.set_boundaries();

	cooked_patches.push_back(shared_ptr<Patch>(new Patch(pa)));
  }
  {
	Patch pa;
	pa.name = name + "14";	// turn right
	ml::Motion m21 = auto_motion("multi/hys_chicken_2", 555, 585);	
	pa.add_motion(m21);

	pa.translate_origin();
	pa.set_boundaries();

	cooked_patches.push_back(shared_ptr<Patch>(new Patch(pa)));
  }
  {
	Patch pa;
	pa.name = name + "15";	// turn right
	ml::Motion m21 = auto_motion("multi/hys_chicken_2", 780, 807);	
	pa.add_motion(m21);

	pa.translate_origin();
	pa.set_boundaries();

	cooked_patches.push_back(shared_ptr<Patch>(new Patch(pa)));
  }
}




