#include "StdAfx.h"
#include "get_patch.h"
#include "ingredient.h"

void put_in_basket_jump(Ingredient & ingr);
void put_in_basket_push(Ingredient & ingr);
void put_in_basket_beat(Ingredient & ingr);
void put_in_basket_beat_4(Ingredient & ingr);
void put_in_basket_bow(Ingredient & ingr);
void put_in_basket_bow_4(Ingredient & ingr);
void put_in_basket_shake_hands(Ingredient & ingr);
void put_in_basket_step_on(Ingredient & ingr);
void put_in_basket_poke_3(Ingredient & ingr);
void put_in_basket_ballet_3(Ingredient & ingr);
void put_in_basket_accicdent(Ingredient & ingr);
void put_in_basket_wheel_poke_1(Ingredient & ingr);
void put_in_basket_wheel_poke_2(Ingredient & ingr);
void put_in_basket_wheel_2_oneway(Ingredient & ingr);
void put_in_basket_highfive(Ingredient & ingr);
void put_in_basket_highfive_2(Ingredient & ingr);
void put_in_basket_kick(Ingredient & ingr);

void put_raw_data_push(Ingredient &ing);


void put_in_basket_jump( Ingredient & ingr )
{
  ml::Motion m21;
  clip_copy_motion(m21, "3/jump", 270+22-20, 393+20);
  m21.translate_time_to_zero();
  m21.translate(cml::vector3(-0.1, 0, 0));
  m21.translate_time(22);

  ml::Motion m22;
  clip_copy_motion(m22, "2/itonic3_wd2_walktoduck", 27-20, 86);
  ml::Motion m23;
  clip_copy_motion(m23, "2/itonic3_wd2_ducktowalk", 26, 100+20);
  m22.stitch(m23);
  m22.translate_time_to_zero();
  m22.rotate(cml::pi() * 0.5);
  m22.translate(cml::vector3(0,0,0.73));
  
  Ingredient::in_motions mix;
  mix.push_back(m21);
  mix.push_back(m22);
  ingr.push_in_basket(mix);
}

void put_in_basket_push( Ingredient & ingr )
{
  ml::Motion m21;
  clip_copy_motion(m21, "3/pushed", 803-75-30, 910-75-41+30);
  m21.translate_time_to_zero();
  m21.rotate(1.58319);
  m21.translate(cml::vector3(-1.61513, 0, 1.01225));
  m21.translate_time(33-38);

  ml::Motion m22;
  clip_copy_motion(m22, "3/push_person", 93-30, 160);
  m22.translate_time_to_zero();
  m22.translate(cml::vector3(-0.309585, 0, 1.91209));

  ml::Motion m22_;
  clip_copy_motion(m22_, "3/push_person", 231, 270+30);
  m22.stitch(m22_);
  
  Ingredient::in_motions mix;
  mix.push_back(m22);
  mix.push_back(m21);
  ingr.push_in_basket(mix);
}

void put_in_basket_highfive_2( Ingredient & ingr )
{
  ml::Motion m21;
  clip_copy_motion(m21, "4/mgmg_highfive1_x2d", 384-15, 457+15);
  m21.translate_time_to_zero();
  m21.translate(cml::vector3(0., 0.15, 0.));
  ml::Motion m22;
  clip_copy_motion(m22, "4/mgmg_highfive1_x2d", 384-15, 457+15);
  m22.translate_time_to_zero();
  m22.rotate(cml::pi());
  m22.translate(cml::vector3(-.67, 0.15, -.92));

  Ingredient::in_motions mix;
  mix.push_back(m21);
  mix.push_back(m22);
  ingr.push_in_basket(mix);
}

void put_in_basket_highfive( Ingredient & ingr )
{
  ml::Motion m21;
  clip_copy_motion(m21, "1/high_five", 86-20, 137+20, 0.02);
  m21.translate_time_to_zero();
  ml::Motion m22;
  clip_copy_motion(m22, "1/high_five", 86-20, 137+20, 0.02);
  m22.translate_time_to_zero();
  m22.rotate(cml::pi());
  m22.translate(cml::vector3(-1.5+0.02, 0.0, -0.4+0.03));

  Ingredient::in_motions mix;
  mix.push_back(m21);
  mix.push_back(m22);
  ingr.push_in_basket(mix);
}

void put_in_basket_wheel_2_oneway( Ingredient & ingr )
{
  ml::Motion m21;
  clip_copy_motion(m21, "2/itonic3_wd2_wheel",19-19, 86+19);
  m21.translate_time_to_zero();
  ml::Motion m22;
  clip_copy_motion(m22, "2/itonic3_wd2_wheel",19-19, 86+19);
  m22.translate_time_to_zero();
  m22.translate(cml::vector3(0., 0., 0.8));
  m21.translateSmoothly(28, 15, 10, m21.posture(28).GetGlobalTranslation(0)+cml::vector3(0., -.14, 0.));
  m22.translateSmoothly(28, 15, 10, m22.posture(28).GetGlobalTranslation(0)+cml::vector3(0., -.14, 0.));

  Ingredient::in_motions mix;
  mix.push_back(m21);
  mix.push_back(m22);
  ingr.push_in_basket(mix);
}

void put_in_basket_wheel_poke_2( Ingredient & ingr )
{
  ml::Motion m31;
  clip_copy_motion(m31, "3/light", 83-15, 179+15);
  m31.translate_time_to_zero();
  ml::Motion m32;
  clip_copy_motion(m32, "3/light", 83-15, 179+15);
  m32.translate_time_to_zero();
  m32.rotate(cml::pi());
  m32.translate(cml::vector3(-0.3, 0., -2.2));
  ml::Motion m33;
  clip_copy_motion(m33, "2/itonic3_wd2_wheel",19-15, 86+15);
  m33.translate_time_to_zero();
  m33.translate(cml::vector3(-0.2, 0., -1.2));
  m33.translate_time(18);
  //m33.translateSmoothly(45-18, 15, 10, m33.posture(45-18).GetGlobalTranslation(0)+cml::vector3(0., -.14, 0.));

  Ingredient::in_motions mix;
  mix.push_back(m31);
  mix.push_back(m32);
  mix.push_back(m33);
  ingr.push_in_basket(mix);
}

void put_in_basket_wheel_poke_1( Ingredient & ingr )
{
  ml::Motion m21;
  clip_copy_motion(m21, "3/light", 83-15, 179+15);
  m21.translate_time_to_zero();
  m21.rotate(cml::pi());
  m21.translate(cml::vector3(-0.3, 0., -2.2));
  ml::Motion m22;
  clip_copy_motion(m22, "2/itonic3_wd2_wheel",19-15, 86+15);
  m22.translate_time_to_zero();
  m22.translate(cml::vector3(-0.2, 0., -1.2));
  m22.translate_time(18);
  //m22.translateSmoothly(46-18, 15, 10, m22.posture(46-18).GetGlobalTranslation(0)+cml::vector3(0., -.14, 0.));

  Ingredient::in_motions mix;
  mix.push_back(m21);
  mix.push_back(m22);
  ingr.push_in_basket(mix);
}


void put_in_basket_accicdent( Ingredient & ingr )
{
  ml::Motion m21;
  clip_copy_motion(m21, "2/itonic3_wd2_walktorun", 15-7, 38);
  ml::Motion m21_;
  clip_copy_motion(m21_, "2/itonic3_wd2_runtowalk_2", 0, 30+7);
  m21.stitch(m21_);
  m21.translate_time_to_zero();

  ml::Motion m22;
  clip_copy_motion(m22, "2/itonic3_wd2_slip", 35-8, 116+7);
  m22.translate_time_to_zero();
  m22.rotate(cml::pi()/2.0);
  m22.translate(cml::vector3(0.8, 0.0, 0.5));
  m22.translate_time(13.0);

  Ingredient::in_motions mix;
  mix.push_back(m21);
  mix.push_back(m22);
  ingr.push_in_basket(mix);
}

void put_in_basket_ballet_3( Ingredient & ingr )
{
  ml::Motion m31;
  clip_copy_motion(m31, "2/itonic3_wd2_vallet_3", 7-7, 62+7);
  m31.translate_time_to_zero();
  ml::Motion m32;
  clip_copy_motion(m32, "2/itonic3_wd2_vallet_3", 7-7, 62+7);
  m32.translate_time_to_zero();
  ml::Motion m33;
  clip_copy_motion(m33, "2/itonic3_wd2_vallet_3", 7-7, 62+7);
  m33.translate_time_to_zero();
  m31.rotate(cml::pi());
  m31.translate(cml::vector3(-0.6, 0., 0.));
  m32.translate(cml::vector3(0., 0., 1.0));
  m33.translate(cml::vector3(0., 0., -0.5));

  Ingredient::in_motions mix;
  mix.push_back(m31);
  mix.push_back(m32);
  mix.push_back(m33);
  ingr.push_in_basket(mix);
}

void put_in_basket_poke_3( Ingredient & ingr )
{
  ml::Motion m31; 
  clip_copy_motion(m31, "3/light", 83-30, 179+30);
  m31.translate_time_to_zero();
  m31.translate(cml::vector3(0., 0., 1.25));

  ml::Motion m32;
  clip_copy_motion(m32, "3/light", 83-30, 179+30);
  m32.translate_time_to_zero();
  m32.rotate(-2.0*cml::pi()/3.0);
  m32.translate(cml::vector3(-1.15, 0., -0.2));

  ml::Motion m33;
  clip_copy_motion(m33, "3/light", 83-30, 179+30);
  m33.translate_time_to_zero();
  m33.rotate(2.0*cml::pi()/3.0);
  m33.translate(cml::vector3(0.7, 0., -0.5));

  Ingredient::in_motions mix;
  mix.push_back(m31);
  mix.push_back(m32);
  mix.push_back(m33);
  ingr.push_in_basket(mix);
}

void put_in_basket_step_on( Ingredient & ingr )
{
  ml::Motion m21;
  clip_copy_motion(m21, "2/itonic3_wd2_walktocrawl_1", 11-10, 54);
  ml::Motion m21_;
  clip_copy_motion(m21_, "2/itonic3_wd2_crawltowalk_1", 44, 112+10);
  m21.stitch(m21_);
  m21.translate_time_to_zero();
  m21.translate(cml::vector3(0.75, 0, 0.04));

  ml::Motion m22;
  clip_copy_motion(m22, "1/box", 57, 133, 0.025);
  size_t step_frame = (83-57+102-57) / 2;
  cml::vector3 footvias (cml::vector3(m22.posture(step_frame).GetGlobalTranslation(3))+cml::vector3(0., .30, 0.));
  cml::vector3 toevias (m22.posture(step_frame).GetGlobalTranslation(4)+cml::vector3(0., .30, 0.));
  cml::vector3 toedumvias (m22.posture(step_frame).GetGlobalTranslation(20)+cml::vector3(0., .30, 0.));
  m22.IkLimbSmoothly(step_frame, 35-10, 0, 3, footvias);
  m22.IkLimbSmoothly(step_frame, 35-10, 0, 4, toevias);
  m22.IkLimbSmoothly(step_frame, 35-10, 0, 20, toedumvias);

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

  Ingredient::in_motions mix;
  mix.push_back(m21);
  mix.push_back(m22);
  ingr.push_in_basket(mix);
}

void put_in_basket_shake_hands( Ingredient & ingr )
{
  ml::Motion m21;
  clip_copy_motion(m21, "3/shake_hand", 686-40, 787+40, 0.015);
  m21.IkLimbSmoothly(721-686, 7, 4, 15, m21.posture(738-686).GetGlobalTranslation(15));
  m21.translate_time_to_zero();

  ml::Motion m22;
  clip_copy_motion(m22, "3/shake_hand", 686-40, 787+40, 0.015);
  m22.IkLimbSmoothly(721-686, 7, 4, 15, m22.posture(738-686).GetGlobalTranslation(15));
  m22.translate_time_to_zero();
  m22.rotate(cml::pi());
  m22.translate(cml::vector3(-2.75, 0.0, -1.4));		

  Ingredient::in_motions mix;
  mix.push_back(m21);
  mix.push_back(m22);
  ingr.push_in_basket(mix);
}

void put_in_basket_bow_4( Ingredient & ingr )
{
  ml::Motion m21;
  clip_copy_motion(m21, "3/bow", 97+21-50, 230+50);
  m21.rotate(3.1415-0.05);
  m21.translate(cml::vector3(-1.59561+0.03, 0, -0.974999 + 0.3-0.05));

  ml::Motion m22;
  clip_copy_motion(m22, "3/bow", 97+21-50, 230+50);
  m22.rotate(-0.05);
  m22.translate(cml::vector3(-0.01,0,0.04));

  ml::Motion m23;
  clip_copy_motion(m23, "3/bow", 97+21-50, 230+50);
  m23.rotate(1.5708-0.05);
  m23.translate(cml::vector3(-1.55+1.3-0.16, 0, 0.4-0.5-1-0.03));

  ml::Motion m24; 
  clip_copy_motion(m24, "3/bow", 97+21-50, 230+50);
  m24.rotate(-1.5708-0.05);
  m24.translate(cml::vector3(-1.3+0.12, 0, 0.4+0.04));

  Ingredient::in_motions mix;
  mix.push_back(m21);
  mix.push_back(m22);
  mix.push_back(m23);
  mix.push_back(m24);
  ingr.push_in_basket(mix);
}

void put_in_basket_bow( Ingredient & ingr )
{
  ml::Motion m21;
  clip_copy_motion(m21, "3/bow", 97+21-30, 230+30, 0.01);
  m21.translate_time_to_zero();
  m21.rotate(3.1);
  m21.translate(cml::vector3(-1.59561, 0, -0.974999 + 0.3));

  ml::Motion m22; 
  clip_copy_motion(m22, "3/bow", 97+21-30, 230+30, 0.01);
  m22.translate_time_to_zero();

  Ingredient::in_motions mix;
  mix.push_back(m21);
  mix.push_back(m22);
  ingr.push_in_basket(mix);
}

void put_in_basket_beat_4( Ingredient & ingr )
{
  ml::Motion m21;
  clip_copy_motion(m21, "3/beat", 84-10, 84+101, 0.02);
  m21.translate_time_to_zero();
  m21.rotate(3.14);
  m21.translate(cml::vector3(-4.28558, 0, 0.237186));
  ml::Motion m21_;
  clip_copy_motion(m21_, "3/throw", 444, 503+10, 0.02);
  m21.stitch(m21_);

  ml::Motion m22; 
  clip_copy_motion(m22, "3/beat", 84-10, 84+101, 0.02);
  m22.translate_time_to_zero();
  m22.rotate(1.5);
  m22.translate(cml::vector3(-2.09239, 0, -2.15418));
  ml::Motion m22_; 
  clip_copy_motion(m22_, "3/throw_1", 630, 679+10, 0.02);
  m22.stitch(m22_);

  ml::Motion m23; 
  clip_copy_motion(m23, "3/beat", 84-10, 84+101, 0.02);
  m23.translate_time_to_zero();
  m23.rotate(-1.5);
  m23.translate(cml::vector3(-1.93742, 0, 2.24613));
  ml::Motion m23_; 
  clip_copy_motion(m23_, "3/throw", 444, 503+10, 0.02);
  m23.stitch(m23_);

  ml::Motion m24; 
  clip_copy_motion(m24, "3/beat", 84-10, 84+101, 0.02);
  m24.translate_time_to_zero();
  ml::Motion m24_; 
  clip_copy_motion(m24_, "1/move_3", 238, 281+10, 0.02);
  m24.stitch(m24_);

  Ingredient::in_motions mix;
  mix.push_back(m21);
  mix.push_back(m22);
  mix.push_back(m23);
  mix.push_back(m24);
  ingr.push_in_basket(mix);
}

void put_in_basket_beat( Ingredient & ingr )
{
  ml::Motion m21;
  clip_copy_motion(m21, "3/beat", 84-40, 84+101, 0.02);
  m21.translate_time_to_zero();
  m21.rotate(3.14);
  m21.translate(cml::vector3(-4.28558, 0, 0.237186));
  ml::Motion m21_;
  clip_copy_motion(m21_, "3/pass_jump", 1712, 1751+40, 0.02);
  m21.stitch(m21_);

  ml::Motion m22; 
  clip_copy_motion(m22, "3/beat", 84-40, 84+101, 0.02);
  m22.translate_time_to_zero();
  ml::Motion m22_;
  clip_copy_motion(m22_, "3/throw", 444, 503+40, 0.02);
  m22.stitch(m22_);

  Ingredient::in_motions mix;
  mix.push_back(m21);
  mix.push_back(m22);
  ingr.push_in_basket(mix);
}

void put_in_basket_kick( Ingredient & ingr )
{
  ml::Motion m21; 
  clip_copy_motion(m21, "3/walk_side", 256+23-30, 351+30);
  m21.translate_time_to_zero();
  m21.rotate(-1.5);
  m21.translate(cml::vector3(-1.55, 0, -0.27));

  ml::Motion m22;
  clip_copy_motion(m22, "1/flying_kick", 282-225-30, 391-225-32+30, 0.02);
  m22.translate_time_to_zero();
  m22.translate_time(26-23);

  Ingredient::in_motions mix;
  mix.push_back(m21);
  mix.push_back(m22);
  ingr.push_in_basket(mix);
}

void put_raw_data_push( Ingredient &ing )
{
  ml::Motion m21;
  clip_copy_motion(m21, "multi/hys_push", 0, 1760);	// 1769	  363, 624

  ml::Motion m22;
  clip_copy_motion(m22, "multi/mmk_push", 0, 1760);	// 1769

  Ingredient::in_motions mix;
  mix.push_back(m21);
  mix.push_back(m22);
  ing.push_in_basket(mix);
}

void put_raw_data_hello( Ingredient &ing )
{
  ml::Motion m21, m22;
  clip_copy_motion(m21, "multi/hys_hello", 473, 1780);
  clip_copy_motion(m22, "multi/mmk_hello", 473, 1780);
  /*clip_copy_motion(m21, "multi/hys_hello", 280, 677);
  clip_copy_motion(m22, "multi/mmk_hello", 280, 677);*/
  /*clip_copy_motion(m21, "multi/hys_hello", 1090, 1500);
  clip_copy_motion(m22, "multi/mmk_hello", 1090, 1500);*/

  Ingredient::in_motions mix;
  mix.push_back(m21);
  mix.push_back(m22);
  ing.push_in_basket(mix);
}

void put_raw_data_highfive( Ingredient &ing )
{
  ml::Motion m21, m22;
  clip_copy_motion(m21, "multi/hys_hifive", 25, 1531);
  clip_copy_motion(m22, "multi/mmk_hifive", 25, 1531);

  Ingredient::in_motions mix;
  mix.push_back(m21);
  mix.push_back(m22);
  ing.push_in_basket(mix);
}


void put_raw_data_sync1( Ingredient &ing )
{
  ml::Motion m21, m22;
  clip_copy_motion(m21, "multi/hys_synch", 0, 560); 
  clip_copy_motion(m22, "multi/mmk_synch", 0, 560);  

  Ingredient::in_motions mix;
  mix.push_back(m21);
  mix.push_back(m22);
  ing.push_in_basket(mix);
}

void put_raw_data_sync2( Ingredient &ing )
{
  ml::Motion m23, m24;
  clip_copy_motion(m23, "multi/hys_synch", 1158, 2463);  // 3534
  clip_copy_motion(m24, "multi/mmk_synch", 1158, 2463);

  Ingredient::in_motions mix;  
  mix.push_back(m23);
  mix.push_back(m24);
  ing.push_in_basket(mix);
}

void put_raw_data_dance1( Ingredient &ing )
{
  ml::Motion m21, m22;
  clip_copy_motion(m21, "multi/hys_dance", 20, 1020);	// 3310 1004
  clip_copy_motion(m22, "multi/mmk_dance", 20, 1020);

  Ingredient::in_motions mix;
  mix.push_back(m21);
  mix.push_back(m22);
  ing.push_in_basket(mix);
}

void put_raw_data_dance2( Ingredient &ing )
{
  ml::Motion m21, m22;
  clip_copy_motion(m21, "multi/hys_dance", 1323, 2153);	// 3310 1004
  clip_copy_motion(m22, "multi/mmk_dance", 1323, 2153);

  Ingredient::in_motions mix;
  mix.push_back(m21);
  mix.push_back(m22);
  ing.push_in_basket(mix);
}

void put_raw_data_dance3( Ingredient &ing )
{
  ml::Motion m21, m22;
  clip_copy_motion(m21, "multi/hys_dance", 2879, 3325);	// 3310 1004
  clip_copy_motion(m22, "multi/mmk_dance", 2879, 3325);

  Ingredient::in_motions mix;
  mix.push_back(m21);
  mix.push_back(m22);
  ing.push_in_basket(mix);
}

void put_raw_data_box_game( Ingredient &ing )
{
  ml::Motion m21, m22;
  clip_copy_motion(m21, "multi/hys_box_game", 60, 2040);
  clip_copy_motion(m22, "multi/mmk_box_game", 60, 2040);

  Ingredient::in_motions mix;
  mix.push_back(m21);
  mix.push_back(m22);
  ing.push_in_basket(mix);
}

void put_raw_data_chicken1( Ingredient &ing )
{
  ml::Motion m21, m22;
  clip_copy_motion(m21, "multi/hys_chicken_2", 1247, 1500, 0.05);
  clip_copy_motion(m22, "multi/mmk_chicken_2", 1247, 1465);

  Ingredient::in_motions mix;
  mix.push_back(m21);
  mix.push_back(m22);
  ing.push_in_basket(mix);
}

void put_raw_data_chicken2( Ingredient &ing )
{
  ml::Motion m21, m22;
  clip_copy_motion(m21, "multi/hys_chicken_2", 40, 1145); // 1145 1550
  clip_copy_motion(m22, "multi/mmk_chicken_2", 40, 1145);

  Ingredient::in_motions mix;
  mix.push_back(m21);
  mix.push_back(m22);
  ing.push_in_basket(mix);
}

void put_raw_data_fighter( Ingredient &ing )
{
  ml::Motion m21, m22;
  clip_copy_motion(m21, "multi/hys_fighter", 20, 1820);
  clip_copy_motion(m22, "multi/mmk_fighter", 20, 1820);

  Ingredient::in_motions mix;
  mix.push_back(m21);
  mix.push_back(m22);
  ing.push_in_basket(mix);
}

void put_raw_data_touch_game( Ingredient &ing )
{
  ml::Motion m21, m22;
  clip_copy_motion(m21, "multi/hys_touch_game", 5, 2480);
  clip_copy_motion(m22, "multi/mmk_touch_game", 5, 2480);

  Ingredient::in_motions mix;
  mix.push_back(m21);
  mix.push_back(m22);
  ing.push_in_basket(mix);
}

void put_raw_data_wave( Ingredient &ing )
{
  ml::Motion m21, m22;
  clip_copy_motion(m21, "multi/hys_wave", 240, 620);
  clip_copy_motion(m22, "multi/mmk_wave", 240, 620);

  Ingredient::in_motions mix;
  mix.push_back(m21);
  mix.push_back(m22);
  ing.push_in_basket(mix);
}

void put_raw_data_march_hand1( Ingredient &ing )
{
  ml::Motion m21, m22;
  clip_copy_motion(m21, "multi/hys_march_hand_1", 0, 2091);
  clip_copy_motion(m22, "multi/mmk_march_hand_1", 0, 2091);

  Ingredient::in_motions mix;
  mix.push_back(m21);
  mix.push_back(m22);
  ing.push_in_basket(mix);
}

void put_raw_data_march_hand2( Ingredient &ing )
{
  ml::Motion m21, m22;
  clip_copy_motion(m21, "multi/hys_march_hand_2", 15, 2005);
  clip_copy_motion(m22, "multi/mmk_march_hand_2", 15, 2005);

  Ingredient::in_motions mix;
  mix.push_back(m21);
  mix.push_back(m22);
  ing.push_in_basket(mix);
}


void mix_synthesized_data( Ingredient &ingr )
{
  put_in_basket_jump(ingr);
  put_in_basket_push(ingr);
  put_in_basket_beat(ingr);
  put_in_basket_beat_4(ingr);
  put_in_basket_bow(ingr);
  ////push_in_basket_bow_4(ingr);
  put_in_basket_shake_hands(ingr);
  put_in_basket_step_on(ingr);
  put_in_basket_poke_3(ingr);
  put_in_basket_ballet_3(ingr);
  put_in_basket_accicdent(ingr);
  put_in_basket_wheel_poke_1(ingr);
  ////push_in_basket_wheel_poke_2(ingr);
  ////push_in_basket_wheel_2_oneway(ingr);
  put_in_basket_highfive(ingr);
  put_in_basket_highfive_2(ingr);
  put_in_basket_kick(ingr);
}

void mix_raw_data( Ingredient &ingre )
{
  //put_raw_data_hello(ingre);
  //put_raw_data_highfive(ingre);
  //put_in_basket_shake_hands(ingre);
  //put_in_basket_bow(ingre);

  //put_raw_data_push(ingre);

  put_raw_data_sync1(ingre);
  //put_raw_data_sync2(ingre);

  //put_raw_data_box_game(ingre);  
  //put_raw_data_chicken1(ingre);
  //put_raw_data_chicken2(ingre);
  put_raw_data_dance1(ingre);
  put_raw_data_dance2(ingre);
  put_raw_data_dance3(ingre);
  
  //put_raw_data_fighter(ingre);
  //put_raw_data_march_hand1(ingre);
  //put_raw_data_march_hand2(ingre);
  //put_raw_data_touch_game(ingre);
  put_raw_data_wave(ingre);
}

size_t Ingredient::size_mot() const
{
  size_t count_ = 0;
  for (auto it = raw_before_cook.begin(); it != raw_before_cook.end(); ++it) {
	count_ += it->size();	
  }
  return count_;
}

pair<size_t,size_t> Ingredient::map_mot_to_bundle( size_t seq_mot_idx ) const
{
  int seq = static_cast<int>(seq_mot_idx);
  int i = 0;
  int count_ = static_cast<int>(raw_before_cook[0].size());
  while (seq - count_ >= 0) {
	count_ += static_cast<int>(raw_before_cook[++i].size());
  } 
  return pair<size_t, size_t>(i, seq - (count_ - static_cast<int>(raw_before_cook[i].size())));
}

size_t Ingredient::map_bundle_to_mot( pair<size_t,size_t> bundle ) const
{
  size_t ret = 0;
  size_t b = bundle.first;
  for (size_t i = 0; i < b; ++i) {
	ret += raw_before_cook[i].size();
  } return ret + bundle.second;
}

size_t Ingredient::size_pos() const
{
  size_t count_ = 0;
  for (auto it = raw_before_cook.begin(); it != raw_before_cook.end(); ++it) {
	for (auto jt = it->begin(); jt != it->end(); ++jt) {
	  count_ += jt->size();
	}
  } return count_;
}
