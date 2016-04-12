// +-------------------------------------------------------------------------
// | online_viewer.h
// | 
// | Author: Manmyung Kim
// +-------------------------------------------------------------------------
// | COPYRIGHT:
// |    Copyright Manmyung Kim 2013
// |    See the included COPYRIGHT.txt file for further details.
// |    
// |    This file is part of the TilingMotionPatch.
// |    TilingMotionPatch is free software: you can redistribute it and/or modify
// |    it under the terms of the MIT License.
// |
// |    You should have received a copy of the MIT License
// |    along with TilingMotionPatch.  If not, see <mit-license.org>.
// +-------------------------------------------------------------------------
#pragma once

#include "fl_gl_slider.h"
#include "patch.h"
#include "environment.h"

class PatchArea;
struct Danglings;
class PatchCooker;

struct Situation
{
  Situation();

  void init();
  void tick( int frame );

  vector<shared_ptr<Patch>> patches;
  Env env;

  cml::vector3 target;
  cml::vector3 target1;
  map<string,Patch> patch_types;
  vector<string> sampling_patches;
  vector<shared_ptr<Patch> > input_patches;
  vector<Patch> *patch_type_unary;
  int need_calc_patch_view;

};

class PatchesViewerOnline : public Fl_Gl_Slider
{
 public:
  std::string image_name;
 
 public:
  PatchesViewerOnline();
  ~PatchesViewerOnline(void);

  void draw_main(int frame);
  void draw_patch_area( int frame );
  int handle_main(int event);
  void draw_patch_object(Patch *patch, int frame, bool is_shadow);

  void calc_camera();
  void calc_characters();
  void calc_patch_area(bool full_body);
  void calc_follow_character();
  void change_bound();

  void control_camera(int frame);

  void common_init();
  // 한 패치를 보기 좋게 뷰를 설정한다.
  void set_record_mode();	
  void capture_image( const char *fileName, int index );
 private:
  struct Character {
	  int num_patches;
	  std::vector<ml::Motion *> motions;
  };
  enum CameraControl {
	  kNoCameraControl = 0, 
	  kContrlExample,
	  kLastCameraControl
  } camera_control;
		
  std::vector<Character> characters;
	
  std::vector<PatchArea *> patch_areas;
  std::set<ml::Motion *> follow_set;
  cml::vector3d next_follow_position;
  vector<pair_section> constraints;  

  int show_phase2;
  //bool root_path;
  int path_mode;
  int id_mode;
  bool bound;
  bool shadow;
  int follow_character;
  bool draw_cons;
  bool draw_contact;

  Situation si;
  
 private:
  bool procedural_tog;
  Danglings *dtable;
};