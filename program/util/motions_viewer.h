#pragma once

#include "fl_gl_slider.h"
#include "motion/ml.h"
#include "patch_cooker.h"


class MotionsViewer : public Fl_Gl_Slider
{
 public:
  MotionsViewer();
  MotionsViewer(const vector<ml::Motion> &motions);
  MotionsViewer(ml::Motion &motion);
  MotionsViewer(vector<ml::Motion> &motions);
  MotionsViewer(vector<ml::Motion> &&motions);
  MotionsViewer(int n, vector<ml::Motion> *p_motions, ...);
  MotionsViewer(const ml::Posture &p1, const ml::Posture &p2);
  MotionsViewer(vector<ml::Posture> &ps);
  MotionsViewer(vector<const ml::Posture *> &ps);
  ~MotionsViewer(void);

  void init();
  void time_estimator();
  void draw_main(int frame);
  int handle_main(int event);
  
  void add_motions(const vector<ml::Motion>& add_mots);
  void add_motions_and_reset_time(const vector<ml::Motion>& add_mots);
  
  vector<ml::Motion> view_motions;
  vector<pair_section> interact_pair;
  //vector<section> interact_line;
  
  bool shadow;
  bool draw_boundary;
  //bool draw_path;
  int path_mode;
  bool draw_contact;
  bool draw_dial;
  bool draw_active;
  bool draw_interact;

private:
  void set_color_inOrder(vector<ml::Motion> & m);
};