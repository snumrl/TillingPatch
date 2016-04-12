#include "StdAfx.h"
#include "motions_viewer.h"
#include "screengrab.h"
#include "collision.h"


cml::transf get_box_transf( const ml::Posture &p )
{
  cml::vector3d l1 = p.GetGlobalTranslation(p.body()->joint_index("LeftHandDummy"));
  cml::vector3d r1 = p.GetGlobalTranslation(p.body()->joint_index("RightHandDummy"));

  double theta;
  {
	cml::vector3d v = l1 - r1;
	v[1] = 0;
	v = v.normalize();
	theta = -acos(v[0]);
	if(v[2] < 0) theta = -theta;
  }

  cml::matrix3 rot_m = cml::roty_mat(theta);
  cml::vector3d diff_v = rot_m * cml::vector3d(0, 0, 0.04);
  cml::vector3d object_trans = ((l1+r1) / 2) + diff_v;
  object_trans[1] += p.object_height_correction;
  object_trans[1] -= 0.1;

  cml::transf tran = cml::trans_transf(object_trans) * cml::make_transf(rot_m, cml::vector3d(0,0,0));

  return tran;
}

void draw_abb(const ABB &box)
{
  glPushMatrix();
  glTranslated(box.origin[0], box.origin[1], box.origin[2]);
  glTranslated(box.length[0]/2.0, box.length[1]/2.0, box.length[2]/2.0);
  glScaled(box.length[0], box.length[1], box.length[2]);
  glutSolidCube(1);
  glPopMatrix();
}

static void DrawPosture(const ml::Posture &p, bool shadow, cml::vector3 color)
{
	if (shadow == false) {
	  if (color[0] < 0) {
	    glColor3d(0.3, 0.5, 0.1);
	  } else {
	    glColor3dv(color.data());
	  }
	} else {
		glColor4f(0,0,0,.5);
	}

	for (int i = 1; i < (int)p.num_joint(); ++i) {
		cml::vector3d offset = p.body()->offset(i);
		cml::transf rot_se3 = cml::make_transf(cml::exp_mat3(cml::between_vector(cml::vector3d(1,0,0),offset)), cml::vector3d(0,0,0));

		glPushMatrix();
		cml::transf se3 = p.GetGlobalTransf(p.body()->parent(i));
		glMultMatrixd(se3.data());
		glMultMatrixd(rot_se3.data());
		double length = offset.length();
		glScaled(length * 1.02, 0.04, 0.07);
		glTranslated(0.5, 0, 0);
		glutSolidCube(1);
		glPopMatrix();

		if (false) {
			cml::transf end_se3 = se3 * rot_se3 * cml::trans_transf(cml::vector3d(length * 1.02, 0, 0));
			cml::vector3d end_pos  = cml::trans(end_se3);
			glPointSize(12);
			glBegin(GL_POINTS);
			glVertex3d(end_pos[0], end_pos[1], end_pos[2]);
			glEnd();
		}
	}

	if (p.object == 1) {
		if (shadow == false) {
		  glColor3f(0.6, 0.3, 0.6);
		} else {
		  glColor4f(0,0,0,.15);
		}

		cml::vector3d l1 = p.GetGlobalTranslation(p.body()->joint_index("LeftHandDummy"));
		cml::vector3d r1 = p.GetGlobalTranslation(p.body()->joint_index("RightHandDummy"));		
		cml::vector3d v = l1 - r1;
		v[1] = 0;
		v = v.normalize();
		double theta = -acos(v[0]);
		if (v[2] < 0) {
		  theta = -theta;
		}

		cml::matrix3 rot_m = cml::roty_mat(theta);
		cml::vector3d diff_v = rot_m * cml::vector3d(0, 0, 0.04);
		cml::vector3d object_trans = ((l1+r1) / 2.0) + diff_v;
		//object_trans[1] += p.object_height_correction;
		glPushMatrix();
		cml::transf tran = cml::trans_transf(object_trans) * cml::make_transf(rot_m, cml::vector3d(0,0,0));

		glMultMatrixd(tran.data());
		glScaled(1, 0.5, 0.5);
		glutSolidCube(.6);
		glPopMatrix();
	}	
}

MotionsViewer::MotionsViewer( ml::Motion &motion ) : Fl_Gl_Slider(1280-512, 100, 1024, 768)
{
  view_motions.push_back(motion);
  init();
}

MotionsViewer::MotionsViewer(vector<ml::Motion> &motions) : Fl_Gl_Slider(1280-512, 100, 1024, 768)
{
  view_motions = motions;
  init();
  //set_color_inOrder(view_motions);
}

MotionsViewer::MotionsViewer(vector<ml::Motion> &&motions) : Fl_Gl_Slider(1280-512, 100, 1024, 768)
{
  view_motions = motions;
  init();
}

MotionsViewer::MotionsViewer(int n, vector<ml::Motion> *p_motions, ...) : Fl_Gl_Slider(1280-512, 100, 1024, 768)
{
  view_motions = *p_motions;

  va_list vl;
  va_start(vl, p_motions);     
  for(int i = 1; i < n; i++)
  {
	vector<ml::Motion> *p_m = va_arg(vl, vector<ml::Motion>*);
	copy(p_m->begin(), p_m->end(), back_inserter(view_motions));
  }
  va_end(vl);

  init();
}

MotionsViewer::MotionsViewer(const ml::Posture &p1, const ml::Posture &p2) : Fl_Gl_Slider(1280-512, 100, 1024, 768)
{
  {
	ml::Motion m;
	m.AddPosture(p1);
	m.last_posture().time = 0.0;
	view_motions.push_back(m);
  }
  {
	ml::Motion m;
	m.AddPosture(p2);
	m.last_posture().time = 0.0;
	view_motions.push_back(m);
  }
}

MotionsViewer::MotionsViewer(vector<ml::Posture> &ps) : Fl_Gl_Slider(1280-512, 100, 1024, 768)
{
  for (auto it = ps.begin(); it != ps.end(); ++it)
  {
	ml::Motion m;
	m.AddPosture(*it);
	m.last_posture().time = 0.0;
	view_motions.push_back(m);
  }
}

MotionsViewer::MotionsViewer(vector<const ml::Posture *> &ps) : Fl_Gl_Slider(1280-512, 100, 1024, 768)
{
  for (auto it = ps.begin(); it != ps.end(); ++it)
  {
	ml::Motion m;
	m.AddPosture(**it);
	m.last_posture().time = 0.0;
	view_motions.push_back(m);
  }
}

MotionsViewer::MotionsViewer( const vector<ml::Motion> &motions ): Fl_Gl_Slider(1280-512, 100, 800, 600)	// 1024, 768
{
  view_motions = motions;
  init();
  //set_color_inOrder(view_motions);
}

MotionsViewer::MotionsViewer(): Fl_Gl_Slider(1280-512, 100, 800, 600)	// 1024, 768
{
  init();
}

void MotionsViewer::init()
{
  //draw_path = true;
  path_mode = 0;
  shadow = true;
  draw_boundary = false;
  draw_contact = false;
  draw_dial = false;
  draw_active = false;
  draw_interact = false;
  time_estimator();
  get_camera().rotateX = -cml::pi() / 2.;
}

MotionsViewer::~MotionsViewer(void)
{

}


void MotionsViewer::draw_main( int frame )
{
  if (shadow) {
	setupShadow();
	glColor4f(0,0,0,.15);
	for (auto it = view_motions.begin(); it != view_motions.end(); ++it) {
	  if ((it->first_posture().time - 0.5 < frame) && (frame < it->last_posture().time + 0.5)) {
		cml::vector3d color = it->color;
		auto p_posture = std::min_element(it->begin(), it->end(), 
										[frame](const ml::Posture &p1, const ml::Posture &p2){return fabs(p1.time - frame) < fabs(p2.time - frame);});
		DrawPosture(*p_posture, true, color);
	  }
	}
	unsetupShadow();
  }


  /*{
  double fixframe = 581;

  setupShadow();
  glColor4f(0,0,0,.15);
  for (auto it = view_motions.begin(); it != view_motions.end(); ++it) {
  if ((it->first_posture().time - 0.5 < fixframe) && (fixframe < it->last_posture().time + 0.5)) {
  cml::vector3d color = it->color;
  auto p_posture = std::min_element(it->begin(), it->end(), 
  [fixframe](const ml::Posture &p1, const ml::Posture &p2){return fabs(p1.time - fixframe) < fabs(p2.time - fixframe);});
  DrawPosture(*p_posture, true, color);
  }
  }
  unsetupShadow();

  for (auto it = view_motions.begin(); it != view_motions.end(); ++it) {
  auto p_posture = min_element(it->begin(), it->end(), 
  [fixframe](ml::Posture &p1, ml::Posture &p2){return fabs(p1.time - fixframe) < fabs(p2.time - fixframe);});
  if (fabs(p_posture->time - fixframe) < 0.5 + 0.25) {
  cml::vector3d color = it->color;
  DrawPosture(*p_posture, false, color);
  }
  }
  }*/
  
  // Draw Postures
  for (auto it = view_motions.begin(); it != view_motions.end(); ++it) {
	auto p_posture = min_element(it->begin(), it->end(), 
								[frame](ml::Posture &p1, ml::Posture &p2){return fabs(p1.time - frame) < fabs(p2.time - frame);});
	if (fabs(p_posture->time - frame) < 0.5 + 0.25) {
	  cml::vector3d color = it->color;
	  DrawPosture(*p_posture, false, color);

	  if (draw_dial) {
		glColor3d(color[0], color[1], color[2]);
		cml::vector3 pos = p_posture->trans() + cml::vector3(0., 0.7, 0.);
		glPushMatrix();
		glTranslated(pos[0] - 0.15, pos[1], pos[2]);
		glScaled(0.002, 0.002, 0.002);

		char string[5];
		itoa(static_cast<int>(it-view_motions.begin()), string, 10);
		for (char *c = string; *c != '\0'; c++) {
		  glutStrokeCharacter(GLUT_STROKE_ROMAN, *c);
		}
		glPopMatrix();
	  }
	  if (draw_contact) {	// contact
		glDisable(GL_LIGHTING);
		if (p_posture->leftFootTouched) {
		  glColor4f(0.3, 0.2, 0.2, 0.2);
		  glPushMatrix();
		  cml::vector3 trans_l = p_posture->GetGlobalTranslation(19);
		  glTranslated(trans_l[0], trans_l[1], trans_l[2]);
		  glutSolidSphere( 0.08, 40, 40);
		  glPopMatrix();
		}
		if (p_posture->rightFootTouched) {
		  glColor4f(0.3, 0.2, 0.2, 0.2);
		  glPushMatrix();
		  cml::vector3 trans_r = p_posture->GetGlobalTranslation(4);
		  glTranslated(trans_r[0], trans_r[1], trans_r[2]);
		  glutSolidSphere( 0.08, 40, 40);
		  glPopMatrix();
		}
		glEnable(GL_LIGHTING);
	  }
	  if (!shadow) {
		ABB abb = bound_abb_posture(*p_posture);
		draw_abb(abb);
	  }
	  if (path_mode == 2) {
		glColor3d(0.0, 0.0, 0.0);
		glBegin(GL_LINE_STRIP);
		for (auto it_p = it->begin(); it_p != p_posture; ++it_p) {
		  const cml::vector3d &position = it_p->trans();
		  if (it_p->status.is_highlight) {
			glEnd();
			glDisable(GL_LIGHTING);
			glColor3d(it_p->status.color[0], it_p->status.color[1], it_p->status.color[2]);
			//glColor3d(0.2, 0.3, 0.7);	
			  		  
			/*glBegin(GL_POINTS);
			glPointSize(2.0);
			glVertex3d(position[0], position[1], position[2]);
			glEnd();*/
			glPushMatrix();
			glTranslated(position[0], position[1], position[2]);
			glutSolidSphere(0.02, 30, 30);
			glPopMatrix();
			  
			glEnable(GL_LIGHTING);
			glColor3d(0.0, 0.0, 0.0);	
			glBegin(GL_LINE_STRIP);
		  } else {
			glVertex3d(position[0], position[1], position[2]);
		  }
		}
		glEnd();

		for (auto it=interact_pair.begin(); it!=interact_pair.end(); ++it) {
		  size_t m[2] = {it->get_motion_idx(0), it->get_motion_idx(1)};
		  size_t s[2] = {it->get_section_beg(0), it->get_section_beg(1)};
		  size_t d[2] = {it->get_section_end(0) - it->get_section_beg(0) + 1, it->get_section_end(1) - it->get_section_beg(1) + 1};
		  int l_idx = (d[0] > d[1]) ? 0 : 1;
		  int s_idx = (l_idx == 1) ? 0 : 1;
		  if (true) {
		    /*if (it->type == pair_section::Contact) {
		      glColor3d(1.0, 0.0, 0.0);
		    } else if (it->type == pair_section::Intimate) {
		      glColor3d(0.0, 1.0, 0.0);
		    } else if (it->type == pair_section::Sync) {
		      glColor3d(0.0, 0.0, 1.0);
		    } else {
		      glColor3d(0.0, 0.0, 0.0);
		    }*/
		    glColor3d(0.7, 0.3, 0.3);
		  } else {
		    glColor3d(0.933, 0.1255, 0.5647);
		  }  
		  
		  for (size_t j = 0; j < d[l_idx]; ++j) {
		    cml::vector3 r1 = view_motions[m[l_idx]].posture(s[l_idx] + j).trans();	
		    double time = view_motions[m[l_idx]].posture(s[l_idx] + j).time;
		    cml::vector3 r2 = view_motions[m[s_idx]].posture_at_time(time).trans();
			if (time < frame + 0.5) {
			  glBegin(GL_LINES);
			  glVertex3d(r1[0], r1[1], r1[2]);
			  glVertex3d(r2[0], r2[1], r2[2]);
			  glEnd();
			}			
		  }		  
		}
	  }
	  if (false) {
		auto orient = ml::shoulder_orientation(*p_posture);
		glBegin(GL_LINE_STRIP);
		glVertex3d(0., 0., 0.);
		glVertex3d(10 * orient[0], 10 * orient[1], 10 * orient[2]);
		glEnd();		
	  }	 
	}
  }

  if (path_mode == 1) {
	for (auto it = view_motions.begin(); it != view_motions.end(); ++it) {
	  glColor3d(0.0, 0.0, 0.0);
	  glBegin(GL_LINE_STRIP);	  
	  for (auto it_p=it->begin(); it_p!=it->end(); ++it_p) {
		const cml::vector3d &position = it_p->trans();
		if (it_p->status.is_highlight) {
		  glEnd();
		  glDisable(GL_LIGHTING);
		  glColor3d(it_p->status.color[0], it_p->status.color[1], it_p->status.color[2]);
		  //glColor3d(0.2, 0.3, 0.7);	
		  		  
		  /*glBegin(GL_POINTS);
		  glPointSize(2.0);
		  glVertex3d(position[0], position[1], position[2]);
		  glEnd();*/
		  glPushMatrix();
		  glTranslated(position[0], position[1], position[2]);
		  glutSolidSphere(0.02, 30, 30);
		  glPopMatrix();
		  
		  glEnable(GL_LIGHTING);
		  glColor3d(0.0, 0.0, 0.0);	
		  glBegin(GL_LINE_STRIP);
		} else {
		  glVertex3d(position[0], position[1], position[2]);
		}
	  }
	  glEnd();
	}
  }

  if (draw_interact) {
	for (auto it=interact_pair.begin(); it!=interact_pair.end(); ++it) {
	  size_t m[2] = {it->get_motion_idx(0), it->get_motion_idx(1)};
	  size_t s[2] = {it->get_section_beg(0), it->get_section_beg(1)};
	  size_t d[2] = {it->get_section_end(0) - it->get_section_beg(0) + 1, it->get_section_end(1) - it->get_section_beg(1) + 1};
	  int l_idx = (d[0] > d[1]) ? 0 : 1;
	  int s_idx = (l_idx == 1) ? 0 : 1;
	  if (true) {
		/*if (it->type == pair_section::Contact) {
		  glColor3d(1.0, 0.0, 0.0);
		} else if (it->type == pair_section::Intimate) {
		  glColor3d(0.0, 1.0, 0.0);
		} else if (it->type == pair_section::Sync) {
		  glColor3d(0.0, 0.0, 1.0);
		} else {
		  glColor3d(0.0, 0.0, 0.0);
		}*/
		glColor3d(0.7, 0.3, 0.2);
	  } else {
		glColor3d(0.933, 0.1255, 0.5647);
	  }
	  
	  glBegin(GL_LINES);
	  for (size_t j = 0; j < d[l_idx]; ++j) {
		cml::vector3 r1 = view_motions[m[l_idx]].posture(s[l_idx] + j).trans();	
		double time = view_motions[m[l_idx]].posture(s[l_idx] + j).time;
		cml::vector3 r2 = view_motions[m[s_idx]].posture_at_time(time).trans();
		glVertex3d(r1[0], r1[1], r1[2]);
		glVertex3d(r2[0], r2[1], r2[2]);
	  }
	  glEnd();
	}
  }
}


int MotionsViewer::handle_main( int event )
{
  int return_val = 0;

  if (Fl::event_key('r')!=0) {
	cout << 'r' << endl;
	return_val = 1;
  } else if (Fl::event_key('s')!=0) {
	shadow = !shadow;
	return_val = 1;
  } else if (Fl::event_key('x')!=0) {
	fl_gl_3d->is_draw_axis = !fl_gl_3d->is_draw_axis;
	return_val = 1;
  } else if (Fl::event_key('z')!=0) {
	fl_gl_3d->is_draw_ground = !fl_gl_3d->is_draw_ground;
	return_val = 1;
  } else if (Fl::event_key('v')!=0) {
	SaveScreenGrab("capture.bmp");
	return_val = 1;
  } else if (Fl::event_key('b')!=0) {
	draw_boundary = !draw_boundary;
	return_val = 1;
  } else if (Fl::event_key('p')!=0) {
	//draw_path = !draw_path;
	++path_mode;
	if (path_mode > 2) path_mode = 0;
	return_val = 1;
  } else if (Fl::event_key('c')!=0) {
	draw_contact = !draw_contact;
	return_val = 1;
  } else if (Fl::event_key('d')!=0) {
	draw_dial = !draw_dial;
	return_val = 1;
  } else if (Fl::event_key('i')!=0) {
	draw_interact = !draw_interact;
	return_val = 1;
  }
  else if (Fl::event_key('q')!=0) {
  }

  return return_val;
}


void MotionsViewer::time_estimator()
{
  double end_time = 0.0;
  if (!view_motions.empty()) {
	vector<double> times;
	transform(view_motions.begin(), view_motions.end(), back_inserter(times), 
			  [](ml::Motion &m){return m.last_posture().time;});
	end_time = *max_element(times.begin(), times.end());
  }
  set_bounds(0, end_time);
}
void MotionsViewer::add_motions( const vector<ml::Motion>& add_mots )
{
  copy(add_mots.begin(), add_mots.end(), back_inserter(view_motions));
}

void MotionsViewer::add_motions_and_reset_time( const vector<ml::Motion>& add_mots )
{
  add_motions(add_mots);
  time_estimator();
}

void MotionsViewer::set_color_inOrder( vector<ml::Motion> &m )
{
  double color_set[2][3] = {
	{178./ 255., 122. / 255., 180. / 255.},	
	{0.2, 0.5 ,0.2}
  };
  for (auto it = m.begin(); it != m.end(); ++it) {
	it->color = cml::vector3(color_set[(it - m.begin()) % 2]);
  }
}
