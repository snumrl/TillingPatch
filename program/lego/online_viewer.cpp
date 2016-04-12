#include "StdAfx.h"
#include "online_viewer.h"
#include "ccml.h"
#include "patch_area.h"
#include "screengrab.h"
#include "get_patch.h"
#include "tiling.h"
#include "patch_cooker.h"
#include "nonslipfoot.h"

const int kNumRepeat = 1;

void draw_se3_box_online(const cml::transf &se3, const cml::vector3 &l)
{
	glPushMatrix();
	glMultMatrixd(se3.data());
	glScaled(l[0], l[1], l[2]);
	glutSolidCube(1);
	glPopMatrix();
}

static void DrawPostureOnline(const ml::Posture &p, bool shadow, cml::vector3 color, Env & env = Env())
{
	int x_diff = 0;
	if (env.is_cyclic["x"])
		x_diff = kNumRepeat;
	int z_diff = 0;
	if (env.is_cyclic["z"])
		z_diff = kNumRepeat;

	int list_no = glGenLists(1);

	glNewList(list_no, GL_COMPILE_AND_EXECUTE);

	if (shadow == false) {
		if (color[0] < 0)
			glColor3d(0.1, 0.1, 0.1);
		else
			glColor3dv(color.data());
	}
	else 
		glColor4f(0,0,0,.15);

	for(int i = 1; i < (int)p.num_joint(); ++i) {
		cml::vector3d offset = p.body()->offset(i);
		cml::transf rot_se3 = cml::make_transf(cml::exp_mat3(cml::between_vector(cml::vector3d(1,0,0),offset)), cml::vector3d(0,0,0) );

		glPushMatrix();
		cml::transf se3 = p.GetGlobalTransf(p.body()->parent(i));
		glMultMatrixd(se3.data());
		glMultMatrixd(rot_se3.data());
		double length = offset.length();
		glScaled(length * 1.02, 0.04, 0.07);
		glTranslated(0.5, 0, 0);
		glutSolidCube(1);
		glPopMatrix();
	}

	if (p.object == 1)
	{
		if (shadow == false)
			glColor3f(0.6, 0.3, 0.6);
		else
			glColor4f(0,0,0,.15);

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

		draw_se3_box_online(tran, cml::vector3(0.6, 0.5, 0.4));
	}
	glEndList();

	for (int x_i = -x_diff; x_i <= x_diff; ++x_i)
	{
		for (int z_i = -z_diff; z_i <= z_diff; ++z_i)
		{
			if(x_i == 0 && z_i == 0) continue;
			double trans_x = (double)x_i*(env.range["x_max"] - env.range["x_min"]);
			double trans_z = (double)z_i*(env.range["z_max"] - env.range["z_min"]);

			glPushMatrix();
			glTranslated(trans_x, 0, trans_z);

			glCallList(list_no);

			glPopMatrix();
		}
	}
	glDeleteLists(list_no, 1);
}

PatchesViewerOnline::PatchesViewerOnline() 
  : Fl_Gl_Slider(1280-512+200, 100, 800, 600)
//PatchesViewer::PatchesViewer( const vector<shared_ptr<Patch>> &patches, Env &env_ /*= Env()*/ ) : Fl_Gl_Slider(1280-512, 100, 1280, 1024), env(env_), patches(patches_)
{
	dtable = new Danglings;
	get_camera().rotateX = -cml::pi() / 4. + 0.3;
	get_camera().distance += 0.8;	
	common_init();
}

void PatchesViewerOnline::common_init()
{
  //root_path = false;
  path_mode = 0;
  id_mode = 3;
  bound = true;
  change_bound();
  shadow = true;
  follow_character = -1;
  show_phase2 = 1;
  camera_control = kNoCameraControl;
  procedural_tog = true;
  get_naive_dangling_table(*dtable, si.patches, si.env);
  draw_cons = false;
  draw_contact = false;
  fl_gl_3d->is_draw_axis = false;
}

PatchesViewerOnline::~PatchesViewerOnline(void)
{
  delete dtable;
}

double color_set_online[9][3] = 
{
	{202. / 255., 84. / 255., 110. / 255.},
	{0., 0.4 ,1.0},
	{0.6, 0.6 ,0.0},
	{0.2, 0.5 ,0.2},
	{0. / 255., 136. / 255., 124. / 255.},
	{178./ 255., 122. / 255., 180. / 255.},
	{246. / 255., 139. / 255., 51. / 255.},
	{130. / 255., 0. / 255., 80. / 255.},
	{24. / 255., 148. / 255., 184. / 255.}
};

void PatchesViewerOnline::draw_main( int frame )
{
  si.tick(frame);

  //capture_image("test", frame);

  if (!follow_set.empty()) calc_camera();
  control_camera(frame);

  // motion shadow
  if (shadow == true) {
	setupShadow();
	glColor4f(0,0,0,.15);

	for (auto it_pa = si.patches.begin(); it_pa != si.patches.end(); ++it_pa) {
	  // do not draw patch added in phase 2
	  if (show_phase2 == 0) {
		bool is_phase1 = false;
		for (auto it = (*it_pa)->inner_cons.begin(); it != (*it_pa)->inner_cons.end(); ++it) {
		  if (it->type == "phase1") { 
			is_phase1 = true; 
			break; 
		  }
		}
		if (!is_phase1) continue;
	  }

	  for (int x_i = -kNumRepeat; x_i <= kNumRepeat; ++x_i) {
		for (int z_i = -kNumRepeat; z_i <= kNumRepeat; ++z_i) {
		  double trans_x = (double)x_i*(si.env.range["x_max"] - si.env.range["x_min"]);
		  double trans_z = (double)z_i*(si.env.range["z_max"] - si.env.range["z_min"]);

		  glPushMatrix();
		  glTranslated(trans_x, 0, trans_z);
		  draw_patch_object(it_pa->get(), frame, true);
		  glPopMatrix();
		}
	  }

	  const vector<ml::Motion> &motions = (*it_pa)->motions;
	  for (auto it = motions.begin(); it != motions.end(); ++it) {
		if (it->first_posture().time - 0.5 < frame && frame < it->last_posture().time + 0.5) {
		  cml::vector3d color = it->color;
		  auto p_posture = std::min_element(it->begin(), it->end(), [frame](const ml::Posture &p1, const ml::Posture &p2){return fabs(p1.time - frame) < fabs(p2.time - frame);});
		  DrawPostureOnline(*p_posture, true, color, si.env);
		}
	  }
	}

	for (auto it = si.env.static_boxes.begin(); it != si.env.static_boxes.end(); ++it) {
	  draw_se3_box_online(it->se3, it->length);
	}
	
	for (auto it = si.env.dynamic_boxes.begin(); it != si.env.dynamic_boxes.end(); ++it) {
	  if (it->begin()->time - 0.5 < frame && frame < (it->end()-1)->time + 0.5) {
		auto it_box = std::min_element(it->begin(), it->end(), [frame](const Box &b1, const Box &b2){return fabs(b1.time - frame) < fabs(b2.time - frame);});
		draw_se3_box_online(it_box->se3, it_box->length);
	  }
	}
	unsetupShadow();
  }

  // motion
  for (auto it_pa = si.patches.begin(); it_pa != si.patches.end(); ++it_pa) {
	// do not draw patch added in phase 2
	if (show_phase2 == 0) {
	  bool is_phase1 = false;
	  for(auto it = (*it_pa)->inner_cons.begin(); it != (*it_pa)->inner_cons.end(); ++it) {
		if(it->type == "phase1") { 
		  is_phase1 = true; 
		  break; 
		}
	  }
	  if(!is_phase1) continue;
	}

	for (int x_i = -kNumRepeat; x_i <= kNumRepeat; ++x_i) {
	  for (int z_i = -kNumRepeat; z_i <= kNumRepeat; ++z_i) {
		double trans_x = (double)x_i*(si.env.range["x_max"] - si.env.range["x_min"]);
		double trans_z = (double)z_i*(si.env.range["z_max"] - si.env.range["z_min"]);

		glPushMatrix();
		glTranslated(trans_x, 0, trans_z);
		draw_patch_object(it_pa->get(), frame, false);
		glPopMatrix();
	  }
	}

	vector<ml::Motion> &motions = (*it_pa)->motions;
	for (auto it = motions.begin(); it != motions.end(); ++it) {
	  auto p_posture = std::min_element(it->begin(), it->end(), [frame](const ml::Posture &p1, const ml::Posture &p2){return fabs(p1.time - frame) < fabs(p2.time - frame);});
	  
	  if (it->first_posture().time-0.5 < frame && frame < it->last_posture().time + 0.5) {
		cml::vector3d color = it->color;

		if(!follow_set.empty()) {
		  if(follow_set.count(&motions[it - motions.begin()]) == 0) {
			color[0] = 0.3;
			color[1] = 0.3;
			color[2] = 0.3;
		  } else {			
			next_follow_position = p_posture->trans();
		  }
		}
		if(show_phase2 == 2) {
		  bool is_phase1 = false;
		  for(auto it = (*it_pa)->inner_cons.begin(); it != (*it_pa)->inner_cons.end(); ++it) {
			if(it->type == "phase1") { 
				is_phase1 = true;	break; 
			}
		  }
		  if(is_phase1) {
			double gcolor = 0.3 * color[0] + 0.59 * color[1] + 0.11 * color[2];
			color[0] = color[1] = color[2] = gcolor;
		  }
		}

        DrawPostureOnline(*p_posture, false, color, si.env);
		
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
		  obb obb_ = bound_obb_posture(*p_posture);
		  draw_se3_box_online(obb_.transf_m, cml::vector3(obb_.lx, obb_.ly, obb_.lz));
		}
		
		if (path_mode == 2) {
		  if (it->first_posture().time - 0.5 < frame && it->last_posture().time + 0.5 > frame) {
			glBegin(GL_LINE_STRIP);
			glColor3d(0,0,0);
			for (auto it_p = it->begin(); it_p != it->end(); ++it_p) {
			  cml::vector3d position = it_p->trans();
			  if (it_p->status.is_highlight) {
				glEnd();
				glDisable(GL_LINE_STRIP);
				glDisable(GL_LIGHTING);
			
				glColor3d(it_p->status.color[0], it_p->status.color[1], it_p->status.color[2]);						
				glBegin(GL_POINTS);		  
				glVertex3d(position[0], position[1], position[2]);
				/*glPushMatrix();
				glTranslated( position[0], position[1], position[2]);
				glutSolidSphere( 0.08, 40, 40);
				glPopMatrix();*/
				glEnd();
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
	  }

	  if (path_mode == 1) {
		glBegin(GL_LINE_STRIP);
		glColor3d(0,0,0);
		for (auto it_p = it->begin(); it_p != it->end(); ++it_p) {
		  cml::vector3d position = it_p->trans();
		  if (it_p->status.is_highlight) {
			glEnd();
			glDisable(GL_LINE_STRIP);
			glDisable(GL_LIGHTING);

			glColor3d(it_p->status.color[0], it_p->status.color[1], it_p->status.color[2]);						
			glBegin(GL_POINTS);		  
			glVertex3d(position[0], position[1], position[2]);
			/*glPushMatrix();
			glTranslated( position[0], position[1], position[2]);
			glutSolidSphere( 0.08, 40, 40);
			glPopMatrix();*/
			glEnd();
			glEnable(GL_LIGHTING);
			glColor3d(0.0, 0.0, 0.0);
			glBegin(GL_LINE_STRIP);
		  } else {					
			glVertex3d(position[0], position[1], position[2]);
		  }
		}
		glEnd();
				
					
		for (auto it_b = (*it_pa)->boundaries.begin(); it_b != (*it_pa)->boundaries.end(); ++it_b) {
		  //glPointSize(19);
		//  //glBegin(GL_POINTS);

		// /* string end_type = get_end_type(*it_b, si.env.range["t_min"], si.env.range["t_max"]);
  //        cout << end_type << ' ' << it_b->posture().trans() << endl;
		//  if (end_type == "connect") {
		//	glColor3d(0,0,1);
		//  } else if (end_type == "boundary") {
		//	glColor3d(0,1,0);
		//  } else if (end_type == "dead") {
		//	glColor3d(0,1,1);
		//  } else {
		//	ASSERT(false);
		//  }	*/
		//  //glVertex3dv(it_b->posture().trans().data());
		//  //glEnd();
		} 
	  }	
	}
  }
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  // Environment
  //
  /*glEnable(GL_BLEND);
  glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
  glEnable(GL_LIGHTING);*/

  glColor3d(0.2, 0.4, 0.7);
  for (auto it = si.env.static_boxes.begin(); it != si.env.static_boxes.end(); ++it)
	draw_se3_box_online(it->se3, it->length);

  for (auto it = si.env.dynamic_boxes.begin(); it != si.env.dynamic_boxes.end(); ++it) {
	if (it->begin()->time - 0.5 < frame && frame < (it->end()-1)->time + 0.5) {
	  auto it_box = std::min_element(it->begin(), it->end(), [frame](const Box &b1, const Box &b2){return fabs(b1.time - frame) < fabs(b2.time - frame);});
	  draw_se3_box_online(it_box->se3, it_box->length);
	}
  }
	/*glBegin(GL_LINE_STRIP);
  glDisable(GL_LIGHTING);
  glVertex3d( env.get_x_min(), 0.5, env.get_z_min());
  glVertex3d( env.get_x_min(), 0.5, env.get_z_max());
  glVertex3d( env.get_x_max(), 0.5, env.get_z_max());
  glVertex3d( env.get_x_max(), 0.5, env.get_z_min());
  glVertex3d( env.get_x_min(), 0.5, env.get_z_min());
	glEnd();*/

  /*glDisable(GL_LIGHTING);
  glDisable(GL_BLEND);*/

  glDisable(GL_LIGHTING);
  draw_patch_area(frame);

  //patch id
  if (id_mode == 1 || id_mode == 2) {
	glLineWidth(2.5);
	for (int i = 0; i < si.patches.size(); ++i) {
	  for (int j = 0; j < si.patches[i]->motions.size(); ++j) {
		const ml::Motion &m = si.patches[i]->motions[j];

		if (id_mode == 1 || (id_mode == 2 && m.first_posture().time - 0.5 < frame && frame < m.last_posture().time + 0.5)) {	
		  cml::vector3 pos;
		  {
			const ml::Posture & p = *std::min_element(m.begin(), m.end(), [frame](const ml::Posture &p1, const ml::Posture &p2){return fabs(p1.time - frame) < fabs(p2.time - frame);});
			pos = p.trans() + cml::vector3(0,0.7,0);
		  }
		  glColor3dv(m.color.data());
		  double * col = color_set_online[j];
		  glColor3d(col[0], col[1], col[2]);

		  glPushMatrix();
		  glTranslated(pos[0] - 0.15, pos[1], pos[2]);
		  glScaled(0.002, 0.002, 0.002);
					
		  std::string str = si.patches[i]->name;
		  char * name = new char[str.size() + 1];
		  strcpy(name, str.c_str());
		  //itoa(i, string, 10);			
		  for (char * c=name; *c != '\0'; c++) {
			glutStrokeCharacter(GLUT_STROKE_ROMAN, *c);
		  }
		  delete name;
		  glPopMatrix();
		}
	  }
	}
	glLineWidth(1);
  }
  glEnable(GL_LIGHTING);

  
  if (draw_cons) {
    for (size_t i = 0; i < si.patches.size(); ++i) {
      for (auto it = si.patches[i]->rel_pos_time.begin(); it != si.patches[i]->rel_pos_time.end(); ++it) {
        size_t m[2] = {it->get_motion_idx(0), it->get_motion_idx(1)};
        size_t s[2] = {it->get_section_beg(0), it->get_section_beg(1)};		  
        glColor3d(0.9, 0.1, 0.1);

        glBegin(GL_LINES);
        cml::vector3 r1 = si.patches[i]->motions[m[0]][s[0]].trans();		  
        cml::vector3 r2 = si.patches[i]->motions[m[1]][s[1]].trans();
        glVertex3d(r1[0], r1[1], r1[2]);
        glVertex3d(r2[0], r2[1], r2[2]);		  
        glEnd();
      }
    }
  }

  //path 목표
  glColor3d(1,0,0);
  glPushMatrix();
  glTranslated(si.target[0],si.target[1],si.target[2]);
  glutSolidSphere(0.25, 17,17);
  glPopMatrix();
  glColor3d(0,0,1);
  glPushMatrix();
  glTranslated(si.target1[0],si.target1[1],si.target1[2]);
  glutSolidSphere(0.25, 17,17);
  glPopMatrix();
  /*
  if(image_name != "") {
	  char buf[1024];
	  sprintf_s(buf, "%s_%d.bmp", image_name.c_str(), frame);
	  SaveScreenGrab(buf);
  }
  */
}

void un_project(int mouse_x, int h_minus_mouse_y, cml::vector3 &line_p, cml::vector3 &line_v)
{
  GLdouble mvmatrix[16], projmatrix[16];
  GLint viewport[4];

  glGetDoublev(GL_MODELVIEW_MATRIX, mvmatrix);
  glGetDoublev(GL_PROJECTION_MATRIX, projmatrix);
  glGetIntegerv(GL_VIEWPORT, viewport);

  GLdouble wx, wy, wz;
  gluUnProject((GLdouble)mouse_x, (GLdouble)h_minus_mouse_y, 0.25, mvmatrix, projmatrix, viewport, &wx, &wy, &wz);
  cml::vector3 p1(wx, wy, wz);
  gluUnProject((GLdouble)mouse_x, (GLdouble)h_minus_mouse_y, 0.75, mvmatrix, projmatrix, viewport, &wx, &wy, &wz);
  cml::vector3 p2(wx, wy, wz);

  line_p = p1;
  line_v = p2 - p1;
}

int PatchesViewerOnline::handle_main( int event )
{
  int return_val = 0;
  if (Fl::event_key('r') != 0) {
	cout << 'r' << endl;
	return_val = 1;
  } else if (Fl::event_key('q') != 0) {
	procedural_tog = !procedural_tog;
	return_val = 1;
  } else if (Fl::event_key('t') != 0) {
	++path_mode;
	if (path_mode > 2) path_mode = 0;
	//root_path = !root_path;	
	return_val = 1;
  } else if (Fl::event_key('d') != 0) {
	id_mode++;
	if (id_mode > 3) id_mode = 1;
	return_val = 1;
  } else if (Fl::event_key('c') != 0) {
	calc_patch_area(true);
	return_val = 1;
  } else if (Fl::event_key('b') != 0) {
	bound = !bound;
	change_bound();
	return_val = 1;
  } else if (Fl::event_key('s') != 0) {
	shadow = !shadow;
	return_val = 1;
  } else if (Fl::event_key('n') != 0) {
	++follow_character;
	calc_follow_character();
	return_val = 1;
  } else if (Fl::event_key('p') != 0) {
	--follow_character;
	calc_follow_character();
	return_val = 1;
  } else if (Fl::event_key('m') != 0) {
	follow_set.clear();
	return_val = 1;
  } else if (Fl::event_key('x') != 0) {
	fl_gl_3d->is_draw_axis = !fl_gl_3d->is_draw_axis;
	return_val = 1;
  } else if (Fl::event_key('z') != 0) {
	fl_gl_3d->is_draw_ground = !fl_gl_3d->is_draw_ground;
	return_val = 1;
  } else if (Fl::event_key('f') != 0) {
	fullscreen();
	return_val = 1;
  } else if (Fl::event_key('v') != 0) {
	SaveScreenGrab("capture.bmp");
	return_val = 1;
  } else if (Fl::event_key('1') != 0) {
	camera_control = (CameraControl)(camera_control + 1);
	if(camera_control >= kLastCameraControl) camera_control = kNoCameraControl;
	return_val = 1;
  } else if (Fl::event_key('2') != 0) {
	show_phase2 = (show_phase2 + 1) % 3;
	return_val = 1;
  } else if (Fl::event_key('i') != 0) {
	draw_cons = !draw_cons;
	return_val = 1;
  } else if (Fl::event_key('g') != 0) {
	draw_contact = !draw_contact;
	return_val = 1;
  }

  
  if (event == FL_PUSH && (Fl::event_ctrl() != 0 || Fl::event_shift() != 0))
  {
    int pushButton = Fl::event_button();
    int mouseX = Fl::event_x();
    int mouseY = Fl::event_y();
  
    cml::vector3 line_p, line_v;
    un_project(mouseX, fl_gl_3d->h() - mouseY, line_p, line_v);
    // p + v * t = (x, height , z)
    // t = (height - py) / vy
    double height = 0.0;
    cml::vector3 new_position = (height - line_p[1]) / line_v[1] * line_v + line_p;
    if (pushButton == 1 && Fl::event_ctrl() != 0) {
      si.target = new_position;
    }
    if (pushButton == 1 && Fl::event_shift() != 0) {
       si.target1 = new_position;
    }
    return_val = 1;
  }

  return return_val;
}

void PatchesViewerOnline::change_bound()
{
	if (si.patches.empty() || bound==true) {
	  set_bounds(si.env.range["t_min"], si.env.range["t_max"]);
	} else {
	  set_bounds(begin_time(si.patches), end_time(si.patches));
	}
}

void PatchesViewerOnline::calc_characters()
{
	for(int i = 0 ; i < si.patches.size(); ++i) {
		for(int j = 0; j < si.patches[i]->motions.size(); ++j) {
			si.patches[i]->motions[j].checked = false;
		}
	}

	characters.clear();
	for(int i = 0; i < si.patches.size(); ++i) {
		for(int j = 0; j < si.patches[i]->motions.size(); ++j) {
			if(si.patches[i]->motions[j].checked == false) {
				Character c;
				c.num_patches = 0;

				ml::Motion *cur_motion = &si.patches[i]->motions[j];
				while(1) {
					ml::Motion *next_motion = cur_motion->connected_back;
					if(!next_motion) break;
					cur_motion = next_motion;
				}
				while(cur_motion) {
					c.motions.push_back(cur_motion);
					c.num_patches++;
					cur_motion->checked = true;
					cur_motion = cur_motion->connected_fore;
				}
				characters.push_back(c);
			}
		}
	}
	std::sort(characters.begin(), characters.end(), [](const Character&c1, const Character&c2){ return c1.num_patches > c2.num_patches;} );
}

void PatchesViewerOnline::calc_follow_character()
{
	calc_characters();
	if(follow_character >= (int)characters.size()) {
		follow_character = 0;
	}
	else if(follow_character < 0) {
		follow_character = characters.size()-1;
	}

	follow_set.clear();

	for(int i = 0; i < characters[follow_character].motions.size(); ++i) {
		follow_set.insert(characters[follow_character].motions[i]);
	}
}

void PatchesViewerOnline::calc_patch_area(bool full_body)
{
  if (!patch_areas.empty()) {
	for (int i = 0; i < (int)patch_areas.size(); ++i) delete patch_areas[i];
	patch_areas.clear();
	return;
  }

  for (int i = 0; i < si.patches.size(); ++i) {
	const std::shared_ptr<Patch>& p = si.patches[i];
	PatchArea *area = new PatchArea();
	area->r = p->motions[0].color[0];
	area->g = p->motions[0].color[1];
	area->b = p->motions[0].color[2];

	for (int j = 0; j < p->motions.size(); ++j) {
	  for (int k = 0; k < p->motions[j].size(); ++k) {
		ml::Posture &posture = p->motions[j][k];
		area->points.push_back( cml::vec2(posture.trans()) );
		if(full_body) {
		  for(int ii = 1; ii < posture.body()->num_joint(); ++ii) {
			area->points.push_back( cml::vec2(posture.GetGlobalTranslation(ii)));
		  }
		}
	  }
	}
	cml::ConvexHull(area->points);
	cml::ConvexHull2(area->points);
	cml::ConvexHull3(area->points);
	area->center = cml::vector2(0., 0.);
	for each(cml::vector2 point in area->points) {
	  area->center += point;
	}
	area->center /= (double)area->points.size();
	patch_areas.push_back(area);
  }
}

void PatchesViewerOnline::draw_patch_area( int frame )
{
	glPushAttrib(GL_COLOR_BUFFER_BIT);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	if(camera_control == kContrlExample) {
		double alpha = 0;

		if(frame >= 500 && frame < 530) alpha = 0.2 - (530 - frame) / 150.;
		else if (frame >= 530 && frame < 560) alpha = (560 - frame) / 150.;
		else if (frame >= 560 && frame < 590) alpha = 0.2 - (590 - frame) / 150.;
		else if (frame >= 590 && frame < 620) alpha = (620 - frame) / 150.;
		else if (frame >= 620 && frame < 650) alpha = 0.2 - (650 - frame) / 150.;
		else if (frame >= 650 && frame < 680) alpha = (680 - frame) / 150.;
		else alpha = 0;

		if(alpha > 0) {
			glColor4d(0.7921568, 0.329412, 0.431373, alpha); 
			glBegin(GL_QUADS);
			glVertex3d(-5, 0.02, -5);
			glVertex3d(-5, 0.02, 5);
			glVertex3d(5, 0.02, 5);
			glVertex3d(5, 0.02, -5);
			glEnd();
		}
		if(frame >= 500 && frame <= 680) {
			double size_x = (si.env.range["x_max"] - si.env.range["x_min"]);
			double size_z = (si.env.range["z_max"] - si.env.range["z_min"]);

			if(frame >= 530 && frame <= 680) alpha = 0.2;
			glColor4d(0, 0, 0, alpha * 5);
			glBegin(GL_LINES);
			glLineWidth(3.0);
	
			for(int i = -kNumRepeat; i <= kNumRepeat+1; ++i) {
				glVertex3d(-kNumRepeat*10.0 - 5.0, 0.05, i * 10.0 - 5.0);
				glVertex3d(kNumRepeat*10.0 + 5.0, 0.05, i * 10.0 - 5.0);

				glVertex3d(i * 10.0 - 5.0, 0.05, -kNumRepeat * 10.0 - 5.0);
				glVertex3d(i * 10.0 - 5.0, 0.05, kNumRepeat * 10.0 + 5.0);
			}

			glEnd();

		}
	}

	for(int i = 0; i < (int)patch_areas.size(); ++i) {
		if(si.patches[i]->get_begin_time() > frame || si.patches[i]->get_last_time() < frame) continue;
		//// do not draw patch added in phase 2
		//if(show_phase2 == 0) {
		//	bool is_phase1 = false;
		//	for(auto it = si.patches[i]->inner_cons.begin(); it != si.patches[i]->inner_cons.end(); ++it) {
		//		if(it->type == "phase1") { is_phase1 = true; break; }
		//	}
		//	if(!is_phase1) continue;
		//}

		if(si.patches[i]->motions.size() <= 1) continue;
		glColor4d(patch_areas[i]->r, patch_areas[i]->g, patch_areas[i]->b, 0.2);
		glBegin(GL_TRIANGLE_FAN);
		cml::vector3 cp = cml::vec3(patch_areas[i]->center);
		cp[1] = 0.05 + i * 0.001;
		glVertex3dv(cp.data());
		for(int j = 0; j < patch_areas[i]->points.size(); ++j) {
			cml::vector3 p = cml::vec3(patch_areas[i]->points[j]);
			p[1] = 0.05 + i * 0.001;
			glVertex3dv(p.data());
		}
		cml::vector3 p = cml::vec3(patch_areas[i]->points[0]);
		p[1] = 0.05 + i * 0.001;
		glVertex3dv(p.data());
		glEnd();
		glLineWidth(2.0);
		glColor4d(patch_areas[i]->r, patch_areas[i]->g, patch_areas[i]->b, 0.6);
		glBegin(GL_LINE_LOOP);
		for(int j = 0; j < patch_areas[i]->points.size(); ++j) {
			cml::vector3 p = cml::vec3(patch_areas[i]->points[j]);
			p[1] = 0.05 + i * 0.001;
			glVertex3dv(p.data());
		}
		glEnd();
	}
	glPopAttrib();
}

void PatchesViewerOnline::calc_camera()
{
	Camera& cam = get_camera();

	cml::vector3d prev_position(cam.centerX, cam.centerY, cam.centerZ);

	prev_position = prev_position * 0.8 + next_follow_position * 0.2;

	cam.centerX = prev_position[0];
	cam.centerY = prev_position[1];
	cam.centerZ = prev_position[2];
}

void PatchesViewerOnline::set_record_mode()
{
	fl_gl_3d->is_draw_axis = false;
	fl_gl_3d->is_draw_ground = false;

	shadow = false;
	//root_path = true;
}

void PatchesViewerOnline::draw_patch_object( Patch *pa, int time, bool is_shadow )
{
	PatchObject& obj = pa->patch_object;

	if(obj.objectType == PatchObject::kPatchBox) {
		ml::Motion &begin_motion = pa->motions[obj.begin_motion];
		ml::Motion &end_motion = pa->motions[obj.end_motion];

		int f1,f2;

		for(f1 = obj.begin_frame - 1; f1 < (int)begin_motion.size() - 1; ++f1) {
			if(begin_motion[f1].object != false && begin_motion[f1+1].object == false ) break;
		}
		for(f2 = obj.end_frame - 1; f2 < (int)end_motion.size()-1; ++f2) {
			if(end_motion[f2-1].object == false && end_motion[f2].object != false) break;
		}

		if(time < begin_motion[f1].time) return;
		if(time > end_motion[f2].time) return;

		ml::Posture &p1 = begin_motion[f1];
		cml::vector3d l1 = p1.GetGlobalTranslation(p1.body()->joint_index("LeftHandDummy"));
		cml::vector3d r1 = p1.GetGlobalTranslation(p1.body()->joint_index("RightHandDummy"));
		l1[1] += begin_motion[f1].object_height_correction;
		r1[1] += begin_motion[f1].object_height_correction;

		ml::Posture &p2 = end_motion[f2];
		cml::vector3d l2 = p2.GetGlobalTranslation(p2.body()->joint_index("RightHandDummy"));
		cml::vector3d r2 = p2.GetGlobalTranslation(p2.body()->joint_index("LeftHandDummy"));
		l2[1] += end_motion[f2].object_height_correction;
		r2[1] += end_motion[f2].object_height_correction;

		// l2, r2 correction
		{
			double theta_2;
			{
				cml::vector3d v = l2 - r2;
				v[1] = 0;
				v = v.normalize();
				theta_2 = -acos(v[0]);
				if(v[2] < 0) theta_2 = -theta_2;
			}
			cml::matrix3 rot_m = cml::roty_mat(theta_2);
			cml::vector3d diff_v = rot_m * cml::vector3d(0, 0, -0.08);
			l2 += diff_v;
			r2 += diff_v;
		}

		double b_time = begin_motion[f1].time;
		double e_time = end_motion[f2].time;

		cml::vector3d lmid = (l1 + l2) / 2.;
		cml::vector3d rmid = (r1 + r2) / 2.;
		cml::vector3d lh, rh;

		if(time <= b_time + obj.range) {
			double t = cml::SpeedFunc((time - b_time) / (double)obj.range);
			lh = t * lmid + (1. - t) * l1;
			rh = t * rmid + (1. - t) * r1;
		}
		else if(time >= e_time - obj.range) {
			double t = cml::SpeedFunc((e_time - time) / (double)obj.range);
			lh = t * lmid + (1. - t) * l2;
			rh = t * rmid + (1. - t) * r2;
		}
		else {
			lh = lmid;
			rh = rmid;
		}

		double theta;
		{
			cml::vector3d v = lh - rh;
			v[1] = 0;
			v = v.normalize();
			theta = -acos(v[0]);
			if(v[2] < 0) theta = -theta;
		}

		cml::matrix3 rot_m = cml::roty_mat(theta);
		cml::vector3d diff_v = rot_m * cml::vector3d(0, 0, 0.04);
		cml::vector3d object_trans = ((lh+rh) / 2) + diff_v;
		object_trans[1] -= 0.1;
		// object_trans[1] += p1.object_height_correction;

		glPushMatrix();
		cml::transf tran = cml::trans_transf(object_trans) * cml::make_transf(rot_m, cml::vector3d(0,0,0));

		if (is_shadow == false) {
			glColor3d(0.6, 0.3, 0.6);
		}

		glMultMatrixd(tran.data());
		glScaled(0.6, 0.5, 0.4);
		glutSolidCube(1.0);
		glPopMatrix();
	}
	else if(obj.objectType == PatchObject::kBoxBlending) {
		ml::Motion &begin_motion = pa->motions[obj.begin_motion];
		ml::Motion &end_motion = pa->motions[obj.end_motion];

		double contact_time1 = begin_motion[obj.begin_frame].time;
		double contact_time2 = end_motion[obj.end_frame].time;

		double begin_time = (begin_motion[obj.begin_frame - obj.range].time + begin_motion[obj.begin_frame - obj.range + 1].time) / 2.;
		double end_time = (end_motion[obj.end_frame + obj.range].time + end_motion[obj.end_frame + obj.range - 1].time) / 2.;

		if( time < begin_time || time > end_time) return;

		ml::Posture &p1 = begin_motion[obj.begin_frame];
		cml::vector3d l1 = p1.GetGlobalTranslation(p1.body()->joint_index("LeftHandDummy"));
		cml::vector3d r1 = p1.GetGlobalTranslation(p1.body()->joint_index("RightHandDummy"));
		l1[1] += begin_motion[obj.begin_frame].object_height_correction;
		r1[1] += begin_motion[obj.begin_frame].object_height_correction;

		ml::Posture &p2 = end_motion[obj.end_frame];
		cml::vector3d l2 = p2.GetGlobalTranslation(p2.body()->joint_index("RightHandDummy"));
		cml::vector3d r2 = p2.GetGlobalTranslation(p2.body()->joint_index("LeftHandDummy"));
		l2[1] += end_motion[obj.end_frame].object_height_correction;
		r2[1] += end_motion[obj.end_frame].object_height_correction;

		// l1, r1 correction
		{
			double theta_1;
			{
				cml::vector3d v = l1 - r1;
				v[1] = 0;
				v = v.normalize();
				theta_1 = -acos(v[0]);
				if(v[2] < 0) theta_1 = -theta_1;
			}
			cml::matrix3 rot_m = cml::roty_mat(theta_1);
			cml::vector3d diff_v = rot_m * cml::vector3d(0, 0, 0.04);
			l1 += diff_v;
			r1 += diff_v;
		}

		// l2, r2 correction
		{
			double theta_2;
			{
				cml::vector3d v = l2 - r2;
				v[1] = 0;
				v = v.normalize();
				theta_2 = -acos(v[0]);
				if(v[2] < 0) theta_2 = -theta_2;
			}
			cml::matrix3 rot_m = cml::roty_mat(theta_2);
			cml::vector3d diff_v = rot_m * cml::vector3d(0, 0, -0.04);
			l2 += diff_v;
			r2 += diff_v;
		}

		cml::vector3d ldelta = l2 - l1;
		cml::vector3d rdelta = r2 - r1;

		cml::vector3d lmid, rmid;

		if(begin_motion[obj.begin_frame].time >= time) {
			auto p_posture = std::min_element(begin_motion.begin() + obj.begin_frame - obj.range, begin_motion.begin() + obj.begin_frame, [time](const ml::Posture &p1, const ml::Posture &p2) { return fabs(p1.time - time) < fabs(p2.time - time); });
			double ratio = 1.0 - (contact_time1 - p_posture->time) / (contact_time1 - begin_time);
			lmid = p_posture->GetGlobalTranslation(p_posture->body()->joint_index("LeftHandDummy")) + (ratio * .5) * ldelta;
			rmid = p_posture->GetGlobalTranslation(p_posture->body()->joint_index("RightHandDummy")) + (ratio * .5) * rdelta;
			lmid[1] += p_posture->object_height_correction;
			rmid[1] += p_posture->object_height_correction;
		}
		else {
			auto p_posture = std::min_element(end_motion.begin() + obj.end_frame, end_motion.begin() + obj.end_frame + obj.end_frame, [time](const ml::Posture &p1, const ml::Posture &p2) { return fabs(p1.time - time) < fabs(p2.time - time); });
			double ratio = 1.0 - (p_posture->time - contact_time2) / (end_time - contact_time2);
			lmid = p_posture->GetGlobalTranslation(p_posture->body()->joint_index("LeftHandDummy")) + (ratio * -.5) * rdelta;
			rmid = p_posture->GetGlobalTranslation(p_posture->body()->joint_index("RightHandDummy")) + (ratio * -.5) * ldelta;
			lmid[1] += p_posture->object_height_correction;
			rmid[1] += p_posture->object_height_correction;
		}

		double theta;
		{
			cml::vector3d v = lmid - rmid;
			v[1] = 0;
			v = v.normalize();
			theta = -acos(v[0]);
			if(v[2] < 0) theta = -theta;
		}

		cml::matrix3 rot_m = cml::roty_mat(theta);
		cml::vector3d diff_v = rot_m * cml::vector3d(0, 0, 0.04);
		cml::vector3d object_trans = ((lmid+rmid) / 2) + diff_v;
		object_trans[1] -= 0.1;
		// object_trans[1] += p1.object_height_correction;

		glPushMatrix();
		cml::transf tran = cml::trans_transf(object_trans) * cml::make_transf(rot_m, cml::vector3d(0,0,0));

		if (is_shadow == false) {
			glColor3d(0.6, 0.3, 0.6);
		}

		glMultMatrixd(tran.data());
		glScaled(0.6, 0.5, 0.4);
		glutSolidCube(1.0);
		glPopMatrix();
	}
	else if(obj.objectType == PatchObject::kPrevBox) {
		for(int i = 0; i < pa->motions.size(); ++i) {
			int frame = -1;
			for(int j = 0; j < pa->motions[i].size(); ++j) {
				if(pa->motions[i].posture(j).object) {frame = j; break;}
			}
			if (frame == -1) continue;
			double contact_time = (pa->motions[i].posture(frame).time + pa->motions[i].posture(frame + 1).time) / 2.;
			if(time >= contact_time) continue;

			ml::Posture &p = pa->motions[i].posture(frame);

			if (is_shadow == false)
				glColor3f(0.6, 0.3, 0.6);
			else
				glColor4f(0,0,0,.15);

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

			draw_se3_box_online(tran, cml::vector3(0.6, 0.5, 0.4));
		}
	}
}

void PatchesViewerOnline::control_camera( int frame )
{
	Camera &cam = get_camera();

	if(camera_control == kNoCameraControl) return;
	else if(camera_control == kContrlExample) {
		if(99 <= frame && frame <= 299) {
			cam.centerX = 0.0;
			cam.centerY = 0.0;
			cam.centerZ = 0.0;
			cam.rotateY = cml::deg2rad(0.0);
			cam.rotateX = cml::deg2rad(-15.0 - (frame - 99) * .3);
			cam.distance = 7.0 + (frame - 99) * .05;
		}
		else if(300 <= frame && frame <= 499) {
			cam.centerX = 0.0;
			cam.centerY = 0.0;
			cam.centerZ = 0.0;
			cam.rotateY = cml::deg2rad(0.0);
			cam.rotateX = cml::deg2rad(-75.0 - (frame - 299) * .075);
			cam.distance = 17.0 + (frame - 299) * .05;
		}
		else if(500 <= frame) {
			cam.centerX = 0.0;
			cam.centerY = 0.0;
			cam.centerZ = 0.0;
			cam.rotateY = cml::deg2rad(0.0);
			cam.rotateX = cml::deg2rad(-90.0);
			cam.distance = 27.0 + (frame - 499) * .05;
		}
		return;
	}
}

#include "screengrab.h"
void PatchesViewerOnline::capture_image( const char *fileName, int index )
{
  char buf[1024];
  sprintf_s(buf, 1024, "data\\capture\\%s_%06d.bmp",fileName, index);
  SaveScreenGrab(buf);
}

Situation::Situation()
{
  init();
}

void Situation::init()
{
  target = cml::vector3(-4,0,0);
  target1 = cml::vector3(4,0,0);
  read_patches(input_patches, "hand_chicken");
  remove_dangling_patch(input_patches);

  for (auto it = input_patches.begin(); it != input_patches.end(); ++it) {
    patch_types[(*it)->name] = (**it);
    bool is_connectable = true; 
    for (auto jt = (*it)->boundaries.begin(); jt != (*it)->boundaries.end(); ++jt) {
      if (jt->posture_type() == -1) is_connectable = false;	  
    }
    if (is_connectable) sampling_patches.push_back((*it)->name);
  }

//patch_type_unary
  patch_type_unary = new vector<Patch>();
  for (auto it = patch_types.begin(); it != patch_types.end(); ++it) {
    const Patch &pa = it->second;
    if (pa.motions.size() == 1) { // && it->first == "pass_mid_to_mid_413_452") {
      patch_type_unary->push_back(pa);
    }
  }


  srand(79797+16+3+7);
  //environment
  env.set_scatter_range(-5,5,-5,5, 100, 10000);
  env.static_boxes.push_back(Box(trans_transf(cml::vector3(0, 0.625, -6.5)), cml::vector3(14, 1.25, 1)));
  env.static_boxes.push_back(Box(trans_transf(cml::vector3(0, 0.625, 6.5)), cml::vector3(14, 1.25, 1)));
  env.static_boxes.push_back(Box(trans_transf(cml::vector3(-6.5, 0.625, 0)), cml::vector3(1, 1.25, 14)));
  env.static_boxes.push_back(Box(trans_transf(cml::vector3(6.5, 0.625, 0)), cml::vector3(1, 1.25, 14)));		

  srand(79797+5+2);

  const int team_num = 4;
  
  int total_num = 0;
  while(true)
  {
   
    random_shuffle(sampling_patches.begin(), sampling_patches.end());
    while(true)
    {
      for (auto it_p = sampling_patches.begin(); it_p != sampling_patches.end(); ++it_p)
      {
        shared_ptr<Patch> pa1(new Patch((patch_types.find(*it_p))->second));
        if (pa1->motions.size() != 1)
          continue;

        if (total_num < team_num)
          env.scatter_range(*pa1, 3, 3+0.1, -3, 3, 99, 101);
        else
          env.scatter_range(*pa1, -3, -3+0.1, -3, 3, 99, 101);

        
        //env.scatter_range(*pa1, -1, 1, -1, 1, 99, 101);
        vector<ConnectInfo> connect_infos;
        get_connect_infos(connect_infos, pa1, patches, 0.01, env);

        if (stitch(patches, pa1, &patch_types, connect_infos, env) == true) 
        {
          //color
          for (auto it_m = pa1->motions.begin(); it_m != pa1->motions.end(); ++it_m)
          {
            if (total_num < team_num)
              it_m->color = cml::vector3(202. / 255., 84. / 255., 110. / 255.);
            else
              it_m->color = cml::vector3(0., 0.4 ,1.0);
          }
                    
          ++total_num;
          goto one_tile;
        }
      }
    }
one_tile:;

    cout << "total_num: " << total_num << endl;
    if (total_num >= 2*team_num)
      break;
  }

  //set_color_one(patches);
}

void Situation::tick( int time )
{
  //cout << time << endl;
  //while(true)
  {
    int min_time = 10000000;
    pair<int, int> min_pair(-1,-1);

    for (int pa2_i = 0; pa2_i < patches.size(); ++pa2_i) {
      for (int b2_i = 0; b2_i < patches[pa2_i]->boundaries.size(); ++b2_i) {
        string end_type = get_end_type(patches[pa2_i]->boundaries[b2_i], env.range["t_min"], env.range["t_max"]);
        if (end_type == "dead") {
          double cur_time = patches[pa2_i]->boundaries[b2_i].posture().time;
          if (cur_time < min_time) {
            min_pair = pair<int,int>(pa2_i, b2_i);
            min_time = cur_time;
          }
        }
      }
    }

    map<string, Patch> * patch_type = &patch_types;
    
   // cout << min_pair.first << ' ' << min_pair.second << ' ' << min_time << ' ' << time << endl;
    if (min_pair.first != -1 && min_time - time < 60)
    {
      vector<Stitch_energy> energies;
      get_energies_path_target(sampling_patches, patch_type, min_pair.first, min_pair.second, patches, env, energies, target, target1);
      sort(energies.begin(), energies.end(), [](const Stitch_energy &a, const Stitch_energy &b){return a.energy > b.energy;});

      const auto it_b2 = &(patches[min_pair.first]->boundaries[min_pair.second]);
      const auto &p2 = it_b2->posture();
      for (auto it = energies.begin(); it != energies.end(); ++it)
      {
        if (it->energy < 0.)
          continue;

        shared_ptr<Patch> pa1(new Patch((patch_type->find(it->name))->second));
        const auto &p1 = pa1->boundaries[it->b].posture();
        pa1->transform_between_posture(p2, p1);
        vector<ConnectInfo> connect_infos;
        get_connect_infos(connect_infos, pa1, patches, 4.0, env);

        if (stitch(patches, pa1, patch_type, connect_infos, env, false) == true) {
          if (pa1->motions.size() >= 2)
            cout << "interaction!" << endl;
          else
            cout << "single!" << endl;

          goto resolve_dead_online;
        }
      }

      //can not stitch
      cout << "can not stitch" << endl;
      //jittering
      if (jitter(*patch_type_unary, patches, patch_type, env, std::cout, min_pair, 1)) { 
        cout << "jitter success" << endl;
      }
      else
      {
        cout << "jitter fail" << endl;

        for (auto it = energies.begin(); it != energies.end(); ++it)
        {
          if (it->energy < 0.)
            continue;

          shared_ptr<Patch> pa1(new Patch((patch_type->find(it->name))->second));
          const auto &p1 = pa1->boundaries[it->b].posture();
          pa1->transform_between_posture(p2, p1);
          vector<ConnectInfo> connect_infos;
          get_connect_infos(connect_infos, pa1, patches, 4.0, env);

          if (stitch(patches, pa1, patch_type, connect_infos, env, false, false, 1, 24, true) == true) {
            if (pa1->motions.size() >= 2)
              cout << "interaction!" << endl;
            else
              cout << "single!" << endl;

            //어쩌다 연결된 dangling말고 지금 연결하려고 하는 dangling에만 history만 기록하자.
            it_b2->stitch_history.insert(pair<string,int>(it->name, it->b));
            goto resolve_dead_online;
          }
        }
      }

      //delete_patch(patches, min_pair.first, cout);
resolve_dead_online:;
     
      //후처리
      set_color_from_back(patches);
      DontSlip noslip;
      noslip.hold_foot(**(patches.end() - 1));
      //noslip.hold_foot(**(patches.end() - 2));
    }
  }
}
