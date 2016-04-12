/*
 *  posture.h
 *  unagi
 *
 *  Created by normit on 09. 09. 09.
 *  Copyright 2009 __MyCompanyName__. All rights reserved.
 *
 */


namespace ml{

class Body;
	
class Posture
{
public:
  Posture();
  Posture(int num_joint);
	
  void init_variable();

  const Body* body() const { return m_body; }
  const cml::vector3& trans() const { return m_trans; }
  const cml::matrix3& rotate(int i) const { return m_rotates[i]; }

  void body(const Body* body);	
  void trans(const cml::vector3& trans) { m_trans = trans; }
  void rotate(int i, const cml::matrix3 &rot) { m_rotates[i] = rot; }
	
  size_t num_joint() const { return m_rotates.size(); }

  void ApplyTransf(const cml::transf& t);

  void IkLimb(int joint, const cml::vector3& pos, bool reverse = false);

  void SwapBody(const Body *tobody);

  void RotateGlobalRotation(int i, const cml::matrix3 &rot);
  void SetGlobalRotation(int i, const cml::matrix3& rot);

  cml::transf GetGlobalTransf(int i) const ;
  cml::matrix3 GetGlobalRoation(int i) const; 
  cml::vector3 GetGlobalTranslation(int i) const;

  const Body *m_body;
  cml::vector3	m_trans;
  std::vector<cml::matrix3> m_rotates;
  double time;

  int type_;
  int object;

  double object_height_correction;
  bool rigid;
	
  struct Status {
	  bool is_highlight;
	  cml::vector3 color;
  } status;
  
  void set_status_in_viewer(bool is_highlight, cml::vector3 color) { status.is_highlight = is_highlight; status.color = color;}
    
  bool leftFootTouched;
  bool rightFootTouched;
  bool is_interact;
};

void smoothing( Posture& p_modified, const Posture& p_fixed, const Posture& orig, int time, int range, bool forward );
cml::vector3 shoulder_orientation( const Posture &pos );

};
