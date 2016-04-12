/*
 *  motion.h
 *  unagi
 *
 *  Created by normit on 09. 09. 09.
 *  Copyright 2009 __MyCompanyName__. All rights reserved.
 *
 */
namespace ml {
  class Motion {
  public:
	Motion(const Body *body = NULL);
	
	template <class In> 
	Motion(const Motion &other, In i, In j) {
	  copy_common(other);
	  std::copy(i, j, std::back_inserter(m_postures));
	}
	
	template <class In> 
	void copy(const Motion &other, In i, In j) {
	  m_postures.clear();
	  copy_common(other);
	  std::copy(i, j, std::back_inserter(m_postures));
	}
	
	virtual ~Motion();

	void copy_common(const Motion &other);
	void get_motion( const Motion &other, int first, int last );	

	const Body *body() const { return m_body; }
	void body(const Body *body) { m_body = body; }

	void size(int size);
	size_t size() const { return m_postures.size();}

	inline Posture& operator[](int i) { return m_postures[i];}
	const Posture& operator[](int i) const { return m_postures[i];}

	Posture&		posture(int i)		  { return m_postures[i];}
	const Posture&	posture(int i) const  { return m_postures[i];}
	void			posture(int i, const Posture& p);
	Posture&		last_posture()		  { return m_postures[size()-1];}
	const Posture&	last_posture() const  { return m_postures[size()-1];}
	Posture&		first_posture()		  { return m_postures[0];}
	const Posture&	first_posture() const { return m_postures[0];}
	Posture&		posture_at_time(double time)	  { return *min_element(begin(), end(), [time](const ml::Posture &p1, const ml::Posture &p2){return fabs(p1.time - time) < fabs(p2.time - time);});}
	const Posture&	posture_at_time(double time)const {return *min_element(begin(), end(), [time](const ml::Posture &p1, const ml::Posture &p2){return fabs(p1.time - time) < fabs(p2.time - time);});}

	typedef vector<Posture>::iterator Iterator;
	typedef vector<Posture>::const_iterator Const_iterator;

	Iterator		begin() { return m_postures.begin(); }
	Const_iterator	begin() const { return m_postures.begin(); }
	Iterator		end()	{ return m_postures.end(); }
	Const_iterator	end() const { return m_postures.end(); }
	Iterator		iterator_at(double time) { return min_element(begin(), end(), [time](const ml::Posture &p1, const ml::Posture &p2){return fabs(p1.time - time) < fabs(p2.time - time);});}
	Const_iterator	iterator_at(double time) const {return min_element(begin(), end(), [time](const ml::Posture &p1, const ml::Posture &p2){return fabs(p1.time - time) < fabs(p2.time - time);});}	

	void AddPosture(const Posture& p);
	Motion* CropMotion(int from, int size, Motion *ret_motion = 0);
	Motion* Clone(Motion *ret_motion = 0);

	void SwapBody(const Body *toBody);
	void Sample(double rate);

	void LoadAMC(const char *amc_file, const char *asf_file, bool human_load = true, double scale = 1.0);
	void LoadAMC_with_contactInfo(const char *amc_file, const char *asf_file, bool human_load = true, double scale = 1.0, double ground_truth = 0.0);
	void LoadBVH(const char *file, bool human_load = true, double scale = 1.0, int sample = 1);

	void IkLimbSmoothly( const size_t frame, const int fduration, const int bduration, const size_t joint, const cml::vector3& pos );
	void SetFootConstraint( int ltoe_j, int lfoot_j, int rtoe_j, int rfoot_j, double toe_speed, double toe_height, double ankle_speed, double ankle_height);

	void stitch(const Motion & const_add_m, bool forward = true);

	void translate(const cml::vector3d &v);
	void translateSmoothly( const size_t posture_frame, const int forward_duration, const int backward_duration, const cml::vector3& pin_pos);
	void rotate( double theta );
	void translate_time( double time);
	void translate_time_to_zero();
	void ApplyTransf(const cml::transf &t, double time = 0.0);
	void transform_between_posture( ml::Posture to_posture, ml::Posture from_posture );	

  public:
	std::vector<Posture> m_postures;
	const Body *m_body;
	cml::vector3 color;
	Motion *connected_fore;
	Motion *connected_back;
	bool checked;
  }; 
};

