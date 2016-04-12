#include "StdAfx.h"
#include "collision.h"
#include "QPerformanceTimer.h"


void set_obb(struct obb* b, int i, const cml::transf &se3, const cml::vector3 & l)
{
	b[i].lx = l[0] / 2.0;
	b[i].ly = l[1] / 2.0;
	b[i].lz = l[2] / 2.0;

	//se3를 해당 부분으로 복사
	b[i].m[0][0] = se3(0,0);	b[i].m[0][1] = se3(0,1);	b[i].m[0][2] = se3(0,2);	b[i].m[0][3] = se3(0,3);
	b[i].m[1][0] = se3(1,0);	b[i].m[1][1] = se3(1,1);	b[i].m[1][2] = se3(1,2);	b[i].m[1][3] = se3(1,3);
	b[i].m[2][0] = se3(2,0);	b[i].m[2][1] = se3(2,1);	b[i].m[2][2] = se3(2,2);	b[i].m[2][3] = se3(2,3);
	b[i].m[3][0] = 0.0;			b[i].m[3][1] = 0.0;			b[i].m[3][2] = 0.0;			b[i].m[3][3] = 1.0;	
}

void copy_obb( const ml::Posture &p, struct obb* b) {
	for (int i = 1; i < (int)p.num_joint(); ++i) {
		cml::vector3d offset = p.body()->offset(i);

		cml::transf rot_se3 = cml::make_transf(cml::exp_mat3(cml::between_vector(cml::vector3d(1,0,0),offset)), cml::vector3d(0,0,0) );
		cml::transf se3 = p.GetGlobalTransf(p.body()->parent(i)) * rot_se3;

		set_obb(b, i, se3, cml::vector3(offset.length() * 1.02, 0.04, 0.07));

		//to locate box center at origin
		b[i].m[0][3] +=  b[i].m[0][0] * b[i].lx;
		b[i].m[1][3] +=  b[i].m[1][0] * b[i].lx;
		b[i].m[2][3] +=  b[i].m[2][0] * b[i].lx;
	}

	if (p.object == 1)
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
		//object_trans[1] -= 0.1;
		cml::transf se3 = cml::trans_transf(object_trans) * cml::make_transf(rot_m, cml::vector3d(0,0,0));

		int i = p.num_joint();
		set_obb(b, i, se3, cml::vector3(0.6, 0.3, 0.3));
	}
}

bool collision_posture_obb( const ml::Posture &p1, const ml::Posture &p2 ) 
{
	int p1_num = p1.num_joint();
	int p2_num = p2.num_joint();
	if (p1.object == 1)
		++p1_num;
	if (p2.object == 1)
		++p2_num;

	obb* obb_p1 = new obb[p1_num];
	obb* obb_p2 = new obb[p2_num];

	copy_obb(p1, obb_p1);
	copy_obb(p2, obb_p2);

	for (int i = 1; i < p1_num; ++i) {
		for (int j = 1; j < p2_num; ++j) {
			if (intersect_obb(&obb_p1[i],&obb_p2[j])) {
				delete[] obb_p1;
				delete[] obb_p2;
				return true;
			}
		}
	}
	delete[] obb_p1;
	delete[] obb_p2;

	return false;
}

bool collision_postures_obb( const vector<const ml::Posture *> ps1, const vector<const ml::Posture *> ps2, double maximum_radius /*= 1.1 */ )
{
	//(ps1[0])->trans();
	if (ps1.empty() || ps2.empty())
		return false;

	//copy_obb를 수행했는지 table에 저장(중복을 막기 위함)
	bool* set_tbl1;
	bool* set_tbl2;

	//int num = (ps1[0])->num_joint();

	vector<obb*> obbs1(ps1.size());
	set_tbl1 = new bool[ps1.size()];
	for (int i = 0; i < ps1.size(); ++i) {
		int p1_num = (ps1[i])->num_joint();
		if ((ps1[i])->object == 1)
			++p1_num;
		
		obbs1[i] = new obb[p1_num];
		set_tbl1[i] = false;
		//copy_obb(*(ps1[i]), obbs1[i]);
	}

	vector<obb*> obbs2(ps2.size());
	set_tbl2 = new bool[ps2.size()];
	for (int i = 0; i < ps2.size(); ++i) {
		int p2_num = (ps2[i])->num_joint();
		if ((ps2[i])->object == 1)
			++p2_num;

		obbs2[i] = new obb[p2_num];
		set_tbl2[i] = false;
		//copy_obb(*(ps2[i]), obbs2[i]);
	}

	for (int o1 = 0; o1 < obbs1.size(); ++o1)
	{
		for (int o2 = 0; o2 < obbs2.size(); ++o2)
		{
			//중심 구해서 컬링
			cml::vector3d c1 = (*(ps1[o1])).trans();
			cml::vector3d c2 = (*(ps2[o2])).trans();
			
			//두 posture가 충분히 떨어져 있으면 다음 posture pair로 continue
			double dist = cml::length(cml::vec2(c1 - c2));
			if (dist > maximum_radius * 2) continue;
			
			//두 posture중 copy obb가 실행되지 않은 posture있으면 실행해줌
			if (!set_tbl1[o1]) {
				copy_obb(*(ps1[o1]), obbs1[o1]);
				set_tbl1[o1] = true;
			}

			if (!set_tbl2[o2]) {
				copy_obb(*(ps2[o2]), (obbs2[o2]));
				set_tbl2[o2] = true;
			}

			int p1_num = (ps1[o1])->num_joint();
			if ((ps1[o1])->object == 1)
				++p1_num;

			int p2_num = (ps2[o2])->num_joint();
			if ((ps2[o2])->object == 1)
				++p2_num;

			for (int i = 1; i < p1_num; ++i) 
			{
				for (int j = 1; j < p2_num; ++j) 
				{
					if (intersect_obb(&(obbs1[o1][i]),&(obbs2[o2][j]))) {
						for (int ii = 0; ii < obbs1.size(); ++ii)
							delete[] obbs1[ii];
						for (int ii = 0; ii < obbs2.size(); ++ii)
							delete[] obbs2[ii];
						delete[] set_tbl1;
						delete[] set_tbl2;
						return true;
					}
				}
			}
		}
	}

	for (int ii = 0; ii < obbs1.size(); ++ii)
		delete[] obbs1[ii];
	for (int ii = 0; ii < obbs2.size(); ++ii)
		delete[] obbs2[ii];
	delete[] set_tbl1;
	delete[] set_tbl2;
	return false;
}

bool collision_postures_boxes_obb( const vector<const ml::Posture *> ps1, const vector<const ml::Posture *> ps2, const vector<const Box *> boxes, double maximum_radius /*= 1.1 */ )
{
	//(ps1[0])->trans();
	/*if (ps1.empty() || ps2.empty())
		return false;*/

	//copy_obb를 수행했는지 table에 저장(중복을 막기 위함)
	bool* set_tbl1;
	bool* set_tbl2;

	//int num = (ps1[0])->num_joint();

	vector<obb*> obbs1(ps1.size());
	set_tbl1 = new bool[ps1.size()];
	for (int i = 0; i < ps1.size(); ++i) {
		int p1_num = (ps1[i])->num_joint();
		if ((ps1[i])->object == 1)
			++p1_num;

		obbs1[i] = new obb[p1_num];
		set_tbl1[i] = false;
		//copy_obb(*(ps1[i]), obbs1[i]);
	}

	vector<obb*> obbs2(ps2.size());
	set_tbl2 = new bool[ps2.size()];
	for (int i = 0; i < ps2.size(); ++i) {
		int p2_num = (ps2[i])->num_joint();
		if ((ps2[i])->object == 1)
			++p2_num;

		obbs2[i] = new obb[p2_num];
		set_tbl2[i] = false;
		//copy_obb(*(ps2[i]), obbs2[i]);
	}

	int obb_box_num = boxes.size();
	obb* obb_box = new obb[obb_box_num];
	for (int i = 0; i < obb_box_num; ++i)
	{
		auto it = boxes[i];
		set_obb(obb_box, i, it->se3, it->length);
	}

	for (int o1 = 0; o1 < obbs1.size(); ++o1)
	{
		//copy obb가 실행되지 않은 posture있으면 실행해줌
		if (!set_tbl1[o1]) {
			copy_obb(*(ps1[o1]), obbs1[o1]);
			set_tbl1[o1] = true;
		}

		int p1_num = (ps1[o1])->num_joint();
		if ((ps1[o1])->object == 1)
			++p1_num;

		for (int o2 = 0; o2 < obbs2.size(); ++o2)
		{
			//중심 구해서 컬링
			cml::vector3d c1 = (*(ps1[o1])).trans();
			cml::vector3d c2 = (*(ps2[o2])).trans();

			//두 posture가 충분히 떨어져 있으면 다음 posture pair로 continue
			double dist = cml::length(cml::vec2(c1 - c2));
			if (dist > maximum_radius * 2) continue;

			//copy obb가 실행되지 않은 posture있으면 실행해줌
			if (!set_tbl2[o2]) {
				copy_obb(*(ps2[o2]), (obbs2[o2]));
				set_tbl2[o2] = true;
			}
			
			int p2_num = (ps2[o2])->num_joint();
			if ((ps2[o2])->object == 1)
				++p2_num;

			for (int i = 1; i < p1_num; ++i) 
			{
				for (int j = 1; j < p2_num; ++j) 
				{
					if (intersect_obb(&(obbs1[o1][i]),&(obbs2[o2][j]))) {
						for (int ii = 0; ii < obbs1.size(); ++ii)
							delete[] obbs1[ii];
						for (int ii = 0; ii < obbs2.size(); ++ii)
							delete[] obbs2[ii];
						delete[] obb_box;
						delete[] set_tbl1;
						delete[] set_tbl2;
						return true;
					}
				}
			}
		}

		for (int i = 1; i < p1_num; ++i) 
		{
			for (int j = 0; j < obb_box_num; ++j) 
			{
				if (intersect_obb(&(obbs1[o1][i]),&(obb_box[j]))) {
					for (int ii = 0; ii < obbs1.size(); ++ii)
						delete[] obbs1[ii];
					for (int ii = 0; ii < obbs2.size(); ++ii)
						delete[] obbs2[ii];
					delete[] obb_box;
					delete[] set_tbl1;
					delete[] set_tbl2;
					return true;
				}
			}
		}
	}

	for (int ii = 0; ii < obbs1.size(); ++ii)
		delete[] obbs1[ii];
	for (int ii = 0; ii < obbs2.size(); ++ii)
		delete[] obbs2[ii];
	delete[] obb_box;
	delete[] set_tbl1;
	delete[] set_tbl2;
	return false;
}

bool collision_postures_obb_nocull( const vector<const ml::Posture *> ps1, const vector<const ml::Posture *> ps2, double maximum_radius /*= 1.1*/ )
{
	if (ps1.empty() || ps2.empty())
		return false;

	int num = (ps1[0])->num_joint();

	vector<obb*> obbs1(ps1.size());
	for (int i = 0; i < ps1.size(); ++i) {
		obbs1[i] = new obb[num];
		copy_obb(*(ps1[i]), obbs1[i]);
	}

	vector<obb*> obbs2(ps2.size());
	for (int i = 0; i < ps2.size(); ++i) {
		obbs2[i] = new obb[num];
		copy_obb(*(ps2[i]), obbs2[i]);
	}

	for (int o1 = 0; o1 < obbs1.size(); ++o1)
	{
		for (int o2 = 0; o2 < obbs2.size(); ++o2)
		{
			for (int i = 1; i < num; ++i) 
			{
				for (int j = 1; j < num; ++j) 
				{
					if (intersect_obb(&(obbs1[o1][i]),&(obbs2[o2][j]))) {
						for (int ii = 0; ii < obbs1.size(); ++ii)
							delete[] obbs1[ii];
						for (int ii = 0; ii < obbs2.size(); ++ii)
							delete[] obbs2[ii];
						return true;
					}
				}
			}
		}
	}

	for (int ii = 0; ii < obbs1.size(); ++ii)
		delete[] obbs1[ii];
	for (int ii = 0; ii < obbs2.size(); ++ii)
		delete[] obbs2[ii];
	return false;

	// 간단하지만 obb를 매번 만들어서 비효율적인 방법.
	/*for (auto it1 = ps1.begin(); it1 != ps1.end(); ++it1)
	{
	for (auto it2 = ps2.begin(); it2 != ps2.end(); ++it2)
	{
	if (collision_posture_obb(*(*it1), *(*it2)))
	return true;
	}
	}
	return false;*/
}


int get_begin_time(const vector<shared_ptr<Patch>> & patches, const set<int> &check_set);
int get_last_time(const vector<shared_ptr<Patch>> & patches, const set<int> &check_set);

void add_collision_posture(const ml::Posture & p, VCollide & vc ) 
{
	for (int bodyi = 1; bodyi < (int)p.num_joint(); ++bodyi)
	{
		cml::vector3d offset = p.body()->offset(bodyi);
		cml::transf rot_se3 = cml::make_transf(cml::exp_mat3(cml::between_vector(cml::vector3d(1,0,0),offset)), cml::vector3d(0,0,0) );
		cml::transf se3 = p.GetGlobalTransf(p.body()->parent(bodyi)) * rot_se3;
		cml::vector3d vx = cml::vec3(se3 * cml::vector4d(offset.length()*1.02,0,0,0));
		cml::vector3d vz = cml::vec3(se3 * cml::vector4d(0,0,0.07,0));

		if (offset.length() <= 0.0)
		{
			std::cout << bodyi << ' ' << offset.length() << std::endl;
			continue;
		}
		cml::vector3d p1 = cml::vec3(se3 * cml::vector4d(0, 0.0, -0.07*0.5, 1));
		cml::vector3 p2 = p1+vx;
		cml::vector3 p3 = p1+vz;
		cml::vector3 p4 = p1+vz+vx;

		double v1[3], v2[3], v3[3];

		v1[0] = p1[0]; v1[1] = p1[1]; v1[2] = p1[2];
		v2[0] = p2[0]; v2[1] = p2[1]; v2[2] = p2[2];
		v3[0] = p3[0]; v3[1] = p3[1]; v3[2] = p3[2]; 
		vc.AddTri(v1, v2, v3,0);

		v1[0] = p4[0]; v1[1] = p4[1]; v1[2] = p4[2];
		v2[0] = p2[0]; v2[1] = p2[1]; v2[2] = p2[2];
		v3[0] = p3[0]; v3[1] = p3[1]; v3[2] = p3[2]; 
		vc.AddTri(v1, v2, v3,0);
	}
}

void add_collision_motion(int time, const ml::Motion & m, VCollide &vc)
{
	auto it = find_if(m.begin()+1, m.end()-1, [time](const ml::Posture &p){ return fabs(p.time - time) < 0.5+0.25;});
	if (it != m.end()-1) //m.end()가 아니라 m.end()-1 로 해야한다는 걸 주의!
	{
		add_collision_posture(*it, vc);
	}
}

void add_collision_patch(int time, const Patch & pa, VCollide &vc)
{
	for (auto it = pa.motions.begin(); it != pa.motions.end(); ++it)
	{
		add_collision_motion(time, *it, vc);
	}
}

bool add_collision_object_patch_immutable(int time, const Patch & pa, VCollide &vc)
{
	if (is_exist(time, pa)) {
		int i;
		vc.NewObject(&i);
		add_collision_patch(time, pa, vc);
		vc.EndObject();
		return true;
	}
	return false;
}

bool collision_posture( const ml::Posture &p1, const ml::Posture &p2 ) 
{
	VCollide vc;
	int id[2];

	vc.NewObject(&id[0]);
	add_collision_posture(p1, vc);
	vc.EndObject();

	vc.NewObject(&id[1]);
	add_collision_posture(p2, vc);
	vc.EndObject();

	VCReport report;
	vc.Collide( &report );  

	return report.numObjPairs() >= 1;
}

int collision_motion( const ml::Motion &m1, const ml::Motion &m2 )
{
	int begin_time = m1.begin()->time;
	int last_time = (m1.end()-1)->time;
	
	for (int time = begin_time; time <= last_time; ++time)
	{
		VCollide vc;
		int id[2];

		vc.NewObject(&id[0]);
		add_collision_motion(time, m1, vc);
		vc.EndObject();

		vc.NewObject(&id[1]);
		add_collision_motion(time, m2, vc);
		vc.EndObject();

		VCReport report;
		vc.Collide(&report);

		if (report.numObjPairs() >= 1) {
			return time;
		}
	}

	return -1;
}

int collision_motion_obb( const ml::Motion &m1, const ml::Motion &m2 )
{
	int begin_time = m1.begin()->time;
	int last_time = (m1.end()-1)->time;

	//모션의 가장자리에서는 충돌체크 안한다.
	for (int time = begin_time+1; time <= last_time-1; ++time)
	{
		const ml::Posture &p1 = m1[time];
		const ml::Posture &p2 = m2[time];

		if (collision_posture_obb(p1, p2))
			return time;
	}

	return -1;
}

int collision_patch( const Patch &pa1, const Patch &pa2 )
{
	int begin_time = pa1.get_begin_time();
	int last_time = pa1.get_last_time();

	for (int time = begin_time; time <= last_time; ++time)
	{
		VCollide vc;
		int id[2];

		vc.NewObject(&id[0]);
		add_collision_patch(time, pa1, vc);
		vc.EndObject();

		vc.NewObject(&id[1]);
		add_collision_patch(time, pa2, vc);
		vc.EndObject();

		VCReport report;
		vc.Collide(&report);

		if (report.numObjPairs() >= 1) {
			return time;
		}
	}

	return -1;
}


void add_posture( const Patch &pa, int time, vector<const ml::Posture *> &ps ) 
{
	for (auto it = pa.motions.begin(); it != pa.motions.end(); ++it)
	{
		const ml::Motion &m = *it;
		//충돌 체크에서 모션의 가장자리는 뺐다.
		if ((m.begin()+1)->time <= time && time <= (m.end()-2)->time)
		{
			auto p_posture = std::min_element(m.begin()+1, m.end()-1, [time](const ml::Posture &p1, const ml::Posture &p2){return fabs(p1.time - time) < fabs(p2.time - time);});
			ps.push_back(&(*p_posture));
		}			
	}
}

int collision_patch_obb( const Patch &pa1, const Patch &pa2 )
{
	int begin_time = pa1.get_begin_time();
	int last_time = pa1.get_last_time();

	for (int time = begin_time; time <= last_time; ++time)
	{
		vector<const ml::Posture *> ps1;
		vector<const ml::Posture *> ps2;

		add_posture(pa1, time, ps1);
		add_posture(pa2, time, ps2);

		if (collision_postures_obb(ps1, ps2))
			return time;
	}

	return -1;
}

bool is_exist(int time, const Patch &pa)
{
	for (auto it_m = pa.motions.begin(); it_m != pa.motions.end(); ++it_m)
	{
		const ml::Motion &m = *it_m;
		auto it = find_if(m.begin()+1, m.end()-1, [time](const ml::Posture &p){ return fabs(p.time - time) < 0.5+0.25;});
		if (it != m.end()-1) //m.end()가 아니라 m.end()-1 로 해야한다는 걸 주의!
		{
			return true;
		}
	}
	return false;
}

bool collision_patches_nopreprocess( const vector<shared_ptr<Patch>> & patches, const set<int> &check_set )
{
	int begin_time = get_begin_time(patches, check_set);
	int last_time = get_last_time(patches, check_set);

	for (int time = begin_time; time <= last_time; ++time)
	{
		VCollide vc;
		int id[2];

		vc.NewObject(&id[0]);
		for (int i = 0; i < patches.size(); ++i)
		{
			if (check_set.find(i) == check_set.end())
				add_collision_patch(time, *patches[i], vc);
		}
		vc.EndObject();

		//check_set 패치들간의 상호충돌도 검출하기 위해 각 패치마다 object로 넣는다.
		for (int i = 0; i < patches.size(); ++i)
		{
			if (check_set.find(i) != check_set.end())
				add_collision_object_patch_immutable(time, *patches[i], vc); //patch 안의 collide_id를 변화시키지 않도록 immutable 함수를 사용했다.
		}
		
		VCReport report;
		vc.Collide(&report);

		if (report.numObjPairs() >= 1) {
			return true;
		}
	}
	return false;
}

bool collision_patches_nopreprocess_twopart( const vector<shared_ptr<Patch>> & patches, const set<int> &check_set )
{
	int begin_time = get_begin_time(patches, check_set);
	int last_time = get_last_time(patches, check_set);

	for (int time = begin_time; time <= last_time; ++time)
	{
		VCollide vc;
		int id[2];

		vc.NewObject(&id[0]);
		for (int i = 0; i < patches.size(); ++i)
		{
			if (check_set.find(i) == check_set.end())
				add_collision_patch(time, *patches[i], vc);
		}
		vc.EndObject();

		//no between inter collision set
		vc.NewObject(&id[1]);
		for (int i = 0; i < patches.size(); ++i)
		{
			if (check_set.find(i) != check_set.end())
				add_collision_patch(time, *patches[i], vc);
		}
		vc.EndObject();

		VCReport report;
		vc.Collide(&report);

		if (report.numObjPairs() >= 1) {
			return true;
		}
	}
	return false;
}

int get_begin_time(const vector<shared_ptr<Patch>> & patches, const set<int> &check_set)
{
	vector<double> begin_times;
	for (auto it = check_set.begin(); it != check_set.end(); ++it)
	{
		begin_times.push_back(patches[*it]->get_begin_time());
	}
	return *min_element(begin_times.begin(), begin_times.end());
}

int get_last_time(const vector<shared_ptr<Patch>> & patches, const set<int> &check_set)
{
	vector<double> last_times;
	for (auto it = check_set.begin(); it != check_set.end(); ++it)
	{
		last_times.push_back(patches[*it]->get_last_time());
	}
	return *max_element(last_times.begin(), last_times.end());
}

int collision_patches_obb( const vector<shared_ptr<Patch>> & patches, const set<int> &check_set )
{
	int begin_time = get_begin_time(patches, check_set);
	int last_time = get_last_time(patches, check_set);

	for (int time = begin_time; time <= last_time; ++time)
	{
		vector<const ml::Posture *> ps1;
		vector<const ml::Posture *> ps2;

		for (int i = 0; i < patches.size(); ++i)
		{
			if (check_set.count(i) > 0)
				add_posture(*patches[i], time, ps1);
			else
				add_posture(*patches[i], time, ps2);
		}

		if (collision_postures_obb(ps1, ps2, 1.0)) {
			return time;
		}
	}

	return -1000;
}

int collision_patches_env_obb( const vector<shared_ptr<Patch>> & patches, const set<int> &check_set, const Env &env )
{
	int begin_time = get_begin_time(patches, check_set);
	int last_time = get_last_time(patches, check_set);

	for (int time = begin_time; time <= last_time; ++time)
	{
		vector<const ml::Posture *> ps1;
		vector<const ml::Posture *> ps2;

		for (int i = 0; i < patches.size(); ++i)
		{
			if (check_set.count(i) > 0)
				add_posture(*patches[i], time, ps1);
			else
				add_posture(*patches[i], time, ps2);
		}

		//cyclic
		vector<const ml::Posture *> ps1_cyclic;
		int ps1_size = ps1.size();
		for (int i = 0; i < ps1_size; ++i)
		{
			const ml::Posture * p1 = ps1[i];

			if (env.is_cyclic.find("x")->second == true)
			{
				if (p1->trans()[0] < env.range.find("x_min")->second + 0.5)
				{
					ml::Posture *p1_cyclic = new ml::Posture(*p1);
					p1_cyclic->ApplyTransf(cml::transf(cml::trans_transf(cml::vector3((env.range.find("x_max")->second - env.range.find("x_min")->second),0,0))));
					ps1.push_back(p1_cyclic);
					ps1_cyclic.push_back(p1_cyclic);
				}
				else if (p1->trans()[0] > env.range.find("x_max")->second - 0.5)
				{
					ml::Posture *p1_cyclic = new ml::Posture(*p1);
					p1_cyclic->ApplyTransf(cml::transf(cml::trans_transf(cml::vector3(-(env.range.find("x_max")->second - env.range.find("x_min")->second),0,0))));
					ps1.push_back(p1_cyclic);
					ps1_cyclic.push_back(p1_cyclic);
				}
			}
		}

		ps1_size = ps1.size();
		for (int i = 0; i < ps1_size; ++i)
		{
			const ml::Posture * p1 = ps1[i];

			if (env.is_cyclic.find("z")->second == true)
			{
				if (p1->trans()[2] < env.range.find("z_min")->second + 0.5)
				{
					ml::Posture *p1_cyclic = new ml::Posture(*p1);
					p1_cyclic->ApplyTransf(cml::transf(cml::trans_transf(cml::vector3(0,0, (env.range.find("z_max")->second - env.range.find("z_min")->second)))));
					ps1.push_back(p1_cyclic);
					ps1_cyclic.push_back(p1_cyclic);
				}
				else if (p1->trans()[2] > env.range.find("z_max")->second - 0.5)
				{
					ml::Posture *p1_cyclic = new ml::Posture(*p1);
					p1_cyclic->ApplyTransf(cml::transf(cml::trans_transf(cml::vector3(0,0, -(env.range.find("z_max")->second - env.range.find("z_min")->second)))));
					ps1.push_back(p1_cyclic);
					ps1_cyclic.push_back(p1_cyclic);
				}
			}
		}

		//static box
		vector<const Box *> boxes;
		for (auto it = env.static_boxes.begin(); it != env.static_boxes.end(); ++it)
			boxes.push_back(&(*it));

		//dynamic box
		for (auto it = env.dynamic_boxes.begin(); it != env.dynamic_boxes.end(); ++it)
		{
			if (it->begin()->time - 0.5 < time && time < (it->end()-1)->time + 0.5) {
				auto it_box = std::min_element(it->begin(), it->end(), [time](const Box &b1, const Box &b2){return fabs(b1.time - time) < fabs(b2.time - time);});
				boxes.push_back(&(*it_box));
			}
		}

		bool is_collision = collision_postures_boxes_obb(ps1, ps2, boxes, 1.0);

		//cyclic 메모리 해제
		for (int ii = 0; ii < ps1_cyclic.size(); ++ii)
			delete ps1_cyclic[ii];

		if (is_collision)
			return time;
	}

	return -1000;
}

bool collision_patches_obb_two( const vector<shared_ptr<Patch>> & patches, int pa1, int pa2 )
{
	set<int> check_set;
	check_set.insert(pa1);
	int begin_time = get_begin_time(patches, check_set);
	int last_time = get_last_time(patches, check_set);

	for (int time = begin_time; time <= last_time; ++time)
	{
		vector<const ml::Posture *> ps1;
		vector<const ml::Posture *> ps2;

		add_posture(*patches[pa1], time, ps1);
		add_posture(*patches[pa2], time, ps2);

		if (collision_postures_obb(ps1, ps2, 1.0)) {
			return true;
		}
	}

	return false;
}

int collision_patches_obb_nocull( const vector<shared_ptr<Patch>> & patches, const set<int> &check_set )
{
	int begin_time = get_begin_time(patches, check_set);
	int last_time = get_last_time(patches, check_set);

	for (int time = begin_time; time <= last_time; ++time)
	{
		vector<const ml::Posture *> ps1;
		vector<const ml::Posture *> ps2;

		for (int i = 0; i < patches.size(); ++i)
		{
			if (check_set.count(i) > 0)
				add_posture(*patches[i], time, ps1);
			else
				add_posture(*patches[i], time, ps2);
		}

		if (collision_postures_obb_nocull(ps1, ps2)) {
			return time;
		}
	}

	return -1;
}

obb bound_obb_posture( const ml::Posture &pos )
{  
  cml::vector3 hand = pos.GetGlobalTranslation(pos.body()->joint_index("RightHandDummy")) - pos.GetGlobalTranslation(pos.body()->joint_index("LeftHandDummy"));
  cml::vector3 hlf = pos.GetGlobalTranslation(pos.body()->joint_index("RightHandDummy")) - pos.GetGlobalTranslation(pos.body()->joint_index("LeftToesDummy"));
  cml::vector3 hrf = pos.GetGlobalTranslation(pos.body()->joint_index("RightHandDummy")) - pos.GetGlobalTranslation(pos.body()->joint_index("RightToesDummy"));
  cml::vector3 lfoot= pos.GetGlobalTranslation(pos.body()->joint_index("HEadDummy")) - pos.GetGlobalTranslation(pos.body()->joint_index("LeftToesDummy"));
  cml::vector3 rfoot= pos.GetGlobalTranslation(pos.body()->joint_index("HEadDummy")) - pos.GetGlobalTranslation(pos.body()->joint_index("RightToesDummy"));
  cml::vector3 foot = lfoot - rfoot;

  obb ret_obb;   
  ret_obb.transf_m = pos.GetGlobalTransf(0);
  const cml::vector3 xaxis(1., 0., 0.);
  ret_obb.lx = abs(cml::dot(hand, cml::mat3(ret_obb.transf_m) * xaxis));
  const cml::vector3 yaxis(0., 1., 0.);
  ret_obb.ly = (abs(cml::dot(lfoot, cml::mat3(ret_obb.transf_m) * yaxis)) > abs(cml::dot(rfoot, cml::mat3(ret_obb.transf_m) * yaxis))) ? abs(cml::dot(lfoot, cml::mat3(ret_obb.transf_m) * yaxis)) : abs(cml::dot(rfoot, cml::mat3(ret_obb.transf_m) * yaxis));  
  const cml::vector3 zaxis(0., 0., 1.);
  double a = abs(cml::dot(hand, cml::mat3(ret_obb.transf_m) * zaxis));
  double b = abs(cml::dot(foot, cml::mat3(ret_obb.transf_m) * zaxis));
  a = (a > b) ? a : b;
  double c = abs(cml::dot(hlf, cml::mat3(ret_obb.transf_m) * zaxis));
  double d = abs(cml::dot(hrf, cml::mat3(ret_obb.transf_m) * zaxis));
  b = (c > d) ? c : d;
  ret_obb.lz = (a > b) ? a : b;
    
  for (size_t i=0; i<4; ++i) {
	for (size_t j=0; j<4; ++j) {
	  ret_obb.m[i][j] = ret_obb.transf_m(j, i);
	}
  }

  return ret_obb;
}
