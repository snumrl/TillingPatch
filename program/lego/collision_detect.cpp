#include "collision_detect.h"

CollisionDetect::CollisionDetect(void)
{
	enable = true;
	//enable = false;
	
	use_vc = true;
	//use_vc = false;

	R1[0][0] = R1[1][1] = R1[2][2] = 1.0;
	R1[0][1] = R1[1][0] = R1[2][0] = 0.0;
	R1[0][2] = R1[1][2] = R1[2][1] = 0.0;

	R2[0][0] = R2[1][1] = R2[2][2] = 1.0;
	R2[0][1] = R2[1][0] = R2[2][0] = 0.0;
	R2[0][2] = R2[1][2] = R2[2][1] = 0.0;

	T1[0] = 0.0;  T1[1] = 0.0; T1[2] = 0.0;
	T2[0] = 0.0;  T2[1] = 0.0; T2[2] = 0.0;
}

CollisionDetect::~CollisionDetect(void)
{
}

void CollisionDetect::init_patches( vector<Patch>& patches )
{
	if (enable == false)
		return;

	if (use_vc == true)
		init_patches_vc(patches);
	else
		init_patches_rp(patches);
}

void CollisionDetect::init_patches_rp(vector<Patch>& patches)
{
	int endtime = patches_endtime(patches);
	for (int time = 0; time <= endtime; ++time)
	{
		occupied[time].BeginModel();
		for (int i = 0; i < patches.size(); ++i)
		{
			make_collision_model_patch_rp(time, patches[i], occupied[time]);
		}
		occupied[time].EndModel();
	}
}

bool CollisionDetect::detect( Patch& p1 )
{
	if (enable == false)
		return false;

	if (use_vc == true)
		return detect_vc(p1);
	else
		return detect_rp(p1);
}

bool CollisionDetect::detect_rp( Patch& p1 )
{
	for (int time = p1.get_begintime(); time <= p1.get_endtime(); ++time)
	{
		RAPID_model b1;
		b1.BeginModel();
		make_collision_model_patch_rp(time, p1, b1);
		b1.EndModel();

		RAPID_Collide(R1, T1, &occupied[time], R2, T2, &b1, RAPID_FIRST_CONTACT);

		if (RAPID_num_contacts >= 1)
			return true;
	}

	return false;
}

bool CollisionDetect::detect_vc( Patch& p1 )
{
	int begin_time = p1.get_begintime();
	int end_time = p1.get_endtime();

	for (int time = begin_time; time <= end_time; ++time)
	{
		vcs[time].DeactivatePairAll();

		bool is_new_object = make_collision_model_patch_vc(time, p1, vcs[time]); 

		if (is_new_object == true)
		{
			VCReport report;
			vcs[time].Collide( &report );
			vcs[time].DeleteObject(p1.collide_id[time]);

			if (report.numObjPairs() >= 1)
			{	
				return true;
			}
		}
	}

	return false;
}

bool CollisionDetect::detect_vc_multi(Patch p1, map<int, Patch> &patches_stitched)
{
	// p1에 모션을 추가하기 때문에 복사해서 넘겼구나.
	for (auto it = patches_stitched.begin(); it != patches_stitched.end(); ++it)
	{
		vector<ml::Motion> &new_motions = it->second.half_motions;
		for (int i = 0; i < new_motions.size();++i)
			p1.half_motions.push_back(new_motions[i]);
	}

	int begin_time = p1.get_begintime();
	int end_time = p1.get_endtime();

	if (begin_time < 0)
		begin_time = 0;

	for (int time = begin_time; time <= end_time; ++time)
	{
		vcs[time].DeactivatePairAll();

		bool is_new_object = make_collision_model_patch_vc(time, p1, vcs[time]); 

		if (is_new_object == true)
		{
			for (auto it = patches_stitched.begin(); it != patches_stitched.end(); ++it)
			{
				int coll_id = it->second.collide_id[time];
				if (coll_id >= 0)
				{
					vcs[time].DeactivateObject(coll_id);
				}
			}

			VCReport report;
			vcs[time].Collide( &report );
			vcs[time].DeleteObject(p1.collide_id[time]);

			for (auto it = patches_stitched.begin(); it != patches_stitched.end(); ++it)
			{
				int coll_id = it->second.collide_id[time];
				if (coll_id >= 0)
				{
					vcs[time].ActivateObject(coll_id);
				}
			}

			if (report.numObjPairs() >= 1)
			{	
				//cout << "col time " << time << endl;
				return true;
			}
		}
	}

	return false;
}

int patch_i_from_collide_id(vector<Patch> &patches, int time, int col_id)
{
	for (int i = 0; i < patches.size(); ++i)
	{
		if (patches[i].collide_id[time] == col_id)
			return i;
	}
}

std::set<std::pair<int, int>> CollisionDetect::init_and_detect_vc( vector<Patch>& patches )
{
	//초기화
	for (auto it = patches.begin(); it != patches.end(); ++it)
	{
		it->init_collide_id();
	}

	int begin_time = patches_begintime(patches);
	int end_time = patches_endtime(patches);
	if (begin_time < 0)
		begin_time = 0;

	std::set<std::pair<int,int>> col_pair;

	for (int time = begin_time; time <= end_time; ++time)
	{
		for (int i = 0; i < patches.size(); ++i)
		{
			make_collision_model_patch_vc(time, patches[i], vcs[time]);
		}

		VCReport report;
		vcs[time].Collide( &report, VC_ALL_CONTACTS );
		//vcs[time].Collide( &report, VC_FIRST_CONTACT);

		for (int j = 0; j < report.numObjPairs(); j++)
		{
			/*cout << time << " Detected collision between objects "
				<< report.obj1ID(j) << " and " << report.obj2ID(j) << "\n";*/
			col_pair.insert(std::pair<int,int>(patch_i_from_collide_id(patches, time, report.obj1ID(j)), patch_i_from_collide_id(patches, time, report.obj2ID(j))));
		}
	}

	return col_pair;
}

void CollisionDetect::init_patches_vc(vector<Patch>& patches)
{
	int endtime = patches_endtime(patches);
	for (int time = 0; time <= endtime; ++time)
	{
		for (int i = 0; i < patches.size(); ++i)
		{
			make_collision_model_patch_vc(time, patches[i], vcs[time]);
		}
	}

}

void CollisionDetect::add_vc(Patch &pa)
{
	//충돌 정보를 새로 생성하고 충돌 체크 하자.
	int begin_time = pa.get_begintime();
	int end_time = pa.get_endtime();
	if (begin_time < 0)
		begin_time = 0;

	for (int time = begin_time; time <= end_time; ++time)
	{
		make_collision_model_patch_vc(time, pa, vcs[time]);
	}

	return;
}

void CollisionDetect::add_vc(std::vector<Patch> &patches, std::set<int> &indexes )
{
	//충돌 정보를 새로 생성하자.
	int begin_time = patches_begintime_indexes(patches, indexes);
	int end_time = patches_endtime_indexes(patches, indexes);
	if (begin_time < 0)
		begin_time = 0;

	for (int time = begin_time; time <= end_time; ++time)
	{
		for (auto it = indexes.begin(); it != indexes.end(); ++it)
		{
			Patch &pa = patches[*it];
			make_collision_model_patch_vc(time, pa, vcs[time]);
		}
	}

	return;
}

void CollisionDetect::delete_vc( std::vector<Patch> &patches, std::set<int> &indexes )
{
	for (auto it = indexes.begin(); it != indexes.end(); ++it)
	{
		Patch &pa = patches[*it];
		for (int time = 0; time < 1000; ++time)
		{
			int coll_id = pa.collide_id[time];
			if (coll_id >= 0)
				vcs[time].DeleteObject(coll_id);
		}
		pa.init_collide_id();
	}
}

bool CollisionDetect::detect_vc_index( std::vector<Patch> &patches, std::set<int> &indexes )
{
	//일단 이전 충돌 정보를 없애자.
	delete_vc(patches, indexes);
	
	//충돌 정보를 새로 생성하고 충돌 체크 하자.
	int begin_time = patches_begintime_indexes(patches, indexes);
	int end_time = patches_endtime_indexes(patches, indexes);
	if (begin_time < 0)
		begin_time = 0;

	for (int time = begin_time; time <= end_time; ++time)
	{
		vcs[time].DeactivatePairAll();

		for (auto it = indexes.begin(); it != indexes.end(); ++it)
		{
			Patch &pa = patches[*it];
			make_collision_model_patch_vc(time, pa, vcs[time]);
		}

		VCReport report;
		vcs[time].Collide( &report );
		if (report.numObjPairs() >= 1)
		{	
			//cout << "col time " << time << endl;
			return true;
		}
	}

	return false;
}

set<int> CollisionDetect::detect_vc_index_all( std::vector<Patch> &patches, std::set<int> &indexes )
{
	set<int> col_indexes; 

	//일단 이전 충돌 정보를 없애자.
	for (auto it = indexes.begin(); it != indexes.end(); ++it)
	{
		Patch &pa = patches[*it];
		for (int time = 0; time < 1000; ++time)
		{
			int coll_id = pa.collide_id[time];
			if (coll_id >= 0)
				vcs[time].DeleteObject(coll_id);
		}
	}

	//충돌 정보를 새로 생성하고 충돌 체크 하자.
	int begin_time = patches_begintime_indexes(patches, indexes);
	int end_time = patches_endtime_indexes(patches, indexes);
	if (begin_time < 0)
		begin_time = 0;

	for (int time = begin_time; time <= end_time; ++time)
	{
		vcs[time].DeactivatePairAll();

		for (auto it = indexes.begin(); it != indexes.end(); ++it)
		{
			Patch &pa = patches[*it];
			make_collision_model_patch_vc(time, pa, vcs[time]);
		}

		VCReport report;
		vcs[time].Collide( &report, VC_ALL_CONTACTS );
		for (int j = 0; j < report.numObjPairs(); j++)
		{
			int pa1 = patch_i_from_collide_id(patches, time, report.obj1ID(j));
			int pa2 = patch_i_from_collide_id(patches, time, report.obj2ID(j));
			if (indexes.find(pa1) == indexes.end())
				col_indexes.insert(pa1);
			if (indexes.find(pa2) == indexes.end())
				col_indexes.insert(pa2);
		}
	}

	return col_indexes;
}

void make_collision_model_patch_rp(int time, Patch& p, RAPID_model &b)
{
	for (int k = 0; k < p.half_motions.size(); ++k)
	{
		std::vector<ml::Posture> &m_postures = p.half_motions[k].m_postures;

		auto posture_p = std::min_element(m_postures.begin(), m_postures.end(), CmpPostureTime(time));

		if (fabs(posture_p->time - time) < 0.5+0.25)
			make_collison_model_rp(*posture_p, b);
	}
}



bool make_collision_model_patch_vc(int time, Patch& p, VCollide &vc)
{
	bool is_new_object = false;
	for (int k = 0; k < p.half_motions.size(); ++k)
	{
		std::vector<ml::Posture> &m_postures = p.half_motions[k].m_postures;

		if (m_postures.size() < 3)
			continue;

		auto posture_p = std::min_element(m_postures.begin()+1, m_postures.end()-1, CmpPostureTime(time));

		if (fabs(posture_p->time - time) < 0.5+0.25) {
			is_new_object = true;
		}
	}

	if (is_new_object == true)
	{
		vc.NewObject(&(p.collide_id[time]));
		for (int k = 0; k < p.half_motions.size(); ++k)
		{
			std::vector<ml::Posture> &m_postures = p.half_motions[k].m_postures;

			if (m_postures.size() < 3)
				continue;

			auto posture_p = std::min_element(m_postures.begin()+1, m_postures.end()-1, CmpPostureTime(time));

			if (fabs(posture_p->time - time) < 0.5+0.25) {
				make_collison_model_vc(*posture_p, vc);
				//cout << "make " << time << endl;
			}
		}
		vc.EndObject();
	}

	return is_new_object;

}

void make_collison_model_rp( ml::Posture & p, RAPID_model & b ) 
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

		//the geometry is a unit cube with one vertex at the origin.
		double v1[3], v2[3], v3[3];

		v1[0] = p1[0]; v1[1] = p1[1]; v1[2] = p1[2];
		v2[0] = p2[0]; v2[1] = p2[1]; v2[2] = p2[2];
		v3[0] = p3[0]; v3[1] = p3[1]; v3[2] = p3[2]; 
		b.AddTri(v1, v2, v3,0);

		v1[0] = p4[0]; v1[1] = p4[1]; v1[2] = p4[2];
		v2[0] = p2[0]; v2[1] = p2[1]; v2[2] = p2[2];
		v3[0] = p3[0]; v3[1] = p3[1]; v3[2] = p3[2]; 
		b.AddTri(v1, v2, v3,0);
	}
}

void make_collison_model_vc( ml::Posture & p, VCollide & vc ) 
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

		//the geometry is a unit cube with one vertex at the origin.
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
