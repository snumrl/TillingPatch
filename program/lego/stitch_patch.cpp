#include "stitch_patch.h"
#include "motion_edit.h"

int cut_index(const ml::Motion &m, bool forward)
{
	if (forward == false)
	{
		for (int r = 0; r != m.size()-1; ++r)
		{
			if (m[r].rigid == true)
				return r;
		}

	}
	else
	{
		for (int r = m.size()-1; r != 0; --r)
		{
			if (m[r].rigid == true)
				return r;
		}
	}
}

void motion_part(ml::Motion &m, const ml::Motion &base_m, bool forward, int cut_i)
{
	if (forward == false)
	{
		m.get_motion(base_m, 0, cut_i);
	}
	else
	{
		m.get_motion(base_m, cut_i, base_m.size()-1);
	}
}

void set_constraints( std::map<int, cml::vector3d> &cons_pos, std::map<int, double> &cons_time, const ml::Motion &m_part, bool forward, int stitch_m_size, int diff = 4) 
{
	if (forward == false)
	{
		cons_pos[stitch_m_size-1] = m_part[m_part.size()-1].trans();
		cons_pos[stitch_m_size-1-diff] = m_part[m_part.size()-1-diff].trans();
		cons_time[stitch_m_size-1] = m_part[m_part.size()-1].time;
	}
	else
	{
		cons_pos[0] = m_part[0].trans();
		cons_pos[0+diff] = m_part[0+diff].trans();
		cons_time[0] = m_part[0].time;
	}
}

void set_to_original_motion( ml::Motion & m, const ml::Motion & stitch_m, bool forward, int cut_index ) 
{
	if (forward == false)
		for (int i = 0; i <= cut_index; ++i)
		{
			m.posture(i, stitch_m.posture(stitch_m.size()-1-cut_index+i));
		}
	else
		for (int i = cut_index; i <= m.size()-1; ++i)
		{
			m.posture(i, stitch_m.posture(i-cut_index));
		}
}

void set_unstitched_motion( ml::Motion & m, const ml::Motion & stitch_m, bool forward) 
{
	int diff;
	if (forward == true)
		diff = 0;
	else
		diff = stitch_m.size()-m.size();

	for (int i = 0; i <= m.size()-1; ++i)
	{
		m.posture(i, stitch_m.posture(i+diff));
	}
}


void stitch_patch_half( Patch &pa1, Patch &pa2, int g1, int g2, int p1, int p2, int after_patch_i1, int after_patch_i2 )
{	
	PointGroup &group1 =  pa1.point_groups[g1];
	PointGroup &group2 =  pa2.point_groups[g2];

	Point &point1 =  group1.points[p1];
	Point &point2 =  group2.points[p2];

	ml::Posture &posture1 = point1.posture;
	ml::Posture &posture2 = point2.posture;

	////pa1.half_motions[g1].m_postures.push_back(posture1);
	////pa2.half_motions[g2].m_postures.push_back(posture2);

	pa1.motion_expand(g1, p1);
	pa2.motion_expand(g2, p2);
	
	ml::Motion &m1 = pa1.half_motions[g1];
	ml::Motion &m2 = pa2.half_motions[g2];
	bool forward1 = pa1.point_groups[g1].forward;
	bool forward2 = pa2.point_groups[g2].forward;

	ml::Motion stitch_m = m1;
	if (forward1 == false) 
		stitch_m.stitch(m2, false);
	else 
		stitch_m.stitch(m2, true);

	std::map<int, cml::vector3d> cons_pos;
	std::map<int, double> cons_time;
	if (pa1.name == "wsw" || pa1.name == "chair")
		set_constraints(cons_pos, cons_time, m1, forward1, stitch_m.size(), 12);
	else
		set_constraints(cons_pos, cons_time, m1, forward1, stitch_m.size());

	if (pa1.name == "wsw" || pa1.name == "chair")
		set_constraints(cons_pos, cons_time, m2, forward2, stitch_m.size(), 12);
	else
		set_constraints(cons_pos, cons_time, m2, forward2, stitch_m.size());

	std::map<int, cml::vector3d> cons_ori;
	editMotion(stitch_m, cons_time, cons_pos, cons_ori, 0.033);

	set_unstitched_motion(m1, stitch_m, forward1);
	set_unstitched_motion(m2, stitch_m, forward2);

	m1.connected_motion = std::pair<int, int>(after_patch_i2, g2);
	m2.connected_motion = std::pair<int, int>(after_patch_i1, g1);
}

void edit_connected_motion( std::vector<Patch> &patches, int index )
{
	Patch &pa = patches[index];
	for (int i = 0; i < pa.half_motions.size(); ++i)
	{
		auto connect = pa.half_motions[i].connected_motion;
		if (connect.first >= 0 )
		{
			edit_patch_half(pa, patches[connect.first], i, connect.second);
		}
	}
}

void edit_patch_half( Patch &pa1, Patch &pa2, int g1, int g2 )
{	
	ml::Motion &m1 = pa1.half_motions[g1];
	ml::Motion &m2 = pa2.half_motions[g2];
	bool forward1 = pa1.point_groups[g1].forward;
	bool forward2 = pa2.point_groups[g2].forward;

	ml::Motion stitch_m = m1;
	if (forward1 == false) 
		stitch_m.stitch(m2, false);
	else 
		stitch_m.stitch(m2, true);

	std::map<int, cml::vector3d> cons_pos;
	std::map<int, double> cons_time;
	set_constraints(cons_pos, cons_time, m1, forward1, stitch_m.size());
	set_constraints(cons_pos, cons_time, m2, forward2, stitch_m.size());
	std::map<int, cml::vector3d> cons_ori;
	editMotion(stitch_m, cons_time, cons_pos, cons_ori, 0.03);

	set_unstitched_motion(m1, stitch_m, forward1);
	set_unstitched_motion(m2, stitch_m, forward2);
}




