#include "dead_end_finder.h"


DeadEndFinder::DeadEndFinder(void)
{
	first = true;
	endes_index = 0;
}


DeadEndFinder::~DeadEndFinder(void)
{
}

pair<int,int> DeadEndFinder::get_dead_end( std::vector<Patch> &result_patches )
{
	while (true)
	{
		if (endes_index == endes.size())
		{
			update_result_patches(result_patches);
			endes_index = 0;
		}

		std::pair<int,int> patch_end = endes[endes_index];
		Patch &pa2 = result_patches[patch_end.first];
		PointGroup &g2 = pa2.point_groups[patch_end.second];

		if (get_end_type(g2.is_connected(pa2),g2.parent_posture(pa2).time, end_time) == 2) {
			cout << patch_end.first << ' ' << patch_end.second << endl;
			++endes_index;
			return patch_end;
		}
					
		++endes_index;
	}
}

void DeadEndFinder::set_endes( std::vector<Patch> &result_patches )
{
	endes.clear();
	for (int pa2_i = 0; pa2_i < result_patches.size(); ++pa2_i)
		for (int g2_i = 0; g2_i < result_patches[pa2_i].point_groups.size(); ++g2_i)
			endes.push_back(std::pair<int,int>(pa2_i, g2_i));
}

void DeadEndFinder::update_result_patches( std::vector<Patch> & result_patches )
{
	cout << "update_result_patches" << endl;
	set_endes(result_patches);
	end_time = patches_endtime(result_patches);
}
