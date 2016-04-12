#include "scatter_patches.h"
#include "stitch_patch.h"
#include "lego.h"
#include "environment.h"
#include <algorithm>

struct is_same_g1 {
	is_same_g1(int g1_): g1(g1_) {}
	bool operator() (const Dist_info& di)
	{
		return di.g1 == g1;
	}

	int g1;
};

struct is_same_pa2_g2 {
	is_same_pa2_g2(int pa2_, int g2_): pa2(pa2_), g2(g2_) {}
	bool operator() (const Dist_info& di)
	{
		return di.pa2 == pa2 && di.g2 == g2;
	}

	int pa2, g2;
};

double distance_rot(const cml::matrix3 &m1, const cml::matrix3 &m2)
{
	return cml::length(cml::log_mat3(cml::PlaneProject_mat3(m2 * cml::inverse(m1))));
}

double get_dist_info(ml::Posture& p1, ml::Posture& p2)
{
	double type_penalty = 0.0;
	if(p1.type != p2.type)
		type_penalty = 100.0;

	double rotate_dist = distance_rot(p1.rotate(0), p2.rotate(0));

	return cml::length(p1.trans() - p2.trans()) + 3.0 * rotate_dist + 0.05 * fabs(p1.time - p2.time) + type_penalty;
}

void get_dist_info_vec(Patch &pa1, std::vector<Patch> &patches, double dist_thres, std::vector<Dist_info> &dist_infos)
{
	for (int g1_i = 0; g1_i < pa1.point_groups.size(); ++g1_i)
	{
		PointGroup &g1 = pa1.point_groups[g1_i];
		for (int pa2_i = 0; pa2_i < patches.size(); ++pa2_i)
		{
			Patch &pa2 = patches[pa2_i];
			for (int g2_i = 0; g2_i < patches[pa2_i].point_groups.size(); ++g2_i)
			{
				PointGroup &g2 = patches[pa2_i].point_groups[g2_i];
				if(g1.forward != g2.forward && pa1.is_group_connected(g1_i) == false && pa2.is_group_connected(g2_i) == false)
				{
					for(int p1_i=0; p1_i < g1.points.size(); ++p1_i)
					{
						Point &p1 = g1.points[p1_i];
						for(int p2_i=0; p2_i < g2.points.size(); ++p2_i)
						{
							Point &p2 = g2.points[p2_i];

							double dist_p1_p2 = get_dist_info(p1.posture, p2.posture);
							if (dist_p1_p2 < dist_thres)
							{
								Dist_info dist_info;
								dist_info.pa2 = pa2_i;
								dist_info.g1 = g1_i;
								dist_info.g2 = g2_i;
								dist_info.p1 = p1_i;
								dist_info.p2 = p2_i;
								dist_info.dist = dist_p1_p2;
								dist_infos.push_back(dist_info);
							}
						}
					}
				}
			}
		}
	}
}

void change_dist_infos(std::vector<Dist_info> &dist_infos, std::list<Dist_info> &dist_info_list)
{
	for (int i = 0; i < dist_infos.size(); ++i) {
		Dist_info& di = dist_infos[i];

		if (find_if(dist_info_list.begin(), dist_info_list.end(), is_same_pa2_g2(di.pa2, di.g2)) != dist_info_list.end())
			continue;

		auto it = find_if(dist_info_list.begin(), dist_info_list.end(), is_same_g1(di.g1));
		if (it == dist_info_list.end()) { 
			dist_info_list.push_back(di);
		}
		else {
			if (di.dist < it->dist )
			{
				dist_info_list.erase(it);
				dist_info_list.push_back(di);
			}
		}
	}
}

list<Dist_info> get_dist_info_list(Patch &pa1, std::vector<Patch> &patches, double dist_thres)
{
	std::vector<Dist_info> dist_info_vec;
	get_dist_info_vec(pa1, patches, dist_thres, dist_info_vec);

	std::list<Dist_info> dist_info_list;
	change_dist_infos(dist_info_vec, dist_info_list);

	return dist_info_list;
}

void add_end_num( Patch * p_patch, int time_end, int &connect_end_num, int &good_end_num, int &dead_end_num )
{
	for (int j=0; j<p_patch->point_groups.size(); ++j)
	{
		PointGroup &group = p_patch->point_groups[j];
		double time = group.parent_posture(*p_patch).time;
		int end_type = get_end_type(p_patch->is_group_connected(j), time, time_end);
		if (end_type == 0)
			connect_end_num++;
		else if (end_type == 1)
			good_end_num++;
		else if (end_type == 2)
			dead_end_num++;
		else
			assert(false);
	}
}

vector<int> get_end_type_num( std::vector<Patch> &patches, int time_end /*= -10*/ )
{
	int dead_end_num = 0;
	int good_end_num = 0;
	int connect_end_num = 0;
	
	for (int i=0; i<patches.size(); ++i)
	{
		Patch* p_patch;
		p_patch = &(patches[i]);
		add_end_num(p_patch, time_end, connect_end_num, good_end_num, dead_end_num);
	}
	vector<int> end_type_num;
	end_type_num.push_back(connect_end_num);
	end_type_num.push_back(good_end_num);
	end_type_num.push_back(dead_end_num);

	return end_type_num;
}

vector<int> get_end_type_num_patch( Patch &patch, int time_end /*= -10*/ )
{
	int dead_end_num = 0;
	int good_end_num = 0;
	int connect_end_num = 0;

	add_end_num(&patch, time_end, connect_end_num, good_end_num, dead_end_num);
	
	vector<int> end_type_num;
	end_type_num.push_back(connect_end_num);
	end_type_num.push_back(good_end_num);
	end_type_num.push_back(dead_end_num);

	return end_type_num;
}

vector<int> get_end_type_num_virtual(std::vector<Patch> &patches, map<int, Patch> &patches_stitched, Patch & new_patch)
{
	int dead_end_num = 0;
	int good_end_num = 0;
	int connect_end_num = 0;
	double time_end = patches_endtime(patches);
	for (int i=0; i<patches.size(); ++i)
	{
		Patch * p_patch;
		if (patches_stitched.find(i) == patches_stitched.end())
			p_patch = &(patches[i]);
		else
			p_patch = &(patches_stitched[i]);

		add_end_num(p_patch, time_end, connect_end_num, good_end_num, dead_end_num);
	}
	add_end_num(&new_patch, time_end, connect_end_num, good_end_num, dead_end_num);

	vector<int> end_type_num;
	end_type_num.push_back(connect_end_num);
	end_type_num.push_back(good_end_num);
	end_type_num.push_back(dead_end_num);

	return end_type_num;
}

void ScatterPatches::print_dead_end()
{
	vector<int> end_type_num = get_end_type_num(lg->result_patches, lg->env->getEndTime());

	std::cout << "connect: " << end_type_num[0] << " boundary: " << end_type_num[1] << " dead: " << end_type_num[2] << " sum: " << end_type_num[0] + end_type_num[1] + end_type_num[2] 	<< std::endl;
}

#include <set>
int dist_infos_set_size( std::vector<Dist_info> &dist_infos )
{
	std::set<std::vector<int>> dist_set;
	for (int i = 0; i < dist_infos.size(); ++i)
	{
		Dist_info &di = dist_infos[i];
		std::vector<int> dist_short;
		dist_short.push_back(di.g1);
		/*dist_short.push_back(di.pa2);
		dist_short.push_back(di.g2);*/
		dist_set.insert(dist_short);
	}
	return dist_set.size();
}

ScatterPatches::ScatterPatches(Lego *lg_)
	:ef_alpha(2.0), ef_beta(0.25), stop(false)

{
	lg = lg_;
}


ScatterPatches::~ScatterPatches(void)
{
}

void ScatterPatches::scatter_patches_simple()
{
	srand(1288755247+11+1+2-15);

	stitch_motion_name[0].push_back("walk_box");
	stitch_motion_name[1].push_back("walk");
	stitch_motion_name[1].push_back("uturn");
	stitch_motion_name[1].push_back("wsw");

	//make_wall();
	//make_circle();
	//make_move();
	//make_wall2();
	//make_sink();
	make_circle2();
	
	if (false)
	//if (true)
	{
		while(true)
		{
			lua_tinker::table con = lg->get_lua_control_simple();
			if (con.get<bool>("stop") == true || stop == true) {
				lg->save_motion(con.get<char *>("save_name"));
				return;
			}

			if (con.get<bool>("random"))
				simple_random();
			else 
				simple_stitch();
		}
	}
	else
	{
		simple_metro();
		simple_stitch();
	}
}

double get_energy(double total_end, double dead_end)
{
	double energy = dead_end / total_end;

	if (total_end == 0)
		energy = 100000.;

	return energy;
}

double get_energy_new(double stitch_n, double boundary_n, double  dead_end)
{
	double total_n = stitch_n + boundary_n + dead_end;
	double energy = 1 - stitch_n / total_n - 0.5 * boundary_n / total_n;

	if (total_n == 0)
		energy = 100000.;

	return energy;
}

void ScatterPatches::simple_metro()
{
	vector<string> patch_type;
	patch_type.push_back("jump");
	patch_type.push_back("chair");
	patch_type.push_back("box");
	patch_type.push_back("kick");
	patch_type.push_back("bow");
	patch_type.push_back("4beat");
	patch_type.push_back("beat");
	patch_type.push_back("push");
	/*patch_type.push_back("walk_box");
	patch_type.push_back("walk");
	patch_type.push_back("jump");
	patch_type.push_back("4beat");
	patch_type.push_back("box");
	patch_type.push_back("beat");
	patch_type.push_back("kick");
	patch_type.push_back("bow");
	patch_type.push_back("4beat");
	patch_type.push_back("beat");
	patch_type.push_back("box");
	patch_type.push_back("push");
	patch_type.push_back("uturn");
	patch_type.push_back("wsw");*/

	cout << "metro start! " << endl << endl;

	while(true)
	{
		CollisionDetect collision_detect;
		collision_detect.init_patches(lg->result_patches);
		int collision_num = 0;

		auto end_n = get_end_type_num(lg->result_patches, lg->env->getEndTime());
		int dead_n = end_n[2];
		int stitch_n = end_n[0];
		int boundary_n = end_n[1];
		int total_n = end_n[0] + end_n[1] + end_n[2];
		double cur_energy = get_energy_new(stitch_n, boundary_n, dead_n);

		//cout << "new turn " << total_n << ' ' << dead_n << ' ' << cur_energy << endl;
	
		int attempt =0;
		while(true)
		{
			random_shuffle(patch_type.begin(), patch_type.end());
			for (auto it_p = patch_type.begin(); it_p != patch_type.end(); ++it_p)
			{
				lua_tinker::table con = lg->get_lua_control_metro();
				if (attempt >= con.get<int>("attempt_thres"))
					return;

				attempt++;

				Patch pa1;
				scatter_a_patch_env(pa1, it_p->data(), *lg->env);

				list<Dist_info> dist_info_list = get_dist_info_list(pa1, lg->result_patches, 2.4);

				/////////////////////
				for (auto it = dist_info_list.begin(); it != dist_info_list.end(); ++it)
				{
					ml::Motion &m1 = pa1.half_motions[it->g1];
					m1.connected_motion = std::pair<int, int>(1, 1);
				}

				auto pa1_n = get_end_type_num_patch(pa1, lg->env->getEndTime());
				int pa1_stitch_n = pa1_n[0];
				int pa1_boundary_n = pa1_n[1];
				int pa1_dead_n = pa1_n[2];
				int pa1_total_n = pa1_n[0] + pa1_n[1] + pa1_n[2];

				int new_total_n = total_n + pa1_total_n;
				int new_dead_n = dead_n + pa1_dead_n - pa1_stitch_n;
				int new_stitch_n = stitch_n + pa1_stitch_n * 2;
				int new_boundary_n = boundary_n + pa1_boundary_n;
				double new_energy = get_energy_new(new_stitch_n, new_boundary_n, new_dead_n);
				//////////////////

				double k = con.get<double>("k");
				double T = 1.;
				double probability = exp(-(new_energy - cur_energy) / (k * T));
				double random_0_1 = random(0, 30000) / 30000.;
				bool is_accept = probability > random_0_1;
				//cout << "probabilty: new- " << new_energy << ' ' << cur_energy << ' ' << new_energy - cur_energy << ' ' << probability << ' ' << random_0_1 << ' ' << is_accept << endl;

				//cout << "attempt: " << attempt << endl; 
				if(is_accept) 
				{
					for (auto it = dist_info_list.begin(); it != dist_info_list.end(); ++it)
					{
						ml::Motion &m1 = pa1.half_motions[it->g1];
						m1.connected_motion = std::pair<int, int>(-1, -1);
					}

					if (collision_and_stitch(pa1, collision_detect, collision_num, dist_info_list))
					{
						cout << "metro energy " << new_total_n << ' ' << new_dead_n << ' ' << new_energy << endl;
						cout << "attemp_num: " << attempt << " motion " << it_p->data() << endl << endl;
						if (new_energy > cur_energy && is_accept)
						{
							cout << "$$$ bad but accept $$$" << endl << endl;
						}
						goto out_attempt;
					}
				}
			}
		}
out_attempt:;
	}
}

void ScatterPatches::simple_random()
{
	int total_attempt_i = 0;
	while(true)
	{
		CollisionDetect collision_detect;
		collision_detect.init_patches(lg->result_patches);

		int this_attempt_i = 0;
		int collision_num = 0;
		while(true)
		{ 
			total_attempt_i++;
			this_attempt_i++;

			cout << this_attempt_i << endl;
			lua_tinker::table con = lg->get_lua_control_simple(this_attempt_i);
			if (con.get<bool>("random") == false || con.get<bool>("stop") == true)
				return;

			lua_tinker::table type = con.get<lua_tinker::table>("type");
			int type_size = lua_tinker::call<int>(lg->L, "table_size", type);

			int type_i = ((total_attempt_i - 1) % type_size) + 1;
		
			Patch pa1;
			scatter_a_patch_env(pa1, type.get<char *>(type_i), *lg->env);

			lua_tinker::table thres = con.get<lua_tinker::table>("thres");
			lua_tinker::table thres_i = thres.get<lua_tinker::table>(1);
			int thres_num = thres_i.get<int>(1);
			double thres_value = thres_i.get<double>(2);

			if (con.get<bool>("save") == true) {
				lg->save_motion(con.get<char *>("save_name"));
			}
			if( test_and_stitch(pa1, collision_detect, collision_num, thres_num, thres_value) ) {
				cout << "attemp_num: " << this_attempt_i << " motion " << type.get<char *>(type_i) << endl << endl;
				goto out_attempt;
			}
		}
out_attempt:;
	}
}

void ScatterPatches::simple_stitch()
{
	map<int, int> delete_history;
	int traverse_i = 0;

	while(true)
	{
		
		//순회 목록 만들기
		vector<pair<int,int>> traverse_vec;
		for (int pa2_i = 0; pa2_i < lg->result_patches.size(); ++pa2_i)
		{
			for (int g2_i = 0; g2_i < lg->result_patches[pa2_i].point_groups.size(); ++g2_i)
			{
				PointGroup &g2 = lg->result_patches[pa2_i].point_groups[g2_i];
				int end_type = get_end_type(g2.is_connected(lg->result_patches[pa2_i]),g2.parent_posture(lg->result_patches[pa2_i]).time, lg->env->getEndTime());
				if (end_type == 2)
				{
					traverse_vec.push_back(pair<int,int>(pa2_i, g2_i));
				}
			}
		}

		//dead end가 없으면 멈춘다.
		if (traverse_vec.empty())
		{
			stop = true;
			return;
		}
		
		//dead end가 있으면 순회하면서 처리하기
		CollisionDetect * p_collision_detect = new CollisionDetect();
		p_collision_detect->init_patches(lg->result_patches);
		std::set<int> delete_patch_list;
		for (auto it = traverse_vec.begin(); it != traverse_vec.end(); ++it)
		{
			//중간에 그만두고 싶을때
			lua_tinker::table con = lg->get_lua_control_simple();
			if (con.get<bool>("random") == true || con.get<bool>("stop") == true) {
				return;
			}
			
			int pa2_i = it->first;
			int g2_i = it->second;
	
			if (con.get<bool>("save") == true) {
				lg->save_motion(con.get<char *>("save_name"));
			}
			if (stitch_to_dead(pa2_i, g2_i, con, *p_collision_detect) == true) {
				delete p_collision_detect;
				p_collision_detect = new CollisionDetect();
				p_collision_detect->init_patches(lg->result_patches);

				//unable_patch 저장
				Patch& pa2 = lg->result_patches[pa2_i];
				int connect_pa = pa2.half_motions[g2_i].connected_motion.first;
				string pa_name = lg->result_patches[connect_pa].name;
				pa2.point_groups[g2_i].unable_patch.insert(pa_name);
				//
								
				cout << "dead " << pa2_i << ' ' << g2_i << " stitched" << endl;
				vector<int> end_type_num = get_end_type_num(lg->result_patches, lg->env->getEndTime());

				if (end_type_num[2] == 0)
				{
					stop = true;
					return;
				}
			}
			else {
				delete_patch_list.insert(pa2_i);
				cout << "dead " << pa2_i << ' ' << g2_i << " will delete" << endl << endl;
			}
		}
		delete p_collision_detect;

		////delete_patch_list에 자주 나타나는 녀석은 문제가 계속 반복 될 수 있으니 그 패치와 연결된 아이 하나를 지우자.
		//for (auto it = delete_patch_list.begin(); it != delete_patch_list.end(); ++it)
		//{
		//	if (delete_history.find(*it) == delete_history.end())
		//		delete_history[*it] = 1;
		//	else
		//		delete_history[*it] += 1;
		//}		
		//		
		//++traverse_i;
		//if (traverse_i == 8) 
		//{
		//	cout << "%%% 8 %%%" << endl;
		//	for (auto it = delete_history.begin(); it != delete_history.end(); ++it)
		//	{
		//		//cout << it->first << " : " << it->second << endl;
		//		if (it->second >= 3)
		//		{
		//			int patch_i = it->first;
		//			set<int> neighbor = connected_patches(lg->result_patches, patch_i);
		//			neighbor.erase(patch_i);
		//			for (auto it_n = neighbor.begin(); it_n != neighbor.end(); ++it_n)
		//			{
		//				cout << "neighbor: " << *it_n << " of pa_i " << patch_i << endl;
		//				delete_patch_list.insert(*it_n);
		//			}
		//		}
		//	}
		//	delete_history.clear();
		//	traverse_i = 0;
		//}
		
		// 길이가 작고 isolated된 패치 ?熾裏?
		for (int i = 0; i < lg->result_patches.size(); ++i )
		{
			Patch &pa = lg->result_patches[i];
			int connected_num = (connected_patches(lg->result_patches, i)).size() - 1;
			bool motion_short = true;
			for (auto it = pa.half_motions.begin(); it != pa.half_motions.end(); ++it)
			{
				if (it->size() > 5)
					motion_short = false;
			}
			if (connected_num == 0 && motion_short == true)
				delete_patch_list.insert(i);
		}

		
		//패치 지우자(set 은 오름차순 정렬되어 있으므로 역순으로 지워??가자)
		for (auto it = delete_patch_list.rbegin(); it != delete_patch_list.rend(); ++it)
		{
			delete_patch(*it);
			cout << "deleted " << *it << endl;
		}
		delete_patch_list.clear();

		// 길이가 작고 isolated된 패치 지우자.
		for (int i = 0; i < lg->result_patches.size(); ++i )
		{
			Patch &pa = lg->result_patches[i];
			int connected_num = (connected_patches(lg->result_patches, i)).size() - 1;
			bool motion_short = true;
			for (auto it = pa.half_motions.begin(); it != pa.half_motions.end(); ++it)
			{
				if (it->size() > 5)
					motion_short = false;
			}
			if (connected_num == 0 && motion_short == true)
				delete_patch_list.insert(i);
		}

		//패치 지우자(set 은 오름차순 정렬되어 있으므로 역순으로 지워나가자)
		for (auto it = delete_patch_list.rbegin(); it != delete_patch_list.rend(); ++it)
		{
			delete_patch(*it);
			cout << "deleted " << *it << endl;
		}
		delete_patch_list.clear();
	}
}

bool ScatterPatches::stitch_to_dead( int pa2_i, int g2_i, lua_tinker::table &con, CollisionDetect &collision_detect )
{
	int collision_num = 0;

	Patch &pa2 = lg->result_patches[pa2_i];
	PointGroup &g2 = pa2.point_groups[g2_i];

	// 한 group의 point들은 한가지 모션으로 되어있다는 가정이 있다.
	int posture_type = g2.points[0].posture.type;

	//string patch_type_name;
	//if (posture_type == 0)
	//	patch_type_name = "walk_box";
	//else if (posture_type == 1)
	//	patch_type_name = "uturn";
	//else
	//	assert(false);
	//Patch pa1(lg->patch_type[patch_type_name]);

	lua_tinker::table thres = con.get<lua_tinker::table>("thres");
	int thres_size = lua_tinker::call<int>(lg->L, "table_size", thres);
	for (int i = 1; i <= thres_size; ++i)
	{
		lua_tinker::table thres_i = thres.get<lua_tinker::table>(i);
		int thres_num = thres_i.get<int>(1);
		double thres_value = thres_i.get<double>(2);

		//다양한 모션 선택
		auto motion_i_random = get_random_vec(stitch_motion_name[posture_type].size());
		for (int motion_i = 0; motion_i < motion_i_random.size();++motion_i)
		{
			string &type_name = stitch_motion_name[posture_type][motion_i_random[motion_i]];

			if (g2.unable_patch.find(type_name) != g2.unable_patch.end())
				continue;

			Patch pa1(lg->patch_type[type_name]);

			auto p2_i_random = get_random_vec(g2.points.size());
			for(int p2_i = 0; p2_i < p2_i_random.size(); ++p2_i)
			{
				Point &p2 = g2.points[p2_i_random[p2_i]];

				auto g1_i_random = get_random_vec(pa1.point_groups.size());
				for (int g1_i = 0; g1_i < g1_i_random.size(); ++g1_i)
				{
					PointGroup &g1 = pa1.point_groups[g1_i_random[g1_i]];
					if (g1.forward == g2.forward)
						continue;

					auto p1_i_random = get_random_vec(g1.points.size());
					for (int p1_i = 0; p1_i < p1_i_random.size(); ++p1_i)
					{
						Point &p1 = g1.points[p1_i_random[p1_i]];

						ml::Posture &pos1 = p1.posture;
						ml::Posture &pos2 = p2.posture;

						pa1.transform_between_posture(pos2, pos1);

						if (non_packing_collision(pa1, *lg->env) == true)
							continue;
						if (test_and_stitch(pa1, collision_detect, collision_num, thres_num, thres_value)) {
							cout << pa1.name << " stitched" << endl;
							return true;
						}
					}
				}
			}
		}
	}
	return false;
}

void ScatterPatches::scatter_patches_random()
{
	double cost_value = 0.0;
	while (scatter_a_patch_random(cost_value, "jump", 1, 2.0))	{
	}
	std::cout << "In case of Randomly scattering, Cost value: " << cost_value << std::endl;
}

bool ScatterPatches::scatter_a_patch_random( double& cost_value, const char* name, const int th_n, const double th_v )
{
	CollisionDetect collision_detect;
	collision_detect.init_patches(lg->result_patches);

	int attempt_i = 0;
	int collision_num = 0;
	while(true)
	{ 
		lua_tinker::table con = lg->get_lua_control();

		//if (con.get<bool>("random") == false || con.get<bool>("stop") == true)
		//	return false;

		if (attempt_i++ > 10000)
			return false;

		Patch pa1(lg->patch_type[name]);		// con.get<char *>("type")
		//pa1.scatter_random();

		//lua_tinker::table thres = con.get<lua_tinker::table>("thres");
		//int thres_size = lua_tinker::call<int>(lg->L, "table_size", thres);
		//for (int i = 1; i <= thres_size; ++i)
		{
			//lua_tinker::table thres_i = thres.get<lua_tinker::table>(i);
			int thres_num =  th_n;		//thres_i.get<int>(1);
			double thres_value = th_v;	//thres_i.get<double>(2);

			double cost_temp = 0.0;
			list<Dist_info> dist_info_list = get_dist_info_list(pa1, lg->result_patches, thres_value);
			if (test_stitch_ok(pa1, con, collision_detect, collision_num, thres_num, dist_info_list, cost_temp)) 
			{
				stitch_all(pa1, dist_info_list);
				print_dead_end();
				cost_value += cost_temp;

				std::cout << "attemp_num: " << attempt_i << std::endl;
				return true;
			}
		}
	}
}

double ScatterPatches::delta_energy_function_annealing(const int idx, Patch& patch, int& thres_num, double& thres_value)
{
	Patch origin_patch (patch);
	double origin_energyValue = energy_function();

	delete_patch(idx);
	double trial_energyValue = trial_energy_function_annealing(patch, thres_num, thres_value);

	list<Dist_info> dist_info_list = get_dist_info_list(origin_patch, lg->result_patches, thres_value);
	lg->result_patches[idx] = (origin_patch);		// patch rolling back~!
	stitch_all(origin_patch, dist_info_list);

	return (trial_energyValue - origin_energyValue);
}

double ScatterPatches::trial_energy_function_annealing(Patch& patch, int& thres_num, double& thres_value)
{
	CollisionDetect collision_detect;
	collision_detect.init_patches(lg->result_patches);
	int attempt_i = 0;
	int collision_num = 0;

	double stage_ratio = 1.0;
	char * name = "jump";
	lua_tinker::table con = lg->get_lua_control();

	while(true)	{ 		
		attempt_i++;

		Patch temp_patch (lg->patch_type[name]);
		//temp_patch.scatter_random(stage_ratio);			// reconfiguration policy

		list<Dist_info> dist_info_list = get_dist_info_list(temp_patch, lg->result_patches, thres_value);
		double energyValue = 0.0;
		if (test_stitch_ok(temp_patch, con, collision_detect, collision_num, thres_num, dist_info_list, energyValue)) 
		{
			std::cout << "attemp_num: " << attempt_i << std::endl;
			patch = temp_patch;
			return energyValue;
		}
	}
}

double ScatterPatches::delta_energy_function_greedy( Patch& patch, int& thres_num, double& thres_value )
{
	double origin_energyValue = energy_function();
	double trial_energyValue = trial_energy_function_greedy(patch, thres_num, thres_value);
	return (trial_energyValue - origin_energyValue);	
}

double ScatterPatches::trial_energy_function_greedy(Patch& patch, int& thres_num, double& thres_value)
{
	CollisionDetect collision_detect;
	collision_detect.init_patches(lg->result_patches);
	int attempt_i = 0;
	int collision_num = 0;

	double stage_ratio = 1.0;
	lua_tinker::table con = lg->get_lua_control();

	char * name = "jump";

	if (thres_num==3) {
		thres_num=2;
		stage_ratio = 1.4;
		name = "walk";
	}

	while(true)
	{ 		
		attempt_i++;

		if (attempt_i>10000 && thres_num==2) {
			name = "jump";
			thres_num = 1;
			thres_value = 2.3;
			stage_ratio = 1.0;
		}

		Patch temp_patch (lg->patch_type[name]);
		//temp_patch.scatter_random(stage_ratio);		// reconfiguration policy

		list<Dist_info> dist_info_list = get_dist_info_list(temp_patch, lg->result_patches, thres_value);
		double energyValue = 0.0;
		if (test_stitch_ok(temp_patch, con, collision_detect, collision_num, thres_num, dist_info_list, energyValue)) 
		{
			std::cout << "attemp_num: " << attempt_i << std::endl;
			patch = temp_patch;
			return energyValue;
		}
	}
}

double ScatterPatches::energy_function()
{
	vector<int> end_type_num = get_end_type_num(lg->result_patches);
	return energy_function(static_cast<double>(end_type_num[0]), static_cast<double>(end_type_num[1]), static_cast<double>(end_type_num[2]));
}

double ScatterPatches::energy_function(const double c, const double g, const double d)
{
	return (ef_alpha * d / (c + g)); // + (ef_beta * (d) / (c+g));
}

bool ScatterPatches::metrop(const double de, const double t)
{
	static int gljdum = -1;
	double ran = ran3(gljdum);
	return (de < 0.0) || (ran < std::exp(-1.0*de/t));
}

bool ScatterPatches::test_stitch_ok( Patch &pa1, lua_tinker::table &con, CollisionDetect &collision_detect, int &collision_num, int thres_num, list<Dist_info> &dist_info_list, double& costValue )
{   
	bool collision_print = false;
	double width = con.get<double>("width");
	cml::vector3 pa1_pos = pa1.patch_position;
	//double pa1_time = pa1.patch_time;
	if (pa1_pos[0] < -width || pa1_pos[0] > width)
		return false;
	if (pa1_pos[2] < -width || pa1_pos[2] > width)
		return false;
	//if (pa1_time < 0 || pa1_time > con.get<double>("width_time"))
	//	return false;
	

	if(dist_info_list.size() >= thres_num) 
	{
		Patch pa1_new(pa1);		//stitch에 의해서 p1이 변하니까 복사해서 사용
		map<int, Patch> patches_stitched;
		for (auto it = dist_info_list.begin(); it != dist_info_list.end(); ++it)
		{
			Dist_info &di = *it;

			if (patches_stitched.find(di.pa2) == patches_stitched.end())
				patches_stitched[di.pa2] = lg->result_patches[di.pa2];
			stitch_patch_half(pa1_new, patches_stitched[di.pa2], di.g1, di.g2, di.p1, di.p2, lg->result_patches.size(), di.pa2);
		}

		if (collision_print == true)
			cout << "col check" << endl;
		if (collision_detect.detect_vc_multi(pa1_new, patches_stitched) == false)
		{
			//////////////////////////////////////////////////////////////////////////
			vector<int> end_num = get_end_type_num_virtual(lg->result_patches, patches_stitched, pa1_new);
			costValue = energy_function(end_num[0], end_num[1], end_num[2]);
			//////////////////////////////////////////////////////////////////////////
			return true;
		}
		else
		{
			if (collision_print == true)
				cout << "col!" << endl;
			collision_num++;
		}
	}

	return false;
}

void update_connection(std::pair<int,int> &connected_motion, int delete_pa_i)
{
	int connected_pa_i = connected_motion.first;
	if (connected_pa_i == delete_pa_i) {
		connected_motion.first = -1;
		connected_motion.second = -1;
	}
	else if(connected_pa_i > delete_pa_i) {
		connected_motion.first -= 1;
	}
}

bool ScatterPatches::delete_patch( int pa_i )
{
	for (auto it = lg->result_patches.begin(); it != lg->result_patches.end(); ++it)
	{
		for (int i = 0; i < it->half_motions.size(); ++i)
		{
			ml::Motion &old_m = it->half_motions[i];
			if (old_m.connected_motion.first == pa_i) {

				ml::Posture old_pos;
				if (old_m.forward == true)
					old_pos = old_m.first_posture();
				else
					old_pos = old_m.last_posture();

				ml::Motion &new_m = lg->patch_type[it->name].half_motions[i];
				ml::Posture new_pos;
				if (new_m.forward == true)
					new_pos = new_m.first_posture();
				else
					new_pos = new_m.last_posture();

				it->half_motions[i] = new_m;

				it->half_motions[i].transform_between_posture(old_pos, new_pos);
			}
			else if (old_m.connected_motion.first > pa_i)
			{
				old_m.connected_motion.first -= 1;
			}
		}
	}

	lg->result_patches.erase(lg->result_patches.begin()+pa_i);
	return true;
}

void ScatterPatches::stitch_all( Patch &pa1, list<Dist_info> &dist_info_list )
{
	Patch pa1_new(pa1); //stitch에 의해서 p1이 변하니까 복사해서 사용.
	for (auto it = dist_info_list.begin(); it != dist_info_list.end(); ++it)
	{
		Dist_info &di = *it;
		stitch_patch_half(pa1_new, lg->result_patches[di.pa2], di.g1, di.g2, di.p1, di.p2, lg->result_patches.size(), di.pa2);
	}
	lg->result_patches.push_back(pa1_new);
}

bool ScatterPatches::collision_and_stitch( Patch &pa1, CollisionDetect &collision_detect, int &collision_num, list<Dist_info>& dist_info_list)
{
	Patch pa1_new(pa1); //stitch에 의해서 p1이 변하니까 복사해서 사용.
	map<int, Patch> patches_stitched;
	for (auto it = dist_info_list.begin(); it != dist_info_list.end(); ++it)
	{
		Dist_info &di = *it;

		if (patches_stitched.find(di.pa2) == patches_stitched.end())
			patches_stitched[di.pa2] = lg->result_patches[di.pa2];
		stitch_patch_half(pa1_new, patches_stitched[di.pa2], di.g1, di.g2, di.p1, di.p2, lg->result_patches.size(), di.pa2);
	}

	if (non_packing_collision(pa1_new, *lg->env) == true) {
		collision_num++;
		return false;
	}
	for (auto it = patches_stitched.begin(); it != patches_stitched.end(); ++it) {
		if (non_packing_collision(it->second, *lg->env) == true) {
			collision_num++;
			return false;
		}
	}

	if (collision_detect.detect_vc_multi(pa1_new, patches_stitched) == false)
	{
		for (auto it = patches_stitched.begin(); it != patches_stitched.end(); ++it)
			lg->result_patches[it->first] = it->second;

		get_next_color(pa1_new.r,pa1_new.g,pa1_new.b);
		lg->result_patches.push_back(pa1_new);

		// print out
		{ 
			cout << "patch_i: " << lg->result_patches.size() << " stitch_size: " << dist_info_list.size() << " collision_num: " << collision_num << endl;
			print_dead_end();
		}
		return true;
	}
	else
	{
		collision_num++;
		return false;
	}
}

bool ScatterPatches::test_and_stitch( Patch &pa1, CollisionDetect &collision_detect, int &collision_num, int thres_num, double thres_value )
{   
	list<Dist_info> dist_info_list = get_dist_info_list(pa1, lg->result_patches, thres_value);

	//cout << thres_size << ' ' << dist_info_list.size() << endl;
	if(dist_info_list.size() >= thres_num) 
	{
		Patch pa1_new(pa1); //stitch에 의해서 p1이 변하니까 복사해서 사??
		map<int, Patch> patches_stitched;
		for (auto it = dist_info_list.begin(); it != dist_info_list.end(); ++it)
		{
			Dist_info &di = *it;

			if (patches_stitched.find(di.pa2) == patches_stitched.end())
				patches_stitched[di.pa2] = lg->result_patches[di.pa2];
			stitch_patch_half(pa1_new, patches_stitched[di.pa2], di.g1, di.g2, di.p1, di.p2, lg->result_patches.size(), di.pa2);
		}

		if (non_packing_collision(pa1_new, *lg->env) == true) {
			collision_num++;
			return false;
		}
		for (auto it = patches_stitched.begin(); it != patches_stitched.end(); ++it) {
			if (non_packing_collision(it->second, *lg->env) == true) {
				collision_num++;
				return false;
			}
		}
	
		if (collision_detect.detect_vc_multi(pa1_new, patches_stitched) == false)
		{
			for (auto it = patches_stitched.begin(); it != patches_stitched.end(); ++it)
				lg->result_patches[it->first] = it->second;

			get_next_color(pa1_new.r,pa1_new.g,pa1_new.b);
			lg->result_patches.push_back(pa1_new);
		
			// print out
			{ 
				cout << "patch_i: " << lg->result_patches.size() << " stitch_size: " << dist_info_list.size() << ' ' << thres_num << " collision_num: " << collision_num << endl;
				print_dead_end();
			}
			return true;
		}
		else
		{
			collision_num++;
		}
	}

	return false;
}

std::vector<int> get_random_vec( int size )
{
	std::vector<int> rv;
	for (int i = 0; i < size; ++i)
		rv.push_back(i);
	random_shuffle(rv.begin(), rv.end());

	return rv;
}


void ScatterPatches::scatter_a_patch_box( Patch &pa, const char* name, Box& box )
{
	while (true) {
		Patch pa1(lg->patch_type["jump"]);
		pa1.scatter_random_in_square(box.x1, box.x2, box.z1, box.z2, box.t1, box.t2);

		if (pa1.get_left_boundary() >= box.x1 &&
			pa1.get_right_boundary() <= box.x2 &&
			pa1.get_down_boudary() >= box.z1 &&  
			pa1.get_up_boundary() <= box.z2 &&
			pa1.get_begintime() >= box.t1 &&  
			pa1.get_endtime() >= box.t2 	) {
				pa = pa1;
				return;
		}
	}
}

void ScatterPatches::scatter_a_patch_env( Patch &pa, const char* name, Environment& env )
{
	while (true) {
		Patch pa1(lg->patch_type[name]);
		if (env.scatter_a_patch(pa1)) {
			pa = pa1;
			return;
		}
	}
}

std::vector<double> for_diff(double diff)
{
	std::vector<double> diffs;
	diffs.push_back(0);
	diffs.push_back(diff*0.5);
	diffs.push_back(-diff*0.5);
	diffs.push_back(diff);
	diffs.push_back(-diff);
	return diffs;
}

std::vector<cml::vector3> get_trans_v(double x, double z, double t)
{
	std::vector<double> x_diff = for_diff(x);
	std::vector<double> z_diff = for_diff(z);
	std::vector<double> t_diff = for_diff(t);

	vector<cml::vector3> pos_v;
	for (auto it_x = x_diff.begin(); it_x != x_diff.end(); ++it_x)
	{
		for (auto it_z = z_diff.begin(); it_z != z_diff.end(); ++it_z) 
		{
			for (auto it_t = t_diff.begin(); it_t != t_diff.end(); ++it_t)

			{
				pos_v.push_back(cml::vector3(*it_x, *it_z, *it_t));
			}
		}
	}
	pos_v.push_back(cml::vector3(0,0,0));

	vector<cml::vector3> trans_v;
	for (int i = 1; i < pos_v.size(); ++i)
		trans_v.push_back(pos_v[i] - pos_v[i-1]);
	
	/*for (int i = 0; i < trans_v.size(); ++i)
		cout << trans_v[i][0] << ' ' << trans_v[i][1] << ' ' << trans_v[i][2] << endl;*/
	
	return trans_v;
}

//0; 연결도 ??했?? 1: 연결?? 했는?? 충돌이 있었고 충돌회피를 안하?킬?못????. 2: 연결하고 충돌이 없거나 ??했다.
int ScatterPatches::settle( Patch &pa, vector<Patch> &patches, CollisionDetect &collision, int thres_num )
{
	list<Dist_info> dist_info_list = get_dist_info_list(pa, patches, 2.0);
	if (dist_info_list.size() >= thres_num)
	{
		cout << "stitch num: " << dist_info_list.size() << endl;
		for (auto it = dist_info_list.begin(); it != dist_info_list.end(); ++it)
		{
			Dist_info &di = *it;
			stitch_patch_half(pa, patches[di.pa2], di.g1, di.g2, di.p1, di.p2, patches.size(), di.pa2);
		}
		patches.push_back(pa);

		//collision check
		int push_patch_index = patches.size()-1;
		set<int> connected_indexes = connected_patches(patches, push_patch_index);
		set<int> col_indexes = collision.detect_vc_index_all(patches, connected_indexes);

		//collision resolve
		if (col_indexes.size() >= 1)
		{
			//정책 결정
			bool is_resolve, is_self_shake;
			/*if (dist_info_list.size() == 0)	{
				is_resolve = false;
			}
			else if (dist_info_list.size() == 1) {
				is_resolve = true;
				is_self_shake = true;
			}
			else if (dist_info_list.size() >= 2) {
				is_resolve = true;
				is_self_shake = false;
			}*/
			is_resolve = false;

			//정책 시행
			if (is_resolve == false) {
				//제거해야 한??.
				collision.delete_vc(patches, connected_indexes);
				delete_patch(push_patch_index);
				connected_indexes.erase(push_patch_index);
				if (connected_indexes.size() > 0)
					collision.add_vc(patches, connected_indexes);
				return 1;
			}
			else {
				if (collision_resolve(patches, collision, col_indexes, is_self_shake) == true) {
					cout << "^^ resolve" << endl;
					return 2;
				}
				else {
					cout << "TT No resolve" << endl;
					
					//제거해야 한다.
					collision.delete_vc(patches, connected_indexes);
					delete_patch(push_patch_index);
					connected_indexes.erase(push_patch_index);
					if (connected_indexes.size() > 0)
						collision.add_vc(patches, connected_indexes);

					return 1;
				}
			}
		}
		else
		{
			return 2;
		}
	}
	else
		return 0;
}

bool collision_resolve( std::vector<Patch> &patches, CollisionDetect &collision_detect, set<int> &near_indexes,bool only_self_shake /*= false */, int near_num_thres /*= 4 */ )
{
	int push_patch_index = patches.size()-1;
	bool self_shake = shake(patches, push_patch_index, collision_detect);
	if (self_shake == true)
		return true;
	else
	{
		if (only_self_shake == true)
			return false;
		
		if (near_indexes.size() > near_num_thres) //충돌한 이웃수가 thres보?? 넘어가?? ??기.
			return false;

		bool resolve = true;
		for (auto it = near_indexes.begin(); it != near_indexes.end(); ++it)
		{
			resolve = resolve && shake(patches, *it, collision_detect);
			if (resolve == false)
				return false;
		}
		return true;
	}
}

bool shake( std::vector<Patch> &patches, int shake_index, CollisionDetect &collision_detect )
{
	std::vector<cml::vector3> trans_v = get_trans_v(0.5, 0.5, 20);
	set<int> check_indexes = connected_patches(patches, shake_index);
	for(auto it = trans_v.begin(); it != trans_v.end(); ++it)
	{
		patches[shake_index].translate(cml::vector3((*it)[0],0,(*it)[1]));
		patches[shake_index].translate_time((*it)[2]);

		edit_connected_motion(patches, shake_index);
		if (collision_detect.detect_vc_index(patches, check_indexes) == false)
		{
			cout << "good ! " << endl;
			return true;
		}
	}
	cout << "bad ! " << endl;
	return false;
}


void ScatterPatches::scatter_test()
{
	for (int i=0; i<300; ++i) {
		Patch pa;
		scatter_a_patch_env(pa, "uturn", *lg->env);
		lg->result_patches.push_back(pa);
	}
}

void ScatterPatches::make_wall()
{
	int end_time = 650;
	double width = 5.0;
	double width_margin = 5.8;

	lg->env->setTotalTimeRange(0, end_time);
	lg->env->make_rectangle_stage(-width, width, -width, width, 0, end_time);
	tShape* v_stage = new Rectangle(true, -width_margin, width_margin, -width_margin, width_margin, 0, end_time);
	lg->env->insert_just_draw(v_stage);
}

void ScatterPatches::make_wall2()
{
	int end_time = 650;
	double width = 7.5;
	double width_margin = 7.8;

	lg->env->setTotalTimeRange(0, end_time);
	lg->env->make_rectangle_stage(-width, width, -width, width, 0, end_time);
	tShape* v_stage = new Rectangle(true, -width_margin, width_margin, -width_margin, width_margin, 0, end_time);
	lg->env->insert_just_draw(v_stage);
}

void ScatterPatches::make_circle()
{
	lg->env->setTotalTimeRange(0, 650);
	lg->env->make_circle_stage(0., 0., 0., 8, 0., 650);
	lg->env->make_circle_obstacle(false, 0., 0., 1.8, 0, 650, cml::vector3f(0.5, 0.45, 0.43));
	Circle* just_draw = new Circle(true, cml::vector3d(0.0, 0.0, 0.0), 8.5, 0, 650);
	lg->env->insert_just_draw(just_draw);
}

void ScatterPatches::make_circle2()
{
	lg->env->setTotalTimeRange(0, 650);
	lg->env->make_circle_stage(0., 0., 0., 6, 0., 650);
	Circle* just_draw = new Circle(true, cml::vector3d(0.0, 0.0, 0.0), 6, 0, 650);
	lg->env->insert_just_draw(just_draw);
}

void ScatterPatches::make_move()
{
	lg->env->setTotalTimeRange(0, 650);
	lg->env->make_circle_stage(0., 0., 0., 7.2, 0., 650);
	lg->env->make_moving_circle_obstacle(false, 85, -5.5, 0., 1.5, 650-85, 5.5, 0., 1.5, cml::vector3(1,0,0));
	lg->env->make_moving_circle_obstacle(false, 85, 0., 5.5, 1.5, 650-85, 0., -5.5, 1.5, cml::vector3(1,0,0));
	Circle* just_draw = new Circle(true, cml::vector3d(0.0, 0.0, 0.0), 7.5, 0, 650);
	lg->env->insert_just_draw(just_draw);
}

void ScatterPatches::make_sink()
{
	lg->env->setTotalTimeRange(0, 650);
	lg->env->make_rectangle_stage(-9., -1., -4., 4., 0, 20);
	lg->env->make_rectangle_stage(1., 9., -4., 4., 630, 650);
	House* lhouse = new House;
	lhouse->set_house(-15., -6., -5., 5., 2., 0., 0, 50);
	lg->env->insert_just_draw(lhouse);
	House* rhouse = new House;
	rhouse->set_house(7., 16., -5., 5., 2., 180., 0, 50);
	lg->env->insert_just_draw(rhouse);
}

void ScatterPatches::make_move_rec()
{
	double width = 6.;
	lg->env->setTotalTimeRange(0, 650);
	lg->env->make_rectangle_stage(-width, width, -width, width, 0., 650);
	lg->env->make_moving_rectangle_obstacle(false, 85, -5.5, -3.5, -1., 1., 650-85, 3.5, 5.5,-1., 1., cml::vector3f(0.5, 0.45, 0.43));
	lg->env->make_moving_rectangle_obstacle(false, 85, -1., 1., -5.5, -3.5, 650-85, -1., 1., 3.5, 5.5, cml::vector3f(0.45, 0.5, 0.46));
	Rectangle* just_draw = new Rectangle(true, -width, width, -width, width, 0, 650);
	lg->env->insert_just_draw(just_draw);

}
