#include "StdAfx.h"
#include "motions_viewer.h"
#include "patches_viewer.h"
#include "online_viewer.h"
#include "motion_edit.h"
#include "tiling.h"
#include "get_patch.h"
#include "nonslipfoot.h"
#include "execute.h"

TEST(chicken_online)
{
  if (false)
  {
    /*DontSlip noslipfoot(18, 19, 24, 3, 4, 20);
    for (auto it = patches.begin(); it != patches.end(); ++it) noslipfoot.hold_foot(**it);*/
    PatchesViewerOnline * v = new PatchesViewerOnline();
    v->get_camera().distance += 2;
  }
}

TEST(chicken_offline)
{
    vector<shared_ptr<Patch> > input_patches;
    read_patches(input_patches, "hand_chicken");
    remove_dangling_patch(input_patches);

    Env env;
    map<string,Patch> patch_types;
    vector<string> sampling_patches;
    for (auto it = input_patches.begin(); it != input_patches.end(); ++it) {
        patch_types[(*it)->name] = (**it);
        bool is_connectable = true; 
        for (auto jt = (*it)->boundaries.begin(); jt != (*it)->boundaries.end(); ++jt) {
            if (jt->posture_type() == -1) is_connectable = false;	  
        }
        if (is_connectable) sampling_patches.push_back((*it)->name);
    }    

    vector< shared_ptr<Patch> > patches;
    srand(79797+16+3+7);
    //environment
    //env.set_scatter_range(-7,7,-2,2, 100, 800);
    env.static_boxes.push_back(Box(trans_transf(cml::vector3(0, 0.625, -5.5)), cml::vector3(40, 1.25, 1))); // 장애물 위치 지정
    env.static_boxes.push_back(Box(trans_transf(cml::vector3(0, 0.625, 5.5)), cml::vector3(40, 1.25, 1)));
    //env.static_boxes.push_back(Box(trans_transf(cml::vector3(-6.5, 0.625, 0)), cml::vector3(1, 1.25, 14)));
    //env.static_boxes.push_back(Box(trans_transf(cml::vector3(6.5, 0.625, 0)), cml::vector3(1, 1.25, 14)));		

    cml::vector3 target1(40,0,0); // 최종 도착 지점
    cml::vector3 target(-40,0,0);

    const int team_num = 20; // 한 팀의 치킨 개수 
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
                    env.scatter_range(*pa1, 6, 6+2, -5, 5, 99, 101);     // 초기값. x축 16에서 18까지 z축 -4에서 4까지 100프레임 부터 
                else
                    env.scatter_range(*pa1, -(6+2), -6, -5, 5, 99, 101); // 초기값. x축 -18에서 -16까지 z축 -4에서 4까지 100프레임 부터 


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

    map<string, Patch> * patch_type = &patch_types;

    //////////////////////////////////////////////////////////////////////////////////////////////////////////
    //set_color_one(patches);
    env.range["t_max"] = 1000;
    for (int ii = 0; ii < 1500; ++ii)

    {
        vector<pair<int,int>> traverse_vec = get_traverser_vec(patches, env);
        for (auto it = traverse_vec.begin(); it != traverse_vec.end(); ++it)
        {
            vector<Stitch_energy> energies;
            //get_energies_general(sampling_patches, patch_type, it->first, it->second, patches, env, energies);
            get_energies_path_target_fight(sampling_patches, patch_type, it->first, it->second, patches, env, energies, target, target1);
            sort(energies.begin(), energies.end(), [](const Stitch_energy &a, const Stitch_energy &b){return a.energy > b.energy;});

            const auto it_b2 = &(patches[it->first]->boundaries[it->second]);
            const auto &p2 = it_b2->posture();
            for (auto it_e = energies.begin(); it_e != energies.end(); ++it_e)
            {
                if (it_e->energy < 0.)
                    continue;

                //이전에 연결했던 시도 안한다.
                if (it_b2->stitch_history.count(pair<string,int>(it_e->name, it_e->b)) > 0)
                    continue;

                shared_ptr<Patch> pa1(new Patch((patch_type->find(it_e->name))->second));
                const auto &p1 = pa1->boundaries[it_e->b].posture();
                pa1->transform_between_posture(p2, p1);
                vector<ConnectInfo> connect_infos;
                get_connect_infos(connect_infos, pa1, patches, 5.5, env);

                if (stitch(patches, pa1, patch_type, connect_infos, env, false) == true) {
                    if (pa1->motions.size() >= 2)
                        cout << "                            double!" << endl;
                    else
                        cout << "single!" << endl; //it_e->name << endl;

                    //어쩌다 연결된 dangling말고 지금 연결하려고 하는 dangling에만 history를 기록하자.
                    it_b2->stitch_history.insert(pair<string,int>(it_e->name, it_e->b));
                    goto resolve_dead;
                }
            }
            //jittering
            for (auto it_e = energies.begin(); it_e != energies.end(); ++it_e)
            {
                if (it_e->energy < 0.)
                    continue;

                //이전에 연결했던 시도 안한다.
                if (it_b2->stitch_history_jitter.count(pair<string,int>(it_e->name, it_e->b)) > 0)
                    continue;

                shared_ptr<Patch> pa1(new Patch((patch_type->find(it_e->name))->second));
                const auto &p1 = pa1->boundaries[it_e->b].posture();
                pa1->transform_between_posture(p2, p1);
                vector<ConnectInfo> connect_infos;
                get_connect_infos(connect_infos, pa1, patches, 5.5, env);

                if (stitch(patches, pa1, patch_type, connect_infos, env, false, true, 1, 24) == true) {
                    if (pa1->motions.size() >= 2)
                        cout << "                              jitter double!" << endl;
                    else
                        cout << "jitter single!" << endl; //it_e->name << endl;

                    //어쩌다 연결된 dangling말고 지금 연결하려고 하는 dangling에만 history를 기록하자.
                    it_b2->stitch_history_jitter.insert(pair<string,int>(it_e->name, it_e->b));
                    goto resolve_dead;
                }
            }

            delete_patch(patches, it->first, cout);
            goto resolve_dead;
        }
resolve_dead:;
        set_color_from_back(patches);
        cout << endl << "ii " << ii << endl;

        int dead_num = get_end_num(patches, env)["dead"];
        cout << "dead num: " << dead_num << endl;

        if ( dead_num == 0) {
            break;
        }
    }
    char buf[10];
    string file_name = "chicken_offline";
    file_name += itoa(team_num,buf,10);
    write_patches_env(patches, env, file_name.c_str()); // 결과 저장 

    //if (false)
    {
        PatchesViewer * v = new PatchesViewer(patches, env);
        v->get_camera().distance += 2;
    }
}

std::string write_consts( const vector<Constraint> &consts, const string &name )
{
	string file_name;
	file_name = string("./data/save/") + name + ".const";

	//cout << "save file name : " << file_name << endl;
	std::ofstream file(file_name, std::ios::out);

	//motions
	file << consts.size() << endl;
	for (auto it = consts.begin(); it != consts.end(); ++it)
	{
		file << it->type << ' ' << it->m_int[0] << ' ' << it->m_int[1] << ' ' << it->m_int[2] << ' ' << it->m_int[3] << endl;
	}

	file.close();
	return file_name;
}

TEST(DISABLED_patch_to_motion)
{
	vector<shared_ptr<Patch>> patches;
	Env env;
	//read_patches_env(patches, env, "motion_small");
	read_patches_env(patches, env, "motion_large");

	DontSlip noslipfoot(18, 19 , 24, 3, 4, 20);
	for (auto it = patches.begin(); it != patches.end(); ++it)
		noslipfoot.hold_foot(**it);

	vector<ml::Motion> motions;
	vector<Constraint> inner_cons;
	convert_connected_motions(patches, motions, inner_cons);

	ml::write_motions(motions, "large");
	write_consts(inner_cons, "large");

	MotionsViewer * m = new MotionsViewer(motions);
	m->get_camera().distance += 10;
}

class TilingTest : public testing::Test 
{
protected: 

	static map<string, Patch> *patch_type;
	static vector<Patch> * patch_type_unary;

	static void SetUpTestCase() {
		patch_type = new map<string, Patch>();
		get_patch(patch_type);
		patch_type_unary = new vector<Patch>();
		get_patch_unary(patch_type_unary, patch_type);
	}

	static void TearDownTestCase() {
		delete patch_type;
		patch_type = NULL;

		delete patch_type_unary;
		patch_type_unary = NULL;
	}

};

map<string, Patch> * TilingTest::patch_type = NULL;
vector<Patch> * TilingTest::patch_type_unary = NULL;

TEST_F(TilingTest, DISABLED_init_patch_type)
{
	vector<shared_ptr<Patch>> patches;
	patches.push_back(shared_ptr<Patch>(new Patch((*patch_type)["jump"])));

	EXPECT_TRUE(patches.size() == 1);

	if (false)
	PatchesViewer * v = new PatchesViewer(patches);
}

TEST_F(TilingTest, DISABLED_patch_render)
{
	Env env;
	env.static_boxes.push_back(Box(trans_transf(cml::vector3(0,0,-6.5)), cml::vector3(13, 2.5, 1)));
	env.static_boxes.push_back(Box(trans_transf(cml::vector3(0,0,6.5)), cml::vector3(13, 2.5, 1)));
	env.static_boxes.push_back(Box(trans_transf(cml::vector3(-6.5,0,0)), cml::vector3(1, 2.5, 13)));
	env.static_boxes.push_back(Box(trans_transf(cml::vector3(6.5,0,0)), cml::vector3(1, 2.5, 13)));

	vector<shared_ptr<Patch>> patches;
	read_patches(patches, "patch_render");

	DontSlip noslipfoot(18, 19 , 24, 3, 4, 20);
	for (auto it = patches.begin(); it != patches.end(); ++it)
		noslipfoot.hold_foot(**it);

	PatchesViewer * v = new PatchesViewer(patches, env);
	v->get_camera().distance += 2;
}

TEST_F(TilingTest, DISABLED_execute)
{
	execute(patch_type, patch_type_unary);
}

TEST_F(TilingTest, DISABLED_SkeletonPatch)
{
	Env env;
	env.set_scatter_range( -5, 5, -5, 5, 100, 600);
	env.static_boxes.push_back(Box(trans_transf(cml::vector3(0,0.625,-6.5)), cml::vector3(14, 1.25, 1)));
	env.static_boxes.push_back(Box(trans_transf(cml::vector3(0,0.625,6.5)), cml::vector3(14, 1.25, 1)));
	env.static_boxes.push_back(Box(trans_transf(cml::vector3(-6.5,0.625,0)), cml::vector3(1, 1.25, 14)));
	env.static_boxes.push_back(Box(trans_transf(cml::vector3(6.5,0.625,0)), cml::vector3(1, 1.25, 14)));

	vector< shared_ptr< Patch>> patches;
	SkeletonPatch(patches, env, patch_type, patch_type_unary);

	EXPECT_GE(patches.size(), 3);
	EXPECT_EQ(get_end_num(patches, env)["dead"], 0);

	int patch_num = patches.size();
	int interaction_patch_num = 0;
	for (auto it = patches.begin(); it != patches.end(); ++it)
		if ((*it)->motions.size() >= 2)
			++interaction_patch_num;

	cout << "interaction Num " << interaction_patch_num << ' ' << patch_num << endl; 
	//EXPECT_GE((double)interaction_patch_num / (double)patch_num, 1.0/4.0);
	
	if (false)
	{
		DontSlip noslipfoot(18, 19, 24, 3, 4, 20);
		for (auto it = patches.begin(); it != patches.end(); ++it)
			noslipfoot.hold_foot(**it);
		set_color(patches);
		PatchesViewer *v = new PatchesViewer(patches, env);
		v->get_camera().distance += 2;
	}
}

TEST_F(TilingTest, DISABLED_Path)
{
	Env env;
	env.set_scatter_range( -5, 5, -5, 5, 100, 1000);

	vector< shared_ptr< Patch>> patches;
	path(patches, env, patch_type, patch_type_unary);

	EXPECT_GE(patches.size(), 3);
	EXPECT_EQ(get_end_num(patches, env)["dead"], 0);

	//if (false)
	{
		DontSlip noslipfoot(18, 19, 24, 3, 4, 20);
		for (auto it = patches.begin(); it != patches.end(); ++it)
			noslipfoot.hold_foot(**it);
		set_color(patches);
		PatchesViewer *v = new PatchesViewer(patches, env);
		v->get_camera().distance += 2;
	}
}

TEST_F(TilingTest, DISABLED_remove_dangling)
{
	srand(79797+4);
	vector<shared_ptr<Patch>> patches;
	
	shared_ptr<Patch> pa1(new Patch((*patch_type)["jump"]));
	pa1->translate_time(400);
	patches.push_back(pa1);

	remove_dangling(patches, *patch_type_unary, patch_type, Env(), ofstream());
		
	EXPECT_EQ(0, get_end_num(patches, 100, 700)["dead"]);
	//EXPECT_EQ(61,patches[13]->motions[0].size());
	
	if (false)
	{
		PatchesViewer * v = new PatchesViewer(patches);
		v->get_camera().distance += 2;
	}
}

TEST_F(TilingTest, DISABLED_remove_dangling2)
{
	srand(79797+4);
	vector<shared_ptr<Patch>> patches;

	{
		shared_ptr<Patch> pa1(new Patch((*patch_type)["jump"]));
		pa1->translate_time(400);
		patches.push_back(pa1);
	}
	
	{
		shared_ptr<Patch> pa1(new Patch((*patch_type)["jump"]));
		pa1->translate_time(400);
		pa1->rotate(cml::pi()*0.5);
		pa1->translate(cml::vector3(1.2,0,1.9));
		patches.push_back(pa1);
	}

	remove_dangling(patches, *patch_type_unary, patch_type, Env(), ofstream());
		
	EXPECT_EQ(0, get_end_num(patches, Env())["dead"]);

	set_color(patches);
	
	if (false)
	{
		PatchesViewer * v = new PatchesViewer(patches);
		v->get_camera().distance += 2;
	}
}

TEST_F(TilingTest, DISABLED_jittering)
{
	vector<shared_ptr<Patch>> view_patches;

	srand(79797+4);
	vector<shared_ptr<Patch>> patches;

	{
		shared_ptr<Patch> pa1(new Patch((*patch_type)["jump"]));
		pa1->translate_time(400);
		patches.push_back(pa1);
	}

	{
		shared_ptr<Patch> pa1(new Patch((*patch_type)["pass_mid_to_mid_413_452"]));
		pa1->translate_time(400);
		
		vector<ConnectInfo> connect_infos;
		connect_infos.push_back(ConnectInfo(1,0,0));
		stitch(patches, pa1, patch_type, connect_infos);
	}

	{
		shared_ptr<Patch> pa1(new Patch((*patch_type)["pass_mid_to_mid_413_452"]));
		pa1->translate_time(400);

		vector<ConnectInfo> connect_infos;
		connect_infos.push_back(ConnectInfo(0,0,1));

		patches.push_back(pa1);
		const set<int> edit_patches = get_edit_patches(connect_infos, patches.size()-1);
		
		patches_edit(patches, patches.size()-1, connect_infos, edit_patches, patch_type, Env());
		copy_patches(view_patches, patches);

		Patch & new_patch = **(patches.end()-1);
		ml::Motion &m = new_patch.motions[0];
		const ml::Posture p = m[m.size()/2];

		vector<cml::vector2d> diff2s;
		get_diff2s(diff2s);

		for (int i = 0; i < diff2s.size(); ++i)
		{
			ml::Posture new_p = p;
			new_p.ApplyTransf(trans_transf(cml::vector3d(0.5*diff2s[i][0], 0, 0.5*diff2s[i][1])));
			patches_edit(patches, patches.size()-1, connect_infos, edit_patches, patch_type, Env(), true, new_p);
			copy_patches(view_patches, patches);
		}
	}

	//if (false)
	{
		PatchesViewer * v = new PatchesViewer(view_patches);
	}
}

TEST_F(TilingTest, DISABLED_warp)
{
	srand(79797+4);
	vector<shared_ptr<Patch>> patches;

	{
		shared_ptr<Patch> pa1(new Patch((*patch_type)["jump"]));
		pa1->translate_time(400);
		patches.push_back(pa1);
	}

	{
		shared_ptr<Patch> pa1(new Patch((*patch_type)["pass_mid_to_mid_413_452"]));
		pa1->translate_time(400);

		vector<ConnectInfo> connect_infos;
		connect_infos.push_back(ConnectInfo(1,0,0));
		stitch(patches, pa1, patch_type, connect_infos);
	}

	{
		shared_ptr<Patch> pa1(new Patch((*patch_type)["pass_mid_to_mid_413_452"]));
		pa1->translate_time(400);

		vector<ConnectInfo> connect_infos;
		connect_infos.push_back(ConnectInfo(0,0,1));
		
		patches.push_back(pa1);
		int new_pa_i = patches.size()-1;
		const set<int> edit_patches = get_edit_patches(connect_infos, new_pa_i);
		patches_edit(patches, new_pa_i, connect_infos, edit_patches, patch_type, Env());
	}

	if (false)
	{
		PatchesViewer * v = new PatchesViewer(patches);
	}
}

TEST_F(TilingTest, DISABLED_compensation)
{
	vector<shared_ptr<Patch>> patches;

	{
		shared_ptr<Patch> pa1(new Patch((*patch_type)["jump"]));
		pa1->translate_time(400);
		patches.push_back(pa1);
	}

	{
		shared_ptr<Patch> pa1(new Patch((*patch_type)["jump"]));
		
		vector<ConnectInfo> connect_infos;
		connect_infos.push_back(ConnectInfo(1,0,0));
		stitch(patches, pa1, patch_type, connect_infos, Env(), false);
	}

	{
		shared_ptr<Patch> pa1(new Patch((*patch_type)["jump"]));
		pa1->translate_time(630);
		pa1->translate(cml::vector3(5.5, 0, 2));
		patches.push_back(pa1);
	}

	{
		shared_ptr<Patch> pa1(new Patch((*patch_type)["jump"]));
		
		vector<ConnectInfo> connect_infos;
		connect_infos.push_back(ConnectInfo(0,0,1));
		stitch(patches, pa1, patch_type, connect_infos, Env(), false);
	}

	
	{
		shared_ptr<Patch> pa1(new Patch((*patch_type)["jump"]));

		vector<ConnectInfo> connect_infos;
		connect_infos.push_back(ConnectInfo(1,2,2));
		stitch(patches, pa1, patch_type, connect_infos, Env(), false);
	}

	//충돌체크용 패치
	{
		shared_ptr<Patch> pa1(new Patch((*patch_type)["jump"]));

		pa1->translate_time(521);
		pa1->translate(cml::vector3(3.0, 0, 1.2));
		patches.push_back(pa1);	 
	}
	
	EXPECT_EQ(false, compensation(patches, 3, patch_type));

	patches[patches.size()-1]->translate(cml::vector3d(-3, 0, 3));
	EXPECT_EQ(true, compensation(patches, 3, patch_type));

	//EXPECT_TRUE(cml::length(cml::vector3(4.26363, 1.06189, 1.19077)- patches[3]->motions[0].last_posture().trans()) < 0.01);

	//if (false)
	{
		set_color(patches);
		PatchesViewer * v = new PatchesViewer(patches);
	}
}

TEST(DISABLED_diffs)
//TEST(diffs)
{
	vector<cml::vector2d> diff2s;
	for (double i = -3; i <= 3; i += 1.0) {
	  for (double j = -3; j <= 3; j += 1.0) {
		diff2s.push_back(cml::vector2d(i, j));
	  }
	}
	sort(diff2s.begin(), diff2s.end(), [](const cml::vector2d &a, const cml::vector2d &b){return a.length() < b.length();});

	{
	  int i = 0;
	  for (auto it = diff2s.begin(); it != diff2s.end(); ++it) {
		cout << "(" << (*it)[0] << ", " << (*it)[1] << ")  " << it->length() << ' ' << i << endl;
		i++;
	  }
	}

	vector<cml::vector3d> diff3s;
	for (double i = -3; i <= 3; i += 1.0) {
	  for (double j = -3; j <= 3; j += 1.0) {
		for (double k = -3; k <= 3; k += 1.0) {
		  diff3s.push_back(cml::vector3d(i, j, k));
		}
	  }
	}
	sort(diff3s.begin(), diff3s.end(), [](const cml::vector3d &a, const cml::vector3d &b){return a.length() < b.length();});

	{
	  int i = 0;
	  for (auto it = diff3s.begin(); it != diff3s.end(); ++it) {
		cout << "(" << (*it)[0] << ", " << (*it)[1] << ", " << (*it)[2] << ")  " << it->length() << ' ' << i << endl;
		i++;
	  }
	}
}

TEST_F(TilingTest, DISABLED_various_walk)
{
	Patch walk = (*patch_type)["pass_mid_to_mid_413_452"];
	//walk.translate_time(50);

	Patch walk1 = walk;
	{
		vector<ml::Motion *> motions;
		motions.push_back(&(walk1.motions[0]));
		auto &m = *(motions[0]);
		vector<Constraint> cons;
		add_cons_pin(cons, 0, m, false);
		add_cons_dir(cons, 0, m.size()-1, cml::roty_mat(cml::pi() * 0.3) * (m[m.size()-1].trans()-m[m.size()-2].trans()));
		multi_motion_edit(motions, cons);
	}
	Patch walk2 = walk;
	{
		vector<ml::Motion *> motions;
		motions.push_back(&(walk2.motions[0]));
		auto &m = *(motions[0]);
		vector<Constraint> cons;
		add_cons_pin(cons, 0, m, false);
		add_cons_dir(cons, 0, m.size()-1, cml::roty_mat(cml::pi() * -0.3) * (m[m.size()-1].trans()-m[m.size()-2].trans()));
		multi_motion_edit(motions, cons);
	}

	vector<shared_ptr<Patch>> patches;
	patches.push_back(shared_ptr<Patch>(new Patch(walk)));
	patches.push_back(shared_ptr<Patch>(new Patch(walk1)));
	patches.push_back(shared_ptr<Patch>(new Patch(walk2)));

	EXPECT_TRUE(patches.size() == 3);

	if (false)
		PatchesViewer * v = new PatchesViewer(patches);
}

TEST_F(TilingTest, DISABLED_dir_ratio)
{
	srand(79797+4);
	vector<shared_ptr<Patch>> patches;

	shared_ptr<Patch> pa1(new Patch((*patch_type)["jump"]));
	pa1->translate_time(400);
	patches.push_back(pa1);

	shared_ptr<Patch> pa2(new Patch((*patch_type)["pass_mid_to_mid_413_452"]));

	vector<ConnectInfo> connect_infos;
	ConnectInfo ci(1, 0, 2, 0.5);
	connect_infos.push_back(ci);

	patches.push_back(pa2);
	int new_pa_i = patches.size()-1;
	const set<int> edit_patches = get_edit_patches(connect_infos, new_pa_i);

	int center_patch_i = new_pa_i;

	vector<ml::Motion*> edit_motions;
	map<int,int> patch2motion_offset;
	int offset = 0;
	for (auto it = edit_patches.begin(); it != edit_patches.end(); ++it)
	{
		patch2motion_offset[*it] = offset;
		for (auto it_m = patches[*it]->motions.begin(); it_m != patches[*it]->motions.end(); ++it_m)
		{
			edit_motions.push_back(&(*it_m));
			offset++;
		}
	}

	vector<Constraint> cons;
	set_inner_cons(patches, patch2motion_offset, cons);
	for (auto it = connect_infos.begin(); it != connect_infos.end(); ++it)
	{
		const ConnectInfo &ci = *it;
		const Boundary &boundary1 = patches[center_patch_i]->boundaries[ci.b1];
		const Boundary &boundary2 = patches[ci.pa2]->boundaries[ci.b2];


		int group1 = patch2motion_offset[center_patch_i] + boundary1.motion_index;
		int index1 = boundary1.p_motion->size()-1;
		int group2 = patch2motion_offset[ci.pa2] + boundary2.motion_index;
		int index2 = 0;

		double dir_ratio = ((*edit_motions[group1])[index1].trans() - (*edit_motions[group1])[index1-1].trans()).length() / ((*edit_motions[group2])[index2+1].trans() - (*edit_motions[group2])[index2].trans()).length();

		add_cons_same_pos_dir_time(cons, patch2motion_offset[center_patch_i] + boundary1.motion_index, boundary1.p_motion->size()-1, patch2motion_offset[ci.pa2] + boundary2.motion_index, 0);
	}

	//boundary
	Pinning(cons, patches, edit_patches, patch2motion_offset);
	multi_motion_edit(edit_motions, cons);

	if (false)
	{
		PatchesViewer * v = new PatchesViewer(patches);
		v->get_camera().distance += 2;
	}
}

//TEST_F(TilingTest, DISABLED_retarget)
//{
//	ml::Motion m = (*patch_type)["pass_mid_to_mid_413_452"].motions[0];
//	m.translate_time_to_zero();
//
//	vector<Constraint> cons;
//	add_cons_pin(cons, 0, m, false);
//	add_cons_pos(cons, 0, m.size()-1, cml::vector3(2, 0, 0));
//
//	vector<ml::Motion *> motions;
//	motions.push_back(&m);
//	multi_motion_edit(motions, cons);
//
//	vector<ml::Motion> view_motions;
//	view_motions.push_back(m);
//	view_motions[0].translate(cml::vector3(0,0,-1));
//
//	DontSlip noslipfoot(18, 19 , 24, 3, 4, 20);
//	noslipfoot.hold(&m);
//
//	view_motions.push_back(m);
//	
//	if (false)
//	{
//		MotionsViewer * v = new MotionsViewer(view_motions);
//	}
//}

TEST_F(TilingTest, DISABLED_save_patch_type)
{
	for (auto it = patch_type->begin(); it != patch_type->end(); ++it)
	{
		vector<shared_ptr<Patch>> patches;
		shared_ptr<Patch> pa(new Patch(it->second));
		patches.push_back(pa);
		write_patches(patches, pa->name);
	}
	cout << "save type" << endl;
}

