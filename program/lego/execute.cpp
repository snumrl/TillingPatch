#include "StdAfx.h"
#include "motions_viewer.h"
#include "patches_viewer.h"
#include "motion_edit.h"
#include "tiling.h"
#include "get_patch.h"
#include "nonslipfoot.h"
#include "execute.h"

void all_phase( bool phase1, bool phase2, vector<shared_ptr<Patch>> &patches, Env &env, map<string, Patch> * patch_type, vector<Patch> * patch_type_unary, int max_time, int max_num, int max_attempt, vector<string> &sampling_patches, double k, double alpha, string &save_file_str);

void execute( map<string, Patch> * patch_type, vector<Patch> * patch_type_unary )
{
	execute_ys(patch_type, patch_type_unary);
	//execute_mm(patch_type, patch_type_unary);
	//execute_kl(patch_type, patch_type_unary);
	//execute_picture(patch_type, patch_type_unary);
}

struct Node_exe
{
	int p1, b1, b2;
	string type2;
	double energy;

	bool operator<(const Node_exe& rhs) const
	{
		return this->energy < rhs.energy;
	}
};

bool lessNode(const Node_exe &n1, const Node_exe &n2)
{
	return n1.energy < n2.energy;
}

void execute_ys( map<string, Patch> * patch_type, vector<Patch> * patch_type_unary )
{
	//한 패치 보기
	if (false) {
	  {
		string patch_name1 = "jump";
		vector<shared_ptr<Patch>> patches;
		patches.push_back(shared_ptr<Patch>(new Patch((*patch_type)[patch_name1])));
		set_color(patches);
		PatchesViewer * v = new PatchesViewer(patches);
	  }
	  {
		string patch_name1 = "kick";
		vector<shared_ptr<Patch>> patches;
		patches.push_back(shared_ptr<Patch>(new Patch((*patch_type)[patch_name1])));
		set_color(patches);
		PatchesViewer * v = new PatchesViewer(patches);
	  }
	  {
		string patch_name1 = "high_five";
		vector<shared_ptr<Patch>> patches;
		patches.push_back(shared_ptr<Patch>(new Patch((*patch_type)[patch_name1])));
		set_color(patches);
		PatchesViewer * v = new PatchesViewer(patches);
	  }
	  {
		string patch_name1 = "bow";
		vector<shared_ptr<Patch>> patches;
		patches.push_back(shared_ptr<Patch>(new Patch((*patch_type)[patch_name1])));
		set_color(patches);
		PatchesViewer * v = new PatchesViewer(patches);
	  }
	  {
		string patch_name1 = "beat";
		vector<shared_ptr<Patch>> patches;
		patches.push_back(shared_ptr<Patch>(new Patch((*patch_type)[patch_name1])));
		set_color(patches);
		PatchesViewer * v = new PatchesViewer(patches);
	  }
	  {
		string patch_name1 = "push";
		vector<shared_ptr<Patch>> patches;
		patches.push_back(shared_ptr<Patch>(new Patch((*patch_type)[patch_name1])));
		set_color(patches);
		PatchesViewer * v = new PatchesViewer(patches);
	  }
	  {
		string patch_name1 = "shake_hands";
		vector<shared_ptr<Patch>> patches;
		patches.push_back(shared_ptr<Patch>(new Patch((*patch_type)[patch_name1])));
		set_color(patches);
		PatchesViewer * v = new PatchesViewer(patches);
	  }
	}
	
	//뷰어
	//if (false)
	{
		vector<shared_ptr<Patch>> patches;
		Env env;
		read_patches_env(patches, env, "240_35_15000_0.2_0.0015_phase2");

		//DontSlip noslipfoot(18, 19 , 24, 3, 4, 20);
		//for (auto it = patches.begin(); it != patches.end(); ++it) noslipfoot.hold_foot(**it);

		PatchesViewer * v = new PatchesViewer(patches, env);
		v->get_camera().distance += 2;
	}

	//만들기
	if (false)
	{
		//phase1이든 phase2이든 중간에 멈추려면 bin/control.txt를 -stop에서 stop으로 바꾸면 된다. 그러면 프로그램은 그때의 패치들이 파일로 저장되고 끝남.

		//phase1에서 patches sampling 정도 조절
		vector<string> sampling_patches;
		for (auto it = patch_type->begin(); it != patch_type->end(); ++it) 	{
			if (it->first != "bow_10") //이건 너무 큰 패치라 control 보여줄 때 아니면 사용하지 않는다.
				sampling_patches.push_back(it->first);
		}
			
		bool phase1 = true;
		bool phase2 = true;

		//phase1 parameter
		int max_time = 300;
		int max_num = 30;
		int max_attempt = 200000;
		double alpha = 0.01; //클수록 patch가 더 빨리 더해진다. 범위 대략: 0~0.4
		double k = 0.00137; //클수록 patch가 더해지는 randomness가 커진다. 범위 대략: 0~0.003

		//environment
		Env env;
		size_t startF = 100, endF = 700, turningF = 400;
		env.set_scatter_range(-5,5,-5,5, startF, endF);
		{	//environment
			env.static_boxes.push_back(Box(trans_transf(cml::vector3(0,0,-6.5)), cml::vector3(13, 2.5, 1)));
			env.static_boxes.push_back(Box(trans_transf(cml::vector3(0,0,6.5)), cml::vector3(13, 2.5, 1)));
			env.static_boxes.push_back(Box(trans_transf(cml::vector3(-6.5,0,0)), cml::vector3(1, 2.5, 13)));
			env.static_boxes.push_back(Box(trans_transf(cml::vector3(6.5,0,0)), cml::vector3(1, 2.5, 13)));

			//{
			//	vector<Box> dynamic_box_f;
			//	for (size_t i = startF; i < turningF; ++i) {
			//		cml::transf se3 = trans_transf(cml::vector3(5.5 - 5.5 * 2 * (i - startF) / (double)(turningF-startF), 0, 1.3)) * cml::roty_transf( 0.5 * cml::pi() + 1.0*cml::pi() * (i - startF) / (double)(turningF-startF));
			//		dynamic_box_f.push_back(Box(se3, cml::vector3(3.0, 0.6, 0.2), i));
			//		env.dynamic_boxes.push_back(dynamic_box_f);
			//	}				
			//	vector<Box> dynamic_box_b;
			//	for (size_t i = turningF; i <= endF; ++i) {
			//		cml::transf se3 = trans_transf(cml::vector3(-5.5 + 5.5 * 2 * (i - turningF) / (double)(endF-turningF), 0, 1.3)) * cml::roty_transf( 0.5 * cml::pi() + 1.0*cml::pi() * (i - turningF) / (double)(endF-turningF));
			//		dynamic_box_b.push_back(Box(se3, cml::vector3(3.0, 0.6, 0.2), i));
			//		env.dynamic_boxes.push_back(dynamic_box_b);
			//	}				
			//}
			//{
			//	vector<Box> dynamic_box_f;
			//	for (size_t i = startF; i < turningF; ++i) {
			//		cml::transf se3 = trans_transf(cml::vector3(5.5 - 5.5 * 2 * (i - startF) / (double)(turningF-startF), 0, -1.2)) * cml::roty_transf( 0.5 * cml::pi() - 1.0*cml::pi() * (i - startF) / (double)(turningF-startF));
			//		dynamic_box_f.push_back(Box(se3, cml::vector3(2.0, 2.0, 0.4), i));
			//		env.dynamic_boxes.push_back(dynamic_box_f);
			//	}
			//	vector<Box> dynamic_box_b;
			//	for (size_t i = turningF; i <= endF; ++i) {
			//		cml::transf se3 = trans_transf(cml::vector3(-5.5 + 5.5 * 2 * (i - turningF) / (double)(endF-turningF), 0, -1.2)) * cml::roty_transf( 0.5 * cml::pi() - 1.0*cml::pi() * (i - turningF) / (double)(endF-turningF));
			//		dynamic_box_b.push_back(Box(se3, cml::vector3(2.0, 2.0, 0.4), i));
			//		env.dynamic_boxes.push_back(dynamic_box_b);
			//	}
			//}
		}

		//알고리즘 시작
		std::ostringstream oss;
		oss << max_time << '_' << max_num << '_' << max_attempt << '_' << alpha << '_' << k;
		string save_file_str(oss.str());

		srand(79797+14);
		vector<shared_ptr<Patch>> patches;

		all_phase(phase1, phase2, patches, env, patch_type, patch_type_unary, max_time, max_num, max_attempt, sampling_patches, k, alpha, save_file_str);

		{
			PatchesViewer * v = new PatchesViewer(patches, env);
			v->get_camera().distance += 2;
		}
	}
}

void execute_mm( map<string, Patch> * patch_type, vector<Patch> * patch_type_unary )
{
	if (false)
	{
		//phase1이든 phase2이든 중간에 멈추려면 bin/control.txt를 -stop에서 stop으로 바꾸면 된다. 그러면 프로그램은 그때의 패치들이 파일로 저장되고 끝남.

		//object모션 추가
		/*get_patch_object(patch_type);
		delete patch_type_unary;
		patch_type_unary = new vector<Patch>();
		get_patch_unary(patch_type_unary, patch_type);*/

		//phase1에서 patches sampling 정도 조절
		vector<string> sampling_patches;
		for (auto it = patch_type->begin(); it != patch_type->end(); ++it) 	{
			if (it->first != "bow_10") //이건 너무 큰 패치라 control 보여줄 때 아니면 사용하지 않는다.
				sampling_patches.push_back(it->first);
		}
		//for (int i = 0; i < 10; ++i) //box 자주 나오도록 
		//	sampling_patches.push_back("box");
		
		bool phase1 = true;
		bool phase2 = true;

		//phase1 parameter
		int max_time = 36012;
		int max_num = 10;
		int max_attempt = 150000;
		double alpha = 0.2; //클수록 patch가 더 빨리 더해진다. 범위 대략: 0~0.4
		double k = 0.0015; //클수록 patch가 더해지는 randomness가 커진다. 범위 대략: 0~0.003
		
		srand(79797+16+3+2);
		//environment
		Env env;
		env.set_scatter_range(-5,5,-5,5, 100, 700);
		/*env.is_cyclic["x"] = true;
		env.is_cyclic["z"] = true;*/
		
		env.static_boxes.push_back(Box(trans_transf(cml::vector3(0,0.625,-6.5)), cml::vector3(14, 1.25, 1)));
		env.static_boxes.push_back(Box(trans_transf(cml::vector3(0,0.625,6.5)), cml::vector3(14, 1.25, 1)));
		env.static_boxes.push_back(Box(trans_transf(cml::vector3(-6.5,0.625,0)), cml::vector3(1, 1.25, 14)));
		env.static_boxes.push_back(Box(trans_transf(cml::vector3(6.5,0.625,0)), cml::vector3(1, 1.25, 14)));

		/*{
			vector<Box> dynamic_box;
			for (int i = 100; i <= 700; ++i) {
				cml::transf se3 = trans_transf(cml::vector3(5 + -5 * 2 * (i - 100) / (double)(700-100), 0.1, 1.75)) * cml::roty_transf( 0. * cml::pi() + 2 * cml::pi() * (i - 100) / (double)(700-100) );
				dynamic_box.push_back(Box(se3, cml::vector3(3.5,0.2,0.2), i));
			}
			env.dynamic_boxes.push_back(dynamic_box);
		}
		{
			vector<Box> dynamic_box;
			for (int i = 100; i <= 700; ++i) {
				cml::transf se3 = trans_transf(cml::vector3(5 - 5 * 2 * (i - 100) / (double)(700-100), 0.625, -1.75)) * cml::roty_transf( 0. * cml::pi() - 2 * cml::pi() * (i - 100) / (double)(700-100) );
				dynamic_box.push_back(Box(se3, cml::vector3(3.5,1.25,0.2), i));
			}
			env.dynamic_boxes.push_back(dynamic_box);
		}*/

		//알고리즘 시작
		time_t cur_time;
		time(&cur_time);

		std::ostringstream oss;
		oss << max_time << '_' << max_num << '_' << max_attempt << '_' << alpha << '_' << k << '_' << cur_time;
		string save_file_str(oss.str());

		vector<shared_ptr<Patch>> patches;

		/////////////////////// control 세팅 //////////////
		/*patches.push_back(shared_ptr<Patch>(new Patch((*patch_type)["bow_10"])));
		patches[0]->translate_time(145);
		add_cons_pin(patches[0]->inner_cons, 0, patches[0]->motions[0], 57);
		add_cons_pin(patches[0]->inner_cons, 9, patches[0]->motions[9], 57);

		patches.push_back(shared_ptr<Patch>(new Patch((*patch_type)["bow_10"])));
		patches[1]->translate_time(450);
		patches[1]->rotate(cml::pi() * 0.5);
		add_cons_pin(patches[1]->inner_cons, 0, patches[1]->motions[0], 57);
		add_cons_pin(patches[1]->inner_cons, 9, patches[1]->motions[9], 57);

		patches.push_back(shared_ptr<Patch>(new Patch((*patch_type)["bow_10"])));
		patches[2]->translate_time(755);
		add_cons_pin(patches[2]->inner_cons, 0, patches[2]->motions[0], 57);
		add_cons_pin(patches[2]->inner_cons, 9, patches[2]->motions[9], 57);

		remove_dangling_oneturn(patches, *patch_type_unary, patch_type, env, cout, time(NULL), ofstream());*/
		////////////////////////////////////////////////////

		all_phase(phase1, phase2, patches, env, patch_type, patch_type_unary, max_time, max_num, max_attempt, sampling_patches, k, alpha, save_file_str);

		{
			PatchesViewer * v = new PatchesViewer(patches, env);
			v->get_camera().distance += 2;
		}
	}

	//실험: frame vs. time
	//if (false)
	{
		//phase1이든 phase2이든 중간에 멈추려면 bin/control.txt를 -stop에서 stop으로 바꾸면 된다. 그러면 프로그램은 그때의 패치들이 파일로 저장되고 끝남.

		//object모션 추가
		/*get_patch_object(patch_type);
		delete patch_type_unary;
		patch_type_unary = new vector<Patch>();
		get_patch_unary(patch_type_unary, patch_type);*/

		//phase1에서 patches sampling 정도 조절
		vector<string> sampling_patches;
		for (auto it = patch_type->begin(); it != patch_type->end(); ++it) 	{
			if (it->first != "bow_10") //이건 너무 큰 패치라 control 보여줄 때 아니면 사용하지 않는다.
				sampling_patches.push_back(it->first);
		}

		//for (int i = 0; i < 10; ++i) //box 자주 나오도록 
		//	sampling_patches.push_back("box");
		
		bool phase1 = true;
		bool phase2 = true;

		string info_file_name;
		info_file_name = string("./data/save/frame_time.txt");
		std::ofstream info_file(info_file_name, std::ios::out);

		for (int max_num = 20; max_num <= 20 * 6; max_num += 20)
		{
			////////////////////////////////////////////////////////////////////////////////////
			time_t start_time;
			time(&start_time);
			//phase1 parameter
			int max_time = 36012;
			int max_attempt = 150000;
			double alpha = 0.2; //클수록 patch가 더 빨리 더해진다. 범위 대략: 0~0.4
			double k = 0.0015; //클수록 patch가 더해지는 randomness가 커진다. 범위 대략: 0~0.003

			srand(79797+16+3+4);
			//environment
			Env env;
			env.set_scatter_range(-5,5,-5,5, 100, 700);
			/*env.is_cyclic["x"] = true;
			env.is_cyclic["z"] = true;*/

			env.static_boxes.push_back(Box(trans_transf(cml::vector3(0,0.625,-6.5)), cml::vector3(14, 1.25, 1)));
			env.static_boxes.push_back(Box(trans_transf(cml::vector3(0,0.625,6.5)), cml::vector3(14, 1.25, 1)));
			env.static_boxes.push_back(Box(trans_transf(cml::vector3(-6.5,0.625,0)), cml::vector3(1, 1.25, 14)));
			env.static_boxes.push_back(Box(trans_transf(cml::vector3(6.5,0.625,0)), cml::vector3(1, 1.25, 14)));

			//알고리즘 시작
			time_t cur_time;
			time(&cur_time);

			std::ostringstream oss;
			oss << max_time << '_' << max_num << '_' << max_attempt << '_' << alpha << '_' << k << '_' << cur_time;
			string save_file_str(oss.str());

			vector<shared_ptr<Patch>> patches;

			all_phase(phase1, phase2, patches, env, patch_type, patch_type_unary, max_time, max_num, max_attempt, sampling_patches, k, alpha, save_file_str);

			time_t current_time;
			time(&current_time);
			int diff = difftime(current_time, start_time);

			//number of frame
			int num_frame = 0;
			for (auto it = patches.begin(); it != patches.end(); ++it)
			{
				for (auto it_m = (*it)->motions.begin(); it_m != (*it)->motions.end(); ++it_m)
				{
					num_frame += it_m->size();
				}
			}

			info_file << " # of frame : " << num_frame << " time : " << diff << " max_num " << max_num << " patch_num " << patches.size() << endl;
			patches.clear();
			////////////////////////////////////////////////////////////////////////////////////////////////
		}

		info_file.close();
		/*if (false)
		{
			PatchesViewer * v = new PatchesViewer(patches, env);
			v->get_camera().distance += 2;
		}*/
	}

	//뷰어
	if (false)
	{
		vector<shared_ptr<Patch>> patches;
		Env env;
		read_patches_env(patches, env, "first_dynamic_environment");
		/*env.is_cyclic["x"] = true;
		env.is_cyclic["z"] = true;*/

		DontSlip noslipfoot(18, 19 , 24, 3, 4, 20);
		for (auto it = patches.begin(); it != patches.end(); ++it)
			noslipfoot.hold_foot(**it);

		{
			PatchesViewer * v = new PatchesViewer(patches, env);
			v->get_camera().distance += 2;
		}
	}
	
	//후처리
	if (false)
	{
		vector<shared_ptr<Patch>> patches;
		Env env;
		read_patches_env(patches, env, "cyclic_36010_10000_200000_0.2_0.0015_phase2");
		env.is_cyclic["x"] = true;
		env.is_cyclic["z"] = true;

		DontSlip noslipfoot(18, 19 , 24, 3, 4, 20);
		for (auto it = patches.begin(); it != patches.end(); ++it)
			noslipfoot.hold_foot(**it);

		{
			PatchesViewer * v = new PatchesViewer(patches, env);
			v->get_camera().distance += 2;
		}
	}

	//한 패치 보기
	if (false)
	{
		string patch_name = "coke3";

		vector<shared_ptr<Patch>> patches;
		patches.push_back(shared_ptr<Patch>(new Patch((*patch_type)[patch_name])));
		set_color(patches);

		PatchesViewer * v = new PatchesViewer(patches);
	}
}

void execute_kl( map<string, Patch> * patch_type, vector<Patch> * patch_type_unary )
{
	//만들기
	//if (false)
	{
		//phase1이든 phase2이든 중간에 멈추려면 bin/control.txt를 -stop에서 stop으로 바꾸면 된다. 그러면 프로그램은 그때의 패치들이 파일로 저장되고 끝남.
		
		//phase1에서 patches sampling 정도 조절
		vector<string> sampling_patches;

		//object모션 추가
		get_patch_object(patch_type);
		delete patch_type_unary;
		patch_type_unary = new vector<Patch>();
		get_patch_unary(patch_type_unary, patch_type);

		for (auto it = patch_type->begin(); it != patch_type->end(); ++it) 	{
			if (it->first != "bow_10" && it->first != "box_from") //이건 너무 큰 패치라 control 보여줄 때 아니면 사용하지 않는다.
				sampling_patches.push_back(it->first);
		}
		for (int i = 0; i < 4; ++i) //box 자주 나오도록 
			sampling_patches.push_back("box");
		for (int i = 0; i < 2; ++i)
			sampling_patches.push_back("chair");

		bool phase1 = true;
		bool phase2 = true;

		//phase1 parameter
		int max_time = 3603;
		int max_num = 2;
		int max_attempt = 200000;
		double alpha = 0.1; //클수록 patch가 더 빨리 더해진다. 범위 대략: 0~0.4
		double k = 0.002; //클수록 patch가 더해지는 randomness가 커진다. 범위 대략: 0~0.003

		//environment
		Env env;
		env.set_scatter_range(-5,5,-5,5, 100, 800);
		env.is_cyclic["x"] = true;
		env.is_cyclic["z"] = true;

		/*
		env.static_boxes.push_back(Box(trans_transf(cml::vector3(0,0,-6.5)), cml::vector3(13, 2.5, 1)));
		env.static_boxes.push_back(Box(trans_transf(cml::vector3(0,0,6.5)), cml::vector3(13, 2.5, 1)));
		env.static_boxes.push_back(Box(trans_transf(cml::vector3(-6.5,0,0)), cml::vector3(1, 2.5, 13)));
		env.static_boxes.push_back(Box(trans_transf(cml::vector3(6.5,0,0)), cml::vector3(1, 2.5, 13)));
		*/

		//알고리즘 시작
		std::ostringstream oss;
		oss << "kl_" << max_time << '_' << max_num << '_' << max_attempt << '_' << alpha << '_' << k;
		string save_file_str(oss.str());

		srand(79797+15);
		vector<shared_ptr<Patch>> patches;
		{
			std::shared_ptr<Patch> patch1 = std::shared_ptr<Patch>(new Patch((*patch_type)["box_from"]));
			patch1->translate_time(190);
			patch1->translate(cml::vector3d(-2, 0, 0));
			patch1->patch_object.objectType = PatchObject::kPrevBox;
			patch1->patch_object.begin_frame = 0;
			patches.push_back(patch1);

			remove_dangling_oneturn(patches, *patch_type_unary, patch_type, env, std::cout, 0, std::cout);

			/*for(int i = 0; i < patch1->motions.size(); ++i) {
				for(int j = 0; j < patch1->motions[i].size(); ++j) {
					ml::Posture &p = patch1->motions[i].posture(j);
					if(p.object) {
						vector<Box> dynamic_box;
						cml::transf se3 = get_box_transf( p );
						for(int i = 80; i < p.time; ++i) {
							dynamic_box.push_back(Box(se3, cml::vector3(0.6,0.5,0.4), i));
						}
						if(dynamic_box.size()) env.dynamic_boxes.push_back(dynamic_box);

						break;
					}
				}
			}*/
			/*
			std::shared_ptr<Patch> patch2 = std::shared_ptr<Patch>(new Patch((*patch_type)["chair"]));
			patch2->translate_time(200);
			//patches.push_back(patch2);

			std::shared_ptr<Patch> patch3 = std::shared_ptr<Patch>(new Patch((*patch_type)["box"]));
			//patches.push_back(patch2);
			*/
		}

		all_phase(phase1, phase2, patches, env, patch_type, patch_type_unary, max_time, max_num, max_attempt, sampling_patches, k, alpha, save_file_str);

		{
			PatchesViewer * v = new PatchesViewer(patches, env);
			v->get_camera().distance += 2;
		}
	}

	//뷰어
	if (false)
	{
		vector<shared_ptr<Patch>> patches;
		Env env;
		env.is_cyclic["x"] = true;
		env.is_cyclic["z"] = true;
		read_patches_env(patches, env, "box_1");

		for(int i = 0; i < patches.size(); ++i) {
			if(patches[i]->name == "box_from") {
				patches[i]->patch_object.objectType = PatchObject::kPrevBox;
				patches[i]->patch_object.begin_frame = 0;
			}
			else if(patches[i]->name == "box") {
				patches[i]->patch_object.objectType = PatchObject::kBoxBlending;
				patches[i]->patch_object.range = 5;
				patches[i]->patch_object.begin_motion = 1;
				patches[i]->patch_object.begin_frame = 63;
				patches[i]->patch_object.end_motion = 0;
				patches[i]->patch_object.end_frame = 33;
			}
			else if(patches[i]->name == "chair") {
				patches[i]->patch_object.objectType = PatchObject::kPatchBox;
				patches[i]->patch_object.begin_motion = 2;
				patches[i]->patch_object.begin_frame = 50;
				patches[i]->patch_object.end_motion = 1;
				patches[i]->patch_object.end_frame = 35;
			}
		}

		DontSlip noslipfoot(18, 19 , 24, 3, 4, 20);
		for (auto it = patches.begin(); it != patches.end(); ++it)
			noslipfoot.hold_foot(**it);

		{
			PatchesViewer * v = new PatchesViewer(patches, env);
			//v->image_name = "cyclic";
			v->get_camera().distance += 2;
		}
	}

	//한 패치 보기
	if (false)
	{
		for(auto it = patch_type->begin(); it != patch_type->end(); ++it) {
			if(it->second.motions.size() > 1) {
				std::cout << it->first << "," << it->second.motions.size()
					<< "," << it->second.get_last_time() - it->second.get_begin_time() 
					<< "," << it->second.inner_cons.size() 
					<< std::endl;
			}
		}

		string patch_name = "chair";

		vector<shared_ptr<Patch>> patches;
		patches.push_back(shared_ptr<Patch>(new Patch((*patch_type)[patch_name])));
		for(int i = 0; i < 17; ++i) {
			set_color(patches);
		}

		PatchesViewer * v = new PatchesViewer(patches);
		v->set_record_mode();
	}
}

void execute_picture( map<string, Patch> * patch_type, vector<Patch> * patch_type_unary )
{
	//if (false)
	{
		srand(79797+3);
		vector<shared_ptr<Patch>> patches;

		{
			shared_ptr<Patch> pa1(new Patch((*patch_type)["shake_hands"]));
			pa1->translate_time(400);
			/*add_cons_pin(pa1->inner_cons, 1, pa1->motions[1], 70);
			add_cons_pin(pa1->inner_cons, 1, pa1->motions[1], 90);*/
			patches.push_back(pa1);
		}

		{
			shared_ptr<Patch> pa1(new Patch((*patch_type)["shake_hands"]));
			pa1->translate_time(400);
			add_cons_pin(pa1->inner_cons, 1, pa1->motions[1], 0);

			vector<Constraint> cons;
			copy(pa1->inner_cons.begin(), pa1->inner_cons.end(), back_inserter(cons));
			add_cons_pos(cons, 0, 0, pa1->motions[0][0].trans() + cml::vector3(-0.25,0,0.6));
			multi_motion_edit(pa1->motions, cons, 0.028);
			patches.push_back(pa1);
		}

		set_color(patches);
		DontSlip noslipfoot(18, 19 , 24, 3, 4, 20);
		for (auto it = patches.begin(); it != patches.end(); ++it)
			noslipfoot.hold_foot(**it);

		Env env;
		env.set_scatter_range(-5,5,-5,5, 100, 800);
		write_patches_env(patches, env, "deform");

		PatchesViewer * v = new PatchesViewer(patches);
	}
}

void all_phase( bool phase1, bool phase2, vector<shared_ptr<Patch>> &patches, Env &env, map<string, Patch> * patch_type, vector<Patch> * patch_type_unary, int max_time, int max_num, int max_attempt, vector<string> &sampling_patches, double k, double alpha, string &save_file_str )
{
	//info 저장
	string info_file_name;
	info_file_name = string("./data/save/") + save_file_str + ".info";
	std::ofstream info_file(info_file_name, std::ios::out);
	info_file << "time, # of dangling, # of connection, # of patches, # of attempt, current energy" << endl;
	time_t start_time;
	time(&start_time);

	if (phase1 == true && phase2 == false) 
	{
		metropolis(patches, patch_type, max_time, max_num, max_attempt, sampling_patches, env, k, alpha, cout, start_time, info_file);
		set_color(patches);
		write_patches_env(patches, env, save_file_str);
	}
	else if (phase1 == false && phase2 == true)
	{
		read_patches_env(patches, env, save_file_str);
		remove_dangling(patches, *patch_type_unary, patch_type, env, cout, start_time, info_file);
		set_color(patches);
		write_patches_env(patches, env, save_file_str + "_phase2");
	}
	else if (phase1 == true && phase2 == true)
	{
		metropolis(patches, patch_type, max_time, max_num, max_attempt, sampling_patches, env, k, alpha, cout, start_time, info_file);
		set_color(patches);
		write_patches_env(patches, env, save_file_str);
		remove_dangling(patches, *patch_type_unary, patch_type, env, cout, start_time, info_file);
		set_color(patches);
		write_patches_env(patches, env, save_file_str + "_phase2");
	}
	info_file.close();

	DontSlip noslipfoot(18, 19, 24, 3, 4, 20);
	for (auto it = patches.begin(); it != patches.end(); ++it)
		noslipfoot.hold_foot(**it);
}

void all_phase_tiling( vector<shared_ptr<Patch>> &patches, Env &env, map<string, Patch> *patch_type, vector<Patch> *unary_patch_type,  int max_time, int max_num, int max_attempt, vector<string> &sampling_patches, double k, double alpha, string & save_file_str)
{
  string info_file_name;
  info_file_name = string("./data/save/") + save_file_str + ".info";
  std::ofstream info_file(info_file_name, std::ios::out);
  info_file << "time, # of dangling, # of connection, # of patches, # of attempt, current energy" << endl;
  
  time_t start_time;
  time(&start_time);
    
  metropolis(patches, patch_type, max_time, max_num, max_attempt, sampling_patches, env, k, alpha, cout, start_time, info_file);
  // set_color(patches);
  write_patches_env(patches, env, save_file_str);
  remove_dangling(patches, *unary_patch_type, patch_type, env, cout, start_time, info_file);
  set_color(patches);
  //set_color_twopart(patches);
  write_patches_env(patches, env, save_file_str + "_phase2");
  info_file.close();  
}

void tiling_by_example( vector<shared_ptr<Patch>> &patches, Env &env, const vector<shared_ptr<Patch> > &input_patches, const string example_name, const int character_num )
{
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

  srand(79797+16+3+7);
  //environment
  env.set_scatter_range(-5,5,-5,5, 100, 800);		
  env.static_boxes.push_back(Box(trans_transf(cml::vector3(0, 0.625, -6.5)), cml::vector3(14, 1.25, 1)));
  env.static_boxes.push_back(Box(trans_transf(cml::vector3(0, 0.625, 6.5)), cml::vector3(14, 1.25, 1)));
  env.static_boxes.push_back(Box(trans_transf(cml::vector3(-6.5, 0.625, 0)), cml::vector3(1, 1.25, 14)));
  env.static_boxes.push_back(Box(trans_transf(cml::vector3(6.5, 0.625, 0)), cml::vector3(1, 1.25, 14)));		

  std::ostringstream oss;
  time_t start_time;
  time(&start_time);
  oss << start_time;
  string save_file_str(oss.str());

  srand(79797+5+2);
  //random scat
  for (int i = 0; i < character_num/2; ++i)
  {
    random_shuffle(sampling_patches.begin(), sampling_patches.end());
    while(true)
    {
      for (auto it_p = sampling_patches.begin(); it_p != sampling_patches.end(); ++it_p)
      {
        shared_ptr<Patch> pa1(new Patch((patch_types.find(*it_p))->second));
        if (pa1->motions.size() == 1)
          continue;

        if (example_name == "general")
          env.rand_scatter(*pa1);
        else
          env.scatter_range(*pa1, -1, 1, -1, 1, 80, 120);
          //env.scatter_range(*pa1, -1-3, 1-3, -1-3, 1-3, 80, 120);

        vector<ConnectInfo> connect_infos;
        get_connect_infos(connect_infos, pa1, patches, 4.0, env);

        if (stitch(patches, pa1, &patch_types, connect_infos, env) == true) {
          goto one_tile;
        }
      }
    }
    one_tile:;
    cout << "size: " << patches.size() << endl;
  }

  //2nd Phase
  if (example_name == "general")
    deterministic(patches, env, &patch_types, get_energies_general);
  else if(example_name == "only_first") 
    { }
  else
    deterministic(patches, env, &patch_types, get_energies_path);
  //remove_dangling(patches, unary_patch, &patch_types, env, cout);

  set_color(patches);
//  set_color_one(patches);
}