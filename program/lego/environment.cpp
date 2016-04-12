#include "stdafx.h"
#include "environment.h"

void write_box(const Box &box, std::ofstream & file)
{
	const cml::transf &se3 = box.se3;
	file	<<	safe_double(se3(0,0)) << ' ' << safe_double(se3(0,1)) << ' ' << safe_double(se3(0,2)) << ' ' << safe_double(se3(0,3)) << ' '
		<<	safe_double(se3(1,0)) << ' ' << safe_double(se3(1,1)) << ' ' << safe_double(se3(1,2)) << ' ' << safe_double(se3(1,3)) << ' '
		<<	safe_double(se3(2,0)) << ' ' << safe_double(se3(2,1)) << ' ' << safe_double(se3(2,2)) << ' ' << safe_double(se3(2,3)) << ' '
		<<	safe_double(se3(3,0)) << ' ' << safe_double(se3(3,1)) << ' ' << safe_double(se3(3,2)) << ' ' << safe_double(se3(3,3)) << ' ';


	const cml::vector3 & length = box.length;
	file << safe_double(length[0]) << ' ' << safe_double(length[1]) << ' ' << safe_double(length[2]) << ' ';

	file << safe_double(box.time) << endl;
}

void read_box(Box &box, std::ifstream & file)
{
	cml::transf se3;
	file	>>	se3(0,0) >> se3(0,1) >> se3(0,2) >> se3(0,3) 
		>>	se3(1,0) >> se3(1,1) >> se3(1,2) >> se3(1,3) 
		>>	se3(2,0) >> se3(2,1) >> se3(2,2) >> se3(2,3)
		>>	se3(3,0) >> se3(3,1) >> se3(3,2) >> se3(3,3) ;

	cml::vector3 length;
	file >> length[0] >> length[1] >> length[2];

	double time;
	file >> time;

	box.se3 = se3;
	box.length = length;
	box.time = time;
}

void write_env(Env &env, const string & name)
{
	//env
	string file_name = string("./data/save/") + name + ".env";

	std::ofstream file(file_name, std::ios::out);

	file << env.range["x_min"] << ' ' << env.range["x_max"] << ' ' << env.range["x_min_end"] << ' ' << env.range["x_max_end"] << ' ' << 
		env.range["z_min"] << ' ' << env.range["z_max"] << ' ' << env.range["z_min_end"] << ' ' << env.range["z_max_end"] << ' ' << 
		env.range["t_min"] << ' ' << env.range["t_max"] << endl;

	file << env.static_boxes.size() << endl;

	//static_boxes
	for (auto it_s = env.static_boxes.begin(); it_s != env.static_boxes.end(); ++it_s)
	{
		write_box(*it_s, file);
	}

	//dynamic boxes;
	file << env.dynamic_boxes.size() << endl;

	for (auto it_d = env.dynamic_boxes.begin(); it_d != env.dynamic_boxes.end(); ++it_d)
	{
		file << it_d->size() << endl;
		for (auto it_b = it_d->begin(); it_b != it_d->end(); ++it_b)
		{
			write_box(*it_b, file);
		}
	}

	file.close();
}

void read_env(Env &env, const string & name)
{
	ifstream file("./data/save/" + name + ".env", std::ios::in);
	if ( file.fail() ) {
		cout << "no env file: " << name << endl;
		return;
	}

	file >> env.range["x_min"] >> env.range["x_max"] >> env.range["x_min_end"] >> env.range["x_max_end"]
	>> env.range["z_min"] >> env.range["z_max"] >> env.range["z_min_end"] >> env.range["z_max_end"]
	>> env.range["t_min"] >> env.range["t_max"];

	//static_boxes
	int static_box_size;
	file >> static_box_size;

	for (int i = 0; i < static_box_size; ++i)
	{
		Box box;
		read_box(box, file);
		env.static_boxes.push_back(box);
	}

	//dynamic boxes;
	int dynamic_box_size;
	file >> dynamic_box_size;

	for (int i = 0; i < dynamic_box_size; ++i)
	{
		int frame_size;
		file >> frame_size;

		vector<Box> dynamic_box;
		for (int j = 0; j < frame_size; ++j)
		{
			Box box;
			read_box(box, file);
			dynamic_box.push_back(box);
		}

		env.dynamic_boxes.push_back(dynamic_box);
	}

	file.close();
}

void write_patches_env( const vector<shared_ptr<Patch>> & patches, Env &env, const string &name )
{
	write_patches(patches, name);
	write_env(env, name);
}

void read_patches_env( vector<shared_ptr<Patch>> &patches, Env &env, const string &name )
{
	read_patches(patches, name);
	read_env(env, name);
}

bool Env::scatter_range(Patch &pa, double x_min, double x_max, double z_min, double z_max, double t_min, double t_max  )
{
	pa.rotate(cml::pi() * random_float(0,2,0.01));
	pa.translate(cml::vector3(random_float(x_min,x_max,0.02), 0, random_float(z_min,z_max,0.02)));
	pa.translate_time(random(t_min, t_max));
	return true;
}


bool Env::rand_scatter( Patch &pa )
{
	pa.rotate(cml::pi() * random_float(0,2, 0.01));
	pa.translate(cml::vector3(random_float(range["x_min"],range["x_max"],0.02), 0, random_float(range["z_min"],range["z_max"],0.02)));
	pa.translate_time(random_float(range["t_min"], range["t_max"], 0.2));
	return true;
}

bool Env::put_center( Patch &pa )
{
  pa.rotate(cml::pi() * random_float(0,2, 0.01));
  pa.translate(cml::vector3((range["x_min"] + range["x_max"]) / 2.0, 0.0, (range["z_min"] + range["z_max"]) / 2.0));
  pa.translate_time((range["t_min"] + range["t_max"]) / 2.0);
  return true;
}

void Env::set_scatter_range( double x_min, double x_max, double z_min, double z_max, double t_min, double t_max )
{
	set_scatter_range(x_min, x_max, x_min, x_max, z_min, z_max, z_min, z_max, t_min, t_max);
}

void Env::set_scatter_range( double x_min, double x_max, double x_min_end, double x_max_end, double z_min, double z_max, double z_min_end, double z_max_end, double t_min, double t_max )
{
	range["x_min"] = x_min;
	range["x_max"] = x_max;
	range["x_min_end"] = x_min_end;
	range["x_max_end"] = x_max_end;
	range["z_min"] = z_min;
	range["z_max"] = z_max;
	range["z_min_end"] = z_min_end;
	range["z_max_end"] = z_max_end;
	range["t_min"] = t_min;
	range["t_max"] = t_max;
}

Env::Env()
{
	set_scatter_range(-5,5,-5,5, 100, 700);

	is_cyclic["x"] = false;
	is_cyclic["z"] = false;
	is_cyclic["t"] = false;
}
