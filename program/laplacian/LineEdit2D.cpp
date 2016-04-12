#include "stdafx.h"
#include "LineEdit2D.h"

std::vector<bool> get_cons_rigid_2d(const std::vector<cml::vector2d> &points, double thres)
{
	std::vector<bool> rigid;
	
	rigid.push_back(false);
	for (int i = 1; i < points.size()-1; ++i)
	{
		double l = (points[i+1] - points[i-1]).length();
		//std::cout << i << ' ' << l << std::endl;
		if ( l < thres)
			rigid.push_back(true);
		else
			rigid.push_back(false);
	}
	rigid.push_back(false);

	return rigid;
}

double edit2D( std::vector<cml::vector2d> &points, std::map<int, cml::vector2d> &cons_pos, std::map<int, cml::vector2d> &cons_ori, std::vector<bool> &cons_rigid /*= std::vector<bool>()*/ )
{
	double error = 0.0;

	int size = points.size();

	//cml::matrixd_c A;
	//cml::vectord b,x;
	//gmm::col_matrix<gmm::wsvector<double> > A;

	cml::matrixd_c A;
	cml::vectord b,x;

	//std::vector<double> b,x;
	std::vector<bool> default_stationary = get_cons_rigid_2d(points);

	for(int step = 0; step < 2; ++step) {
		SQPSolver solver(size*2);

		//constraints
		for ( std::map<int, cml::vector2d>::iterator iter=cons_pos.begin();iter != cons_pos.end();iter++)
		{
			int k = (*iter).first; // index

			for(int j=0; j < 2; ++j) 
			{
				solver.addSquared(1, 1.0, k*2+j, -(*iter).second[j]);
			}
		}

		for ( std::map<int, cml::vector2d>::iterator iter=cons_ori.begin();iter != cons_ori.end();iter++)
		{
			int k = (*iter).first; // index
			cml::vector2d orientation_vec = (*iter).second;
			orientation_vec = orientation_vec * ( cml::length(points[k+1]-points[k-1]) / cml::length(orientation_vec));

			for(int j=0; j < 2; ++j) 
			{
				solver.addSquared(2, 1.0, (k+1)*2+j, -1.0, (k-1)*2+j, -orientation_vec[j]);
			}
		}

		// Shape-Preserving Manipulation
		if(step == 0) {

			for(int i=1; i < size-1; ++i) {
				cml::vector2d vec = points[i+1] - points[i-1];

				cml::vector2d x = vec;
				cml::vector2d y(-x[1], x[0]);

				double lx = x.length_squared();
				double ly = y.length_squared();

				cml::vector2d delta = points[i] - points[i-1];

				double cx = dot(delta, x) / lx;
				double cy = dot(delta, y) / ly;

				solver.addSquared(5, cx, (i+1)*2, 1-cx, (i-1)*2, -cy, (i+1)*2+1, cy, (i-1)*2+1, -1., i*2, 0.);
				solver.addSquared(5, cx, (i+1)*2+1, 1-cx, (i-1)*2+1, cy, (i+1)*2, -cy, (i-1)*2, -1., i*2+1,  0.);

				////stationary constraints
				if (cons_rigid.size() == points.size())
				{
					if (cons_rigid[i] == true)
					{
						solver.addCon(5, cx, (i+1)*2, 1-cx, (i-1)*2, -cy, (i+1)*2+1, cy, (i-1)*2+1, -1., i*2, 0.);
						solver.addCon(5, cx, (i+1)*2+1, 1-cx, (i-1)*2+1, cy, (i+1)*2, -cy, (i-1)*2, -1., i*2+1,  0.);
					}
				}
			}
		}
		// Scale Compensation
		else {
			std::vector<cml::vector2d> vels, accs;
			vels.resize(size);
			accs.resize(size);

			for(int i=0; i < size-1; ++i) {
				cml::vector2d velocity; 
				for(int j=0; j < 2; ++j) velocity[j] = x[(i+1)*2+j]-x[i*2+j];
				velocity.normalize();
				velocity *= (points[i+1] - points[i]).length();

				vels[i] = velocity;
			}
				
			for(int i=1; i < size-1; ++i) {
				accs[i] = vels[i] - vels[i-1];
			}

			for(int i=1; i < size-1; ++i) {
				for(int j=0; j < 2; ++j)
					solver.addSquared(3, 1.0, (i+1)*2+j, -2.0, i*2+j, 1.0, (i-1)*2+j, -accs[i][j]);
			}

			/*for(int i=0; i < size-1; ++i) {
				for(int j=0; j < 2; ++j)
				solver.addSquared(2, 1.0, (i+1)*2+j, -1.0, i*2+j, -vels[i][j]);
			}*/

			//stationary constraints
			if (cons_rigid.size() == points.size())
			{
				for(int i=0; i < size-1; ++i) {
					if (cons_rigid[i] == true)
					{
						for(int j=0; j < 2; ++j)
							solver.addCon(2, 1.0, (i+1)*2+j, -1.0, i*2+j, -vels[i][j]);
					}
				}
			}
		}

		solver.buildSystem(A,b);
		error += solver.solve(A,b,x);
	}

	for(int i=0; i < size; ++i) {
		points[i][0] = x[i*2];
		points[i][1] = x[i*2+1];
	}
	return error;
}

void get_plus_minus_diff( int p1, int size, int &plus_diff, int &minus_diff )
{
	if (p1 == 0) {
		plus_diff = 1;
		minus_diff = 0;
	} else if (p1 == size-1) {
		plus_diff = 0;
		minus_diff = 1;
	} else if ( p1 > 0 && p1 < size-1) {
		plus_diff = 1;
		minus_diff = 1;
	} else {
		cout << "Error get_plus_minus" << endl;
	}
}


double multi_edit2D( std::vector<std::vector<cml::vector2d>> &multi_points,const std::vector<Constraint> &cons, double rigid_thres /*= 0.027*/ )
{
  double error = 0.0;

  vector< vector<bool> > multi_rigid;
  transform(multi_points.begin(), multi_points.end(), back_inserter(multi_rigid), [rigid_thres](vector<cml::vector2d> &points){return get_cons_rigid_2d(points, rigid_thres);});

  cml::matrixd_c A;
  cml::vectord b,x;

  vector<const int> multi_offset;
  int offset_ = 0;
  for (auto it = multi_points.begin(); it != multi_points.end(); ++it) {
	multi_offset.push_back(offset_);
	offset_ += it->size();
  }
  const int multi_size = offset_;

  for(int step = 0; step < 2; ++step) {
	SQPSolver solver(multi_size*2);

	for (auto it = cons.begin(); it != cons.end(); ++it) {
	  if (it->type == "pos") {
		int index = multi_offset[it->m_int[0]]+it->m_int[1];
		cml::vector2d pos = it->m_vector2d[0];
		for(int j=0; j < 2; ++j) {
		  solver.addSquared(1, 1.0, index*2+j, -pos[j]);
		}
	  } else if (it->type == "dir") {
		int g1 = it->m_int[0];
		int p1 = it->m_int[1];
				
		int plus_diff; 
		int minus_diff;
		get_plus_minus_diff(p1, multi_points[g1].size(), plus_diff, minus_diff);

		cml::vector2d dir = it->m_vector2d[0]; 
		cml::vector2d scaled_dir = dir / cml::length(dir) * cml::length(multi_points[g1][p1+plus_diff] - multi_points[g1][p1-minus_diff]);
		int index = multi_offset[g1]+p1;

		for (int j=0; j < 2; ++j) {
		  solver.addSquared(2, 1.0, (index+plus_diff)*2+j, -1.0, (index-minus_diff)*2+j, -scaled_dir[j]);
		}
	  } else if (it->type == "rel_pos") {
		double x = it->m_vector2d[0][0];
		double y = it->m_vector2d[0][1];
		int g1 = it->m_int[0];
		int i1 = it->m_int[1];
		int g2 = it->m_int[2];
		int i2 = it->m_int[3];
		int g1_i1_plus1 = multi_offset[g1]+i1+1;
		int g1_i1_minus1 = multi_offset[g1]+i1-1;
		int g1_i1 = multi_offset[g1]+i1;
		int g2_i2 = multi_offset[g2]+i2;

		solver.addSquared(6, x, g1_i1_plus1*2 , -x, g1_i1_minus1*2, -y, g1_i1_plus1*2+1, y, g1_i1_minus1*2+1, -1., g2_i2*2, 1., g1_i1*2, 0.);
		solver.addSquared(6, x, g1_i1_plus1*2+1, -x, g1_i1_minus1*2+1, y, g1_i1_plus1*2, -y, g1_i1_minus1*2, -1., g2_i2*2+1, 1., g1_i1*2+1, 0.);
	  } else if (it->type == "same_pos") {
		int index1 = multi_offset[it->m_int[0]]+it->m_int[1];
		int index2 = multi_offset[it->m_int[2]]+it->m_int[3];
		for (int j = 0; j < 2; ++j) {
		  solver.addSquared(2, 1.0, index1*2+j, -1.0, index2*2+j, 0.0);
		}
	  } else if (it->type == "same_dir") {
		int index1 = multi_offset[it->m_int[0]]+it->m_int[1];
		int index2 = multi_offset[it->m_int[2]]+it->m_int[3];
		int plus_diff1, minus_diff1, plus_diff2, minus_diff2;
		get_plus_minus_diff(it->m_int[1], multi_points[it->m_int[0]].size(), plus_diff1, minus_diff1);
		get_plus_minus_diff(it->m_int[3], multi_points[it->m_int[2]].size(), plus_diff2, minus_diff2);
				
		double len1 =  length(multi_points[it->m_int[0]][it->m_int[1] + plus_diff1] - multi_points[it->m_int[0]][it->m_int[1] - minus_diff1]);
		double len2 =  length(multi_points[it->m_int[2]][it->m_int[3] + plus_diff2] - multi_points[it->m_int[2]][it->m_int[3] - minus_diff2]);				
		double dir_ratio = len1 / len2;

		//cout << len1 << ' ' << len2 << ' ' << len1 / len2 << ' ' << dir_ratio << endl;
		for (int j = 0; j < 2; ++j) {
		  solver.addSquared(4, 1.0 / dir_ratio, (index1+plus_diff1)*2+j, -1.0 / dir_ratio, (index1-minus_diff1)*2+j, -1.0, (index2+plus_diff2)*2+j, 1.0, (index2-minus_diff2)*2+j, 0.0);
		}
	  }
	}
	
	// Shape-Preserving Manipulation
	if (step == 0) {
	  for (int group = 0; group < multi_points.size(); ++group) {
		vector<cml::vector2d> &points = multi_points[group];
		vector<bool> &rigid = multi_rigid[group];
		int offset = multi_offset[group];
		for(int i=1; i < points.size()-1; ++i) {
		  int offset_i = offset+i;
		  cml::vector2d vec = points[i+1] - points[i-1];

		  cml::vector2d x = vec;
		  cml::vector2d y(-x[1], x[0]);

		  double lx = x.length_squared();
		  double ly = y.length_squared();

		  cml::vector2d delta = points[i] - points[i-1];

		  double cx = dot(delta, x) / lx;
		  double cy = dot(delta, y) / ly;

		  solver.addSquared(5, cx, (offset_i+1)*2, 1-cx, (offset_i-1)*2, -cy, (offset_i+1)*2+1, cy, (offset_i-1)*2+1, -1., (offset_i)*2, 0.);
		  solver.addSquared(5, cx, (offset_i+1)*2+1, 1-cx, (offset_i-1)*2+1, cy, (offset_i+1)*2, -cy, (offset_i-1)*2, -1., (offset_i)*2+1,  0.);

		  ////stationary constraints
		  if (rigid.size() == points.size()) {
			if (rigid[i] == true) {
			  solver.addCon(5, cx, (offset_i+1)*2, 1-cx, (offset_i-1)*2, -cy, (offset_i+1)*2+1, cy, (offset_i-1)*2+1, -1., (offset_i)*2, 0.);
			  solver.addCon(5, cx, (offset_i+1)*2+1, 1-cx, (offset_i-1)*2+1, cy, (offset_i+1)*2, -cy, (offset_i-1)*2, -1., (offset_i)*2+1,  0.);
			}
		  }
		}
	  }
	} else {
	// Scale Compensation	  
	  for (int group = 0; group < multi_points.size(); ++group) {
		vector<cml::vector2d> &points = multi_points[group];
		vector<bool> &rigid = multi_rigid[group];
		int offset = multi_offset[group];
		int size = points.size();

		std::vector<cml::vector2d> vels, accs;
		vels.resize(size);
		accs.resize(size);

		for(int i=0; i < size-1; ++i) {
		  int offset_i = offset+i;
		  cml::vector2d velocity; 
		  for(int j=0; j < 2; ++j) velocity[j] = x[(offset_i+1)*2+j]-x[(offset_i)*2+j];
		  velocity.normalize();
		  velocity *= (points[i+1] - points[i]).length();

		  vels[i] = velocity;
		}

		for(int i=1; i < size-1; ++i) {
		  accs[i] = vels[i] - vels[i-1];
		}

		for(int i=1; i < size-1; ++i) {
		  int offset_i = offset+i;
		  for (int j=0; j < 2; ++j) solver.addSquared(3, 1.0, (offset_i+1)*2+j, -2.0, (offset_i)*2+j, 1.0, (offset_i-1)*2+j, -accs[i][j]);
		}

		///*for(int i=0; i < size-1; ++i) {
		//	for(int j=0; j < 2; ++j)
		//	solver.addSquared(2, 1.0, (i+1)*2+j, -1.0, i*2+j, -vels[i][j]);
		//}*/

		//stationary constraints
		if (rigid.size() == points.size()) {
		  for(int i=0; i < size-1; ++i) {
			  int offset_i = offset+i;
			  if (rigid[i] == true) {
				for(int j=0; j < 2; ++j)
				  solver.addCon(2, 1.0, (offset_i+1)*2+j, -1.0, (offset_i)*2+j, -vels[i][j]);
			  }
		  }
		}
	  }
	}
	solver.buildSystem(A, b);
	error += solver.solve(A, b, x);
  }

  for (int group = 0; group < multi_points.size(); ++group) {
	for (int i = 0; i < multi_points[group].size(); ++i) {
	  multi_points[group][i][0] = x[(multi_offset[group]+i)*2];
	  multi_points[group][i][1] = x[(multi_offset[group]+i)*2+1];
	}
  }
  return error;
}

double multi_edit2D_time( std::vector<std::vector<cml::vector2d>> &multi_points,const std::vector<Constraint> &cons, double rigid_thres /*= 0.027*/ )
{
	double error = 0.0;

	vector<vector<bool>> multi_rigid;
	transform(multi_points.begin(), multi_points.end(), back_inserter(multi_rigid), [rigid_thres](vector<cml::vector2d> &points){return get_cons_rigid_2d(points, rigid_thres);});

	cml::matrixd_c A;
	cml::vectord b,x;

	vector<const int> multi_offset;
	int offset_ = 0;
	for (auto it = multi_points.begin(); it != multi_points.end(); ++it) {
		multi_offset.push_back(offset_);
		offset_ += it->size();
	}
	const int multi_size = offset_;

	for(int step = 0; step < 1; ++step) {
		SQPSolver solver(multi_size*2);

		//default dir
		for (int g1 = 0; g1 < multi_points.size(); ++g1)
		{
			int p1 = 0;

			int plus_diff; 
			int minus_diff;
			get_plus_minus_diff(p1, multi_points[g1].size(), plus_diff, minus_diff);

			cml::vector2d dir = cml::vector2d(1, 0); 
			cml::vector2d scaled_dir = dir / cml::length(dir) * cml::length(multi_points[g1][p1+plus_diff] - multi_points[g1][p1-minus_diff]);
			int index = multi_offset[g1]+p1;

			for(int j=0; j < 2; ++j) 
			{
				solver.addSquared(2, 1.0, (index+plus_diff)*2+j, -1.0, (index-minus_diff)*2+j, -scaled_dir[j]);
			}
		}
		for (auto it = cons.begin(); it != cons.end(); ++it)
		{
			if (it->type == "time")
			{
				int index = multi_offset[it->m_int[0]]+it->m_int[1];
				cml::vector2d time(it->m_double[0], 0.0);
				for(int j=0; j < 2; ++j) 
				{
					solver.addSquared(1, 1.0, index*2+j, -time[j]);
				}
			}
			else if (it->type == "same_time")
			{
				int index1 = multi_offset[it->m_int[0]]+it->m_int[1];
				int index2 = multi_offset[it->m_int[2]]+it->m_int[3];
				for(int j=0; j < 2; ++j) 
				{
					solver.addSquared(2, 1.0, index1*2+j, -1.0, index2*2+j, 0.0);
				}

			}
		}
	
		// Shape-Preserving Manipulation
		if(step == 0) 
		{
			for (int group = 0; group < multi_points.size(); ++group)
			{
				vector<cml::vector2d> &points = multi_points[group];
				vector<bool> &rigid = multi_rigid[group];
				int offset = multi_offset[group];
				for(int i=1; i < points.size()-1; ++i) {
					int offset_i = offset+i;
					cml::vector2d vec = points[i+1] - points[i-1];

					cml::vector2d x = vec;
					cml::vector2d y(-x[1], x[0]);

					double lx = x.length_squared();
					double ly = y.length_squared();

					cml::vector2d delta = points[i] - points[i-1];

					double cx = dot(delta, x) / lx;
					double cy = dot(delta, y) / ly;

					solver.addSquared(5, cx, (offset_i+1)*2, 1-cx, (offset_i-1)*2, -cy, (offset_i+1)*2+1, cy, (offset_i-1)*2+1, -1., (offset_i)*2, 0.);
					solver.addSquared(5, cx, (offset_i+1)*2+1, 1-cx, (offset_i-1)*2+1, cy, (offset_i+1)*2, -cy, (offset_i-1)*2, -1., (offset_i)*2+1,  0.);

					////stationary constraints
					if (rigid.size() == points.size())
					{
						if (rigid[i] == true)
						{
							solver.addCon(5, cx, (offset_i+1)*2, 1-cx, (offset_i-1)*2, -cy, (offset_i+1)*2+1, cy, (offset_i-1)*2+1, -1., (offset_i)*2, 0.);
							solver.addCon(5, cx, (offset_i+1)*2+1, 1-cx, (offset_i-1)*2+1, cy, (offset_i+1)*2, -cy, (offset_i-1)*2, -1., (offset_i)*2+1,  0.);
						}
					}
				}
			}
		}
		// Scale Compensation
		else 
		{
			for (int group = 0; group < multi_points.size(); ++group)
			{
				vector<cml::vector2d> &points = multi_points[group];
				vector<bool> &rigid = multi_rigid[group];
				int offset = multi_offset[group];
				int size = points.size();

				std::vector<cml::vector2d> vels, accs;
				vels.resize(size);
				accs.resize(size);

				for(int i=0; i < size-1; ++i) {
					int offset_i = offset+i;
					cml::vector2d velocity; 
					for(int j=0; j < 2; ++j) velocity[j] = x[(offset_i+1)*2+j]-x[(offset_i)*2+j];
					velocity.normalize();
					velocity *= (points[i+1] - points[i]).length();

					vels[i] = velocity;
				}

				for(int i=1; i < size-1; ++i) {
					accs[i] = vels[i] - vels[i-1];
				}

				for(int i=1; i < size-1; ++i) {
					int offset_i = offset+i;
					for(int j=0; j < 2; ++j)
						solver.addSquared(3, 1.0, (offset_i+1)*2+j, -2.0, (offset_i)*2+j, 1.0, (offset_i-1)*2+j, -accs[i][j]);
				}

				///*for(int i=0; i < size-1; ++i) {
				//	for(int j=0; j < 2; ++j)
				//	solver.addSquared(2, 1.0, (i+1)*2+j, -1.0, i*2+j, -vels[i][j]);
				//}*/

				//stationary constraints
				if (rigid.size() == points.size())
				{
					for(int i=0; i < size-1; ++i) {
						int offset_i = offset+i;
						if (rigid[i] == true)
						{
							for(int j=0; j < 2; ++j)
								solver.addCon(2, 1.0, (offset_i+1)*2+j, -1.0, (offset_i)*2+j, -vels[i][j]);
						}
					}
				}
			}
		}

		solver.buildSystem(A,b);
		error += solver.solve(A,b,x);
	}

	for (int group = 0; group < multi_points.size(); ++group)
	{
		for (int i = 0; i < multi_points[group].size(); ++i)
		{
			multi_points[group][i][0] = x[(multi_offset[group]+i)*2];
			multi_points[group][i][1] = x[(multi_offset[group]+i)*2+1];
		}
	}

	return error;
}
