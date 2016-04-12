#include "stdafx.h"
#include "HybridEdit2D.h"


std::vector<std::map<int,int>> get_cons_rigid_2d_for_hybrid(const std::vector<cml::vector2d> &points, double thres /*= 0.027*/ )
{
	int pointN = points.size(); 
	std::vector<std::map<int,int>> rigid;
	int j =0;
	for (int i =1; i< points.size()-1; i+=j)
	{
		double l1 = (points[i+1] - points[i-1]).length();
		if ( l1 < thres)
		{
			std::map<int,int> map;
			int idx1 = i;
			int cnt = 1;
			double l2 = .0;
			do 
 			{
 				l2 = (points[i+cnt+1] - points[i+cnt-1]).length();
				cnt++;
 			} while (l2 < thres); 	

 			if (cnt > 1)
  			{
  				int idx2 = i+(cnt-1);
				if(idx2 == pointN-1) idx2 = pointN-2;
  				map.insert(std::make_pair(idx1,idx2));
 				rigid.push_back(map);
 				j = cnt;
  			}
 			else
 			{
 				j = 1;
 			}
 		}
		else
		{
			j = 1;
		}
	}
	return rigid;
}

void multi_cages_setup(const int groupN, const vector<int>& pointNs, const vector<vector<cml::vector2d>>& multi_points, vector<vector<cml::vector2d>>& multi_cageCoords )
{
	vector<KLIB::Polyline2f> multi_boundary_;

	multi_boundary_.resize(groupN);
	multi_cageCoords.resize(groupN);
 
 	vector<double> maxXs, minXs, maxYs, minYs;
 	minXs.resize(groupN);
 	maxXs.resize(groupN);
 	minYs.resize(groupN);
 	maxYs.resize(groupN);
 
 	for (int i =0; i< groupN; i++)
 	{
 		auto it = std::min_element(multi_points[i].begin(), multi_points[i].end(),[](cml::vector2d p1, cml::vector2d p2){return (p1[0] < p2[0]);});
 		minXs[i] = it->data()[0];
 		it = std::max_element(multi_points[i].begin(), multi_points[i].end(), [](cml::vector2d p1, cml::vector2d p2){return (p1[0] < p2[0]);});
 		maxXs[i] = it->data()[0];
 		it = std::min_element(multi_points[i].begin(), multi_points[i].end(), [](cml::vector2d p1, cml::vector2d p2){return (p1[1] < p2[1]);});
 		minYs[i] = it->data()[1];
 		it = std::max_element(multi_points[i].begin(), multi_points[i].end(), [](cml::vector2d p1, cml::vector2d p2){return (p1[1] < p2[1]);});
 		maxYs[i] = it->data()[1];
 	}
 
 	for (int i =0; i< groupN; i++) 
 	{
 		double offset = .5;
 		int pointN = pointNs[i];
 		Vector2f v1(minXs[i]-offset, minYs[i]-offset);
 		Vector2f v2(minXs[i]-offset, maxYs[i]+offset);
 		Vector2f v3(maxXs[i]+offset, maxYs[i]+offset);
 		Vector2f v4(maxXs[i]+offset, minYs[i]-offset);
 
 		// clock-wise 
 		multi_boundary_[i].push_back(v1);
 		multi_boundary_[i].push_back(v2);
 		multi_boundary_[i].push_back(v3);
 		multi_boundary_[i].push_back(v4);
 
 		// counter clock-wise
 		//multi_boundary_[i].push_back(v4);
 		//multi_boundary_[i].push_back(v3);
 		//multi_boundary_[i].push_back(v2);
 		//multi_boundary_[i].push_back(v1);
 	}
 	for (int i =0; i< groupN; i++) 
 	{
 		multi_boundary_[i].setLoop(true);
 		size_t targetNumPoints = static_cast<size_t>(multi_boundary_[i].length() / 1.5);
 		targetNumPoints = 20;
 		multi_boundary_[i].resample(targetNumPoints);
 		int cageN  = targetNumPoints ;
 		for (int k = 0; k< cageN; k++)
 		{
 			Vector2f v1 = multi_boundary_[i].at(k);
 			multi_cageCoords[i].push_back(cml::vector2d(v1[0],v1[1]));
 		}	
 	}
}
double multi_hybrid_edit2D(std::vector<std::vector<cml::vector2d>> &multi_points, const std::vector<Constraint> &cons, double rigid_thres /*= 0.027*/ )
{
	double error = 0.0;

	return error;
}

