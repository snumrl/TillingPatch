/*
*  body.cpp
*  unagi
*
*  Created by normit on 09. 09. 09.
*  Copyright 2009 __MyCompanyName__. All rights reserved.
*
*/

#include "stdafx.h"
#include "ml.h"

using namespace ml;

Body::Body()
{

}

Body::Body(const std::vector<Joint>& joints, const std::map<std::string, int>& joint_map)
{
	m_joints = joints;
	m_jointMap = joint_map;
}
Body::~Body()
{

}


int Body::joint_index( const std::string& name ) const
{
	m_jointMap.find(name);

	std::map<std::string,int>::const_iterator it = m_jointMap.find(name);
	if(it != m_jointMap.end()) return it->second;
	else return -1;
}

void ml::Body::GetLeafNodes( std::vector<int>& vec ) const
{
	vec.clear();
	std::vector<int> checked(m_joints.size(),0);
	for(size_t i = 1; i < m_joints.size(); ++i) {
		checked[m_joints[i].parent] = 1;
	}
	for(size_t i = 0; i < m_joints.size(); ++i) {
		if(!checked[i]) vec.push_back((int)i);
	}
}

std::string ml::Body::joint_name( int joint ) const
{
	for(auto it = joint_map().begin(); it != joint_map().end(); ++it) {
		if(it->second == joint) return it->first;
	}
	return "Unknown";
}
