/*
 *  body.h
 *  unagi
 *
 *  Created by normit on 09. 09. 09.
 *  Copyright 2009 __MyCompanyName__. All rights reserved.
 *
 */

namespace ml{

	struct Joint
	{
		cml::vector3 offset;
		int parent;
	};

	class Body
	{
	public:
		Body();
		Body(const std::vector<Joint>& joints, const std::map<std::string, int>& joint_map);
		~Body();

		size_t num_joint() const { return m_joints.size();  }

		int joint_index(const std::string& name) const;
		std::string joint_name(int joint) const;

		const std::map<std::string, int>& joint_map() const { return m_jointMap; }

		int parent(int joint) const { return m_joints[joint].parent; }
		const cml::vector3& offset(int joint) const { return m_joints[joint].offset; }
		cml::vector3& offset(int joint) { return m_joints[joint].offset; }

		void GetLeafNodes(std::vector<int>& vec) const;

	protected:
		std::vector<Joint> m_joints;
		std::map<std::string, int> m_jointMap;
	};
};