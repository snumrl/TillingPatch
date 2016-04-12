/*
 *  reader.h
 *  unagi
 *
 *  Created by normit on 09. 09. 10.
 *  Copyright 2009 __MyCompanyName__. All rights reserved.
 *
 */

namespace ml {
	class BVHReader
	{
	public:
		void LoadBVH(const char *file, Motion *motion, bool human_load, double scale, int sample = 1);

	protected:
		enum Channel { XPOS = 1, YPOS, ZPOS, ZROT, XROT, YROT };

		void AddChannel(int i, Channel ch);

		int NewNode(const std::string&name, int parent);

		std::vector<std::vector<Channel> > m_channels;
		std::vector<Joint> m_joints;
		std::map<std::string, int> m_jointMap;

	};

	class AMCReader
	{
	public:
		void LoadAMC(const char *amc_file, const char *asf_file, Motion *motion, bool human_load, double scale, int sample = 1);
	protected:
		enum Channel { XPOS = 1, YPOS, ZPOS, ZROT, XROT, YROT };
		struct AMCJoint
		{
			std::string name;
			cml::vector3 dir;
			cml::vector3 offset;
			cml::matrix3 m;
			bool is_parent;
			int parent;
		};
		void AddChannel(int i, Channel ch);
		int NewNode(const std::string& name);

		std::vector<std::vector<Channel> > m_channels;
		std::vector<AMCJoint> m_joints;
		std::map<std::string, int> m_jointMap;
	};

};