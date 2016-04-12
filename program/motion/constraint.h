namespace ml {

	enum ConstraintMask {
		C_UNCONSTRAINED = 0x00,
		C_POSITION = 0x01,
		C_ORIENTATION = 0x02,
		C_TRANSF = 0x03,
	};

	struct ConstraintEntity {
		ConstraintMask mask;
		int joint;
		cml::transf value;

		ConstraintEntity() {
			mask = C_UNCONSTRAINED;
			joint = -1;
		}

		void ApplyTransf(const cml::transf &t) {
			value *= t;
		}
	};

	class Constraint {
	public:
		void Clear() { m_entity.clear(); }
		size_t num_entity() { return m_entity.size(); }

		void ApplyTransf(const cml::transf& t);

		bool IsConstrained(int joint) const;

		void Push( int joint, ConstraintMask m, const cml::transf &t);
		void Push( int joint, const cml::vector3 &v);
		void Push( int joint, const cml::matrix3 &m);
		void Push( int joint, const cml::transf &t);

	protected:
		std::vector<ConstraintEntity> m_entity;
	};
}