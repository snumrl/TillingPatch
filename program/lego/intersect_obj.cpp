#include "StdAfx.h"
#include "intersect_obj.h"


inline void make_relative(double m1[4][4], double m2[4][4], double res[4][4]) {
	// res = m1^-1 m2	
	res[0][0] = m1[0][0] * m2[0][0] + m1[1][0] * m2[1][0] + m1[2][0] * m2[2][0];
	res[0][1] = m1[0][0] * m2[0][1] + m1[1][0] * m2[1][1] + m1[2][0] * m2[2][1];
	res[0][2] = m1[0][0] * m2[0][2] + m1[1][0] * m2[1][2] + m1[2][0] * m2[2][2];

	res[1][0] = m1[0][1] * m2[0][0] + m1[1][1] * m2[1][0] + m1[2][1] * m2[2][0];
	res[1][1] = m1[0][1] * m2[0][1] + m1[1][1] * m2[1][1] + m1[2][1] * m2[2][1];
	res[1][2] = m1[0][1] * m2[0][2] + m1[1][1] * m2[1][2] + m1[2][1] * m2[2][2];

	res[2][0] = m1[0][2] * m2[0][0] + m1[1][2] * m2[1][0] + m1[2][2] * m2[2][0];
	res[2][1] = m1[0][2] * m2[0][1] + m1[1][2] * m2[1][1] + m1[2][2] * m2[2][1];
	res[2][2] = m1[0][2] * m2[0][2] + m1[1][2] * m2[1][2] + m1[2][2] * m2[2][2];

	double tbma[3];
	tbma[0] = m2[0][3] - m1[0][3];
	tbma[1] = m2[1][3] - m1[1][3];
	tbma[2] = m2[2][3] - m1[2][3];

	res[0][3] = m1[0][0] * tbma[0] + m1[1][0] * tbma[1] + m1[2][0] * tbma[2];
	res[1][3] = m1[0][1] * tbma[0] + m1[1][1] * tbma[1] + m1[2][1] * tbma[2];
	res[2][3] = m1[0][2] * tbma[0] + m1[1][2] * tbma[1] + m1[2][2] * tbma[2];

	res[3][0] = 0.0;	res[3][1] = 0.0;	res[3][2] = 0.0;	res[3][3] = 1.0;
}

bool intersect_obb(obb *b1, obb *b2) {
	//두 obb간 충돌 감지 코드
	double m[4][4];

	make_relative(b1->m, b2->m, m);
		
	register double t, s;
	register int r;

	const double reps = 1e-6;
	double Bf[3][3];
	double nT[3];

	nT[0] = m[0][3];
	nT[1] = m[1][3];
	nT[2] = m[2][3];
	
	Bf[0][0] = fabs(m[0][0]);  Bf[0][0] += reps;
	Bf[0][1] = fabs(m[0][1]);  Bf[0][1] += reps;
	Bf[0][2] = fabs(m[0][2]);  Bf[0][2] += reps;
	Bf[1][0] = fabs(m[1][0]);  Bf[1][0] += reps;
	Bf[1][1] = fabs(m[1][1]);  Bf[1][1] += reps;
	Bf[1][2] = fabs(m[1][2]);  Bf[1][2] += reps;
	Bf[2][0] = fabs(m[2][0]);  Bf[2][0] += reps;
	Bf[2][1] = fabs(m[2][1]);  Bf[2][1] += reps;
	Bf[2][2] = fabs(m[2][2]);  Bf[2][2] += reps;

  // if any of these tests are one-sided, then the polyhedra are disjoint

	r = 1;

	// A1 x A2 = A0	축이 A0
	t = fabs(nT[0]);
	r &= (t <= 
		(b1->lx + b2->lx * Bf[0][0] + b2->ly * Bf[0][1] + b2->lz * Bf[0][2]));
	if (!r) {
		return false;
	}

	// A2 x A0 = A1	축은 A1
	t = fabs(nT[1]);
	r &= ( t <=	
		(b1->ly + b2->lx * Bf[1][0] + b2->ly * Bf[1][1] + b2->lz * Bf[1][2]));
	if (!r) {
		return false;
	}

	// A0 x A1 = A2	축은 A2
	t = fabs(nT[2]);  
	r &= ( t <= 
		(b1->lz + b2->lx * Bf[2][0] + b2->ly * Bf[2][1] + b2->lz * Bf[2][2]));
	if (!r) {
		return false;
	}

	// B1 x B2 = B0	축은 B0
	s = nT[0]*m[0][0] + nT[1]*m[1][0] + nT[2]*m[2][0];
	t = fabs(s);

	r &= ( t <=
		(b2->lx + b1->lx * Bf[0][0] + b1->ly * Bf[1][0] + b1->lz * Bf[2][0]));
	if (!r) {
		return false;
	}

	// B2 x B0 = B1	축은 B1
	s = nT[0]*m[0][1] + nT[1]*m[1][1] + nT[2]*m[2][1];
	t = fabs(s);

	r &= ( t <=
		(b2->ly + b1->lx * Bf[0][1] + b1->ly * Bf[1][1] + b1->lz * Bf[2][1]));
	if (!r) {
		return false;
	}

  // B0 x B1 = B2	축은 B2
	s = nT[0]*m[0][2] + nT[1]*m[1][2] + nT[2]*m[2][2];
	t = fabs(s);

	r &= ( t <=
		(b2->lz + b1->lx * Bf[0][2] + b1->ly * Bf[1][2] + b1->lz * Bf[2][2]));
	if (!r) {
		return false;
	}

	// A0 x B0	축은 A0 x B0 = (0, -B20, B10)
	s = nT[2] * m[1][0] - nT[1] * m[2][0];
	t = fabs(s);
  
	r &= ( t <= 
		(b1->ly * Bf[2][0] + b1->lz * Bf[1][0] +
		b2->ly * Bf[0][2] + b2->lz * Bf[0][1]));	//B0xB1 = B2임을 이용해서 풀었다
 	if (!r) {
		return false;
	}
  
	// A0 x B1
	s = nT[2] * m[1][1] - nT[1] * m[2][1];
	t = fabs(s);

	r &= ( t <=
		(b1->ly * Bf[2][1] + b1->lz * Bf[1][1] +
		b2->lx * Bf[0][2] + b2->lz * Bf[0][0]));
 	if (!r) {
	  return false;
  }

	// A0 x B2
	s = nT[2] * m[1][2] - nT[1] * m[2][2];
	t = fabs(s);

	r &= ( t <=
		(b1->ly * Bf[2][2] + b1->lz * Bf[1][2] +
		b2->lx * Bf[0][1] + b2->ly * Bf[0][0]));
	if (!r) {
		return false;
	}

	// A1 x B0
	s = nT[0] * m[2][0] - nT[2] * m[0][0];
	t = fabs(s);

	r &= ( t <=
		(b1->lx * Bf[2][0] + b1->lz * Bf[0][0] +
		b2->ly * Bf[1][2] + b2->lz * Bf[1][1]));
 	if (!r) {
		return false;
	}

	// A1 x B1
	s = nT[0] * m[2][1] - nT[2] * m[0][1];
	t = fabs(s);

	r &= ( t <=
		(b1->lx * Bf[2][1] + b1->lz * Bf[0][1] +
		 b2->lx * Bf[1][2] + b2->lz * Bf[1][0]));
	if (!r) {
		return false;
	}

	// A1 x B2
	s = nT[0] * m[2][2] - nT[2] * m[0][2];
	t = fabs(s);

	r &= (t <=
		(b1->lx * Bf[2][2] + b1->lz * Bf[0][2] +
		 b2->lx * Bf[1][1] + b2->ly * Bf[1][0]));
	if (!r) {
		return false;
	}

	// A2 x B0
	s = nT[1] * m[0][0] - nT[0] * m[1][0];
	t = fabs(s);

	r &= (t <=
		(b1->lx * Bf[1][0] + b1->ly * Bf[0][0] +
		 b2->ly * Bf[2][2] + b2->lz * Bf[2][1]));
	if (!r) {
		return false;
	}

	// A2 x B1
	s = nT[1] * m[0][1] - nT[0] * m[1][1];
	t = fabs(s);

	r &= ( t <=
		(b1->lx * Bf[1][1] + b1->ly * Bf[0][1] +
		 b2->lx * Bf[2][2] + b2->lz * Bf[2][0]));
 	if (!r) {
		return false;
	}

	// A2 x B2
	s = nT[1] * m[0][2] - nT[0] * m[1][2];
	t = fabs(s);

	r &= ( t <=
		(b1->lx * Bf[1][2] + b1->ly * Bf[0][2] +
		 b2->lx * Bf[2][1] + b2->ly * Bf[2][0]));
	if (!r) {
		return false;
	}
	return true;
}

