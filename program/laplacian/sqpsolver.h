#pragma once
#include <vector>
#include <list>
#include "cml/cml.h"
#include "umfsolver.h"

class SQPSolver
{
protected:
	int	m_numVar;
	int m_numCon;
public:
	struct Con{
		std::vector<int> index;
		std::vector<double>	coef;
	};

	SQPSolver (int numVar) : m_numVar(numVar), m_numCon(0) {}
	virtual ~SQPSolver ();

	/**
		(3x+4y+5z+1)^2 을 추가하고 싶으면, addSquared(3, 3.0, x, 4.0, y, 5.0, z, 1.0);	where x=0, y=1, z=2
	*/
	void addSquared(int n, double coef1, int index1, ...);

	/**
		4*(3x+4y+5z+1)^2 를 추가하고 싶으면, addSquaredWeighted(4.0, 3, 3.0, x, 4.0, y, 5.0, z, 1.0); where x=0, y=1, z=2
	*/
	void addSquaredWeighted(double weight, int n, double coef1, int index1, ...);

	std::list<Con*> m_SQTermList;
	std::list<Con*> m_ConTermList;

	void addCon(int n, double coef1, int index1, ...);

	void buildSystem(cml::matrixd_c& A, cml::vectord &b);

	double solve(cml::matrixd_c const& A, cml::vectord const& b, cml::vectord & x)
	{
		sm::UMFsolve(A, b, x);
		return getObjectiveValue(x);
		/*cml::vectord c(m_numVar);
		
		for ( int i=0; i<m_numVar; i++ )
		{
			c[i] = A(i,0)*x[0];
			for ( int j=1; j<m_numVar; j++ )
			{
				c[i] += A(i,j)*x[j];
			}
			c[i] = c[i]-b[i];
		}
		return cml::length( c );*/
	}

	void solveMosek(cml::vectord &x);
	double getObjectiveValue(const cml::vectord & x);
};
