#ifndef UMFSOLVER_H
#define UMFSOLVER_H

#include "UMFPACK/include/umfpack.h"


// add ../dependencies/UMFPACK5.2/ to input directory.
#pragma comment(lib, "amd.lib")
#pragma comment(lib, "umfpack.lib")

class Umfsolver{
	
	std::vector<int> Ap, Ai;
	std::vector<double> Ax;
	int _n, _m ;

public:
	Umfsolver()	{}
	~Umfsolver(){}

	// factorize matrix A
	void umf_factorize(cml::matrixd_c const& A)
	{
		_n = (int)A.rows();
		_m = (int)A.cols();

		Ai.resize(0);
		Ax.resize(0);
		Ap.resize(_n+1);

		for (int j =0; j<_m; j++)
		{
			Ap[j] = (int)Ai.size() ;

			for (int i =0; i<_n; i++)
			{
				//cout << "[ " << i << ", " << j <<" ]" << A[i][j] << endl;

				if (A(i,j) != 0.0)
				{
					Ai.push_back(i);
					Ax.push_back(A(i,j));
				}
			}
		}

		
		Ap[_n] = (int)Ai.size();		
	}

	// solve Ax=b
	void umf_solve(cml::vectord & x, cml::vectord const& b) const
	{
		int n=_n;
		int m=_m;

		x.resize(m);
		void *Symbolic, *Numeric ;
		double *null = (double *) NULL ;
		const int *App = &Ap[0];
		const int *Aii = &Ai[0];
		const double *Axx = &Ax[0];
		

		(void) umfpack_di_symbolic (n, m, App, Aii, Axx, &Symbolic, null, null) ;
		(void) umfpack_di_numeric (App, Aii, Axx , Symbolic, &Numeric, null, null) ;
		umfpack_di_free_symbolic (&Symbolic) ;
		(void) umfpack_di_solve (UMFPACK_A, App, Aii, Axx, x.data(), b.data(), Numeric, null, null) ;
		umfpack_di_free_numeric (&Numeric) ;
	}

private:
};

namespace sm
{
	template <class MAT_TYPE>
	static void UMFsolve(MAT_TYPE const& A, cml::vectord const& b, cml::vectord & x)
	{
		Umfsolver umf_A;
		umf_A.umf_factorize(A);
		umf_A.umf_solve(x , b);
	}
}

#endif