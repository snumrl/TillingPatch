#include "stdafx.h"

#include "sqpsolver.h"
#include "stdarg.h"
#include "math.h"
#include "assert.h"
#include "stdarg.h"


#ifndef SQR
#define SQR(x) ((x)*(x))
#endif






///////////////////////////////////////////////////////////////////////////////////////////////////////////////

SQPSolver::~SQPSolver(void)
{
	for(std::list<Con*>::iterator i=m_SQTermList.begin(); i!=m_SQTermList.end(); ++i)
		delete (*i);

	for(std::list<Con*>::iterator i=m_ConTermList.begin(); i!=m_ConTermList.end(); ++i)
		delete (*i);
}

void SQPSolver::addSquared( int n, double coef1, int index1, ... )
{
	Con* term=new Con;
	m_SQTermList.push_back(term);

	term->index.resize(n);
	term->coef.resize(n+1);

	term->coef[0]=coef1;
	term->index[0]=index1;

	va_list marker;
	va_start( marker, index1);     /* Initialize variable arguments. */

	int i;
	for(i=1; i<n; i++)
	{
		term->coef[i]=va_arg(marker, double);
		term->index[i]=va_arg(marker, int);
	}

	term->coef[i]=va_arg(marker, double);
	va_end(marker);
}

void SQPSolver::addSquaredWeighted( double weight, int n, double coef1, int index1, ... )
{
	Con* term=new Con;
	m_SQTermList.push_back(term);

	weight = std::sqrt(weight);

	term->index.resize(n);
	term->coef.resize(n+1);

	term->coef[0]=coef1*weight;
	term->index[0]=index1;

	va_list marker;
	va_start( marker, index1);     /* Initialize variable arguments. */

	int i;
	for(i=1; i<n; i++)
	{
		term->coef[i]=va_arg(marker, double) * weight;
		term->index[i]=va_arg(marker, int);
	}

	term->coef[i]=va_arg(marker, double)*weight;
	va_end(marker);
}

void SQPSolver::buildSystem( cml::matrixd_c& A, cml::vectord &b )
{

	A.zero();
	b.zero();

	A.resize(m_numVar+m_numCon, m_numVar+m_numCon);
	b.resize(m_numVar+m_numCon);

	/*
	for(int i=0; i < m_numCon+m_numVar; ++i) {
		for(int j=0; j < m_numCon+m_numVar; ++j) A(i,j) = 0;
		b[i] = 0;
	}
	*/

	for(std::list<Con*>::iterator i=m_SQTermList.begin(); i!=m_SQTermList.end(); ++i)
	{
		std::vector<int> & index=(*i)->index;
		std::vector<double> & value=(*i)->coef;

		// updateH
		for(int i=0; i<(int)index.size(); i++)
			A(index[i],index[i])+=2.0*SQR(value[i]);
		for(int i=0; i<(int)index.size(); i++)
			for(int j=i+1; j<(int)index.size(); j++)
			{
				double prev=A(index[j],index[i]);
				A(index[i],index[j])=A(index[j],index[i])=prev+2.0*value[i]*value[j];
			}

			// update R
			for(int i=0; i<(int)index.size(); i++)
			{
				b[index[i]]-=2.0*value[index.size()]*value[i];
			}
	}

	int icon=0;
	for(std::list<Con*>::iterator i=m_ConTermList.begin(); i!=m_ConTermList.end(); ++i)
	{
		Con* con=*i;

		for(int i=0; i<(int)con->index.size(); i++)
		{
			A(icon+m_numVar,con->index[i])=con->coef[i];
			A(con->index[i], icon+m_numVar)=con->coef[i];
		}		
		b[m_numVar+icon]=con->coef[con->index.size()]*-1.0;
		icon++;
	}
	assert(icon==m_numCon);
}
 
void SQPSolver::addCon( int n, double coef1, int index1, ... )
{
	Con* term=new Con;
	m_ConTermList.push_back(term);

	term->index.resize(n);
	term->coef.resize(n+1);

	term->coef[0]=coef1;
	term->index[0]=index1;

	va_list marker;
	va_start( marker, index1);     

	int i;
	for(i=1; i<n; i++)
	{
		term->coef[i]=va_arg(marker, double);
		term->index[i]=va_arg(marker, int);
	}

	term->coef[i]=va_arg(marker, double);
	va_end(marker);

	m_numCon += 1;
}

double SQPSolver::getObjectiveValue( const cml::vectord & x )
{
	double objective_value = 0.0;
	for(std::list<Con*>::iterator i=m_SQTermList.begin(); i!=m_SQTermList.end(); ++i)
	{
		std::vector<int>& index=(*i)->index;
		std::vector<double>& value=(*i)->coef;

		double ax_minus_b = 0.0;

		for(int i=0; i<(int)index.size(); i++) {
			ax_minus_b += value[i] * x[index[i]];
		}
		ax_minus_b += value[value.size()-1];

		objective_value += (ax_minus_b * ax_minus_b);
	}

	return objective_value;
}

/////////////////////////////////////////////////////////////
