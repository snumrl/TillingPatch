//char_comb_ex.cpp
//This program finds combinations using the recursive method

/*****Instructions*******
To see how it works for other combinations
Try setting N_NUM to other numbers (must not be greater than 10)
Try setting R_NUM to other numbers (must not be greater than N_NUM)
*************************/

#include "stdafx.h"
#include<iostream>

using namespace std;

//function prototype
void char_combination(char n[],int n_column,
            char r[], int r_size, int r_column, int loop);

int main()
{
	const int N_NUM=6; 
	const int R_NUM=4;
	
	char n[]="1234567890";

	char r[R_NUM+1];//+1 for the NULL character
	r[R_NUM]='\0';

	char_combination(n,0,r,R_NUM,0,N_NUM-R_NUM);
	
	cout<<"Complete!"<<endl;
	return 0;
}

void char_combination(char n[],int n_column,
            char r[], int r_size, int r_column, int loop)
{
	int localloop=loop;
	int local_n_column=n_column;
	
	///////Display the string code/////////
	if(r_column>(r_size-1))
	{
		cout<<r<<endl;
		return;
	}
	/////End of displaying string code//////
	
	for(int i=0;i<=loop;++i)
	{
		r[r_column]=n[n_column+i];
		++local_n_column;
		char_combination(n,local_n_column,r,r_size,r_column+1,localloop);
		--localloop;
	}
}
