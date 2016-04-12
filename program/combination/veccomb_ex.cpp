// TestComb.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include <iostream>
#include <vector>
#include <string>
#include "veccomb.h"

int _tmain(int argc, _TCHAR* argv[])
{
    using namespace std;
	typedef vector<char> Set;

     vector<Set> VecOrig;

     Set v1, v2, v3;
     v1.push_back('A');
     v1.push_back('B');
     v1.push_back('C');
     v1.push_back('D');
     v2.push_back('A');
     v2.push_back('B');
     v2.push_back('C');
     v3.push_back('A');
     v3.push_back('B');
     v3.push_back('C');
     VecOrig.push_back(v1);
     VecOrig.push_back(v2);
     VecOrig.push_back(v3);

     vector<Set> VecResults;

     VecResults.push_back( Set() );
     VecResults.push_back( Set() );
     VecResults.push_back( Set() );

     VecResults.at(0).push_back( v1.at(0) );
     VecResults.at(0).push_back( v1.at(1) );
     VecResults.at(0).push_back( v1.at(2) );
     VecResults.at(1).push_back( v2.at(0) );
     VecResults.at(1).push_back( v2.at(1) );
     VecResults.at(2).push_back( v2.at(0) );
     VecResults.at(2).push_back( v2.at(1) );

	 while(FindVecComb(
		VecOrig,
		VecResults ) )
	 {
		std::string str="";
		for( size_t i=0; i<VecResults.size(); ++i )
		{
			for( size_t j=0; j<VecResults.at(i).size(); ++j )
			{
				str+=VecResults.at(i).at(j);
			}
			str+=" | ";
		}
		cout<<str.c_str()<<endl;
	 }

	 return 0;
}