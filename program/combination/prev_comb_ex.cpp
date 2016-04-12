#include<iostream>
#include<vector>
#include<string>
#include"combination.h"

using namespace std;
using namespace stdcomb;

/*
//using pointers
int main()
{
  char n[]="2345678";
  char r[]="678";
  
  do
  {
    cout<<r<<endl;    
  }
  while(prev_combination(n,n+7,r,r+3) );
  return 0;
}
*/

//using iterators
int main()
{
  vector<string> n;
  n.push_back("1");
  n.push_back("2");
  n.push_back("3");
  n.push_back("4");
  n.push_back("5");
  n.push_back("6");
  n.push_back("7");
  n.push_back("8");
  vector<string> r;
  r.push_back("6");
  r.push_back("7");
  r.push_back("8");
  
  do
  {
    for(vector<string>::iterator it=r.begin();it!=r.end();++it)
      cout<<*it;
    cout<<endl;    
  }
  while( prev_combination(n.begin(),n.end(),r.begin(),r.end() ) );
  return 0;
}
