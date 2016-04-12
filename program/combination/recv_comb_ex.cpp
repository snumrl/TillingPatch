//This recv_comb_ex.cpp tests the recursive combination function out
#include<iostream>
#include<vector>
#include<string>
#include"combination.h"

using namespace std;
using namespace stdcomb;


//test recursive_combination() with pointers

void display(char* begin,char* end)
{
  cout<<begin<<endl;
}
int main()
{
  char ca[]="123456";
  char cb[]="1234";
   
  recursive_combination(ca,ca+6,0,
                  cb,cb+4,0,6-4,display);
  cout<<"Complete!"<<endl;
	return 0;
}


/*
//test recursive_combination() with iterator
typedef vector<int>::iterator vii;

void display(vii begin,vii end)
{
  for (vii it=begin;it!=end;++it)
      cout<<*it;
  cout<<endl;
}

int main()
{
  vector<int> ca;
  ca.push_back (1);
  ca.push_back (2);
  ca.push_back (3);
  ca.push_back (4);
  ca.push_back (5);
  ca.push_back (6);
  vector<int> cb;
  cb.push_back (1);
  cb.push_back (2);
  cb.push_back (3);
  cb.push_back (4);
   
  recursive_combination(ca.begin (),ca.end(),0,
                  cb.begin(),cb.end(),0,6-4,display);
  cout<<"Complete!"<<endl;
	return 0;
}
*/

/*
//test recursive_combination() with array of strings

void display(string* begin,string* end)
{
  for (string* it=begin;it!=end;++it)
      cout<<*it<<" ";
  cout<<endl;
}


int main()
{
  string* strarray=new string[3];
  strarray[0]="Red";
  strarray[1]="Green";
  strarray[2]="Blue";
  string* strarray1=new string[2];
  strarray1[0]="Red";
  strarray1[1]="Green";
  
  recursive_combination(strarray,strarray+3,0,
                  strarray1,strarray1+2,0,3-2,display);
  cout<<"Complete!"<<endl;
  delete [] strarray;
  delete [] strarray1;

	return 0;
  
}
*/

