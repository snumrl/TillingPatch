//This recv_comb_ex.cpp tests the recursive combination function out
#include<iostream>
#include<vector>
#include<string>
#include"combination.h"


// for use with next_combination examples!
template<class BidIt>
void display(BidIt begin,BidIt end)
{
  for (BidIt it=begin;it!=end;++it)
      cout<<*it<<" ";
  cout<<endl;
}


using namespace std;
using namespace stdcomb;



//test next_combination() with pointers

int main()
{
  char ca[]="123456";
  char cb[]="1234";
   
  do
  {
    cout<<cb<<endl;
  }
  while(next_combination(ca,ca+6,cb,cb+4));
  cout<<"Complete!"<<endl;
  
  return 0;
}


/*
//test next_combination() with iterators
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
   
  do
  {
    display(cb.begin(),cb.end());
  }
  while(next_combination(ca.begin (),ca.end (),cb.begin (),cb.end()) );
  
  cout<<"Complete!"<<endl;
  
  return 0;
}
*/

/*
//test next_combination() with strings
int main()
{
  vector<string> str;
  str.push_back ("Red");
  str.push_back ("Green");
  str.push_back ("Blue");
  vector<string> str1;
  str1.push_back ("Red");
  str1.push_back ("Green");
   
  do
  {
    display(str1.begin(),str1.end());
  }
  while(next_combination(str.begin (),str.end (),str1.begin (),str1.end()));

  cout<<"Complete!"<<endl;
  
	return 0;
  
}

*/