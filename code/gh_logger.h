#pragma once

#include <vector>

struct Log{
	std::vector<int> q;

	int nminus;
	int nplus;
	int nextra1;
	int nextra2;

	Log():
		nminus(0),
		nplus(0),
		nextra1(0),
		nextra2(0)
	{
	};
};

Log ghlog;

void addq(int n){
	if(n==0)
		ghlog.nplus++;
	else if(n==1)
		ghlog.nminus++;
	else if(n==2)
		ghlog.nextra1++;
	else if(n==3)
		ghlog.nextra2++;
}