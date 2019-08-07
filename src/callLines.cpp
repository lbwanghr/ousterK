#include <iostream>
#include <stdio.h>
#include <thread>
#include "ouster/lines.h"

#ifdef __cplusplus
extern "C" {
#endif
Lines lines;
double data[88];

void printData(const double data[], int size){
	std::cout<<std::setw(13);
	for(int i = 0; i < 64; i++){	
		std::cout<<data[i]<<std::setw(13);
		if(i==15) std::cout<<std::endl;
		if(i==31) std::cout<<std::endl;
		if(i==47) std::cout<<std::endl;
	}	
	std::cout << std::endl;
	for(int i = 64; i < size; i++){	
		std::cout<<data[i]<<" ";
	}	
	std::cout << std::endl;
	std::cout << "RRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRR" << std::endl;
}

void run_ouster(){
    lines.run();
}


double* get_obs(){
    lines.getData(data);
    double* p;
    p = &data[0];
    return p;
}

#ifdef __cplusplus
}
#endif

