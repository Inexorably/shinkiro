#include <iostream>

#include "linkage.h"

int main(){
    //Test the inverse dynamics with some default linkage.
    shinkiro::Linkage l;

    std::cout << l.f_inverseDynamics();



    return 1;
}