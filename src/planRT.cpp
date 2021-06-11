//
// Created by Capoo on 2021/2/6.
//

#include "planRT.h"

#include<cmath>
#include<iostream>
#include <algorithm>
#include <array>
#include <stdlib.h>
#include <string>
#include <bitset>
#include "kaanh.h"
#include "robot.h"

using namespace std;

auto Curve::getCurve(int count) -> double {
    if (count==1){
        flag=0;
    }
    if (flag == 0){
        member_count = count;
        flag = 1;
    }
        double s = 0;
        int t = count - member_count + 1;
        cout<<"t:"<<t<<endl;
        std::cout << "member_count: " << member_count <<std::endl;

        // Now let's generate the target curve!
        if (2 * tr_ < T_){
                // Generate the T-Curve.

                    if (t <= tr_ * 1000) {
                        s = 0.5 * a_ * t * t / 1000 / 1000;
                    } else if (t > tr_ * 1000 && t < (T_ * 1000 - tr_ * 1000)) {
                        s = v_ * t / 1000 - v_ * v_ / 2.0 / a_;
                    } else {
                        s = (2 * a_ * v_ * T_ - 2 * v_ * v_ - a_ * a_ * (t / 1000.0 - T_) * (t / 1000.0 - T_)) / (2 * a_);
                        if (s>=1){flag = 0;
                        cout<<"flag2"<<flag<<endl;}
                    }
               std::cout << "s: " << s <<std::endl;
           }
        else{

                // Generate the Triangle-Curve.

                    if (t < tr_ * 1000) {
                        s = 0.5 * a_ * t * t / 1000.0 / 1000.0;
                    } else {
                        s = 0.5 * a_ * tr_ * tr_ + 0.5 * (t / 1000.0 - tr_) * (2 * v_ - a_ * (t / 1000.0 - tr_));
                        if (s>=1){flag = 0;
                        cout<<"flag3"<<flag<<endl;}
                    }

             std::cout << "s: " << s <<std::endl;


        }
        return s;

}

