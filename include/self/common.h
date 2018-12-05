//
// Created by tonychen on 18-11-23.
//

#ifndef COMMON_H
#define COMMON_H

// define the commonly included file to avoid a long include list

// for ceres
#include <ceres/ceres.h>
#include <ceres/rotation.h>

// std
#include <vector>
#include <list>
#include <memory>
#include <string>
#include <iostream>
#include <fstream>
#include <set>
#include <unordered_map>
#include <map>
namespace Slam2D
{
    template <typename T>
    struct Point2{
        T x;
        T y;
        Point2(){}
        Point2(const T x, const T y):x(x), y(y){}
        Point2(const T* xy):x(xy[0]), y(xy[1]){}
        static Point2 zeros(){return Point2(T(0), T(0));}
        static Point2 ones(){return Point2(T(1), T(1));}

        Point2 operator-(const Point2& a) const {
            return Point2(this->x-a.x, this->y-a.y);
        }
        Point2 operator+(const Point2& a) const {
            return Point2(this->x+a.x, this->y+a.y);
        }
        Point2 operator*(const Point2& a) const {
            return Point2(this->x*a.x, this->y*a.y);
        }
        Point2 operator*(const T a) const {
            return Point2(this->x*a, this->y*a);
        }
        Point2 rotate(T theta){
            return Point2(x*cos(theta)-y*sin(theta), x*sin(theta)+y*cos(theta));}
    };
    typedef Point2<double> Point2d;
    typedef Point2<float> Point2f;
}
using namespace std;
#endif //COMMON_H