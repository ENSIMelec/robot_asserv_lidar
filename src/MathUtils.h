//
// Created by Taoufik on 16/11/2019.
//

#ifndef ROBOT_MATHUTILS_H
#define ROBOT_MATHUTILS_H

#include "cmath"


class MathUtils final {

public:

    static float periodicmod(float x, float y);
    static float inrange(float x, float min, float max);
    static float constrain(float value, float min, float max);
    static float deg2rad(float deg);
    static float rad2deg(float rad);
    static float micros2sec(float sec);
	static float millis2sec(float millis);
};


#endif //ROBOT_MATHUTILS_H
