#ifndef VELOCITY_H
#define VELOCITY_H

#pragma once

#include <stdexcept>

using namespace std;

class Velocity {
    float dx;
    float dy;
    float dz;
public:
    Velocity () : dx(0), dy(0), dz(0) {}
    Velocity (float dx0, float dy0, float dz0) : dx(dx0), dy(dy0), dz(dz0) {}
    Velocity& operator= (const Velocity&);
	Velocity& operator+=(const Velocity&);
    bool operator< (const Velocity&) const;
    float getdX (void) const;
    float getdY (void) const;
    float getdZ (void) const;
};
#endif