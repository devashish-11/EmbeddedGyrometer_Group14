#ifndef LOCATION_H
#define LOCATION_H

#pragma once

#include <stdexcept>

using namespace std;

class Location {
    float x;
    float y;
    float z;
public:
    Location () : x(0), y(0), z(0) {}
    Location (float x0, float y0, float z0) : x(x0), y(y0), z(z0) {}
    Location& operator= (const Location&);
	Location& operator+=(const Location&);
    bool operator< (const Location&) const;
    float getX (void) const;
    float getY (void) const;
    float getZ (void) const;
};

#endif