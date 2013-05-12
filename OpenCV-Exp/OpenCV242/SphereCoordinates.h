#ifndef _SPHERE_COORDINATES
#define _SPHERE_COORDINATES

#include <stdio.h>
#include <math.h>

void cartesianToSpherical( double &r, double &theta, double &phi, double x, double y, double z);
void sphericalToCartesian( double &x, double &y, double &z, double r, double theta, double phi);

int main_sphere_coordinates(int argc, char* argv[]);

#endif