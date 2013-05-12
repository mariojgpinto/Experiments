#include "SphereCoordinates.h"

/******************************//**
 * \brief Conversion from cartesian to spherical coordinates (if the
 *        resulting radius is 0 also the azimutal and inclination
 *        angles get set to 0).
 *
 * @param r      Reference to return the radius
 * @param theta  Reference to return the inclination angle (in radian)
 * @param phi    Reference to return the azimutal angle (in radian)
 * @param x      x coordinate
 * @param y      y coordinate
 * @param z      z coordinate
 ******************************/

void
cartesianToSpherical( double & r,
                      double & theta,
                      double & phi,
                      double   x,
                      double   y,
                      double   z )
{
    if ( ( r = sqrt( x * x + y * y + z * z ) ) != 0.0 )
    {
        theta = acos( z / r );
        phi   = atan2( y, x );
    }
    else
        theta = phi = 0.0;
}


/******************************//**
 * \brief Conversion from spherical to cartesian coordinates
 *
 * @param x      Reference to return the x coordinate
 * @param y      Reference to return the y coordinate
 * @param z      Reference to return the z coordinate
 * @param r      Radius (must be non-negative)
 * @param theta  Inclination angle (in radian)
 * @param phi    Azimutal angle (in radian)
 ******************************/

void
sphericalToCartesian( double & x,
                      double & y,
                      double & z,
                      double   r,
                      double   theta,
                      double   phi )
{
        if ( r < 0.0 )
                throw "Negative radius in sphericalToCartesian()";

    x = r * sin( theta ) * cos( phi );
    y = r * sin( theta ) * sin( phi );
    z = r * cos( theta );
}


int main_sphere_coordinates(int argc, char* argv[]){
	double ptx = -0.03, pty = 0.09, ptz = -0.16;
	double sfr, sft,sfp;
	double ptx2, pty2, ptz2;

	cartesianToSpherical(sfr, sft, sfp, ptx,pty,ptz);

	printf("");

	sphericalToCartesian(ptx2, pty2, ptz2, sfr,sft,sfp);

    return 0;
}