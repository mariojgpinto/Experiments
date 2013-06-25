#include <_ID.h>

#include "SphereCoordinates.h"
#include "FaceDetect.h"
#include "EyeDetect.h"
#include "CamShift.h"
#include "CenterOfMass.h"
#include "Mapping.h"
#include "CV_Remapping.h"
#include "CV_Homography.h"
#include <opencv2\opencv.hpp>

int main( int argc, char* argv[] )
{
	//cv::Vec3f vec(-0.0124,-0.1659,0.1449);

	//double coiso = cv::norm(vec.operator cv::Matx<float, 3, 1>());
	//double coiso2 = sqrt(vec.val[0]*vec.val[0] + vec.val[1]*vec.val[1] + vec.val[2]*vec.val[2]);

	//printf("%.6f\n",coiso);
	//printf("%.6f\n",coiso2);

	//printf("p - %.3f %.3f %.3f = %.2f", ((vec.val[0]*vec.val[0])/(coiso*coiso))*100, 
	//									((vec.val[1]*vec.val[1])/(coiso*coiso))*100, 
	//									((vec.val[2]*vec.val[2])/(coiso*coiso))*100,
	//									((vec.val[0]*vec.val[0])/(coiso*coiso))*100+
	//									((vec.val[1]*vec.val[1])/(coiso*coiso))*100+ 
	//									((vec.val[2]*vec.val[2])/(coiso*coiso))*100);

	//main_sphere_coordinates(argc, argv);
	//return main_face_detect(argc,argv);
	//return main_cam_shift(argc,argv);
	//main_eye_detect(argc,argv);
	//main_center_of_mass(argc,argv);
	//main_mapping(argc,argv);
	//main_cv_remapping(argc,argv);
	//main_cv_homography(argc,argv);

	int result;

#ifdef _CCG
	result = main_mapping(argc,argv);
#endif

#ifdef _HOME
	result = main_cam_shift(argc,argv);
#endif

	return 0;
}
