#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <iostream>

using namespace cv;
using namespace std;

Mat world;
Point startPt(0,0);
Point endPt(0,0);
float dist;

void onMouse( int event, int x, int y, int flags, void* )
{
    if( event == CV_EVENT_LBUTTONUP) startPt = Point(x,y);
    if( event == CV_EVENT_RBUTTONUP) {
        endPt   = Point(x,y);
        Vec3f s = world.at<Vec3f>(startPt.y, startPt.x);
        Vec3f e = world.at<Vec3f>(endPt.y, endPt.x);
        float dx = e[0]-s[0];
        float dy = e[1]-s[1];
        float dz = e[2]-s[2];
        dist = sqrt(dx*dx + dy*dy + dz*dz);
    }
}


int main( /*int argc, char* argv[]*/ ){
    VideoCapture capture;
    capture.open(CV_CAP_OPENNI);
    capture.set( CV_CAP_PROP_OPENNI_REGISTRATION , 0);

    if( !capture.isOpened() ){
        cout << "Can not open a capture object." << endl;
        return -1;
    }

    namedWindow( "depth", 1 );
    setMouseCallback( "depth", onMouse, 0 );
    for(;;){
        if( !capture.grab() ){
            cout << "Can not grab images." << endl;
            return -1;
        }else{
            Mat depthMap,show;
            capture.retrieve(world, CV_CAP_OPENNI_POINT_CLOUD_MAP);
            if( capture.retrieve( depthMap, CV_CAP_OPENNI_DEPTH_MAP ) ) depthMap.convertTo( show, CV_8UC1, 0.05f);
            line(show,startPt,endPt,Scalar(255));
            putText(show,format("distance: %f m",dist),Point(5,15),FONT_HERSHEY_PLAIN,1,Scalar(255));
            imshow("depth",show);
        }
        if( waitKey( 30 ) >= 0 )    break;
    }
}