#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <iostream>

using namespace cv;
using namespace std;

Mat world;
Mat1f depthMap;
Mat show;
Mat show_diff;
Mat show_diff_erode;
Mat show_avg;
Mat rgb;

bool train = false;
int ac_train = 0;
std::vector<cv::Mat1f*> _background_images_depth;
bool avg_ready = false;
cv::Mat1f _average_depth_image;

bool diff_ready = false;
Mat1f diff;
Mat diff_bin;
Mat1b diff_bin_erode;
Mat1f diff_erode;
cv::Mat *_element_7;

void process_key(char c){
	switch(c){
		case 't':
			namedWindow( "average", 1 );
			train = true;
			break;
		case 'd':
			//namedWindow( "diff", 1 );
			//namedWindow( "diff_erode", 1 );
			diff_ready = true;
			_element_7 = new cv::Mat(7,7,CV_8U,CV_SHAPE_ELLIPSE);
			//diff_bin.create(_average_depth_image.size());
			_average_depth_image.copyTo(diff);
			break;
		case 27:
			exit(0);
		default:break;
	}

}

void calc_average_subtraction_image(){
	Mat1f aux;
	_background_images_depth.at(0)->copyTo(aux);


	for(int i = 0 ; i < _background_images_depth.at(0)->rows ; i++){
		for(int j = 0 ; j < _background_images_depth.at(0)->cols ; j++){
			float ac = 0;
			int blanc = 0;
			for(int k = 0 ; k < _background_images_depth.size() ; k++){
				if((*_background_images_depth.at(k))[i][j] == 0)
					blanc++;
				ac += (*_background_images_depth.at(k))[i][j];
			}
			aux[i][j] = ac/((float)(_background_images_depth.size() - blanc));
		}
	}
	aux.copyTo(_average_depth_image);
	aux.convertTo( show_avg, CV_8UC1, 0.05f);
	imshow("sfaf",show_avg);
	//aux.convertTo( _average_depth_image, CV_8UC1, 0.05f);
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
	

    //setMouseCallback( "depth", onMouse, 0 );
    for(;;){
        if( !capture.grab() ){
            cout << "Can not grab images." << endl;
            return -1;
        }else{
            
            //capture.retrieve(world, CV_CAP_OPENNI_POINT_CLOUD_MAP);
            if( capture.retrieve( depthMap, CV_CAP_OPENNI_DEPTH_MAP ) ) depthMap.convertTo( show, CV_8UC1, 0.05f);
			capture.retrieve( rgb, CV_CAP_OPENNI_BGR_IMAGE );

			if(train){
				cv::Mat1f *frame_aux = new cv::Mat1f();
				depthMap.copyTo(*frame_aux);
				_background_images_depth.push_back(frame_aux);


				if(ac_train++ < 10){
					calc_average_subtraction_image();
					
					ac_train = 0;
					train = false;
					avg_ready = true;
				}
			}

			if(diff_ready){
				cv::absdiff(show,show_avg,diff);
				diff.convertTo( show_diff, CV_8U);
				
				cv::threshold(diff,diff_bin,10,255,CV_THRESH_BINARY);


//				cv::erode(diff,diff_erode,*_element_7);

				//diff.convertTo( show_diff, CV_8UC1, 0.05f);
				//diff_erode.convertTo( show_diff_erode, CV_8U);
			}

            //line(show,startPt,endPt,Scalar(255));
            //putText(show,format("distance: %f m",dist),Point(5,15),FONT_HERSHEY_PLAIN,1,Scalar(255));
            imshow("depth",show);
			if(avg_ready)imshow("average",_average_depth_image);
			if(diff_ready){
				imshow("diff",show_diff);
				imshow("diff_erode",diff_bin);
			}

        }
		
		process_key(waitKey(30));

    }
}