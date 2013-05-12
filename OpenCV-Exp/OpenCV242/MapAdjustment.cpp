#include <opencv2\opencv.hpp>


cv::Mat3b orig_map;
cv::Mat3b orig_map_flipped;
cv::Mat3b orig_map_resized;
cv::Mat3b orig_map_rotated;
cv::Mat3b orig_map_moved;
cv::Mat3b orig_map_fitt;
cv::Mat3b orig_map_pointed;

cv::Mat1b kinect_map;
cv::Mat3b kinect_map_color;
cv::Mat3b kinect_map_color_fitt;

cv::Mat3b mix_map;

cv::Mat map_x, map_y;


//scale = 118 = 1.18 | rotate = 17 => -3 | MoveX = 504 => 204 | MoveY = 231 => -69
//double scale = 1.18;
double scale = 1.;
//double rotation = -3.0;
double rotation = 0;

int min_rot = 20, max_rot = 40, middle_rot = 20;
int min_scale = 100, max_scale = 200;

int move_x = 0;
int move_y = 0;
int min_x = 300, max_x = 600, middle_x = 300;
int min_y = 300, max_y = 600, middle_y = 300;

cv::Size last_fitt; 

int flip = 0;
int flip_code;

void create_map(cv::Size size, cv::Mat3b src, float scale = 0){
	//map_x.create(src.size(),CV_32FC1);
	//map_y.create(src.size(),CV_32FC1);

	//float half;

	//for( int i = 0 ; i < src.rows ; i++){
	//	for( int j = 0 ; j < src.cols ; j++){

	//	}
	//}
}

void unproject_point(cv::Point point, cv::Mat3b &dest){
	orig_map.copyTo(dest);

	cv::Point aux(point);
	
	//Undo Last Fitt
	aux.x -= last_fitt.width;
	aux.y -= last_fitt.height;

	//Undo Move
	aux.x -= move_x;
	aux.y -= move_y;

	////Undo Rotation
	float rad = ((rotation) * (CV_PI/180));
	float cenas_x = (orig_map_rotated.cols/2.0) - aux.x;
	float cenas_y = (orig_map_rotated.rows/2.0) - aux.y;

	//cv::circle(orig_map_rotated,aux,7,cv::Scalar(0,255,0),-1);

	int x_ = cenas_x * cos(rad) - cenas_y * sin(rad); 
	int y_ = cenas_x * sin(rad) + cenas_y * cos(rad); 

	aux.x = (orig_map_rotated.cols/2.0) - x_;
	aux.y = (orig_map_rotated.rows/2.0) - y_;

	//cv::circle(orig_map_rotated,cv::Point(x_,y_),7,cv::Scalar(255,0,0),-1);
	//cv::circle(orig_map_rotated,cv::Point(orig_map_rotated.cols/2.0,orig_map_rotated.rows/2.0),7,cv::Scalar(0,0,255),-1);
	//cv::circle(orig_map_rotated,aux,7,cv::Scalar(0,255,255),3);
	//cv::imshow("asd",orig_map_rotated);
	//cv::waitKey();

	int diff_x = (orig_map_rotated.cols - orig_map_resized.cols)/2.0;
	int diff_y = (orig_map_rotated.rows - orig_map_resized.rows)/2.0;

	aux.x -= diff_x;
	aux.y -= diff_y;

	////Undo Scale
	aux.x /= scale;
	aux.y /= scale;

	if(flip){
		if(flip == 1){ // flipping around the x-axis
			aux.y = dest.size().height - aux.y;
		}
		if(flip == 2){ // flipping around the y-axis 
			aux.x = dest.size().width - aux.x;
		}
		if(flip == 3){ // flipping around both axes
			aux.x = dest.size().width - aux.x;
			aux.y = dest.size().height - aux.y;
		}
	}

	cv::circle(dest,aux,5,cv::Scalar(0,0,255),-1);

	cv::imshow("map",dest);
	//cv::waitKey();
	printf("DAFUCK(%d,%d)",aux.x,aux.y);

}

void fitt(cv::Mat3b src, cv::Mat3b &dest, cv::Size size){
	dest.create(size);
	cv::Mat3b aux(size,cv::Vec3b(0,0,0));

	cv::Size new_size; 
	last_fitt.width = new_size.width = (size.width - src.cols) / 2.0;
	last_fitt.height = new_size.height = (size.height - src.rows) / 2.0;

	cv::Point2f srcTri[3];
	cv::Point2f dstTri[3];

	   /// Set your 3 points to calculate the  Affine Transform
	srcTri[0] = cv::Point2f( 0,0 );
	srcTri[1] = cv::Point2f( 0,1);
	srcTri[2] = cv::Point2f( 1,0);

	dstTri[0] = cv::Point2f( new_size.width,new_size.height);
	dstTri[1] = cv::Point2f( new_size.width, new_size.height+1);
	dstTri[2] = cv::Point2f( new_size.width+1, new_size.height);

	cv::Mat warp_mat( 2, 3, CV_32FC1 );
	warp_mat = getAffineTransform( srcTri, dstTri );

	cv::warpAffine( src, aux, warp_mat, dest.size() );

	aux.assignTo(dest);
}

void rotateImage(cv::Mat3b src, cv::Mat3b &dest, float angleDegrees)
{
	float diagonal = sqrt(pow(src.cols,2.0) + pow(src.rows,2.0));
	CvSize sizeRotated;
	sizeRotated.width = diagonal;
	sizeRotated.height = diagonal;

	cv::Mat3b aux(sizeRotated,cv::Vec3b(0,0,0));

	dest.create(sizeRotated);

	src.copyTo(aux(cv::Rect((sizeRotated.width - src.cols)/2.0,(sizeRotated.height - src.rows)/2.0,src.cols,src.rows)));
	cv::Mat rot_mat = cv::getRotationMatrix2D( cv::Point(sizeRotated.width/2.0,sizeRotated.height/2.0), angleDegrees, 1 );

	// Transform the image
	cv::warpAffine(aux,aux,rot_mat,sizeRotated,CV_INTER_LINEAR,0,cv::Scalar(0,0,0));	//cvGetQuadrangleSubPix( src, imageRotated, &M);

	aux.assignTo(dest);
}

void scaleImage(cv::Mat3b src, cv::Mat3b &dest, float scale_){
	cv::Size resize(src.cols*scale_,src.rows*scale_);
	cv::Mat3b aux(resize,cv::Vec3b(0,0,0));
	cv::resize(src,aux,resize);
	aux.assignTo(dest);
}

void moveImage(cv::Mat3b src,cv::Mat3b &dest, int px, int py){
	dest.create(src.size());
	cv::Mat3b aux(src.size());
	aux.zeros(src.size());
	 
	cv::Point2f srcTri[3];
	cv::Point2f dstTri[3];

	   /// Set your 3 points to calculate the  Affine Transform
	srcTri[0] = cv::Point2f( 0,0 );
	srcTri[1] = cv::Point2f( 0,1);
	srcTri[2] = cv::Point2f( 1,0);

	dstTri[0] = cv::Point2f( px,py);
	dstTri[1] = cv::Point2f( px, py+1);
	dstTri[2] = cv::Point2f( px+1, py);
	cv::Mat warp_mat( 2, 3, CV_32FC1 );
	warp_mat = getAffineTransform( srcTri, dstTri );

	cv::warpAffine( src, dest, warp_mat, dest.size() );
}

void on_trackbar_rotation( int, void* )
{
	rotation = min_rot - middle_rot;
}

void on_trackbar_scale( int, void* )
{
	if(min_scale > 40)
		scale = min_scale/100.0;
}

void on_trackbar_move_x( int, void* )
{
	move_x = min_x - middle_x;
}

void on_trackbar_move_y( int, void* )
{
	move_y = min_y - middle_y;
}

void on_tracker_ground_mouse(int event, int x, int y, int flags, void *void_data){
	if (event != CV_EVENT_LBUTTONUP)
        return;

	cv::Point p(x,y);
	unproject_point(p,orig_map_pointed);

}

void set_flip_mode(int flip_flag){
	switch(flip_flag){
		case 1: // flipping around the x-axis
			flip_code = 0;
			break;
		case 2: // flipping around the y-axis
			flip_code = 1;
			break;
		case 3: // flipping around both axes
			flip_code = -1;
			break;
		default:orig_map.copyTo(orig_map_flipped);
	}
}

int main(int argc, char *argv[]){
	orig_map = cv::imread("ccg_map.bmp",1);
	orig_map.copyTo(orig_map_flipped);

	kinect_map = cv::imread("kinect_map.bmp",0);
	cv::cvtColor(kinect_map,kinect_map_color,CV_GRAY2RGB);
	
	cv::namedWindow("mix");
	cv::namedWindow("controls",0);
	cv::moveWindow("controls",1280,0);
	cv::createTrackbar("Rotate","controls",&min_rot,max_rot,on_trackbar_rotation);
	cv::createTrackbar("Scale","controls",&min_scale,max_scale,on_trackbar_scale);
	cv::createTrackbar("MoveY","controls",&min_y,max_y,on_trackbar_move_y);
	cv::createTrackbar("MoveX","controls",&min_x,max_x,on_trackbar_move_x);
	
	cv::Mat3b img(cv::Size(640,1),cv::Vec3b(0,0,0));
	cv::imshow("controls",img);
	cv::waitKey(1);

	cv::setMouseCallback("mix", on_tracker_ground_mouse);
	//cv::createButton("flip V",callback_button_flip_v,NULL,CV_CHECKBOX);
	//cv::




	char c = 0;
	while((c = cv::waitKey(30)) != 27){
		if(c == 'f'){
			flip = (flip + 1) % 4;
			set_flip_mode(flip);

			if(flip){
				cv::flip(orig_map,orig_map_flipped,flip_code);
			}
		}

		scaleImage(orig_map_flipped,orig_map_resized,scale);

		rotateImage(orig_map_resized,orig_map_rotated,rotation);

		moveImage(orig_map_rotated,orig_map_moved,move_x,move_y);

		fitt(orig_map_moved,orig_map_fitt,kinect_map_color.size());

		//cv::imshow("transform",orig_map_moved);
		//cv::imshow("controls");
		
		//cv::imshow("orig",orig_map);
		//cv::imshow("kinect",kinect_map_color);
		//cv::imshow("orig_map_resized",orig_map_resized);
		cv::imshow("mix",kinect_map_color + orig_map_fitt*0.5);
	}

	cv::destroyAllWindows();

	return 0;
}
