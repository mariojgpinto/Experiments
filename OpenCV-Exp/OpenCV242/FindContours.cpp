#include <opencv2\opencv.hpp>

void manage_bounding_boxes(std::vector<cv::Rect>* bb, cv::Rect* mean){
	//TODO TEST Join multiple bounding boxes
	if(!bb || !mean) return;

	int xx = INT_MAX,yy = INT_MAX, ww = INT_MIN,hh = INT_MIN;
	for(unsigned int i = 0 ; i < bb->size() ; i++){
		//if(bb->at(i).x >= 0 && bb->at(i).y >= 0 && bb->at(i).width >= 0 && bb->at(i).height >= 0){
			if(bb->at(i).x < xx) xx = bb->at(i).x;
			if(bb->at(i).y < yy) yy = bb->at(i).y;
			if((bb->at(i).x + bb->at(i).width) > ww) ww = (bb->at(i).x + bb->at(i).width);
			if((bb->at(i).y + bb->at(i).height) > hh) hh = (bb->at(i).y + bb->at(i).height);
		//}
	}

	mean->x = xx;
	mean->y = yy;
	mean->width = ww-xx;
	mean->height = hh-yy;
}

int main(){
	cv::Mat3b img = cv::imread("contours2.png");
	cv::Mat img_f;
	cv::Mat1b img_b(img.size());


	cv::Mat3b out(img.size());

	cv::cvtColor( img, img_f, CV_BGR2GRAY );
	cv::threshold(img_f,img_b,0.1,255,cv::THRESH_BINARY);



	cv::vector<cv::vector<cv::Point> > contours;
          cv::vector<cv::Vec4i> hierarchy;

		 // img_b.convertTo(coiso,CV_32F);

        cv::Mat threshold_output;

		
		cv::findContours( img_b, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );

		for( int i = 0; i < contours.size(); i++ ){
			cv::drawContours(out,contours,i,cv::Scalar(255,0,0),-1,16);
		}



		cv::Rect bbox;

	if(contours.size()){
		//Multiples BBoxes
		if(contours.size() > 1){
			//Create array of BBoxes
			cv::vector<cv::Rect> bound_rects( contours.size() );

			for(unsigned int i = 0; i < contours.size(); i++ ){
				bound_rects[i] = boundingRect( cv::Mat(contours[i]) );
			}

			//Join Bounding Boxes (Only one BBoxe per user)
			manage_bounding_boxes(&bound_rects,&bbox);
			//bbox = boundingRect( cv::Mat(contours[0]) );
		}
		else{
			//Only one user
			bbox = boundingRect( cv::Mat(contours[0]) );

		}
	}

	cv::rectangle(out,bbox,cv::Scalar(0,0,255));

	cv::imshow("sa",out);
	//cv::imwrite("img_q_2.png",img);
	cv::waitKey();

	return 0;
}