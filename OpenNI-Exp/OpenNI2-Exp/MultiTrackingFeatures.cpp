#include "MultiTrackingFeatures.h"

#include <NIKinect2Manager.h>

namespace{

#define M_PI 3.141592654

class User{
	public:
		User():_hog_hist_bbox(cv::vector<float>(9)),_hog_hist_mask(cv::vector<float>(9)){}
		User(User* user){
			this->_bbox = user->_bbox;
			this->_centroid_3d = user->_centroid_3d;
			this->_centroid_2d = user->_centroid_2d;

			user->_color_full.copyTo(this->_color_full);
			user->_mask_full.copyTo(this->_mask_full);
		
			user->_color_user.copyTo(this->_color_user);
			user->_mask_user.copyTo(this->_mask_user);

			this->_id = user->_id;
			this->_conf = user->_conf;
		}
		~User(){}

		cv::Rect	_bbox;
		cv::Point3f	_centroid_3d;
		cv::Point	_centroid_2d;

		cv::Mat _color_full;
		cv::Mat _mask_full;
		
		cv::Mat _color_user;
		cv::Mat _mask_user;
		
		cv::Mat _gray_user_bbox;
		cv::Mat _gray_user_mask;

		cv::Mat _hog_vis_bbox;
		cv::Mat _hog_vis_mask;

		cv::vector<float> _hog_hist_bbox;
		cv::vector<float> _hog_hist_mask;
		cv::Mat _hog_vis_hist_bbox;
		cv::Mat _hog_vis_hist_mask;

		int _id;
		double _conf;
};

void init_user(User* user, NIKinect2* kinect, nite::UserData* data, cv::Mat color){
	if(!user || !kinect || !data || !color.rows || !color.cols) return;

	color.copyTo(user->_color_full);

	const nite::BoundingBox bbox_nite = data->getBoundingBox();

	user->_bbox.x = bbox_nite.min.x * 2;
	user->_bbox.y = bbox_nite.min.y * 2;
	user->_bbox.width = bbox_nite.max.x * 2 - bbox_nite.min.x * 2;
	user->_bbox.height = bbox_nite.max.y * 2 - bbox_nite.min.y * 2;



	nite::Point3f world_pt = data->getCenterOfMass();
	user->_centroid_3d.x = world_pt.x;
	user->_centroid_3d.y = world_pt.y;
	user->_centroid_3d.z = world_pt.z;

	float xx,yy,zz;

	openni::CoordinateConverter::convertWorldToDepth(*kinect->get_depth_stream(),
													 world_pt.x, world_pt.y, world_pt.z, 
													 &xx,&yy,&zz);

	user->_centroid_2d.x = xx * 2;
	user->_centroid_2d.y = yy * 2;

	cv::Mat mask_640;
	kinect->get_user_mask(data->getId(),mask_640);
	cv::resize(mask_640,user->_mask_full,cv::Size(1280,960));
	

	cv::dilate(user->_mask_full,user->_mask_full,cv::Mat(3,3,CV_8UC1));
	cv::erode(user->_mask_full,user->_mask_full,cv::Mat(7,7,CV_8UC1));

	color(user->_bbox).copyTo(user->_color_user);
	user->_mask_full(user->_bbox).copyTo(user->_mask_user);
}

void calc_lbp(){
	//cv::calculate_LBP_Histogram
}

int lowThreshold = 50;
int const max_lowThreshold = 100;

void canny(cv::Mat& image){
	cv::Mat gray;
	int edgeThresh = 1;
	
	
	int ratio = 3;
	int kernel_size = 3;

	cv::cvtColor(image,gray,CV_BGR2GRAY);

	cv::Canny( gray, gray, lowThreshold, lowThreshold*ratio, kernel_size );

	cv::imshow("Canny",gray);
}

cv::Mat get_hogdescriptor_visu(cv::Mat& origImg, cv::vector<float>& descriptorValues){   
    cv::Mat color_origImg;
    cvtColor(origImg, color_origImg, CV_GRAY2RGB);
 
    float zoomFac = 3;
    cv::Mat visu;
    resize(color_origImg, visu, cv::Size(color_origImg.cols*zoomFac, color_origImg.rows*zoomFac));
 canny(visu);
    int blockSize       = 16;
    int cellSize        = 8;
    int gradientBinSize = 9;
    float radRangeForOneBin = M_PI/(float)gradientBinSize; // dividing 180° into 9 bins, how large (in rad) is one bin?
 
    // prepare data structure: 9 orientation / gradient strenghts for each cell
    int cells_in_x_dir = origImg.cols / cellSize;
    int cells_in_y_dir = origImg.rows / cellSize;
    int totalnrofcells = cells_in_x_dir * cells_in_y_dir;
    float*** gradientStrengths = new float**[cells_in_y_dir];
    int** cellUpdateCounter   = new int*[cells_in_y_dir];
    for (int y=0; y<cells_in_y_dir; y++)
    {
        gradientStrengths[y] = new float*[cells_in_x_dir];
        cellUpdateCounter[y] = new int[cells_in_x_dir];
        for (int x=0; x<cells_in_x_dir; x++)
        {
            gradientStrengths[y][x] = new float[gradientBinSize];
            cellUpdateCounter[y][x] = 0;
 
            for (int bin=0; bin<gradientBinSize; bin++)
                gradientStrengths[y][x][bin] = 0.0;
        }
    }
 
    // nr of blocks = nr of cells - 1
    // since there is a new block on each cell (overlapping blocks!) but the last one
    int blocks_in_x_dir = cells_in_x_dir - 1;
    int blocks_in_y_dir = cells_in_y_dir - 1;
 
    // compute gradient strengths per cell
    int descriptorDataIdx = 0;
    int cellx = 0;
    int celly = 0;
 
    for (int blockx=0; blockx<blocks_in_x_dir; blockx++)
    {
        for (int blocky=0; blocky<blocks_in_y_dir; blocky++)            
        {
            // 4 cells per block ...
            for (int cellNr=0; cellNr<4; cellNr++)
            {
                // compute corresponding cell nr
                int cellx = blockx;
                int celly = blocky;
                if (cellNr==1) celly++;
                if (cellNr==2) cellx++;
                if (cellNr==3)
                {
                    cellx++;
                    celly++;
                }
 
                for (int bin=0; bin<gradientBinSize; bin++)
                {
                    float gradientStrength = descriptorValues[ descriptorDataIdx ];
                    descriptorDataIdx++;
 
                    gradientStrengths[celly][cellx][bin] += gradientStrength;
 
                } // for (all bins)
 
 
                // note: overlapping blocks lead to multiple updates of this sum!
                // we therefore keep track how often a cell was updated,
                // to compute average gradient strengths
                cellUpdateCounter[celly][cellx]++;
 
            } // for (all cells)
        } // for (all block x pos)
    } // for (all block y pos)
 
 
    // compute average gradient strengths
    for (int celly=0; celly<cells_in_y_dir; celly++)
    {
        for (int cellx=0; cellx<cells_in_x_dir; cellx++)
        {
 
            float NrUpdatesForThisCell = (float)cellUpdateCounter[celly][cellx];
 
            // compute average gradient strenghts for each gradient bin direction
            for (int bin=0; bin<gradientBinSize; bin++)
            {
                gradientStrengths[celly][cellx][bin] /= NrUpdatesForThisCell;
            }
        }
    }
 
 
    //cout << "descriptorDataIdx = " << descriptorDataIdx << endl;
 
    // draw cells
    for (int celly=0; celly<cells_in_y_dir; celly++)
    {
        for (int cellx=0; cellx<cells_in_x_dir; cellx++)
        {
            int drawX = cellx * cellSize;
            int drawY = celly * cellSize;
 
            int mx = drawX + cellSize/2;
            int my = drawY + cellSize/2;
 
            rectangle(visu, cv::Point(drawX*zoomFac,drawY*zoomFac), cv::Point((drawX+cellSize)*zoomFac,(drawY+cellSize)*zoomFac), CV_RGB(100,100,100), 1);
 
			float max = -1, max_id = 0;
			for (int bin=0; bin<gradientBinSize; bin++)
            {
				if(gradientStrengths[celly][cellx][bin] > max){
					max = gradientStrengths[celly][cellx][bin];
					max_id = bin;
				}
			}
            // draw in each cell all 9 gradient strengths
            //for (int bin=0; bin<gradientBinSize; bin++)
			int bin = max_id;
            {
                float currentGradStrength = gradientStrengths[celly][cellx][bin];
 
                // no line to draw?
                if (currentGradStrength==0)
                    continue;
 
                float currRad = bin * radRangeForOneBin + radRangeForOneBin/2;
 
                float dirVecX = cos( currRad );
                float dirVecY = sin( currRad );
                float maxVecLen = cellSize/2;
                float scale = 2.5; // just a visualization scale, to see the lines better
 
                // compute line coordinates
                float x1 = mx;// - dirVecX * currentGradStrength * maxVecLen * scale;
                float y1 = my;// - dirVecY * currentGradStrength * maxVecLen * scale;
                float x2 = mx + dirVecX * currentGradStrength * maxVecLen * scale;
                float y2 = my + dirVecY * currentGradStrength * maxVecLen * scale;
 
                // draw gradient visualization
                line(visu, cv::Point(x1*zoomFac,y1*zoomFac), cv::Point(x2*zoomFac,y2*zoomFac), CV_RGB(0,255,0), 1);
 
            } // for (all bins)
 
        } // for (cellx)
    } // for (celly)
 
 
    // don't forget to free memory allocated by helper data structures!
    for (int y=0; y<cells_in_y_dir; y++)
    {
      for (int x=0; x<cells_in_x_dir; x++)
      {
           delete[] gradientStrengths[y][x];            
      }
      delete[] gradientStrengths[y];
      delete[] cellUpdateCounter[y];
    }
    delete[] gradientStrengths;
    delete[] cellUpdateCounter;
 
    return visu;
 
} // get_hogdescriptor_visu

double fitt_image(cv::Mat& orig, cv::Mat& out, int out_width = 640, int out_height = 480, cv::Rect* roi_out = 0){
		//Test Input
		if(orig.cols <= 0 || orig.rows <= 0) return -1;

		//Create Output
		out = cv::Mat::zeros(cv::Size(out_width,out_height),orig.type());

		//Calc ROI and Resize Image
		if(!roi_out) roi_out = new cv::Rect();
		cv::Mat temp;
		int w,h;
		double output_ratio = (double)out.cols / (double)out.rows;
		double input_ratio = (double)orig.cols / (double)orig.rows;

		double ratio = 1.0;

		if(input_ratio < output_ratio){
			ratio = (float)out.rows/(float)orig.rows;
			w = orig.cols *ratio;
			h = orig.rows *ratio;

			cv::resize(orig,temp,cv::Size(w,h),0.0,0.0,cv::INTER_CUBIC);
		
			roi_out->x = (out.cols - w) / 2.0;
			roi_out->y = 0;
			roi_out->width = w;
			roi_out->height = h;
		}
		else{
			ratio = (float)out.cols/(float)orig.cols;
			w = orig.cols *ratio;
			h = orig.rows *ratio;

			cv::resize(orig,temp,cv::Size(w,h),0.0,0.0,cv::INTER_CUBIC);
		
			roi_out->x = 0;
			roi_out->y = (out.rows - h) / 2.0;
			roi_out->width = w;
			roi_out->height = h;
		}

		//Copy to Destination
		temp.copyTo(out(*roi_out));

		return ratio;
	}

bool calc_hog(User* user){
	if(!user) return false;
	
	//cv::cvtColor(user->_color_user,user->_gray_user_bbox,CV_RGB2GRAY);
	//user->_gray_user_bbox.copyTo(user->_gray_user_mask,user->_mask_user);

	cv::Mat gray_bbox;
	cv::Mat gray_mask;
	
	cv::cvtColor(user->_color_user,gray_bbox,CV_RGB2GRAY);
	gray_bbox.copyTo(gray_mask,user->_mask_user);

	fitt_image(gray_bbox,user->_gray_user_bbox,128,256);
	fitt_image(gray_mask,user->_gray_user_mask,128,256);

	

	//cv::Size s_bbox((gray_bbox.cols % 2) ? gray_bbox.cols-1 : gray_bbox.cols,
	//				(gray_bbox.rows % 2) ? gray_bbox.rows-1 : gray_bbox.rows);
	//cv::Size s_mask((gray_mask.cols % 2) ? gray_mask.cols-1 : gray_mask.cols,
	//				(gray_mask.rows % 2) ? gray_mask.rows-1 : gray_mask.rows);
	//cv::Size s_bbox(gray_bbox.cols,gray_bbox.rows);
	//cv::Size s_mask(gray_mask.cols,gray_mask.rows);

	//while(s_bbox.width % 8)
	//	s_bbox.width--;
	//while(s_bbox.height % 8)
	//	s_bbox.height--;
	//while(s_mask.width % 8)
	//	s_mask.width--;
	//while(s_mask.height % 8)
	//	s_mask.height--;


	//cv::resize(gray_bbox,user->_gray_user_bbox,s_bbox);
	//cv::resize(gray_mask,user->_gray_user_mask,s_mask);
	
	cv::Size cellSize(8,8);//(user->_gray_user_bbox.cols / 16,user->_gray_user_bbox.rows / 16);
	cv::Size winSize(128,256);//(user->_gray_user_bbox.cols,user->_gray_user_bbox.rows);		//(64,128)
	cv::Size blockSize(16,16);//(user->_gray_user_bbox.cols / 8,user->_gray_user_bbox.rows / 8);
	cv::Size blockStride(8,8);//((winSize.width - blockSize.width)/16,(winSize.height - blockSize.height)/16);
	//

	int q00 = blockSize.width % cellSize.width;
	int q01 = blockSize.height % cellSize.height;

	int q1 = winSize.width - blockSize.width;
	int q2 = q1 % blockStride.width;
	int q3 = winSize.height - blockSize.height;
	int q4 = q3 % blockStride.height;

	cv::HOGDescriptor		hog_bbox(winSize, blockSize, blockStride, cellSize, 9);
									//cv::Size(128,64), //winSize cv::Size(128,64)
									//cv::Size(16,16), //blocksize cv::Size(16,16)
									//cv::Size(8,8), //blockStride, cv::Size(8,8)
									//cv::Size(8,8), //cellSize, cv::Size(8,8)
									//9); //nbins,
	cv::vector<float>		ders_bbox;
	cv::vector<cv::Point>	locs_bbox;

	hog_bbox.compute(user->_gray_user_bbox,ders_bbox,cv::Size(64,64),cv::Size(0,0),locs_bbox);

	if(ders_bbox.size())
		user->_hog_vis_bbox = get_hogdescriptor_visu(user->_gray_user_bbox,ders_bbox);
	

	cv::HOGDescriptor		hog_mask(winSize, blockSize, blockStride, cellSize, 9);
									//cv::Size(128,64), //winSize cv::Size(128,64)
									//cv::Size(16,16), //blocksize cv::Size(16,16)
									//cv::Size(8,8), //blockStride, cv::Size(8,8)
									//cv::Size(8,8), //cellSize, cv::Size(8,8)
									//9); //nbins,
	cv::vector<float>		ders_mask;
	cv::vector<cv::Point>	locs_mask;

	hog_mask.compute(user->_gray_user_mask,ders_mask,cv::Size(64,64),cv::Size(0,0),locs_mask);
		
	user->_hog_vis_mask= get_hogdescriptor_visu(user->_gray_user_mask,ders_mask);
	

	//Calc Histograms
	//for(int i = 0 ; i < ders_mask.size() ; i+=9){
		for(int j = 0 ; j < 9 ; ++j){
			user->_hog_hist_bbox[j] = 0;
			user->_hog_hist_mask[j] = 0;
		}
	//}
	for(int i = 0 ; i < ders_mask.size() ; i+=9){
		for(int j = 0 ; j < 9; ++j){
			//user->_hog_hist_bbox[j] += ders_bbox[i + j];
			user->_hog_hist_mask[j] += ders_mask[i + j];
		}
	}
	
	for(int i = 0 ; i < ders_bbox.size() ; i+=9){
		for(int j = 0 ; j < 9; ++j){
			user->_hog_hist_bbox[j] += ders_bbox[i + j];
			//user->_hog_hist_mask[j] += ders_mask[i + j];
		}
	}

	int n_bins = 9;
	int hist_w = 270; int hist_h = 200;
	int bin_w = cvRound( (double) hist_w/n_bins );
	

	user->_hog_vis_hist_bbox = cv::Mat( hist_h, hist_w, CV_8UC3, cv::Scalar( 0,0,0) );
	user->_hog_vis_hist_mask = cv::Mat( hist_h, hist_w, CV_8UC3, cv::Scalar( 0,0,0) );

	cv::normalize(user->_hog_hist_bbox,user->_hog_hist_bbox);
	cv::normalize(user->_hog_hist_mask,user->_hog_hist_mask);

	for( int i = 1; i < n_bins; i++ )	{
		cv::line( user->_hog_vis_hist_bbox, 
						cv::Point( bin_w*(i-1), hist_h - hist_h*user->_hog_hist_bbox[i-1]),
						cv::Point( bin_w*(i)  , hist_h - hist_h*user->_hog_hist_bbox[i]  ),
						cv::Scalar( 255, 0, 0), 2, 8, 0  );

		cv::line( user->_hog_vis_hist_mask, 
						cv::Point( bin_w*(i-1), hist_h - hist_h*user->_hog_hist_mask[i-1]),
						cv::Point( bin_w*(i)  , hist_h - hist_h*user->_hog_hist_mask[i]  ),
						cv::Scalar( 0, 255, 0), 2, 8, 0  );
		//cv::line( user->_hsv_hist_v, cv::Point( bin_w*(i-1), hist_h - cvRound(user->_histogram_rgb_r.at<float>(i-1)) ) ,
		//				cv::Point( bin_w*(i), hist_h - cvRound(user->_histogram_rgb_r.at<float>(i)) ),
		//				cv::Scalar( 0, 0, 255), 2, 8, 0  );
	}

	//cv::line( user->_hog_vis_hist_bbox, 
	//			cv::Point( bin_w*(n_bins-1), hist_h - hist_h*user->_hog_hist_bbox[n_bins-1]),
	//				cv::Point( bin_w*(0)  , hist_h - hist_h*user->_hog_hist_bbox[0]  ),
	//				cv::Scalar( 255, 0, 0), 2, 8, 0  );

	//cv::line( user->_hog_vis_hist_mask, 
	//				cv::Point( bin_w*(n_bins-1), hist_h - hist_h*user->_hog_hist_mask[n_bins-1]),
	//				cv::Point( bin_w*(0)  , hist_h - hist_h*user->_hog_hist_mask[0]  ),
	//				cv::Scalar( 0, 255, 0), 2, 8, 0  );


	return user->_hog_vis_mask.rows && user->_hog_vis_bbox.rows;
}



}

int main_multi_tracking_features(int argc, char* argv[]){
	NIKinect2Manager* kinect_manager = new NIKinect2Manager();

	NIKinect2::ni_initialize();

	int n_kinects = kinect_manager->initialize_all_kinects();

	std::vector<std::vector<User*>> users_kinect(n_kinects);
	int* users_kinect_ac = (int*)malloc(sizeof(int) * n_kinects);

	for(int k = 0 ; k < n_kinects ; ++k){
		for(int i = 0 ; i < 16 ; ++i){
			users_kinect[k].push_back(new User());
		}
		users_kinect_ac[k] = 0;

		kinect_manager->get_kinect(k)->set_color_hd(true);
	}


	cv::namedWindow("Canny");
	cv::createTrackbar("canny","Canny",&lowThreshold, max_lowThreshold);

	char c = 0;
	while((c = cv::waitKey(11)) != 27){
		if(!kinect_manager->update_all())
			break;

		for(int k = 0 ; k < n_kinects ; ++k){
			cv::Mat color,depth,mask;

			NIKinect2* kinect = kinect_manager->get_kinect(k);
			
			if(kinect){
				kinect->get_depth_8(depth);
				kinect->get_color(color);
				kinect->get_users_map(mask);

						//if(c == 'c'){
							//canny(color);
						//}

				std::vector<nite::UserData*>* users_data = kinect->get_users_data();
				std::vector<int> *users_ids = kinect->get_users_ids();

				if(users_data && users_ids){
					users_kinect_ac[k] = 0;
					for(int u = 0 ; u < users_ids->size() ; ++u){
						nite::UserData data;

						if(kinect->get_user_data(users_ids->at(u),data) && data.getCenterOfMass().x){
							init_user(users_kinect[k][u], kinect, &data, color);

							users_kinect_ac[k]++;

							if(calc_hog(users_kinect[k][u])){
								//char win_buff_hog_bbox[128];
								char win_buff_hog_mask[128];
								//char win_buff_hog_hist_bbox[128];
								//char win_buff_hog_hist_mask[128];
								
								//sprintf(win_buff_hog_bbox,"Hog_bbox(%d)(%d)",k,u);
								sprintf(win_buff_hog_mask,"Hog_mask(%d)(%d)",k,u);
								//sprintf(win_buff_hog_hist_bbox,"Hog_bbox_Hist(%d)(%d)",k,u);
								//sprintf(win_buff_hog_hist_mask,"Hog_mask_Hist(%d)(%d)",k,u);
								
								//cv::imshow(win_buff_hog_bbox,users_kinect[k][u]->_hog_vis_bbox);
								cv::imshow(win_buff_hog_mask,users_kinect[k][u]->_hog_vis_mask);
								//cv::imshow(win_buff_hog_hist_bbox,users_kinect[k][u]->_hog_vis_hist_bbox);
								//cv::imshow(win_buff_hog_hist_mask,users_kinect[k][u]->_hog_vis_hist_mask);
							}

							cv::circle(color,users_kinect[k][u]->_centroid_2d,7,cv::Scalar(0,0,255),-1);
							cv::rectangle(color,users_kinect[k][u]->_bbox,cv::Scalar(0,255,0),3);
						}
					}

					if(users_ids->size() == 2){

					}
				}

				char win_color[128];
				sprintf(win_color,"Color(%d)",k);
				cv::imshow(win_color,color);
			}
		}

		std::vector<User*> temp_users;
		for(int i = 0 ; i < users_kinect_ac[0] ; ++i){
			temp_users.push_back(users_kinect[0][i]);
		}
		for(int i = 0 ; i < users_kinect_ac[1] ; ++i){
			temp_users.push_back(users_kinect[1][i]);
		}

		if(temp_users.size() == 2){
			std::vector<float> diff(9);

			for(int i = 0 ; i < 9 ; ++i){
				diff[i] = abs(temp_users[0]->_hog_hist_mask[i] - temp_users[1]->_hog_hist_mask[i]);
			}
			
			int n_bins = 9;
			int hist_w = 270; int hist_h = 200;
			int bin_w = cvRound( (double) hist_w/n_bins );
	
			cv::Mat diff_mat = cv::Mat( hist_h, hist_w, CV_8UC3, cv::Scalar( 0,0,0) );
			
			for( int i = 1; i < n_bins; i++ )	{
				cv::line(	diff_mat, 
							cv::Point( bin_w*(i-1), hist_h - hist_h*diff[i-1]),
							cv::Point( bin_w*(i)  , hist_h - hist_h*diff[i]  ),
							cv::Scalar( 0, 0, 255), 2, 8, 0  );
				cv::line(	diff_mat,
							cv::Point( bin_w*(i-1), hist_h - hist_h*temp_users[0]->_hog_hist_mask[i-1]),
							cv::Point( bin_w*(i)  , hist_h - hist_h*temp_users[0]->_hog_hist_mask[i]  ),
							cv::Scalar( 255, 0, 0), 2, 8, 0  );
				cv::line(	diff_mat, 
							cv::Point( bin_w*(i-1), hist_h - hist_h*temp_users[1]->_hog_hist_mask[i-1]),
							cv::Point( bin_w*(i)  , hist_h - hist_h*temp_users[1]->_hog_hist_mask[i]  ),
							cv::Scalar( 0, 255, 0), 2, 8, 0  );
			}

			//cv::line(	diff_mat, 
			//			cv::Point( bin_w*(n_bins-1), hist_h - hist_h*diff[n_bins-1]),
			//			cv::Point( bin_w*(0)  , hist_h - hist_h*diff[0]  ),
			//			cv::Scalar( 0, 0, 255), 2, 8, 0  );
			//cv::line(	diff_mat,
			//			cv::Point( bin_w*(n_bins-1), hist_h - hist_h*temp_users[0]->_hog_hist_mask[n_bins-1]),
			//			cv::Point( bin_w*(0)  , hist_h - hist_h*temp_users[0]->_hog_hist_mask[0]  ),
			//			cv::Scalar( 255, 0, 0), 2, 8, 0  );
			//cv::line(	diff_mat, 
			//			cv::Point( bin_w*(n_bins-1), hist_h - hist_h*temp_users[1]->_hog_hist_mask[n_bins-1]),
			//			cv::Point( bin_w*(0)  , hist_h - hist_h*temp_users[1]->_hog_hist_mask[0]  ),
			//			cv::Scalar( 0, 255, 0), 2, 8, 0  );

			int compare_method = 0;
			double compare_value = cv::compareHist(temp_users[0]->_hog_hist_mask,temp_users[1]->_hog_hist_mask,compare_method);

			char buff[128];
			sprintf(buff,"%.4f",compare_value);
			cv::putText(diff_mat, buff, cv::Point(150,30), cv::FONT_HERSHEY_DUPLEX, .75, cv::Scalar(255,255,255), 0.75);

			cv::imshow("Diff",diff_mat);
		}

	} //while

	kinect_manager->~NIKinect2Manager();

	return 0;
}

/*
cv::Mat get_hogdescriptor_visu(cv::Mat& origImg, cv::vector<float>& descriptorValues){   
    cv::Mat color_origImg;
    cvtColor(origImg, color_origImg, CV_GRAY2RGB);
 
    float zoomFac = 3;
    cv::Mat visu;
    resize(color_origImg, visu, cv::Size(color_origImg.cols*zoomFac, color_origImg.rows*zoomFac));
 
    int blockSize       = 16;
    int cellSize        = 8;
    int gradientBinSize = 9;
    float radRangeForOneBin = M_PI/(float)gradientBinSize; // dividing 180° into 9 bins, how large (in rad) is one bin?
 
    // prepare data structure: 9 orientation / gradient strenghts for each cell
    int cells_in_x_dir = 64 / cellSize;
    int cells_in_y_dir = 128 / cellSize;
    int totalnrofcells = cells_in_x_dir * cells_in_y_dir;
    float*** gradientStrengths = new float**[cells_in_y_dir];
    int** cellUpdateCounter   = new int*[cells_in_y_dir];
    for (int y=0; y<cells_in_y_dir; y++)
    {
        gradientStrengths[y] = new float*[cells_in_x_dir];
        cellUpdateCounter[y] = new int[cells_in_x_dir];
        for (int x=0; x<cells_in_x_dir; x++)
        {
            gradientStrengths[y][x] = new float[gradientBinSize];
            cellUpdateCounter[y][x] = 0;
 
            for (int bin=0; bin<gradientBinSize; bin++)
                gradientStrengths[y][x][bin] = 0.0;
        }
    }
 
    // nr of blocks = nr of cells - 1
    // since there is a new block on each cell (overlapping blocks!) but the last one
    int blocks_in_x_dir = cells_in_x_dir - 1;
    int blocks_in_y_dir = cells_in_y_dir - 1;
 
    // compute gradient strengths per cell
    int descriptorDataIdx = 0;
    int cellx = 0;
    int celly = 0;
 
    for (int blockx=0; blockx<blocks_in_x_dir; blockx++)
    {
        for (int blocky=0; blocky<blocks_in_y_dir; blocky++)            
        {
            // 4 cells per block ...
            for (int cellNr=0; cellNr<4; cellNr++)
            {
                // compute corresponding cell nr
                int cellx = blockx;
                int celly = blocky;
                if (cellNr==1) celly++;
                if (cellNr==2) cellx++;
                if (cellNr==3)
                {
                    cellx++;
                    celly++;
                }
 
                for (int bin=0; bin<gradientBinSize; bin++)
                {
                    float gradientStrength = descriptorValues[ descriptorDataIdx ];
                    descriptorDataIdx++;
 
                    gradientStrengths[celly][cellx][bin] += gradientStrength;
 
                } // for (all bins)
 
 
                // note: overlapping blocks lead to multiple updates of this sum!
                // we therefore keep track how often a cell was updated,
                // to compute average gradient strengths
                cellUpdateCounter[celly][cellx]++;
 
            } // for (all cells)
 
 
        } // for (all block x pos)
    } // for (all block y pos)
 
 
    // compute average gradient strengths
    for (int celly=0; celly<cells_in_y_dir; celly++)
    {
        for (int cellx=0; cellx<cells_in_x_dir; cellx++)
        {
 
            float NrUpdatesForThisCell = (float)cellUpdateCounter[celly][cellx];
 
            // compute average gradient strenghts for each gradient bin direction
            for (int bin=0; bin<gradientBinSize; bin++)
            {
                gradientStrengths[celly][cellx][bin] /= NrUpdatesForThisCell;
            }
        }
    }
 
 
    //cout << "descriptorDataIdx = " << descriptorDataIdx << endl;
 
    // draw cells
    for (int celly=0; celly<cells_in_y_dir; celly++)
    {
        for (int cellx=0; cellx<cells_in_x_dir; cellx++)
        {
            int drawX = cellx * cellSize;
            int drawY = celly * cellSize;
 
            int mx = drawX + cellSize/2;
            int my = drawY + cellSize/2;
 
            rectangle(visu, cv::Point(drawX*zoomFac,drawY*zoomFac), cv::Point((drawX+cellSize)*zoomFac,(drawY+cellSize)*zoomFac), CV_RGB(100,100,100), 1);
 
            // draw in each cell all 9 gradient strengths
            for (int bin=0; bin<gradientBinSize; bin++)
            {
                float currentGradStrength = gradientStrengths[celly][cellx][bin];
 
                // no line to draw?
                if (currentGradStrength==0)
                    continue;
 
                float currRad = bin * radRangeForOneBin + radRangeForOneBin/2;
 
                float dirVecX = cos( currRad );
                float dirVecY = sin( currRad );
                float maxVecLen = cellSize/2;
                float scale = 2.5; // just a visualization scale, to see the lines better
 
                // compute line coordinates
                float x1 = mx - dirVecX * currentGradStrength * maxVecLen * scale;
                float y1 = my - dirVecY * currentGradStrength * maxVecLen * scale;
                float x2 = mx + dirVecX * currentGradStrength * maxVecLen * scale;
                float y2 = my + dirVecY * currentGradStrength * maxVecLen * scale;
 
                // draw gradient visualization
                line(visu, cv::Point(x1*zoomFac,y1*zoomFac), cv::Point(x2*zoomFac,y2*zoomFac), CV_RGB(0,255,0), 1);
 
            } // for (all bins)
 
        } // for (cellx)
    } // for (celly)
 
 
    // don't forget to free memory allocated by helper data structures!
    for (int y=0; y<cells_in_y_dir; y++)
    {
      for (int x=0; x<cells_in_x_dir; x++)
      {
           delete[] gradientStrengths[y][x];            
      }
      delete[] gradientStrengths[y];
      delete[] cellUpdateCounter[y];
    }
    delete[] gradientStrengths;
    delete[] cellUpdateCounter;
 
    return visu;
 
} // get_hogdescriptor_visu

double fitt_image(cv::Mat& orig, cv::Mat& out, int out_width = 640, int out_height = 480, cv::Rect* roi_out = 0){
		//Test Input
		if(orig.cols <= 0 || orig.rows <= 0) return -1;

		//Create Output
		out = cv::Mat::zeros(cv::Size(out_width,out_height),orig.type());

		//Calc ROI and Resize Image
		if(!roi_out) roi_out = new cv::Rect();
		cv::Mat temp;
		int w,h;
		double output_ratio = (double)out.cols / (double)out.rows;
		double input_ratio = (double)orig.cols / (double)orig.rows;

		double ratio = 1.0;

		if(input_ratio < output_ratio){
			ratio = (float)out.rows/(float)orig.rows;
			w = orig.cols *ratio;
			h = orig.rows *ratio;

			cv::resize(orig,temp,cv::Size(w,h),0.0,0.0,cv::INTER_CUBIC);
		
			roi_out->x = (out.cols - w) / 2.0;
			roi_out->y = 0;
			roi_out->width = w;
			roi_out->height = h;
		}
		else{
			ratio = (float)out.cols/(float)orig.cols;
			w = orig.cols *ratio;
			h = orig.rows *ratio;

			cv::resize(orig,temp,cv::Size(w,h),0.0,0.0,cv::INTER_CUBIC);
		
			roi_out->x = 0;
			roi_out->y = (out.rows - h) / 2.0;
			roi_out->width = w;
			roi_out->height = h;
		}

		//Copy to Destination
		temp.copyTo(out(*roi_out));

		return ratio;
	}

bool calc_hog(User* user){
	if(!user) return false;

	//cv::cvtColor(user->_color_user,user->_gray_user_bbox,CV_RGB2GRAY);
	//user->_gray_user_bbox.copyTo(user->_gray_user_mask,user->_mask_user);

	cv::Mat gray_bbox;
	cv::Mat gray_mask;
	
	cv::cvtColor(user->_color_user,gray_bbox,CV_RGB2GRAY);
	gray_bbox.copyTo(gray_mask,user->_mask_user);

	fitt_image(gray_bbox,user->_gray_user_bbox,128,256);
	fitt_image(gray_mask,user->_gray_user_mask,128,256);

	//cv::Size s_bbox((gray_bbox.cols % 2) ? gray_bbox.cols-1 : gray_bbox.cols,
	//				(gray_bbox.rows % 2) ? gray_bbox.rows-1 : gray_bbox.rows);
	//cv::Size s_mask((gray_mask.cols % 2) ? gray_mask.cols-1 : gray_mask.cols,
	//				(gray_mask.rows % 2) ? gray_mask.rows-1 : gray_mask.rows);
	//cv::Size s_bbox(gray_bbox.cols,gray_bbox.rows);
	//cv::Size s_mask(gray_mask.cols,gray_mask.rows);

	//while(s_bbox.width % 8)
	//	s_bbox.width--;
	//while(s_bbox.height % 8)
	//	s_bbox.height--;
	//while(s_mask.width % 8)
	//	s_mask.width--;
	//while(s_mask.height % 8)
	//	s_mask.height--;


	//cv::resize(gray_bbox,user->_gray_user_bbox,s_bbox);
	//cv::resize(gray_mask,user->_gray_user_mask,s_mask);
	
	//cv::Size cellSize(user->_gray_user_bbox.cols / 16,user->_gray_user_bbox.rows / 16);
	//cv::Size winSize(user->_gray_user_bbox.cols,user->_gray_user_bbox.rows);		//(128,64)
	//cv::Size blockSize(user->_gray_user_bbox.cols / 8,user->_gray_user_bbox.rows / 8);
	//cv::Size blockStride((winSize.width - blockSize.width)/16,(winSize.height - blockSize.height)/16);
	//

	//int q00 = blockSize.width % cellSize.width;
	//int q01 = blockSize.height % cellSize.height;

	//int q1 = winSize.width - blockSize.width;
	//int q2 = q1 % blockStride.width;
	//int q3 = winSize.height - blockSize.height;
	//int q4 = q3 % blockStride.height;

	cv::HOGDescriptor		hog_bbox;//(//;//(winSize, blockSize, blockStride, cellSize, 9);
									//cv::Size(128,64), //winSize cv::Size(128,64)
									//cv::Size(16,16), //blocksize cv::Size(16,16)
									//cv::Size(8,8), //blockStride, cv::Size(8,8)
									//cv::Size(8,8), //cellSize, cv::Size(8,8)
									//9); //nbins,
	cv::vector<float>		ders_bbox;
	cv::vector<cv::Point>	locs_bbox;

	hog_bbox.compute(user->_gray_user_bbox,ders_bbox,cv::Size(32,32),cv::Size(0,0),locs_bbox);

	if(ders_bbox.size())
		user->_hog_vis_bbox = get_hogdescriptor_visu(user->_gray_user_bbox,ders_bbox);
	

	cv::HOGDescriptor		hog_mask;//(//;//(winSize, blockSize, blockStride, cellSize, 9);
									//cv::Size(128,64), //winSize cv::Size(128,64)
									//cv::Size(16,16), //blocksize cv::Size(16,16)
									//cv::Size(8,8), //blockStride, cv::Size(8,8)
									//cv::Size(8,8), //cellSize, cv::Size(8,8)
									//9); //nbins,
	cv::vector<float>		ders_mask;
	cv::vector<cv::Point>	locs_mask;

	hog_mask.compute(user->_gray_user_mask,ders_mask,cv::Size(32,32),cv::Size(0,0),locs_mask);
		
	user->_hog_vis_mask= get_hogdescriptor_visu(user->_gray_user_mask,ders_mask);
	

	//Calc Histograms
	for(int i = 0 ; i < ders_mask.size() ; i+=9){
		for(int j = 0 ; j < 9 ; ++j){
			user->_hog_hist_bbox[j] = 0;
			user->_hog_hist_mask[j] = 0;
		}
	}
	for(int i = 0 ; i < ders_mask.size() ; i+=9){
		for(int j = 0 ; j < 9; ++j){
			user->_hog_hist_bbox[j] += ders_bbox[i + j];
			user->_hog_hist_mask[j] += ders_mask[i + j];
		}
	}
	
	int n_bins = 9;
	int hist_w = 270; int hist_h = 200;
	int bin_w = cvRound( (double) hist_w/n_bins );
	

	user->_hog_vis_hist_bbox = cv::Mat( hist_h, hist_w, CV_8UC3, cv::Scalar( 0,0,0) );
	user->_hog_vis_hist_mask = cv::Mat( hist_h, hist_w, CV_8UC3, cv::Scalar( 0,0,0) );

	cv::normalize(user->_hog_hist_bbox,user->_hog_hist_bbox);
	cv::normalize(user->_hog_hist_mask,user->_hog_hist_mask);

	for( int i = 1; i < n_bins; i++ )	{
		cv::line( user->_hog_vis_hist_bbox, 
						cv::Point( bin_w*(i-1), hist_h - hist_h*user->_hog_hist_bbox[i-1]),
						cv::Point( bin_w*(i)  , hist_h - hist_h*user->_hog_hist_bbox[i]  ),
						cv::Scalar( 255, 0, 0), 2, 8, 0  );

		cv::line( user->_hog_vis_hist_mask, 
						cv::Point( bin_w*(i-1), hist_h - hist_h*user->_hog_hist_mask[i-1]),
						cv::Point( bin_w*(i)  , hist_h - hist_h*user->_hog_hist_mask[i]  ),
						cv::Scalar( 0, 255, 0), 2, 8, 0  );
		//cv::line( user->_hsv_hist_v, cv::Point( bin_w*(i-1), hist_h - cvRound(user->_histogram_rgb_r.at<float>(i-1)) ) ,
		//				cv::Point( bin_w*(i), hist_h - cvRound(user->_histogram_rgb_r.at<float>(i)) ),
		//				cv::Scalar( 0, 0, 255), 2, 8, 0  );
	}

	return user->_hog_vis_mask.rows && user->_hog_vis_bbox.rows;
}

*/