#include "FaceTemplateMatch.h"

#include <NIKinect2.h>

#include <direct.h> //mkdir()

namespace{
NIKinect2* kinect;

cv::CascadeClassifier face_cascade_lbp;
cv::CascadeClassifier face_cascade_haar;
cv::CascadeClassifier right_eye_cascade_haar;
cv::CascadeClassifier left_eye_cascade_haar;
cv::CascadeClassifier mouth_cascade_haar;
cv::CascadeClassifier nose_cascade_haar;


class User{
public:
	User(){};
	~User(){};

	cv::Mat user_masked;

	cv::Rect bbox;
	cv::Point com;

	cv::Mat face_mat;
	cv::Rect face_rect;

	bool left_eye_flag;
	cv::Rect left_eye_rect;
	bool right_eye_flag;
	cv::Rect right_eye_rect;

	bool mouth_flag;
	cv::Rect mouth_rect;
	bool nose_flag;
	cv::Rect nose_rect;
};

bool train_user = false;
bool user_detection = false;
int training_idx = 0;
std::vector<std::vector<cv::Mat>> training_faces;
std::vector<std::vector<cv::Mat>> training_faces_orig;
std::vector<std::vector<cv::Mat>> training_faces_mask;
std::vector<std::vector<cv::Rect>> training_faces_rect;

std::vector<int> training_tags;

int img_size = 200;


bool face_detection(cv::Mat &color, User* user){
	if(user->bbox.width > 30){
		cv::Mat image_user_temp;

		user->user_masked(cv::Rect(user->bbox.x,
					   user->bbox.y,
					   user->bbox.width, 
					   user->bbox.height)).copyTo(image_user_temp);

		cv::Mat gray;
		cv::cvtColor(image_user_temp, gray, CV_BGR2GRAY);

		std::vector<cv::Rect> faces_haar;
		face_cascade_haar.detectMultiScale(gray, faces_haar, 1.1, 2, 0|CV_HAAR_FIND_BIGGEST_OBJECT, cv::Size(30,30));
		if(faces_haar.size()){// || faces_lbp.size()){
			user->face_rect = faces_haar[faces_haar.size()-1];
			user->face_rect.x += user->bbox.x;
			user->face_rect.y += user->bbox.y;

			user->user_masked(user->face_rect).copyTo(user->face_mat);

			return true;
		}
		else{
			printf("FACES LBP\n");
			std::vector<cv::Rect> faces_lbp;
			face_cascade_lbp.detectMultiScale(gray, faces_lbp, 1.1, 2, 0|CV_HAAR_FIND_BIGGEST_OBJECT, cv::Size(30,30));

			if(faces_lbp.size()){// || faces_lbp.size()){
				user->face_rect = faces_lbp[faces_lbp.size()-1];
				user->face_rect.x += user->bbox.x;
				user->face_rect.y += user->bbox.y;

				user->user_masked(user->face_rect).copyTo(user->face_mat);

				return true;
			}
		}
	}
	return false;
}

bool face_features_detection(User* user){
	if(user && user->face_rect.width > 0){
		cv::Mat gray;
		cv::cvtColor(user->face_mat, gray, CV_BGR2GRAY);

		std::vector<cv::Rect> mouth_rect;
		mouth_cascade_haar.detectMultiScale(gray, mouth_rect, 1.1, 2, 0|CV_HAAR_FIND_BIGGEST_OBJECT,cv::Size(10,10));
		if(mouth_rect.size()){
			user->mouth_rect = mouth_rect[0];
			user->mouth_flag = true;
		}
		else
			user->mouth_flag = false;

		std::vector<cv::Rect> nose_rect;
		nose_cascade_haar.detectMultiScale(gray, nose_rect, 1.1, 2, 0|CV_HAAR_FIND_BIGGEST_OBJECT);
		if(nose_rect.size()){
			user->nose_rect = nose_rect[0];
			user->nose_flag = true;
		}
		else
			user->nose_flag = false;

		std::vector<cv::Rect> left_eye_rect;
		left_eye_cascade_haar.detectMultiScale(gray, left_eye_rect, 1.1, 2, 0|CV_HAAR_FIND_BIGGEST_OBJECT,cv::Size(5,5));
		if(left_eye_rect.size()){
			user->left_eye_rect = left_eye_rect[0];
			user->left_eye_flag = true;
		}
		else
			user->left_eye_flag = false;

		std::vector<cv::Rect> right_eye_rect;
		right_eye_cascade_haar.detectMultiScale(gray, right_eye_rect, 1.1, 2, 0|CV_HAAR_FIND_BIGGEST_OBJECT,cv::Size(5,5));
		if(right_eye_rect.size()){
			user->right_eye_rect = right_eye_rect[0];
			user->right_eye_flag = true;
		}
		else
			user->right_eye_flag = false;

		if(user->mouth_flag || user->nose_flag || user->left_eye_flag || user->right_eye_flag)
			return true;
		else
			return false;
	}
	else
		return false;
}

#define DEFAUL_PATH "FaceRec"
#define DEFAUL_FILE_PATH "FaceRec/training_data.txt"




cv::Ptr<cv::FaceRecognizer> model = cv::createFisherFaceRecognizer();

bool train_add_image(User *user, cv::Mat &color, cv::Mat &mask){
	if(user && user->face_mat.rows > 0){
		cv::Mat face,face_temp,face_temp2;
		cv::resize(user->face_mat,face_temp,cv::Size(img_size,img_size));
		cv::cvtColor(face_temp,face,CV_RGB2GRAY,1);

		if(training_faces[training_idx].size() > 1){
			cv::Mat temp;
			cv::absdiff(face,training_faces[training_idx][training_faces[training_idx].size()-1],temp);

			temp-= 25;
			double diff = ((float)cv::countNonZero(temp))/((float)(img_size*img_size));

			cv::waitKey(100); //Slow down the capture

			if(diff > 0.20){
				printf("New Image (%d)\n",training_faces[training_idx].size());
				training_faces[training_idx].push_back(cv::Mat(face));
				training_faces_orig[training_idx].push_back(cv::Mat(color));
				training_faces_mask[training_idx].push_back(cv::Mat(mask));
				training_faces_rect[training_idx].push_back(cv::Rect(user->face_rect));

				//cv::imshow("coiso",training_faces_orig[training_idx][training_faces_orig[training_idx].size()-1]);
				//cv::waitKey();
			}			
		}
		else
			training_faces[training_idx].push_back(cv::Mat(face));
			training_faces_orig[training_idx].push_back(cv::Mat(color));
			training_faces_mask[training_idx].push_back(cv::Mat(mask));
			training_faces_rect[training_idx].push_back(cv::Rect(user->face_rect));

		return true;
	}
	else
		return false;
}

bool online_save_images(){
	if(training_idx >= 0 && training_faces.size() && training_faces.size() == training_tags.size()){
		mkdir(DEFAUL_PATH);
		FILE* file = fopen("FaceRec\\training_data.txt","a+");

		if(file == NULL) return false;

		//for(int i = 0 ; i < training_idx ; ++i){
			char path[128];
			sprintf(path,"%s\\%d\\",DEFAUL_PATH,training_tags[training_idx]);
			_mkdir((const char*)path);

			for(int j = 0 ; j < training_faces[training_idx].size() ; ++j){
				char image_path[256];
				sprintf(image_path,"%s%d.png",path,j);

				cv::imwrite(image_path,training_faces[training_idx][j]);
				
				fprintf(file,"%s ; %d\n",image_path,training_idx);
			}
		//}

		fclose(file);


		FILE* file2 = fopen("FaceRec\\training_data_full.txt","a+");

		if(file2 == NULL) return false;

		//for(int i = 0 ; i < training_idx ; ++i){
			//char path[128];
			//sprintf(path,"%s\\%d\\",DEFAUL_PATH,training_tags[training_idx]);
			//_mkdir((const char*)path);

			for(int j = 0 ; j < training_faces[training_idx].size() ; ++j){
				char face_path[256];
				char image_path[256];
				char depth_path[256];
				char rect[256];
				sprintf(face_path,"%sface_%d.png",path,j);
				sprintf(image_path,"%sorig_%d.png",path,j);
				sprintf(depth_path,"%smask_%d.png",path,j);
				sprintf(rect,"%d , %d , %d , %d",	training_faces_rect[training_idx][j].x,
													training_faces_rect[training_idx][j].y,
													training_faces_rect[training_idx][j].width,
													training_faces_rect[training_idx][j].height);

				cv::imwrite(face_path,training_faces[training_idx][j]);
				cv::imwrite(image_path,training_faces_orig[training_idx][j]);
				cv::imwrite(depth_path,training_faces_mask[training_idx][j]);
				//training_faces_orig[i][j]
				fprintf(file2,"%d ; %s ; %s ; %s ; %s\n",training_idx, face_path, image_path, depth_path, rect);
			}
		//}

		fclose(file2);

		return true;
	}
	else
		return false;
}

bool save_images(){
	if(training_idx && training_faces.size() && training_faces.size() == training_tags.size()){
		mkdir(DEFAUL_PATH);
		FILE* file = fopen("FaceRec\\training_data.txt","a+");

		if(file == NULL) return false;

		for(int i = 0 ; i < training_idx ; ++i){
			char path[128];
			sprintf(path,"%s\\%d\\",DEFAUL_PATH,training_tags[i]);
			_mkdir((const char*)path);

			for(int j = 0 ; j < training_faces[i].size() ; ++j){
				char image_path[256];
				sprintf(image_path,"%s%d.png",path,j);

				cv::imwrite(image_path,training_faces[i][j]);
				
				fprintf(file,"%s ; %d\n",image_path,i);
			}
		}

		fclose(file);


		FILE* file2 = fopen("FaceRec\\training_data_full.txt","a+");

		if(file2 == NULL) return false;

		for(int i = 0 ; i < training_idx ; ++i){
			char path[128];
			sprintf(path,"%s\\%d\\",DEFAUL_PATH,training_tags[i]);
			_mkdir((const char*)path);

			for(int j = 0 ; j < training_faces[i].size() ; ++j){
				char face_path[256];
				char image_path[256];
				char depth_path[256];
				char rect[256];
				sprintf(face_path,"%sface_%d.png",path,j);
				sprintf(image_path,"%sorig_%d.png",path,j);
				sprintf(depth_path,"%smask_%d.png",path,j);
				sprintf(rect,"%d , %d , %d , %d",	training_faces_rect[i][j].x,
													training_faces_rect[i][j].y,
													training_faces_rect[i][j].width,
													training_faces_rect[i][j].height);

				cv::imwrite(face_path,training_faces[i][j]);
				cv::imwrite(image_path,training_faces_orig[i][j]);
				cv::imwrite(depth_path,training_faces_mask[i][j]);
				//training_faces_orig[i][j]
				fprintf(file2,"%d ; %s ; %s ; %s ; %s\n",i, face_path, image_path, depth_path, rect);
			}
		}

		fclose(file2);

		return true;
	}
	else
		return false;
}

bool train_face_rec_db(){
	
	std::vector<cv::Mat> faces;
	std::vector<int> labels;

	for(int i = 0 ; i < training_faces.size() ; ++i){
		for(int j = 0 ; j < training_faces[i].size() ; ++j){
			faces.push_back(training_faces[i][j]);
			labels.push_back(training_tags[i]);
		}
	}

	model->train(faces, labels);
	model->save("db");
	return true;
}

bool init_load(){
	FILE* file = fopen("FaceRec\\training_data.txt","r");

	int max_id = -1;
	char buff[1024];
	while(fgets(buff,1024,file)){
		char path[256];
		int id;

		sscanf(buff,"%s ; %d\n",&path,&id);

		if(id > max_id){
			training_tags.push_back(id);
			training_faces.push_back(*(new std::vector<cv::Mat>()));
			training_faces_mask.push_back(*(new std::vector<cv::Mat>()));
			training_faces_orig.push_back(*(new std::vector<cv::Mat>()));
			training_faces_rect.push_back(*(new std::vector<cv::Rect>()));
			max_id = id;
		}

		cv::Mat img = cv::imread(path,0);
		cv::Mat img_resize,img_norm,img_color;

		//cv::Mat gray;
		
		cv::resize(img,img_resize,cv::Size(img_size,img_size));
		cv::equalizeHist(img_resize,img_norm);

		training_faces[id].push_back(img_norm);
		//labels.push_back(id);


	}

	fclose(file);

	//for(int i = 0 ; i <= max_id ; ++i){
	//	training_tags.push_back(i);
	//	training_faces.push_back(*(new std::vector<cv::Mat>()));
	//	training_faces_mask.push_back(*(new std::vector<cv::Mat>()));
	//	training_faces_orig.push_back(*(new std::vector<cv::Mat>()));
	//	training_faces_rect.push_back(*(new std::vector<cv::Rect>()));
	//}

	training_idx = max_id+1;

	return true;
}

bool load_face_db(){
	
	FILE* file = fopen("FaceRec\\training_data.txt","r");

	std::vector<cv::Mat> faces;
	std::vector<int> labels;

	char buff[1024];
	while(fgets(buff,1024,file)){
		char path[256];
		int id;

		sscanf(buff,"%s ; %d\n",&path,&id);

		cv::Mat img = cv::imread(path,0);
		cv::Mat img_resize,img_norm,img_color;

		//cv::Mat gray;
		
		cv::resize(img,img_resize,cv::Size(img_size,img_size));
		cv::equalizeHist(img_resize,img_norm);
		//
		//cv::cvtColor(img_resize,img_color,CV_GRAY2BGR);
		//std::vector<cv::Rect> mouth_rect;
		//mouth_cascade_haar.detectMultiScale(img_norm, mouth_rect, 1.1, 2, 0|CV_HAAR_FIND_BIGGEST_OBJECT,cv::Size(10,10));
		//if(mouth_rect.size()){//Red
		//	cv::rectangle(img_color,mouth_rect[0],cv::Scalar(0,0,255),1);
		//}
		//
		//std::vector<cv::Rect> nose_rect;
		//nose_cascade_haar.detectMultiScale(img_norm, nose_rect, 1.1, 2, 0|CV_HAAR_FIND_BIGGEST_OBJECT);
		//if(nose_rect.size()){//Yellow
		//	cv::rectangle(img_color,nose_rect[0],cv::Scalar(0,255,255),1);
		//}
		//
		//std::vector<cv::Rect> left_eye_rect;
		//left_eye_cascade_haar.detectMultiScale(img_norm, left_eye_rect, 1.1, 2, 0|CV_HAAR_FIND_BIGGEST_OBJECT,cv::Size(5,5));
		//if(left_eye_rect.size()){ //Blue
		//	cv::rectangle(img_color,left_eye_rect[0],cv::Scalar(255,0,0),1);
		//}

		//std::vector<cv::Rect> right_eye_rect;
		//right_eye_cascade_haar.detectMultiScale(img_norm, right_eye_rect, 1.1, 2, 0|CV_HAAR_FIND_BIGGEST_OBJECT,cv::Size(5,5));
		//if(right_eye_rect.size()){ //Green
		//	cv::rectangle(img_color,right_eye_rect[0],cv::Scalar(0,255,0),1);
		//}

		//cv::normalize(img_resize,img_norm, 0, 255, cv::NORM_MINMAX, CV_8UC1);
		//cv::imshow("Gray",img);
		//cv::imshow("Face",img_color);
		//cv::waitKey();

		faces.push_back(img_norm);
		labels.push_back(id);
	}

	model->train(faces, labels);
	model->save("db");
	//model->load("db");
	return true;
}

}

int main_face_template_match(int argc, char* argv[]){
	NIKinect2::ni_initialize();

	kinect = new NIKinect2();

	kinect->initialize();

	kinect->enable_depth_generator();
	kinect->enable_color_generator();
	kinect->enable_user_generator();

	kinect->set_depth_color_registration(true);

	kinect->set_color_hd(true);

	init_load();

	cv::Mat image_depth, image_color;
	cv::Mat image_user,image_user_mask;
	cv::Mat mask_user, mask_user_resized;

	//cv::CascadeClassifier face_cascade_lbp;
	//cv::CascadeClassifier face_cascade_haar;
	if(!face_cascade_lbp.load("lbpcascades\\lbpcascade_frontalface.xml") || face_cascade_lbp.empty()){
		printf("lbpcascade_frontalface not found\n");
		return -1;
	}
	if(!face_cascade_haar.load("haarcascades\\haarcascade_frontalface_alt2.xml") || face_cascade_haar.empty()){
		printf("haarcascade_frontalface_alt2 not found\n");
		return -1;
	}

	if(!right_eye_cascade_haar.load("haarcascades\\haarcascade_righteye_2splits.xml") || right_eye_cascade_haar.empty()){
		printf("haarcascade_mcs_righteye not found\n");
		return -1;
	}
	if(!left_eye_cascade_haar.load("haarcascades\\haarcascade_lefteye_2splits.xml") || left_eye_cascade_haar.empty()){
		printf("haarcascade_mcs_lefteye not found\n");
		return -1;
	}
	if(!mouth_cascade_haar.load("haarcascades\\haarcascade_mcs_mouth.xml") || mouth_cascade_haar.empty()){
		printf("haarcascade_mcs_mouth not found\n");
		return -1;
	}
	if(!nose_cascade_haar.load("haarcascades\\haarcascade_mcs_nose.xml") || nose_cascade_haar.empty()){
		printf("haarcascade_mcs_nose not found\n");
		return -1;
	}

	int name_ac[10][4];// = {0};

	for(int i = 0 ; i < 10 ; ++i){
		for(int j = 0 ; j < 4 ; ++j){
			name_ac[i][j] = 0;
		}
	}

	std::vector<int> *users_id = NULL;
	bool user = false;
	char c = 0;
	while((c = cv::waitKey(20)) != 27){
		if(!NIKinect2::ni_update() || !kinect->update())
			break;

		//CV::MAT UPDATE
		kinect->get_depth_8(image_depth);
		kinect->get_color(image_color);
		cv::Mat image_color_orig;
		image_color.copyTo(image_color_orig);
		user = false;

		kinect->get_users_map(mask_user);
		cv::resize(mask_user,mask_user_resized,cv::Size(image_color.cols,image_color.rows));
		cv::dilate(mask_user_resized,mask_user_resized,cv::Mat(5,5,CV_8UC1));
		//cv::Mat mask_resized;
		//cv::resize(mask_user,mask_resized,cv::Size(1280,960));

		if((users_id = kinect->get_users_ids()) != NULL){
			for(int i = 0 ; i < users_id->size() ; ++i){
				nite::UserData data;
			
				if(kinect->get_user_data(users_id->at(i),data)){

					User user;

					image_color.copyTo(user.user_masked,mask_user_resized);

					const nite::BoundingBox bbox = data.getBoundingBox();
					const nite::Point3f cm =data.getCenterOfMass();

					float px,py,pz;
					openni::CoordinateConverter::convertWorldToDepth(*kinect->get_depth_stream(),cm.x,cm.y,cm.z,&px,&py,&pz);

					user.bbox.x = bbox.min.x * 2;
					user.bbox.y = bbox.min.y * 2;
					user.bbox.width = bbox.max.x * 2 - bbox.min.x * 2;
					user.bbox.height = bbox.max.y * 2 - bbox.min.y * 2;
					user.com.x = px * 2;
					user.com.y = py * 2;

					//user.bbox.x = bbox.min.x;
					//user.bbox.y = bbox.min.y;
					//user.bbox.width = bbox.max.x - bbox.min.x;
					//user.bbox.height = bbox.max.y - bbox.min.y;
					//user.com.x = px;
					//user.com.y = py;

					if(face_detection(image_color, &user)){
						
						if(user_detection){
							cv::Mat face_resized;
							cv::Mat gray, normalized;
							cvtColor(user.face_mat, gray, CV_BGR2GRAY);
							//cv::normalize(gray,normalized, 0, 255, cv::NORM_MINMAX, CV_8UC1);
							cv::equalizeHist(gray,normalized);
							cv::resize(normalized, face_resized, cv::Size(img_size, img_size), 1.0, 1.0, cv::INTER_CUBIC);

							// Now perform the prediction, see how easy that is:

							cv::Mat range,range_inv;
							cv::inRange(face_resized,0,0,range);
							cv::threshold(range,range_inv,0,1,CV_THRESH_BINARY_INV);
						
							cv::vector<cv::vector<cv::Point> > contours;
							cv::vector<cv::Vec4i> hierarchy;

							
							/// Find contours
							cv::findContours(range_inv, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );
							int max = 0, max_id = 0;


							for( int i = 0; i < contours.size(); i++ ){
								if(contours[i].size() > max){
									max = contours[i].size();
									max_id = i;
								}
							}

							cv::Rect r = boundingRect( cv::Mat(contours[max_id]) );
							r.x += r.width/7;
							r.y += r.height/7;

							r.width -= 2 * r.width/7;
							r.height -= 2 * r.height/7;

							
							cv::rectangle(face_resized,r,cv::Scalar::all(255));
							cv::Mat toShow;
							face_resized(r).copyTo(toShow);
							
							int min_idx_i = 0, min_idx_j = 0, min_value = INT_MAX;

							for(int faces_i = 0 ; faces_i < training_faces.size() ; ++faces_i){
								for(int faces_j = 0 ; faces_j < training_faces[i].size() ; ++faces_j){
									int match_method = CV_TM_SQDIFF;
									cv::Mat result;
									int result_cols =  training_faces[faces_i][faces_j].cols - toShow.cols + 1;
									int result_rows = training_faces[faces_i][faces_j].rows  - toShow.rows + 1;

									result.create( result_cols, result_rows, CV_32FC1 );

									cv::Mat img_copy; toShow.copyTo(img_copy);
									cv::Mat img_copy2; training_faces[faces_i][faces_j].copyTo(img_copy2);
									
									cv::matchTemplate( img_copy, img_copy2, result,0 );
									cv::normalize( result, result, 0, 1, cv::NORM_MINMAX, -1, cv::Mat() );

									/// Localizing the best match with minMaxLoc
									double minVal; double maxVal; cv::Point minLoc; cv::Point maxLoc;
									cv::Point matchLoc;

									cv::minMaxLoc( result, &minVal, &maxVal, &minLoc, &maxLoc, cv::Mat() );

									if( match_method  == CV_TM_SQDIFF || match_method == CV_TM_SQDIFF_NORMED )
									{ matchLoc = minLoc; }
									else
									{ matchLoc = maxLoc; }

									//cv::rectangle( img_copy, matchLoc, cv::Point( matchLoc.x + img_copy.cols , matchLoc.y + img_copy.rows ), cv::Scalar::all(255), 2, 8, 0 );
									//cv::rectangle( img_copy2, matchLoc, cv::Point( matchLoc.x + img_copy.cols , matchLoc.y + img_copy.rows ), cv::Scalar::all(255), 2, 8, 0 );
 
									cv::Mat crop, sub;
									img_copy2(cv::Rect(matchLoc.x, matchLoc.y, img_copy.cols, img_copy.rows)).copyTo(crop);

									cv::absdiff(crop,img_copy,sub);

									cv::Scalar x_sum = cv::sum(sub);

									if(x_sum.val[0] < min_value){
										min_idx_i = faces_i;
										min_idx_j = faces_j;
										min_value = x_sum.val[0];
									}

									//printf("Sum: %.2f, %.2f, %.2f, %.2f\n",x_sum.val[0],x_sum.val[1],x_sum.val[2],x_sum.val[3]);
									//char comp_str[128];
									//sprintf(comp_str,"ID(%d)Face(%d)",faces_i,faces_j);
									//cv::imshow("Detected",img_copy);
									//cv::imshow("DB_Face",crop);
									//cv::imshow("Result",sub);
									//cv::waitKey(50);

								}
							}
							printf("Best Match: [%d][%d] - %d\n",min_idx_i,min_idx_j,min_value);
							cv::imshow("BestMatch",training_faces[min_idx_i][min_idx_j]);
							cv::waitKey();

							cv::imshow("Detected",face_resized);
							cv::imshow("Range",toShow);
						}
					}
				}
			}
		}
		cv::imshow("Depth",image_depth);
		cv::imshow("Color",image_color);

		if(c == 'd'){
			user_detection = !user_detection;
			printf("%s\n", (user_detection) ? "Recognizing Users" : "Stop Recognizing");
		}
	}

	return 0;
}