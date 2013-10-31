#include "FaceTracker.h"

#include <NIKinect2.h>

#include <direct.h> //mkdir()

cv::CascadeClassifier face_cascade_lbp;
cv::CascadeClassifier face_cascade_haar;
cv::CascadeClassifier right_eye_cascade_haar;
cv::CascadeClassifier left_eye_cascade_haar;
cv::CascadeClassifier mouth_cascade_haar;
cv::CascadeClassifier nose_cascade_haar;

NIKinect2* kinect;

class User{
public:
	User(){};
	~User(){};

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

bool face_detection(cv::Mat &color, User* user){
	if(user->bbox.width > 30){
		cv::Mat image_user_temp;

		color(cv::Rect(user->bbox.x,
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

			color(user->face_rect).copyTo(user->face_mat);

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

				color(user->face_rect).copyTo(user->face_mat);

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


bool train_user = false;
bool user_detection = false;
int training_idx = 0;
std::vector<std::vector<cv::Mat>> training_faces;
std::vector<int> training_tags;

int img_size = 100;

cv::Ptr<cv::FaceRecognizer> model = cv::createLBPHFaceRecognizer();

bool train_add_image(User *user){
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
			}			
		}
		else
			training_faces[training_idx].push_back(cv::Mat(face));

		return true;
	}
	else
		return false;
}

bool save_images(){
	if(training_idx && training_faces.size() && training_faces.size() == training_tags.size()){
		mkdir(DEFAUL_PATH);
		FILE* file = fopen("FaceRec\\training_data.txt","w+");

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
		cv::Mat img_resize,img_norm;

		//cv::Mat gray;
		//cv::cvtColor(img,gray,CV_BGR2GRAY);
		cv::resize(img,img_resize,cv::Size(img_size,img_size));

		cv::equalizeHist(img_resize,img_norm);

		//cv::normalize(img_resize,img_norm, 0, 255, cv::NORM_MINMAX, CV_8UC1);
		//cv::imshow("Gray",img);
		//cv::imshow("Norm",img_norm);
		//cv::waitKey();

		faces.push_back(img_norm);
		labels.push_back(id);
	}

	model->train(faces, labels);
	//model->load("db");
	return true;
}

int main_face_tracker(int argc, char* argv[]){
	NIKinect2::ni_initialize();

	kinect = new NIKinect2();

	kinect->initialize();

	kinect->enable_depth_generator();
	kinect->enable_color_generator();
	kinect->enable_user_generator();

	kinect->set_depth_color_registration(true);

	kinect->set_color_hd(true);

	cv::Mat image_depth, image_color;
	cv::Mat image_user,image_user_mask;
	cv::Mat mask_user;

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

	if(!right_eye_cascade_haar.load("haarcascades\\haarcascade_mcs_righteye.xml") || right_eye_cascade_haar.empty()){
		printf("haarcascade_mcs_righteye not found\n");
		return -1;
	}
	if(!left_eye_cascade_haar.load("haarcascades\\haarcascade_mcs_lefteye.xml") || left_eye_cascade_haar.empty()){
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

	int name_ac[10] = {0};

	std::vector<int> *users_id = NULL;
	bool user = false;
	char c = 0;
	while((c = cv::waitKey(20)) != 27){
		if(!NIKinect2::ni_update() || !kinect->update())
			break;

		//CV::MAT UPDATE
		kinect->get_depth_8(image_depth);
		kinect->get_color(image_color);
		user = false;
		//kinect->get_users_map(mask_user);

		if((users_id = kinect->get_users_ids()) != NULL){
			for(int i = 0 ; i < users_id->size() ; ++i){
				nite::UserData data;
			
				if(kinect->get_user_data(users_id->at(i),data)){

					User user;

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

					if(face_detection(image_color, &user)){

						//if(face_features_detection(&user)){
						//	if(user.mouth_flag){ //Red
						//		cv::rectangle(user.face_mat,user.mouth_rect,cv::Scalar(0,0,255),1);
						//	}
						//	if(user.nose_flag){ //Yellow
						//		cv::rectangle(user.face_mat,user.nose_rect,cv::Scalar(0,255,255),1);
						//	}
						//	if(user.left_eye_flag){ //Blue
						//		cv::rectangle(user.face_mat,user.left_eye_rect,cv::Scalar(255,0,0),1);
						//	}
						//	if(user.right_eye_flag){ //Green
						//		cv::rectangle(user.face_mat,user.right_eye_rect,cv::Scalar(0,255,0),1);
						//	}
						//}

						if(train_user){
							train_add_image(&user);

							if(training_faces[training_idx].size() == 75){
								printf("Fifty shades of someone\n");
								c = 'r';
							}
						}

						if(user_detection){
							cv::Mat face_resized;
							cv::Mat gray, normalized;
							cvtColor(user.face_mat, gray, CV_BGR2GRAY);
							//cv::normalize(gray,normalized, 0, 255, cv::NORM_MINMAX, CV_8UC1);
							cv::equalizeHist(gray,normalized);
							cv::resize(normalized, face_resized, cv::Size(img_size, img_size), 1.0, 1.0, cv::INTER_CUBIC);
							// Now perform the prediction, see how easy that is:
							int prediction;
							double conf = 0.0;

							


							model->predict(face_resized,prediction,conf);

							//printf("Prediction: %d\n",prediction);
							char box_text[1024]; 
							if(prediction >= 0){
								name_ac[prediction]++;
								sprintf(box_text,"%d(%s), Conf: %.2f",	prediction,
																		/*(prediction == 0) ? "Creepy" : 
																		(prediction == 1) ? "Normal" :  
																		(prediction == 2) ? "Sad" : "UNKNOWN",conf);*/
																		(prediction == 0) ? "Gustavo" : 
																		(prediction == 1) ? "Mario" :  
																		(prediction == 2) ? "Rui" :  
																		(prediction == 3) ? "Nuno" : 
																		(prediction == 4) ? "Abel" : "UNKNOWN",conf);
								// Calculate the position for annotated text (make sure we don't
								// put illegal values in there):
							}
							int pos_x = std::max(user.face_rect.tl().x - 20, 0);
							int pos_y = std::max(user.face_rect.tl().y - 20, 0);
							// And now put it into the image:
							cv::rectangle(image_color,cv::Rect(pos_x,pos_y-30,350,50),cv::Scalar(255,255,255),-1);
							cv::putText(image_color, box_text, cv::Point(pos_x, pos_y), cv::FONT_HERSHEY_DUPLEX, 1, CV_RGB(0,0,0), 1.0);


							char name_0[128]; 
							sprintf(name_0,"0 - %d",name_ac[0]);
							char name_1[128]; 
							sprintf(name_1,"1 - %d",name_ac[1]);
							char name_2[128]; 
							sprintf(name_2,"2 - %d",name_ac[2]);
							char name_3[128]; 
							sprintf(name_3,"3 - %d",name_ac[3]);
							char name_4[128]; 
							sprintf(name_4,"4 - %d",name_ac[4]);

							cv::rectangle(image_color,cv::Rect(0,0,100,100),cv::Scalar(255,255,255),-1);

							cv::putText(image_color, name_0, cv::Point(10, 10), cv::FONT_HERSHEY_DUPLEX, 0.5, CV_RGB(0,0,0), 1.0);
							cv::putText(image_color, name_1, cv::Point(10, 25), cv::FONT_HERSHEY_DUPLEX, 0.5, CV_RGB(0,0,0), 1.0);
							cv::putText(image_color, name_2, cv::Point(10, 40), cv::FONT_HERSHEY_DUPLEX, 0.5, CV_RGB(0,0,0), 1.0);
							cv::putText(image_color, name_3, cv::Point(10, 55), cv::FONT_HERSHEY_DUPLEX, 0.5, CV_RGB(0,0,0), 1.0);
							cv::putText(image_color, name_4, cv::Point(10, 70), cv::FONT_HERSHEY_DUPLEX, 0.5, CV_RGB(0,0,0), 1.0);
						}

						char win_users[128];
						sprintf(win_users,"Users(%d) ID(%d)",i,data.getId());
						cv::rectangle(image_color,user.face_rect,cv::Scalar(255,0,0),1);
						cv::imshow(win_users,user.face_mat);
					}
				}
			}
		}

		cv::imshow("Depth",image_depth);
		cv::imshow("Color",image_color);
		//if(user){
		//	cv::imshow("User",image_user);
		//}

		if(c == 'r'){
			if(train_user){
				if(training_faces[training_idx].size()){
					training_tags.push_back(training_idx);
					training_idx++;
				}

				train_user = false;
			}
			else{
				printf("Start Recording Images\n");
				training_faces.push_back(*(new std::vector<cv::Mat>()));
				train_user = true;
			}
		}

		if(c == 't'){
			printf("Start trainning\n");

			printf("%s\n", (train_face_rec_db()) ? "Trainning Successful" : "Trainning Failed");
		}

		if(c == 's'){
			printf("%s\n", (save_images()) ? "Images Recorded" : "Recording Failed");
		}

		if(c == 'd'){
			user_detection = !user_detection;
			printf("%s\n", (user_detection) ? "Recognizing Users" : "Stop Recognizing");
		}

		if(c == 'l'){
			load_face_db();
			printf("Load DB");
		}

		if(c == 'z'){
			name_ac[0] = 0;
			name_ac[1] = 0;
			name_ac[2] = 0;
			name_ac[3] = 0;
			name_ac[4] = 0;
			name_ac[5] = 0;
			name_ac[6] = 0;
			name_ac[7] = 0;
			name_ac[8] = 0;
			name_ac[9] = 0;
		}
	}

	return 0;
}