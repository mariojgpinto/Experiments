#include "CameraCalibration.h"

#include <NIKinect2.h>

#include <OpenNI.h>
#include <opencv2\opencv.hpp>

#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <vector>
#include <string>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>

using namespace cv;
using namespace std;


#define KINECT 0
#define CAMERA 1

cv::Mat color[2];


static void help()
{
    cout <<  "This is a camera calibration sample." << endl
         <<  "Usage: calibration configurationFile"  << endl
         <<  "Near the sample file you'll find the configuration file, which has detailed help of "
             "how to edit it.  It may be any OpenCV supported file format XML/YAML." << endl;
}
class Settings
{
public:
    Settings() : goodInput(false) {}
    enum Pattern { NOT_EXISTING, CHESSBOARD, CIRCLES_GRID, ASYMMETRIC_CIRCLES_GRID };
    enum InputType {INVALID, CAMER4, VIDEO_FILE, IMAGE_LIST};

    void write(FileStorage& fs) const                        //Write serialization for this class
    {
        fs << "{" << "BoardSize_Width"  << boardSize.width
                  << "BoardSize_Height" << boardSize.height
                  << "Square_Size"         << squareSize
                  << "Calibrate_Pattern" << patternToUse
                  << "Calibrate_NrOfFrameToUse" << nrFrames
                  << "Calibrate_FixAspectRatio" << aspectRatio
                  << "Calibrate_AssumeZeroTangentialDistortion" << calibZeroTangentDist
                  << "Calibrate_FixPrincipalPointAtTheCenter" << calibFixPrincipalPoint

                  << "Write_DetectedFeaturePoints" << bwritePoints
                  << "Write_extrinsicParameters"   << bwriteExtrinsics
                  << "Write_outputFileName"  << outputFileName

                  << "Show_UndistortedImage" << showUndistorsed

                  << "Input_FlipAroundHorizontalAxis" << flipVertical
                  << "Input_Delay" << delay
                  << "Input" << input
           << "}";
    }
    void read(const FileNode& node)                          //Read serialization for this class
    {
        node["BoardSize_Width" ] >> boardSize.width;
        node["BoardSize_Height"] >> boardSize.height;
        node["Calibrate_Pattern"] >> patternToUse;
        node["Square_Size"]  >> squareSize;
        node["Calibrate_NrOfFrameToUse"] >> nrFrames;
        node["Calibrate_FixAspectRatio"] >> aspectRatio;
        node["Write_DetectedFeaturePoints"] >> bwritePoints;
        node["Write_extrinsicParameters"] >> bwriteExtrinsics;
        node["Write_outputFileName"] >> outputFileName;
        node["Calibrate_AssumeZeroTangentialDistortion"] >> calibZeroTangentDist;
        node["Calibrate_FixPrincipalPointAtTheCenter"] >> calibFixPrincipalPoint;
        node["Input_FlipAroundHorizontalAxis"] >> flipVertical;
        node["Show_UndistortedImage"] >> showUndistorsed;
        node["Input"] >> input;
        node["Input_Delay"] >> delay;
        interprate();
    }
    void interprate()
    {
        goodInput = true;
        if (boardSize.width <= 0 || boardSize.height <= 0)
        {
            cerr << "Invalid Board size: " << boardSize.width << " " << boardSize.height << endl;
            goodInput = false;
        }
        if (squareSize <= 10e-6)
        {
            cerr << "Invalid square size " << squareSize << endl;
            goodInput = false;
        }
        if (nrFrames <= 0)
        {
            cerr << "Invalid number of frames " << nrFrames << endl;
            goodInput = false;
        }

        if (input.empty())      // Check for valid input
                inputType = INVALID;
        else
        {
            if (input[0] >= '0' && input[0] <= '9')
            {
                stringstream ss(input);
                ss >> cameraID;
                inputType = CAMERA;
            }
            else
            {
                if (readStringList(input, imageList))
                    {
                        inputType = IMAGE_LIST;
                        nrFrames = (nrFrames < (int)imageList.size()) ? nrFrames : (int)imageList.size();
                    }
                else
                    inputType = VIDEO_FILE;
            }
            if (inputType == CAMERA)
                inputCapture.open(cameraID);
            if (inputType == VIDEO_FILE)
                inputCapture.open(input);
            if (inputType != IMAGE_LIST && !inputCapture.isOpened())
                    inputType = INVALID;
        }
        if (inputType == INVALID)
        {
            cerr << " Inexistent input: " << input;
            goodInput = false;
        }

        flag = 0;
        if(calibFixPrincipalPoint) flag |= CV_CALIB_FIX_PRINCIPAL_POINT;
        if(calibZeroTangentDist)   flag |= CV_CALIB_ZERO_TANGENT_DIST;
        if(aspectRatio)            flag |= CV_CALIB_FIX_ASPECT_RATIO;


        calibrationPattern = NOT_EXISTING;
        if (!patternToUse.compare("CHESSBOARD")) calibrationPattern = CHESSBOARD;
        if (!patternToUse.compare("CIRCLES_GRID")) calibrationPattern = CIRCLES_GRID;
        if (!patternToUse.compare("ASYMMETRIC_CIRCLES_GRID")) calibrationPattern = ASYMMETRIC_CIRCLES_GRID;
        if (calibrationPattern == NOT_EXISTING)
            {
                cerr << " Inexistent camera calibration mode: " << patternToUse << endl;
                goodInput = false;
            }
        atImageList = 0;

    }
    Mat nextImage()
    {
        Mat result;
        if( inputCapture.isOpened() )
        {
            Mat view0;
            inputCapture >> view0;
            view0.copyTo(result);
        }
        else if( atImageList < (int)imageList.size() )
            result = imread(imageList[atImageList++], CV_LOAD_IMAGE_COLOR);

        return result;
    }

    static bool readStringList( const string& filename, vector<string>& l )
    {
        l.clear();
        FileStorage fs(filename, FileStorage::READ);
        if( !fs.isOpened() )
            return false;
        FileNode n = fs.getFirstTopLevelNode();
        if( n.type() != FileNode::SEQ )
            return false;
        FileNodeIterator it = n.begin(), it_end = n.end();
        for( ; it != it_end; ++it )
            l.push_back((string)*it);
        return true;
    }
public:
    Size boardSize;            // The size of the board -> Number of items by width and height
    Pattern calibrationPattern;// One of the Chessboard, circles, or asymmetric circle pattern
    float squareSize;          // The size of a square in your defined unit (point, millimeter,etc).
    int nrFrames;              // The number of frames to use from the input for calibration
    float aspectRatio;         // The aspect ratio
    int delay;                 // In case of a video input
    bool bwritePoints;         //  Write detected feature points
    bool bwriteExtrinsics;     // Write extrinsic parameters
    bool calibZeroTangentDist; // Assume zero tangential distortion
    bool calibFixPrincipalPoint;// Fix the principal point at the center
    bool flipVertical;          // Flip the captured images around the horizontal axis
    string outputFileName;      // The name of the file where to write
    bool showUndistorsed;       // Show undistorted images after calibration
    string input;               // The input ->



    int cameraID;
    vector<string> imageList;
    int atImageList;
    VideoCapture inputCapture;
    InputType inputType;
    bool goodInput;
    int flag;

private:
    string patternToUse;


};

static void read(const FileNode& node, Settings& x, const Settings& default_value = Settings())
{
    if(node.empty())
        x = default_value;
    else
        x.read(node);
}

enum { DETECTION = 0, CAPTURING = 1, CALIBRATED = 2 };

bool runCalibrationAndSave(Settings& s, Size imageSize, Mat&  cameraMatrix, Mat& distCoeffs,
                           vector<vector<Point2f> > imagePoints );

int main(int argc, char* argv[])
{
    help();
    Settings s;
    const string inputSettingsFile = argc > 1 ? argv[1] : "default.xml";
    FileStorage fs(inputSettingsFile, FileStorage::READ); // Read the settings
    if (!fs.isOpened())
    {
        cout << "Could not open the configuration file: \"" << inputSettingsFile << "\"" << endl;
        return -1;
    }
    fs["Settings"] >> s;
    fs.release();                                         // close Settings file

    if (!s.goodInput)
    {
        cout << "Invalid input detected. Application stopping. " << endl;
        return -1;
    }

    vector<vector<Point2f> > imagePoints;
    Mat cameraMatrix, distCoeffs;
    Size imageSize;
    int mode = s.inputType == Settings::IMAGE_LIST ? CAPTURING : DETECTION;
    clock_t prevTimestamp = 0;
    const Scalar RED(0,0,255), GREEN(0,255,0);
    const char ESC_KEY = 27;

    for(int i = 0;;++i)
    {
      Mat view;
      bool blinkOutput = false;

      view = s.nextImage();

      //-----  If no more image, or got enough, then stop calibration and show result -------------
      if( mode == CAPTURING && imagePoints.size() >= (unsigned)s.nrFrames )
      {
          if( runCalibrationAndSave(s, imageSize,  cameraMatrix, distCoeffs, imagePoints))
              mode = CALIBRATED;
          else
              mode = DETECTION;
      }
      if(view.empty())          // If no more images then run calibration, save and stop loop.
      {
            if( imagePoints.size() > 0 )
                runCalibrationAndSave(s, imageSize,  cameraMatrix, distCoeffs, imagePoints);
            break;
      }


        imageSize = view.size();  // Format input image.
        if( s.flipVertical )    flip( view, view, 0 );

        vector<Point2f> pointBuf;

        bool found;
        switch( s.calibrationPattern ) // Find feature points on the input format
        {
        case Settings::CHESSBOARD:
            found = findChessboardCorners( view, s.boardSize, pointBuf,
                CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);
            break;
        case Settings::CIRCLES_GRID:
            found = findCirclesGrid( view, s.boardSize, pointBuf );
            break;
        case Settings::ASYMMETRIC_CIRCLES_GRID:
            found = findCirclesGrid( view, s.boardSize, pointBuf, CALIB_CB_ASYMMETRIC_GRID );
            break;
        default:
            found = false;
            break;
        }

        if ( found)                // If done with success,
        {
              // improve the found corners' coordinate accuracy for chessboard
                if( s.calibrationPattern == Settings::CHESSBOARD)
                {
                    Mat viewGray;
                    cvtColor(view, viewGray, CV_BGR2GRAY);
                    cornerSubPix( viewGray, pointBuf, Size(11,11),
                        Size(-1,-1), TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));
                }

                if( mode == CAPTURING &&  // For camera only take new samples after delay time
                    (!s.inputCapture.isOpened() || clock() - prevTimestamp > s.delay*1e-3*CLOCKS_PER_SEC) )
                {
                    imagePoints.push_back(pointBuf);
                    prevTimestamp = clock();
                    blinkOutput = s.inputCapture.isOpened();
                }

                // Draw the corners.
                drawChessboardCorners( view, s.boardSize, Mat(pointBuf), found );
        }

        //----------------------------- Output Text ------------------------------------------------
        string msg = (mode == CAPTURING) ? "100/100" :
                      mode == CALIBRATED ? "Calibrated" : "Press 'g' to start";
        int baseLine = 0;
        Size textSize = getTextSize(msg, 1, 1, 1, &baseLine);
        Point textOrigin(view.cols - 2*textSize.width - 10, view.rows - 2*baseLine - 10);

        if( mode == CAPTURING )
        {
            if(s.showUndistorsed)
                msg = format( "%d/%d Undist", (int)imagePoints.size(), s.nrFrames );
            else
                msg = format( "%d/%d", (int)imagePoints.size(), s.nrFrames );
        }

        putText( view, msg, textOrigin, 1, 1, mode == CALIBRATED ?  GREEN : RED);

        if( blinkOutput )
            bitwise_not(view, view);

        //------------------------- Video capture  output  undistorted ------------------------------
        if( mode == CALIBRATED && s.showUndistorsed )
        {
            Mat temp = view.clone();
            undistort(temp, view, cameraMatrix, distCoeffs);
        }

        //------------------------------ Show image and check for input commands -------------------
        imshow("Image View", view);
        char key = (char)waitKey(s.inputCapture.isOpened() ? 50 : s.delay);

        if( key  == ESC_KEY )
            break;

        if( key == 'u' && mode == CALIBRATED )
           s.showUndistorsed = !s.showUndistorsed;

        if( s.inputCapture.isOpened() && key == 'g' )
        {
            mode = CAPTURING;
            imagePoints.clear();
        }
    }

    // -----------------------Show the undistorted image for the image list ------------------------
    if( s.inputType == Settings::IMAGE_LIST && s.showUndistorsed )
    {
        Mat view, rview, map1, map2;
        initUndistortRectifyMap(cameraMatrix, distCoeffs, Mat(),
            getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1, imageSize, 0),
            imageSize, CV_16SC2, map1, map2);

        for(int i = 0; i < (int)s.imageList.size(); i++ )
        {
            view = imread(s.imageList[i], 1);
            if(view.empty())
                continue;
            remap(view, rview, map1, map2, INTER_LINEAR);
            imshow("Image View", rview);
            char c = (char)waitKey();
            if( c  == ESC_KEY || c == 'q' || c == 'Q' )
                break;
        }
    }


    return 0;
}

static double computeReprojectionErrors( const vector<vector<Point3f> >& objectPoints,
                                         const vector<vector<Point2f> >& imagePoints,
                                         const vector<Mat>& rvecs, const vector<Mat>& tvecs,
                                         const Mat& cameraMatrix , const Mat& distCoeffs,
                                         vector<float>& perViewErrors)
{
    vector<Point2f> imagePoints2;
    int i, totalPoints = 0;
    double totalErr = 0, err;
    perViewErrors.resize(objectPoints.size());

    for( i = 0; i < (int)objectPoints.size(); ++i )
    {
        projectPoints( Mat(objectPoints[i]), rvecs[i], tvecs[i], cameraMatrix,
                       distCoeffs, imagePoints2);
        err = norm(Mat(imagePoints[i]), Mat(imagePoints2), CV_L2);

        int n = (int)objectPoints[i].size();
        perViewErrors[i] = (float) std::sqrt(err*err/n);
        totalErr        += err*err;
        totalPoints     += n;
    }

    return std::sqrt(totalErr/totalPoints);
}

static void calcBoardCornerPositions(Size boardSize, float squareSize, vector<Point3f>& corners,
                                     Settings::Pattern patternType /*= Settings::CHESSBOARD*/)
{
    corners.clear();

    switch(patternType)
    {
    case Settings::CHESSBOARD:
    case Settings::CIRCLES_GRID:
        for( int i = 0; i < boardSize.height; ++i )
            for( int j = 0; j < boardSize.width; ++j )
                corners.push_back(Point3f(float( j*squareSize ), float( i*squareSize ), 0));
        break;

    case Settings::ASYMMETRIC_CIRCLES_GRID:
        for( int i = 0; i < boardSize.height; i++ )
            for( int j = 0; j < boardSize.width; j++ )
                corners.push_back(Point3f(float((2*j + i % 2)*squareSize), float(i*squareSize), 0));
        break;
    default:
        break;
    }
}

static bool runCalibration( Settings& s, Size& imageSize, Mat& cameraMatrix, Mat& distCoeffs,
                            vector<vector<Point2f> > imagePoints, vector<Mat>& rvecs, vector<Mat>& tvecs,
                            vector<float>& reprojErrs,  double& totalAvgErr)
{

    cameraMatrix = Mat::eye(3, 3, CV_64F);
    if( s.flag & CV_CALIB_FIX_ASPECT_RATIO )
        cameraMatrix.at<double>(0,0) = 1.0;

    distCoeffs = Mat::zeros(8, 1, CV_64F);

    vector<vector<Point3f> > objectPoints(1);
    calcBoardCornerPositions(s.boardSize, s.squareSize, objectPoints[0], s.calibrationPattern);

    objectPoints.resize(imagePoints.size(),objectPoints[0]);

    //Find intrinsic and extrinsic camera parameters
    double rms = calibrateCamera(objectPoints, imagePoints, imageSize, cameraMatrix,
                                 distCoeffs, rvecs, tvecs, s.flag|CV_CALIB_FIX_K4|CV_CALIB_FIX_K5);

    cout << "Re-projection error reported by calibrateCamera: "<< rms << endl;

    bool ok = checkRange(cameraMatrix) && checkRange(distCoeffs);

    totalAvgErr = computeReprojectionErrors(objectPoints, imagePoints,
                                             rvecs, tvecs, cameraMatrix, distCoeffs, reprojErrs);

    return ok;
}

// Print camera parameters to the output file
static void saveCameraParams( Settings& s, Size& imageSize, Mat& cameraMatrix, Mat& distCoeffs,
                              const vector<Mat>& rvecs, const vector<Mat>& tvecs,
                              const vector<float>& reprojErrs, const vector<vector<Point2f> >& imagePoints,
                              double totalAvgErr )
{
    FileStorage fs( s.outputFileName, FileStorage::WRITE );

    time_t tm;
    time( &tm );
    struct tm *t2 = localtime( &tm );
    char buf[1024];
    strftime( buf, sizeof(buf)-1, "%c", t2 );

    fs << "calibration_Time" << buf;

    if( !rvecs.empty() || !reprojErrs.empty() )
        fs << "nrOfFrames" << (int)std::max(rvecs.size(), reprojErrs.size());
    fs << "image_Width" << imageSize.width;
    fs << "image_Height" << imageSize.height;
    fs << "board_Width" << s.boardSize.width;
    fs << "board_Height" << s.boardSize.height;
    fs << "square_Size" << s.squareSize;

    if( s.flag & CV_CALIB_FIX_ASPECT_RATIO )
        fs << "FixAspectRatio" << s.aspectRatio;

    if( s.flag )
    {
        sprintf( buf, "flags: %s%s%s%s",
            s.flag & CV_CALIB_USE_INTRINSIC_GUESS ? " +use_intrinsic_guess" : "",
            s.flag & CV_CALIB_FIX_ASPECT_RATIO ? " +fix_aspectRatio" : "",
            s.flag & CV_CALIB_FIX_PRINCIPAL_POINT ? " +fix_principal_point" : "",
            s.flag & CV_CALIB_ZERO_TANGENT_DIST ? " +zero_tangent_dist" : "" );
        cvWriteComment( *fs, buf, 0 );

    }

    fs << "flagValue" << s.flag;

    fs << "Camera_Matrix" << cameraMatrix;
    fs << "Distortion_Coefficients" << distCoeffs;

    fs << "Avg_Reprojection_Error" << totalAvgErr;
    if( !reprojErrs.empty() )
        fs << "Per_View_Reprojection_Errors" << Mat(reprojErrs);

    if( !rvecs.empty() && !tvecs.empty() )
    {
        CV_Assert(rvecs[0].type() == tvecs[0].type());
        Mat bigmat((int)rvecs.size(), 6, rvecs[0].type());
        for( int i = 0; i < (int)rvecs.size(); i++ )
        {
            Mat r = bigmat(Range(i, i+1), Range(0,3));
            Mat t = bigmat(Range(i, i+1), Range(3,6));

            CV_Assert(rvecs[i].rows == 3 && rvecs[i].cols == 1);
            CV_Assert(tvecs[i].rows == 3 && tvecs[i].cols == 1);
            //*.t() is MatExpr (not Mat) so we can use assignment operator
            r = rvecs[i].t();
            t = tvecs[i].t();
        }
        cvWriteComment( *fs, "a set of 6-tuples (rotation vector + translation vector) for each view", 0 );
        fs << "Extrinsic_Parameters" << bigmat;
    }

    if( !imagePoints.empty() )
    {
        Mat imagePtMat((int)imagePoints.size(), (int)imagePoints[0].size(), CV_32FC2);
        for( int i = 0; i < (int)imagePoints.size(); i++ )
        {
            Mat r = imagePtMat.row(i).reshape(2, imagePtMat.cols);
            Mat imgpti(imagePoints[i]);
            imgpti.copyTo(r);
        }
        fs << "Image_points" << imagePtMat;
    }
}

bool runCalibrationAndSave(Settings& s, Size imageSize, Mat&  cameraMatrix, Mat& distCoeffs,vector<vector<Point2f> > imagePoints )
{
    vector<Mat> rvecs, tvecs;
    vector<float> reprojErrs;
    double totalAvgErr = 0;

    bool ok = runCalibration(s,imageSize, cameraMatrix, distCoeffs, imagePoints, rvecs, tvecs,
                             reprojErrs, totalAvgErr);
    cout << (ok ? "Calibration succeeded" : "Calibration failed")
        << ". avg re projection error = "  << totalAvgErr ;

    if( ok )
        saveCameraParams( s, imageSize, cameraMatrix, distCoeffs, rvecs ,tvecs, reprojErrs,
                            imagePoints, totalAvgErr);
    return ok;
}


int main_kinect_calibration(int argc, char* argv[]){
	double _last_tick = 0;
	int _frame_counter = 0;
	float _frame_rate = 0;

	bool result;

	if(!NIKinect2::ni_initialize())
		return -1;
	
	NIKinect2* kinect = new NIKinect2();
	result = kinect->initialize();
	if(result){
		kinect->enable_depth_generator();
		kinect->enable_color_generator();
		//kinect->enable_user_generator();
		kinect->set_depth_color_registration(true);
	}

	cv::Mat depth_kinect;

	cv::vector<cv::Point2f> point_kinect, point_camera;


	cv::VideoCapture cap(0);
	if (!cap.isOpened()){
		printf("Error openning camera.\n");
		getchar();
		return -1;
	}

	//result = cap.set(CV_CAP_PROP_FRAME_WIDTH,320);
	//result = cap.set(CV_CAP_PROP_FRAME_HEIGHT,240);
	//result = cap.set(CV_CAP_PROP_FPS,30);
	//
	double dWidth = cap.get(CV_CAP_PROP_FRAME_WIDTH); //get the width of frames of the video
	double dHeight = cap.get(CV_CAP_PROP_FRAME_HEIGHT); //get the height of frames of the video

	//cv::Size boardSize(5,4);
	cv::Size boardSize(9,6);

	FILE* file = NULL;
	int n_images = 0;

	bool homography = false;
	bool remap = false;
	bool remap2 = false;

	int time = 1;
	char c = 0;
	while(c != 27){
		if(!cap.read(color[CAMERA])) 
			break;

		if(!NIKinect2::ni_update() || !kinect->update())
			break;
		
		if(!kinect->get_color(color[KINECT]) || !color[KINECT].rows)
			break;

		imshow("Kinect",color[KINECT]);
		imshow("camera",color[CAMERA]);

		c = cv::waitKey(11);
	}

	exit(0);
}

//bool chess = false, record = false;
//
//
//cv::Mat R1, R2, P1, P2, Q;
//cv::Mat rmap[2][2];
//cv::Size imageSize;
//Mat cameraMatrix[2], distCoeffs[2];
//Rect validRoi[2];
//
//
//CvMat *_Q2 = (CvMat *)cvLoad("Q.xml",NULL,NULL,NULL);
//CvMat *_mx1 = (CvMat *)cvLoad("mx1.xml",NULL,NULL,NULL);
//CvMat *_my1 = (CvMat *)cvLoad("my1.xml",NULL,NULL,NULL);
//CvMat *_mx2 = (CvMat *)cvLoad("mx2.xml",NULL,NULL,NULL);
//CvMat *_my2 = (CvMat *)cvLoad("my2.xml",NULL,NULL,NULL);
//
//cv::Mat Q2 = cv::imread("Q.xml");
//cv::Mat mx1 = cv::imread("mx1.xml");//(CvMat *)cvLoad("mx1.xml",NULL,NULL,NULL);
//cv::Mat my1 = cv::imread("my1.xml");//(CvMat *)cvLoad("my1.xml",NULL,NULL,NULL);
//cv::Mat mx2 = cv::imread("mx2.xml");//(CvMat *)cvLoad("mx2.xml",NULL,NULL,NULL);
//cv::Mat my2 = cv::imread("my2.xml");//(CvMat *)cvLoad("my2.xml",NULL,NULL,NULL);
//
//
//void
//StereoCalib(const vector<string>& imagelist, Size boardSize, bool useCalibrated=true, bool showRectified=true)
//{
//    if( imagelist.size() % 2 != 0 )
//    {
//        cout << "Error: the image list contains odd (non-even) number of elements\n";
//        return;
//    }
//
//    bool displayCorners = false;//true;
//    const int maxScale = 2;
//    const float squareSize = 4.5f;  // Set this to your actual square size
//    // ARRAY AND VECTOR STORAGE:
//
//    vector<vector<Point2f> > imagePoints[2];
//    vector<vector<Point3f> > objectPoints;
//    //Size imageSize;
//
//    int i, j, k, nimages = (int)imagelist.size()/2;
//
//    imagePoints[0].resize(nimages);
//    imagePoints[1].resize(nimages);
//    vector<string> goodImageList;
//
//    for( i = j = 0; i < nimages; i++ )
//    {
//        for( k = 0; k < 2; k++ )
//        {
//            const string& filename = imagelist[i*2+k];
//            Mat img = imread(filename, 0);
//            if(img.empty())
//                break;
//            if( imageSize == Size() )
//                imageSize = img.size();
//            else if( img.size() != imageSize )
//            {
//                cout << "The image " << filename << " has the size different from the first image size. Skipping the pair\n";
//                break;
//            }
//            bool found = false;
//            vector<Point2f>& corners = imagePoints[k][j];
//            for( int scale = 1; scale <= maxScale; scale++ )
//            {
//                Mat timg;
//                if( scale == 1 )
//                    timg = img;
//                else
//                    resize(img, timg, Size(), scale, scale);
//                found = findChessboardCorners(timg, boardSize, corners,
//                    CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE);
//                if( found )
//                {
//                    if( scale > 1 )
//                    {
//                        Mat cornersMat(corners);
//                        cornersMat *= 1./scale;
//                    }
//                    break;
//                }
//            }
//            if( displayCorners )
//            {
//                cout << filename << endl;
//                Mat cimg, cimg1;
//                cvtColor(img, cimg, CV_GRAY2BGR);
//                drawChessboardCorners(cimg, boardSize, corners, found);
//                double sf = 640./MAX(img.rows, img.cols);
//                resize(cimg, cimg1, Size(), sf, sf);
//                imshow("corners", cimg1);
//                //char c = (char)waitKey(500);
//                //if( c == 27 || c == 'q' || c == 'Q' ) //Allow ESC to quit
//                //    exit(-1);
//            }
//            else
//                putchar('.');
//            if( !found )
//                break;
//            cornerSubPix(img, corners, Size(11,11), Size(-1,-1),
//                         TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS,
//                                      30, 0.01));
//        }
//        if( k == 2 )
//        {
//            goodImageList.push_back(imagelist[i*2]);
//            goodImageList.push_back(imagelist[i*2+1]);
//            j++;
//        }
//    }
//    cout << j << " pairs have been successfully detected.\n";
//    nimages = j;
//    if( nimages < 2 )
//    {
//        cout << "Error: too little pairs to run the calibration\n";
//        return;
//    }
//
//    imagePoints[0].resize(nimages);
//    imagePoints[1].resize(nimages);
//    objectPoints.resize(nimages);
//
//    for( i = 0; i < nimages; i++ )
//    {
//        for( j = 0; j < boardSize.height; j++ )
//            for( k = 0; k < boardSize.width; k++ )
//                objectPoints[i].push_back(Point3f(j*squareSize, k*squareSize, 0));
//    }
//
//    cout << "Running stereo calibration ...\n";
//
//    //Mat cameraMatrix[2], distCoeffs[2];
//    cameraMatrix[0] = Mat::eye(3, 3, CV_64F);
//    cameraMatrix[1] = Mat::eye(3, 3, CV_64F);
//    Mat R, T, E, F;
//
//    double rms = stereoCalibrate(objectPoints, imagePoints[0], imagePoints[1],
//                    cameraMatrix[0], distCoeffs[0],
//                    cameraMatrix[1], distCoeffs[1],
//                    imageSize, R, T, E, F,
//                    TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 100, 1e-5),
//                    CV_CALIB_FIX_ASPECT_RATIO 
//                    //CV_CALIB_ZERO_TANGENT_DIST +
//                    //CV_CALIB_SAME_FOCAL_LENGTH +
//                    //CV_CALIB_RATIONAL_MODEL +
//                    /*CV_CALIB_FIX_K3 + CV_CALIB_FIX_K4 + CV_CALIB_FIX_K5*/);
//    cout << "done with RMS error=" << rms << endl;
//
//// CALIBRATION QUALITY CHECK
//// because the output fundamental matrix implicitly
//// includes all the output information,
//// we can check the quality of calibration using the
//// epipolar geometry constraint: m2^t*F*m1=0
//    double err = 0;
//    int npoints = 0;
//    vector<Vec3f> lines[2];
//    for( i = 0; i < nimages; i++ )
//    {
//        int npt = (int)imagePoints[0][i].size();
//        Mat imgpt[2];
//        for( k = 0; k < 2; k++ )
//        {
//            imgpt[k] = Mat(imagePoints[k][i]);
//            undistortPoints(imgpt[k], imgpt[k], cameraMatrix[k], distCoeffs[k], Mat(), cameraMatrix[k]);
//            computeCorrespondEpilines(imgpt[k], k+1, F, lines[k]);
//        }
//        for( j = 0; j < npt; j++ )
//        {
//            double errij = fabs(imagePoints[0][i][j].x*lines[1][j][0] +
//                                imagePoints[0][i][j].y*lines[1][j][1] + lines[1][j][2]) +
//                           fabs(imagePoints[1][i][j].x*lines[0][j][0] +
//                                imagePoints[1][i][j].y*lines[0][j][1] + lines[0][j][2]);
//            err += errij;
//        }
//        npoints += npt;
//    }
//    cout << "average reprojection err = " <<  err/npoints << endl;
//
//    // save intrinsic parameters
//    FileStorage fs("intrinsics.yml", CV_STORAGE_WRITE);
//    if( fs.isOpened() )
//    {
//        fs << "M1" << cameraMatrix[0] << "D1" << distCoeffs[0] <<
//            "M2" << cameraMatrix[1] << "D2" << distCoeffs[1];
//        fs.release();
//    }
//    else
//        cout << "Error: can not save the intrinsic parameters\n";
//
//    //Mat R1, R2, P1, P2, Q;
//    //Rect validRoi[2];
//
//    stereoRectify(cameraMatrix[0], distCoeffs[0],
//                  cameraMatrix[1], distCoeffs[1],
//                  imageSize, R, T, R1, R2, P1, P2, Q,
//                  0, 1., imageSize, &validRoi[0], &validRoi[1]);
//
//    fs.open("extrinsics.yml", CV_STORAGE_WRITE);
//    if( fs.isOpened() )
//    {
//        fs << "R" << R << "T" << T << "R1" << R1 << "R2" << R2 << "P1" << P1 << "P2" << P2 << "Q" << Q;
//        fs.release();
//    }
//    else
//        cout << "Error: can not save the intrinsic parameters\n";
//
//    // OpenCV can handle left-right
//    // or up-down camera arrangements
//    bool isVerticalStereo = fabs(P2.at<double>(1, 3)) > fabs(P2.at<double>(0, 3));
//
//// COMPUTE AND DISPLAY RECTIFICATION
//    if( !showRectified )
//        return;
//
//    Mat rmap[2][2];
//// IF BY CALIBRATED (BOUGUET'S METHOD)
//    if( useCalibrated )
//    {
//        // we already computed everything
//    }
//// OR ELSE HARTLEY'S METHOD
//    else
// // use intrinsic parameters of each camera, but
// // compute the rectification transformation directly
// // from the fundamental matrix
//    {
//        vector<Point2f> allimgpt[2];
//        for( k = 0; k < 2; k++ )
//        {
//            for( i = 0; i < nimages; i++ )
//                std::copy(imagePoints[k][i].begin(), imagePoints[k][i].end(), back_inserter(allimgpt[k]));
//        }
//        F = findFundamentalMat(Mat(allimgpt[0]), Mat(allimgpt[1]), FM_8POINT, 0, 0);
//        Mat H1, H2;
//        stereoRectifyUncalibrated(Mat(allimgpt[0]), Mat(allimgpt[1]), F, imageSize, H1, H2, 3);
//
//        R1 = cameraMatrix[0].inv()*H1*cameraMatrix[0];
//        R2 = cameraMatrix[1].inv()*H2*cameraMatrix[1];
//        P1 = cameraMatrix[0];
//        P2 = cameraMatrix[1];
//    }
//
//    //Precompute maps for cv::remap()
//    initUndistortRectifyMap(cameraMatrix[0], distCoeffs[0], R1, P1, imageSize, CV_16SC2, rmap[0][0], rmap[0][1]);
//    initUndistortRectifyMap(cameraMatrix[1], distCoeffs[1], R2, P2, imageSize, CV_16SC2, rmap[1][0], rmap[1][1]);
//
//    Mat canvas;
//    double sf;
//    int w, h;
//    if( !isVerticalStereo )
//    {
//        sf = 600./MAX(imageSize.width, imageSize.height);
//        w = cvRound(imageSize.width*sf);
//        h = cvRound(imageSize.height*sf);
//        canvas.create(h, w*2, CV_8UC3);
//    }
//    else
//    {
//        sf = 300./MAX(imageSize.width, imageSize.height);
//        w = cvRound(imageSize.width*sf);
//        h = cvRound(imageSize.height*sf);
//        canvas.create(h*2, w, CV_8UC3);
//    }
//
//    for( i = 0; i < nimages; i++ )
//    {
//        for( k = 0; k < 2; k++ )
//        {
//            Mat img = imread(goodImageList[i*2+k], 0), rimg, cimg;
//            remap(img, rimg, rmap[k][0], rmap[k][1], CV_INTER_LINEAR);
//            cvtColor(rimg, cimg, CV_GRAY2BGR);
//            Mat canvasPart = !isVerticalStereo ? canvas(Rect(w*k, 0, w, h)) : canvas(Rect(0, h*k, w, h));
//            resize(cimg, canvasPart, canvasPart.size(), 0, 0, CV_INTER_AREA);
//            if( useCalibrated )
//            {
//                Rect vroi(cvRound(validRoi[k].x*sf), cvRound(validRoi[k].y*sf),
//                          cvRound(validRoi[k].width*sf), cvRound(validRoi[k].height*sf));
//                rectangle(canvasPart, vroi, Scalar(0,0,255), 3, 8);
//            }
//        }
//
//        if( !isVerticalStereo )
//            for( j = 0; j < canvas.rows; j += 16 )
//                line(canvas, Point(0, j), Point(canvas.cols, j), Scalar(0, 255, 0), 1, 8);
//        else
//            for( j = 0; j < canvas.cols; j += 16 )
//                line(canvas, Point(j, 0), Point(j, canvas.rows), Scalar(0, 255, 0), 1, 8);
//        imshow("rectified", canvas);
//		printf("image(%d)\n",i);
//        char c = (char)waitKey();
//        if( c == 27 || c == 'q' || c == 'Q' )
//            break;
//    }
//}
//
//
//static void
//StereoCalib2(const char* imageList, int nx, int ny, int useUncalibrated, float _squareSize)
//{
//    int displayCorners = 1;
//    int showUndistorted = 1;
//    bool isVerticalStereo = false;//OpenCV can handle left-right
//                                      //or up-down camera arrangements
//    const int maxScale = 1;
//    const float squareSize = _squareSize; //Chessboard square size in cm
//    FILE* f = fopen(imageList, "rt");
//    int i, j, lr, nframes, n = nx*ny, N = 0;
//    vector<string> imageNames[2];
//    vector<CvPoint3D32f> objectPoints;
//    vector<CvPoint2D32f> points[2];
//    vector<int> npoints;
//    vector<uchar> active[2];
//    vector<CvPoint2D32f> temp(n);
//    CvSize imageSize = {0,0};
//    // ARRAY AND VECTOR STORAGE:
//    double M1[3][3], M2[3][3], D1[5], D2[5];
//    double R[3][3], T[3], E[3][3], F[3][3];
//    double Q[4][4];
//    CvMat _M1 = cvMat(3, 3, CV_64F, M1 );
//    CvMat _M2 = cvMat(3, 3, CV_64F, M2 );
//    CvMat _D1 = cvMat(1, 5, CV_64F, D1 );
//    CvMat _D2 = cvMat(1, 5, CV_64F, D2 );
//    CvMat _R = cvMat(3, 3, CV_64F, R );
//    CvMat _T = cvMat(3, 1, CV_64F, T );
//    CvMat _E = cvMat(3, 3, CV_64F, E );
//    CvMat _F = cvMat(3, 3, CV_64F, F );
//    CvMat _Q = cvMat(4,4, CV_64F, Q);
//    if( displayCorners )
//        cvNamedWindow( "corners", 1 );
//// READ IN THE LIST OF CHESSBOARDS:
//    if( !f )
//    {
//        fprintf(stderr, "can not open file %s\n", imageList );
//        return;
//    }
//    for(i=0;;i++)
//    {
//        char buf[1024];
//        int count = 0, result=0;
//        lr = i % 2;
//        vector<CvPoint2D32f>& pts = points[lr];
//        if( !fgets( buf, sizeof(buf)-3, f ))
//            break;
//        size_t len = strlen(buf);
//        while( len > 0 && isspace(buf[len-1]))
//            buf[--len] = '\0';
//        if( buf[0] == '#')
//            continue;
//        IplImage* img = cvLoadImage( buf, 0 );
//        if( !img )
//            break;
//        imageSize = cvGetSize(img);
//        imageNames[lr].push_back(buf);
//    //FIND CHESSBOARDS AND CORNERS THEREIN:
//        for( int s = 1; s <= maxScale; s++ )
//        {
//            IplImage* timg = img;
//            if( s > 1 )
//            {
//                timg = cvCreateImage(cvSize(img->width*s,img->height*s),
//                    img->depth, img->nChannels );
//                cvResize( img, timg, CV_INTER_CUBIC );
//            }
//            result = cvFindChessboardCorners( timg, cvSize(nx, ny),
//                &temp[0], &count,
//                CV_CALIB_CB_ADAPTIVE_THRESH |
//                CV_CALIB_CB_NORMALIZE_IMAGE);
//            if( timg != img )
//                cvReleaseImage( &timg );
//            if( result || s == maxScale )
//                for( j = 0; j < count; j++ )
//            {
//                temp[j].x /= s;
//                temp[j].y /= s;
//            }
//            if( result )
//                break;
//        }
//        if( displayCorners )
//        {
//            printf("%s\n", buf);
//            IplImage* cimg = cvCreateImage( imageSize, 8, 3 );
//            cvCvtColor( img, cimg, CV_GRAY2BGR );
//            cvDrawChessboardCorners( cimg, cvSize(nx, ny), &temp[0],
//                count, result );
//            cvShowImage( "corners", cimg );
//            cvReleaseImage( &cimg );
//            if( cvWaitKey(0) == 27 ) //Allow ESC to quit
//                exit(-1);
//        }
//        else
//            putchar('.');
//        N = pts.size();
//        pts.resize(N + n, cvPoint2D32f(0,0));
//        active[lr].push_back((uchar)result);
//    //assert( result != 0 );
//        if( result )
//        {
//         //Calibration will suffer without subpixel interpolation
//            cvFindCornerSubPix( img, &temp[0], count,
//                cvSize(11, 11), cvSize(-1,-1),
//                cvTermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS,
//                30, 0.01) );
//            copy( temp.begin(), temp.end(), pts.begin() + N );
//        }
//        cvReleaseImage( &img );
//    }
//    fclose(f);
//    printf("\n");
//// HARVEST CHESSBOARD 3D OBJECT POINT LIST:
//    nframes = active[0].size();//Number of good chessboads found
//    objectPoints.resize(nframes*n);
//    for( i = 0; i < ny; i++ )
//        for( j = 0; j < nx; j++ )
//        objectPoints[i*nx + j] = cvPoint3D32f(i*squareSize, j*squareSize, 0);
//    for( i = 1; i < nframes; i++ )
//        copy( objectPoints.begin(), objectPoints.begin() + n,
//        objectPoints.begin() + i*n );
//    npoints.resize(nframes,n);
//    N = nframes*n;
//    CvMat _objectPoints = cvMat(1, N, CV_32FC3, &objectPoints[0] );
//    CvMat _imagePoints1 = cvMat(1, N, CV_32FC2, &points[0][0] );
//    CvMat _imagePoints2 = cvMat(1, N, CV_32FC2, &points[1][0] );
//    CvMat _npoints = cvMat(1, npoints.size(), CV_32S, &npoints[0] );
//    cvSetIdentity(&_M1);
//    cvSetIdentity(&_M2);
//    cvZero(&_D1);
//    cvZero(&_D2);
//
//// CALIBRATE THE STEREO CAMERAS
//    printf("Running stereo calibration ...");
//    fflush(stdout);
//    cvStereoCalibrate( &_objectPoints, &_imagePoints1,
//        &_imagePoints2, &_npoints,
//        &_M1, &_D1, &_M2, &_D2,
//        imageSize, &_R, &_T, &_E, &_F,
//        cvTermCriteria(CV_TERMCRIT_ITER+
//        CV_TERMCRIT_EPS, 100, 1e-5),
//        CV_CALIB_FIX_ASPECT_RATIO +
//        CV_CALIB_ZERO_TANGENT_DIST );
//    printf(" done\n");
//// CALIBRATION QUALITY CHECK
//// because the output fundamental matrix implicitly
//// includes all the output information,
//// we can check the quality of calibration using the
//// epipolar geometry constraint: m2^t*F*m1=0
//    vector<CvPoint3D32f> lines[2];
//    points[0].resize(N);
//    points[1].resize(N);
//    _imagePoints1 = cvMat(1, N, CV_32FC2, &points[0][0] );
//    _imagePoints2 = cvMat(1, N, CV_32FC2, &points[1][0] );
//    lines[0].resize(N);
//    lines[1].resize(N);
//    CvMat _L1 = cvMat(1, N, CV_32FC3, &lines[0][0]);
//    CvMat _L2 = cvMat(1, N, CV_32FC3, &lines[1][0]);
////Always work in undistorted space
//    cvUndistortPoints( &_imagePoints1, &_imagePoints1,
//        &_M1, &_D1, 0, &_M1 );
//    cvUndistortPoints( &_imagePoints2, &_imagePoints2,
//        &_M2, &_D2, 0, &_M2 );
//    cvComputeCorrespondEpilines( &_imagePoints1, 1, &_F, &_L1 );
//    cvComputeCorrespondEpilines( &_imagePoints2, 2, &_F, &_L2 );
//    double avgErr = 0;
//    for( i = 0; i < N; i++ )
//    {
//        double err = fabs(points[0][i].x*lines[1][i].x +
//            points[0][i].y*lines[1][i].y + lines[1][i].z)
//            + fabs(points[1][i].x*lines[0][i].x +
//            points[1][i].y*lines[0][i].y + lines[0][i].z);
//        avgErr += err;
//    }
//    printf( "avg err = %g\n", avgErr/(nframes*n) );
////COMPUTE AND DISPLAY RECTIFICATION
//    if( showUndistorted )
//    {
//        CvMat* mx1 = cvCreateMat( imageSize.height,
//            imageSize.width, CV_32F );
//        CvMat* my1 = cvCreateMat( imageSize.height,
//            imageSize.width, CV_32F );
//        CvMat* mx2 = cvCreateMat( imageSize.height,
//
//            imageSize.width, CV_32F );
//        CvMat* my2 = cvCreateMat( imageSize.height,
//            imageSize.width, CV_32F );
//        CvMat* img1r = cvCreateMat( imageSize.height,
//            imageSize.width, CV_8U );
//        CvMat* img2r = cvCreateMat( imageSize.height,
//            imageSize.width, CV_8U );
//        CvMat* disp = cvCreateMat( imageSize.height,
//            imageSize.width, CV_16S );
//        CvMat* vdisp = cvCreateMat( imageSize.height,
//            imageSize.width, CV_8U );
//        CvMat* pair;
//        double R1[3][3], R2[3][3], P1[3][4], P2[3][4];
//        CvMat _R1 = cvMat(3, 3, CV_64F, R1);
//        CvMat _R2 = cvMat(3, 3, CV_64F, R2);
//// IF BY CALIBRATED (BOUGUET'S METHOD)
//        if( useUncalibrated == 0 )
//        {
//            CvMat _P1 = cvMat(3, 4, CV_64F, P1);
//            CvMat _P2 = cvMat(3, 4, CV_64F, P2);
//            cvStereoRectify( &_M1, &_M2, &_D1, &_D2, imageSize,
//                &_R, &_T,
//                &_R1, &_R2, &_P1, &_P2, &_Q,
//                0/*CV_CALIB_ZERO_DISPARITY*/ );
//            isVerticalStereo = fabs(P2[1][3]) > fabs(P2[0][3]);
//    //Precompute maps for cvRemap()
//            cvInitUndistortRectifyMap(&_M1,&_D1,&_R1,&_P1,mx1,my1);
//            cvInitUndistortRectifyMap(&_M2,&_D2,&_R2,&_P2,mx2,my2);
//            
//    //Save parameters
//            cvSave("M1.xml",&_M1);
//            cvSave("D1.xml",&_D1);
//            cvSave("R1.xml",&_R1);
//            cvSave("P1.xml",&_P1);
//            cvSave("M2.xml",&_M2);
//            cvSave("D2.xml",&_D2);
//            cvSave("R2.xml",&_R2);
//            cvSave("P2.xml",&_P2);
//            cvSave("Q.xml",&_Q);
//            cvSave("mx1.xml",mx1);
//            cvSave("my1.xml",my1);
//            cvSave("mx2.xml",mx2);
//            cvSave("my2.xml",my2);
//
//        }
////OR ELSE HARTLEY'S METHOD
//        else if( useUncalibrated == 1 || useUncalibrated == 2 )
//     // use intrinsic parameters of each camera, but
//     // compute the rectification transformation directly
//     // from the fundamental matrix
//        {
//            double H1[3][3], H2[3][3], iM[3][3];
//            CvMat _H1 = cvMat(3, 3, CV_64F, H1);
//            CvMat _H2 = cvMat(3, 3, CV_64F, H2);
//            CvMat _iM = cvMat(3, 3, CV_64F, iM);
//    //Just to show you could have independently used F
//            if( useUncalibrated == 2 )
//                cvFindFundamentalMat( &_imagePoints1,
//                &_imagePoints2, &_F);
//            cvStereoRectifyUncalibrated( &_imagePoints1,
//                &_imagePoints2, &_F,
//                imageSize,
//                &_H1, &_H2, 3);
//            cvInvert(&_M1, &_iM);
//            cvMatMul(&_H1, &_M1, &_R1);
//            cvMatMul(&_iM, &_R1, &_R1);
//            cvInvert(&_M2, &_iM);
//            cvMatMul(&_H2, &_M2, &_R2);
//            cvMatMul(&_iM, &_R2, &_R2);
//    //Precompute map for cvRemap()
//            cvInitUndistortRectifyMap(&_M1,&_D1,&_R1,&_M1,mx1,my1);
//
//            cvInitUndistortRectifyMap(&_M2,&_D1,&_R2,&_M2,mx2,my2);
//        }
//        else
//            assert(0);
//        cvNamedWindow( "rectified", 1 );
//// RECTIFY THE IMAGES AND FIND DISPARITY MAPS
//        if( !isVerticalStereo )
//            pair = cvCreateMat( imageSize.height, imageSize.width*2,
//            CV_8UC3 );
//        else
//            pair = cvCreateMat( imageSize.height*2, imageSize.width,
//            CV_8UC3 );
////Setup for finding stereo corrrespondences
//        CvStereoBMState *BMState = cvCreateStereoBMState();
//        assert(BMState != 0);
//        BMState->preFilterSize=41;
//        BMState->preFilterCap=31;
//        BMState->SADWindowSize=41;
//        BMState->minDisparity=-64;
//        BMState->numberOfDisparities=128;
//        BMState->textureThreshold=10;
//        BMState->uniquenessRatio=15;
//        for( i = 0; i < nframes; i++ )
//        {
//            IplImage* img1=cvLoadImage(imageNames[0][i].c_str(),0);
//            IplImage* img2=cvLoadImage(imageNames[1][i].c_str(),0);
//            if( img1 && img2 )
//            {
//                CvMat part;
//                cvRemap( img1, img1r, mx1, my1 );
//                cvRemap( img2, img2r, mx2, my2 );
//                if( !isVerticalStereo || useUncalibrated != 0 )
//                {
//              // When the stereo camera is oriented vertically,
//              // useUncalibrated==0 does not transpose the
//              // image, so the epipolar lines in the rectified
//              // images are vertical. Stereo correspondence
//              // function does not support such a case.
//                    cvFindStereoCorrespondenceBM( img1r, img2r, disp,
//                        BMState);
//                    cvNormalize( disp, vdisp, 0, 256, CV_MINMAX );
//                    cvNamedWindow( "disparity" );
//                    cvShowImage( "disparity", vdisp );
//                }
//                if( !isVerticalStereo )
//                {
//                    cvGetCols( pair, &part, 0, imageSize.width );
//                    cvCvtColor( img1r, &part, CV_GRAY2BGR );
//                    cvGetCols( pair, &part, imageSize.width,
//                        imageSize.width*2 );
//                    cvCvtColor( img2r, &part, CV_GRAY2BGR );
//                    for( j = 0; j < imageSize.height; j += 16 )
//                        cvLine( pair, cvPoint(0,j),
//                        cvPoint(imageSize.width*2,j),
//                        CV_RGB(0,255,0));
//                }
//                else
//                {
//                    cvGetRows( pair, &part, 0, imageSize.height );
//                    cvCvtColor( img1r, &part, CV_GRAY2BGR );
//                    cvGetRows( pair, &part, imageSize.height,
//                        imageSize.height*2 );
//                    cvCvtColor( img2r, &part, CV_GRAY2BGR );
//                    for( j = 0; j < imageSize.width; j += 16 )
//                        cvLine( pair, cvPoint(j,0),
//                        cvPoint(j,imageSize.height*2),
//                        CV_RGB(0,255,0));
//                }
//                cvShowImage( "rectified", pair );
//                if( cvWaitKey() == 27 )
//                    break;
//            }
//            cvReleaseImage( &img1 );
//            cvReleaseImage( &img2 );
//        }
//        cvReleaseStereoBMState(&BMState);
//        cvReleaseMat( &mx1 );
//        cvReleaseMat( &my1 );
//        cvReleaseMat( &mx2 );
//        cvReleaseMat( &my2 );
//        cvReleaseMat( &img1r );
//        cvReleaseMat( &img2r );
//        cvReleaseMat( &disp );
//    }
//}
//
//
//
//int main_kinect_calibration2(int argc, char* argv[]){
//	double _last_tick = 0;
//	int _frame_counter = 0;
//	float _frame_rate = 0;
//
//	bool result;
//
//	if(!NIKinect2::ni_initialize())
//		return -1;
//	
//	NIKinect2* kinect = new NIKinect2();
//	result = kinect->initialize();
//	if(result){
//		kinect->enable_depth_generator();
//		kinect->enable_color_generator();
//		//kinect->enable_user_generator();
//		kinect->set_depth_color_registration(true);
//	}
//
//	cv::Mat depth_kinect;
//	cv::Mat color_kinect,	color_camera;
//	cv::Mat gray_kinect ,	gray_camera;
//	cv::vector<cv::Point2f> point_kinect, point_camera;
//
//
//	cv::VideoCapture cap(0);
//	if (!cap.isOpened()){
//		printf("Error openning camera.\n");
//		getchar();
//		return -1;
//	}
//
//	//result = cap.set(CV_CAP_PROP_FRAME_WIDTH,320);
//	//result = cap.set(CV_CAP_PROP_FRAME_HEIGHT,240);
//	//result = cap.set(CV_CAP_PROP_FPS,30);
//	//
//	double dWidth = cap.get(CV_CAP_PROP_FRAME_WIDTH); //get the width of frames of the video
//	double dHeight = cap.get(CV_CAP_PROP_FRAME_HEIGHT); //get the height of frames of the video
//
//	//cv::Size boardSize(5,4);
//	cv::Size boardSize(9,6);
//
//	FILE* file = NULL;
//	int n_images = 0;
//
//	bool homography = false;
//	bool remap = false;
//	bool remap2 = false;
//
//	int time = 1;
//	char c = 0;
//	while(c != 27){
//		if(!cap.read(color_camera)) 
//			break;
//
//		if(!NIKinect2::ni_update() || !kinect->update())
//			break;
//		
//		if(!kinect->get_color(color_kinect) || !color_kinect.rows)
//			break;
//		if(c == '4')
//			remap = !remap;
//		if(c == '5'){
//			remap2 = !remap2;
//
//			mx1 = cv::Mat(_mx1);
//			my1 = cv::Mat(_my1);
//			mx2 = cv::Mat(_mx2);
//			my2 = cv::Mat(_my2);
//		}
//
//		if(c == 'c')
//			chess = !chess;
//		if(c == 'r'){
//			record = !record;
//			//chess = false;
//
//			if(record){
//				file = fopen("Calib\\images.txt","w+");
//				n_images = 0;
//			}
//			else{
//				if(file)
//					fclose(file);
//			}
//		}
//		if(c == 'h')
//			homography = !homography;
//		
//		//if(c == ' '){
//		//	cv::imwrite("Kinect.png",color_kinect);
//		//	cv::imwrite("Camera.png",color_camera);
//		//}
//
//		if(c == 'l'){
//			FILE* file_to_read = fopen("Calib\\images.txt","r");
//			//std::vector<cv::Mat> images_kinect;
//			//std::vector<cv::Mat> images_camera;
//			std::vector<std::string> images_paths;
//			char buff[128];
//
//			while(fgets(buff,128,file_to_read)){
//				buff[strlen(buff)-1] = '\0';
//				//images_kinect.push_back(cv::imread(buff));
//				images_paths.push_back(std::string(buff));
//
//				fgets(buff,128,file_to_read);
//				buff[strlen(buff)-1] = '\0';
//				//images_camera.push_back(cv::imread(buff));
//				images_paths.push_back(std::string(buff));
//			}
//
//			StereoCalib(images_paths, boardSize, true, true);
//
//
//		}
//
//		if(c == 'z'){
//			StereoCalib2("Calib\\images.txt",9,6,0,4.5);
//		}
//		
//
//		if(chess){
//			bool _chess_on_camera = findChessboardCorners( color_camera, boardSize, point_camera,CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);
//			bool _chess_on_kinect = findChessboardCorners( color_kinect, boardSize, point_kinect,CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);
//			if(_chess_on_camera	&& _chess_on_kinect){
//				
//				if(c == ' '){
//					char buff_kinect[128];
//					char buff_camera[128];
//
//					sprintf(buff_kinect,"Calib\\kinect_%d.png",n_images);
//					sprintf(buff_camera,"Calib\\camera_%d.png",n_images);
//
//					cv::imwrite(buff_kinect,color_kinect);
//					cv::imwrite(buff_camera,color_camera);
//
//					puts(buff_kinect);
//					puts(buff_camera);
//
//					fprintf(file,"%s\n%s\n",buff_kinect,buff_camera);
//
//					n_images++;
//
//
//				}
//
//				if(homography){
//					cv::cvtColor(color_kinect,gray_kinect,CV_BGR2GRAY);
//					cv::cvtColor(color_camera,gray_camera,CV_BGR2GRAY);
//
//					cv::cornerSubPix( gray_kinect, point_kinect, cv::Size(11,11), cv::Size(-1,-1), cv::TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.01 ));
//					cv::cornerSubPix( gray_camera, point_camera, cv::Size(11,11), cv::Size(-1,-1), cv::TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.01 ));
//					
//					{
//						cv::circle(color_kinect,cv::Point(point_kinect[0].x,point_kinect[0].y),9,cv::Scalar(0,0,255),-1);
//						cv::circle(color_kinect,cv::Point(point_kinect[boardSize.width-1].x,point_kinect[boardSize.width-1].y),9,cv::Scalar(0,0,255),-1);
//						cv::circle(color_kinect,cv::Point(point_kinect[boardSize.width*(boardSize.height-1)].x,point_kinect[boardSize.width*(boardSize.height-1)].y),9,cv::Scalar(0,0,255),-1);
//						cv::circle(color_kinect,cv::Point(point_kinect[boardSize.width*boardSize.height -1].x,point_kinect[boardSize.width*boardSize.height -1].y),9,cv::Scalar(0,0,255),-1);
//					
//						cv::circle(color_camera,cv::Point(point_camera[0].x,point_camera[0].y),9,cv::Scalar(0,0,255),-1);
//						cv::circle(color_camera,cv::Point(point_camera[boardSize.width-1].x,point_camera[boardSize.width-1].y),9,cv::Scalar(0,0,255),-1);
//						cv::circle(color_camera,cv::Point(point_camera[boardSize.width*(boardSize.height-1)].x,point_camera[boardSize.width*(boardSize.height-1)].y),9,cv::Scalar(0,0,255),-1);
//						cv::circle(color_camera,cv::Point(point_camera[boardSize.width*boardSize.height -1].x,point_camera[boardSize.width*boardSize.height -1].y),9,cv::Scalar(0,0,255),-1);
//					}
//
//					cv::Point2f points_src[20];
//					cv::Point2f points_top[20];
//					//int ac = 0;
//					//for(int i = 0 ; i < 20 ; i+=4, ac++){
//					//	points_src[ac].x = point_kinect[i].x;	points_src[ac].y = point_kinect[i].y;
//					//	points_top[ac].x = point_camera[i].x;	points_top[ac].y = point_camera[i].y;
//					//}
//
//					points_src[0].x = point_kinect[0].x;	points_src[0].y = point_kinect[0].y;
//					points_src[1].x = point_kinect[boardSize.width-1].x;	points_src[1].y = point_kinect[boardSize.width-1].y;
//					points_src[2].x = point_kinect[boardSize.width*(boardSize.height-1)].x;	points_src[2].y = point_kinect[boardSize.width*(boardSize.height-1)].y;
//					points_src[3].x = point_kinect[boardSize.width*boardSize.height -1].x;	points_src[3].y = point_kinect[boardSize.width*boardSize.height -1].y;
//
//					points_top[0].x = point_camera[0].x;	points_top[0].y = point_camera[0].y;
//					points_top[1].x = point_camera[boardSize.width-1].x;	points_top[1].y = point_camera[boardSize.width-1].y;
//					points_top[2].x = point_camera[boardSize.width*(boardSize.height-1)].x;	points_top[2].y = point_camera[boardSize.width*(boardSize.height-1)].y;
//					points_top[3].x = point_camera[boardSize.width*boardSize.height -1].x;	points_top[3].y = point_camera[boardSize.width*boardSize.height -1].y;
//
//					cv::Mat trans;
//					cv::Mat _mat_perpective;
//
//					trans = cv::getPerspectiveTransform(points_top,points_src);
//
//					cv::warpPerspective(color_camera,_mat_perpective,trans,cv::Size(color_kinect.cols,color_kinect.rows));
//
//					cv::imshow("Coiso",(_mat_perpective*0.5) + (color_kinect*0.5));
//				}
//
//				cv::drawChessboardCorners( color_kinect, boardSize, cv::Mat(point_kinect), true );
//				cv::drawChessboardCorners( color_camera, boardSize, cv::Mat(point_camera), true );
//			}
//			else{
//				if(_chess_on_camera)
//					cv::drawChessboardCorners( color_camera, boardSize, cv::Mat(point_camera), true );
//				else
//					if(_chess_on_kinect)
//						cv::drawChessboardCorners( color_camera, boardSize, cv::Mat(point_camera), true );
//			}
//		}
//
//		if(remap){
//			//imageSize.width = 320; imageSize.height =240;
//
//			cv::initUndistortRectifyMap(cameraMatrix[0],
//										//getOptimalNewCameraMatrix(cameraMatrix[0], distCoeffs[0], imageSize, 1.),
//										distCoeffs[0], R1, P1, imageSize, CV_16SC2, rmap[0][0], rmap[0][1]);
//			cv::initUndistortRectifyMap(cameraMatrix[1],
//										//getOptimalNewCameraMatrix(cameraMatrix[1], distCoeffs[1], imageSize, 1.),
//										distCoeffs[1], R2, P2, imageSize, CV_16SC2, rmap[1][0], rmap[1][1]);
//
//			double sf;
//			int w, h;
//			
//				sf = 300./MAX(imageSize.width, imageSize.height);
//				w = cvRound(imageSize.width*sf);
//				h = cvRound(imageSize.height*sf);
//
//			cv::Rect vroi0(	cvRound(validRoi[0].x),		cvRound(validRoi[0].y),
//							cvRound(validRoi[0].width), cvRound(validRoi[0].height));
//			cv::Rect vroi1(	cvRound(validRoi[1].x),		cvRound(validRoi[1].y),
//							cvRound(validRoi[1].width), cvRound(validRoi[1].height));
//			cv::rectangle(color_camera,vroi0,cv::Scalar(255,0,0));
//			cv::rectangle(color_kinect,vroi1,cv::Scalar(255,0,0)); 
//
//			cv::Mat img0 = color_camera, img1 = color_kinect , rimg0, rimg1;
//			
//			//cv::rectangle(img,vroi,cv::Scalar(255,0,0));
//			cv::remap(img0/*(vroi0)*/, rimg0, rmap[1][0], rmap[1][1], CV_INTER_LINEAR);
//			cv::remap(img1/*(vroi1)*/, rimg1, rmap[0][0], rmap[0][1], CV_INTER_LINEAR);
//			
//			//cv::Mat rimg2,rimg3,rimg4;
//			//cv::resize(rimg1,rimg2,cv::Size(640*1.43,480*1.43));
//			//rimg2(cv::Rect(0,0,640,480)).copyTo(rimg3);
//
//			cv::imshow("RIMG",(rimg0*0.5) + (rimg1*0.5));
//			cv::imshow("RIMG0",rimg0);
//			cv::imshow("RIMG1",rimg1);
//            //cv::cvtColor(rimg, cimg, CV_GRAY2BGR);
//            //cv::Mat canvasPart = canvas(cv::Rect(0, h*1, w, h));
//            //cv::resize(rimg, canvasPart, canvasPart.size(), 0, 0, CV_INTER_AREA);
//            ////if( useCalibrated )
//            ////{
//            //    cv::Rect vroi(cvRound(validRoi[1].x*sf), cvRound(validRoi[1].y*sf),
//            //              cvRound(validRoi[1].width*sf), cvRound(validRoi[1].height*sf));
//            //    cv::rectangle(canvasPart, vroi, Scalar(0,0,255), 3, 8);
//            //}
//		}
//
//		if(remap2){
//			cv::Mat img0 = color_camera, img1 = color_kinect , rimg0, rimg1;
//			
//			//cv::rectangle(img,vroi,cv::Scalar(255,0,0));
//			cv::remap(img0/*(vroi0)*/, rimg0, mx1, my1, CV_INTER_LINEAR);
//			cv::remap(img1/*(vroi1)*/, rimg1, mx2, my2, CV_INTER_LINEAR);
//
//			cv::imshow("RIMG",(rimg0*0.5) + (rimg1*0.5));
//			cv::imshow("RIMG0",rimg0);
//			cv::imshow("RIMG1",rimg1);
//		}
//		
//		//if(record){
//		//	if( findChessboardCorners( color_kinect, boardSize, point_kinect,CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE) &&
//		//		findChessboardCorners( color_camera, boardSize, point_camera,CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE)) {
//		//		
//		//		char buff_kinect[128];
//		//		char buff_camera[128];
//
//		//		sprintf(buff_kinect,"Calib2\\kinect_%d.png",n_images);
//		//		sprintf(buff_camera,"Calib2\\camera_%d.png",n_images);
//
//		//		cv::imwrite(buff_kinect,color_kinect);
//		//		cv::imwrite(buff_camera,color_camera);
//
//		//		puts(buff_kinect);
//		//		puts(buff_camera);
//
//		//		fprintf(file,"%s\n%s\n",buff_kinect,buff_camera);
//
//		//		n_images++;
//		//	}
//		//}
//
//		 
//		//if(found_kinect){
//		//	cv::cornerSubPix( gray_kinect, point_kinect, cv::Size(11,11), cv::Size(-1,-1), cv::TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.01 ));
//		//	//cv::drawChessboardCorners( color_kinect, boardSize, cv::Mat(point_kinect), true );
//		//	cv::circle(color_kinect,cv::Point(point_kinect[0].x,point_kinect[0].y),9,cv::Scalar(0,0,255),-1);
//		//	cv::circle(color_kinect,cv::Point(point_kinect[boardSize.width-1].x,point_kinect[boardSize.width-1].y),9,cv::Scalar(0,0,255),-1);
//		//	cv::circle(color_kinect,cv::Point(point_kinect[boardSize.width*(boardSize.height-1)].x,point_kinect[boardSize.width*(boardSize.height-1)].y),9,cv::Scalar(0,0,255),-1);
//		//	cv::circle(color_kinect,cv::Point(point_kinect[boardSize.width*boardSize.height -1].x,point_kinect[boardSize.width*boardSize.height -1].y),9,cv::Scalar(0,0,255),-1);
//		//}
//
//		//if(found_camera){
//		//	cv::cornerSubPix( gray_camera, point_camera, cv::Size(11,11), cv::Size(-1,-1), cv::TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.01 ));
//		//	//cv::drawChessboardCorners( color_camera, boardSize, cv::Mat(point_camera), true );
//		//	cv::circle(color_camera,cv::Point(point_camera[0].x,point_camera[0].y),9,cv::Scalar(0,0,255),-1);
//		//	cv::circle(color_camera,cv::Point(point_camera[boardSize.width-1].x,point_camera[boardSize.width-1].y),9,cv::Scalar(0,0,255),-1);
//		//	cv::circle(color_camera,cv::Point(point_camera[boardSize.width*(boardSize.height-1)].x,point_camera[boardSize.width*(boardSize.height-1)].y),9,cv::Scalar(0,0,255),-1);
//		//	cv::circle(color_camera,cv::Point(point_camera[boardSize.width*boardSize.height -1].x,point_camera[boardSize.width*boardSize.height -1].y),9,cv::Scalar(0,0,255),-1);
//		//}
//
//		cv::imshow("color_camera",color_camera);
//		cv::imshow("color_kinect",color_kinect);
//		
//
//		++_frame_counter;
//		if (_frame_counter == 15)
//		{
//			double current_tick = cv::getTickCount();
//			_frame_rate = _frame_counter / ((current_tick - _last_tick)/cv::getTickFrequency());
//			_last_tick = current_tick;
//			_frame_counter = 0;
//			printf("%.2f\n",_frame_rate);
//		}
//
//		c = cv::waitKey(time);
//	}
//
//	exit(0);
//}