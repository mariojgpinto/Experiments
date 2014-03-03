#include "Sample2.h"

#include <Windows.h>
#include <Ole2.h>

#include <gl/GL.h>
#include <gl/GLU.h>
#include <gl/glut.h>

#include <NuiApi.h>
#include <NuiImageCamera.h>
#include <NuiSensor.h>

#include <opencv2\opencv.hpp>

#pragma once
const int width = 640;
const int height = 480;

bool init(int argc, char* argv[]);
void draw();
void execute();

void drawKinectData();

// OpenGL Variables
GLuint textureId;
GLubyte data[width*height*4];
UINT16 data_cv[width*height];

// Kinect variables
HANDLE depthStream;
INuiSensor* sensor;

bool initKinect() {
    // Get a working kinect sensor
    int numSensors;
    if (NuiGetSensorCount(&numSensors) < 0 || numSensors < 1) return false;
    if (NuiCreateSensorByIndex(0, &sensor) < 0) return false;

    // Initialize sensor
    sensor->NuiInitialize(NUI_INITIALIZE_FLAG_USES_DEPTH | NUI_INITIALIZE_FLAG_USES_COLOR);
    sensor->NuiImageStreamOpen(	NUI_IMAGE_TYPE_DEPTH,				// Depth camera or rgb camera?
								NUI_IMAGE_RESOLUTION_640x480,		// Image resolution
								0,//NUI_IMAGE_STREAM_FLAG_ENABLE_NEAR_MODE,       // Image stream flags, e.g. near mode
        2,        // Number of frames to buffer
        NULL,   // Event handle
        &depthStream);

	//sensor->NuiSetDepthFilter(
    return sensor;
}

void getKinectData(GLubyte* dest) {
    NUI_IMAGE_FRAME imageFrame;
    NUI_LOCKED_RECT LockedRect;
    if (sensor->NuiImageStreamGetNextFrame(depthStream, 0, &imageFrame) < 0) return;
    INuiFrameTexture* texture = imageFrame.pFrameTexture;
    texture->LockRect(0, &LockedRect, NULL, 0);
    if (LockedRect.Pitch != 0)
    {
        const USHORT* curr = (const USHORT*) LockedRect.pBits;
        const USHORT* dataEnd = curr + (width*height);

        while (curr < dataEnd) {
            // Get depth in millimeters
            USHORT depth = NuiDepthPixelToDepth(*curr++);

            // Draw a grayscale image of the depth:
            // B,G,R are all set to depth%256, alpha set to 1.
            for (int i = 0; i < 3; ++i)
                *dest++ = (BYTE) depth%256;
            *dest++ = 0xff;
        }
    }
    texture->UnlockRect(0);
    sensor->NuiImageStreamReleaseFrame(depthStream, &imageFrame);
}

void getKinectDataMat(UINT16* dest) {
    NUI_IMAGE_FRAME imageFrame;
    NUI_LOCKED_RECT LockedRect;
    if (sensor->NuiImageStreamGetNextFrame(depthStream, 0, &imageFrame) < 0) return;
    INuiFrameTexture* texture = imageFrame.pFrameTexture;
    texture->LockRect(0, &LockedRect, NULL, 0);
    if (LockedRect.Pitch != 0)
    {
        const USHORT* curr = (const USHORT*) LockedRect.pBits;
        const USHORT* dataEnd = curr + (width*height);

        while (curr < dataEnd) {
            // Get depth in millimeters
            USHORT depth = NuiDepthPixelToDepth(*curr++);

            // Draw a grayscale image of the depth:
            // B,G,R are all set to depth%256, alpha set to 1.
                *dest++ = depth;//(BYTE) depth%256;
        }
    }
    texture->UnlockRect(0);
    sensor->NuiImageStreamReleaseFrame(depthStream, &imageFrame);
}

void drawKinectData() {
    //glBindTexture(GL_TEXTURE_2D, textureId);
    //getKinectData(data);

	getKinectDataMat(data_cv);
	cv::Mat depth(480,640,CV_16UC1,(void*)data_cv);
	cv::Mat depth8;
	depth.convertTo(depth8, CV_8UC1,0.05);

	cv::imshow("Depth",depth8);

    //glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, width, height, GL_BGRA_EXT, GL_UNSIGNED_BYTE, (GLvoid*)data);
    //glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    //glBegin(GL_QUADS);
    //    glTexCoord2f(0.0f, 0.0f);
    //    glVertex3f(0, 0, 0);
    //    glTexCoord2f(1.0f, 0.0f);
    //    glVertex3f(width, 0, 0);
    //    glTexCoord2f(1.0f, 1.0f);
    //    glVertex3f(width, height, 0.0f);
    //    glTexCoord2f(0.0f, 1.0f);
    //    glVertex3f(0, height, 0.0f);
    //glEnd();
}

void draw() {
   drawKinectData();
   glutSwapBuffers();
}

void execute() {
    glutMainLoop();
}

bool init(int argc, char* argv[]) {
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGBA);
    glutInitWindowSize(width,height);
    glutCreateWindow("Kinect SDK Tutorial");
    glutDisplayFunc(draw);
    glutIdleFunc(draw);
    return true;
}

int main_sample2(int argc, char* argv[]) {
    if (!init(argc, argv)) return 1;
    if (!initKinect()) return 1;

    // Initialize textures
    glGenTextures(1, &textureId);
    glBindTexture(GL_TEXTURE_2D, textureId);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, width, height, 0, GL_BGRA_EXT, GL_UNSIGNED_BYTE, (GLvoid*) data);
    glBindTexture(GL_TEXTURE_2D, 0);

    // OpenGL setup
    glClearColor(0,0,0,0);
    glClearDepth(1.0f);
    glEnable(GL_TEXTURE_2D);

    // Camera setup
    glViewport(0, 0, width, height);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(0, width, height, 0, 1, -1);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    // Main loop
    execute();
    return 0;
}