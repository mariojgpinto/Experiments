#ifndef _MAIN_GUI
#define _MAIN_GUI

#include "Viewer.h"

class __declspec(dllexport) MainGUI{
	public:
		MainGUI(int argc, char* argv[]);
		~MainGUI();

		void add_kinect(NIKinect* kinect);

		void run(int argc, char** argv);
		
		void update();
		void update_cycle();

		Viewer *viewer;

	private:
		QApplication *a;
		NIKinect *kinect;
		Controller *controller;
		Preferences *preferences;
		

		bool up;
};

int __declspec(dllexport) main_gui(int argc, char *argv[]);

#endif