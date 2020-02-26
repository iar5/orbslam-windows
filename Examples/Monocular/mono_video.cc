/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#include<string>
#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<thread>

#include<opencv2/core/core.hpp>
#include<System.h>
#include<time.h>
#include "Converter.h"

using namespace std;


int main(int argc, char **argv)
{
	if (argc != 4 && argc != 5){
		cerr << endl << "Usage: mono_video.exe video_path vocabulary_path settings_path (map_path)" << endl;
		return 1;
	}

	string videoPath = argv[1];
	string vocabPath = argv[2];
	string settingsPath = argv[3];
	string mapPath = (argc == 5 ? argv[4] : "");

	ORB_SLAM2::System SLAM(vocabPath, settingsPath, ORB_SLAM2::System::MONOCULAR, true, mapPath);

	cout << endl << "-------" << endl;
	cout << "Start processing sequence ..." << endl;


	// WebCam or other videosource 
	cv::VideoCapture* cap = (videoPath == "0") ? new cv::VideoCapture(0) : new cv::VideoCapture(videoPath);
	if (!cap->isOpened()) {
		cout << "Error opening video stream or file" << endl;
		return -1;
	}

	cv::Mat im;
	cv::Mat Tcw;


	while (true)
	{
		cap->read(im); 

		if(im.empty())
			continue;

		// Pass the image and timestamp to the SLAM system
		// From http://stackoverflow.com/questions/19555121/how-to-get-current-timestamp-in-milliseconds-since-1970-just-the-way-java-gets
		__int64 curNow = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
		Tcw = SLAM.TrackMonocular(im, curNow / 1000.0);

		cv::imshow("Image", im);

		if (cv::waitKey(1) >= 0)
			break;
	}

	SLAM.Shutdown();
	cap->release();

	return 0;
}



