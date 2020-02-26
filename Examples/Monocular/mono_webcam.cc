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

#include <boost/beast/core.hpp>
#include <boost/beast/http.hpp>
#include <boost/beast/version.hpp>
#include <boost/asio/connect.hpp>
#include <boost/asio/ip/tcp.hpp>

#include<string>
#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<thread>
#include<cmath>

#include<opencv2/core/core.hpp>
#include<System.h>
#include<time.h>
#include "Converter.h"


using namespace std;

static cv::Mat frame;
static std::mutex m;

namespace http = boost::beast::http;
using tcp = boost::asio::ip::tcp;

boost::asio::io_context ioc;

string host;
string port;
string positionTarget;
string videoTarget;
string savePath;
bool saveStreamVideo;

__int64 latestTimestamp;



static __int64 getTimestamp() {
	return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
}

static void testServer(tcp::socket &socket, tcp::resolver::results_type results)
{
	// https://www.boost.org/doc/libs/1_66_0/libs/beast/example/http/client/sync/http_client_sync.cpp
	boost::asio::connect(socket, results.begin(), results.end()); // Make the connection on the IP address we get from a lookup (bei jeder Request neu)		
	http::request<http::string_body> req;
	req.method(http::verb::get);
	req.target("/");
	req.set(http::field::version, 11); // http version 1.1
	req.set(http::field::host, host);
	req.set(http::field::user_agent, BOOST_BEAST_VERSION_STRING);
	req.set(http::field::content_type, "text/plain");
	http::write(socket, req);

	boost::beast::flat_buffer buffer;
	http::response<http::dynamic_body> res;
	http::read(socket, buffer, res);
	// cout << res << endl;

	boost::system::error_code ec;
	socket.shutdown(tcp::socket::shutdown_both, ec);
	if (ec && ec != boost::system::errc::not_connected) {
		throw boost::system::system_error{ ec };
	}
}

static void sendPosition(tcp::socket &socket, tcp::resolver::results_type results, cv::Mat Tcw)
{
	std::stringstream positionString;

	if (!Tcw.empty())
	{
		// TUM Format https://vision.in.tum.de/data/datasets/rgbd-dataset/file_formats
		// Rotationsmatrix https://stackoverflow.com/questions/15022630/how-to-calculate-the-angle-from-rotation-matrix
		// Euler https://en.wikipedia.org/wiki/Euler_angles
		// NED zu ENU Koordinaten https://en.wikipedia.org/wiki/Local_tangent_plane_coordinates
		// x = z
		// y = -x
		// z = -y

		cv::Mat Rwc = Tcw.rowRange(0, 3).colRange(0, 3).t(); // Rotation 
		cv::Mat twc = -Rwc*Tcw.rowRange(0, 3).col(3); // Translation
		vector<float> q = ORB_SLAM2::Converter::toQuaternion(Rwc);

		positionString << twc.at<float>(0) << " " << twc.at<float>(1) << " " << twc.at<float>(2);
		positionString << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3];

		//Eigen::Matrix<double, 3, 3> m = ORB_SLAM2::Converter::toMatrix3d(Rwc);
		//Eigen::Quaterniond q(m);
		//Eigen::Vector3d euler = q.toRotationMatrix().eulerAngles(2, 1, 0);
		//positionString << euler[0] << " " << euler[1] << " " << euler[2];
		//cout << positionString.str() << endl;
	}

	boost::asio::connect(socket, results.begin(), results.end()); 
	http::request<http::string_body> req;
	req.method(http::verb::put);
	req.target(positionTarget);
	req.set(http::field::version, 11); 
	req.set(http::field::host, host);
	req.set(http::field::user_agent, BOOST_BEAST_VERSION_STRING);
	req.set(http::field::content_type, "text/plain");
	req.body() = positionString.str();
	req.prepare_payload();
	http::write(socket, req);
}

static void sendImage(tcp::socket &socket, tcp::resolver::results_type results, cv::Mat im)
{
	vector<uchar> buf;
	cv::imencode(".jpg", im, buf);
	std::string content(buf.begin(), buf.end()); 

	boost::asio::connect(socket, results.begin(), results.end());
	http::request<http::string_body> req;
	req.method(http::verb::post);
	req.target(videoTarget);
	req.set(http::field::version, 11);
	req.set(http::field::host, host);
	req.set(http::field::user_agent, BOOST_BEAST_VERSION_STRING);
	req.set(http::field::content_type, "text/plain");
	req.body() = content;
	req.prepare_payload();
	http::write(socket, req);
}


static void processor(ORB_SLAM2::System *SLAM)
{
	cout << "Sending orbslam data to server " << host << endl;
	cout << "Testing server availability: " << flush;

	tcp::resolver resolver{ ioc };
	tcp::socket socket{ ioc };
	tcp::resolver::results_type results = resolver.resolve(host, port); // Look up the domain name -> Server muss laufen

	testServer(socket, results);
	cout << "test passed." << endl;
	cout << "Start processing sequence ..." << endl;

	cv::Mat im;

	while (true) 
	{
		m.lock();
		im = frame.clone();
		m.unlock();

		if (im.empty())	
			continue;
			
		__int64 timestamp = getTimestamp();

		cv::Mat Tcw = SLAM->TrackMonocular(im, timestamp / 1000.0);

		if (timestamp - latestTimestamp > 500)
		{
			sendPosition(socket, results, Tcw);
			
			sendImage(socket, results, im);
			latestTimestamp = timestamp;
		}

		// cv::imshow("Image", im);

		if (cv::waitKey(1) >= 0)
			break;		
	}
	cerr << "Shutting down video processing thread" << endl;
	SLAM->Shutdown();
}



static void producer(string videoPath)
{
	cv::VideoCapture cap;
	if (videoPath == "0")
		cap = cv::VideoCapture(0); // webcam
	else
		cap = cv::VideoCapture(videoPath); // ip stream, video

	if (!cap.isOpened()) 
		cout << "Error opening video stream or file" << endl;
	

	cout << "Start producing sequence ..." << endl;

	cv::Mat im;

	if (saveStreamVideo) {
		string fileName = savePath + "\\" + std::to_string(getTimestamp()) + ".avi";
		cv::VideoWriter video(fileName, CV_FOURCC('M', 'J', 'P', 'G'), 10, cv::Size(cap.get(CV_CAP_PROP_FRAME_WIDTH), cap.get(CV_CAP_PROP_FRAME_HEIGHT)), true);

		while (true){
			cap.read(im);

			m.lock();
			frame = im.clone();
			m.unlock();

			video.write(im);
		}
	}
	else {
		while (true){
			cap.read(im);

			m.lock();
			frame = im.clone();
			m.unlock();
		}
	}

	cerr << "Shutting down video capturing thread" << endl;
	cap.release();
}



int main(int argc, char **argv)
{
	if (argc != 4 && argc != 5) {
		cerr << endl << "Usage: mono_webcam.exe video_path vocabulary_path settings_path (map_path)" << endl;
		return 1;
	}

	string videoPath = argv[1];
	string vocabPath = argv[2];
	string settingsPath = argv[3];
	string mapPath = (argc == 5 ? argv[4] : "");

	cv::FileStorage fsSettings(settingsPath.c_str(), cv::FileStorage::READ);
	if (!fsSettings.isOpened()) {
		cerr << "Failed to open settings file at: " << settingsPath << endl;
		exit(-1);
	}

	host = (string)fsSettings["Server.host"];
	port = (string)fsSettings["Server.port"];
	positionTarget = (string)fsSettings["Server.positionTarget"];
	videoTarget = (string)fsSettings["Server.videoTarget"];

	if ((string)fsSettings["VideoStream.saveVideo"] == "true") {
		savePath = (string)fsSettings["VideoStream.savePath"];
		saveStreamVideo = true;
	} else {
		saveStreamVideo = false;
	}

	cout << endl << "Settingsfile read succesfully" << endl;

	ORB_SLAM2::System SLAM(vocabPath, settingsPath, ORB_SLAM2::System::MONOCULAR, true, mapPath);
	latestTimestamp = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

	cout << endl << "-------" << endl;

	std::thread producer_t(producer, videoPath);
	std::thread processor_t(processor, &SLAM);

	producer_t.join();

	cerr << "Shutting down programm" << endl;

	return 0;
}
