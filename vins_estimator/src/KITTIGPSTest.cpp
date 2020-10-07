/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Qin Tong (qintonguav@gmail.com)
 *******************************************************/

#include <iostream>
#include <stdio.h>
#include <cmath>
#include <string>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include "estimator/estimator.h"
#include "utility/visualization.h"

using namespace std;
using namespace Eigen;

Estimator estimator;
ros::Publisher pubGPS;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "vins_estimator");
	ros::NodeHandle n("~");
	ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info); //编译时显示的是Info级别

	pubGPS = n.advertise<sensor_msgs::NavSatFix>("/gps", 1000); //创建一个Publisher，发布名为gps的topic，消息类型为sensor_msgs::NavSatGix,发布信息队列长度为1000

	if(argc != 3)
	{
		printf("please intput: rosrun vins kitti_gps_test [config file] [data folder] \n"
			   "for example: rosrun vins kitti_gps_test "
			   "~/catkin_ws/src/VINS-Fusion/config/kitti_raw/kitti_10_03_config.yaml "
			   "/media/tony-ws1/disk_D/kitti/2011_10_03/2011_10_03_drive_0027_sync/ \n");
		return 1;
	}

	string config_file = argv[1]; //配置文件
	printf("config_file: %s\n", argv[1]);
	string sequence = argv[2]; //数据序列
	printf("read sequence: %s\n", argv[2]);
	string dataPath = sequence + "/";

	//加载图像序列
	FILE* file;
	file = std::fopen((dataPath + "image_00/timestamps.txt").c_str() , "r"); //r: read  为输入操作打开文件，文件必须存在
	                                                                         //c_str()函数返回一个指向正规C字符串的指针  此处是返回指向dataPath +"image_00/timestamps.txt"的指针
																			//函数c_str()就是将C++的string转化为C的字符串数组
	if(file == NULL){
	    printf("cannot find file: %simage_00/timestamps.txt \n", dataPath.c_str());
	    ROS_BREAK();
	    return 0;          
	}
	vector<double> imageTimeList; //时间序列，单位为s
	int year, month, day;
	int hour, minute;
	double second;
	// fscanf从一个流中执行格式化输入,fscanf遇到空格和换行时结束，注意空格时也结束。
	//int fscanf(FILE *stream, char *format,[argument...]);   返回值：整型，数值等于[argument...]的个数
	//第一个参数：file文件对象的标识六的指针，第二个参数：输入的格式，第三个参数：用于接受输入变量的指针
	//KITTI里的格式： 2011-10-03 12:55:34.939038464
	while (fscanf(file, "%d-%d-%d %d:%d:%lf", &year, &month, &day, &hour, &minute, &second) != EOF)
	{
		//printf("%lf\n", second);
	    imageTimeList.push_back(hour * 60 * 60 + minute * 60 + second); //将秒分钟转化为秒
	}
	std::fclose(file); //关闭

	//加载GPS时间序列
	vector<double> GPSTimeList;
	{
		FILE* file;
		file = std::fopen((dataPath + "oxts/timestamps.txt").c_str() , "r");
		if(file == NULL){
		    printf("cannot find file: %soxts/timestamps.txt \n", dataPath.c_str());
		    ROS_BREAK();
		    return 0;          
		}
		int year, month, day;
		int hour, minute;
		double second;
		while (fscanf(file, "%d-%d-%d %d:%d:%lf", &year, &month, &day, &hour, &minute, &second) != EOF) //EOF实际是-1，用来表示文本文件的结束，代表文件没输入结束就循环。 
		{
			//printf("%lf\n", second);
		    GPSTimeList.push_back(hour * 60 * 60 + minute * 60 + second);
		}
		std::fclose(file);
	}

	readParameters(config_file);
	estimator.setParameter();
	registerPub(n);

	FILE* outFile;
	outFile = fopen((OUTPUT_FOLDER + "/vio.txt").c_str(),"w");
	if(outFile == NULL)
		printf("Output path dosen't exist: %s\n", OUTPUT_FOLDER.c_str());
	string leftImagePath, rightImagePath;
	cv::Mat imLeft, imRight;
	double baseTime;

	for (size_t i = 0; i < imageTimeList.size(); i++)
	{	
		if(ros::ok())
		{
			//图像跟GPS时间序列谁小就作为基础时间
			if(imageTimeList[0] < GPSTimeList[0])
				baseTime = imageTimeList[0];
			else
				baseTime = GPSTimeList[0];
			
			//printf("base time is %f\n", baseTime);
			printf("process image %d\n", (int)i);
			
			// stringstream：流的输入和输出操作
			//可以使用 str() 方法，将 stringstream 类型转换为 string 类型；
			//可以将多个字符串放入 stringstream 中，实现字符串的拼接目的；
			//如果想清空 stringstream，必须使用 sstream.str("")
			stringstream ss;
			ss << setfill('0') << setw(10) << i; //输出9个0和一个i共占10个位置

			/*加载图像数据*/
			leftImagePath = dataPath + "image_00/data/" + ss.str() + ".png";
			rightImagePath = dataPath + "image_01/data/" + ss.str() + ".png";
			printf("%s\n", leftImagePath.c_str() ); //函数c_str()就是将C++的string转化为C的字符串数组   输出字符串
			printf("%s\n", rightImagePath.c_str() );

			imLeft = cv::imread(leftImagePath, CV_LOAD_IMAGE_GRAYSCALE );
			imRight = cv::imread(rightImagePath, CV_LOAD_IMAGE_GRAYSCALE );

			double imgTime = imageTimeList[i] - baseTime;

			//加载GPS数据
			FILE* GPSFile;
			string GPSFilePath = dataPath + "oxts/data/" + ss.str() + ".txt";
			GPSFile = std::fopen(GPSFilePath.c_str() , "r");
			if(GPSFile == NULL){
			    printf("cannot find file: %s\n", GPSFilePath.c_str());
			    ROS_BREAK();
			    return 0;          
			}
			//pos_accuracy:  position accuracy (north/east in m)
			//vel_accuracy:  velocity accuracy (north/east in m/s)
			//navstat:       navigation status (see navstat_to_string)
			//numsats:       number of satellites tracked by primary GPS receiver
			//posmode:       position mode of primary GPS receiver (see gps_mode_to_string)
			//velmode:       velocity mode of primary GPS receiver (see gps_mode_to_string)
			//orimode:       orientation mode of primary GPS receiver (see gps_mode_to_string)
			double lat, lon, alt, roll, pitch, yaw;
			double vn, ve, vf, vl, vu;
			double ax, ay, az, af, al, au;
			double wx, wy, wz, wf, wl, wu;
			double pos_accuracy, vel_accuracy;
			double navstat, numsats;
			double velmode, orimode;
			
			fscanf(GPSFile, "%lf %lf %lf %lf %lf %lf ", &lat, &lon, &alt, &roll, &pitch, &yaw);
			//printf("lat:%lf lon:%lf alt:%lf roll:%lf pitch:%lf yaw:%lf \n",  lat, lon, alt, roll, pitch, yaw);
			fscanf(GPSFile, "%lf %lf %lf %lf %lf ", &vn, &ve, &vf, &vl, &vu);
			//printf("vn:%lf ve:%lf vf:%lf vl:%lf vu:%lf \n",  vn, ve, vf, vl, vu);
			fscanf(GPSFile, "%lf %lf %lf %lf %lf %lf ", &ax, &ay, &az, &af, &al, &au);
			//printf("ax:%lf ay:%lf az:%lf af:%lf al:%lf au:%lf\n",  ax, ay, az, af, al, au);
			fscanf(GPSFile, "%lf %lf %lf %lf %lf %lf ", &wx, &wy, &wz, &wf, &wl, &wu);
			//printf("wx:%lf wy:%lf wz:%lf wf:%lf wl:%lf wu:%lf\n",  wx, wy, wz, wf, wl, wu);
			fscanf(GPSFile, "%lf %lf %lf %lf %lf %lf ", &pos_accuracy, &vel_accuracy, &navstat, &numsats, &velmode, &orimode);
			//printf("pos_accuracy:%lf vel_accuracy:%lf navstat:%lf numsats:%lf velmode:%lf orimode:%lf\n", 
			//	    pos_accuracy, vel_accuracy, navstat, numsats, velmode, orimode);

			std::fclose(GPSFile);

			sensor_msgs::NavSatFix gps_position;
			gps_position.header.frame_id = "NED";
			gps_position.header.stamp = ros::Time(imgTime);
			gps_position.status.status = navstat;
			gps_position.status.service = numsats;
			gps_position.latitude  = lat;
			gps_position.longitude = lon;
			gps_position.altitude  = alt;
			gps_position.position_covariance[0] = pos_accuracy;
			//printf("pos_accuracy %f \n", pos_accuracy);
			pubGPS.publish(gps_position);

			estimator.inputImage(imgTime, imLeft, imRight);
			
			Eigen::Matrix<double, 4, 4> pose; //位姿矩阵
			estimator.getPoseInWorldFrame(pose);
			if(outFile != NULL)
				fprintf (outFile, "%f %f %f %f %f %f %f %f %f %f %f %f \n",pose(0,0), pose(0,1), pose(0,2),pose(0,3),
																	       pose(1,0), pose(1,1), pose(1,2),pose(1,3),
																	       pose(2,0), pose(2,1), pose(2,2),pose(2,3));
			
			// cv::imshow("leftImage", imLeft);
			// cv::imshow("rightImage", imRight);
			// cv::waitKey(2);
		}
		else
			break;
	}
	if(outFile != NULL)
		fclose (outFile);
	return 0;
}

