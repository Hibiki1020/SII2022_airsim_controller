#ifndef __DRONE_RANDOM_POSE_H
#define __DRONE_RANDOM_POSE_H

#include <iostream>
#include "api/RpcLibClientBase.hpp"
#include <fstream>
#include <opencv2/opencv.hpp>
#include"cnpy.h"

class DroneRandomPose{

    private:
        //client
        msr::airlib::RpcLibClientBase _client;
		/*state*/
		msr::airlib::Pose _pose;
		msr::airlib::ImuBase::Output _imu;
        /*list*/
		std::vector<std::string> _list_camera;
		std::vector<msr::airlib::WorldSimApiBase::WeatherParameter> _list_weather;
		/*csv*/
		std::ofstream _csvfile;
        /*parameter-save*/
		const bool _save_data = true;
		const int _num_sampling = 100;
		const std::string _save_root_path = "/home/airsim_ws/airsim_controller/save/tmp";
		std::string _save_csv_path = _save_root_path + "/imu.csv";
		/*parameter-condition*/
		const bool _lidar_is_available = false;
		const bool _randomize_whether = false;
		const int _wait_time_millisec = 200;
		/*parameter-pose*/
		const float _x_range = 200.0;	//Neighborhood: 200, SoccerField: 350
		const float _y_range = 200.0;	//Neighborhood: 200, SoccerField: 300
		const float _z_min = -3.0;
		const float _z_max = -2.0;
		const float _rp_range = M_PI/6.0;
		/*parameter-lidar*/
		const int _num_rings = 32;
		const int _points_per_ring = 1812;
		const double _fov_upper_deg = 15;
		const double _fov_lower_deg = -25;

    public:
        DroneRandomPose();
        ~DroneRandomPose();
        void client_initialization();
        void update_state();
        void leave_param_note();
        void csv_initialization();

        void start_sampling();
        void random_weather();
        void random_pose();
        bool save_data();
        bool save_images(std::vector<std::string>& list_save_colorimage_name, std::vector<std::string>& list_save_colorimage_path, std::vector<cv::Mat>& list_colorimage_cv);
        bool save_lidar_data(std::string& save_depthimg_name, std::string& save_depthimg_path, std::vector<double>& depthimage_mat);
        void eular_to_quat(float r, float p, float y, Eigen::Quaternionf& q);
};

#endif