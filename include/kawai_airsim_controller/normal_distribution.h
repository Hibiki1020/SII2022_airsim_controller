#ifndef __NORMAL_DISTRIBUTION_H
#define __NORMAL_DISTRIBUTION_H

#include <iostream>
#include <vector>
#include <string>
#include <random>
#include <unistd.h>
#include "api/RpcLibClientBase.hpp"
#include <fstream>
#include <opencv2/opencv.hpp>
#include"cnpy.h"
#include <time.h>

class GetDroneImage{
    private:
        //client
        msr::airlib::RpcLibClientBase _client;
		/*state*/
		msr::airlib::Pose _pose;
		msr::airlib::ImuBase::Output _imu;
        /*list*/
		std::vector<std::string> _list_camera;
		std::vector<msr::airlib::WorldSimApiBase::WeatherParameter> _list_weather;

        //config
        std::string place_csv_root_path = "/home/airsim_ws/kawai_airsim_controller/place_data/AirSimNH/";
        std::string place_csv_name = "random_place.csv";
        
        std::string image_top_path = "/home/ssd_dir/airsim_dataset_kawai/AirSimNH/normal_distribution_data2_image_490000/";
        std::string integrated_csv_name = "image_attitude.csv";
        std::string image_file_top_name = "image";
        std::string image_format = ".png";

        const int num_image = 70000;
        const int num_window = 7;
        const int size_image = 672;
        const int error_feature_limit = 10;
        const int feature_number_minimum = 120;

        //CSV data
        std::vector< std::vector<std::string> > csv_data;
        size_t csv_data_size = 0; //csvファイル内のデータ数
        int image_counter = 0;

        //Parameters
        const bool _random_weather = true;
        const int _wait_time_millisec = 50;

        /*parameter-pose*/
        const float _roll_max = M_PI/180.0 * 180.0;
        const float _roll_min = -1.0 * M_PI/180.0 * 180.0;

        const float _pitch_max = M_PI/180.0 * 180.0;
        const float _pitch_min = -1.0 * M_PI/180.0 * 180.0;

        const float _yaw_max = M_PI/180.0 * 10.0;
        const float _yaw_min = -1.0 * M_PI/180.0 * 10.0;

        const float resolution = M_PI/180.0 * 1.0;
    
    public:
        GetDroneImage();
        ~GetDroneImage();
        void client_initialization();
        void update_state();
        void leave_param_note();
        bool load_csv();
        std::vector<std::string> split(std::string& input, char delimiter);

        void spin();
        void save_image_data();
        int random_int(int min, int max);
        std::vector<float> string_to_float(std::vector<std::string> tmp_place_csv_data);
        float check_deg(float deg);
        void eular_to_quat(float r, float p, float y, Eigen::Quaternionf& q);
        cv::Mat get_mono_image();
        std::vector<cv::Mat> get_image_windows(cv::Mat original_mono_image);
        cv::Mat get_window(cv::Mat original_mono_image);
        bool check_window(cv::Mat window);
        void save_image_place(std::vector<cv::Mat> windows, float x, float y, float z, float roll, float pitch, float yaw);
};


#endif