#ifndef __GET_DRONE_IMAGE_H
#define __GET_DRONE_IMAGE_H

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
        std::string place_csv_root_path = "/home/amsl/cpp/kawai_airsim_controller/place_data/Building99/";
        std::string place_csv_name = "random_place.csv";
        
        std::string image_top_path = "/media/amsl/96fde31e-3b9b-4160-8d8a-a4b913579ca21/airsim_dataset_kawai/Building99/infer_image_range_30_30/dir1_infer_image1000/";
        std::string integrated_csv_name = "image_attitude.csv";
        std::string image_file_top_name = "image";
        std::string image_format = ".png";

        const int num_image = 1000;
        const int num_window = 1;
        const int size_image = 672;
        const int error_feature_limit = 10;
        const int feature_number_minimum = 120;
        bool get_window_checker = false;

        //CSV data
        std::vector< std::vector<std::string> > csv_data;
        size_t csv_data_size = 0; //csvファイル内のデータ数
        int image_counter = 0;
        bool save_color_checker = false;

        //Parameters
        const bool _random_weather = true;
        const int _wait_time_millisec = 20;

        /*parameter-pose*/
        const float _roll_max = M_PI/180.0 * 30.0;
        const float _roll_min = -1.0 * M_PI/180.0 * 30.0;

        const float _pitch_max = M_PI/180.0 * 30.0;
        const float _pitch_min = -1.0 * M_PI/180.0 * 30.0;

        const float _yaw_max = M_PI/180.0 * 40.0;
        const float _yaw_min = -1.0 * M_PI/180.0 * 40.0;

        const float resolution = M_PI/180.0 * 1.0;

        bool check_1_deg_increment = false;
    
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
        float convert_angle(float rad);
        void eular_to_quat(float r, float p, float y, Eigen::Quaternionf& q);
        cv::Mat get_image();
        std::vector<cv::Mat> get_image_windows(cv::Mat original_mono_image);
        cv::Mat get_window(cv::Mat original_mono_image);
        bool check_window(cv::Mat window);
        void save_image_place(std::vector<cv::Mat> windows, float x, float y, float z, float roll, float pitch, float yaw);
};


#endif