#ifndef __GET_INFERENCE_IMAGE_H
#define __GET_INFERENCE_IMAGE_H

#include <iostream>
#include "api/RpcLibClientBase.hpp"
#include <fstream>
#include <opencv2/opencv.hpp>
#include"cnpy.h"
#include <time.h>

class GetInferenceImage{
    private:
        //client
        msr::airlib::RpcLibClientBase _client;
		/*state*/
		msr::airlib::Pose _pose;
		msr::airlib::ImuBase::Output _imu;
        /*list*/
		std::vector<std::string> _list_camera;
		std::vector<msr::airlib::WorldSimApiBase::WeatherParameter> _list_weather;

        //Save Parameters
        const bool _save_data = false;
        const int _num_sampling = 300;
        const std::string _save_data_top_path = "/media/amsl/96fde31e-3b9b-4160-8d8a-a4b913579ca21/airsim_dataset_kawai/AirSimNH/verify_data5/";
        const std::string _save_data_csv_name = "image_attitude.csv";
        std::ofstream _csvfile;

        std::vector< std::vector<float> > place_list;

        //Parameters
        const bool _random_weather = false;
        const int _wait_time_millisec = 200;

        /*parameter-pose*/
		const float _x_range = 600.0;	//Neighborhood: 200, SoccerField: 350
		const float _y_range = 600.0;	//Neighborhood: 200, SoccerField: 300
		const float _z_min = -1.6; //Usualy -1.6 Building99 -2.5
		const float _z_max = -1.4; //Usually -1.4 Builging99 1.0

        const float _roll_max = M_PI;
        const float _roll_min = -1.0 * M_PI;

        const float _pitch_max = M_PI;
        const float _pitch_min = -1.0 * M_PI;

        const float _yaw_max = M_PI;
        const float _yaw_min = -1.0 * M_PI;

        const float resolution = M_PI/180.0 * 1.0;

        std::vector<cv::Mat> image_array;
        float tmp_roll = 0.0;
        float tmp_pitch = 0.0;
        float tmp_yaw = 0.0;

        //csv place data
        std::string place_csv_root_path = "/home/amsl/cpp/kawai_airsim_controller/place_data/AirSimNH/";
        std::string place_csv_name = "random_place.csv";
        std::vector< std::vector<std::string> > csv_data;
        size_t csv_data_size;

    public:
        GetInferenceImage();
        void client_initialization();
        void csv_initialization();
        void update_state();
        void leave_param_note();
        bool load_csv();
        std::vector<std::string> split(std::string& input, char delimiter);

        //Loop Process Function
        void spin();
        void random_weather();
        void random_place();
        void random_place_from_csv();
        int random_int(int min, int max);
        float check_deg(float deg);
        std::vector<float> string_to_float(std::vector<std::string> tmp_place_csv_data);
        void eular_to_quat(float r, float p, float y, Eigen::Quaternionf& q);
        int image_checker();
        void save_place();

        ~GetInferenceImage();

};

#endif