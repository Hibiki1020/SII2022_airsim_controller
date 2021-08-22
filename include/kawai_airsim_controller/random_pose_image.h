#ifndef __RANDOM_POSE_IMAGE_H
#define __RANDOM_POSE_IMAGE_H

#include <iostream>
#include "api/RpcLibClientBase.hpp"
#include <fstream>
#include <opencv2/opencv.hpp>
#include"cnpy.h"
#include <time.h>

class ImageDroneRandomPose{

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
        const int _num_sampling = 150;
        const std::string _save_data_top_path = "/home/amsl/cpp/kawai_airsim_controller/place_data/Africa/";
        const std::string _save_data_csv_name = "random_place.csv";
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

        const float _roll_max = M_PI/180.0 * 40.0;
        const float _roll_min = -1.0 * M_PI/180.0 * 40.0;

        const float _pitch_max = M_PI/180.0 * 40.0;
        const float _pitch_min = -1.0 * M_PI/180.0 * 40.0;

        const float _yaw_max = M_PI/180.0 * 179.9;
        const float _yaw_min = -1.0 * M_PI/180.0 * 179.9;

        const float resolution = M_PI/180.0 * 1.0;
    
    public:
        //Initialization function
        ImageDroneRandomPose();
        void client_initialization();
        void update_state();
        void leave_param_note();
        void csv_initialization();

        //Loop Process Function
        void spin();
        void random_weather();
        void random_place();
        void eular_to_quat(float r, float p, float y, Eigen::Quaternionf& q);
        int image_checker();
        void save_place();


        //Destracta
        ~ImageDroneRandomPose();

};

#endif