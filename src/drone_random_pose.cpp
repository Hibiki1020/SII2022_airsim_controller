#include "kawai_airsim_controller/drone_random_pose.h"

DroneRandomPose::DroneRandomPose(){
    std::cout << "Drone Random Pose" << std::endl;
    //client
    client_initialization();

    /*camera list*/
	_list_camera = {
		"camera_0"
	};
	/*
	_list_camera = {
		"camera_0",
		"camera_288",
		"camera_216",
		"camera_144",
		"camera_72"
	};
	*/
	/*csv*/
	if(_lidar_is_available)	_save_csv_path.insert(_save_csv_path.size() - std::string(".csv").size(), "_lidar");
	if(_list_camera.size() > 0)	_save_csv_path.insert(_save_csv_path.size() - std::string(".csv").size(), "_camera");
	if(_save_data){
		leave_param_note();
		csv_initialization();
	}

    std::cout << "Initial Setting Done" << std::endl;
}

DroneRandomPose::~DroneRandomPose(){
    std::cout << "End Drone Random Pose" << std::endl;
}

void DroneRandomPose::client_initialization(){
    //connect
    _client.confirmConnection(); //msr::airlib::RpcLibClientBase's function

    //reset
    std::cout << "Reset" << std::endl;
    _client.reset();

    //pose
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    update_state();

    /*weather*/
	if(_randomize_whether)	_client.simEnableWeather(true);
	_list_weather = {
		msr::airlib::WorldSimApiBase::WeatherParameter::Rain,
		msr::airlib::WorldSimApiBase::WeatherParameter::Roadwetness,
		msr::airlib::WorldSimApiBase::WeatherParameter::Snow,
		msr::airlib::WorldSimApiBase::WeatherParameter::RoadSnow,
		msr::airlib::WorldSimApiBase::WeatherParameter::MapleLeaf,
		msr::airlib::WorldSimApiBase::WeatherParameter::RoadLeaf,
		msr::airlib::WorldSimApiBase::WeatherParameter::Dust,
		msr::airlib::WorldSimApiBase::WeatherParameter::Fog
		// msr::airlib::WorldSimApiBase::WeatherParameter::Enabled
	};
	/*time*/
	// _client.simSetTimeOfDay(true, "2020-01-01 17:00:00", false, 0.01, 1.0, true);
}

void DroneRandomPose::update_state(){
    /*pose*/
	_pose = _client.simGetVehiclePose();
	std::cout << "Pose: " << std::endl;
	std::cout << " Position: "	//Eigen::Vector3f
		<< _pose.position.x() << ", "
		<< _pose.position.y() << ", "
		<< _pose.position.z() << std::endl;
	std::cout << " Orientation: "	//Eigen::Quaternionf
		<< _pose.orientation.w() << ", "
		<< _pose.orientation.x() << ", "
		<< _pose.orientation.y() << ", "
		<< _pose.orientation.z() << std::endl;
	/*imu*/
	_imu = _client.getImuData();
	std::cout << "IMU: " << std::endl;
	std::cout << " linear_acceleration: "	//Eigen::Vector3f
		<< _imu.linear_acceleration.x() << ", "
		<< _imu.linear_acceleration.y() << ", "
		<< _imu.linear_acceleration.z() << std::endl;
}

void DroneRandomPose::leave_param_note(){
    /*open*/
	std::ofstream txtfile;
	const std::string _save_txt_path = _save_root_path + "/param_note.txt";
	// txtfile.open(_save_txt_path, std::ios::out);
	txtfile.open(_save_txt_path, std::ios::app);
	if(!txtfile){
		std::cout << "Cannot open " << _save_txt_path << std::endl;
		exit(1);
	}
	/*write*/
	txtfile
		<< "----------" << std::endl
		<< "_randomize_whether" << ": " << (bool)_randomize_whether << std::endl
		<< "_num_sampling" << ": " << _num_sampling << std::endl
		<< "_x_range" << ": " << _x_range << std::endl
		<< "_y_range" << ": " << _y_range << std::endl
		<< "_z_min" << ": " << _z_min << std::endl
		<< "_z_max" << ": " << _z_max << std::endl
		<< "_rp_range" << ": " << _rp_range/M_PI*180.0 << std::endl;
	/*close*/
	txtfile.close();
}

void DroneRandomPose::csv_initialization(){
    /*check*/
	// std::ifstream ifs(_save_csv_path);
	// if(ifs.is_open()){
	//	std::cout << _save_csv_path << " already exists" << std::endl;
	//	exit(1);
	// }
	/*open*/
	_csvfile.open(_save_csv_path, std::ios::app);
	if(!_csvfile){
		std::cout << "Cannot open " << _save_csv_path << std::endl;
		exit(1);
	}
}

void DroneRandomPose::start_sampling(){
    std::cout << "Start Sampling" << std::endl;

    for(int i=0; i<_num_sampling; i++){
        std::cout << "--- sample " << i << " / " << _num_sampling << " ---" << std::endl;
        if( _randomize_whether ) random_weather();
        
        random_pose();
        
        _client.simPause(true);
        
        update_state();
        
        if(_save_data){
			if(save_data())	++i;
		}
		else{
            ++i;
        }

		_client.simPause(false);
    }
	_csvfile.close();
}

void DroneRandomPose::random_weather(void)
{
	/*reset*/
	for(size_t i=0; i<_list_weather.size(); ++i)	_client.simSetWeatherParameter(_list_weather[i], 0.0);
	/*random*/
	std::random_device rd;
	std::mt19937 mt(rd());
	std::uniform_int_distribution<> uid_index(0, _list_weather.size()-1);
	std::uniform_real_distribution<> urd_val(0.0, 1.0);
	for(int i=1; i<=uid_index(mt); ++i){
		int weather_index = uid_index(mt);
		double weather_val = urd_val(mt);
		_client.simSetWeatherParameter(_list_weather[weather_index], weather_val);
	}
}

void DroneRandomPose::random_pose(){
    /*random*/
	std::random_device rd;
	std::mt19937 mt(rd());
	std::uniform_real_distribution<> urd_x(-_x_range, _x_range);
	std::uniform_real_distribution<> urd_y(-_y_range, _y_range);
	std::uniform_real_distribution<> urd_z(_z_min, _z_max);
	std::uniform_real_distribution<> urd_roll_pitch(-_rp_range, _rp_range);
	std::uniform_real_distribution<> urd_yaw(-M_PI, M_PI);
	/*set pose*/
	Eigen::Vector3f position(urd_x(mt), urd_y(mt), urd_z(mt));
	float roll = urd_roll_pitch(mt);
	float pitch = urd_roll_pitch(mt);
	float yaw = urd_yaw(mt);
	Eigen::Quaternionf orientation;
	eular_to_quat(roll, pitch, yaw, orientation);
	msr::airlib::Pose goal = msr::airlib::Pose(position, orientation);
	std::cout << "Move to: " << std::endl
		<< " XYZ[m]: " 
			<< goal.position.x() << ", "
			<< goal.position.y() << ", "
			<< goal.position.z() << std::endl
		<< " RPY[deg]: "
			<< roll/M_PI*180.0 << ", "
			<< pitch/M_PI*180.0 << ", "
			<< yaw/M_PI*180.0 << std::endl
		<< " Quat: "
			<< goal.orientation.w() << ", "
			<< goal.orientation.x() << ", "
			<< goal.orientation.y() << ", "
			<< goal.orientation.z() << std::endl;
	/*teleport*/
	_client.simSetVehiclePose(goal, true);
	std::this_thread::sleep_for(std::chrono::milliseconds(_wait_time_millisec));
}

bool DroneRandomPose::save_data(){
    /*get image*/
	std::vector<std::string> list_save_colorimage_name(_list_camera.size());
	std::vector<std::string> list_save_colorimage_path(_list_camera.size());
	std::vector<cv::Mat> list_colorimage_cv(_list_camera.size());
	if(!_list_camera.empty()){
		if(!save_images(list_save_colorimage_name, list_save_colorimage_path, list_colorimage_cv))	return false;
	}
	/*get lidar*/
	std::string save_depthimg_name;
	std::string save_depthimg_path;
	std::vector<double> depthimage_mat(_num_rings*_points_per_ring, -1);
	if(_lidar_is_available){
		if(!save_lidar_data(save_depthimg_name, save_depthimg_path, depthimage_mat))	return false;
	}
	/*save*/
	for(size_t i=0; i<list_save_colorimage_path.size(); ++i){
		std::cout << "Saved: " << list_save_colorimage_path[i] << std::endl;
		cv::imwrite(list_save_colorimage_path[i], list_colorimage_cv[i]);
	}
	if(_lidar_is_available){
		cnpy::npy_save(save_depthimg_path, &depthimage_mat[0], {(long unsigned int)_num_rings, (long unsigned int)_points_per_ring}, "w");
		std::cout << "Saved: " << save_depthimg_path << std::endl;
	}
	/*imu (NEU) with other*/
	// std::cout << _imu.time_stamp << std::endl;
	_csvfile 
		<< _imu.linear_acceleration.x() << "," 
		<< -_imu.linear_acceleration.y() << "," 
		<< -_imu.linear_acceleration.z();
	if(_lidar_is_available)	_csvfile << "," << save_depthimg_name;
	for(size_t i=0; i<list_save_colorimage_name.size(); ++i){
		_csvfile << "," << list_save_colorimage_name[i];
	}
	_csvfile << std::endl;

	return true;
}

bool DroneRandomPose::save_images(std::vector<std::string>& list_save_colorimage_name, std::vector<std::string>& list_save_colorimage_path, std::vector<cv::Mat>& list_colorimage_cv)
{
	/*request-responce*/
	std::vector<msr::airlib::ImageCaptureBase::ImageRequest> list_request(_list_camera.size());
	for(size_t i=0; i<_list_camera.size(); ++i){
		list_request[i] = msr::airlib::ImageCaptureBase::ImageRequest(_list_camera[i], msr::airlib::ImageCaptureBase::ImageType::Scene, false, false);
	}
	std::vector<msr::airlib::ImageCaptureBase::ImageResponse> list_response = _client.simGetImages(list_request);
	/*access each image*/
	for(size_t i=0; i<list_response.size(); ++i){
		list_save_colorimage_name[i] = std::to_string(list_response[i].time_stamp) + "_" +  _list_camera[i] + ".jpg";
		std::string save_path = _save_root_path + "/" + list_save_colorimage_name[i];
		/*check-file*/
		std::ifstream ifs(save_path);
		if(ifs.is_open()){
			std::cout << save_path << " already exists" << std::endl;
			return false;
		}
		/*check-timestamp*/
		if((list_response[i].time_stamp - _imu.time_stamp)*1e-9 > _wait_time_millisec*1e-3){
			std::cout << "(list_response[i].time_stamp - _imu.time_stamp)*1e-9 = " << (list_response[i].time_stamp - _imu.time_stamp)*1e-9 << " > " << _wait_time_millisec*1e-3 << std::endl;
			return false;
		}
		/*std::vector -> cv::mat*/
		cv::Mat img_cv = cv::Mat(list_response[i].height, list_response[i].width, CV_8UC3);
		for(int row=0; row<list_response[i].height; ++row){
			for(int col=0; col<list_response[i].width; ++col){
				img_cv.at<cv::Vec3b>(row, col)[0] = list_response[i].image_data_uint8[3*row*list_response[i].width + 3*col + 0];
				img_cv.at<cv::Vec3b>(row, col)[1] = list_response[i].image_data_uint8[3*row*list_response[i].width + 3*col + 1];
				img_cv.at<cv::Vec3b>(row, col)[2] = list_response[i].image_data_uint8[3*row*list_response[i].width + 3*col + 2];
			}
		}
		list_save_colorimage_path[i] = save_path;
		list_colorimage_cv[i] = img_cv;
		// std::cout << "size: " << list_response[i].image_data_uint8.size() << std::endl;
		// std::cout << "height: " << list_response[i].height << std::endl;
		// std::cout << "width: " << list_response[i].width << std::endl;
	}
	return true;
}

bool DroneRandomPose::save_lidar_data(std::string& save_depthimg_name, std::string& save_depthimg_path, std::vector<double>& depthimage_mat)
{
	/*resolution*/
	double angle_h_resolution = (_fov_upper_deg - _fov_lower_deg)/180.0*M_PI/(double)(_num_rings - 1);
	double angle_w_resolution = 2*M_PI/(double)_points_per_ring;
	/*get*/
	msr::airlib::LidarData lidar_data = _client.getLidarData("");
	/*path*/
	save_depthimg_name = std::to_string(lidar_data.time_stamp) + ".npy";
	save_depthimg_path = _save_root_path + "/" + save_depthimg_name;
	/*check-file*/
	std::ifstream ifs(save_depthimg_path);
	if(ifs.is_open()){
		std::cout << save_depthimg_path << " already exists" << std::endl;
		return false;
	}
	/*check-timestamp*/
	if((lidar_data.time_stamp - _imu.time_stamp)*1e-9 > _wait_time_millisec*1e-3){
		std::cout << "(lidar_data.time_stamp - _imu.time_stamp)*1e-9 = " << (lidar_data.time_stamp - _imu.time_stamp)*1e-9 << " > " << _wait_time_millisec*1e-3 << std::endl;
		return false;
	}
	/*input*/
	for(size_t i=0; i<lidar_data.point_cloud.size(); i+=3){
		/*NED -> NEU*/
		double p_x = lidar_data.point_cloud[i];
		double p_y = -lidar_data.point_cloud[i+1];
		double p_z = -lidar_data.point_cloud[i+2];
		/*row*/
		double angle_h = atan2(p_z, sqrt(p_x*p_x + p_y*p_y));
		int row = (_fov_upper_deg/180.0*M_PI - angle_h)/angle_h_resolution;
		if(row < 0 || row >= _num_rings){
			std::cout << "ERROR: row = " << row << std::endl;
			// exit(1);
			return false;
		}
		/*col*/
		double angle_w = atan2(p_y, p_x);
		int col = (_points_per_ring - 1) - (int)((angle_w + M_PI)/angle_w_resolution);
		if(col < 0 || col >= _points_per_ring){
			std::cout << "ERROR col" << std::endl;
			// exit(1);
			return false;
		}
		/*depth*/
		double depth = sqrt(p_x*p_x + p_y*p_y);
		/*input*/
		depthimage_mat[row*_points_per_ring + col] = depth;
	}
	/*test*/
	//int test_counter = 0;
	//std::cout << "lidar_data.point_cloud.size()/3 = " << lidar_data.point_cloud.size()/3 << std::endl;
	//std::cout << "depthimage_mat.size() = " << depthimage_mat.size() << std::endl;
	//for(size_t i=0; i<depthimage_mat.size(); ++i){
	//	if(depthimage_mat[i] == 0)  ++test_counter;
	//}
	//std::cout << "test_counter = " << test_counter << std::endl;

	return true;
}

void DroneRandomPose::eular_to_quat(float r, float p, float y, Eigen::Quaternionf& q)
{
	q = Eigen::AngleAxisf(y, Eigen::Vector3f::UnitZ())
		* Eigen::AngleAxisf(p, Eigen::Vector3f::UnitY())
		* Eigen::AngleAxisf(r, Eigen::Vector3f::UnitX());
}


int main(){
    DroneRandomPose drone_random_pose;
    drone_random_pose.start_sampling();

    return 0;
}
