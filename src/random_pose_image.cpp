#include "kawai_airsim_controller/random_pose_image.h"

ImageDroneRandomPose::ImageDroneRandomPose(){
    std::cout << "Image Drone Random Pose" << std::endl;

    //client initialization
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
    if(_save_data){
        leave_param_note();
        csv_initialization();
    }

    std::cout << "Initialization Done" << std::endl;
}

ImageDroneRandomPose::~ImageDroneRandomPose(){}

void ImageDroneRandomPose::client_initialization(){
    //connect
    _client.confirmConnection();

    //reset
    std::cout << "Reset" << std::endl;
    _client.reset();

    //pose
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    update_state();

    /*weather*/
	if(_random_weather)	_client.simEnableWeather(true);
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

void ImageDroneRandomPose::update_state(){
    /*pose*/
	_pose = _client.simGetVehiclePose();

    /*
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
    */

	/*imu*/
	_imu = _client.getImuData();

    /*
	std::cout << "IMU: " << std::endl;
	std::cout << " linear_acceleration: "	//Eigen::Vector3f
		<< _imu.linear_acceleration.x() << ", "
		<< _imu.linear_acceleration.y() << ", "
		<< _imu.linear_acceleration.z() << std::endl;
    */
}

void ImageDroneRandomPose::leave_param_note(){
    //open
    std::ofstream txtfile;
    const std::string _save_txt_path = _save_data_top_path + "/param_note.txt";
    txtfile.open(_save_txt_path, std::ios::app);

    if(!txtfile){
        std::cout << "Can't Make .txt File!! Exit...." << std::endl;
        exit(1);
    }

    txtfile << "----------" << std::endl
		<< "_randomize_weather" << ": " << (bool)_random_weather << std::endl
		<< "_num_sampling" << ": " << _num_sampling << std::endl
		<< "_x_range" << ": " << _x_range << std::endl
		<< "_y_range" << ": " << _y_range << std::endl
		<< "_z_min" << ": " << _z_min << std::endl
		<< "_z_max" << ": " << _z_max << std::endl
		<< "_roll_max" << ": " << _roll_max/M_PI*180.0 << std::endl
        << "_roll_min" << ": " << _roll_min/M_PI*180.0 << std::endl
        << "_pitch_max" << ": " << _pitch_max/M_PI*180.0 << std::endl
        << "_pitch_min" << ": " << _pitch_min/M_PI*180.0 << std::endl
        << "_yaw_max" << ": " << _roll_max/M_PI*180.0 << std::endl
        << "_yaw_min" << ": " << _roll_min/M_PI*180.0 << std::endl
        << "resolution" << ": " << resolution << std::endl;

        txtfile.close();
}

void ImageDroneRandomPose::csv_initialization(){
    _csvfile.open(_save_data_top_path, std::ios::app);
	if(!_csvfile){
		std::cout << "Cannot open " << _save_data_top_path << std::endl;
		exit(1);
	}
}

void ImageDroneRandomPose::spin(){
    std::cout << "Start Sampling" << std::endl;

    bool continue_func = true;

    int i = 0;

    while(continue_func==true){
        std::cout << "----SAMPLE " << i << " -----" << std::endl;
        if(_random_weather) random_weather();
        random_place();
        _client.simPause(true);
        update_state();
        int save_image_checker = image_checker();
        if(save_image_checker==0){
            std::cout << "Save Place" << std::endl;
            std::vector<float> tmp_place{_pose.position.x(), _pose.position.y(), _pose.position.z(), _pose.orientation.x(), _pose.orientation.y(), _pose.orientation.z(), _pose.orientation.z()};

            place_list.push_back(tmp_place);
        }
        else if(save_image_checker==1){
            std::cout << "Not Appropriate Position and Reset Position" << std::endl;
        }
        else{
            std::cout << "Place Sampling Done. Save place as .csv file" << std::endl;
            continue_func = false;
        }
        _client.simPause(false); //位置更新に必須
        i++;
    }

    save_place();
}

void ImageDroneRandomPose::random_weather(){
    /*reset*/
	for(size_t i=0; i<_list_weather.size(); ++i)	_client.simSetWeatherParameter(_list_weather[i], 0.0);
	/*random*/
	std::random_device rd;
	std::mt19937 mt(rd());
	std::uniform_int_distribution<> uid_index(0, _list_weather.size()-1);
	std::uniform_real_distribution<> urd_val(0.0, 1.0); //天候の程度を表す？
	for(int i=1; i<=uid_index(mt); ++i){
		int weather_index = uid_index(mt);
		double weather_val = urd_val(mt);
		_client.simSetWeatherParameter(_list_weather[weather_index], weather_val);
	}
}

void ImageDroneRandomPose::random_place(){
    /*random*/
	std::random_device rd;

	std::mt19937 mt(rd());

	std::uniform_real_distribution<> urd_x(-_x_range, _x_range);
	std::uniform_real_distribution<> urd_y(-_y_range, _y_range);
	std::uniform_real_distribution<> urd_z(_z_min, _z_max);

    std::uniform_real_distribution<> urd_yaw(_yaw_min, _yaw_max);

    Eigen::Vector3f position( urd_x(mt), urd_y(mt), urd_z(mt));
    float roll = 0.001;
    float pitch = 0.001;
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

void ImageDroneRandomPose::eular_to_quat(float r, float p, float y, Eigen::Quaternionf& q)
{
	q = Eigen::AngleAxisf(y, Eigen::Vector3f::UnitZ())
		* Eigen::AngleAxisf(p, Eigen::Vector3f::UnitY())
		* Eigen::AngleAxisf(r, Eigen::Vector3f::UnitX());
}

int ImageDroneRandomPose::image_checker(){
    
    std::cout << "Checking Image" << std::endl;
    
    int result = 0;
    //Request Image per Drone's camera
    std::vector<msr::airlib::ImageCaptureBase::ImageRequest> list_request(_list_camera.size());
    
    //std::cout << "Get Image" << std::endl;
    
    for(size_t i=0; i<_list_camera.size(); ++i){
		list_request[i] = msr::airlib::ImageCaptureBase::ImageRequest(_list_camera[i], msr::airlib::ImageCaptureBase::ImageType::Scene, false, false);
        //sr::airlib::ImageCaptureBase::ImageRequest tmp_request = msr::airlib::ImageCaptureBase::ImageRequest(_list_camera[i], msr::airlib::ImageCaptureBase::ImageType::Scene, false, false);
        //list_request.push_back(tmp_request);
	}

    //Get Image Data
    //std::cout << "Response" << std::endl;
    std::vector<msr::airlib::ImageCaptureBase::ImageResponse> list_response = _client.simGetImages(list_request);
    //std::cout << "Catch" << std::endl;


    cv::Mat img_cv = cv::Mat(list_response[0].height, list_response[0].width, CV_8UC3);
    for(int row=0; row<list_response[0].height; ++row){
		for(int col=0; col<list_response[0].width; ++col){
			img_cv.at<cv::Vec3b>(row, col)[0] = list_response[0].image_data_uint8[3*row*list_response[0].width + 3*col + 0];
			img_cv.at<cv::Vec3b>(row, col)[1] = list_response[0].image_data_uint8[3*row*list_response[0].width + 3*col + 1];
			img_cv.at<cv::Vec3b>(row, col)[2] = list_response[0].image_data_uint8[3*row*list_response[0].width + 3*col + 2];
		}
    }	
    
    std::cout << "Do you want to save this place's data?" << std::endl
        << "Answer Yes[y] or No[n] or Quit[q]." << std::endl;

    cv::imshow("Place Photo", img_cv);
    int answer = cv::waitKey(0);

    if(answer == 0x79 /* y */){
        result = 0;
        std::cout << "Yes" << std::endl;
    }
    else if(answer == 0x6E /* n */){
        result = 1;
        std::cout << "No" << std::endl;
    }
    else if(answer == 0x71 /* q */){
        result = 2;
        std::cout << "Quit" << std::endl;
    }
    else{
        result = 1;
        std::cout << "No" << std::endl;
    }

    cv::destroyAllWindows();

    return result;
}

void ImageDroneRandomPose::save_place(){

    std::string save_file_path = _save_data_top_path + _save_data_csv_name;

    std::ofstream csvfile(save_file_path);

    for(size_t i=0; i<place_list.size(); ++i){
        csvfile << place_list[i][0] << ","
        << place_list[i][1] << ","
        << place_list[i][2] << ","
        << place_list[i][3] << ","
        << place_list[i][4] << ","
        << place_list[i][5] << ","
        << place_list[i][6] << std::endl;
    }

    csvfile.close();
}


int main(){
    ImageDroneRandomPose image_drone_random_pose;
    image_drone_random_pose.spin();

    return 0;
}