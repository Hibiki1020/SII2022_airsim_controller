#include "kawai_airsim_controller/original_image.h"

OriginalImage::OriginalImage(){
    std::cout << "Get Original Image" << std::endl;
    //client
    client_initialization();
    /*camera list*/
	_list_camera = {
		"camera_0"
	};
    //leave_param_note();
}

OriginalImage::~OriginalImage(){
    std::cout << "End Get Original Image" << std::endl;
}

void OriginalImage::client_initialization(){
    //connect
    _client.confirmConnection(); //msr::airlib::RpcLibClientBase's function

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

void OriginalImage::leave_param_note(){
    std::ofstream txtfile;
    const std::string _save_txt_path = image_top_path + "param_note.txt";
    txtfile.open(_save_txt_path, std::ios::app);

    if(!txtfile){
        std::cout << "Can't Make .txt File!! Exit...." << std::endl;
        exit(1);
    }

    txtfile << "----------" << std::endl
		<< "_randomize_weather" << ": " << (bool)_random_weather << std::endl
		<< "_roll_max" << ": " << _roll_max/M_PI*180.0 << std::endl
        << "_roll_min" << ": " << _roll_min/M_PI*180.0 << std::endl
        << "_pitch_max" << ": " << _pitch_max/M_PI*180.0 << std::endl
        << "_pitch_min" << ": " << _pitch_min/M_PI*180.0 << std::endl
        << "_yaw_max" << ": " << _roll_max/M_PI*180.0 << std::endl
        << "_yaw_min" << ": " << _roll_min/M_PI*180.0 << std::endl
        << "resolution" << ": " << resolution << std::endl;

        txtfile.close();
}

void OriginalImage::update_state(){
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

std::vector<std::string> OriginalImage::split(std::string& input, char delimiter)
{
    std::istringstream stream(input);
    std::string field;
    std::vector<std::string> result;
    while (getline(stream, field, delimiter)) {
        result.push_back(field);
    }
    return result;
}

bool OriginalImage::load_csv(){
    bool checker = true;
    std::string place_csv_path = place_csv_root_path + place_csv_name;

    std::ifstream csv_file(place_csv_path);
    if(!csv_file){
        std::cout << "CSV File " << place_csv_path << " not found. Exit..." << std::endl;
        checker = false;
    }
    else{
        std::string line;
        while( getline(csv_file, line)){
            std::vector<std::string> tmp_data = split(line, ',');
            csv_data.push_back(tmp_data);
        }

        csv_data_size = csv_data.size();
    }
    return checker;
}

std::vector<float> OriginalImage::string_to_float(std::vector<std::string> tmp_place_csv_data){
    std::vector<float> farray;

    for(size_t i=0; i < tmp_place_csv_data.size(); ++i){
        std::string tmp_string = tmp_place_csv_data[i];
        float tmp_f = std::stof(tmp_string);

        farray.push_back(tmp_f);
    }

    return farray;
}

void OriginalImage::eular_to_quat(float r, float p, float y, Eigen::Quaternionf& q)
{
	q = Eigen::AngleAxisf(y, Eigen::Vector3f::UnitZ())
		* Eigen::AngleAxisf(p, Eigen::Vector3f::UnitY())
		* Eigen::AngleAxisf(r, Eigen::Vector3f::UnitX());
}

float OriginalImage::check_deg(float deg){
    float result = 0;
    if(deg > M_PI){
        float tmp = deg - M_PI;
        result = -1.0 * M_PI + tmp;
        std::cout << "upper" << std::endl;
    }
    else if( deg < -1.0 * M_PI){
        float tmp = deg + M_PI;
        result = M_PI + tmp;
        std::cout << "lower" << std::endl;
    }
    else{
        result = deg;
    }

    return result;
}


cv::Mat OriginalImage::get_mono_image(){
    std::vector<msr::airlib::ImageCaptureBase::ImageRequest> list_request(_list_camera.size());
    for(size_t i=0; i<_list_camera.size(); ++i){
		list_request[i] = msr::airlib::ImageCaptureBase::ImageRequest(_list_camera[i], msr::airlib::ImageCaptureBase::ImageType::Scene, false, false);
	}
    std::vector<msr::airlib::ImageCaptureBase::ImageResponse> list_response = _client.simGetImages(list_request);
    cv::Mat img_cv = cv::Mat(list_response[0].height, list_response[0].width, CV_8UC3);
    for(int row=0; row<list_response[0].height; ++row){
		for(int col=0; col<list_response[0].width; ++col){
			img_cv.at<cv::Vec3b>(row, col)[0] = list_response[0].image_data_uint8[3*row*list_response[0].width + 3*col + 0];
			img_cv.at<cv::Vec3b>(row, col)[1] = list_response[0].image_data_uint8[3*row*list_response[0].width + 3*col + 1];
			img_cv.at<cv::Vec3b>(row, col)[2] = list_response[0].image_data_uint8[3*row*list_response[0].width + 3*col + 2];
		}
    }
    cv::Mat gray_img;
    cvtColor(img_cv, gray_img, cv::COLOR_BGR2GRAY);
    return gray_img;
}

void OriginalImage::save_image_place(cv::Mat original_mono_image, float x, float y, float z, float roll, float pitch, float yaw){
    std::string image_place_csv_path = image_top_path + integrated_csv_name;
    
    //Open csv file
    std::ofstream final_csvfile(image_place_csv_path, std::ios::app); //ios::app ??????????????????????????????

    std::string tmp_x = std::to_string(x);
    std::string tmp_y = std::to_string(y);
    std::string tmp_z = std::to_string(z);
    
    std::string tmp_roll = std::to_string(roll);
    std::string tmp_pitch = std::to_string(pitch);
    std::string tmp_yaw = std::to_string(yaw);

    std::string save_image_name = image_file_top_name + std::to_string(image_counter) + image_format;
    std::string save_image_path = image_top_path + save_image_name;

    cv::imwrite(save_image_path, original_mono_image);
    final_csvfile << save_image_name << ","
        << tmp_x << ","
        << tmp_y << ","
        << tmp_z << ","
        << tmp_roll << ","
        << tmp_pitch << ","
        << tmp_yaw << std::endl;

    image_counter += 1;
}

int OriginalImage::random_int(int min, int max){
    int num = 0;
    std::mt19937 mt{ std::random_device{}() };
    std::uniform_int_distribution<int> dist(min, max);

    num = dist(mt);
    return num;
}

void OriginalImage::save_image_data(){
    int num = 0;

    std::random_device rd;
    std::mt19937 mt(rd());

    std::cout << "CSV Data Size: " << csv_data_size << std::endl;

    while(num < num_image){
        int place_index = random_int(0, csv_data_size-1);
        //std::cout << "Choosed Index: " << place_index << std::endl;
        std::vector<std::string> tmp_place_csv_data = csv_data[place_index];

        //X Y Z Roll Pitch Yaw
        //std::cout << "Random Place" << std::endl;  
        std::vector<float> place_data = string_to_float(tmp_place_csv_data);

        std::uniform_real_distribution<float> urd_roll(_roll_min , _roll_max);
        std::uniform_real_distribution<float> urd_pitch( _pitch_min, _pitch_max);
        std::uniform_real_distribution<float> urd_yaw( place_data[5] + _yaw_min, place_data[5] + _yaw_max);

        Eigen::Vector3f position(place_data[0], place_data[1], place_data[2]); //x y z

        float roll = check_deg( urd_roll(mt) );
        float pitch = check_deg( urd_pitch(mt) );
        float yaw = check_deg( urd_yaw(mt) );

        Eigen::Quaternionf orientation;
        eular_to_quat(roll, pitch, yaw, orientation);
        msr::airlib::Pose goal = msr::airlib::Pose(position, orientation);

/*
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
*/

        _client.simSetVehiclePose(goal, true);
	    std::this_thread::sleep_for(std::chrono::milliseconds(_wait_time_millisec));
        _client.simPause(true);

        cv::Mat original_mono_image = get_mono_image();

        save_image_place(original_mono_image, place_data[0], place_data[1], place_data[2], roll, pitch, yaw);
        _client.simPause(false);

        //sleep(1);
        std::cout << "Image: " << num << std::endl;
        num += 1;
    }
}

void OriginalImage::spin(){
    //Load csv file
    bool csv_checker = load_csv();
    if(!csv_checker){
        exit(1);
    }

    //Save Image data
    save_image_data();
}

int main(){
    OriginalImage original_image;
    original_image.spin();

    return 0;
}