#include <map>
#include <fstream>
#include <string>
#include <optional>
#include <math.h>
#include <android/log.h>
//#include  <android_fopen.h>
#include <android/asset_manager.h>

#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <../Eigen/Dense>

#include "csv_iterator.hpp"
#include "common/error_util.hpp"
#include "common/file_helper.h"

#define LOG(...) ((void)__android_log_print(ANDROID_LOG_INFO, "data-loading", __VA_ARGS__))

// timestamp
// p_RS_R_x [m], p_RS_R_y [m], p_RS_R_z [m]
// q_RS_w [], q_RS_x [], q_RS_y [], q_RS_z []
// v_RS_R_x [m s^-1], v_RS_R_y [m s^-1], v_RS_R_z [m s^-1]
// b_w_RS_S_x [rad s^-1], b_w_RS_S_y [rad s^-1], b_w_RS_S_z [rad s^-1]
// b_a_RS_S_x [m s^-2], b_a_RS_S_y [m s^-2], b_a_RS_S_z [m s^-2]

using namespace ILLIXR;

typedef pose_type sensor_types;

static
std::map<ullong, sensor_types>
load_data() {
	const char* illixr_data_c_str = std::getenv("ILLIXR_DATA");
	if (!illixr_data_c_str) {
	    LOG("Please define ILLIXR_DATA");
        ILLIXR::abort("Please define ILLIXR_DATA");
	}
	std::string illixr_data = std::string{illixr_data_c_str};

	std::map<ullong, sensor_types> data;

	//std::ifstream gt_file { illixr_data + "/state_groundtruth_estimate0/data.csv"};
	//std::string filename = "/home/madhuparna/AndroidStudioProjects/illixr-native-activity/app/src/main/cpp/ILLIXR_DATA/state_groundtruth_estimate0/data.csv";
	std::string filename = "/home/madhuparna/random.txt";
	std::ifstream gt_file ; //(filename);
    gt_file.open(filename);

	//AAssetManager* mgr = AAsetManager_fromJava(env, assetManager);
	//AAssetManager *aAssetManager = AndroidHelper::getAssetManager();
	int n = filename.size();
    char ch_file[n+1];
    strcpy(ch_file, filename.c_str());
	if (!gt_file.good()) {
		LOG("Not a good path .... %s", ch_file);
	    std::cerr << "${ILLIXR_DATA}/state_groundtruth_estimate0/data.csv (" << illixr_data <<  "/state_groundtruth_estimate0/data.csv) is not a good path" << std::endl;
        ILLIXR::abort();
	}
	LOG("befo .... %s", ch_file);
	//ILLIXR::abort();
	for(CSVIterator row{gt_file, 1}; row != CSVIterator{}; ++row) {
		LOG("inside .... %s", ch_file);
		ullong t = std::stoull(row[0]);
		Eigen::Vector3f av {std::stof(row[1]), std::stof(row[2]), std::stof(row[3])};
		Eigen::Quaternionf la {std::stof(row[4]), std::stof(row[5]), std::stof(row[6]), std::stof(row[7])};
		data[t] = {{}, av, la};
	}
	LOG("outside .... %s", ch_file);

	return data;
}
