#include <string>
#include <iostream>
#include <unordered_map>

#include "gtest/gtest.h"
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <boost/filesystem.hpp>
#include <json.hpp>

#include "core/camera.h"
#include "calibration/calibrator.h"
#include "paths.h"

namespace fs = boost::filesystem;

namespace TestConfig{
    static const fs::path assets_path = fs::path(getAssetsDirPath());
    static const fs::path intrinsic_calib_path = fs::path(getAssetsDirPath()+"/test_intrinsic_calib.json");
    static const fs::path dataset_path = fs::path(getAssetsDirPath()+"/test_dataset.json");
    static const float features_conf_thresh = 0.4;
    static const float bundle_conf_thresh = 0.4;
    static const laz::BundleAdjustor adjustor = laz::BundleAdjustor::REPROJ;
}

laz::CalibrationCam parse_intrinsic_json(const nlohmann::json &cam_params){
    std::array<double, 9> casted_intrinsic_coeffs = cam_params["intrinsic"].get<std::array<double, 9>>();
    cv::Mat_<double> intrinsic(3,3, &casted_intrinsic_coeffs[0]);
    const auto casted_dist_coeffs = cam_params["dist_coeffs"].get<std::vector<double>>();
    const auto casted_dims = cam_params["dims"].get<std::array<int, 2>>();
    const laz::IntrinsicCamera cam(intrinsic, casted_dist_coeffs, cv::Size(casted_dims.at(0), casted_dims.at(1)));
    return laz::CalibrationCam(cam);
}

TEST(CalibrationTests, GeneralExtrinsicCalibration){
    // Parse intrinsic_calib_path
    auto path = TestConfig::intrinsic_calib_path.string();
    ASSERT_TRUE(fs::exists(TestConfig::intrinsic_calib_path));
    std::ifstream intrinsic_calib_file(TestConfig::intrinsic_calib_path);
    nlohmann::json intrinsic_json = nlohmann::json::parse(intrinsic_calib_file);

    // Parse dataset_path
    ASSERT_TRUE(fs::exists(TestConfig::dataset_path));
    std::ifstream dataset_file(TestConfig::dataset_path);
    nlohmann::json dataset_json = nlohmann::json::parse(dataset_file);

    laz::Calibrator calibrator(TestConfig::features_conf_thresh,
                                             TestConfig::bundle_conf_thresh,
                                             TestConfig::adjustor);
    std::unordered_map<int, fs::path> images_id_to_path;
    int img_id = 0;
    for (const auto& [cam_name, cam_params] : intrinsic_json.items())
    {
        if(not dataset_json.contains(cam_name))
            continue;

        laz::CalibrationCam calib_cam = parse_intrinsic_json(cam_params);

        for (const auto& image_path : dataset_json[cam_name]){
            fs::path abs_img_path = TestConfig::assets_path;
            abs_img_path /= image_path.get<std::string>();
            cv::Mat img = cv::imread(abs_img_path.string(), cv::IMREAD_COLOR);
            cv::cvtColor(img, img, cv::COLOR_BGR2GRAY); // Color to 1 channel Gray img

            ASSERT_TRUE(img.cols != 0 && img.rows != 0);
            calib_cam.addImage(img_id, img);
            images_id_to_path.insert(std::pair<int, fs::path>(img_id, abs_img_path));
            img_id++;
        }
        calibrator.addCamera(calib_cam);
    }

    calibrator.calibrate();
    for (laz::CalibrationCam& calibrated_cam : calibrator.get_cameras()){
        for (auto& it: calibrated_cam.get_calibration()){
            img_id = it.first;
            const cv::detail::CameraParams &params = it.second;
            cv::Mat diff = params.R != cv::Mat::eye(3, 3, CV_32F);
            ASSERT_NE(cv::countNonZero(diff), 0);
            diff = params.K() != cv::Mat::eye(3, 3, CV_64F);
            ASSERT_NE(cv::countNonZero(diff), 0);
        }
    }
}

//-------------------------------------------------------------------------------
// Unit Tests
//-------------------------------------------------------------------------------
int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}