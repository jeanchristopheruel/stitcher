#include <assert.h>
#include "dataloader/dataloader.h"

namespace fs = boost::filesystem;

std::vector<float> to_vec(const cv::Mat &mat){
    cv::Mat float_mat;
    mat.convertTo(float_mat, CV_32FC1);
    float_mat = float_mat.reshape(1, float_mat.total()*float_mat.channels());
    return float_mat.isContinuous()? float_mat : float_mat.clone();
}

namespace laz {

    // ----------------------------------------------------------------------------------------------
    // To Json
    // ----------------------------------------------------------------------------------------------
    nlohmann::json to_json(const Camera& _cam)
    {
        nlohmann::json base_json;
        base_json["dims"] = {_cam.size().width, _cam.size().height};
        return base_json;
    }

    nlohmann::json to_json(const IntrinsicCamera& _cam)
    {
        nlohmann::json intrinsic_json;
        intrinsic_json = to_json(dynamic_cast<const Camera&>(_cam));
        intrinsic_json["intrinsic"] = to_vec(_cam.get_intrinsic());
        intrinsic_json["dist_coeffs"] = to_vec(_cam.get_dist_coeffs());
        return intrinsic_json;
    }

    nlohmann::json to_json(const ExtrinsicCamera& _cam)
    {
        nlohmann::json extrinsic_json = to_json(dynamic_cast<const IntrinsicCamera&>(_cam));
        extrinsic_json["extrinsic"] = to_vec(_cam.get_extrinsic());
        return extrinsic_json;
    }

    nlohmann::json to_json(const RotationCamera& _cam)
    {
        nlohmann::json rot_json = to_json(dynamic_cast<const ExtrinsicCamera&>(_cam));
        rot_json["rotation"] = to_vec(_cam.get_rotation());
        rot_json["focal"] = _cam.get_radius();
        return rot_json;
    }

    // ----------------------------------------------------------------------------------------------
    // From Json
    // ----------------------------------------------------------------------------------------------
    template <>
    Camera* from_json<Camera>(const std::string& cam_name, const nlohmann::json &cam_json)
    {
        const auto casted_dims = cam_json["dims"].get < std::array < int, 2 >> ();
        return new Camera(cam_name, cv::Size(casted_dims.at(0), casted_dims.at(1)));
    }

    template <>
    IntrinsicCamera* from_json<IntrinsicCamera>(const std::string& cam_name, const nlohmann::json &cam_json)
    {
        std::array < float, 9 > casted_intrinsic_coeffs = cam_json["intrinsic"].get < std::array < float, 9 >> ();
        cv::Mat intrinsic(3, 3, CV_32FC1, &casted_intrinsic_coeffs[0]);
        const auto casted_dist_coeffs = cam_json["dist_coeffs"].get < std::vector < float >> ();
        Camera* cam = from_json<Camera>(cam_name, cam_json);
        auto* intrinsic_cam = new IntrinsicCamera(*cam, intrinsic, casted_dist_coeffs);
        delete cam; cam = nullptr;
        return intrinsic_cam;
    }

    template <>
    ExtrinsicCamera* from_json<ExtrinsicCamera>(const std::string& cam_name, const nlohmann::json &cam_json)
    {
        std::array < float, 9 > casted_extrinsic = cam_json["extrinsic"].get <std::array<float, 9>>();
        cv::Mat extrinsic(3, 3, CV_32FC1, &casted_extrinsic[0]);
        IntrinsicCamera* intrinsic_cam = from_json<IntrinsicCamera>(cam_name, cam_json);
        auto* extrinsic_cam = new ExtrinsicCamera(*intrinsic_cam, extrinsic);
        delete intrinsic_cam; intrinsic_cam = nullptr;
        return extrinsic_cam;
    }

    template <>
    RotationCamera* from_json<RotationCamera>(const std::string& cam_name, const nlohmann::json &cam_json)
    {
        std::array < float, 9 > casted_rotation = cam_json["rotation"].get <std::array<float, 9>>();
        cv::Mat rotation(3, 3, CV_32FC1, &casted_rotation[0]);
        const float &focal_length = cam_json["focal"].get<float>();
        ExtrinsicCamera* extrinsic_cam = from_json<ExtrinsicCamera>(cam_name, cam_json);
        auto* rot_cam = new RotationCamera(*extrinsic_cam, rotation, focal_length);
        delete extrinsic_cam; extrinsic_cam = nullptr;
        return rot_cam;
    }

    template <typename T>
    T* from_json(const std::string& cam_name, const nlohmann::json &cam_json)
    {
        RotationCamera* rotation_cam = from_json<RotationCamera>(cam_name, cam_json);
        auto* cam = new T(*rotation_cam);
        delete rotation_cam; rotation_cam = nullptr;
        return cam;
    }

    template CvCylindricalCamera* from_json<CvCylindricalCamera>(const std::string& cam_name,
            const nlohmann::json &cam_json);
    template CvSphericalCamera* from_json<CvSphericalCamera>(const std::string& cam_name,
            const nlohmann::json &cam_json);
    template CylindricalCamera* from_json<CylindricalCamera>(const std::string& cam_name,
            const nlohmann::json &cam_json);

    // ----------------------------------------------------------------------------------------------
    // CameraFakeStream Loader
    // ----------------------------------------------------------------------------------------------
    void to_abs_path(std::string &_img_path, const std::string& dataset_path)
    {
        if (not does_file_exist(_img_path)){
            // Define abs path
            size_t dataset_dir_index = dataset_path.find_last_of("/");
            _img_path = dataset_path.substr(0, dataset_dir_index) + "/" + _img_path.substr(0, _img_path.size());
        }
        if (not does_file_exist(_img_path))
            throw std::runtime_error("Error: '" + _img_path +"' does not exists.");
    }

    template<typename T>
    std::vector<std::tuple<T*,std::vector<std::string>>> load_fakestream(const std::string& calibration_path, const std::string& dataset_path)
    {
        // Parse cylindrical calibration json
        std::ifstream calibration_file(calibration_path);
        const nlohmann::json& calibration_json = nlohmann::json::parse(calibration_file);

        // Parse dataset_path
        std::ifstream dataset_file(dataset_path);
        const fs::path dataset_abs_path = fs::canonical(dataset_path);
        const nlohmann::json& dataset_json = nlohmann::json::parse(dataset_file);

        std::vector<std::tuple<T*,std::vector<std::string>>> cameras;
        for (const auto& [cam_name, cam_json] : calibration_json["cameras"].items()) {
            PLOGI << "Registering '" << cam_name << ".";

            if (not dataset_json.contains(cam_name)) {
                PLOGE << "Error: '" << cam_name << "' was not found in dataset .\n"
                      << "       Skipping '" << cam_name << "'.";
                continue;
            }

            const nlohmann::json imgpaths_json = dataset_json[cam_name];
            std::vector<std::string> img_paths = imgpaths_json.get<std::vector<std::string>>();
            for(auto& img_path : img_paths)
                to_abs_path(img_path, dataset_path);

            T* cam = from_json<T>(cam_name, cam_json);
            std::tuple<T*,std::vector<std::string>> tuple = std::make_tuple(cam, img_paths);
            cameras.push_back( tuple );
        }
        assert(cameras.size() > 1);
        return cameras;
    }

    template std::vector<std::tuple<IntrinsicCamera*,std::vector<std::string>>>
            load_fakestream<IntrinsicCamera>(const std::string& calibration_path, const std::string& dataset_path);
    template std::vector<std::tuple<ExtrinsicCamera*,std::vector<std::string>>>
            load_fakestream<ExtrinsicCamera>(const std::string& calibration_path, const std::string& dataset_path);
    template std::vector<std::tuple<CylindricalCamera*,std::vector<std::string>>>
            load_fakestream<CylindricalCamera>(const std::string& calibration_path, const std::string& dataset_path);
    template std::vector<std::tuple<CvCylindricalCamera*,std::vector<std::string>>>
            load_fakestream<CvCylindricalCamera>(const std::string& calibration_path, const std::string& dataset_path);
    template std::vector<std::tuple<CvSphericalCamera*,std::vector<std::string>>>
    load_fakestream<CvSphericalCamera>(const std::string& calibration_path, const std::string& dataset_path);


    // ----------------------------------------------------------------------------------------------
    // Extrinsic Calibration Loader
    // ----------------------------------------------------------------------------------------------
    std::vector<CameraCalibration*> load_calibration_cams(const std::string& calibration_path, const std::string& dataset_path)
    {
        // Parse cylindrical calibration json
        std::ifstream calibration_file(calibration_path);
        const nlohmann::json& calibration_json = nlohmann::json::parse(calibration_file);

        // Parse dataset_path
        std::ifstream dataset_file(dataset_path);
        const fs::path dataset_abs_path = fs::canonical(dataset_path);
        const nlohmann::json& dataset_json = nlohmann::json::parse(dataset_file);

        std::vector<CameraCalibration*> cameras;
        for (const auto& [cam_name, cam_json] : calibration_json["cameras"].items()) {
            if (not dataset_json.contains(cam_name)) {
                PLOGW << "'" << cam_name << "' was not found in dataset. Skipping '" << cam_name << "'.";
                continue;
            }

            const nlohmann::json imgpaths_json = dataset_json[cam_name];
            std::vector<std::string> img_paths = imgpaths_json.get<std::vector<std::string>>();
            for(auto& img_path : img_paths) {
                to_abs_path(img_path, dataset_path);
                IntrinsicCamera* intrinsic_cam = from_json<IntrinsicCamera>(cam_name, cam_json);
                auto* calib_cam = new CameraCalibration(*intrinsic_cam, img_path);
                delete intrinsic_cam; intrinsic_cam = nullptr;
                cameras.push_back(calib_cam);
            }
            PLOGI << "Camera '" << cam_name << "' registered.";
        }
        assert(cameras.size() > 1);
        return cameras;
    }

} // namespace laz