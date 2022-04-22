#include <string>
#include <iostream>
#include <iomanip>
#include <assert.h>
#include <unordered_map>
#include <set>

#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <boost/filesystem.hpp>

#include <plog/Log.h>
#include <plog/Init.h>
#include <plog/Formatters/TxtFormatter.h>
#include <plog/Appenders/ColorConsoleAppender.h>

#include "core/camera.h"
#include "core/cvcamera.h"
#include "dataloader/dataloader.h"
#include "calibration/calibrator.h"

namespace fs = boost::filesystem;

namespace default_values{
    static const float features_conf_thresh = 0.65;
    static const std::string adjustor_type = "ray";
}
namespace options_parser{
    static const std::map<std::string, laz::BundleAdjustor> adjustor_type = {
            {"no", laz::BundleAdjustor::NO},
            {"ray", laz::BundleAdjustor::RAY},
            {"reproj", laz::BundleAdjustor::REPROJ}
    };
}

std::string format_adjustor_options(){
    std::string formatting;
    auto final_iter = options_parser::adjustor_type.end(); --final_iter;
    for (auto it = options_parser::adjustor_type.begin(); it != options_parser::adjustor_type.end(); ++it)
        formatting += ( it == final_iter) ? "'" + std::string(it->first) + "'" : "'" + std::string(it->first) + "', ";
    return formatting;
}

static void printUsage(){
    std::cout <<
              "[Extrinsic Calibration for Cylindrical Stitching] \n"
              "This tool evaluate the homography transformation matrix R for each provided images using \n"
              "precalibrated intrinsic and distortion coefficients. It works by computing features pairing between \n"
              "all the provided images. A bundle adjuster can be used to minimise the global reprojection error. \n"
              "\n"
              "FLAGS\n"
              "  --help                [-h]  Display this message.\n"
              "\n"
              "  --calibration_path    [-c]  MANDATORY\n"
              "                              Specify the path to the intrinsic calibration json that contains \n"
              "                              intrinsic parameters, dist_coeffs and dimensions for each registered \n"
              "                              cameras.\n"
              "\n"
              "  --dataset_path        [-d]  MANDATORY\n"
              "                              Specify the path of the json file that contains the paths to the \n"
              "                              calibration images associated with each registered cameras.\n"
              "\n"
              "  --output_path         [-o]  MANDATORY\n"
              "                              Specify the path to the json output file that will contains the path to \n"
              "                              the results for the calibration of the extrinsic parameters.\n"
              "\n"
              "  --features_thresh           OPTIONAL\n"
              "                              DEFAULT: " << default_values::features_conf_thresh << "\n"
              "                              Specify the threshold for features extraction using ORB.\n"
              "\n"
              "  --adjustor_type             OPTIONAL\n"
              "                              DEFAULT: " << default_values::adjustor_type << "\n"
              "                              Specify the OpenCV Bundle Adjustor Type Used for features matching.\n"
              "                              Options: " << format_adjustor_options() << " \n"
              "\n\n";
}

int main(int argc, char** argv){
    static plog::ConsoleAppender<plog::TxtFormatter> consoleAppender;
    plog::init(plog::debug, &consoleAppender);

    // Mandatory Parameters
    fs::path calibration_path, dataset_path, output_path;

    // Optional Parameters
    float features_conf_thresh = default_values::features_conf_thresh;
    laz::BundleAdjustor adjustor = options_parser::adjustor_type.at(default_values::adjustor_type);

    std::set<std::string> unused_param = {"--calibration_path", "--dataset_path", "--output_path"};

    if (argc == 1){
        printUsage();
        return EXIT_FAILURE;
    }
    for(int i=1; i<argc; ++i){
        if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h"){
            printUsage();
            return 0;
        }
        else if (std::string(argv[i]) == "--calibration_path" || std::string(argv[i]) == "-c"){
            i++;
            calibration_path = fs::path(argv[i]);
            if (not fs::exists(calibration_path) )
                throw std::runtime_error("Error: Intrinsic calibration file '" +
                                                 calibration_path.string() +"' does not exists.");
            unused_param.erase("--calibration_path");
        }
        else if (std::string(argv[i]) == "--dataset_path" || std::string(argv[i]) == "-d" ){
            i++;
            dataset_path = fs::path(argv[i]);
            if (not fs::exists(dataset_path))
                throw std::runtime_error("Error: Dataset file '" +
                dataset_path.string() +"' does not exists.");
            unused_param.erase("--dataset_path");
        }
        else if (std::string(argv[i]) == "--output_path" || std::string(argv[i]) == "-o" ){
            i++;
            output_path = fs::path(argv[i]);
            /*if (fs::exists(output_path))
                throw std::runtime_error("Error: Cannot overwrite output_path because the file '" +
                output_path.string() +"' already exists.");
                */
            if (output_path.extension() != ".json")
                throw std::runtime_error("Error: Extension for output_path must be '.json'.");
            unused_param.erase("--output_path");
        }
        else if (std::string(argv[i]) == "--adjustor_type" ){
            i++;
            if ( options_parser::adjustor_type.find(argv[i]) == options_parser::adjustor_type.end() ) {
                std::string error_msg = "Error: adjustor_type unknown. Options: ";
                throw std::runtime_error(error_msg);
            }
            adjustor = options_parser::adjustor_type.at(argv[i]);
        }
        else
        {
            std::string error_msg = "Unknown parameter '" + std::string(argv[i]) + "'." + std::to_string(i);
            throw std::runtime_error(error_msg);
        }
    }

    if (!unused_param.empty()){
        std::string error_msg = "One or more mandatory parameters have not been set:\n";
        for (const auto& param : unused_param)
            error_msg += "\t" + param + "\n";
        throw std::runtime_error(error_msg);
    }

    // Parse intrinsic_calib_path
    std::vector<laz::CameraCalibration*> calib_cams = laz::load_calibration_cams(
            calibration_path.string(), dataset_path.string());

    std::vector<std::string> exist_names;
    for(int i=0;i<calib_cams.size();i++)
    {
        std::string cam_name = calib_cams[i]->get_name();
        int cam_idx = 1;
        while (std::find(exist_names.begin(), exist_names.end(), cam_name) != exist_names.end())
        {
            cam_name = calib_cams[i]->get_name() + "_" + std::to_string(cam_idx);
            cam_idx++;
        }
        calib_cams[i]->set_name(cam_name);
        exist_names.push_back(cam_name);
    }

    laz::Calibrator calibrator(features_conf_thresh, adjustor);
    std::vector<laz::RotationCamera> calibrated_cams = calibrator.calibrate(calib_cams);

    nlohmann::json calibration_outfile;
    nlohmann::json& cam_json = calibration_outfile["cameras"];
    for(const auto& cam: calibrated_cams)
    {
        PLOGD << "camera: "<< cam.get_name();
        PLOGD << "Radius: "<<cam.get_radius();
        PLOGD << "R: \n"<<cam.get_rotation();
        PLOGD << "K: \n\n"<<cam.get_extrinsic();
        cam_json[cam.get_name()] = laz::to_json(cam);
    }

    std::ofstream out_file;
    out_file.open(output_path.string());
    out_file << std::setw(4) << calibration_outfile << std::endl;
    out_file.close();
}