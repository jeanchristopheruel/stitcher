#include <assert.h>
#include <set>

#include <plog/Log.h>
#include <plog/Init.h>
#include <plog/Formatters/TxtFormatter.h>
#include <plog/Appenders/ColorConsoleAppender.h>

#include "dataloader/dataloader.h"
#include "stitching/stitcher.h"

namespace fs = boost::filesystem;

namespace default_values{
    // Optional Parameters
    static const bool do_update_exposure = false;
    static const bool do_update_seams = false;
    static const float scale_factor = 1.0f;
    static const float blend_strength = 5.0f;
}

static void printUsage(){
    std::cout <<
              "[Custom Stitching] \n"
              "This tool performs the stitching of multiple camera stream for a given projection model. \n"
              "The stream can be faked by providing a dataset of images for each camera.\n"
              "It could also potentially be used on live camera with further optimizations.\n"
              "\n"
              "FLAGS\n"
              "  --help                [-h]  Display this message.\n"
              "\n"
              "  --calibration_path    [-c]  MANDATORY\n"
              "                              Specify the path to the calibration json that contains \n"
              "                              calibration parameters for each registered cameras.\n"
              "\n"
              "  --dataset_path        [-d]  MANDATORY\n"
              "                              Specify the path of the json file that contains the paths to the \n"
              "                              images associated with each registered cameras.\n"
              "\n"
              "  --blend_strength      [-s]  OPTIONAL\n"
              "                              DEFAULT: " << default_values::blend_strength << "\n"
              "                              Blending strength from [0,100] range. The default is 5. \n"
              "\n"
              "  --do_update_exposure        OPTIONAL\n"
              "                              DEFAULT: " << default_values::do_update_exposure << "\n"
              "                              Allow to update exposure at each stitched frame. \n"
              "\n"
              "  --do_update_seams           OPTIONAL\n"
              "                              DEFAULT: " << default_values::do_update_seams << "\n"
              "                              Allow to update seams at each stitched frame. \n"
              "\n"
              "  --scale_factor              OPTIONAL\n"
              "                              DEFAULT: " << default_values::scale_factor << "\n"
              "                              Camera scaling factor. Used to reduce memory consumption.\n"
              "                              Must be withing this interval: ]0, 1]. \n"
              "\n\n";
}

int main(int argc, char** argv) {
    static plog::ConsoleAppender<plog::TxtFormatter> consoleAppender;
    plog::init(plog::debug, &consoleAppender);

    // Mandatory Parameters
    fs::path calibration_path, dataset_path;

    // Optional Parameters
    bool do_update_exposure = default_values::do_update_exposure;
    bool do_update_seams = default_values::do_update_seams;
    float scale_factor = default_values::scale_factor;
    float blend_strength = default_values::blend_strength;

    // Unarg Parameters
    float gamma_corr_alpha = 1.8f;
    uint8_t gamma_corr_beta = 10;

    std::set<std::string> unused_param = {"--calibration_path", "--dataset_path"};

    if (argc == 1) {
        printUsage();
        return EXIT_FAILURE;
    }
    for (int i = 1; i < argc; ++i) {
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
        else if (std::string(argv[i]) == "--blend_strength" || std::string(argv[i]) == "-s" ){
            i++;
            blend_strength = std::atof(argv[i]);
        }
        else if (std::string(argv[i]) == "--do_update_exposure"){
            do_update_exposure = true;
        }
        else if (std::string(argv[i]) == "--do_update_seams"){
            do_update_seams = true;
        }
        else if (std::string(argv[i]) == "--scale_factor"){
            i++;
            scale_factor = std::atof(argv[i]);
        }
        else
        {
            std::string error_msg = "Unknown parameter '" + std::string(argv[i]) + "'.";
            throw std::runtime_error(error_msg);
        }
    }

    if (!unused_param.empty()){
        std::string error_msg = "One or more mandatory parameters have not been set:\n";
        for (const auto& param : unused_param)
            error_msg += "\t" + param + "\n";
        throw std::runtime_error(error_msg);
    }

    const std::vector<std::tuple<laz::CvCylindricalCamera*,std::vector<std::string>>>& cameras_data =
            laz::load_fakestream<laz::CvCylindricalCamera>(calibration_path.string(), dataset_path.string());

    std::vector<laz::CameraStream*> streams;
    for(int i=0;i<cameras_data.size();i++)
    {
        laz::CvCylindricalCamera* cam_ptr = std::get<0>(cameras_data[i]);
        const std::vector<std::string>& img_paths = std::get<1>(cameras_data[i]);
        streams.push_back(new laz::CameraFakeStream(cam_ptr, img_paths));
    }

    // Components declaration
    auto* stream_bundle = new laz::StreamBundler(streams);
    stream_bundle->connect();
    auto* gamma_corrector = new laz::GammaCorrector(gamma_corr_alpha, gamma_corr_beta);
    auto* exposure_compensator = new laz::CvExposureCompensatorChannelsBlocks();
    auto* seam_finder = new laz::CvSeamFinderGcColorGrad (scale_factor);
    auto* blender = new laz::CvBlenderFeather(blend_strength);

    // Stitcher build
    auto stitcher = laz::Stitcher::StitcherBuilder(stream_bundle, blender)
            .attach_gamma_corrector(gamma_corrector)
            .attach_exp_compensator(exposure_compensator)
            .attach_seam_finder(seam_finder)
            .build();
    stitcher.init_from_current_stream();

    cv::Mat mosaic;
    int img_idx=0;
    while (stitcher.read(mosaic, do_update_exposure, do_update_seams)){
        img_idx++;
        cv::imwrite("mosaic_" + std::to_string(img_idx)+".png", mosaic);
    }
    PLOGI << "Stitching Done.";

    delete stream_bundle; stream_bundle = nullptr;
    delete gamma_corrector; gamma_corrector = nullptr;
    delete exposure_compensator; exposure_compensator = nullptr;
    delete seam_finder; seam_finder = nullptr;
    delete blender; blender = nullptr;

    for(int i=0;i<cameras_data.size();i++)
    {
        laz::CameraStream* stream_ptr = streams[i];
        delete stream_ptr; stream_ptr = nullptr;

        laz::CvCylindricalCamera* cam_ptr = std::get<0>(cameras_data[i]);
        delete cam_ptr; cam_ptr = nullptr;
    }
    return 0;
}