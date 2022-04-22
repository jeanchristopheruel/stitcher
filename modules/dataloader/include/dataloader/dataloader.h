#ifndef LIVESTITCHER_DATALOADER_H
#define LIVESTITCHER_DATALOADER_H
#include <memory.h>
#include <iomanip>
#include <fstream>
#include <string>
#include <tuple>
#include <iostream>

#include <opencv2/core.hpp>
#include <nlohmann/json.hpp>
#include <boost/filesystem.hpp>

#include "core/camerastream.h"
#include "core/streambundler.h"

#pragma once

/**
 * LAZ-Standardised files format:
 *      intrinsic_json Structure:
 *      {
 *          "cameras": {
 *              cam1: {
 *                  "intrinsic": <list of float, len<9>>,
 *                  "dist_coeffs": <list of load, len<5>>,
 *                  "dims": [<width>, <height>]
 *              },
 *              ...
 *           }
 *      }
 *
 *      extrinsic_json Structure:
 *      {
 *          "cameras": {
 *              "cam1": {
 *                  "intrinsic": <list of float, len<9>>,
 *                  "dist_coeffs": <list of load, len<5>>,
 *                  "dims": [<width>, <height>],
 *                  "extrinsic": <list, len<9>>
 *              },
 *              ...
 *          }
 *      }
 *
 *      rotation_json Structure:
 *      {
 *          "cameras": {
 *              "cam1": {
 *                  "intrinsic": <list of float, len<9>>,
 *                  "dist_coeffs": <list of load, len<5>>,
 *                  "dims": [<width>, <height>],
 *                  "extrinsic": <list, len<9>>
 *                  "focal": <double>,
 *              },
 *              ...
 *          }
 *      }
 * @param dataset_json
 *      dataset Structure:
 *      {
 *          "cam1": <list of img paths, len<X>>,
 *          "cam2": <list of img paths, len<X>>,
 *          ...
 *      }
 */

namespace laz {

    bool does_file_exist(const std::string& name) {
        std::ifstream f(name.c_str());
        return f.good();
    };

    nlohmann::json to_json(const Camera& _cam);
    nlohmann::json to_json(const IntrinsicCamera& _cam);
    nlohmann::json to_json(const ExtrinsicCamera& _cam);
    nlohmann::json to_json(const RotationCamera& _cam);

    /**
     * Parse calibration file from camera calibration
     * @param cam_name: Camera name from calibration json
     * @param cam_json: Specific subjson from Calibration json
     * @return laz::IntrinsicCamera
     */
    template<typename T>
    T* from_json(const std::string& cam_name, const nlohmann::json &cam_json);

    /**
     * Parse calibration file from intrinsic camera calibration
     * @param cam_name: Bundle Calibration json
     * @param dataset_json: Bundle Dataset Json containing img path list for each CameraFakeStream
     * @return laz::IntrinsicCamera
     */
    template<typename T>
    std::vector<std::tuple<T*,std::vector<std::string>>> load_fakestream(const std::string& calibration_path,
                                                                        const std::string& dataset_path);

    std::vector<CameraCalibration*> load_calibration_cams(const std::string& calibration_path, const std::string& dataset_path);
} // namespace laz 

#endif //LIVESTITCHER_DATALOADER_H
