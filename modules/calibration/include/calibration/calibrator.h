#ifndef LIVESTITCHER_CALIBRATOR_H
#define LIVESTITCHER_CALIBRATOR_H
#include <vector>
#include <string>
#include <assert.h>
#include <unordered_map>
#include <iostream>
#include "opencv2/core.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/stitching/detail/warpers.hpp"
#include "opencv2/stitching/detail/autocalib.hpp"
#include "opencv2/stitching/detail/matchers.hpp"
#include "opencv2/stitching/detail/motion_estimators.hpp"

#include "core/camera.h"
#include "core/cvcamera.h"
#include "core/camerastream.h"

namespace laz {

    enum BundleAdjustor
    {
        REPROJ =0,
        RAY,
        NO
    };

    class Calibrator {
    public:
        Calibrator(const float &_features_conf_thresh=0.65,
                   const BundleAdjustor &_bundle_adjustor=BundleAdjustor::RAY):
                   m_features_conf_thresh(_features_conf_thresh),
                   m_conf_thresh(0.95),
                   m_bundle_adjustor(_bundle_adjustor)
                   {};

        const std::vector<RotationCamera> calibrate(const std::vector<CameraCalibration*>& _cameras);

    protected:
        const float m_features_conf_thresh, m_conf_thresh;
        const BundleAdjustor m_bundle_adjustor;
    };
} // namespace laz


#endif //LIVESTITCHER_CALIBRATOR_H
