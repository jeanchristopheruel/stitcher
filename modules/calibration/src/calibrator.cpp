#include "calibration/calibrator.h"
#include "core/camera.h"

#ifdef HAVE_OPENCV_XFEATURES2D
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"
#endif

namespace laz {

    template <typename T>
    T get_median(const std::vector<T>& _items)
    {
        std::vector<float> items = _items;

        sort(items.begin(), items.end());
        if (items.size() % 2 == 1)
            return items[items.size() / 2];
        else
            return T( (items[items.size() / 2 - 1] + items[items.size() / 2]) / 2);
    }

    const std::vector<RotationCamera> Calibrator::calibrate(const std::vector<CameraCalibration*>& _cameras) {
        assert(!_cameras.empty());

    #ifdef HAVE_OPENCV_XFEATURES2D
        int minHessian = 400;
        auto finder = cv::xfeatures2d::SURF::create(minHessian);
    #else
        auto finder = cv::ORB::create();
    #endif

        // Find features with ORB
        std::vector <cv::detail::ImageFeatures> features(_cameras.size());
        for (int i=0; i< _cameras.size(); i++) {
            cv::Mat undistorted_img = _cameras[i]->read();
            computeImageFeatures(finder, undistorted_img, features[i]);
            features[i].img_idx = i;

            PLOGI << "\tFeatures in image #"<< i << " (" << _cameras[i]->get_name() << "): " << features[i].keypoints.size();
        }

        //Pairwise matching
        std::vector <cv::detail::MatchesInfo> pairwise_matches;
        cv::detail::BestOf2NearestMatcher matcher(false, m_features_conf_thresh);

        matcher(features, pairwise_matches);
        matcher.collectGarbage();

        // Leave only images we are sure are from the same panorama
        // Size of features may change....
        std::vector<int> ret_indices = leaveBiggestComponent(features, pairwise_matches, m_conf_thresh);
        std::vector<int> diff, all_indices;
        for(int i=0; i< _cameras.size(); i++){all_indices.push_back(i);}
        std::set_difference(all_indices.begin(), all_indices.end(), ret_indices.begin(), ret_indices.end(),
                            std::inserter(diff, diff.begin()));

        if (not diff.empty()){
            std::string formatted_msg = "Ignoring theses images because they don't match well enough: [";
            for (const auto& i : diff){formatted_msg += std::to_string(i) + ", ";}
            formatted_msg += "]";
            PLOGW << formatted_msg << std::endl;
        }

        // Check if we still have enough images
        if (ret_indices.size() <= 2)
        {
            PLOGE << "Error: Images were all eliminated in leaveBiggestComponent";
            return {};
        }

        // Estimate Homography
        cv::detail::HomographyBasedEstimator estimator;
        std::vector<cv::detail::CameraParams> cameras_params(ret_indices.size());
        for (int i =0; i<features.size(); i++) {
            cv::Mat intrinsic, intrinsic64f;
            const int &cam_idx = features.at(i).img_idx;

            intrinsic = _cameras[cam_idx]->get_intrinsic();
            intrinsic.convertTo(intrinsic64f, CV_64F);

            cv::detail::CameraParams& calibrated_params = cameras_params[i];
            calibrated_params.focal = intrinsic64f.at<double>(0,0);
            calibrated_params.aspect = intrinsic64f.at<double>(1,1)/intrinsic64f.at<double>(0,0);
            calibrated_params.ppx = intrinsic64f.at<double>(0,2);
            calibrated_params.ppy = intrinsic64f.at<double>(1,2);
        }
        if (!estimator(features, pairwise_matches, cameras_params)) {
            PLOGE << "Homography estimation failed.";
            return {};
        }

        std::unique_ptr<cv::detail::BundleAdjusterBase> adjuster;
        if (m_bundle_adjustor == BundleAdjustor::REPROJ) adjuster.reset(new cv::detail::BundleAdjusterReproj());
        else if (m_bundle_adjustor == BundleAdjustor::RAY) adjuster.reset( new cv::detail::BundleAdjusterRay());
        else if (m_bundle_adjustor == BundleAdjustor::NO) adjuster.reset( new cv::detail::NoBundleAdjuster());

        adjuster->setConfThresh(m_conf_thresh);

        cv::Mat_<uchar> refine_mask = cv::Mat::zeros(3, 3, CV_8U);
        /*
        refine_mask(0, 0) = 1;
        refine_mask(0, 1) = 1;
        refine_mask(0, 2) = 1;
        refine_mask(1, 1) = 1;
        refine_mask(1, 2) = 1;
         */
        adjuster->setRefinementMask(refine_mask);

         // Bundle adjuster
        for (auto & params : cameras_params){params.R.convertTo(params.R, CV_32F);} // Patch for adjuster
        if (!(*adjuster)(features, pairwise_matches, cameras_params)) {
            PLOGE << "IntrinsicCamera parameters adjusting failed.";
            return {};
        }

        std::vector<cv::Mat> rmats;
        for (size_t i = 0; i < cameras_params.size(); ++i)
            rmats.push_back(cameras_params[i].R.clone());
        cv::detail::waveCorrect(rmats, cv::detail::WAVE_CORRECT_HORIZ);
        for (size_t i = 0; i < cameras_params.size(); ++i)
            cameras_params[i].R = rmats[i];

        // Find median focal length
        std::vector<float> focals(_cameras.size());
        for (int i=0; i< _cameras.size(); i++)
            focals[i] = _cameras.at(i)->get_focal();
        float median_focal = get_median(focals);

        std::vector<RotationCamera> result_rot_cams;
        for (int i =0; i<features.size(); i++) {
            const int &cam_idx = features.at(i).img_idx;
            cv::detail::CameraParams& calibrated_params = cameras_params.at(i);
            ExtrinsicCamera extrinsic_cam(*_cameras[cam_idx], calibrated_params.K());
            RotationCamera cyl_cam(extrinsic_cam, calibrated_params.R, median_focal);
            result_rot_cams.push_back(cyl_cam);
            PLOGI << "Camera '"<< _cameras[i]->get_name() << "': ";
            PLOGI << "R:\n" << calibrated_params.R;
            PLOGI << "K:\n" << calibrated_params.K();
        }
        PLOGI << "Median focal: " << median_focal;

        return result_rot_cams;
    }
} // namespace laz {