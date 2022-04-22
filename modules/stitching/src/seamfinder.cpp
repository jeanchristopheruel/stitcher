#include "stitching/seamfinder.h"

namespace laz {
    void CvSeamFinder::init(const std::vector <cv::Mat>& img_bundle,
                            const std::vector <cv::Mat>& mask_bundle,
                            const std::vector <cv::Rect>& corners_bundle) {

        assert(img_bundle.size() == corners_bundle.size());
        assert(img_bundle.size() == mask_bundle.size());

        std::vector<cv::UMat> img_Ubundle(mask_bundle.size());
        for (int i = 0; i < img_Ubundle.size(); i++)
        {
            img_Ubundle[i] = img_bundle.at(i).getUMat(cv::ACCESS_FAST);
            // m_seam_downscale is used to make the seam finding faster than using the full image
            cv::resize(img_Ubundle[i], img_Ubundle[i], cv::Size(), m_seam_downscale, m_seam_downscale, cv::INTER_LINEAR_EXACT);
            img_Ubundle[i].convertTo(img_Ubundle[i], CV_32F);
        }

        std::vector<cv::UMat> mask_Ubundle(mask_bundle.size());
        for (int i = 0; i < mask_Ubundle.size(); i++)
        {
            mask_Ubundle[i] = mask_bundle.at(i).getUMat(cv::ACCESS_FAST);
            // m_seam_downscale is used to make the seam finding faster than using the full image
            cv::resize(mask_Ubundle[i], mask_Ubundle[i], cv::Size(), m_seam_downscale, m_seam_downscale, cv::INTER_NEAREST);
        }

        std::vector<cv::Point> tl_point_bundle(corners_bundle.size());
        for (int i = 0; i < tl_point_bundle.size(); i++)
            tl_point_bundle[i] = corners_bundle.at(i).tl()*m_seam_downscale;

        PLOGI << "Finding Optimal Seams...";
        m_seam_finder->find(img_Ubundle, tl_point_bundle, mask_Ubundle);
        PLOGI << "Finding Optimal Seams SUCCESS";

        std::vector<cv::Mat> mask_result(mask_bundle.size());
        for (int i =0; i < mask_bundle.size(); i++) {
            cv::Mat dilated_mask, seam_mask;
            cv::dilate(mask_Ubundle[i], dilated_mask, cv::Mat());
            cv::resize(dilated_mask, seam_mask, mask_bundle.at(i).size(), 0, 0, cv::INTER_LINEAR_EXACT);
            mask_result[i] = mask_bundle.at(i) & seam_mask;
        }
        m_seam_masks = mask_result;
    }

    CvSeamFinderVoronoi::CvSeamFinderVoronoi(const float& _seam_downscale) : CvSeamFinder(_seam_downscale)
    {
        m_seam_finder = cv::makePtr<cv::detail::VoronoiSeamFinder>();
    }

    CvSeamFinderGcColor::CvSeamFinderGcColor(const float& _seam_downscale) : CvSeamFinder(_seam_downscale)
    {
        m_seam_finder = cv::makePtr<cv::detail::GraphCutSeamFinder>(
                cv::detail::GraphCutSeamFinderBase::COST_COLOR);
    }

    CvSeamFinderGcColorGrad::CvSeamFinderGcColorGrad(const float& _seam_downscale) : CvSeamFinder(_seam_downscale)
    {
        m_seam_finder = cv::makePtr<cv::detail::GraphCutSeamFinder>(
                cv::detail::GraphCutSeamFinderBase::COST_COLOR_GRAD);
    }

    CvSeamFinderDpColor::CvSeamFinderDpColor(const float& _seam_downscale) : CvSeamFinder(_seam_downscale)
    {
        m_seam_finder = cv::makePtr<cv::detail::DpSeamFinder>(
                cv::detail::DpSeamFinder::COLOR);
    }

    CvSeamFinderDpColorGrad::CvSeamFinderDpColorGrad(const float& _seam_downscale) : CvSeamFinder(_seam_downscale)
    {
        m_seam_finder = cv::makePtr<cv::detail::DpSeamFinder>(
                cv::detail::DpSeamFinder::COLOR_GRAD);
    }

} // namespace laz

