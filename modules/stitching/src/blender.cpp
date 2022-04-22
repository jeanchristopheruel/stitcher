#include "stitching/blender.h"
#include "cmath"

namespace laz {

    void Blender::update_masks(const std::vector <cv::Mat> &_mask_bundle)
    {
        m_mask_bundle = _mask_bundle;
    }

    void Blender::update_corners(const std::vector <cv::Rect>& _corners_bundle)
    {
        std::vector<cv::Point> tl_point_bundle(_corners_bundle.size());
        for (int i = 0; i < tl_point_bundle.size(); i++)
            tl_point_bundle[i] = _corners_bundle.at(i).tl();
        m_tl_point_bundle = tl_point_bundle;
    }

    void Blender::update_sizes(const std::vector <cv::Size>& _size_bundle)
    {
        m_size_bundle = _size_bundle;
    }

    CvBlender::CvBlender(const float& _blend_strength) : m_blender(nullptr)
    {
        m_blend_width = this->get_blend_width(_blend_strength);
        if (not m_blend_width >= 1.f){
            std::string msg = "Error: blend width is below 1.0: " + std::to_string(m_blend_width) + "\n";
            PLOGE << msg;
            throw std::runtime_error(msg);
        }
    }

    float CvBlender::get_blend_width(const float& _blend_strength)
    {
        cv::Size dst_sz = cv::detail::resultRoi(m_tl_point_bundle, m_size_bundle).size();
        return std::sqrt(static_cast<float>(dst_sz.area())) * _blend_strength / 100.f;
    }

    void CvBlender::init(const std::vector <cv::Mat>& _mask_bundle,
                         const std::vector <cv::Rect>& _corners_bundle,
                         const std::vector <cv::Size>& _size_bundle)
    {
        std::vector<cv::Point> tl_point_bundle(_corners_bundle.size());
        for (int i = 0; i < tl_point_bundle.size(); i++)
            tl_point_bundle[i] = _corners_bundle.at(i).tl();

        m_mask_bundle = _mask_bundle;
        m_tl_point_bundle = tl_point_bundle;
        m_size_bundle = _size_bundle;
    }

    void CvBlender::blend(const std::vector <cv::Mat>& _images_bundle, cv::OutputArray _dst)
    {
        assert( _images_bundle.size() == m_mask_bundle.size() );
        assert( _images_bundle.size() == m_tl_point_bundle.size() );
        assert(m_blender);

        m_blender->prepare(m_tl_point_bundle, m_size_bundle);

        for (int i=0; i< _images_bundle.size(); i++)
        {
            cv::Mat img_warped_s ;
            _images_bundle.at(i).convertTo(img_warped_s, CV_16S);
            m_blender->feed(img_warped_s, m_mask_bundle[i], m_tl_point_bundle[i]);
        }
        cv::Mat mosaic, mosaic_mask;
        m_blender->blend(mosaic, mosaic_mask);
        _dst.assign(mosaic);
    }

    CvBlenderFeather::CvBlenderFeather(const float& _blend_strength) : CvBlender(_blend_strength) {
        m_blender = cv::detail::Blender::createDefault(cv::detail::Blender::FEATHER, false);
        cv::detail::FeatherBlender* fb = dynamic_cast<cv::detail::FeatherBlender*>(m_blender.get());
        fb->setSharpness(1.f/m_blend_width);
    }

    CvBlenderMultiBand::CvBlenderMultiBand(const float& _blend_strength) : CvBlender(_blend_strength) {
        m_blender = cv::detail::Blender::createDefault(cv::detail::Blender::MULTI_BAND, false);
        cv::detail::MultiBandBlender* mb = dynamic_cast<cv::detail::MultiBandBlender*>(m_blender.get());
        mb->setNumBands(static_cast<int>(std::ceil(std::log(m_blend_width)/std::log(2.)) - 1.));
    }

}
