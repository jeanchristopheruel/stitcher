#include "stitching/gammacorrector.h"
#include "assert.h"


namespace laz {

    GammaCorrector::GammaCorrector(const float& _alpha, const uint8_t& _beta) : m_alpha(_alpha), m_beta(_beta)
    {
        assert(m_alpha >= 1.f and m_alpha <= 3.f);
        assert(m_beta >= 0 and m_beta <= 100);
    }

    void GammaCorrector::apply(std::vector <cv::Mat> &_img_bundle, const std::vector<cv::Mat>& _mask_bundle) 
    {
        assert(_img_bundle.size() == _mask_bundle.size());

        PLOGI << "Applying gamma correction.";
        for (int i=0; i<_img_bundle.size();i++)
        {
            auto& img = _img_bundle.at(i);
            auto& mask = _mask_bundle.at(i);

            cv::Mat mask_with_channels;
            std::vector<cv::Mat> channels(img.channels());
            for(int i=0;i<channels.size();i++) channels[i] = mask;
            cv::merge(channels, mask_with_channels);

            cv::min(m_alpha * img + m_beta, mask_with_channels, img);
        }
    }
}  //namespace laz
