#ifndef LIVESTITCHER_LIGHTENINGCOMPENSATOR_H
#define LIVESTITCHER_LIGHTENINGCOMPENSATOR_H
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <plog/Log.h>

namespace laz {

    class GammaCorrector {
    public:
        /**
         *
         * @param alpha : Simple contrast control [1.0-3.0]
         * @param beta : Simple brightness control [0-100]
         */
        GammaCorrector(const float& _alpha, const uint8_t& _beta);
        virtual ~GammaCorrector() = default;

        virtual void apply(std::vector <cv::Mat> &_img_bundle, const std::vector<cv::Mat>& _mask_bundle);

    protected:
        float m_alpha;
        uint8_t m_beta;

        std::vector<cv::Mat> m_gamma_maps;
    };
} //namespace laz
#endif //LIVESTITCHER_LIGHTENINGCOMPENSATOR_H
