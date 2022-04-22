#ifndef LIVESTITCHER_EXPOSURECOMPENSATOR_H
#define LIVESTITCHER_EXPOSURECOMPENSATOR_H
#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/stitching/detail/exposure_compensate.hpp>

#include <plog/Log.h>

namespace laz {

    class ExposureCompensator {
    public:
        ExposureCompensator() = default;
        virtual ~ExposureCompensator() = default;
        virtual void init(const std::vector <cv::Mat>& img_bundle,
                          const std::vector <cv::Mat>& mask_bundle,
                          const std::vector <cv::Rect>& corners_bundle) = 0;

        virtual void apply(std::vector <cv::Mat>& _img_bundle) const = 0;

    protected:
        std::vector <cv::Mat> m_mask_bundle;
        std::vector <cv::Point> m_tl_point_bundle;
    };

    class CvExposureCompensator : public ExposureCompensator {
    public:
        CvExposureCompensator() = default;
        virtual ~CvExposureCompensator() = default;
        virtual void init(const std::vector <cv::Mat>& img_bundle,
                          const std::vector <cv::Mat>& mask_bundle,
                          const std::vector <cv::Rect>& corners_bundle) override;

        virtual void apply(std::vector <cv::Mat>& _img_bundle) const override;

    protected:
        cv::Ptr<cv::detail::ExposureCompensator> m_compensator{};
    };

    class CvExposureCompensatorGain : public CvExposureCompensator {
    public:
        CvExposureCompensatorGain(const int &nr_feeds = 1);
        virtual ~CvExposureCompensatorGain() = default;
    };

    class CvExposureCompensatorChannels : public CvExposureCompensator {
    public:
        CvExposureCompensatorChannels(const int &nr_feeds = 1);
        virtual ~CvExposureCompensatorChannels() = default;
    };

    class CvExposureCompensatorGainBlocks : public CvExposureCompensator {
    public:
        CvExposureCompensatorGainBlocks(const int &nr_feeds = 1, const int &nr_gains = 2, const int &block_size = 32);
        virtual ~CvExposureCompensatorGainBlocks() = default;
    };

    class CvExposureCompensatorChannelsBlocks : public CvExposureCompensator {
    public:
        CvExposureCompensatorChannelsBlocks(const int &nr_feeds = 1, const int &nr_gains = 2,
                                            const int &block_size = 32);
        virtual ~CvExposureCompensatorChannelsBlocks() = default;
    };

} //namespace laz

#endif //LIVESTITCHER_EXPOSURECOMPENSATOR_H
