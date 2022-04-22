#include "stitching/exposurecompensator.h"
#include "assert.h"

namespace laz {
    void CvExposureCompensator::init(const std::vector <cv::Mat>& img_bundle,
                                     const std::vector <cv::Mat>& mask_bundle,
                                     const std::vector <cv::Rect>& corners_bundle) {
        assert(img_bundle.size() == corners_bundle.size());
        assert(img_bundle.size() == mask_bundle.size());

        std::vector<cv::UMat> img_Ubundle(mask_bundle.size());
        for (int i = 0; i < img_Ubundle.size(); i++)
            img_Ubundle[i] = img_bundle.at(i).getUMat(cv::ACCESS_FAST);

        std::vector<cv::UMat> mask_Ubundle(mask_bundle.size());
        for (int i = 0; i < mask_Ubundle.size(); i++)
            mask_Ubundle[i] = mask_bundle.at(i).getUMat(cv::ACCESS_FAST);

        std::vector<cv::Point> tl_point_bundle(corners_bundle.size());
        for (int i = 0; i < tl_point_bundle.size(); i++)
            tl_point_bundle[i] = corners_bundle.at(i).tl();

        m_mask_bundle = mask_bundle;
        m_tl_point_bundle = tl_point_bundle;
        PLOGI << "Initializing exposition compensation...";
        m_compensator->feed(tl_point_bundle, img_Ubundle, mask_Ubundle);
        PLOGI << "Initializing exposition compensation SUCCESS";
    }

    void CvExposureCompensator::apply(std::vector <cv::Mat>& _img_bundle) const
    {
        assert( _img_bundle.size() == m_mask_bundle.size() );
        assert( _img_bundle.size() == m_tl_point_bundle.size() );
        PLOGI << "Applying exposition compensation.";

        // Compensate exposure
        for (int i = 0; i < m_mask_bundle.size(); i++)
            m_compensator->apply(i, m_tl_point_bundle[i], _img_bundle[i], m_mask_bundle[i]);
    }

    CvExposureCompensatorGain::CvExposureCompensatorGain(const int &nr_feeds) {
        m_compensator = cv::detail::ExposureCompensator::createDefault(
                cv::detail::ExposureCompensator::GAIN);
        cv::detail::GainCompensator *gcompensator = dynamic_cast<cv::detail::GainCompensator *>(m_compensator.get());
        gcompensator->setNrFeeds(nr_feeds);
    }

    CvExposureCompensatorChannels::CvExposureCompensatorChannels(const int &nr_feeds) {
        m_compensator = cv::detail::ExposureCompensator::createDefault(
                cv::detail::ExposureCompensator::CHANNELS);
        cv::detail::ChannelsCompensator *ccompensator = dynamic_cast<cv::detail::ChannelsCompensator *>(m_compensator.get());
        ccompensator->setNrFeeds(nr_feeds);
    }

    CvExposureCompensatorGainBlocks::CvExposureCompensatorGainBlocks(const int &nr_feeds,
                                                                     const int &nr_gains,
                                                                     const int &block_size) {
        m_compensator = cv::detail::ExposureCompensator::createDefault(
                cv::detail::ExposureCompensator::GAIN_BLOCKS);
        cv::detail::BlocksCompensator *bcompensator = dynamic_cast<cv::detail::BlocksCompensator *>(m_compensator.get());
        bcompensator->setNrFeeds(nr_feeds);
        bcompensator->setNrGainsFilteringIterations(nr_gains);
        bcompensator->setBlockSize(block_size, block_size);
    }

    CvExposureCompensatorChannelsBlocks::CvExposureCompensatorChannelsBlocks(const int &nr_feeds,
                                                                             const int &nr_gains,
                                                                             const int &block_size) {
        m_compensator = cv::detail::ExposureCompensator::createDefault(
                cv::detail::ExposureCompensator::CHANNELS_BLOCKS);
        cv::detail::BlocksCompensator *bcompensator = dynamic_cast<cv::detail::BlocksCompensator *>(m_compensator.get());
        bcompensator->setNrFeeds(nr_feeds);
        bcompensator->setNrGainsFilteringIterations(nr_gains);
        bcompensator->setBlockSize(block_size, block_size);
    }

} // namespace laz