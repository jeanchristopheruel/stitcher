#include "core/streambundler.h"

namespace laz {

    // ----------------------------------------------------------------------------------------------
    // StreamBundler
    // ----------------------------------------------------------------------------------------------
    StreamBundler::StreamBundler(const std::vector<CameraStream*>& _streams) : m_streams(_streams)
    {
        this->init_cache();
    }

    StreamBundler::~StreamBundler()
    {
        this->disconnect();
    }

    void StreamBundler::init_cache() const
    {
        get_mask_bundle();
        get_corners_bundle();
        get_size_bundle();
    }

    void StreamBundler::reset()
    {
        for (CameraStream* stream : m_streams)
            stream->reset();
    }

    std::vector<cv::Mat> StreamBundler::read() const {
        std::vector<cv::Mat> img_bundle(m_streams.size());

        if (this->get_status() != StreamStatus::CONNECTED)
        {
            PLOGW << "Tried to read from unconnected bundle. Abort.";
            return img_bundle;
        }

        for (int i=0; i<m_streams.size(); i++)
        {
            if (m_streams.at(i)->get_status() != StreamStatus::CONNECTED)
            {
                PLOGW << "Tried to read from unconnected camera '"<<m_streams.at(i)->get_name()<<". Abort.";
                return img_bundle;
            }
            img_bundle[i] = m_streams.at(i)->read();
        }
        return img_bundle;
    }

    std::vector<cv::Mat> StreamBundler::get_mask_bundle() const {
        if (!m_cache.mask_bundle.empty())
            return m_cache.mask_bundle;

        std::vector<cv::Mat> mask_bundle(m_streams.size());
        for (int i=0; i<m_streams.size(); i++)
            mask_bundle[i] = m_streams.at(i)->get_mask();
        m_cache.mask_bundle = mask_bundle;
        return mask_bundle;
    }

    std::vector<cv::Rect> StreamBundler::get_corners_bundle() const {
        if (!m_cache.corners_bundle.empty())
            return m_cache.corners_bundle;

        std::vector<cv::Rect> corners_bundle(m_streams.size());
        for (int i=0; i<m_streams.size(); i++)
            corners_bundle[i] = m_streams.at(i)->get_corners();
        m_cache.corners_bundle = corners_bundle;
        return corners_bundle;
    }

    std::vector<cv::Size> StreamBundler::get_size_bundle() const {
        if (!m_cache.size_bundle.empty())
            return m_cache.size_bundle;

        std::vector<cv::Size> size_bundle(m_streams.size());
        for (int i=0; i<m_streams.size(); i++)
            size_bundle[i] = m_streams.at(i)->size();
        m_cache.size_bundle = size_bundle;
        return size_bundle;
    }
} // namespace laz