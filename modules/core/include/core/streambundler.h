#ifndef LIVESTITCHER_STREAMBUNDLER_H
#define LIVESTITCHER_STREAMBUNDLER_H
#include <memory.h>
#include "core/camerastream.h"
#include "nlohmann/json.hpp"

namespace laz {

    class StreamBundler {
    public:
        StreamBundler(const std::vector<CameraStream*>& _streams);
        virtual ~StreamBundler();

        virtual StreamStatus connect() {
            PLOGI << "Connecting Bundle.";
            for (int i = 0; i < m_streams.size(); i++) {
                const StreamStatus &ret = m_streams.at(i)->connect();
                if (ret != StreamStatus::CONNECTED) {
                    PLOGE << "Failed to connect Bundle.";
                    this->disconnect();
                    m_bundle_status = StreamStatus::FAILED;
                    return m_bundle_status;
                }
            }
            PLOGI << "Bundle successfully connected.";
            m_bundle_status = StreamStatus::CONNECTED;
            return m_bundle_status;
        }

        virtual StreamStatus disconnect() {
            PLOGI << "Disconnecting Bundle.";
            for (int i = 0; i < m_streams.size(); i++) {
                const StreamStatus &ret = m_streams.at(i)->disconnect();
                if (ret != StreamStatus::DISCONNECTED) {
                    PLOGE << "Failed to disconnect Bundle.";
                    this->disconnect();
                    m_bundle_status = StreamStatus::FAILED;
                    return m_bundle_status;
                }
            }
            PLOGI << "Bundle successfully disconnected.";
            m_bundle_status = StreamStatus::DISCONNECTED;
            return m_bundle_status;
        };

        virtual StreamStatus get_status() const { return m_bundle_status; };

        void reset();

        int size() const {return m_streams.size(); };

        std::vector<cv::Mat> read() const;

        std::vector<cv::Mat> get_mask_bundle() const;

        std::vector<cv::Rect> get_corners_bundle() const;

        std::vector<cv::Size> get_size_bundle() const;



        //nlohmann::json to_json(const std::string &calibration_path) const;

    protected:
        void init_cache() const;

        mutable StreamStatus m_bundle_status;

        class CachedBundle {
        public:
            std::vector<cv::Mat> mask_bundle;
            std::vector<cv::Rect> corners_bundle;
            std::vector<cv::Size> size_bundle;

            void reset() {
                corners_bundle.clear();
                mask_bundle.clear();
                size_bundle.clear();
            }
        };

        mutable CachedBundle m_cache;
        std::vector<CameraStream*> m_streams{};
    };
} // namespace laz
#endif //LIVESTITCHER_STREAMBUNDLER_H
