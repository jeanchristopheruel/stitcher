#include "core/camerastream.h"

namespace laz{

    CameraStream::~CameraStream()
    {
        this->disconnect();
    }

    StreamStatus CameraStream::connect()
    {
        PLOGI << "Connecting camera '" << this->get_name() << "'...";
        const StreamStatus ret = this->_connect();
        switch (ret) {
            case StreamStatus::CONNECTED:
                PLOGI << "Camera '" << this->get_name() << "' connected.";
                break;
            case StreamStatus::DISCONNECTED:
                PLOGE << "Camera '" << this->get_name() << "' still disconnected.";
                break;
            case StreamStatus::FAILED:
                PLOGE << "Camera '" << this->get_name() << "' failed to connect.";
                break;
            case StreamStatus::FINISHED:
                PLOGE << "Camera '" << this->get_name() << "' finished.";
                break;
        }
        return ret;
    }

    StreamStatus CameraStream::disconnect() {
        PLOGI << "Disconnecting camera '" << this->get_name() << "'...";
        const StreamStatus ret = this->_disconnect();
        switch (ret) {
            case StreamStatus::CONNECTED:
                PLOGE << "Camera '" << this->get_name() << "' still connected.";
                break;
            case StreamStatus::DISCONNECTED:
                PLOGI << "Camera '" << this->get_name() << "' disconnected.";
                break;
            case StreamStatus::FAILED:
                PLOGE << "Camera '" << this->get_name() << "' failed to disconnect.";
                break;
            case StreamStatus::FINISHED:
                PLOGE << "Camera '" << this->get_name() << "' finished.";
                break;
        }
        return ret;
    };

    StreamStatus CameraStream::_disconnect()
    {
        this->m_status = StreamStatus::DISCONNECTED;
        return this->m_status;
    }

    StreamStatus CameraStream::_connect()
    {
        this->m_status = StreamStatus::CONNECTED;
        return this->m_status;
    }

    StreamStatus CameraFakeStream::_connect()
    {
        this->m_status = StreamStatus::CONNECTED;
        m_read_idx = 0;
        return this->m_status;
    }

    cv::Mat CameraFakeStream::read() const
    {

        if (m_read_idx == this->stream_size() - 1) return cv::Mat();
        cv::Mat remapped = this->read(m_read_idx);
        m_read_idx++;
        return remapped;
    }

    cv::Mat CameraFakeStream::read(const int& idx) const
    {
        assert(idx < this->stream_size() - 1);
        cv::Mat loaded_img = cv::imread(m_img_paths[idx], cv::IMREAD_COLOR);
        cv::Mat remapped_img;
        m_cam->remap(loaded_img, remapped_img, cv::INTER_LINEAR, cv::BORDER_REFLECT);
        return remapped_img;
    }

    cv::Mat CameraCalibration::read() const
    {
        cv::Mat loaded_img = cv::imread(m_img_path, cv::IMREAD_COLOR);
        cv::Mat remapped_img;
        this->remap(loaded_img, remapped_img, cv::INTER_LINEAR, cv::BORDER_REFLECT);
        return remapped_img;
    }
} // namespace laz{
