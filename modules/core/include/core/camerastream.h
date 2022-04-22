#ifndef LIVESTITCHER_CAMERASTREAM_H
#define LIVESTITCHER_CAMERASTREAM_H
#include <opencv2/imgcodecs.hpp>

#include "core/camera.h"
#include "core/cvcamera.h"

namespace laz {
    enum StreamStatus {
        CONNECTED = 0,      // Currently Connected
        DISCONNECTED,       // Disconnected
        FAILED,             // Connection Failed
        FINISHED            // No more images to stream
    };

    class CameraStream {
    public:
        explicit CameraStream(Camera const* _cam) : m_cam(_cam), m_status(StreamStatus::DISCONNECTED) {};
        virtual ~CameraStream();

        StreamStatus connect();
        StreamStatus disconnect();

        virtual StreamStatus get_status() const { return m_status; };
        std::string get_name() const {return m_cam->get_name();}
        cv::Mat get_mask() const {return m_cam->get_mask();}
        cv::Rect get_corners() const {return m_cam->get_corners();}
        cv::Size size() const {return m_cam->size();}

        virtual void reset(){};
        virtual cv::Mat read() const { return cv::Mat(); };

    protected:
        virtual StreamStatus _connect();
        virtual StreamStatus _disconnect();

        Camera const* m_cam;
        StreamStatus m_status;
    };


    // ----------------------------------------------------------------------------------------------
    // CameraFakeStream
    // ----------------------------------------------------------------------------------------------
    class CameraFakeStream : public CameraStream
    {
    public:
        CameraFakeStream(Camera const* _cam, const std::vector<std::string>& _img_paths):
                CameraStream(_cam), m_img_paths(_img_paths), m_read_idx(0) {};
        virtual ~CameraFakeStream() = default;

        virtual void reset() { m_read_idx = 0; }
        virtual cv::Mat read() const;
        virtual cv::Mat read(const int& idx) const;

        int stream_size() const {return m_img_paths.size();}
        std::vector<std::string> get_all_paths() const {return m_img_paths;}

    protected:
        virtual StreamStatus _connect();

    private:
        mutable int m_read_idx;
        std::vector<std::string> m_img_paths;
    };


    // ----------------------------------------------------------------------------------------------
    // CameraCalibration
    // ----------------------------------------------------------------------------------------------
    class CameraCalibration : public IntrinsicCamera
    {
    public:
        CameraCalibration(const IntrinsicCamera& _cam, const std::string& _img_path):
                IntrinsicCamera(_cam), m_img_path(_img_path){};

        cv::Mat read() const;

    private:
        std::string m_img_path;
    };

} //namespace laz
#endif //LIVESTITCHER_CAMERASTREAM_H
