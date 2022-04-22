#ifndef LIVESTITCHER_CAMERA_H
#define LIVESTITCHER_CAMERA_H
#include <assert.h>
#include <vector>
#include <string>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/stitching/detail/warpers.hpp>

#include <plog/Log.h>

#include "core/math.h"

#pragma once

namespace laz {

    class Camera{
    public:
        Camera(const std::string& _name, const cv::Size& _dims) :  m_name(_name), m_dims(_dims) {};
        virtual ~Camera() {};

        void remap(cv::InputArray &_src, cv::OutputArray &_dst,
                   const int &interpolation=cv::INTER_LINEAR,
                   const int &borderMode=cv::BORDER_REFLECT,
                   const cv::Scalar &scalar=cv::Scalar(0)) const;
        virtual cv::Mat get_mask() const;
        virtual cv::Rect get_corners() const {return cv::Rect(0,0,m_dims.width, m_dims.height);};

        cv::Size size() const { return this->get_corners().size(); }
        std::string get_name() const { return m_name; }

        void set_name(const std::string& _name) { m_name = _name; }

    protected:
        std::string m_name;
        cv::Mat m_mapx, m_mapy;
        const cv::Size m_dims;
    };

    class IntrinsicCamera : public Camera {
    public:
        IntrinsicCamera(const Camera& _cam,
                        cv::InputArray _intrinsic,
                        cv::InputArray _dist_coeffs);

        IntrinsicCamera(const std::string& _name,
                        cv::InputArray _intrinsic,
                        cv::InputArray _dist_coeffs,
                        const cv::Size& _dims);

        virtual ~IntrinsicCamera() = default;

        float get_focal() const;
        cv::Mat get_intrinsic() const { return m_intrinsic; }
        cv::Mat get_dist_coeffs() const { return m_dist_coeffs; }

    private:
        void init_undistort_maps();

    protected:
        const cv::Mat m_dist_coeffs, m_intrinsic;
    };

    class ExtrinsicCamera : public IntrinsicCamera {
    public:
        ExtrinsicCamera(const IntrinsicCamera &cam, cv::InputArray _extrinsic);

        cv::Mat get_extrinsic() const { return m_extrinsic; }

    protected:
        cv::Rect _get_corners() const;
        const cv::Mat m_extrinsic;       // Rotation matrix
    };

    class RotationCamera : public ExtrinsicCamera {
    public:
        RotationCamera(const ExtrinsicCamera &_extrinsic_cam, cv::InputArray _rotation, const float& _rot_radius) :
                ExtrinsicCamera(_extrinsic_cam), m_rotation(_rotation.getMat().clone()), m_rot_radius(_rot_radius)
        {};
        ~RotationCamera() = default;

        cv::Mat get_rotation() const { return m_rotation; }
        float get_radius() const { return m_rot_radius; }

    protected:
        const float m_rot_radius;
        const cv::Mat m_rotation;
        cv::Mat m_warp_mapx, m_warp_mapy;
    };

    class CylindricalCamera : public RotationCamera {
    public:
        explicit CylindricalCamera(const RotationCamera& _cam);
        ~CylindricalCamera() = default;

    private:
        void init_cylindrical_maps();
    };
} // namespace laz
#endif //LIVESTITCHER_CAMERA_H
