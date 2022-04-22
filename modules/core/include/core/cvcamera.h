#ifndef LIVESTITCHER_CVCAMERA_H
#define LIVESTITCHER_CVCAMERA_H
#include <opencv2/core.hpp>
#include <opencv2/stitching/detail/warpers.hpp>
#include <opencv2/stitching/warpers.hpp>

#include "core/camera.h"

namespace laz {
    class CvCylindricalCamera : public RotationCamera {
    public:
        explicit CvCylindricalCamera(const RotationCamera& _cam);
        virtual ~CvCylindricalCamera() = default;

        /*
        virtual void remap(cv::InputArray &_src, cv::OutputArray &_dst,
                           const int &interpolation=cv::INTER_LINEAR,
                           const int &borderMode=cv::BORDER_REFLECT) const;
         */

        virtual cv::Mat get_mask() const override;
        virtual cv::Rect get_corners() const override;

    protected:
        cv::Rect m_corners;

    private:
        void init_cylindrical_maps();
        virtual cv::Ptr<cv::WarperCreator> create_cvwarper() const;
    };

    class CvSphericalCamera : public CvCylindricalCamera {
    public:
        explicit CvSphericalCamera(const RotationCamera &_cam) : CvCylindricalCamera(_cam) {};
        virtual ~CvSphericalCamera() = default;

    private:
        virtual cv::Ptr<cv::WarperCreator> create_cvwarper() const override;
    };
} // namespace laz

#endif //LIVESTITCHER_CVCAMERA_H
