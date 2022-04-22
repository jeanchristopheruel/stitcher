#include "core/cvcamera.h"

namespace laz {
    // ----------------------------------------------------------------------------------------------
    // CvCylindricalCamera
    // ----------------------------------------------------------------------------------------------
    CvCylindricalCamera::CvCylindricalCamera(const RotationCamera& _cam) : RotationCamera(_cam)
    {
        this->init_cylindrical_maps();
    }

    void CvCylindricalCamera::init_cylindrical_maps()
    {
        cv::Mat extrinsic_32f, rotation_32f;
        m_rotation.convertTo(rotation_32f, CV_32FC1);
        m_extrinsic.convertTo(extrinsic_32f, CV_32FC1);

        // Cylindrical Projection
        cv::Ptr<cv::WarperCreator> warper_creator = this->create_cvwarper();
        cv::Ptr<cv::detail::RotationWarper> warper = warper_creator->create(static_cast<float>(m_rot_radius));

        warper->buildMaps(m_dims, extrinsic_32f, rotation_32f, m_warp_mapx, m_warp_mapy);
        // Merge maps
        cv::remap(this->m_mapx, this->m_mapx, m_warp_mapx, m_warp_mapy, cv::INTER_LINEAR, cv::BORDER_TRANSPARENT);
        cv::remap(this->m_mapy, this->m_mapy, m_warp_mapx, m_warp_mapy, cv::INTER_LINEAR, cv::BORDER_TRANSPARENT);
    }

    cv::Rect CvCylindricalCamera::get_corners() const
    {
        cv::Mat extrinsic_32f, rotation_32f;
        m_rotation.convertTo(rotation_32f, CV_32FC1);
        m_extrinsic.convertTo(extrinsic_32f, CV_32FC1);

        // Cylindrical Projection
        cv::Ptr<cv::WarperCreator> warper_creator = this->create_cvwarper();
        cv::Ptr<cv::detail::RotationWarper> warper = warper_creator->create(static_cast<float>(m_rot_radius));
        return warper->warpRoi(m_dims, extrinsic_32f, rotation_32f);
    }

    cv::Mat CvCylindricalCamera::get_mask() const
    {
        // Create mask
        cv::Mat mask(m_dims, CV_8UC1, cv::Scalar(255)), warped_mask;

        cv::Mat extrinsic_32f, rotation_32f;
        m_rotation.convertTo(rotation_32f, CV_32FC1);
        m_extrinsic.convertTo(extrinsic_32f, CV_32FC1);

        // Merge maps
        cv::Mat maskmapx, maskmapy;
        cv::remap(mask, warped_mask, m_warp_mapx, m_warp_mapy, cv::INTER_NEAREST, cv::BORDER_CONSTANT, cv::Scalar(0));
        return warped_mask;
    }

    cv::Ptr<cv::WarperCreator> CvCylindricalCamera::create_cvwarper() const
    {
        return cv::makePtr<cv::CylindricalWarper>();
    }

    cv::Ptr<cv::WarperCreator> CvSphericalCamera::create_cvwarper() const
    {
        return cv::makePtr<cv::SphericalWarper>();
    }

} // namespace laz