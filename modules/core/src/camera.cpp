#include "core/camera.h"
#include <cmath>

namespace laz {

    // ----------------------------------------------------------------------------------------------
    // Camera (Base Class)
    // ----------------------------------------------------------------------------------------------
    void Camera::remap(cv::InputArray &_src, cv::OutputArray &_dst,
                                const int &interpolation,
                                const int &borderMode,
                                const cv::Scalar &scalar) const
    {
        assert(!m_mapx.empty() and !m_mapy.empty());
        cv::remap(_src, _dst, m_mapx, m_mapy, interpolation, borderMode, scalar);
    }

    cv::Mat Camera::get_mask() const
    {
        // Create mask
        cv::Mat mask(m_dims, CV_8UC1, cv::Scalar(255)), warped_mask;
        // Mask must be remapped with INTER_NEAREST and BORDER_CONSTANT
        this->remap(mask, warped_mask, cv::INTER_NEAREST, cv::BORDER_CONSTANT, cv::Scalar(0));
        return warped_mask;
    }

    // ----------------------------------------------------------------------------------------------
    // IntrinsicCamera
    // ----------------------------------------------------------------------------------------------
    IntrinsicCamera::IntrinsicCamera(const Camera& _cam,
                                     cv::InputArray _intrinsic,
                                     cv::InputArray _dist_coeffs):
                                     Camera(_cam),
                                     m_intrinsic(_intrinsic.getMat().clone()),
                                     m_dist_coeffs(_dist_coeffs.getMat().clone())
    {
        this->init_undistort_maps();
    }

    void IntrinsicCamera::init_undistort_maps()
    {
        cv::initUndistortRectifyMap(m_intrinsic, m_dist_coeffs, cv::Mat(), m_intrinsic,
                                    m_dims, CV_32FC1, m_mapx, m_mapy);
    }

    float IntrinsicCamera::get_focal() const
    {
        cv::Mat intrinsic;
        m_intrinsic.convertTo(intrinsic, CV_32FC1);
        float fx = intrinsic.at<float>(0, 0) / intrinsic.at<float>(2, 2);
        //float fy = intrinsic.at<float>(1, 1) / intrinsic.at<float>(2, 2);
        return fx;
    }

    // ----------------------------------------------------------------------------------------------
    // ExtrinsicCamera
    // ----------------------------------------------------------------------------------------------
        ExtrinsicCamera::ExtrinsicCamera(const IntrinsicCamera &_cam, cv::InputArray _extrinsic) :
            IntrinsicCamera(_cam), m_extrinsic(_extrinsic.getMat().clone())
    {}

    cv::Rect ExtrinsicCamera::_get_corners() const
    {
        // Homogenous declaration
        float h_initial_corners[12] = { 0.f, 0.f, 1.f,
                                        0.f, float(m_dims.height), 1.f,
                                        float(m_dims.width), float(m_dims.height), 1.f,
                                        float(m_dims.width), 0.f, 1.f };

        cv::Mat extrinsic;
        m_extrinsic.convertTo(extrinsic, CV_32F);
        cv::Mat mat_initial_corners(4,3, CV_32F, h_initial_corners);
        cv::Mat h_target_corners = extrinsic*mat_initial_corners.t();
        PLOGD << "mat_initial_corners = " << "\n "  << mat_initial_corners << "\n\n";
        PLOGD << "m_extrinsic" "\n "  << m_extrinsic << "\n\n";
        PLOGD << "h_target_corners = " << "\n "  << h_target_corners << "\n\n";

        // Cartesian redefinition
        cv::Mat c_target_corners;
        math::homogenous_to_cartesian(h_target_corners, c_target_corners);
        return math::to_bbox(c_target_corners);
    }

    // ----------------------------------------------------------------------------------------------
    // CylindricalCamera
    // ----------------------------------------------------------------------------------------------
    CylindricalCamera::CylindricalCamera(const RotationCamera& _cam) : RotationCamera(_cam)
    {
        this->init_cylindrical_maps();
    }

    void CylindricalCamera::init_cylindrical_maps(){
        // This function returns the cylindrical warp for a given image and intrinsics matrix K
        float virtual_intrinsic_data[9] = {float(m_rot_radius), 0, float(RotationCamera::size().width/2),
                                           0, float(m_rot_radius), float(RotationCamera::size().height/2),
                                           0, 0, 1};
        cv::Mat virtual_intrinsic(3, 3, CV_32FC1, virtual_intrinsic_data);
        const cv::Mat& virtual_intrinsic_inv = virtual_intrinsic.inv();
        const cv::Mat& extrinsic_inv = m_extrinsic.inv();

        // Cylindrical Projection
        m_warp_mapx = cv::Mat(m_mapx.size(), CV_32FC1);
        m_warp_mapy = cv::Mat(m_mapy.size(), CV_32FC1);
        for(int j=0; j < m_mapx.cols; j++){
            for(int i=0; i < m_mapx.rows; i++){
                // To normalized coords
                const cv::Vec3f& n_vec = cv::Mat( virtual_intrinsic_inv * extrinsic_inv * cv::Mat(cv::Vec3f(i,j,1)) ).at<cv::Vec3f>(0);
                // Cylindrical coords
                const cv::Vec3f cyl_vec(std::sin(n_vec[0]), n_vec[1], std::cos(n_vec[0]));
                const cv::Vec3f& cyl_proj = cv::Mat( virtual_intrinsic * m_extrinsic * cv::Mat(cyl_vec) ).at<cv::Vec3f>(0);
                cv::Vec2f c_cyl_proj = cv::Vec2f(cyl_proj[0]/cyl_proj[2], cyl_proj[1]/cyl_proj[2]);

                if (c_cyl_proj[0] < 0 or c_cyl_proj[0] > m_dims.width)
                    c_cyl_proj[0] = -1.0;
                if (c_cyl_proj[1] < 0 or c_cyl_proj[1] > m_dims.height)
                    c_cyl_proj[1] = -1.0;

                m_warp_mapx.at<float>(i,j) = c_cyl_proj[0];
                m_warp_mapy.at<float>(i,j) = c_cyl_proj[1];
            }
        }

        // Merge maps
        cv::remap(this->m_mapx, this->m_mapx, m_warp_mapx, m_warp_mapy, cv::INTER_LINEAR, cv::BORDER_TRANSPARENT);
        cv::remap(this->m_mapy, this->m_mapy, m_warp_mapx, m_warp_mapy, cv::INTER_LINEAR, cv::BORDER_TRANSPARENT);
    }
} // namespace laz