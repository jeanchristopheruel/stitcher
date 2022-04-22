#ifndef LIVESTITCHER_BLENDER_H
#define LIVESTITCHER_BLENDER_H
#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/stitching/detail/blenders.hpp>
#include <opencv2/stitching/detail/util.hpp>

#include <plog/Log.h>

namespace laz {

    class Blender {
    public:
        Blender() = default;
        virtual ~Blender(){};
        virtual void init(const std::vector <cv::Mat>& _mask_bundle,
                          const std::vector <cv::Rect>& _corners_bundle,
                          const std::vector <cv::Size>& _size_bundle) = 0;

        virtual void update_masks(const std::vector <cv::Mat>& _mask_bundle);
        virtual void update_corners(const std::vector <cv::Rect>& _corners_bundle);
        virtual void update_sizes( const std::vector <cv::Size>& _size_bundle);

        virtual void blend(const std::vector <cv::Mat>& _images_bundle, cv::OutputArray _dst) = 0;

    protected:
        std::vector <cv::Mat> m_mask_bundle;
        std::vector <cv::Point> m_tl_point_bundle;
        std::vector <cv::Size> m_size_bundle;
    };

    class CvBlender : public Blender {
    public:
        CvBlender(const float& _blend_strength=5.f);

        virtual void init(const std::vector <cv::Mat>& _mask_bundle,
                          const std::vector <cv::Rect>& _corners_bundle,
                          const std::vector <cv::Size>& _size_bundle) override;

        virtual void blend(const std::vector <cv::Mat>& _images_bundle, cv::OutputArray _dst) override;

    private:
        float get_blend_width(const float& _blend_strength);

    protected:
        float m_blend_width;
        cv::Ptr<cv::detail::Blender> m_blender;
    };

    class CvBlenderFeather : public CvBlender {
    public:
        explicit CvBlenderFeather(const float& _blend_strength=5.f);
    };

    class CvBlenderMultiBand : public CvBlender {
    public:
        explicit CvBlenderMultiBand(const float& _blend_strength=5.f);
    };

} // namespace laz

#endif //LIVESTITCHER_BLENDER_H
