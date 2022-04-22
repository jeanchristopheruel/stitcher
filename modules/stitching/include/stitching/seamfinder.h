#ifndef LIVESTITCHER_SEAMFINDER_H
#define LIVESTITCHER_SEAMFINDER_H
#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/stitching/detail/seam_finders.hpp>

#include <plog/Log.h>

namespace laz {

    class SeamFinder {
    public:
        SeamFinder(const float& _seam_downscale=0.5f): m_seam_downscale(_seam_downscale){};
        virtual ~SeamFinder() = default;
        virtual void  init(const std::vector <cv::Mat>& img_bundle,
                           const std::vector <cv::Mat>& mask_bundle,
                           const std::vector <cv::Rect>& corners_bundle) = 0;
        std::vector <cv::Mat> get_seam_masks() const { return m_seam_masks; };

    protected:
        std::vector <cv::Mat> m_seam_masks;
        float m_seam_downscale;
    };

    class CvSeamFinder : public SeamFinder {
    public:
        CvSeamFinder(const float& _seam_downscale=0.5f) : m_seam_finder(nullptr), SeamFinder(_seam_downscale) {};
        virtual ~CvSeamFinder() = default;
        virtual void init(const std::vector <cv::Mat>& img_bundle,
                          const std::vector <cv::Mat>& mask_bundle,
                          const std::vector <cv::Rect>& corners_bundle) override;

    protected:
        cv::Ptr<cv::detail::SeamFinder> m_seam_finder;
    };

    class CvSeamFinderVoronoi : public CvSeamFinder {
    public:
        explicit CvSeamFinderVoronoi(const float& _seam_downscale=0.5f);
        virtual ~CvSeamFinderVoronoi() = default;
    };

    class CvSeamFinderGcColor : public CvSeamFinder {
    public:
        explicit CvSeamFinderGcColor(const float& _seam_downscale=0.5f);
        virtual ~CvSeamFinderGcColor() = default;
    };

    class CvSeamFinderGcColorGrad : public CvSeamFinder {
    public:
        explicit CvSeamFinderGcColorGrad(const float& _seam_downscale=0.5f);
        virtual ~CvSeamFinderGcColorGrad() = default;
    };

    class CvSeamFinderDpColor : public CvSeamFinder {
    public:
        explicit CvSeamFinderDpColor(const float& _seam_downscale=0.5f);
        virtual ~CvSeamFinderDpColor() = default;
    };

    class CvSeamFinderDpColorGrad : public CvSeamFinder {
    public:
        explicit CvSeamFinderDpColorGrad(const float& _seam_downscale=0.5f);
        virtual ~CvSeamFinderDpColorGrad() = default;
    };
}

#endif //LIVESTITCHER_SEAMFINDER_H
