#include "stitching/stitcher.h"
#include <cmath>


namespace laz {

    Stitcher::Stitcher(const StitcherComponents& _components) : m_components(_components)
    {
        this->init_blender();
    }

    void Stitcher::init(const std::vector<cv::Mat>& _src)
    {
        std::vector<cv::Mat> image_bundle = _src;
        const std::vector<cv::Mat>& mask_bundle = m_components.streamer->get_mask_bundle();

        // Gamma Corrector
        if (m_components.gamma_corrector)
            // No Initialization to do here
            m_components.gamma_corrector->apply(image_bundle, mask_bundle);

        // Compensator
        if (m_components.exp_compensator)
        {
            this->init_exp_compensator(image_bundle);
            m_components.exp_compensator->apply(image_bundle);
        }

        // Seam Finder
        if (m_components.seam_finder)
            this->init_seam_finder(image_bundle);
    }

    void Stitcher::init_exp_compensator(const std::vector<cv::Mat>& _src)
    {
        const std::vector<cv::Mat>& mask_bundle = m_components.streamer->get_mask_bundle();
        const std::vector<cv::Rect>& corners_bundle = m_components.streamer->get_corners_bundle();
        m_components.exp_compensator->init(_src, mask_bundle, corners_bundle);
    }

    void Stitcher::init_seam_finder(const std::vector<cv::Mat>& _src)
    {
        const std::vector<cv::Mat>& mask_bundle = m_components.streamer->get_mask_bundle();
        const std::vector<cv::Rect>& corners_bundle = m_components.streamer->get_corners_bundle();
        m_components.seam_finder->init(_src, mask_bundle, corners_bundle);
    }

    void Stitcher::init_blender()
    {
        const std::vector<cv::Mat>& mask_bundle = m_components.streamer->get_mask_bundle();
        const std::vector<cv::Rect>& corners_bundle = m_components.streamer->get_corners_bundle();
        const std::vector<cv::Size>& size_bundle = m_components.streamer->get_size_bundle();
        m_components.blender->init(mask_bundle, corners_bundle, size_bundle);
    }

    void Stitcher::init_from_current_stream()
    {
        // Read a first batch of image and reset the Bundler to avoid skipping a frame in the stitching process.
        // ex. in case of CameraFakeStreamer
        std::vector<cv::Mat> initializer_bundle = m_components.streamer->read();
        m_components.streamer->reset();
        this->init(initializer_bundle);
    }

    bool Stitcher::read(cv::OutputArray _dst,
                        const bool& _do_update_exposure,
                        const bool& _do_update_seams)
    {

        std::vector<cv::Mat> img_bundle = m_components.streamer->read();
        for(auto& mat : img_bundle)
            if (mat.empty())
                return false;

        const std::vector <cv::Mat>& mask_bundle = m_components.seam_finder->get_seam_masks();

        // Gamma Corrector
        if (m_components.gamma_corrector)
            m_components.gamma_corrector->apply(img_bundle, mask_bundle);

        // Compensator
        if (m_components.exp_compensator){
            if (_do_update_exposure)
                this->init_exp_compensator(img_bundle);
            m_components.exp_compensator->apply(img_bundle);
        }

        // Seam Finder
        if (m_components.seam_finder){
            if (_do_update_seams)
                this->init_seam_finder(img_bundle);
            const std::vector <cv::Mat>& updated_seam_masks = m_components.seam_finder->get_seam_masks();
            m_components.blender->update_masks(updated_seam_masks);
        }

        cv::Mat result, result_mask;
        m_components.blender->blend(img_bundle, _dst);
        return !_dst.empty();
    }
} //namespace laz

