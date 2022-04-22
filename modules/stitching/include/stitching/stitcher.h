//
// Created by jcruel on 2021-01-16.
//

#ifndef LIVESTITCHER_STITCHER_H
#define LIVESTITCHER_STITCHER_H
#include <vector>
#include <memory>
#include <cmath>

#include <plog/Log.h>

#include "core/streambundler.h"
#include "stitching/gammacorrector.h"
#include "stitching/exposurecompensator.h"
#include "stitching/seamfinder.h"
#include "stitching/blender.h"

namespace laz {

    class Stitcher {
    private:
        class StitcherComponents {

        private:

            StitcherComponents() : streamer(nullptr),
                                   gamma_corrector(nullptr),
                                   exp_compensator(nullptr),
                                   seam_finder(nullptr),
                                   blender(nullptr)
                                   {}

            StreamBundler* streamer;
            GammaCorrector* gamma_corrector;
            ExposureCompensator* exp_compensator;
            SeamFinder* seam_finder;
            Blender* blender;

            friend class Stitcher;
            friend class StitcherBuilder;
        };

    public:
        class StitcherBuilder {
        public:

            StitcherBuilder(StreamBundler* _streamer, Blender* _blender) {
                assert(_streamer != nullptr);
                assert(_blender != nullptr);
                m_components.streamer = _streamer;
                m_components.blender = _blender;
            }

            StitcherBuilder &attach_gamma_corrector(GammaCorrector* _gamma_corrector) {
                assert(_gamma_corrector != nullptr);
                m_components.gamma_corrector = _gamma_corrector;
                return *this;
            }

            StitcherBuilder &attach_exp_compensator(ExposureCompensator* _compensator) {
                assert(_compensator != nullptr);
                m_components.exp_compensator = _compensator;
                return *this;
            }

            StitcherBuilder &attach_seam_finder(SeamFinder* _seam_finder) {
                assert(_seam_finder != nullptr);
                m_components.seam_finder = _seam_finder;
                return *this;
            }

            Stitcher build() {
                return Stitcher(m_components);
            }

        private:
            StitcherComponents m_components;
        };

        /**
         * Initialize Stitching using a given vector of calibration images for each Camera Stream.
         * @param _src : vector of calibration images
         */
        void init(const std::vector<cv::Mat>& _src);

        void init_exp_compensator(const std::vector<cv::Mat>& _src);

        void init_seam_finder(const std::vector<cv::Mat>& _src);

        void init_blender();

        /**
        * Initialize Stitching using the first images for each Camera Stream.
        */
        void init_from_current_stream();

        /**
         * Read from Camera Stream and stitch the stream.
         * @return Stitched mosaic
         */
        bool read(cv::OutputArray _dst,
                  const bool& _do_update_exposure=false,
                  const bool& _do_update_seams=false);

    private:
        Stitcher(const StitcherComponents& _components);

        StitcherComponents m_components;
    };
} // namespace laz

#endif //LIVESTITCHER_STITCHER_H
