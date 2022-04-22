#include "stitching/histogramequal.h"
#include "assert.h"


namespace laz {

    void HistogramEqualizer::apply(std::vector <cv::Mat> &_img_bundle, const std::vector <cv::Mat> &_mask_bundle)
    {
        assert(_img_bundle.size() == _mask_bundle.size());

        PLOGI << "Applying histogram equalization.";
        for (int i = 0; i < _img_bundle.size(); i++) {
            auto &img = _img_bundle.at(i);
            auto &mask = _mask_bundle.at(i);

            cv::Mat lab_image;
            cv::cvtColor(img, lab_image, cv::COLOR_BGR2Lab);

            // Extract the L channel
            std::vector <cv::Mat> lab_planes(3);
            cv::split(lab_image, lab_planes);  // now we have the L image in lab_planes[0]

            // apply the CLAHE algorithm to the L channel
            cv::Ptr <cv::CLAHE> clahe = cv::createCLAHE();
            clahe->setClipLimit(4);
            cv::Mat dst;
            clahe->apply(lab_planes[0], dst);

            // Merge the the color planes back into an Lab image
            dst.copyTo(lab_planes[0]);
            cv::merge(lab_planes, lab_image);
            cv::cvtColor(lab_image, img, cv::COLOR_Lab2BGR);
        }
    }
} //namespace laz