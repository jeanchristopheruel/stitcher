#ifndef LIVESTITCHER_MATH_H
#define LIVESTITCHER_MATH_H
#include <assert.h>
#include <opencv2/core.hpp>

namespace math{
    /**
     * Convert homogenous representation of vectors to cartesian.
     * @param _src
     * @param _dst
     */
    void homogenous_to_cartesian(cv::InputArray _src, cv::OutputArray _dst);

    /**
     * Takes an homogenous or cartesian column-wise representation of vectors and compute the bounding box.
     * @param _src
     * @return cv::Rect
     */
    cv::Rect to_bbox(cv::InputArray _src);
} // namespace math





#endif //LIVESTITCHER_MATH_H
