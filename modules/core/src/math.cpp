#include "core/math.h"

namespace math{
    void homogenous_to_cartesian(cv::InputArray _src, cv::OutputArray _dst)
    {
        cv::Mat h_mat = _src.getMat();
        h_mat.convertTo(h_mat, CV_64FC1);
        assert(h_mat.rows == 3);
        cv::Mat c_mat(2, h_mat.cols, CV_64FC1);

        for(int i=0; i<c_mat.rows; i++)
        {
            cv::Vec3d h_point = h_mat.at<cv::Vec3d>(i);
            c_mat.at<cv::Vec2d>(i) = cv::Vec2d(h_point[0]/h_point[2], h_point[1]/h_point[2]);
        }

        c_mat.convertTo(c_mat, _src.type());
        c_mat.copyTo(_dst);
    }

    cv::Rect to_bbox(cv::InputArray _src)
    {
        cv::Mat vectors = _src.getMat();
        vectors.convertTo(vectors, CV_64FC1);
        assert(vectors.rows == 3 or vectors.rows == 2);
        assert(vectors.cols > 1);

        cv::Mat cartesian_vectors;
        if (vectors.rows == 3)
            homogenous_to_cartesian(vectors, cartesian_vectors);
        else
            cartesian_vectors = vectors;

        double xmin=0, xmax=0, ymin=0, ymax=0;
        cv::minMaxLoc(cartesian_vectors.row(0), &xmin, &xmax, nullptr, nullptr);
        cv::minMaxLoc(cartesian_vectors.row(1), &ymin, &ymax, nullptr, nullptr);
        return cv::Rect(xmin,ymin,xmax-xmin,ymax-ymin);
    }
} // namespace math

