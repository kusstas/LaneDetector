#ifndef LANEDETECTOR_H
#define LANEDETECTOR_H

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

#include <vector>


class LaneDetector
{
public:
    using Filter = std::vector<float>;

    LaneDetector();

    cv::Size const& targetSize() const;

    void setKernelBlur(int kernel);
    void setKernelloseMorph(cv::Mat const& kernel);
    void addFilter(Filter filter);
    void setTargetSize(cv::Size const& size);

    void setThresholdCanny(std::pair<int, int> const& thresholdCanny);

    void setTheta(double theta);
    void setRho(int rho);
    void setThresholdHoughLines(int thresholdHoughLines);
    void setMinLengthLine(int minLengthLine);
    void setMaxLineGap(int maxLineGap);
    void setHoughLinesThickness(int houghLinesThickness);

    void detect(cv::Mat frame);

private:
    void applyFilters(cv::Mat& frame);
    void houghLines(cv::Mat& frame);
    void houghLane(cv::Mat& frame);

private:
    int m_kernelBlur = 3;
    cv::Mat m_kernelCloseMorph = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Point(4, 4));
    std::vector<Filter> m_filters = {};
    cv::Size m_targetSize = cv::Size(0, 0);

    std::pair<int, int> m_thresholdCanny = {20, 70};

    double m_theta = CV_PI / 180;
    int m_rho = 1;
    int m_thresholdHoughLines = 5;
    int m_minLengthLine = 10;
    int m_maxLineGap = 15;
    int m_houghLinesThickness = 2;
    cv::Mat m_dilateKernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Point(4, 4));

    std::vector<cv::Point> m_startFillPoints = {};
    uchar m_fillValue = 100;
    cv::Mat m_fillKernelCloseMorph = cv::getStructuringElement(cv::MORPH_RECT, cv::Point(12, 12));
};

#endif // LANEDETECTOR_H
