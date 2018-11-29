#include "LaneDetector.h"

#include <opencv2/highgui/highgui.hpp>


static constexpr auto S_ORIGINAL_WINDOW = "Original";
static constexpr auto S_CONTROLS_WINDOW = "Controls";
static constexpr auto S_CORE_WINDOW = "Core";
static constexpr auto S_ROAD_WINDOW = "Road";

LaneDetector::LaneDetector()
{
    cv::namedWindow(S_ORIGINAL_WINDOW, cv::WINDOW_AUTOSIZE);
    cv::namedWindow(S_CONTROLS_WINDOW, cv::WINDOW_AUTOSIZE);
    cv::namedWindow(S_CORE_WINDOW, cv::WINDOW_AUTOSIZE);
    cv::namedWindow(S_ROAD_WINDOW, cv::WINDOW_AUTOSIZE);

    cv::createTrackbar("Th1 Canny", S_CONTROLS_WINDOW, &m_thresholdCanny.first, 250);
    cv::createTrackbar("Th2 Canny", S_CONTROLS_WINDOW, &m_thresholdCanny.second, 250);
    cv::createTrackbar("Threshhold Hough", S_CONTROLS_WINDOW, &m_thresholdHoughLines, 100);
    cv::createTrackbar("Min Length Line", S_CONTROLS_WINDOW, &m_minLengthLine, 100);
    cv::createTrackbar("Max Line Gap", S_CONTROLS_WINDOW, &m_maxLineGap, 100);
}

cv::Size const& LaneDetector::targetSize() const
{
    return m_targetSize;
}

void LaneDetector::setKernelBlur(int kernel)
{
    m_kernelBlur = kernel;
}

void LaneDetector::setKernelloseMorph(cv::Mat const& kernel)
{
    m_kernelCloseMorph = kernel;
}

void LaneDetector::addFilter(Filter filter)
{
    m_filters.push_back(std::move(filter));
}

void LaneDetector::setTargetSize(cv::Size const& size)
{
    m_targetSize = size;
}

void LaneDetector::setThresholdCanny(std::pair<int, int> const& thresholdCanny)
{
    m_thresholdCanny = thresholdCanny;
}

void LaneDetector::setTheta(double theta)
{
    m_theta = theta;
}

void LaneDetector::setRho(int rho)
{
    m_rho = rho;
}

void LaneDetector::setThresholdHoughLines(int thresholdHoughLines)
{
    m_thresholdHoughLines = thresholdHoughLines;
}

void LaneDetector::setMinLengthLine(int minLengthLine)
{
    m_minLengthLine = minLengthLine;
}

void LaneDetector::setMaxLineGap(int maxLineGap)
{
    m_maxLineGap = maxLineGap;
}

void LaneDetector::setHoughLinesThickness(int houghLinesThickness)
{
    m_houghLinesThickness = houghLinesThickness;
}

void LaneDetector::detect(cv::Mat frame)
{
    cv::Mat original;
    frame.copyTo(original);

    cv::cvtColor(frame, frame, CV_BGR2GRAY);

    if (m_targetSize.width > 0 && m_targetSize.height > 0)
    {
        cv::resize(frame, frame, m_targetSize);
    }

    cv::morphologyEx(frame, frame, CV_MOP_CLOSE, m_kernelCloseMorph);
    cv::medianBlur(frame, frame, m_kernelBlur);
    applyFilters(frame);

    cv::Canny(frame, frame, m_thresholdCanny.first, m_thresholdCanny.second);
    houghLines(frame);

    cv::imshow(S_CORE_WINDOW, frame);

    m_startFillPoints.clear();
    m_startFillPoints.push_back(cv::Point(frame.cols / 2, static_cast<int>(frame.rows * 0.75f)));
    houghLane(frame);

    cv::imshow(S_ROAD_WINDOW, frame);

    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;

    cv::findContours(frame, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);

    auto it = std::max_element(contours.begin(), contours.end(), [] (std::vector<cv::Point> const& a,
                               std::vector<cv::Point> const& b){
            return  a.size() < b.size();
    });

    if (it != contours.end())
    {
        for (auto& point : *it)
        {
            point *= 2;
        }

        int i_max = std::distance(contours.begin(), it);
        cv::drawContours(original, contours, i_max, cv::Scalar(0, 255, 0), 2, 8, hierarchy, 0);
    }

    cv::imshow(S_ORIGINAL_WINDOW, original);
}

void LaneDetector::applyFilters(cv::Mat& frame)
{
    for (auto const& filter : m_filters)
    {
        cv::filter2D(frame, frame, frame.depth(), filter);
    }
}

void LaneDetector::houghLines(cv::Mat& frame)
{
    std::vector<cv::Vec4i> lines;
    cv::HoughLinesP(frame, lines, m_rho, m_theta, m_thresholdHoughLines, m_minLengthLine, m_maxLineGap);

    for (auto const& l : lines)
    {
        cv::line(frame, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(255), m_houghLinesThickness);
    }

    cv::dilate(frame, frame, m_dilateKernel);
}

void LaneDetector::houghLane(cv::Mat& frame)
{
    for (auto const& point : m_startFillPoints)
    {
        cv::floodFill(frame, point, cv::Scalar(m_fillValue));
    }

    for (int i = 0; i < frame.rows; i++)
    {
        for (int j = 0; j < frame.cols; j++)
        {
            uchar& pixel = frame.at<uchar>(i, j);
            pixel = pixel == m_fillValue ? 255 : 0;
        }
    }

    cv::morphologyEx(frame, frame, CV_MOP_CLOSE, m_fillKernelCloseMorph);
}
