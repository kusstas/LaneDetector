#include <iostream>

#include "LaneDetector.h"

constexpr auto S_WAIT_MS = 25;
constexpr auto S_ESC_KEY = 27;

int main(int argn, char** argv)
{
    if (argn < 2)
    {
        std::cout << "Error run! Missing argument to source" << std::endl;
    }
    char const* pathToVideo = argv[1];

    LaneDetector laneDetector;

    laneDetector.addFilter({-0.1f, -0.1f, -0.1f,
                            -0.1f,  2.0f, -0.1f,
                            -0.1f, -0.1f, -0.1f});

    cv::VideoCapture cap(pathToVideo);

    if (!cap.isOpened())
    {
        std::cout << "Cannot open video with path: " << pathToVideo << std::endl;
    }

    while (cap.isOpened())
    {
        cv::Mat frame;
        cap >> frame;

        if (frame.empty())
        {
            std::cout << "Empty frame. Video stopped" << std::endl;
            break;
        }

        laneDetector.setTargetSize(cv::Size(frame.cols / 2, frame.rows / 2));
        laneDetector.detect(frame);

        if (cv::waitKey(S_WAIT_MS) == S_ESC_KEY)
        {
            std::cout << "Esc pressed. Video stopped" << std::endl;
            break;
        }
    }

    cap.release();
    cv::destroyAllWindows();

    return 0;
}
