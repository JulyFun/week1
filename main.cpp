#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <vector>
#include <assert.h>

using namespace::cv;
using namespace::std;


int main(int argc, char **argv) {
    cv::Mat src = cv::imread("apple.png");
    cv::Mat tmp = src.clone();
    
    cv::Mat hsv;
    cv::cvtColor(src, hsv, cv::COLOR_BGR2HSV);

    cv::Mat hsv_part1, hsv_part2;
    cv::inRange(hsv, cv::Scalar(0, 150, 80), cv::Scalar(22, 255, 255), hsv_part1);
    cv::inRange(hsv, cv::Scalar(156, 100, 46), cv::Scalar(180, 255, 255), hsv_part2);

    cv::Mat ones_mat = cv::Mat::ones(cv::Size(src.cols, src.rows), CV_8UC1);
    cv::Mat hsv_result = 255 * (ones_mat - (ones_mat - hsv_part1 / 255).mul(ones_mat - hsv_part2 / 255));

    cv::Mat blur_result;
    cv::medianBlur(hsv_result, blur_result, 7);

    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(25, 25));
    cv::Mat dst_res;
    cv::erode(blur_result, dst_res, element);

    cv::Mat dil_res;
    cv::dilate(dst_res, dil_res, element);


    cv::imshow("fin", dil_res);


    vector< vector<Point> > contours;
    vector<Vec4i> hierarcy;
    findContours(dil_res, contours, hierarcy, RETR_EXTERNAL, CHAIN_APPROX_NONE);
    
    vector<Rect> boundRect(contours.size());

    int x0 = 0, y0 = 0, w0 = 0, h0 = 0;
    for (int i = 0; i < contours.size(); ++i) {
        boundRect[i] = boundingRect((Mat)contours[i]);
        drawContours(tmp, contours, i, Scalar(0, 0, 255), 2, 8);
        x0 = boundRect[i].x;
        y0 = boundRect[i].y;
        w0 = boundRect[i].width;
        h0 = boundRect[i].height;
        rectangle(tmp, Point(x0, y0), Point(x0 + w0, y0 + h0), Scalar(0, 255, 0), 2, 8);
    }

    imshow("bound_rect", tmp);

    cv::waitKey(0);
    return 0;
}