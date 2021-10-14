#include <opencv2/core/core.hpp>  
#include <opencv2/highgui/highgui.hpp>  
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>
#include <iostream>

using namespace std;
using namespace cv;

const double MIN_AREA = 1000;
const double INF = 1e18;

inline double sq(double x) {
    return x * x;
}

double getdis(const Point2f& A, const Point2f& B) {
    return sqrt(sq(A.x - B.x) + sq(A.y - B.y));
}

Point2f last_center = Point2f(-1, -1);

Mat calc(Mat &src) {
    cout << '\102' << endl;
    int cnt = 0;

   //  Mat templ_sword = imread("sword.jpg");
   //  Mat templ_hammer = imread("hammer.jpg");

    Mat drawer = src.clone();

    Mat hsv;
    cvtColor(src, hsv, COLOR_BGR2HSV);
    Mat hsv_part1, hsv_part2;
    inRange(hsv, Scalar(0, 43, 46), Scalar(25, 255, 255), hsv_part1);
    inRange(hsv, Scalar(156, 43, 46), Scalar(180, 255, 255), hsv_part2);
    Mat ones_mat = Mat::ones(Size(src.cols, src.rows), CV_8UC1);
    Mat hsv_result = 255 * (ones_mat - (ones_mat - hsv_part1 / 255).mul(ones_mat - hsv_part2 / 255));

    Mat ele = getStructuringElement(MORPH_RECT, Size(5, 5)); // 这个大小差不多刚好
    Mat open_res, close_res;
    morphologyEx(hsv_result, open_res, MORPH_OPEN, ele);
    morphologyEx(open_res, close_res, MORPH_CLOSE, ele);

    dilate(open_res, close_res, ele);
    open_res = close_res.clone();

    vector<vector<Point>> contours;
    vector<Vec4i> hierachy;
    findContours(open_res, contours, hierachy,
        RETR_CCOMP, CHAIN_APPROX_SIMPLE);

    RotatedRect rect_tmp; // 最小可旋转外接矩形

    Mat fin;
    Point2f center = Point2f(1, 1);

    int fanid[5]; int idcnt = 0;


    int flag = 0;
    if (hierachy.size())
        for (int i = 0; ~i; i = hierachy[i][0]) { // 只对最外层轮廓
            rect_tmp = minAreaRect(contours[i]); // 获得最小外接矩形
            Point2f p[4];
            rect_tmp.points(p); // 获得四个顶点的坐标

            Point2f src_rect[4]; // 往下面看你就知道干啥用
            Point2f dst_rect[4];

            double width;
            double height;

            width = getdis(p[0], p[1]); 
            height = getdis(p[1], p[2]);

            if (width > height) { // 理想情况为 宽度>高度
                for (int j = 0; j <= 3; ++j)
                    src_rect[j] = p[j];
            }
            else {
                swap(width, height);
                for (int j = 0; j <= 3; ++j)
                    src_rect[j] = p[(j + 1) % 4];
            }

            double area = height * width;

            if (area > 100 and area < 1000) { 
                
                center = Point2f((p[0].x + p[2].x) / 2, (p[0].y + p[2].y) / 2);
                cout << '#' << last_center << endl;
                cout << center << endl;
                cout << getdis(last_center, center) << endl;
                 if (last_center.x == -1 or getdis(last_center, center) < 4.0) {
                //  if (1) {   
                    circle(drawer, center, getdis(p[0], p[1]) / 2, Scalar(0, 255, 0), 2);
                    flag = 1;
                }
                else
                    cout << "fail1" << endl;
                // cout << area << endl;
                // 面积太小的不使用
                /*dst_rect[0] = Point2f(0, 0);
                dst_rect[1] = Point2f(width, 0);
                dst_rect[2] = Point2f(width, height);
                dst_rect[3] = Point2f(0, height);

                // __test
                for (int j = 0; j <= 3; ++j)
                    cout << src_rect[j] << endl;
                cout << "#" << endl;
                
                // dst_rect 貌似只是用来计算的
                
                Mat trans_reg = getPerspectiveTransform(src_rect, dst_rect);
                // 这个trans_reg是变换之规则 而且还能在原图中定位..

                Mat pers_img; // 
                warpPerspective(open_res, pers_img, trans_reg, open_res.size());

                Mat pers_res = pers_img(Rect(0, 0, width, height));
                
                
                return pers_res;

            
                Point matchLoc;*/

                
            }
            else if (area > 1000) {
                fanid[cnt++] = i;
            }
        }
    
    if (flag == 0)
        cout << "this flame Fail" << endl;
    if (last_center.x == -1)
        last_center = center;
    // return drawer;
   /*for (int i = 0; i < cnt; ++i) {
        int id = fanid[i];
        for (int j = )
    }*/
    
    Mat hit_it = imread("hit_it.png");
    if (hierachy.size())
        for (int i = 0; ~i; i = hierachy[i][0]) { // 只对最外层轮廓
            rect_tmp = minAreaRect(contours[i]); // 获得最小外接矩形
            Point2f p[4];
            rect_tmp.points(p); // 获得四个顶点的坐标

            Point2f src_rect[4]; // 往下面看你就知道干啥用
            Point2f dst_rect[4];

            double width;
            double height;

            width = getdis(p[0], p[1]); 
            height = getdis(p[1], p[2]);

            if (width > height) { // 理想情况为 宽度>高度
                for (int j = 0; j <= 3; ++j)
                    src_rect[j] = p[j];
            }
            else {
                swap(width, height);
                for (int j = 0; j <= 3; ++j)
                    src_rect[j] = p[(j + 1) % 4];
            }

            double area = height * width;

            if (area > 1000) { 
                drawContours(drawer, contours, i, Scalar(255, 0, 0), 1);
                center = Point2f((p[0].x + p[2].x) / 2, (p[0].y + p[2].y) / 2);
                int soncnt = 0;
                double maxdis = -INF, farson = -1;
                for (int j = hierachy[i][2]; ~j; j = hierachy[j][0]) {
                    ++soncnt;
                    double sondis = pointPolygonTest(contours[j], center, 1);
                    if (sondis > maxdis) {
                        maxdis = sondis;
                        farson = j;
                    }
                    cout << "contous" << j << endl;
                   drawContours(drawer, contours, j, Scalar(0, 0, 255), 1);
                }
                if (soncnt <= 1 and farson != -1) {
                    drawContours(drawer, contours, farson, Scalar(255, 255, 255), 3);
                    Mat place_hit_it = Mat(drawer, Rect(center.x, center.y - 20, hit_it.cols, hit_it.rows));
                    hit_it.copyTo(place_hit_it);
                }
                else {

                }
                // cout << area << endl;
                // 面积太小的不使用
                /*
                dst_rect[0] = Point2f(0, 0);
                dst_rect[1] = Point2f(width, 0);
                dst_rect[2] = Point2f(width, height);
                dst_rect[3] = Point2f(0, height);

                // __test
                for (int j = 0; j <= 3; ++j)
                    cout << src_rect[j] << endl;
                cout << "#" << endl;
                
                // dst_rect 貌似只是用来计算的
                
                Mat trans_reg = getPerspectiveTransform(src_rect, dst_rect);
                // 这个trans_reg是变换之规则 而且还能在原图中定位..

                Mat pers_img; // 
                warpPerspective(open_res, pers_img, trans_reg, open_res.size());

                Mat pers_res = pers_img(Rect(0, 0, width, height));
                
                
                return pers_res;

            
                Point matchLoc;*/
            }
        }
    
    if (last_center.x == -1) 
        last_center = center;

    return drawer;

}

int main() {
    VideoCapture capture("video.mp4");
    Mat src;
    while (capture.read(src)) {
        Mat res = calc(src);
        imshow("result", res);
        waitKey(20);
    }
    return 0;
}
