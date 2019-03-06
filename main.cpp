
#include "Boustrophedon.h"

using namespace std;
using namespace cv;

int main() {
    //must input binary gray image.
    Mat img= imread("../global_map_rotate.png", 0);
    if(img.empty()){
        cout << "Load image failed!" << endl;
        return 0;
    }
//    imshow("Source input", img);
//    waitKey();
    Mat img_b;
    threshold(img, img_b, 1, 255, THRESH_BINARY_INV);//通过阈值操作把灰度图变成二值图
    img_b = 255 - img_b;
    imshow("Binary img, Source map", img_b);
    waitKey();
    int dilate_pixels = 4;
    cv::Mat element = getStructuringElement(cv::MORPH_RECT, cv::Size(2 * dilate_pixels, 2 * dilate_pixels),
                                            cv::Point(dilate_pixels, dilate_pixels));
    dilate(img_b, img_b, element);
    imshow("obstacle dilate", img_b);
    waitKey();

    Boustrophedon niu;
    niu.setRobot_radius(8);
    niu.setOverlap(10);
    niu.setBinary_map(img_b);
    niu.PlanBowPath();
    cout << "Separate gray map sucessed! " << endl;
/*
    RotateImageClockwise90(img_b);
    Boustrophedon niu2;
    niu2.setRobot_radius(8);
    niu2.setOverlap(5);
    niu2.setBinary_map(img_b);
    niu2.PlanBowPath();
    cout << "Separate gray map sucessed! " << endl;
*/
    return 0;
}


