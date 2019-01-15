
#include "Boustrophedon.h"

using namespace std;
using namespace cv;

int main() {
    //must input binary gray image.
    Mat img= imread("../yard4.png", 0);
    if(img.empty()){
        cout << "Load image failed!" << endl;
        return 0;
    }
    imshow("Source input", img);
    waitKey();
    Mat img_b;
    threshold(img, img_b, 1, 255, THRESH_BINARY_INV);//通过阈值操作把灰度图变成二值图
//    imshow("Binary img, Source map", img_b);
//    waitKey();

    Boustrophedon niu;
    niu.setRobot_radius(8);
    niu.setOverlap(5);
    niu.setBinary_map(img_b);
    niu.PlanBowPath();
    cout << "Separate gray map sucessed! " << endl;


    return 0;
}


