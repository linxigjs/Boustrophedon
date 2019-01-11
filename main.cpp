
#include "Boustrophedon.h"
#include "utils.h"

using namespace std;
using namespace cv;

int main() {
    //must input binary gray image.
    Mat img= imread("../yard2.png", 0);
    if(img.empty()){
        cout << "Load image failed!" << endl;
        return 0;
    }
//    hist(img);

//    imshow("Gray_Window", img);
    Mat img_b;
    threshold(img, img_b, 1, 255, THRESH_BINARY_INV);//通过阈值操作把灰度图变成二值图
    imshow("Binary img", img_b);
    cout << img_b.size << endl;

    int regions_cnt=0;
    Boustrophedon niu;
    Mat separate_map= niu.Calcbcd(img_b, regions_cnt);
    cout << "Separate image sucessed! " << regions_cnt << endl;
    hist(separate_map);
    imshow("separate_map", separate_map);
    waitKey();

    Mat temp(separate_map.size(), CV_8UC3, cv::Scalar::all(0));
    for(int i=0; i<separate_map.rows; ++i) {
        for(int j=0; j<separate_map.cols; ++j) {
            if(separate_map.at<u_char>(i,j) == 0) {
//                temp.at<Vec3b>(i,j)[0] = 0;
//                temp.at<Vec3b>(i,j)[1] = 255;
//                temp.at<Vec3b>(i,j)[2] = 0;
                circle(temp, Point(j,i), 2, Scalar(0,255,0), -1);
            }
        }
    }
    imshow("temp", temp);
    waitKey();

    Mat display_mat= niu.DisplaySeparateMap(separate_map, regions_cnt);
    cout << "Display Separate image sucessed!" << endl;

    imshow("BCD_Window", display_mat);
    waitKey();
    return 0;
}


