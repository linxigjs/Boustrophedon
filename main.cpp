
#include "Boustrophedon.h"

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
//    imshow("Binary img", img_b);
    cout << img_b.size << endl;

    int regions_cnt=0;
    Boustrophedon niu;
    Mat separate_map= niu.Calcbcd(img_b, regions_cnt);
    cout << "Separate image sucessed! " << regions_cnt << endl;
    hist(separate_map);

    Mat display_mat= niu.DisplaySeparateMap(separate_map, regions_cnt);
    cout << "Display Separate image sucessed!" << endl;
    imshow("separate_map", display_mat);
//    waitKey();

    niu.SplitToRegions(display_mat, regions_cnt);

    return 0;
}


