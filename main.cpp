
#include "Boustrophedon.h"
#include "utils.h"

using namespace std;
using namespace cv;

int main() {
    //must input binary gray image.
    Mat img= imread("../yard3.png", 0);
    if(img.empty()){
        cout << "Load image failed!" << endl;
        return 0;
    }
    hist(img);

    imshow("Gray_Window", img);
    Mat img_b;
    threshold(img, img_b, 0, 1, THRESH_BINARY_INV);//通过阈值操作把灰度图变成二值图

    imshow("Bin_Window", img_b);
    int cells=0;
    Mat separate_map=Boustrophedon::calc_bcd(img_b, cells);
    cout << "Separate image sucessed!" << endl;
    Mat display_mat=Boustrophedon::display_separate_map(separate_map, cells);
    cout << "Display Separate image sucessed!" << endl;

    imshow("BCD_Window", display_mat);
    waitKey();
    return 0;
}


