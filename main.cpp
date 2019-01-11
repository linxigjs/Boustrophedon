
#include "Boustrophedon.h"

using namespace std;
using namespace cv;

int main(int argc, char** argv)
{

    Mat img= imread("../yard3.png");
    if(img.empty()){
        cout << "Load image failed!" << endl;
        return 0;
    }
    Mat img_R;
    cvtColor(img, img_R, CV_BGR2GRAY);//三通道的图转化为1通道的灰度图

    imshow("Gray_Window", img_R);
    Mat img_b;
    threshold(img_R, img_b, 0, 1, THRESH_BINARY_INV);//通过阈值操作把灰度图变成二值图

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


