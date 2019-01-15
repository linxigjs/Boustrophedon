//
// Created by gjs on 19-1-11.
//

#ifndef BOUSTROPHEDON_UTILS_H
#define BOUSTROPHEDON_UTILS_H

#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;
using namespace std;

inline int hist(Mat image) {
    const int channels=image.channels();
    int nRows=image.rows;
    int nCols=image.cols*channels;
    vector<int> hist_result(257);
    //用指针访问像素，速度最快
    uchar *p;
    for(int i=0;i<nRows;i++)
    {
        p=image.ptr<uchar>(i);//获取每行首地址
        for(int j=0;j<nCols;++j)
        {
            ++hist_result[p[j]];
//            if(p[j]>128)
//                p[j]=0;
//            else
//                p[j]=255;
        }
    }
    for(int i=0; i < static_cast<int>(hist_result.size()); i++) {
        if(hist_result[i] != 0) {
            cout << "灰度值=" << i << " ，个数=" << hist_result[i] << endl;
        }
    }
    return count_if(hist_result.begin(), hist_result.end(), [](int v){ return v != 0;});
}

inline int sum(const Mat &mat) {
    int sum=0;
    for(int i=0; i<mat.rows; i++) {
        for(int j=0; j<mat.cols; j++) {
            sum += mat.at<u_char>(i,j);
        }
    }
    return sum;
}

inline bool IsOdd(int v) {
    return v%2 == 1;
}



#endif //BOUSTROPHEDON_UTILS_H
