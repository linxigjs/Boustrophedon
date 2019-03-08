//
// Created by gjs on 19-1-11.
//

#ifndef BOUSTROPHEDON_UTILS_H
#define BOUSTROPHEDON_UTILS_H

#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;
using namespace std;

inline int RotateImageClockwise90(cv::Mat &src) {
    if (src.empty()) {
        cout << "Image src is empty!" << endl;
        return 1;
    }
    cv::Mat temp;
    // 矩阵转置
    cv::transpose(src, temp);
    //0: 沿X轴翻转； >0: 沿Y轴翻转； <0: 沿X轴和Y轴翻转
    cv::flip(temp, src, 1);// 翻转模式，flipCode == 0垂直翻转（沿X轴翻转），flipCode>0水平翻转（沿Y轴翻转），flipCode<0水平垂直翻转（先沿X轴翻转，再沿Y轴翻转，等价于旋转180°）
    return 0;
}

inline int hist(Mat image, bool info = false) {
    const int channels=image.channels();
    int nRows=image.rows;
    int nCols=image.cols*channels;
    vector<int> hist_result(257);
    //用指针访问像素，速度最快
    uchar *p;
    for(int i=0;i<nRows;i++) {
        p = image.ptr<uchar>(i);//获取每行首地址
        for(int j=0; j<nCols; ++j) {
            ++hist_result[p[j]];
        }
    }
    if(info) {
        for(int i=0; i < static_cast<int>(hist_result.size()); i++) {
            if(hist_result[i] != 0) {
                cout << "灰度值=" << i << " ，个数=" << hist_result[i] << endl;
            }
        }
    }
    int cnt = 0;
    for(int e : hist_result) {
        if(e != 0) {
            cnt++;
        }
    }
    cout << "数数=" << cnt << "   ";

    int res = count_if(hist_result.begin(), hist_result.end(), [](int v){ return v != 0;});
    cout << ", count_if=" << res << endl;
    return res;
}
//
//inline int sum(const Mat &mat) {
//    int sum=0;
//    for(int i=0; i<mat.rows; i++) {
//        for(int j=0; j<mat.cols; j++) {
//            sum += mat.at<u_char>(i,j);
//        }
//    }
//    return sum;
//}

inline int CountPixels(const Mat &mat, int v) {
    int cnt=0;
    for(int i=0; i<mat.rows; i++) {
        for(int j=0; j<mat.cols; j++) {
            if(mat.at<u_char>(i,j) == v)
                cnt++;
        }
    }
    return cnt;
}

inline bool IsOdd(int v) {
    return v%2 == 1;
}

inline int DilateImage(cv::Mat &img, int dilate_pixels, bool show) {
    cv::Mat temp_after_expand, temp_after_dilate;
    int expand_pixels = 20;
    cv::copyMakeBorder(img, temp_after_expand, expand_pixels, expand_pixels, expand_pixels, expand_pixels,
                       cv::BORDER_CONSTANT, cv::Scalar(255,255,255));
    cv::Mat element = getStructuringElement(cv::MORPH_RECT, cv::Size(2 * dilate_pixels, 2 * dilate_pixels),
                                            cv::Point(dilate_pixels, dilate_pixels));
    cv::dilate(temp_after_expand, temp_after_dilate, element); //膨胀腐蚀不改变原图
    cv::Rect kernel = cv::Rect(expand_pixels, expand_pixels, img.cols, img.rows);
    img = temp_after_dilate(kernel);
    if(show) {
        cv::imshow("dst image after dilate", img);
        cv::waitKey();
    }
    return 0;
}


#endif //BOUSTROPHEDON_UTILS_H
