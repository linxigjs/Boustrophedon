//
// Created by gjs on 19-1-11.
//

#ifndef BOUSTROPHEDON_UTILS_H
#define BOUSTROPHEDON_UTILS_H

#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;
using namespace std;

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


/**
 * @brief expandEdge 扩展边界函数
 * @param img:输入图像，单通道二值图，深度为8
 * @param edge  边界数组，存放4条边界值
 * @param edgeID 当前边界号
 * @return 布尔值 确定当前边界是否可以扩展
 */
inline bool expandEdge(const Mat & img, int edge[], const int edgeID)
{
    //[1] --初始化参数
    int nc=img.cols;
    int nr=img.rows;

    switch (edgeID) {
        case 0:
            if(edge[0]>nr)
                return false;
            for(int i=edge[3];i<=edge[1];++i)
            {
                if(img.at<uchar>(edge[0],i)==0)
                    return false;
            }
            edge[0]++;
            return true;
            break;
        case 1:
            if(edge[1]>nc)
                return false;
            for(int i=edge[2];i<=edge[0];++i)
            {
                if(img.at<uchar>(i,edge[1])==0)
                    return false;
            }
            edge[1]++;
            return true;
            break;
        case 2:
            if(edge[2]<0)
                return false;
            for(int i=edge[3];i<=edge[1];++i)
            {
                if(img.at<uchar>(edge[2],i)==0)
                    return false;
            }
            edge[2]--;
            return true;
            break;
        case 3:
            if(edge[3]<0)
                return false;
            for(int i=edge[2];i<=edge[0];++i)
            {
                if(img.at<uchar>(i,edge[3])==0)
                    return false;
            }
            edge[3]--;
            return true;
            break;
        default:
            return false;
            break;
    }
}


/**
 * @brief 求取连通区域内接矩
 * @param img:输入图像，单通道二值图，深度为8
 * @param center:最小外接矩的中心
 * @return  最大内接矩形
 * 基于中心扩展算法
 */
inline cv::Rect CalcInscribedRect(Mat &img, const Point center)
{
    // --[1]参数检测
    if(img.empty() || img.channels()>1 || img.depth()>8)
        return Rect();
    //[1]

    // --[2] 初始化变量
    int edge[4];
    edge[0]=center.y+1;//top
    edge[1]=center.x+1;//right
    edge[2]=center.y-1;//bottom
    edge[3]=center.x-1;//left
    //[2]

    // --[3]边界扩展(中心扩散法)
    bool EXPAND[4]={1,1,1,1};//扩展标记位
    int n=0;
    while (EXPAND[0]||EXPAND[1]||EXPAND[2]||EXPAND[3])
    {
        int edgeID=n%4;
        EXPAND[edgeID]=expandEdge(img,edge,edgeID);

        n++;
    }
    //[3]

    cout << edge[0] << " " <<edge[1] << " " << edge[2] << " " << edge[3] << endl;
    Point tl=Point(edge[3],edge[0]);
    Point br=Point(edge[1],edge[2]);
    return Rect(tl,br);
}



#endif //BOUSTROPHEDON_UTILS_H
