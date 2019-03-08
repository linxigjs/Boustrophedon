#ifndef BOUSTROPHEDON_BOUSTROPHEDON_H
#define BOUSTROPHEDON_BOUSTROPHEDON_H

#include <vector>
#include <math.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include "utils.h"
#include "InscribedRectFinder.h"

using namespace cv;
using namespace std;

class InscribedRectFinder;

//Todo:在具体应用中，某些区域含有极少的像素数，建议把这些区域检索出来做特殊处理
//Todo:子区域边界处留的空隙较大，可以通过“交替纵横弓字形切割+沿边”来解决。（该方法无法彻底解决）

//输入该类的地图（图片）必须是二值灰度图像，白色表示障碍物，黑色表示可行区域

class Boustrophedon{
private:
    int obstacle_ = 255, valid_ = 0;
    int robot_radius_ = 2, overlap_ = 2;
    int regions_cnt_ = 0;

    Mat binary_map_, split_graymap_;
    vector<vector<Point>> bow_path_;    //其中每一个vector都是一个子分区内的弓字路径
    map<int, Mat> regions_;

public:
    void PlanBowPath() {
        SplitMapToRegions();
        Mat colormap = ShowSplitMap();
        GetRegions();
        CalcBowPath();
        ShowBowPath(colormap);
    }

    int SplitMapToRegions() {
        set<int> regions_cnt_helper;
        int valid_segs_cnt_in_last_col = 0;     //考察的上一列像素的区域个数
        vector<Range> valid_segs_in_last_col;   //考察的上一列像素的区域范围
        regions_cnt_ = 1;
        vector<int> split_regions_indexes;      //存放分割的子区域的序号
        binary_map_.copyTo(split_graymap_);     //将单像素所属的区域序号作为split_graymap_中的像素值

        for(int col=0; col<binary_map_.cols; ++col) {
            Mat current_col = binary_map_.col(col);     //逐列取出
            int valid_segs_cnt_in_1col;
            vector<Range> valid_segs_in_1col = CalcConnectivity(current_col, valid_segs_cnt_in_1col);

            if(valid_segs_cnt_in_last_col == 0) {
                split_regions_indexes.clear();
                for(int i=0; i<valid_segs_cnt_in_1col; ++i){
                    split_regions_indexes.push_back(regions_cnt_);
                    regions_cnt_ += 1;
                }
            }else if(valid_segs_cnt_in_1col == 0) {
                split_regions_indexes.clear();
//        continue;
            }else{
                Mat adj_matrix = GetAdjMatrix(valid_segs_in_last_col, valid_segs_in_1col);
                vector<int> temp_regions_indexes(valid_segs_cnt_in_1col, 0);    //temp_regions_indexes用来考察当前列在之前结果基础上的区域划分

                //i代表左侧列第n段，j代表右侧列第n段
                for(int i=0; i<adj_matrix.rows; ++i) {
                    Mat row = adj_matrix.row(i);
                    if (sum(row)[0] == 1) {        //情况1：左列某1段和右列某1段仅有一段邻接
                        for(int j=0; j<row.cols; ++j) {
                            if(row.at<uchar>(j)>0) {
                                temp_regions_indexes[j] = split_regions_indexes[i];     //把左列的区域编号赋值给右列，以前->现在
                                break;      //因为只有1段邻接，没必要继续遍历其余列了
                            }
                        }
                    } else if(sum(row)[0] > 1) {   //情况2：左列1段和右列多段有邻接
                        for(int j=0; j<row.cols; ++j) {
                            if(row.at<uchar>(j)>0) {
                                temp_regions_indexes[j] = regions_cnt_;
                                regions_cnt_ += 1;
                                //右列中至少2段与左侧同一段邻接，说明右列被（障碍物）打断了，因此多了2个新的分区，故总计数++
                            }
                        }
                    }
                }

                //对以上分类讨论的补充，i代表右侧列第n段
                for(int i=0; i<adj_matrix.cols; ++i) {
                    Mat col = adj_matrix.col(i);
                    if(sum(col)[0] > 1 ) {
                        //情况3：对应右列某段与左列至少2段邻接的情况，对情况1的修正
                        temp_regions_indexes[i] = regions_cnt_++;
                    }else if(sum(col)[0] == 0) {
                        //情况4：对应右列某段与左列都不邻接的情况
                        temp_regions_indexes[i] = regions_cnt_++;
                    }
                }
                split_regions_indexes = temp_regions_indexes;
                for(int ind : temp_regions_indexes) {
                    regions_cnt_helper.emplace(ind);
                }
            }

            for(int i=0; i<valid_segs_cnt_in_1col; ++i) {
                split_graymap_(Range(valid_segs_in_1col[i].start, valid_segs_in_1col[i].end+1), Range(col, col+1)) = split_regions_indexes[i];
            }
            valid_segs_in_last_col = valid_segs_in_1col;
            valid_segs_cnt_in_last_col = valid_segs_cnt_in_1col;
        }
        hist(split_graymap_, true);
        regions_cnt_ = static_cast<int>(regions_cnt_helper.size()) + 1;
        cout << "regions_cnt_helper, regions_cnt_=" << regions_cnt_ << endl;
        return regions_cnt_;
    }

    Mat ShowSplitMap() const {
        Mat colormap(split_graymap_.size(), CV_8UC3, cv::Scalar::all(0));
        Mat random_colors;
        random_colors.create(regions_cnt_, 3, CV_8UC1);
        cv::RNG rnger(1);
        rnger.fill(random_colors, cv::RNG::UNIFORM, cv::Scalar::all(0), cv::Scalar::all(255));

        for(int i=0; i<split_graymap_.rows; i++) {
            for(int j=0; j<split_graymap_.cols; j++) {
                int v = split_graymap_.at<uchar>(i,j);
                if(v != 255) {
                    Mat color = random_colors.row(v-1);
                    colormap.at<Vec3b>(i,j) = color;
                }
            }
        }

        imshow("split result - color map", colormap);
        waitKey();
        return colormap;
    }

    map<int, Mat> GetRegions() {
//        imshow("split result - gray map", split_graymap_);
//        waitKey();
        int hist_res = hist(split_graymap_, true);
        cout << "hist_res=" << hist_res << ", regions_cnt_=" << regions_cnt_ << endl;
        if(hist_res != regions_cnt_) {
            throw logic_error("has reduplicate gray values");
        }

        vector<uchar> grayvalues(1, 255);
        for(int i=0; i<split_graymap_.rows; i++) {
            for(int j=0; j<split_graymap_.cols; j++) {
                int v = split_graymap_.at<uchar>(i,j);
                if(find(grayvalues.begin(), grayvalues.end(), v) == grayvalues.end()) {
                    grayvalues.emplace_back(v);
                    Mat reg(split_graymap_.size(), CV_8UC1, Scalar::all(0));
                    regions_.insert({v, reg});
                }
                if(v != 255) {
                    regions_[v].at<uchar>(i,j) = 255;
                }
            }
        }
        cout << "regions_.size()=" << regions_.size() << ", grayvalues.size()=" << grayvalues.size()
             << ", regions_cnt_=" << regions_cnt_ << endl;
        if(regions_.size() != regions_cnt_-1 || grayvalues.size() != regions_cnt_) {
            throw logic_error("regions' count and split are wrong");
        }
        return regions_;
    }

    void CalcBowPath() {
        for(auto it = regions_.begin(); it!=regions_.end(); it++) {
            if(!CheckPassable(it->second)) {
                cout << "This region is too narrow, ignore it." << " gray value: " << it->first << endl;
                continue;
            }
//            string winname("grayvalue ");
//            winname += to_string(it->first);
            int expand_pixels = 20;
            Mat gray_region;
            copyMakeBorder(it->second, gray_region, expand_pixels, expand_pixels, expand_pixels, expand_pixels,
                           cv::BORDER_CONSTANT, Scalar::all(0));
//            imshow(winname, gray_region);
//            waitKey();
            CalcBowPath(gray_region, expand_pixels);
        }
    }

    Mat ShowBowPath(Mat colormap) {
        for(const auto &pts : bow_path_) {
            for(int i=0; i<pts.size()-1; i++) {
                line(colormap, pts[i], pts[i+1], Scalar(0,255,0));
            }
        }
        imshow("Bow Path", colormap);
        waitKey();
        return colormap;
    }

public:
    int getObstacle() const {
        return obstacle_;
    }
    void setObstacle(int obstacle_) {
        Boustrophedon::obstacle_ = obstacle_;
    }
    int getValid() const {
        return valid_;
    }
    void setValid(int valid_) {
        Boustrophedon::valid_ = valid_;
    }
    int getRobot_radius() const {
        return robot_radius_;
    }
    void setRobot_radius(int robot_radius_) {
        Boustrophedon::robot_radius_ = robot_radius_;
    }
    int getOverlap() const {
        return overlap_;
    }
    void setOverlap(int overlap_) {
        Boustrophedon::overlap_ = overlap_;
    }
    const Mat &getBinary_map() const {
        return binary_map_;
    }
    void setBinary_map(const Mat &binary_map_) {
        Boustrophedon::binary_map_ = binary_map_;
    }
    vector<vector<Point>> getBow_path() const {
        return bow_path_;
    }
    map<int, Mat> getRegions() const {
        return regions_;
    }
    int getRegions_cnt() const {
        return regions_cnt_;
    }

private:
    //slice是图片的一列像素
    vector<Range> CalcConnectivity(const Mat &one_col, int &valid_seg_cnt_in_1col) {
        valid_seg_cnt_in_1col = 0;
        int last_data = obstacle_;
        bool open_start = false;
        vector<Range> valid_seg_in_1col;
        int start_point = 0;
        int end_point = 0;
        for(int i=0; i<one_col.rows; ++i) {
            int data = one_col.at<uchar>(i,0);     //slice只有一列，因此第二个坐标一定是0
            if(last_data == obstacle_ && data == valid_) {        //黑色-认为开始处
                open_start = true;
                start_point = i;
            } else if((last_data==valid_ && data==obstacle_ && open_start) || (open_start && i==one_col.rows-1)) {     //右侧条件对应底部边界
                open_start = false;
                valid_seg_cnt_in_1col += 1;
                end_point=i;
                valid_seg_in_1col.push_back(Range(start_point, end_point));
            }
            last_data = data;
        }
        return valid_seg_in_1col;
    }

    Mat GetAdjMatrix(const vector<Range> &last_col, const vector<Range> &cur_col) {
        int last_col_segs_cnt = last_col.size(), cur_col_segs_cnt = cur_col.size();
        Mat adj_mat(last_col_segs_cnt, cur_col_segs_cnt, CV_8UC1);      //Mat(int rows, int cols, int type);
        for(int i=0; i<last_col_segs_cnt; ++i) {
            Range last_col_1seg = last_col[i];
            for(int j=0; j<cur_col_segs_cnt; ++j) {
                Range cur_col_1seg = cur_col[j];
                if((std::min(last_col_1seg.end,cur_col_1seg.end) - std::max(last_col_1seg.start,cur_col_1seg.start)) > 0) {
                    adj_mat.at<uchar>(i,j) = 1;        //1表示相邻2列有邻接的valid像素
                } else {
                    adj_mat.at<uchar>(i,j) = 0;        //0表示相邻2列没有邻接的valid像素
                }
            }
        }
        return adj_mat;
    }

    void ReOrderBow(vector<Point> &path) {
        if(IsOdd(path.size())) {
            cout << "Warning: points num is odd." << endl;
        }
        vector<Point> temp;
        bool turn = false;
        for(int i=0; i<path.size(); i=i+2) {
            if(turn) {
                temp.emplace_back(path[i+1]);
                temp.emplace_back(path[i]);
                turn = false;
            } else {
                temp.emplace_back(path[i]);
                temp.emplace_back(path[i+1]);
                turn = true;
            }
        }
        path = temp;
    }

    void CalcBowPath(Mat gray_region, int expand_pixels) {
        //计算子分区的宽度（图片列数）
        vector<vector<cv::Point>> contours;
        findContours(gray_region, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
        cv::Rect bounding_rect = boundingRect(contours[0]);
        cv::Mat element = getStructuringElement(cv::MORPH_RECT, cv::Size(2 * robot_radius_, 2 * robot_radius_),
                                                cv::Point(robot_radius_, robot_radius_));
        threshold(gray_region, gray_region, 1, 255, THRESH_BINARY_INV);   //反色
        Mat reg_dilate;
        cv::dilate(gray_region, reg_dilate, element);

        int lastv = 255, lastj = -1;
        vector<Point> tps;
        for(int j=bounding_rect.x; j<bounding_rect.x+bounding_rect.width; j+=overlap_) {
            for(int i=0; i<reg_dilate.rows; i++) {
                int v = reg_dilate.at<uchar>(i,j);
                if(v != lastv) {
                    tps.emplace_back(Point(j-expand_pixels, i-expand_pixels));
                }
                lastv = v;
            }
            lastj = j;
        }
        //处理每个子区域剩余的窄条
        int last_width = (bounding_rect.x+bounding_rect.width) - lastj;
        if(last_width > 5) {
            int j = lastj + (last_width>>1);
            for(int i=0; i<reg_dilate.rows; i++) {
                int v = reg_dilate.at<uchar>(i,j);
                if(v != lastv) {
                    tps.emplace_back(Point(j-expand_pixels, i-expand_pixels));
                }
                lastv = v;
            }
        }
        if(!tps.empty()) {
            ReOrderBow(tps);
            bow_path_.emplace_back(tps);
        }
    }

    bool CheckPassable(Mat binary_map) {
//        imshow("CheckPassable", binary_map);
//        waitKey();
        vector<vector<cv::Point>> contours;
        findContours(binary_map, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
        int thresh = 1.5*robot_radius_;
        RotatedRect external_rect = minAreaRect(contours[0]);
//        Rect external_rect = boundingRect(contours[0]);
        //使用子区域的内接矩形判段是否容纳的下Robot，但是findRectangle()非常慢
        InscribedRectFinder finder;
        Rect inscribed_rect = finder.findRectangle(binary_map);
//        if(external_rect.width < thresh || external_rect.height < thresh) {
        if(external_rect.boundingRect().width < thresh || external_rect.boundingRect().height < thresh) {
            return false;
        }
        return true;
    }
};

#endif //BOUSTROPHEDON_BOUSTROPHEDON_H
