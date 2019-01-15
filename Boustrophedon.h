#ifndef BOUSTROPHEDON_BOUSTROPHEDON_H
#define BOUSTROPHEDON_BOUSTROPHEDON_H

#include <vector>
#include <math.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include "utils.h"

using namespace cv;
using namespace std;

//Todo:在具体应用中，某些区域含有极少的像素数，建议把这些区域检索出来做特殊处理

class Boustrophedon{
private:
    int obstacle_ = 255, valid_ = 0;
public:
    //slice是图片的一列像素
    vector<Range> CalcConnectivity(const Mat &one_col, int &valid_seg_cnt_in_1col) {
        valid_seg_cnt_in_1col = 0;
        int last_data = obstacle_;
        bool open_start = false;
        vector<Range> valid_seg_in_1col;
        int start_point = 0;
        int end_point = 0;
        for(int i=0; i<one_col.rows; ++i) {
            int data = one_col.at<u_char>(i,0);     //slice只有一列，因此第二个坐标一定是0
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
                    adj_mat.at<u_char>(i,j) = 1;        //1表示相邻2列有邻接的valid像素
                } else {
                    adj_mat.at<u_char>(i,j) = 0;        //0表示相邻2列没有邻接的valid像素
                }
            }
        }
        return adj_mat;
    }

    Mat Calcbcd(const Mat &entire_map, int &regions_cnt) {
        int valid_segs_cnt_in_last_col = 0;     //考察的上一列像素的区域个数
        vector<Range> valid_segs_in_last_col;   //考察的上一列像素的区域范围
        regions_cnt = 1;
        vector<int> split_regions_indexes;      //存放分割的子区域的序号
        Mat seperate_map;       //将单像素所属的区域序号作为其像素值
        entire_map.copyTo(seperate_map);

        for(int col=0; col<entire_map.cols; ++col) {
            Mat current_col = entire_map.col(col);     //逐列取出
            int valid_segs_cnt_in_1col;
            vector<Range> valid_segs_in_1col = CalcConnectivity(current_col, valid_segs_cnt_in_1col);
//            for(auto seg : valid_segs_in_1col) {
//                cout << "列" << col << " : " << seg.start << " " << seg.end << endl;
//            }

            if(valid_segs_cnt_in_last_col == 0) {
                split_regions_indexes.clear();
                for(int i=0; i<valid_segs_cnt_in_1col; ++i){
                    split_regions_indexes.push_back(regions_cnt);
                    regions_cnt += 1;
                }
            }else if(valid_segs_cnt_in_1col == 0) {
                split_regions_indexes.clear();
//        continue;
            }else{
                Mat adj_matrix = GetAdjMatrix(valid_segs_in_last_col, valid_segs_in_1col);
                vector<int> temp_regions_indexes(valid_segs_cnt_in_1col, 0);    //temp_regions_indexes用来考察当前列在之前结果基础上的区域划分
//                for(int i=0; i<valid_segs_cnt_in_1col; ++i) {
//                    temp_regions_indexes.push_back(0);
//                }

                //i代表左侧列第n段，j代表右侧列第n段
                for(int i=0; i<adj_matrix.rows; ++i) {
                    Mat row = adj_matrix.row(i);
                    if (sum(row) == 1) {        //情况1：左列某1段和右列某1段仅有一段邻接
                        for(int j=0; j<row.cols; ++j) {
                            if(row.at<u_char>(j)>0) {
                                temp_regions_indexes[j] = split_regions_indexes[i];     //把左列的区域编号赋值给右列，以前->现在
                                break;      //因为只有1段邻接，没必要继续遍历其余列了
                            }
                        }
                    } else if(sum(row) > 1) {   //情况2：左列1段和右列多段有邻接
                        for(int j=0; j<row.cols; ++j) {
                            if(row.at<u_char>(j)>0) {
                                temp_regions_indexes[j] = regions_cnt;
                                regions_cnt += 1;
                                //右列中至少2段与左侧同一段邻接，说明右列被（障碍物）打断了，因此多了2个新的分区，故总计数++
                            }
                        }
                    }
                }

                //对以上分类讨论的补充，i代表右侧列第n段
                for(int i=0; i<adj_matrix.cols; ++i) {
                    Mat col = adj_matrix.col(i);
                    if(sum(col) > 1 ) {
                        //情况3：对应右列某段与左列至少2段邻接的情况，对情况1的修正
                        temp_regions_indexes[i] = regions_cnt++;
                    }else if(sum(col) == 0) {
                        //情况4：对应右列某段与左列都不邻接的情况
                        temp_regions_indexes[i] = regions_cnt++;
                    }
                }
                split_regions_indexes = temp_regions_indexes;
            }

            for(int i=0; i<valid_segs_cnt_in_1col; ++i) {
                seperate_map(Range(valid_segs_in_1col[i].start, valid_segs_in_1col[i].end+1), Range(col, col+1)) = split_regions_indexes[i];
            }
            valid_segs_in_last_col = valid_segs_in_1col;
            valid_segs_cnt_in_last_col = valid_segs_cnt_in_1col;

        }
        return seperate_map;
    }

    Mat DisplaySeparateMap(const Mat &separate_map, int regions_cnt) {
        Mat display_mat(separate_map.size(), CV_8UC3, cv::Scalar::all(0));

        Mat random_colors;
        random_colors.create(regions_cnt, 3, CV_8UC1);
        cv::RNG rnger(1);
        rnger.fill(random_colors, cv::RNG::UNIFORM, cv::Scalar::all(0), cv::Scalar::all(255));

        for(int region_index=0; region_index<regions_cnt; ++region_index) {
            Mat color = random_colors.row(region_index);
            for(int i=0; i<separate_map.rows; ++i) {
                for(int j=0; j<separate_map.cols; ++j) {
                    if(separate_map.at<u_char>(i,j) == region_index) {
                        display_mat.at<Vec3b>(i,j) = color;
                    }
                }
            }
//            imshow("separate_map", display_mat);
//            waitKey();
        }

        return display_mat;
    }

    void SplitToRegions(const Mat &entire_map, int regions_cnt) {
        vector<vector<Point>> contours;
        Mat temp(entire_map.size(), CV_8UC3, Scalar::all(0)), entire_graymap;

        cvtColor(entire_map, entire_graymap, COLOR_BGR2GRAY);
        imshow("gray map", entire_graymap);
        if(hist(entire_graymap) != regions_cnt) {
            cout << hist(entire_graymap) << "  " << regions_cnt << endl;
            cout << "Error: has reduplicate gray values." << endl;
            return;
        }

        map<int, Mat> regions;
        vector<int> grayvalues(1, 0);
        for(int i=0; i<entire_graymap.rows; i++) {
            for(int j=0; j<entire_graymap.cols; j++) {
                int v = entire_graymap.at<u_char>(i,j);
                if(find(grayvalues.begin(), grayvalues.end(), v) == grayvalues.end()) {
                    grayvalues.emplace_back(v);
                    Mat reg(entire_graymap.size(), CV_8UC1, Scalar::all(0));
                    regions.insert({v, reg});
                }
                if(v != 0) {
                    regions[v].at<u_char>(i,j) = 255;
                }
            }
        }
        if(regions.size() != regions_cnt-1 || grayvalues.size() != regions_cnt) {
            cout << "Error: regions' count and split are wrong." << endl;
            return;
        }
        vector<vector<Point>> turn_points;
        for(auto &elem : regions) {
            string winname("grayvalue ");
            winname += to_string(elem.first);
            int expand_pixels = 20;
            copyMakeBorder(elem.second, elem.second, expand_pixels, expand_pixels, expand_pixels, expand_pixels,
                           cv::BORDER_CONSTANT, Scalar::all(0));
            imshow(winname, elem.second);
            CalcBow(elem.second, expand_pixels, turn_points);
        }

        ShowBow(entire_map, turn_points);

    }

    void CalcBow(Mat region, int expand_pixels, vector<vector<Point>> &turn_points) {
        int robot_radius = 5, overlap = 5;
        cv::Mat element = getStructuringElement(cv::MORPH_RECT, cv::Size(2 * robot_radius, 2 * robot_radius),
                                                cv::Point(robot_radius, robot_radius));
        threshold(region, region, 1, 255, THRESH_BINARY_INV);   //反色
        Mat reg_dilate;
        cv::dilate(region, reg_dilate, element);
        int lastv = 0;
        vector<Point> tps;
        for(int j=0; j<reg_dilate.cols; j+=overlap) {
            for(int i=0; i<reg_dilate.rows; i++) {
                int v = reg_dilate.at<u_char>(i,j);
                if(v != lastv) {
                    circle(region, Point(j,i), 2, Scalar(128), -1);
                    tps.emplace_back(Point(j-expand_pixels, i-expand_pixels));
                }
                lastv = v;
            }
        }
        imshow("region", region);
        waitKey();
        ReOrderBow(tps);
        turn_points.emplace_back(tps);
    }

    template <typename T>
    void ReOrderBow(vector<T> &tps) {
        if(IsOdd(tps.size())) {
            cout << "Warning: points num is odd." << endl;
//            tps.pop_back();
        }
        vector<T> temp;
        bool turn = false;
        for(int i=0; i<tps.size(); i=i+2) {
            if(turn) {
                temp.emplace_back(tps[i+1]);
                temp.emplace_back(tps[i]);
                turn = false;
            } else {
                temp.emplace_back(tps[i]);
                temp.emplace_back(tps[i+1]);
                turn = true;
            }
        }
        tps = temp;
    }

    Mat ShowBow(Mat entire_map, const vector<vector<Point>> &turn_points) {
        for(const auto &pts : turn_points) {
            for(int i=0; i<pts.size()-1; i++) {
                line(entire_map, pts[i], pts[i+1], Scalar(0,255,0));
            }
        }
        imshow("Bow", entire_map);
        waitKey();
        return entire_map;
    }

};

#endif //BOUSTROPHEDON_BOUSTROPHEDON_H
