#ifndef BOUSTROPHEDON_BOUSTROPHEDON_H
#define BOUSTROPHEDON_BOUSTROPHEDON_H

#include <vector>
#include <math.h>
#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;
using namespace std;


struct biTuple
{
biTuple(int a,int b) {
    this->a=a;
    this->b=b;
}

int a;
int b;
};

typedef std::vector<biTuple> Slice;

class Boustrophedon{
public:
    static Slice calc_connectivity(const Mat &slice, int &connectivity){
        connectivity=0;
        int last_data=1;
        bool open_start=false;
        Slice connective_parts;
        int start_point=0;
        int end_point=0;
        for(int i=0;i<slice.rows;++i){
            int data=slice.at<u_char>(i,0);
            if(last_data==1 && data==0){
                open_start=true;
                start_point=i;
            }else if((last_data==0 && data==1 && open_start)||(open_start && i==slice.rows-1)){
                open_start=false;
                connectivity+=1;
                end_point=i;
                connective_parts.push_back(biTuple(start_point,end_point));
            }
            last_data=data;
        }
        return connective_parts;
    }

    static Mat get_adjecency_matrix(const Slice &parts_left,const Slice &parts_right){
        Mat adjacency_mat(parts_left.size(),parts_right.size(),CV_8UC1);
        for(int i=0;i<parts_left.size();++i){
            biTuple lparts=parts_left[i];
            for(int j=0;j<parts_right.size();++j){
                biTuple rparts=parts_right[j];
                if((min(lparts.b,rparts.b) - max(lparts.a,rparts.a))>0){
                    adjacency_mat.at<u_char>(i,j)=1;
                }else{
                    adjacency_mat.at<u_char>(i,j)=0;
                }
            }
        }
        return adjacency_mat;
    }

    static Mat calc_bcd(const Mat &map, int &current_cell){
        int last_connectivity =0;
        Slice last_connective_parts;
        current_cell=1;
        Mat current_cells;
        Mat seperate_map;
        map.copyTo(seperate_map);

        for(int col=0;col<map.cols;++col){
            Mat current_slice=map.col(col);
            int connectivity;
            Slice connective_parts=calc_connectivity(current_slice,connectivity);

            if(last_connectivity==0){
                current_cells.release();
                for(int i=0;i<connectivity;++i){
                    current_cells.push_back(current_cell);
                    current_cell+=1;
                }
            }else if(connectivity==0){
                current_cells.release();
//        continue;
            }else{
                Mat adj_matrix=get_adjecency_matrix(last_connective_parts,connective_parts);
                Mat new_cells;
                for(int i=0;i<connectivity;++i){
                    new_cells.push_back(0);
                }

                for(int i=0;i<adj_matrix.rows;++i){
                    Mat row=adj_matrix.row(i);
                    if (sum(row)==1){
                        for(int j=0;j<row.cols;++j){
                            if(row.at<u_char>(j)>0){
                                new_cells.at<u_char>(j)=current_cells.at<u_char>(i);
                                break;
                            }
                        }
                    }else if(sum(row)>1){
                        for(int j=0;j<row.cols;++j){
                            if(row.at<u_char>(j)>0){
                                new_cells.at<u_char>(j)=current_cell;
                                current_cell+=1;
                            }
                        }
                    }
                }

                for(int i=0;i<adj_matrix.cols;++i){
                    Mat col=adj_matrix.col(i);
                    if(sum(col)>1){
                        new_cells.at<u_char>(i)=current_cell++;
                    }else if(sum(col)==0){
                        new_cells.at<u_char>(i)=current_cell++;
                    }
                }
                new_cells.copyTo(current_cells);
            }

            for(int i=0;i<connectivity;++i){
                seperate_map(Range(connective_parts[i].a,connective_parts[i].b),Range(col,col+1))=current_cells.at<u_char>(i);
            }
            last_connective_parts=connective_parts;
            last_connectivity=connectivity;

        }
        return seperate_map;
    }

    static int sum(const Mat &mat){
        int sum=0;
        for(int i=0;i<mat.rows;i++){
            for(int j=0;j<mat.cols;j++){
                sum+=mat.at<u_char>(i,j);
            }
        }
        return sum;
    }

    static int min(const int &a,const int &b){
        return(a>b?b:a);
    }
    static int max(const int &a,const int &b){
        return(a>b?a:b);
    }

    static Mat display_separate_map(const Mat &separate_map,const int &cells){
        int rows=separate_map.rows;
        int cols=separate_map.cols;
        Mat display_mat(rows,cols,CV_8UC3);
        Mat random_colors;
        random_colors.create(cells,3,CV_8UC1);
        cv::RNG rnger(cv::getTickCount());
        rnger.fill(random_colors,cv::RNG::UNIFORM,cv::Scalar::all(0),cv::Scalar::all(256));

        for(int cell=0;cell<cells;++cell){
            Mat color=random_colors.row(cell);
            for(int i=0;i<rows;++i){
                for(int j=0;j<cols;++j){
                    if(separate_map.at<u_char>(i,j)==cell){
//            display_mat.at<Vec3b>(i,j)[0]=color.at<u_char>(0);
//            display_mat.at<Vec3b>(i,j)[1]=color.at<u_char>(1);
//            display_mat.at<Vec3b>(i,j)[2]=color.at<u_char>(2);
                        display_mat.at<Vec3b>(i,j)=color;
                    }
                }
            }
        }

        return display_mat;
    }
};

#endif //BOUSTROPHEDON_BOUSTROPHEDON_H
