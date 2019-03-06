//
// Created by gjs on 19-3-6.
//

#include <algorithm>
#include <iostream>

using namespace std;

int main() {
    vector<int> hist_result{2,3,4,5,6,7,0,0,0,0,9};
    int res = count_if(hist_result.begin(), hist_result.end(), [](int v){ return v != 0;});
    cout << res << endl;
}