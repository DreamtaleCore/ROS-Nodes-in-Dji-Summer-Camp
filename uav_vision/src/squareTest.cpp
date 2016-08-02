#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "squareDetect.h"
#include <iostream>
#include <iostream>
#include <vector>
using namespace cv;
using namespace std;

int main()
{
    float time_start=getTickCount();
    Mat img_src = imread("/home/yf/Pictures/fub1.jpg");
    if (!img_src.data)
        return 1;
    imshow("Original Image", img_src);

    SquareDetect test;
    RotatedRect box = test.squareDet(img_src);
    Point2f vertex[4];
    box.points(vertex);
    for(int i=0;i<4;i++){
        line(img_src,vertex[i],vertex[(i+1)%4],Scalar(255,0,0),5);
    }
    //rectangle(img_src,box,Scalar(0,255,0),5);

    Mat imgROI = test.dollDetect(img_src,box);
    time_start = getTickCount() - time_start;
    cout << "run time:" << time_start/getTickFrequency() << "s" << endl;
    imshow("1",img_src);
    waitKey(0);
    return 0;
}

