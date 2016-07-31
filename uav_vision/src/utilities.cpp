#include "include/utilities.h"
#include <vector>
#include <iostream>
#include <math.h>
#define HsvType int

using namespace std;
using namespace cv;
Mat Utilities::colorConversion(Mat img, HsvType imgtype){

    GaussianBlur(img, img, Size(5,5),0,0);

    if(imgtype==HSV||imgtype==HSV_H||imgtype==HSV_S||imgtype==HSV_V){
        Mat img_hsv, img_h, img_s, img_v;
        cvtColor(img, img_hsv, CV_BGR2HSV);
        vector<Mat> hsv_channels;
        split(img_hsv, hsv_channels);
        img_h = hsv_channels[0];
        img_s = hsv_channels[1];
        img_v = hsv_channels[2];
        switch (imgtype){
        case 0:
            return img_hsv;
        case 1:
            return img_h;
        case 2:
            return img_s;
        case 3:
            return img_v;
        }
    }

    if(imgtype==BGR||imgtype==BGR_B||imgtype==BGR_G||imgtype==BGR_R){
        Mat b_img, g_img, r_img;
        vector<Mat> bgr_channels;
        split(img,bgr_channels);
        b_img=bgr_channels[0];
        g_img=bgr_channels[1];
        r_img=bgr_channels[2];

        switch(imgtype){
        case 4:
            return img;
        case 5:
            return b_img;
        case 6:
            return g_img;
        case 7:
            return r_img;
        }
        return img;
    }
}

Mat Utilities::decectMarkerContour(Mat img){

    Mat img_h, img_threshold, morph_img;
    Mat img_canny(img.rows, img.cols,CV_8UC3,Scalar(0,0,0));
    Mat img_show(img.rows, img.cols,CV_8UC3,Scalar(0,0,0));

    img_h=colorConversion(img, BGR_R);
    threshold(img_h,img_threshold,60,255,1);
    Mat element = getStructuringElement(MORPH_RECT,Size(3,3));
    morphologyEx(img_threshold,img_threshold,MORPH_ERODE,element);
    //imshow("1",img_threshold);
    Canny(img_threshold,img_canny,30,90);
    //imshow("2",img_canny);

    //Mat element = getStructuringElement(MORPH_RECT,Size(5,5));
    //morphologyEx(img_threshold,morph_img,MORPH_CLOSE,element);

    vector<vector<Point>> contours;
    vector<Point> approxCurve;
    vector<Vec4i> hierarchy;
    bool longEdge = 1;
    float markerArea = 0;
    bool sign_bigger = false;
    bool sign_small = false;
    Point2f pointsIn[4]={Point2f(0,0)};
    namedWindow("img_show");

    //vector<Rect> box(contours.size());
    findContours(img_canny,contours,hierarchy,RETR_TREE,CHAIN_APPROX_NONE);

    approxCurve.resize(contours.size());
    for(unsigned int i=0; i<contours.size();i++){

        float contour_area=contourArea(contours[i]);


        //marker
        if(contour_area>20000){  //remove the noise
            markerArea = contour_area;
            //hierarchy[i][3]<0 &&
            //box[i]=boundingRect(Mat(contours[i]));
            approxPolyDP(contours[i], approxCurve, double(contours[i].size()) * 0.05, true);
            if (isContourConvex(Mat(approxCurve))) {
                sign_bigger = true;
                int min_x = INT_MAX;
                int min_index = 0;
                for(int j=0;j<4;j++){
                    //circle(img_show,Point(approxCurve[j].x,approxCurve[j].y),8,Scalar(0,0,255));
                    //cout << approxCurve[j] << endl;
                    if(sqrt(approxCurve[j].x*approxCurve[j].x+approxCurve[j].y*approxCurve[j].y) < min_x){
                        min_x = sqrt(approxCurve[j].x*approxCurve[j].x+approxCurve[j].y*approxCurve[j].y);
                        min_index = j;
                    }
                }

                if(min_index == 0 ){
                    for(int j=0;j<4 ;j++){
                        pointsIn[j] = approxCurve[j];
                        circle(img_show,Point(pointsIn[j].x,pointsIn[j].y),8,Scalar(0,0,255));
                    }
                }
                else{
                    vector<int> tmp={1,2,3,0,1,2,3,};
                    vector<int>::iterator tmp_itr = tmp.begin();
                    int k=0;
                    tmp_itr = find(tmp.begin(),tmp.end(),min_index);


                    for(;min_index!=0 && tmp_itr!=tmp.end() && k!=4;tmp_itr++){
                        pointsIn[k++] = approxCurve[*tmp_itr];
                        circle(img_show,Point(pointsIn[k-1].x,pointsIn[k-1].y),8,Scalar(0,0,255));
                    }
                }

                drawContours(img_show,contours,i,Scalar(0,255,0),2,8);

                //rectangle(src_frame, boundRect[i], Scalar(0,0,255), 2,8,0);
            }


        }
        //mini rectangle
        else if(sign_bigger = true && markerArea != 0 && 0.01*markerArea < contour_area && contour_area < 0.03*markerArea){
            //approxPolyDP(contours[i], approxCurve, double(contours[i].size()) * 0.05, true);
            //approxPolyDP(contours[i], approxCurve, double(contours[i].size()) * 0.05, true);
            sign_small = true;
            //            int longEdge=0;
            //            if(longEdge == 1 && abs(sqrt(approxCurve[0].x*approxCurve[0].x+approxCurve[0].y*approxCurve[0].y)-sqrt(approxCurve[1].x*approxCurve[1].x+approxCurve[1].y*approxCurve[1].y)) < abs(sqrt(approxCurve[0].x*approxCurve[0].x+approxCurve[0].y*approxCurve[0].y)-sqrt(approxCurve[3].x*approxCurve[3].x+approxCurve[3].y*approxCurve[3].y)))
            //                longEdge = 0;
            //drawContours(img_show,contours,i,Scalar(0,255,0),2,8);
        }

        if(sign_bigger == true && sign_small == true){
            Mat move_pos = getPosition(img_h, pointsIn);

            //cout << "*******************" << endl;
        }
        imshow("img_show",img_show);
    }


    return img;
}


Mat Utilities::getPosition(Mat &img, Point2f* pointsIn){

    Mat img_out(400,400,CV_8UC1,Scalar(0,0,0));
    Point2f pointsRes[4];

    //Mat imgROI = img(Rect(pointsIn[0].x,pointsIn[1].y),)

    pointsRes[0] = Point2f(0, 0);
    pointsRes[1] = Point2f(400 - 1, 0);
    pointsRes[2] = Point2f(400 - 1, 400 - 1);
    pointsRes[3] = Point2f(0, 400 - 1);

    vector<Point3f> objectPoints;
    objectPoints.push_back(Point3f(0,0,0));
    objectPoints.push_back(Point3f(399,0,0));
    objectPoints.push_back(Point3f(399,0,0));
    objectPoints.push_back(Point3f(0,399,0));
    vector<Point2f> imagePoints;    
    imagePoints.push_back(Point2f(pointsIn[0].x,pointsIn[0].y));
    imagePoints.push_back(Point2f(pointsIn[1].x,pointsIn[1].y));
    imagePoints.push_back(Point2f(pointsIn[2].x,pointsIn[2].y));
    imagePoints.push_back(Point2f(pointsIn[3].x,pointsIn[3].y));

    float tmp1[3][4] = {{720.0694,0,319.5}
                        ,{0, 720.0694, 239.5}
                        ,{0,0,1}};
    Mat cameMatrix(3,3,CV_32F, tmp1);
//    cameMatrix[0][0] = 720.0694;
//    cameMatrix[1][1] = 720.0694;
//    cameMatrix[0][2] = 319.5;
//    cameMatrix[1][2] = 239.5;

    //float tmp2[3][4] = {{-0.175335,0,0.360192,0},{0, 720.0694, -1.464377,0},{0,0,0,0}};
    Mat distCoeffs;
//    distCoeffs[0][0] = -0.175335;
//    distCoeffs[1][1] = 0.360192;
//    distCoeffs[0][2] = 0;
//    distCoeffs[1][2] = -1.464377;

    Mat rvec(3,3,CV_32F), tvec(3,3,CV_32F);

    solvePnP(objectPoints,imagePoints,cameMatrix,distCoeffs,rvec,tvec);
    //Mat M = getPerspectiveTransform(pointsIn, pointsRes);
    //cout << M << endl;
    //warpPerspective(img, img_out, M, Size(400,400), cv::INTER_NEAREST);
    cout<<"rvec:"<<rvec<<endl;
    cout<<"tvec:"<<tvec<<endl;
    cout<<"*****************************"<<endl;
    //namedWindow("img_out");
    //imshow("img_out",img_out);
    return rvec;
}



Utilities::Utilities()
{

}
