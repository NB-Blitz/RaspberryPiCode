// OpenCV.cpp : This file contains the 'main' function. Program execution begins and ends there. 
/* Code by Soham feat. Ruilin */

#include <iostream>
#include <opencv2/core/core.hpp> 
#include <opencv2/imgproc.hpp> 
#include <opencv2/opencv.hpp> 
#include <opencv2/imgproc/imgproc.hpp> 
#include <stdio.h>
#include <stdlib.h> 
#include "cameraserver/CameraServer.h"
#include "cscore_oo.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableInstance.h"

using namespace cv;
using namespace std;

int main()
{

    // Create a VideoCapture object and use camera to capture the video 
    VideoCapture cap(0);

    // Check if camera opened successfully 
    if (!cap.isOpened())
    {
        cout << "Error opening video stream" << endl;
        return -1;
    }

    /*
    namedWindow("gui");
    createTrackbar("r min", "gui", 0, 255);
    createTrackbar("g min", "gui", 0, 255);
    createTrackbar("b min", "gui", 0, 255);
    createTrackbar("r max", "gui", 0, 255);
    createTrackbar("g max", "gui", 0, 255);
    createTrackbar("b max", "gui", 0, 255);

    setTrackbarPos("r min", "gui", 246);
    setTrackbarPos("g min", "gui", 255);
    setTrackbarPos("b min", "gui", 0);
    setTrackbarPos("r max", "gui", 255);
    setTrackbarPos("g max", "gui", 255);
    setTrackbarPos("b max", "gui", 255);
    */

    //frc::CameraServer::GetInstance()->StartAutomaticCapture();
    //cs::CvSink cvSink = frc::CameraServer::GetInstance()->GetVideo();
    cs::CvSource cvSource("cvsource", cs::VideoMode::kMJPEG, 320, 240, 30);
    cs::MjpegServer cvMjpegServer("serve_USB Camera 0", 1181);
    cvMjpegServer.SetSource(cvSource);

    nt::NetworkTableEntry xEntry;
    auto inst = nt::NetworkTableInstance::GetDefault();
    auto table = inst.GetTable("datatable");
    xEntry = table->GetEntry("X");

    while (1)
    {
        Mat frame0;

        // Capture frame-by-frame  
        cap >> frame0;

        // If the frame is empty, break immediately 
        if (frame0.empty())
            break;

        // RGB threshold 
        int rMin = 246;
        int gMin = 255;
        int bMin = 0;
        int rMax = 255;
        int gMax = 255;
        int bMax = 255;
        Mat rgbThreshold;
        cv::inRange(frame0, cv::Scalar(rMin, gMin, bMin), cv::Scalar(rMax, gMax, bMax), rgbThreshold);

        // Blur 
        Mat blur;
        medianBlur(rgbThreshold, blur, 11);

        // Display the resulting frame     
        //imshow("Frame0", blur); 

        //Find contours 
        vector<vector<Point> > contours;
        findContours(blur, contours, 1, 1);

        Mat contourDrawing = Mat::zeros(blur.size(), CV_8UC3);

        vector<vector<Point> > contours_poly(contours.size());
        vector<Rect> boundRect(contours.size());
        vector<Point2f>center(contours.size());
        vector<float>radius(contours.size());

        for (int i = 0; i < contours.size(); i++) {
            try {

                approxPolyDP(Mat(contours[i]), contours_poly[i], 3, true);
                boundRect[i] = boundingRect(Mat(contours_poly[i]));
                minEnclosingCircle((Mat)contours_poly[i], center[i], radius[i]);
            }
            catch (exception e) { cout << "exception"; }
        }

        //Draw contours 
        drawContours(contourDrawing, contours, -1, Scalar(200, 200, 200), -1);
        //namedWindow("Contours");


        Point cameraCenter = Point(contourDrawing.cols / 2.0, contourDrawing.rows / 2.0); //Center of the camera 
        Point largestPosition; //Position of the largest contour 

        if (contours.size() != 0) {

            //Find largest contour (code from OpenCV documentation) 
            int largest(0);
            Rect rect;
            Rect largestRect;
            for (int i = 0; i < contours.size(); i++) {
                rect = boundRect.at(i);
                if (rect.width * rect.height > largestRect.width* largestRect.height) {
                    largest = i;
                    largestRect = boundRect.at(i);
                }
            }
            largestPosition = center[largest];

            //Draw position of contour 
            rectangle(contourDrawing, boundRect[largest].tl(), boundRect[largest].br(), Scalar(0, 255, 0)); //Rectangle around the contour 
            circle(contourDrawing, largestPosition, 5, Scalar(0, 0, 255), -1); //Center of the contour 

            cout << largestPosition;

        }
        else
            largestPosition = cameraCenter;

        //Find direction to move 
        int direction = 0; //1 is right, -1 is left 
        circle(contourDrawing, cameraCenter, 9, Scalar(0, 255, 255));
        if (abs(largestPosition.x - cameraCenter.x) < 5)
            direction = 0;
        else if (largestPosition.x > cameraCenter.x)
            direction = 1;
        else if (largestPosition.x < cameraCenter.x)
            direction = -1;
        arrowedLine(contourDrawing, cameraCenter, Point(cameraCenter.x + 16 * direction, cameraCenter.y), Scalar(0, 255, 255), 3, 0, 0, 0.5); //Display direction 

        //Display window 
        try {
            //imshow("Contours", contourDrawing);
            //cs::CvSource outputStream = frc::CameraServer::GetInstance()->PutVideo("Contours", contourDrawing.cols, contourDrawing.rows);

            //value = (x - w/2) / (w/2)
            double value = ((1.0*largestPosition.x) - (1.0*cameraCenter.x)) / (1.0*cameraCenter.x);
            putText(contourDrawing, to_string(value), cameraCenter, FONT_HERSHEY_SIMPLEX, 1, Scalar(255,0,0), 2);
            xEntry.SetDouble(value);
            cvSource.PutFrame(contourDrawing);
        }
        catch (exception e) {}

        // Press  ESC on keyboard to  exit 
        /*
        char c = (char)waitKey(1);
        if (c == 27)
            break;*/
    }

    // When everything done, release the video capture and write object 
    cap.release();
    return 0;

}