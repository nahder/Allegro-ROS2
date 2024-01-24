/////////////////////////////////////////////////////////////////////////////////////////
// Program main

#include "can_communicator.h"
#include "opencv2/highgui/highgui.hpp"
#include <iostream>
/////////////////////////////////////////////////////////////////////////////////////////
// Application main-loop. It handles the commands from rPanelManipulator and keyboard events
void MainLoop();

void MainLoop()
{
    int desiredWidth=640,desiredHeight=480;
    int j1{0};

    cv::namedWindow("Joint Controller", cv::WINDOW_NORMAL);
    cv::resizeWindow("Joint Controller", desiredWidth,desiredHeight);
    cv::createTrackbar("j1","Joint Controller", &j1, 114);

    cv::imshow("Joint Controller", cv::Mat::zeros(desiredHeight, desiredWidth, CV_8UC1));
    cv::waitKey(1);

    bool bRun = true;
    while (bRun)
    {
        int c = Getch();
        switch (c)
        {
        case 'q':
            if (pBHand) pBHand->SetMotionType(eMotionType_NONE);
            bRun = false;
            break;

        case 'h':
            if (pBHand) pBHand->SetMotionType(eMotionType_HOME);
            break;

        case 'r':
            if (pBHand) pBHand->SetMotionType(eMotionType_READY);
            break;

        case 'g':
            if (pBHand) pBHand->SetMotionType(eMotionType_GRASP_3);
            break;

        case 'k':
            if (pBHand) pBHand->SetMotionType(eMotionType_GRASP_4);
            break;

        case 'p':
            if (pBHand) pBHand->SetMotionType(eMotionType_PINCH_IT);
            break;

        case 'm':
            if (pBHand) pBHand->SetMotionType(eMotionType_PINCH_MT);
            break;

        case 'a':
            if (pBHand) pBHand->SetMotionType(eMotionType_GRAVITY_COMP);
            break;

        case 'e':
            if (pBHand) pBHand->SetMotionType(eMotionType_ENVELOP);
            break;

        case 'f':
            if (pBHand) pBHand->SetMotionType(eMotionType_NONE);
            break;

        case '1':
            MotionRock();
            break;

        case '2':
            MotionScissors();
            break;

        case '3':
            MotionPaper();
            break;
        
        case '7': 
            MotionCustom1();
            break;
        
        case '8':
            MotionCustom2();
            break;

        case '0':
            resetHand();
            break;
        }
    }
}

int main(int argc, TCHAR* argv[])
{
    PrintInstruction();

    memset(&vars, 0, sizeof(vars));
    memset(q, 0, sizeof(q));
    memset(q_des, 0, sizeof(q_des));
    memset(tau_des, 0, sizeof(tau_des));
    memset(cur_des, 0, sizeof(cur_des));
    curTime = 0.0;

    if (CreateBHandAlgorithm() && OpenCAN())
        MainLoop();

    CloseCAN();
    DestroyBHandAlgorithm();
    return 0;
}