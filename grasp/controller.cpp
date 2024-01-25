/////////////////////////////////////////////////////////////////////////////////////////
// Program main

#include "can_communicator.h"
#include "opencv2/highgui/highgui.hpp"
#include <iostream>
#include <vector>
#include <string>
#include <map> 

/////////////////////////////////////////////////////////////////////////////////////////
// Application main-loop. It handles the commands from rPanelManipulator and keyboard events

std::vector<int> jointValues(16, 0);
std::vector<int> jointIndices(16); 
std::map<int, std::tuple<int,int>> jointLimits = {
    {0, std::make_tuple(-0.47471316618668479, 0.47181227113054078)},
    {1, std::make_tuple(-0.19691276729768068, 1.6367399715833842)},
    {2, std::make_tuple(-0.17401187224153672, 1.7098808147084331)},
    {3, std::make_tuple(-0.22753605719833834, 1.61854352396125431)},

    {4, std::make_tuple(-0.49471316618668479, 0.47181227113054078)},
    {5, std::make_tuple(-0.19691276729768068, 1.6367399715833842)},
    {6, std::make_tuple(-0.17401187224153672, 1.7098808147084331)},
    {7, std::make_tuple(-0.22753605719833834, 1.61854352396125431)},

    {8, std::make_tuple(-0.49471316618668479, 0.47181227113054078)},
    {9, std::make_tuple(-0.19691276729768068, 1.6367399715833842)},
    {10, std::make_tuple(-0.17401187224153672, 1.7098808147084331)},
    {11, std::make_tuple(-0.22753605719833834, 1.61854352396125431)},

    {12, std::make_tuple(0.2635738998060688, 1.3968131524486665)},
    {13, std::make_tuple(-0.10504289759570773, 1.1630997544532125)},
    {14, std::make_tuple(-0.18972295140796106, 1.6440185506322363)},
    {15, std::make_tuple(-0.16220637207693537, 1.7199110516903878)}
};

void MainLoop();

static void on_trackbar(int pos, void* userdata)
{
    int jointIndex = *static_cast<int*>(userdata);
    jointValues[jointIndex] = pos;
    std::cout << "Joint " << jointIndex + 1 << " value: " << pos << std::endl;
}

void MainLoop()
{
    int desiredWidth = 640, desiredHeight = 480;
    cv::namedWindow("Joint Controller", cv::WINDOW_NORMAL);
    cv::resizeWindow("Joint Controller", desiredWidth, desiredHeight);

    for (int i = 0; i < 16; ++i)
    {
        jointIndices[i] = i; 
        std::string trackbarName = "Joint " + std::to_string(i + 1);
        cv::createTrackbar(trackbarName, "Joint Controller", NULL, 114, on_trackbar, &jointIndices[i]);
    }
    
    bool bRun = true;
    while (bRun)
    {
        cv::imshow("Joint Controller", cv::Mat::zeros(desiredHeight, desiredWidth, CV_8UC1));

        cv::waitKey(50000);

        // int c = Getch();
        // switch (c)
        // {
        // case 'q':
        //     if (pBHand) pBHand->SetMotionType(eMotionType_NONE);
        //     bRun = false;
        //     break;

        // case 'h':
        //     if (pBHand) pBHand->SetMotionType(eMotionType_HOME);
        //     break;

        // case 'r':
        //     if (pBHand) pBHand->SetMotionType(eMotionType_READY);
        //     break;

        // case 'g':
        //     if (pBHand) pBHand->SetMotionType(eMotionType_GRASP_3);
        //     break;

        // case 'k':
        //     if (pBHand) pBHand->SetMotionType(eMotionType_GRASP_4);
        //     break;

        // case 'p':
        //     if (pBHand) pBHand->SetMotionType(eMotionType_PINCH_IT);
        //     break;

        // case 'm':
        //     if (pBHand) pBHand->SetMotionType(eMotionType_PINCH_MT);
        //     break;

        // case 'a':
        //     if (pBHand) pBHand->SetMotionType(eMotionType_GRAVITY_COMP);
        //     break;

        // case 'e':
        //     if (pBHand) pBHand->SetMotionType(eMotionType_ENVELOP);
        //     break;

        // case 'f':
        //     if (pBHand) pBHand->SetMotionType(eMotionType_NONE);
        //     break;

        // case '1':
        //     MotionRock();
        //     break;

        // case '2':
        //     MotionScissors();
        //     break;

        // case '3':
        //     MotionPaper();
        //     break;
        
        // case '7': 
        //     MotionCustom1();
        //     break;
        
        // case '8':
        //     MotionCustom2();
        //     break;

        // case '0':
        //     resetHand();
        //     break;
        // }
    }
}

int main(int argc, TCHAR* argv[])
{
    // PrintInstruction();

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