#include "ORB-SLAM/Viewer.h"
#include <pangolin/pangolin.h>
#include <mutex>
#include <unistd.h>

namespace ORB_SLAM2
{
    Viewer::Viewer(System* pSystem, FrameDrawer *pFrameDrawer, MapDrawer *pMapDrawer, const std::vector<std::pair<std::string, std::string>>& parameters)
        :
        mpSystem(pSystem),
        mpFrameDrawer(pFrameDrawer),
        mpMapDrawer(pMapDrawer),
        mbFinishRequested(false),
        mbFinished(true),
        mbStopped(true),
        mbStopRequested(false)
    {
        int setCount = 0;
        for (const std::pair<std::string, std::string>& parameter : parameters)
        {
            try
            {
                if (parameter.first == "Viewer_ViewpointX") { mViewpointX = std::stof(parameter.second); setCount++; }
                else if (parameter.first == "Viewer_ViewpointY") { mViewpointY = std::stof(parameter.second); setCount++; }
                else if (parameter.first == "Viewer_ViewpointZ") { mViewpointZ = std::stof(parameter.second); setCount++; }
                else if (parameter.first == "Viewer_ViewpointF") { mViewpointF= std::stof(parameter.second); setCount++; }
            }
            catch ( ... )
            {
                std::cout << "VSLAM - could not set parameter: " << parameter.first << " with << " << parameter.second << "." << std::endl;
                exit(-1);
            }
        }

        if (setCount != 4)
        {
            std::cout << "VSLAM - required viewer parameter not provided." << std::endl;
            exit(-1);
        }


        // Run viewer at 30.0 fps.
        mT = 1000.0 / 30.0;
    }

    void Viewer::Run()
    {
        mbFinished = false;
        mbStopped = false;

        pangolin::CreateWindowAndBind("ORB-SLAM2: Map Viewer",1024,768);

        // 3D Mouse handler requires depth testing to be enabled
        glEnable(GL_DEPTH_TEST);

        // Issue specific OpenGl we might need
        glEnable (GL_BLEND);
        glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        pangolin::CreatePanel("menu").SetBounds(0.0,1.0,0.0,pangolin::Attach::Pix(175));
        pangolin::Var<bool> menuFollowCamera("menu.Follow Camera",true,true);
        pangolin::Var<bool> menuShowPoints("menu.Show Points",true,true);
        pangolin::Var<bool> menuShowKeyFrames("menu.Show KeyFrames",true,true);
        pangolin::Var<bool> menuShowGraph("menu.Show Graph",true,true);
        pangolin::Var<bool> menuLocalizationMode("menu.Localization Mode",false,true);
        pangolin::Var<bool> menuReset("menu.Reset",false,false);

        // Define Camera Render Object (for view / scene browsing)
        pangolin::OpenGlRenderState s_cam(
                    pangolin::ProjectionMatrix(1024,768,mViewpointF,mViewpointF,512,389,0.1,1000),
                    pangolin::ModelViewLookAt(mViewpointX,mViewpointY,mViewpointZ, 0,0,0,0.0,-1.0, 0.0)
                    );

        // Add named OpenGL viewport to window and provide 3D Handler
        pangolin::View& d_cam = pangolin::CreateDisplay()
                .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f/768.0f)
                .SetHandler(new pangolin::Handler3D(s_cam));

        pangolin::OpenGlMatrix Twc;
        Twc.SetIdentity();

        cv::namedWindow("ORB-SLAM2: Current Frame");

        bool bFollow = true;
        bool bLocalizationMode = false;

        while (true)
        {
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            mpMapDrawer->GetCurrentOpenGLCameraMatrix(Twc);

            if(menuFollowCamera && bFollow)
            {
                s_cam.Follow(Twc);
            }
            else if(menuFollowCamera && !bFollow)
            {
                s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(mViewpointX,mViewpointY,mViewpointZ, 0,0,0,0.0,-1.0, 0.0));
                s_cam.Follow(Twc);
                bFollow = true;
            }
            else if(!menuFollowCamera && bFollow)
            {
                bFollow = false;
            }

            if(menuLocalizationMode && !bLocalizationMode)
            {
                mpSystem->ActivateLocalizationMode();
                bLocalizationMode = true;
            }
            else if(!menuLocalizationMode && bLocalizationMode)
            {
                mpSystem->DeactivateLocalizationMode();
                bLocalizationMode = false;
            }

            d_cam.Activate(s_cam);
            glClearColor(1.0f,1.0f,1.0f,1.0f);
            mpMapDrawer->DrawCurrentCamera(Twc);
            if(menuShowKeyFrames || menuShowGraph)
                mpMapDrawer->DrawKeyFrames(menuShowKeyFrames,menuShowGraph);
            if(menuShowPoints)
                mpMapDrawer->DrawMapPoints();

            pangolin::FinishFrame();

            cv::Mat im = mpFrameDrawer->DrawFrame();
            cv::imshow("ORB-SLAM2: Current Frame",im);
            cv::waitKey(mT);

            if(menuReset)
            {
                menuShowGraph = true;
                menuShowKeyFrames = true;
                menuShowPoints = true;
                menuLocalizationMode = false;
                if(bLocalizationMode)
                    mpSystem->DeactivateLocalizationMode();
                bLocalizationMode = false;
                bFollow = true;
                menuFollowCamera = true;
                mpSystem->Reset();
                menuReset = false;
            }

            if(Stop())
            {
                while(isStopped())
                {
                    usleep(3000);
                }
            }

            if(CheckFinish())
                break;
        }

        SetFinish();
    }

    void Viewer::RequestFinish()
    {
        unique_lock<mutex> lock(mMutexFinish);
        mbFinishRequested = true;
    }

    bool Viewer::CheckFinish()
    {
        unique_lock<mutex> lock(mMutexFinish);
        return mbFinishRequested;
    }

    void Viewer::SetFinish()
    {
        unique_lock<mutex> lock(mMutexFinish);
        mbFinished = true;
    }

    bool Viewer::isFinished()
    {
        unique_lock<mutex> lock(mMutexFinish);
        return mbFinished;
    }

    void Viewer::RequestStop()
    {
        unique_lock<mutex> lock(mMutexStop);
        if(!mbStopped)
            mbStopRequested = true;
    }

    bool Viewer::isStopped()
    {
        unique_lock<mutex> lock(mMutexStop);
        return mbStopped;
    }

    bool Viewer::Stop()
    {
        unique_lock<mutex> lock(mMutexStop);
        unique_lock<mutex> lock2(mMutexFinish);

        if(mbFinishRequested)
            return false;
        else if(mbStopRequested)
        {
            mbStopped = true;
            mbStopRequested = false;
            return true;
        }

        return false;
    }

    void Viewer::Release()
    {
        unique_lock<mutex> lock(mMutexStop);
        mbStopped = false;
    }
}
