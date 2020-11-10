#ifndef OPENCVHELPERS_H
#define OPENCVHELPERS_H

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <string>
#include <sstream>
#include <iostream>

namespace OpenCVHelpers
{

    using namespace std;
    using namespace cv;

    /**
     * Show a depth image useing JET colormap
     * 
     * img should be float32 depth image measured in meters
     * http://stackoverflow.com/questions/13840013/opencv-how-to-visualize-a-depth-image
     */
    static void showDepthImage(const string &wndTitle, const Mat &img)
    {
        Mat mask = (img > 0.001f) & (img < 100000000.0f);
        double min, max;
        minMaxIdx(img, &min, &max, 0, 0, mask);
        cout << "  showDepthImage scaled:  min: " << min << " max:" << max << endl;

        Mat depthMap;
        float scale = 255.0f / (max - min);
        img.convertTo(depthMap, CV_8UC1, scale, -min * scale);

        Mat heatMap;
        applyColorMap(depthMap, heatMap, cv::COLORMAP_JET);
        heatMap.setTo(255, 255 - mask);

        cv::imshow(wndTitle, heatMap);
    }

    /**
     * Map image type from int to human readable string
     */
    static std::string getImageType(int imgTypeInt)
    {
        int numImgTypes = 35; // 7 base types, with five channel options each (none or C1, ..., C4)

        int enum_ints[] = {CV_8U, CV_8UC1, CV_8UC2, CV_8UC3, CV_8UC4,
                           CV_8S, CV_8SC1, CV_8SC2, CV_8SC3, CV_8SC4,
                           CV_16U, CV_16UC1, CV_16UC2, CV_16UC3, CV_16UC4,
                           CV_16S, CV_16SC1, CV_16SC2, CV_16SC3, CV_16SC4,
                           CV_32S, CV_32SC1, CV_32SC2, CV_32SC3, CV_32SC4,
                           CV_32F, CV_32FC1, CV_32FC2, CV_32FC3, CV_32FC4,
                           CV_64F, CV_64FC1, CV_64FC2, CV_64FC3, CV_64FC4};

        string enum_strings[] = {"CV_8U", "CV_8UC1", "CV_8UC2", "CV_8UC3", "CV_8UC4",
                                 "CV_8S", "CV_8SC1", "CV_8SC2", "CV_8SC3", "CV_8SC4",
                                 "CV_16U", "CV_16UC1", "CV_16UC2", "CV_16UC3", "CV_16UC4",
                                 "CV_16S", "CV_16SC1", "CV_16SC2", "CV_16SC3", "CV_16SC4",
                                 "CV_32S", "CV_32SC1", "CV_32SC2", "CV_32SC3", "CV_32SC4",
                                 "CV_32F", "CV_32FC1", "CV_32FC2", "CV_32FC3", "CV_32FC4",
                                 "CV_64F", "CV_64FC1", "CV_64FC2", "CV_64FC3", "CV_64FC4"};

        for (int i = 0; i < numImgTypes; i++)
        {
            if (imgTypeInt == enum_ints[i])
                return enum_strings[i];
        }
        return "unknown image type";
    }

    /**
     * Read depth image as 16bit unsigned .png from kinect or 32 bit .exr from blender 
     */
    static cv::Mat readDepthPngExr(const std::string &filename, bool show)
    {
        cv::Mat depth = cv::imread(filename, cv::IMREAD_UNCHANGED);
        int type = depth.type();
        int nc = depth.channels();
        cout << "LoadingSaving::loadPointCloudFromDepthMap after cv::imread " << filename << " \t type: " << OpenCVHelpers::getImageType(type) << " channels:" << nc << endl;

        if (type == CV_16U && nc == 1)
        { //depth_0.png from kinect cam, 0 means no measure, depth in mm (470 means 0.47m);
            //mask = (depth > 0.001f);
            //depth.convertTo( depth, CV_32FC1, 0.001); // convert to meters
        }
        else if (type == CV_32FC3 && nc == 3)
        { //depth_000000.exr made from blender, inf means no measure, depth in mm
            cv::cvtColor(depth, depth, cv::COLOR_BGR2GRAY);
            //mask = (depth != std::numeric_limits<float>::infinity());
            //depth.convertTo( depth, CV_32FC1, 0.001); // convert to meters
        }
        else
        {
            cout << "unsupported depth image type" << endl;
        }
        if (show)
        {
            OpenCVHelpers::showDepthImage("depth", depth);
            cv::waitKey(2);
        }

        return depth;
    }

} // namespace OpenCVHelpers

#endif // OPENCVHELPERS_H
