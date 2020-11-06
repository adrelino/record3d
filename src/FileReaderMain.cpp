#include <iostream>
#include <vector>
#include <string>
#include <lzfse.h>
#include <mutex>

#ifdef HAS_OPENCV
#include <opencv2/opencv.hpp>
#include <opencv2/core/utils/filesystem.hpp>
#endif

using namespace std;

typedef struct Metadata{
    cv::Mat K;
    int fps,w,h;
} Metadata;

Metadata readMetadata(std::string path){
    std::string json = path+".json";
    {//save with .json ending so FileStorage can determine type
        std::ifstream myfile(path);
        std::string line;
        std::getline(myfile,line);
        cout<<line<<endl;
        std::ofstream ofile(json);
        ofile << line;
    }
    
    cv::FileStorage fs(json, cv::FileStorage::READ);
    
    cv::FileNode mat = fs["K"];
    cv::Mat K = cv::Mat(3,3,CV_64FC1);
    for (int i=0; i<mat.size(); i++) {
        auto n = mat[i];
        double val = n.real();
        K.at<double>(i) = val;//rowwise
    };

    int fps = fs["fps"].real();
    int w = fs["w"].real();
    int h = fs["h"].real();

    return Metadata{K,fps,w,h};
}

//https://gist.github.com/looopTools/64edd6f0be3067971e0595e1e4328cbc
inline auto read_vector_from_disk(std::string file_path)
{
    std::ifstream instream(file_path, std::ios::in | std::ios_base::binary);
    std::vector<uint8_t> data((std::istreambuf_iterator<char>(instream)), std::istreambuf_iterator<char>());
    return data;
}

cv::Mat readDepth(std::string path, int w, int h){
    std::vector<uint8_t>  rawMessageBuffer = read_vector_from_disk(path);
    //cout<<"compressed size:\t"<<rawMessageBuffer.size()<<endl;

    std::vector<uint8_t> lzfseScratchBuffer_; /** Preallocated LZFSE scratch buffer. */
    lzfseScratchBuffer_.resize(lzfse_decode_scratch_size());
    //cout<<"lzfseScratchBuffer:\t"<<lzfseScratchBuffer_.size()<<endl;

    cv::Mat imgDepth_ = cv::Mat(h,w,CV_32FC1);
    size_t sizeInBytes = imgDepth_.total() * imgDepth_.elemSize();
    //cout<<"OpenCV image size:\t"<<sizeInBytes<<endl;

    size_t outSize = lzfse_decode_buffer(imgDepth_.data, sizeInBytes, rawMessageBuffer.data(), rawMessageBuffer.size(),lzfseScratchBuffer_.data());
    //cout<<"decompressed size:\t"<<outSize<<endl;

    return imgDepth_;
}

cv::Mat readRGB(std::string path){
    return cv::imread(path);
}

void display(cv::Mat imgRGB_, cv::Mat imgDepth_, int fps, bool flip = false){
    if ( flip )
    {
        cv::flip( imgRGB_, imgRGB_, 1 );
        cv::flip( imgDepth_, imgDepth_, 1 );
    }

    // Show images
    cv::imshow( "RGB", imgRGB_ );
    cv::imshow( "Depth", imgDepth_ );
    cv::waitKey(1000.0/fps);
}

int main(int argc, char** argv)
{
    if ( argc != 2 ){
        cout<<"usage: "<< argv[0] <<" <.r3d folder>" << endl;
        return -1;
    }

    std::string folder = argv[1];

    Metadata m = readMetadata(cv::utils::fs::join(folder,"metadata"));
    std::cout<<m.K<<endl;
    std::cout<<m.fps<<" "<<m.w<< " "<<m.h<<endl;

    std::vector<cv::String> rgbs,ds;
    std::string rgbd = cv::utils::fs::join(folder,"rgbd");
    cv::glob(cv::utils::fs::join(rgbd,"*.jpg"),rgbs);
    cv::glob(cv::utils::fs::join(rgbd,"*.depth"),ds);
    int numFiles = rgbs.size();
    cout<<"found "<<numFiles<<" rgb"<<endl;
    cout<<"found "<<ds.size()<<" depth"<<endl;
    assert(numFiles == ds.size());

    for(int i=0; i<numFiles; i++){
        cv::Mat rgb = readRGB(cv::utils::fs::join(rgbd,std::to_string(i)+".jpg"));
        cv::Mat depth = readDepth(cv::utils::fs::join(rgbd,std::to_string(i)+".depth"),m.w,m.h);
        //cout<<i<<endl;
        display(rgb, depth, m.fps);
    }

}