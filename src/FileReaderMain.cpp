#include <iostream>
#include <vector>
#include <string>
#include <lzfse.h>
#include "RVL.h"
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

void write_buffer_to_disk(std::string file_path, char* data, int numBytes)
{
    std::ofstream ostream(file_path, std::ios::out | std::ios_base::binary);
    ostream.write(reinterpret_cast<const char *>(data),numBytes);
}

cv::Mat readDepth(std::string path, int w, int h){
    std::vector<uint8_t>  rawMessageBuffer = read_vector_from_disk(path);

    std::vector<uint8_t> lzfseScratchBuffer_; /** Preallocated LZFSE scratch buffer. */
    lzfseScratchBuffer_.resize(lzfse_decode_scratch_size());
    //cout<<"lzfseScratchBuffer:\t"<<lzfseScratchBuffer_.size()<<endl;

    cv::Mat imgDepth_ = cv::Mat(h,w,CV_32FC1);
    size_t sizeInBytes = imgDepth_.total() * imgDepth_.elemSize();
    //cout<<"OpenCV image size:\t"<<sizeInBytes<<endl;

    size_t outSize = lzfse_decode_buffer(imgDepth_.data, sizeInBytes, rawMessageBuffer.data(), rawMessageBuffer.size(),lzfseScratchBuffer_.data());
    //cout<<"decompressed size:\t"<<outSize<<endl;

    double min, max;
    cv::minMaxLoc(imgDepth_, &min, &max);
    cout<<"lzfse\tsize:\t"<<rawMessageBuffer.size() / 1000.0<<" kB \t range ["<<min<<","<<max<<"] m"<<endl;

    return imgDepth_;
}

cv::Mat readDepthRVL(std::string path, int w, int h){
    std::vector<uint8_t>  rawMessageBuffer = read_vector_from_disk(path);

    int nPixels = w*h;

    cv::Mat imgDepth_ = cv::Mat(h,w,CV_16FC1);
    RvlCodec codec2; codec2.DecompressRVL((const unsigned char*) rawMessageBuffer.data(), (unsigned short*) imgDepth_.data, nPixels);
    //DecompressRVL((char*) buf, (short*) imgDepth_.data, nPixels); 

    //CV_16FC1 support is very limited in opencv, no minMaxLoc https://github.com/opencv/opencv/issues/14624
    imgDepth_.convertTo(imgDepth_,CV_32FC1);

    double min, max;
    cv::minMaxLoc(imgDepth_, &min, &max);
    cout<<"RVL\tsize:\t"<<rawMessageBuffer.size() / 1000.0 <<" kB \t range ["<<min<<","<<max<<"] m"<<endl;
  
    return imgDepth_;
}

cv::Mat readRGB(std::string path){
    return cv::imread(path);
}

void display(cv::Mat imgRGB_, cv::Mat imgDepth_, cv::Mat imgDepth16_, cv::Mat diff, int fps, bool flip = false){
    if ( flip )
    {
        cv::flip( imgRGB_, imgRGB_, 1 );
        cv::flip( imgDepth_, imgDepth_, 1 );
    }

    // Show images
    cv::imshow( "RGB", imgRGB_ );
    cv::imshow( "Depth_lzfse_32bit", imgDepth_ );
    cv::imshow( "Depth_RVL_16bit", imgDepth16_ );
    cv::imshow( "diff", diff );
    cv::waitKey(0);//1000.0/fps);
}

void compressAndSave(std::string filename, cv::Mat depth){  //16 bit depth
    RvlCodec codec;
    int nPixels = depth.total();
    short buf[nPixels]; //usually too large
    int nBytes2 = codec.CompressRVL((const unsigned short*) depth.data, (unsigned char*) buf,nPixels);
    //int nBytes2 = CompressRVL((short*) $depthFrame.data(), (char*) buf, nPixels);
    write_buffer_to_disk(filename, (char*) buf,nBytes2);
    //cout << "nPixels " << nPixels << "   nBytes2 " << nBytes2 << endl;
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
        cout<<"frame: "<<i<<endl;
        cv::Mat rgb = readRGB(cv::utils::fs::join(rgbd,std::to_string(i)+".jpg"));
        
        cv::Mat depth32 = readDepth(cv::utils::fs::join(rgbd,std::to_string(i)+".depth"),m.w,m.h);

        auto rvlFile = cv::utils::fs::join(rgbd,std::to_string(i)+".rvl");
        {
            cv::Mat depth16;
            depth32.convertTo(depth16,CV_16FC1);
            compressAndSave(rvlFile,depth16);
        }
        
        cv::Mat depth16 = readDepthRVL(rvlFile,m.w,m.h);

        cv::Mat diff;// = depth32-depth16;
        cv::absdiff(depth32,depth16,diff);

        cv::Mat nanMask = diff == diff;
        cv::Scalar meanDiff = cv::mean( diff, nanMask);
        float meanD = meanDiff.val[0];

        //cout<<diff<<endl;
        double min,max;
        cv::minMaxLoc(diff, &min, &max);
        cout<<"diff\tmean:\t"<<meanD*1000<<" mm\t max: "<<max*1000<< " mm"<<endl;
        double range = max-min;
        diff.convertTo(diff,CV_32FC1,1.0/range,-min);

        display(rgb, depth32, depth16, diff, m.fps);
    }

}