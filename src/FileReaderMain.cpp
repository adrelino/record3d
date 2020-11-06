#include <iostream>
#include <vector>
#include <record3d/Record3DStream.h>
#include <lzfse.h>
#include <mutex>

#ifdef HAS_OPENCV
#include <opencv2/opencv.hpp>
#endif

using namespace std;
using namespace Record3D;


//https://gist.github.com/looopTools/64edd6f0be3067971e0595e1e4328cbc
inline auto read_vector_from_disk(std::string file_path)
{
    std::ifstream instream(file_path, std::ios::in | std::ios_base::binary);
    std::vector<uint8_t> data((std::istreambuf_iterator<char>(instream)), std::istreambuf_iterator<char>());
    return data;
}

int main()
{
    std::vector<uint8_t>  rawMessageBuffer = read_vector_from_disk("../data/rgbd/1.depth");
    cout<<"compressed size:\t"<<rawMessageBuffer.size()<<endl;

    std::vector<uint8_t> depthImageBuffer_{};
    static constexpr size_t depthBufferSize_{ Record3DStream::MAXIMUM_FRAME_WIDTH * Record3DStream::MAXIMUM_FRAME_HEIGHT * sizeof( float )};
    depthImageBuffer_.resize( depthBufferSize_ );

    cout<<"depth buffer size:\t"<<depthImageBuffer_.size()<<endl;
    size_t currSize = rawMessageBuffer.size();

    std::vector<uint8_t> lzfseScratchBuffer_; /** Preallocated LZFSE scratch buffer. */
    lzfseScratchBuffer_.resize(lzfse_decode_scratch_size());
    cout<<"lzfseScratchBuffer:\t"<<lzfseScratchBuffer_.size()<<endl;

    size_t outSize = lzfse_decode_buffer(depthImageBuffer_.data(), depthImageBuffer_.size(), rawMessageBuffer.data(), rawMessageBuffer.size(),lzfseScratchBuffer_.data());

    cout<<"decompressed size:\t"<<outSize<<endl;

    cv::Mat imgDepth_ = cv::Mat::zeros(Record3DStream::MAXIMUM_FRAME_HEIGHT, Record3DStream::MAXIMUM_FRAME_WIDTH, CV_32F );
    size_t sizeInBytes = imgDepth_.total() * imgDepth_.elemSize();
    
    cout<<"OpenCV image size:\t"<<sizeInBytes<<endl;

    memcpy( imgDepth_.data, depthImageBuffer_.data(), depthBufferSize_);

    // The TrueDepth camera is a selfie camera; we mirror the RGBD frame so it looks plausible.
    bool areTrueDepthDataBeingStreamed = imgDepth_.rows == Record3D::Record3DStream::MAXIMUM_FRAME_HEIGHT && imgDepth_.cols == Record3D::Record3DStream::MAXIMUM_FRAME_WIDTH;
    if ( areTrueDepthDataBeingStreamed )
    {
        //cv::flip( imgRGB_, imgRGB_, 1 );
        cv::flip( imgDepth_, imgDepth_, 1 );
    }

    // Show images
    //cv::imshow( "RGB", imgRGB_ );
    cv::imshow( "Depth", imgDepth_ );
    cv::waitKey();

}