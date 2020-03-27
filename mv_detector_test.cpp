#include<ctime>
#include<iomanip>
#include<iostream>
#include<vector>
#include<chrono>
#include<functional>

// FFmpeg
extern "C" {
  #include<libavformat/avformat.h>
  #include<libavcodec/avcodec.h>
  #include<libavutil/avutil.h>
  #include<libavutil/pixdesc.h>
  #include<libswscale/swscale.h>
  #include<libavutil/motion_vector.h>
}

// OpenCV
#include<opencv2/core.hpp>
#include<opencv2/videoio.hpp>
#include<opencv2/imgproc.hpp>
#include<opencv2/imgcodecs.hpp>
#include<opencv2/highgui.hpp>
#include<opencv2/photo.hpp>

#include <boost/circular_buffer.hpp>
#include <boost/format.hpp>

#include "mv_detector.h"

// ADJUST THIS PARAMETER IN APP SIDE to change sensitivity
#define AVG_MOVEMENT_THRESHOLD 0.5

//TODO: warning: 'AVStream::codec' is deprecated

using namespace std;
static char run_mode = 'r';

int main(int argc, char* argv[])
{
//    if (argc < 2) {
  //      std::cout << "Usage: mv_detector_test <infile>" << std::endl;
    //    return 1;
   // }
    const char* infile = argv[1];
//    const char* infile = "falldow0.mp4";
    char pict_type;

    //av_log_set_level(AV_LOG_DEBUG);
    av_register_all();

    int ret, ret2;

    // open input file context
    AVFormatContext* inctx = nullptr;
    ret = avformat_open_input(&inctx, infile, nullptr, nullptr);
    if (ret < 0) {
        std::cerr << "fail to avforamt_open_input(\"" << infile << "\"): ret=" << ret;
        return 2;
    }

    // retrive input stream information
    ret = avformat_find_stream_info(inctx, nullptr);
    if (ret < 0) {
        std::cerr << "fail to avformat_find_stream_info: ret=" << ret;
        return 2;
    }

    // find primary video stream
    AVCodec* vcodec = nullptr;
    ret = av_find_best_stream(inctx, AVMEDIA_TYPE_VIDEO, -1, -1, &vcodec, 0);
    if (ret < 0) {
        std::cerr << "fail to av_find_best_stream: ret=" << ret;
        return 2;
    }

    const int vstrm_idx = ret;
    AVStream* vstrm = inctx->streams[vstrm_idx];
    int frame_width = vstrm->codecpar->width;
    int frame_height = vstrm->codecpar->height;

    //MotionVectorDetector mvd(vstrm->codec->width, vstrm->codec->height);

    AVDictionary *opts = nullptr;
    av_dict_set(&opts, "flags2", "+export_mvs", 0);

    // open video decoder context
    ret = avcodec_open2(vstrm->codec, vcodec, &opts);
    if (ret < 0) {
        std::cerr << "fail to avcodec_open2: ret=" << ret;
        return 2;
    }

    AVDictionaryEntry *tag = nullptr;
    while ((tag = av_dict_get(opts, "", tag, AV_DICT_IGNORE_SUFFIX)))
        printf("%s=%s\n", tag->key, tag->value);
    av_dict_free(&opts);

    // print input video stream informataion
    std::cout
        << "input file: " << infile << "\n"
        << "format: " << inctx->iformat->name << "\n"
        << "vcodec: " << vcodec->name << "\n"
        << "size:   " << vstrm->codecpar->width << "   x      " << vstrm->codecpar->height << "\n"
        << "fps:    " << av_q2d(vstrm->codec->framerate) << " [fps]\n"
        << "length: " << av_rescale_q(vstrm->duration, vstrm->time_base, {1,1000}) / 1000. << " [sec]\n"
        << "pixfmt: " << av_get_pix_fmt_name(vstrm->codec->pix_fmt) << "\n"
        << "frame:  " << vstrm->nb_frames << "\n"
        << std::flush;

    // initialize sample scaler
    // don't need to run this if input and process img size is same
    const int dst_width = vstrm->codecpar->width;
    const int dst_height = vstrm->codecpar->height;
    const AVPixelFormat dst_pix_fmt = AV_PIX_FMT_BGR24;
    SwsContext* swsctx = sws_getCachedContext(nullptr,
            vstrm->codec->width, vstrm->codec->height,
            vstrm->codec->pix_fmt,
            dst_width, dst_height, dst_pix_fmt, SWS_BICUBIC, nullptr, nullptr, nullptr);
    //sws_freeContext(swsctx);
    if (!swsctx) {
        std::cerr << "fail to sws_getCachedContext";
        return 2;
    }
    std::cout << "output: " << dst_width << 'x' << dst_height << ',' << av_get_pix_fmt_name(dst_pix_fmt) << std::endl;


    // allocate frame buffer for output
    AVFrame* frame = av_frame_alloc();
    std::vector<uint8_t> framebuf(avpicture_get_size(dst_pix_fmt, dst_width, dst_height));
    avpicture_fill(reinterpret_cast<AVPicture*>(frame), framebuf.data(), dst_pix_fmt, dst_width, dst_height);

    // decoding loop
    AVFrame* decframe = av_frame_alloc();

    int frameIndex = 1;
    bool end_of_stream = false;
    int got_pic = 0;
    AVPacket pkt;
    AVFrameSideData* sd = nullptr;
    ::std::chrono::steady_clock::time_point startTime = std::chrono::steady_clock::now();

    MVDetector mv_detector = MVDetector(std::make_pair(dst_width, dst_height));

    do {
        do {
            if (!end_of_stream) {
                // read packet from input file
                ret = av_read_frame(inctx, &pkt);

                if (ret < 0 && ret != AVERROR_EOF) {
                    std::cerr << "fail to av_read_frame: ret=" << ret << std::endl;
                    end_of_stream = true;
                    break;
                }

                if (ret == 0 && pkt.stream_index != vstrm_idx) {
                    // non video stream
                    //std::cerr << "fail to av_read_frame: ret=" << ret << std::endl;
                    break;
                }
                end_of_stream = (ret == AVERROR_EOF);
            }

            if (end_of_stream) {
                // null packet for bumping process
                av_init_packet(&pkt);
                pkt.data = nullptr;
                pkt.size = 0;
            }

            // decode video frame
            Measure m("decode");
            avcodec_decode_video2(vstrm->codec, decframe, &got_pic, &pkt);
            double decode_duration = m.elapsed();

            if (!got_pic)
                break;

            // get picture type
            char pict_type = av_get_picture_type_char(decframe->pict_type);

            // get pts
            int64_t pts = decframe->pts != AV_NOPTS_VALUE ? decframe->pts : (decframe->pkt_dts != AV_NOPTS_VALUE ? decframe->pkt_dts : pts + 1);

            std::cout << boost::format("ix(%1%) pict(%2%) pts(%3%) %4% ms -- ")%frameIndex%pict_type%pts%(decode_duration*100) << std::endl;

            // convert frame to OpenCV matrix
            sws_scale(swsctx, decframe->data, decframe->linesize, 0, decframe->height, frame->data, frame->linesize);

            {
	//	printf("diennv0 %d %d %d %d\n", dst_height, dst_width, dst_height*dst_width*3, frame->linesize[0]);
                //cv::Mat img(dst_height, dst_width, CV_8UC3, framebuf.data(), frame->linesize[0]);
                cv::Mat img(dst_height, dst_width, CV_8UC3, framebuf.data(), frame->linesize[0]);
                cv::Mat img2 = img.clone();
                cv::Mat img3 = img.clone();

                AVFrameSideData* sd = av_frame_get_side_data(decframe, AV_FRAME_DATA_MOTION_VECTORS);

                if(sd != nullptr) { // sd == nullptr when I frame also
                    // reading motion vectors, see ff_print_debug_info2 in ffmpeg's libavcodec/mpegvideo.c for reference and a fresh doc/examples/extract_mvs.c
                    AVMotionVector* mvs = (AVMotionVector*)sd->data;
                    int mvcount = sd->size / sizeof(AVMotionVector);
                    //Measure m2("proc");
                    bool movement = mv_detector.process_frame(pts, frameIndex, pict_type, std::vector<AVMotionVector>(mvs, mvs+mvcount));
                    //std::cout << "proc: " << m2.elapsed() << std::endl;

                 mv_detector.draw_occupancy(img2);
                 mv_detector.draw_motion_vectors(img3);
                    //std::cout << "avg_movment=" << avg_movement << std::endl;

                    if(movement) {
                        cv::putText(img2, "Movement", cv::Point(10,200), cv::FONT_HERSHEY_SIMPLEX, 2, CV_RGB(0,0,255),2, cv::LINE_AA);
                    }
                    cv::imshow("motion vectors", img3);
                    cv::imshow("occupancy", img2);
#if 1
					switch(cv::waitKey(run_mode=='r'?10:0)) {
						case 0x1b:
							break;
						case 'p':
							run_mode = cv::waitKey(0) != 'r'?'p':'r';
							break;
						default:
							run_mode = run_mode != 'r'?'p':'r';
					}
#endif
                }
                else {
                    mv_detector.process_frame(pts, frameIndex, pict_type, std::vector<AVMotionVector>());
                }

                ++frameIndex;
            }

        } while(0);
        av_packet_unref(&pkt);

    } while (!end_of_stream || got_pic);

    ::std::chrono::steady_clock::duration elapsedTime = ::std::chrono::steady_clock::now() - startTime;
    double duration = ::std::chrono::duration_cast< ::std::chrono::duration< double > >(elapsedTime).count();
    std::cout << std::fixed << std::setprecision(8) << duration * 1000 << " (ms), ";
    std::cout << frameIndex << " frames decoded, " << std::endl;
    std::cout << (double)frameIndex/duration << "fps" << std::endl;

    //TODO: call process_vectors one more with frame_index = -1

    av_frame_free(&decframe);
    av_frame_free(&frame);

    avcodec_close(vstrm->codec);
    avformat_close_input(&inctx);
    return 0;
}
