#include "video_reader.h"
#include "video_reader_p.h"

#include <fstream>
#include <algorithm>

using namespace cv;

namespace cove {

// *********************************************************************
// *                         VideoReader object                        *
// *********************************************************************
VideoReader::VideoReader () : impl(new VideoReaderPrivate()) {

}

VideoReader::VideoReader (const string& filename) : VideoReader() {
    open(filename);
}

VideoReader::~VideoReader () {
    release();
}

/*
double VideoReader::getDuration () const
{
    return impl->videoDuration - impl->seekTable.front().time;
}
*/

// *********************************************************************
// *                          Video open/close                         *
// *********************************************************************
bool VideoReader::open (const string &filename)
{
    if (impl->openVideo(filename)) {
        if (!impl->loadSeekTable(impl->getSeekTableFile())) {
            cout << "Generating seek index" << endl;
            impl->buildSeekTable();
            impl->saveSeekTable(impl->getSeekTableFile());
        }
        return true;
    } else {
        return false;
    }
}

void VideoReader::release ()
{
    impl->closeVideo();
}


double VideoReader::get(int propid) const {

    if (!isOpened()) return 0;

    switch (propid) {
    case CAP_PROP_POS_MSEC: return (impl->currentFrameTime - impl->seekTable.front().time) * 1000;
    case CAP_PROP_POS_FRAMES: return impl->currentFrameIndex - impl->seekTable.front().index;
    case CAP_PROP_FRAME_WIDTH: return impl->frame->width;
    case CAP_PROP_FRAME_HEIGHT: return impl->frame->height;
    case CAP_PROP_FPS: {
        double duration = (impl->videoDuration - impl->seekTable.front().time);
        double frames = impl->currentFrameIndex - impl->seekTable.front().index;
        return frames / duration;
    }
    case CAP_PROP_FOURCC: return 0;
    case CAP_PROP_FRAME_COUNT: {
        return impl->numberOfFrames - impl->seekTable.front().index;
    }
    case CAP_PROP_FORMAT: return CV_8UC3;
    //case CAP_PROP_MODE:
    //case CAP_PROP_BRIGHTNESS:
    //case CAP_PROP_CONTRAST:
    //case CAP_PROP_SATURATION:
    //case CAP_PROP_HUE:
    //case CAP_PROP_GAIN Gain of the image (only for cameras).
    //case CAP_PROP_EXPOSURE:
    //case CAP_PROP_CONVERT_RGB:
    //case CAP_PROP_WHITE_BALANCE:
    //case CAP_PROP_RECTIFICATION:
    default: return 0;
    }
}

bool VideoReader::set(int propid, double value) {

    if (!isOpened()) return false;

    switch (propid) {
    case CAP_PROP_POS_MSEC: {

        return true;
    }
    case CAP_PROP_POS_FRAMES: {
        impl->seekToFrame((int) value + impl->seekTable.front().index);
        return true;
    }
    default: return false;
    }
}

bool VideoReader::isOpened() const {
    return impl->isOpened();
}

bool VideoReader::grab() {

    if (!isOpened() || !impl->frameAvailable) return false;

    bool prev = impl->seekTableComplete;

    impl->stepForward();

    if (!prev && impl->seekTableComplete) {
        impl->saveSeekTable(impl->getSeekTableFile());
    }

    return true;
}

bool VideoReader::retrieve (OutputArray image, int flag) {

    if (!isOpened() || !impl->frameAvailable) {
        image.release();
        return false;
    }

    const uint8_t *buffer;
    int width, height, linesize;

    impl->retrieveImage (buffer, width, height, linesize);

    cv::Mat tmp(height, width, CV_8UC3, const_cast<uint8_t *>(buffer));

    cv::cvtColor(tmp, image, CV_BGR2RGB);

    return true;
}

void VideoReader::read(OutputArray image) {

    retrieve(image);
    grab();

}


// *********************************************************************
// *                      Private implementation                       *
// *********************************************************************
VideoReaderPrivate::VideoReaderPrivate () : fastSeekDistance(50),
    formatContext(NULL), codecContext(NULL), scaleContext(NULL), videoCodec(NULL),
    frame(NULL), frameRGB(NULL), bufferRGB(NULL), middleOfKeyFrameSeek(false)
{
    // Register video codecs
    av_register_all();

    // Clear/initialize state
    clearState();
}

VideoReaderPrivate::~VideoReaderPrivate ()
{
    closeVideo();
}


void VideoReaderPrivate::clearState ()
{
    // Close / clear FFMPEG variables
    avformat_close_input(&formatContext);

    av_free(bufferRGB);
    bufferRGB = NULL;

    av_frame_free(&frameRGB);
    av_frame_free(&frame);

    sws_freeContext(scaleContext);
    scaleContext = NULL;

    videoCodec = NULL;
    videoStreamIndex = -1;

    // Clear filename
    videoFileName.clear();

    // Clear state variables
    frameAvailable = true;
    seekTableComplete = false;

    firstDts = AV_NOPTS_VALUE;
    previousDts = AV_NOPTS_VALUE;
    currentDts = AV_NOPTS_VALUE;
    keyFramePacketDts = AV_NOPTS_VALUE;

    firstFramePts = AV_NOPTS_VALUE;

    currentFrameIndex = 0;
    currentFrameTime = 0;

    numberOfFrames = 0;

    // Clear seek table
    seekTable.clear();
}

string VideoReaderPrivate::getSeekTableFile() {

    size_t lastindex = videoFileName.find_last_of(".");
    string rawname = videoFileName.substr(0, lastindex);

    return rawname + string(".vdx");

}

// *********************************************************************
// *                             Video open                            *
// *********************************************************************
bool VideoReaderPrivate::openVideo (const string &filename)
{
    // Close any opened video
    closeVideo();

    // Open file
    formatContext = 0;
    if (avformat_open_input(&formatContext, filename.c_str(), NULL, NULL)) {
        return false;
    }

    // Find stream info
    if (avformat_find_stream_info(formatContext, NULL) < 0) {
        return false;
    }

    // Find video stream
    codecContext = 0;
    for (unsigned int i = 0; i < formatContext->nb_streams; i++) {
        codecContext = formatContext->streams[i]->codec;
        if (codecContext->codec_type == AVMEDIA_TYPE_VIDEO) {
            videoStreamIndex = i;
            break;
        } else {
            codecContext = 0;
        }
    }

    if (!codecContext) {
        cerr << "Could not find video stream" << endl;
        clearState();
        return false;
    }

    videoFileName = filename;

    // Find the decoder for the video stream
    videoCodec = avcodec_find_decoder(codecContext->codec_id);
    if (!videoCodec) {
        cerr << "Unsupported codec" << endl;
        clearState();
        return false;
    }

    // Open codec
    if (avcodec_open2(codecContext, videoCodec, NULL) < 0) {
        cerr << "Could not open codec!" << endl;
        clearState();
        return false;
    }

    // Allocate frame buffer
    frame = av_frame_alloc();
    frameRGB = av_frame_alloc();

    // Raw data buffer
    int numBytes = avpicture_get_size(AV_PIX_FMT_RGB24, codecContext->width, codecContext->height);
    bufferRGB = (uint8_t *)av_malloc(numBytes * sizeof(uint8_t));

    avpicture_fill((AVPicture *)frameRGB, bufferRGB, AV_PIX_FMT_RGB24, codecContext->width, codecContext->height);

    // Scaling context (for RGB24 conversion)
    scaleContext = sws_getContext(codecContext->width, codecContext->height, codecContext->pix_fmt,
                                  codecContext->width, codecContext->height, AV_PIX_FMT_RGB24, SWS_BILINEAR, 0, 0, 0);

    // Read first key frame
    while (!seekTable.size()) {
        stepForward();

        if (!frameAvailable) {
            cerr << "Could not find first key frame (no valid key frames in video stream?)" << endl;
            clearState();
            return false;
        }
    }

    return true;
}


bool VideoReaderPrivate::closeVideo ()
{
    if (!isOpened()) return false;
    // Clear the state
    clearState();
    return true;
}

bool VideoReaderPrivate::isOpened()
{
    return videoCodec != NULL;
}

// *********************************************************************
// *                     Forward-stepping function                     *
// *********************************************************************
bool VideoReaderPrivate::stepForward()
{

    // Prevent access beyond the end of video stream
    if (!frameAvailable) {
        return false;
    }

    currentFrameIndex++;

    AVPacket packet;
    int frameFinished = 0;
    bool flushing = false;

    while (!frameFinished) {
        // Read a packet
        if (av_read_frame(formatContext, &packet) < 0) {

            // Create empty packet to flush
            av_init_packet(&packet);
            packet.data = NULL;
            packet.size = 0;
            flushing = true;

        }

        if (packet.stream_index == videoStreamIndex) {
            // Store packet's DTS
            previousDts = currentDts;
            currentDts = packet.dts;

            // Store first DTS, if necessary
            if (firstDts == AV_NOPTS_VALUE) {
                firstDts = packet.dts;
            }

            // Check if packet belongs to a key frame; if it does, store
            // the previous packet's DTS as key frame packet's DTS. If
            // unavailable, use the current packet DTS instead
            if (packet.flags & AV_PKT_FLAG_KEY) {
                if (previousDts == AV_NOPTS_VALUE) {
                    keyFramePacketDts = packet.dts;
                } else {
                    keyFramePacketDts = previousDts;
                }
            }

            // Decode video frame
            avcodec_decode_video2(codecContext, frame, &frameFinished, &packet);

            if (frameFinished) {
                // Compute frame time (PTS)
                double pts = 0;

                if (packet.dts == AV_NOPTS_VALUE && frame->opaque && *(int64_t *)frame->opaque != AV_NOPTS_VALUE) {
                    pts = *(int64_t *)frame->opaque;
                } else if (packet.dts != AV_NOPTS_VALUE) {
                    pts = packet.dts;
                } else {
                    pts = 0;
                }

                // Store first frame PTS, which we will use to offset
                // all other PTS values (so that video stream starts at
                // time 0)
                if (currentFrameIndex == 0 && firstFramePts == AV_NOPTS_VALUE) {
                    firstFramePts = pts;
                }
                pts -= firstFramePts;
                pts *= av_q2d(formatContext->streams[videoStreamIndex]->time_base);

                // Store current frame time
                currentFrameTime = pts;

                // If frame is a key frame, add its entry to our seek table
                if (frame->key_frame) {
                    SeekTableEntry entry;

                    entry.index = currentFrameIndex;
                    entry.time = currentFrameTime;
                    entry.firstPacketDts = (currentFrameIndex == 0) ? firstDts : keyFramePacketDts;
                    entry.lastPacketDts = packet.dts;

                    appendSeekTableEntry(entry);
                }

                // Update number of frames and duration
                numberOfFrames = std::max(currentFrameIndex + 1, numberOfFrames);
                videoDuration = std::max(currentFrameTime, videoDuration);

                // If we reached a key-frame, reset our frame cache; but do
                // this only if we are not in between a key-frame seek
                // (because in the latter case, we sometime seem to get
                // spurious key-frames)
                if (!middleOfKeyFrameSeek && frame->key_frame) {
                    resetFrameCache();
                }

                break;
            } else {

                if (flushing) {
                    // No more packets, mark the end of stream
                    frameAvailable = false;

                    if (!seekTableComplete) {
                        seekTableComplete = true; // Since we are at the end of the stream, our seek table is complete
                    }

                    return false;
                }


            }
        }

        av_free_packet(&packet);
    }

    av_free_packet(&packet);

    return true;
}


// *********************************************************************
// *                         Frame-based seek                          *
// *********************************************************************
void VideoReaderPrivate::seekToFrame (uint32_t targetFrameIndex) throw (string) {
    // Make sure we are not already there
    if (targetFrameIndex == currentFrameIndex) {
        return;
    }

    // Decide whether we need to seek to key frame that is nearest to target
    bool seekRequired = true;

    if (targetFrameIndex >= seekTable[currentKeyFrameEntry].index && (currentKeyFrameEntry == seekTable.size() - 1 || targetFrameIndex < seekTable[currentKeyFrameEntry + 1].index)) {
        // We are still within the current GOP
        // Try to get frame from cache (beneficial if we are going backwards)
        if (false) {
            return;
        }

        // If we are going forward, we do not need to seek to the keyframe,
        // because being in GOP implies we have already decoded it
        if (targetFrameIndex >= currentFrameIndex) {
            seekRequired = false;
        }
    } else if (targetFrameIndex >= currentFrameIndex && targetFrameIndex <= currentFrameIndex + fastSeekDistance) {
        // We are not inside the current GOP, but we are seeking forward;
        // it turns out that for certain distances, it is still cheaper
        // to simply step forward than seek to the next key-frame
        seekRequired = false;
    }

    // Seek to nearest target key frame, if necessary
    if (seekRequired) {
        seekToNearestKeyFrame(targetFrameIndex);

        if (currentFrameIndex > targetFrameIndex) {
            throw string("Error advancing to key frame before seek (index is wrong!)");
        }
    }

    // Step-forward to the correct frame
    while (currentFrameIndex < targetFrameIndex) {
        if (frameAvailable) {
            stepForward();
        } else {
            throw string("Error advancing to the requested frame - probably out of range!");
        }
    }
}

void VideoReaderPrivate::seekToNearestKeyFrame (uint32_t targetFrameIndex) throw (string)
{
    return seekToNearestKeyFrame(targetFrameIndex, 0);
}

void VideoReaderPrivate::seekToNearestKeyFrame (uint32_t targetFrameIndex, uint32_t  offset) throw (string)
{
    const SeekTableEntry entry = getNearestSeekTableEntry(targetFrameIndex, offset);

    // Make sure we are not already there
    if (entry.index == currentFrameIndex) {
        return;
    }

    // Set up flag indicating that we are in the middle of key-frame seek
    middleOfKeyFrameSeek = true;

    // Set bad current frame index, in case something goes wrong
    currentFrameIndex = -2;
    frameAvailable = true;

    // Are we seeking forward or backward?
    int seekFlags = 0;
    if (entry.firstPacketDts <= currentDts) {
        seekFlags = AVSEEK_FLAG_BACKWARD;
    }

    // Seek
    if (av_seek_frame(formatContext, videoStreamIndex, entry.firstPacketDts, seekFlags) < 0) {
        throw string("Seek failed!");
    }

    // Flush buffers
    avcodec_flush_buffers (codecContext);

    // Now, step forward
    stepForward();

    if (!frameAvailable) {
        return seekToNearestKeyFrame(targetFrameIndex, offset + 1);
    }

    while (currentDts < entry.lastPacketDts) {
        stepForward();
    }

    if (currentDts != entry.lastPacketDts) {
        //qWarning() << "Missed key-frame, trying previous one!";
        return seekToNearestKeyFrame(targetFrameIndex, offset + 1);
    }

    // Some additional complications due to the fact that some videos
    // have bad key-frames that do not get decoded properly, in which
    // case we need to go to the previous key-frame. Sometimes, none of
    // the frames are labelled as key-frames, and in that case, we need
    // to allow seeking to frame 0 even if it is not labelled as a key-
    // frame.
    if (!frame->key_frame && entry.index != 0) {
        //qWarning() << "Found key-frame not labelled as a key-frame, so trying the previous one!";
        return seekToNearestKeyFrame(entry.index - 1, 0);
    }

    currentFrameIndex = entry.index;

    currentKeyFrameEntry = reverseKeyFrameMap[currentFrameIndex];
    // Mark the end of key-frame seek...
    middleOfKeyFrameSeek = false;

    // ... and reset our frame cache
    resetFrameCache();
}


void VideoReaderPrivate::resetFrameCache ()
{
    currentKeyFrameEntry = reverseKeyFrameMap[currentFrameIndex];
    //qDebug() << "Frame cache reset: new entry:" << currentKeyFrameEntry << "frame" << currentFrameIndex;
}


// *********************************************************************
// *                          Image retrieval                          *
// *********************************************************************
void VideoReaderPrivate::retrieveImage (const uint8_t *&buffer, int &width, int &height, int &linesize) const
{
    // Convert the image from its native format to RGB
    sws_scale(scaleContext, (uint8_t const * const *)frame->data, frame->linesize, 0, codecContext->height, frameRGB->data, frameRGB->linesize);

    buffer = bufferRGB;
    width = frame->width;
    height = frame->height;
    linesize = frameRGB->linesize[0];
}


// *********************************************************************
// *                            Seek table                             *
// *********************************************************************
bool VideoReaderPrivate::buildSeekTable () {

    // Nothing to do if seek table is already complete
    if (seekTableComplete) {
        return true;
    }

    // Seek to the last known key frame
    seekToNearestKeyFrame(numberOfFrames - 1);

    // Go to the end of the stream
    while (frameAvailable) {
        stepForward();
    }

    return true;

}

void VideoReaderPrivate::appendSeekTableEntry (const VideoReaderPrivate::SeekTableEntry &entry)
{

    // If seek table is not empty, make sure we are not trying to add
    // an already-existing entry
    if (!seekTable.empty()) {
        if (seekTable.back().index >= entry.index) {
            return;
        }
    }

    seekTable.push_back(entry);
    //emit q->seekTableUpdated(entry.index, entry.time);

    reverseKeyFrameMap[entry.index] = seekTable.size() - 1;
}


// Using offset > 0 returns a modified seek table entry that sets the
// seek position to be offset key-frames in the past
VideoReaderPrivate::SeekTableEntry VideoReaderPrivate::getNearestSeekTableEntry (uint32_t targetFrameIndex, uint32_t offset) throw (string)
{
    if (seekTable.empty()) {
        throw string("Seek table is empty!");
    }

    if (targetFrameIndex < seekTable.front().index) {
        throw string("Tried to seek to frame before first seek table entry!");
    }

    uint32_t i;
    for (i = 0; i < seekTable.size(); i++) {
        if (seekTable[i].index > targetFrameIndex) {
            break;
        }
    }

    i = i - 1;

    if (i < offset) {
        throw string("Target index with offset is before the first seek table entry!");
    }

    SeekTableEntry entry = seekTable[i];
    entry.firstPacketDts = seekTable[i - offset].firstPacketDts; // Modify first packet DTS by offset

    return entry;
}

template < class T >
std::ostream& operator << (std::ostream& stream, const std::vector<T>& v) {
    stream << v.size() << " ";
    for (typename std::vector<T>::const_iterator ii = v.begin(); ii != v.end(); ++ii)
    {
        stream << *ii << " ";
    }
    return stream;
}

template < class T >
std::istream& operator >> (std::istream& stream, std::vector<T>& v) {
    size_t numel;
    stream >> numel;

    v.resize(numel);
    for (size_t i = 0; i < numel; i++)
    {
        stream >> v[i];
    }
    return stream;
}

template<typename T1, typename T2>
std::ostream &operator << (std::ostream &stream, const std::map<T1, T2>& map) {
    stream << map.size() << " ";
    for (typename std::map<T1, T2>::const_iterator it = map.begin();
            it != map.end();
            ++it)
    {
        stream << (*it).first << " " << (*it).second << " ";
    }
    return stream;
}

template<typename T1, typename T2>
std::istream &operator >> (std::istream &stream, std::map<T1, T2>& map) {
    size_t numel;
    stream >> numel;

    for (size_t i = 0; i < numel; i++)
    {
        T1 k;
        T2 v;
        stream >> k;
        stream >> v;
        map[k] = v;
    }
    return stream;
}

static const char seek_table_header[4] = { '#', 'v', 'd', 'x' };

std::istream &operator >> (std::istream &stream, VideoReaderPrivate::SeekTableEntry  &entry)
{
    stream >> entry.index;
    stream >> entry.time;
    stream >> entry.firstPacketDts;
    stream >> entry.lastPacketDts;

    return stream;
}

std::ostream &operator << (std::ostream &stream, const VideoReaderPrivate::SeekTableEntry &entry)
{
    stream << entry.index << " ";
    stream << entry.time << " ";
    stream << entry.firstPacketDts << " ";
    stream << entry.lastPacketDts << " ";

    return stream;
}

bool VideoReaderPrivate::loadSeekTable (const string &filename)
{
    std::ifstream stream;

    stream.open(filename.c_str(), std::ifstream::in);

    if (!stream.is_open()) {
        return false;
    }

    // Read header
    char header[4];

    stream.read(header, sizeof(header));
    if (memcmp(header, seek_table_header, 4)) {
        stream.close();
        return false;
    }

    // Read data
    stream >> numberOfFrames;
    stream >> videoDuration;
    stream >> seekTable;
    stream >> reverseKeyFrameMap;

    // Seek table successfuly loaded
    seekTableComplete = true;

    // Reset position
    seekToFrame(seekTable.front().index);

    stream.close();

    return true;
}

bool VideoReaderPrivate::saveSeekTable (const string &filename) {
    if (!seekTableComplete) {
        return false;
    }

    std::ofstream stream;

    stream.open(filename.c_str(), std::ofstream::out);

    if (!stream.is_open()) return false;

    // Header
    stream.write(seek_table_header, sizeof(seek_table_header));

    // Data
    stream << " ";
    stream << numberOfFrames << " ";
    stream << videoDuration << " ";
    stream << seekTable << " ";
    stream << reverseKeyFrameMap << " ";

    stream.close();

    return true;
}

}