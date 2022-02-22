#ifndef MYIMAGEHANDLER_H
#define MYIMAGEHANDLER_H
#include "camerahandler.h"
#include "motiondata.h"
#include <string>
#include <list>
#include <mutex>

class MyCameraHandler: public CameraHandler
{
public:
    MyCameraHandler(std::string name);
    void handleRawFrame(const RawImageFrame *rawFrame);
    int getMotionListSize()
    {
        return mMotionList.size();
    }
    void readMotionData(MotionData& data)
    {
        std::lock_guard<std::mutex> lock(mMutex);
        data = mMotionList.front();
        mMotionList.pop_front();
    }
    void addMotionData(MotionData& data)
    {
        std::lock_guard<std::mutex> lock(mMutex);
        mMotionList.push_back(data);
    }
protected:
    void processFrame(int frameId, const char *extended, size_t size);

private:
    std::string mName;
    std::list<MotionData> mMotionList;
    std::mutex mMutex;
};

#endif // MYIMAGEHANDLER_H
