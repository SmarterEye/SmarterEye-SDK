# C++ SDK Documentation


# Version update record
Online date | Version number | Update content
:-: | :-: | :-:
2019.02.23 | 2.8 | The first version of the document, the revision number is unified with the corresponding SdkVer
2019.03.01 | 2.9 | Obstacle structure changes
2019.03.22 | 2.11 | Added Demo, fixed bugs, and added 4 interface descriptions
2019.04.29 | 2.14 | Modify Demo, use CMake, fix bugs
2019.05.05 | 0.2.15 | Added connection status acquisition interface
2019.07.05 | 0.2.17 | Added the return value description of the upgrade interface
2019.09.07 | 0.2.18 | Added frame rate setting, access interface and description
2019.09.20 | 0.2.19 | Added header file frameext.h, adjusted GetCompoundDemo and GetLaneExtDemo
2019.10.18 | 0.2.20 | Network library replacement
2019.11.28 | 0.2.21 | 1. Add the interface and description for cross-thread execution function, and add DisplayFramesDemo and description; 2. Add the interface description for yuv to rgb
2019.12.26 | 0.2.22 | Added interface for obtaining ambient light brightness
2019.12.31 | 0.2.23 | Bug fixes
2020.02.10 | 0.2.25 | Added PointCloudDemo, GetMotionDataDemo and related interfaces, adjusted DisplayFramesDemo
2020.03.23 | 0.2.26 | Fix bugs: GetMotionDataDemo, PointCloudDemo, optimize lane line drawing
2022.02.15 | 0.2.33 | Added GetDeviceInfoDemo and description;

# Introduction

This document is mainly aimed at secondary developers who use the Orion series products of SMARTEREYE, and describes the relevant technical content of the image processing interface service of SMARTEREYE. If you have any questions about the content of the document, you can contact our technical support and technical management personnel, contact information:

## Interface capabilities
Provide image data, algorithm data, camera calibration data, etc.
Image data and algorithm data general structure `RawImageFrame`.

````C++
struct RawImageFrame
{
uint16_t frameId; //Data type ID
int64_t time; 			//timestamp
uint16_t index; 			//index (reserved field, default is 0)
uint16_t format; 		//frame format
uint16_t width; 			//image width
uint16_t height; 		//image height
uint32_t speed; 			// travel speed
uint32_t dataSize; 		//image data size
uint8_t image[0]; 		//First address of image data
};
````
The `frameId` is the unique identifier to distinguish different image data identities, and its definition is as follows.
````C++
struct FrameId
{
enum Enumeration {
NotUsed = 0,
LeftCamera = 1 << 0, 		//The original image of the left camera
RightCamera = 1 << 1, 		//Right camera original image
CalibLeftCamera = 1 << 2, 		// left camera calibration map
CalibRightCamera = 1 << 3, 		//Right camera calibration map
DisparityDSBak = 1 << 4,
DisparityUV = 1 << 5,
Disparity = 1 << 6, 		// disparity data
DisparityPlus = 1 << 7,
DisparityDS = 1 << 8, 		// downsample disparity data
Lane = 1 << 9, 		// Lane line data
Obstacle = 1 << 10, 		// Obstacle data
Compound = 1 << 11, 		// Composite data of left calibration map and obstacle data
LDownSample = 1 << 12, 		// downsample left calibration map
RDownSample = 1 << 13, 		// downsample right calibration map
LaneExt = 1 << 14, 		// Composite data of left calibration map and lane line data
};
};
````
The relationship between `camera original image` and `camera calibration image` is: the calibration image is obtained by applying calibration data to the original image.
At present, all shipped devices have completed the calibration process and are in `normal mode`. In this mode, the `left and right camera calibration map` is provided instead of the `left and right camera original image`.

### It is worth noting that:
1. FrameId::LeftCamera cannot be obtained at the same time as FrameId::CalibLeftCamera.
2. FrameId::RightCamera cannot be obtained at the same time as FrameId::CalibRightCamera.
3. FrameId::Lane cannot be obtained at the same time as FrameId::LaneExt.
4. FrameId::Obstacle cannot be obtained at the same time as FrameId::Compound.
5. FrameId::Disparity means that there is a 104-pixel blank on the left side of the disparity data, which is filled after matching.

# Quick start

## Install SmartEye SDK

#### SmartEye SDK directory structure

```bash
Sdk-Release_linux-g++_v0.2.33
├── Runtime
│ └── Bin
└── Src
└── SDK
├── comm.pri
├── Examples
│ ├── Disparity2Real3dDemo
│ ├── Disparity2RGBDemo
│ ├── DisplayFramesDemo
│ ├── GetCameraParamDemo
│ ├── GetCompoundDemo
│ ├── GetLaneExtDemo
│ ├── GetMotionDataDemo
│ ├── PointCloudDemo
│ │── StereoCameraDemo
│ └── GetDeviceInfoDemo
├── inc
└── SatpExt
````

#### Minimum C++11+ support
#### Minimum CMake 3.0 support
#### Qt: Linux - 5.12; Windows - 5.13

#### OpenCV: 3.4.1

#### The steps to use the development package directly are as follows:

1. Download from the official website or provide the specified C + + SDK compressed package by FAE.

2. Unzip the downloaded package, which contains the SDK implementation code and related demo.

3. Execute build. Under the scripts path SH script to compile.

4. The compiled SDK module and demo program are automatically stored in `/sdkrelease_vX.X.X/Runtime/bin ` path.

5. Add the path of the library file to the environment variable,

   eg：`export LD_ LIBRARY_ PATH=/root/adas/SdkRelease_vX.X.X/Runtime/Bin`.

6. If the machine has QT installed by default, please delete the default QT directory.

7. Then run the corresponding demo, eg: `./StereoCameraDemo`.

## example description
### StereoCameraDemo
`StereoCameraDemo` covers multiple functions such as parameter request, rotation matrix request, firmware upgrade, acquisition of 3D point cloud data, etc. It is suitable for secondary development with extensive functional requirements. However, due to the complex functions, there are many things to be implemented in `mycamerahandler.cpp`.

````C++
#include <iostream>
#include <string>
#include <stdio.h>

#include "stereocamera.h"
#include "frameid.h"
#include "taskiddef.h"
#include "mycamerahandler.h"
#include "calibrationparams.h"
#include "rotationmatrix.h"

int main(int argc, char *argv[])
{
	//Call the static method connect, create a StereoCamera instance, complete the initialization, specify the IP address of the device, and establish a connection through Ethernet.
StereoCamera *cameraA = StereoCamera::connect("192.168.1.251");

	//Create an instance to receive and process camera data
MyCameraHandler *cameraHandlerA = new MyCameraHandler("camera A");

//Select the task you want to run
cameraA->enableTasks(TaskId::ObstacleTask | TaskId::LaneTask | TaskId::DisplayTask);

	//Set the ID of the data corresponding to the request, you can create multiple StereoCamera objects at the same time, and establish connections with multiple cameras.
	//By configuring different FrameIds, different types of data of the same camera can be obtained at the same time.
cameraA->requestFrame(cameraHandlerA, FrameId::Disparity );

//Another instance can be created through the same method to connect to other services
// StereoCamera *cameraB = StereoCamera::connect("192.168.20.100");
// MyCameraHandler *cameraHandlerB = new MyCameraHandler("camera B");
// cameraB->requestFrame(cameraHandlerB, FrameId::CalibRightCamera);

// prevent app to exit.
int c = 0;
while (c!= 'x')
{
switch (c)
{
case 'f':
{
			//Request camera calibration parameters for converting parallax data to distance information
StereoCalibrationParameters params;
if (cameraA->requestStereoCameraParameters(params)) {
cameraHandlerA->setStereoCalibParams(params);
} else {
std::cout << "Stereo camera parameters request failed." << std::endl;
}
}
break;
case 'r':
{
			//Request a rotation matrix for the transformation of pixel coordinates between the image coordinate system and the world coordinate system
RotationMatrix rotationMatrix;
if (cameraA->requestRotationMatrix(rotationMatrix)) {
cameraHandlerA->setRotationMatrix(rotationMatrix);
} else {
std::cout << "Rotation matrix request failed." << std::endl;
}
}
break;
case 'l':
{
        	//Get the ambient light brightness
int light = -1;
bool result = false;
result = cameraA->getAmbientLight(light);
if(result){
switch(light){
case 1:
std::cout<<"day"<<std::endl;
break;
case 2:
std::cout<<"dusk"<<std::endl;
break;
case 3:
std::cout<<"night"<<std::endl;
break;
default:
std::cout<<"unknown"<<std::endl;
}
}
}
break;
case 'z':
{
        	//Disconnect
cameraA->disconnectFromServer();
}
break;
default:
break;
}
c = getchar();
}
}
````
In `mycamerahandler.cpp`, except for the `setRotationMatrix` function, there are specific instructions in other demos, so I won't go into details.
In the sample code, `setRotationMatrix` is used to write the two matrices into the specified file after requesting the rotation matrix. The Demo can be modified according to personal needs.
````C++
void MyCameraHandler::setRotationMatrix(RotationMatrix &rotationMatrix)
{
mRotationMatrix = rotationMatrix;
FILE * fp = nullptr;
fp = fopen(kRotationMartrixFilePath.data(), "wb+");
if (!fp) {
std::cout << kRotationMartrixFilePath << "file not open" << std::endl;
return;
}

fprintf(fp, "Real3DToImage:\n");
for (int row = 0; row < 3; row++) {
for (int col = 0; col < 4; col++) {
fprintf(fp, "%e\t", mRotationMatrix.real3DToImage[col + 4 * row]);
}
fprintf(fp, "\n");
}

fprintf(fp, "ImageToReal3D:\n");
for (int row = 0; row < 3; row++) {
for (int col = 0; col < 3; col++) {
fprintf(fp, "%e\t", mRotationMatrix.imageToReal3D[col + 3 * row]);
}
fprintf(fp, "\n");
}

fclose(fp);
}
````


### Disparity2Real3dDemo - Disparity data converted to 3D point cloud data
After creating an instance in `main.cpp` to complete the initialization and successfully connecting the device through the network, set the running Task `DisplayTask` through `enableTasks`, call `requestStereoCameraParameters` to request calibration parameters, and call `requestFrame` to request image data `Disparity` .

````C++
#include <iostream>
#include <string>
#include <chrono>
#include <thread>

#include "stereocamera.h"
#include "frameid.h"
#include "taskiddef.h"
#include "mycamerahandler.h"
#include "protocol.h"
#include "calibrationparams.h"

int main(int argc, char *argv[])
{
StereoCamera *cameraA = StereoCamera::connect("192.168.1.251");
MyCameraHandler *cameraHandlerA = new MyCameraHandler("camera A");

while (!cameraA->getProtocol()->isConnected()) {
printf("connecting...\n");
std::this_thread::sleep_for(std::chrono::seconds(1));
}

std::this_thread::sleep_for(std::chrono::seconds(3));

cameraA->enableTasks(TaskId::DisplayTask);

StereoCalibrationParameters params;
cameraA->requestStereoCameraParameters(params);
cameraHandlerA->setStereoCalibParams(params);

cameraA->requestFrame(cameraHandlerA, FrameId::Disparity);

int c = 0;
while (c!= 'x')
{
c = getchar();
}
}
````
To realize the function of converting parallax data into 3D point cloud data, it is also necessary to implement in `mycamerahandler.cpp`:

1. `setStereoCalibParams`, assign the requested camera calibration parameters to the variable `mStereoCalibrationParameters` for subsequent data conversion.

````C++
void MyCameraHandler::setStereoCalibParams(StereoCalibrationParameters &params)
{
mStereoCalibrationParameters = params;
mIsCalibParamReady = true;
std::cout << "calib params is ready!!!" << std::endl;
}
````
2. `handleRawFrame` inherits from the class `FrameHandler`, and handles the raw data of the `RawImageFrame` type structure from the device side, which may be simple image data, the result of algorithm processing, or the composite data superimposed by the two.

````C++
void MyCameraHandler::handleRawFrame(const RawImageFrame *rawFrame)
{
std::cout << mName
<< ", got image, id: " << rawFrame->frameId
<< " , time stamp: " << rawFrame->time
<< std::endl;
              
processFrame(rawFrame->frameId, (char*)rawFrame + sizeof(RawImageFrame),
rawFrame->dataSize, rawFrame->width, rawFrame->height, rawFrame->format);
}
````
3. `processFrame` is the next processing logic, users can perform different processing according to their own needs. The differences between the demos are also generated from the processing logic.
According to the function we want to achieve - disparity to distance, choose the disparity data whose `frameId` is `Disparity` and combine the camera calibration parameters and the number of decimal places of the disparity data (obtained by `getDisparityBitNum`) for further processing. If the previous request for camera calibration parameters fails, the subsequent processing is meaningless.

````C++
void MyCameraHandler::processFrame(int frameId, char *image, uint32_t dataSize, int width, int height, int frameFormat)
{
switch(frameId) {
case FrameId::Disparity:
{
int bitNum = DisparityConvertor::getDisparityBitNum(frameFormat);
if (mIsCalibParamReady) {
handleDisparityByLookupTable((unsigned char *)image, width, height, bitNum);
} else {
std::cout << "calib params is not ready!!!" << std::endl;
}
}
break;
default:
break;
}
}
````
Here are two methods to choose from `handleDisparityByLookupTable` and `handleDisparityPointByPoint`.

`handleDisparityByLookupTable` is to complete data conversion by looking up the table:

1. First, generate lookup tables in three directions of x, y, and z through the three interfaces `generateLookUpTableX`, `generateLookUpTableY` and `generateLookUpTableZ` respectively, which can only be generated once during the demo running.
2. Call the interface `getWholeXDistanceByLookupTable` to output the image data, calibration data and the x-direction lookup table, and you can get the coordinate values of all pixels in the full-frame image in the x-direction. In the same way, y and z coordinate values are obtained through `generateLookUpTableY` and `generateLookUpTableZ`. So far, the conversion of full-frame image parallax data into distance information has been completed.
3. In the Demo, the converted 3D point cloud data is written into the specified file as a reference, and it is written only once, and can be modified according to personal needs.

````c++
void MyCameraHandler::handleDisparityByLookupTable(unsigned char *image, int width, int height, int bitNum)
{
std::cout << "width: " << width << ", height: " << height << std::endl;

static float *lookUpTableX = new float[kDisparityCount*(int)pow(2, bitNum)*width];
static float *lookUpTableY = new float[kDisparityCount*(int)pow(2, bitNum)*height];
static float *lookUpTableZ = new float[kDisparityCount*(int)pow(2, bitNum)];
if(!mIsLookupTableGenerated) {
DisparityConvertor::generateLookUpTableX(width, bitNum, mStereoCalibrationParameters.Tx, mStereoCalibrationParameters.cx, lookUpTableX);
DisparityConvertor::generateLookUpTableY(height, bitNum, mStereoCalibrationParameters.Tx, mStereoCalibrationParameters.cy, lookUpTableY);
DisparityConvertor::generateLookUpTableZ(bitNum, mStereoCalibrationParameters.Tx, mStereoCalibrationParameters.focus, lookUpTableZ);
mIsLookupTableGenerated = true;
}

float *xDistance = new float[width*height];
DisparityConvertor::getWholeXDistanceByLookupTable(image, width, height, bitNum, lookUpTableX, xDistance);
float *yDistance = new float[width*height];
DisparityConvertor::getWholeYDistanceByLookupTable(image, width, height, bitNum, lookUpTableY, yDistance);
float *zDistance = new float[width*height];
DisparityConvertor::getWholeZDistanceByLookupTable(image, width, height, lookUpTableZ, zDistance);

static int index = 0;
index++;
FILE * fp = nullptr;
if (index == 1) {
fp = fopen(k3dByLookUpTableFilePath.data(), "wb+");
if (!fp) {
std::cout << k3dByLookUpTableFilePath << "file not open" << std::endl;
return;
}
}
for(int i = 0; i < width*height; i++) {
float x, y, z;
x = xDistance[i];
y = yDistance[i];
z = zDistance[i];
if((fabs(x) > 200000.0f)||(fabs(y) > 200000.0f)||(fabs(z) > 200000.0f)) {
x = 0.0f;
y = 0.0f;
z = 0.0f;
}
if (index == 1) {
fprintf(fp, "%f %f %f %d\n", x, y, z, i);
}
}
if (index == 1) {
fclose(fp);
}

delete [] xDistance;
delete [] yDistance;
delete [] zDistance;
}
````
`handleDisparityPointByPoint` is in pixels, and obtains the coordinate value of each point in the world coordinate system one by one:
1. Since our disparity data is not stored in `float` type, we first need to call `convertDisparity2FloatFormat` to convert the disparity data to float type for further processing.
2. Use the converted parallax data, camera calibration parameters, and the coordinates of the pixel on the image `(posX, posY)` as input parameters, call `getPointXYZDistance` to get the coordinate value of the pixel in the world coordinate system` (x, y, z)` .
3. Perform the above processing on all `(width×height)` pixels of the full-frame image, that is, to obtain the coordinate values of all the pixels of the full-frame image in the world coordinate system. The conversion of disparity data to distance information is complete.
4. The 3D point cloud data is stored once for reference in the Demo, and can be modified according to personal needs.

````C++
void MyCameraHandler::handleDisparityPointByPoint(unsigned char *image, int width, int height, int bitNum)
{
std::cout << "width: " << width << ", height: " << height << std::endl;

static float *floatData = new float[width * height];
DisparityConvertor::convertDisparity2FloatFormat(image, width, height, bitNum, floatData);

static int index = 0;
index++;
FILE * fp = nullptr;
if (index == 1) {
fp = fopen(k3dPointByPointFilePath.data(), "wb+");
if (!fp) {
std::cout << k3dPointByPointFilePath << "file not open" << std::endl;
return;
}
}

for(int posY = 0; posY < height; posY++) {
for(int posX = 0; posX < width; posX++) {
float x, y, z;
DisparityConvertor::getPointXYZDistance(image, width, height, bitNum, mStereoCalibrationParameters.Tx, mStereoCalibrationParameters.focus, mStereoCalibrationParameters.cx, mStereoCalibrationParameters.cy, posX, posY, x, y, z);

if((fabs(x) > 200000.0f)||(fabs(y) > 200000.0f)||(fabs(z) > 200000.0f)) {
x = 0.0f;
y = 0.0f;
z = 0.0f;
}
if (index == 1) {
fprintf(fp, "%f %f %f %d\n", x, y, z, (posY * width + posX));
}
}
}
if (index == 1) {
fclose(fp);
}
}
````

### Disparity2RGBDemo - Pseudocolor rendering of disparity data
According to the difference of disparity values in different regions in the image, pseudo-color rendering is performed to achieve the purpose of visually distinguishing.
The selection of `TaskId` and `FrameId` in `main.cpp` is the same as `Disparity2Real3dDemo`, but does not require camera calibration parameters.
````C++
#include <iostream>
#include <string>
#include <cstdio>

#include "stereocamera.h"
#include "frameid.h"
#include "taskiddef.h"
#include "mycamerahandler.h"

int main(int argc, char *argv[])
{
StereoCamera *cameraA = StereoCamera::connect("192.168.1.251");
MyCameraHandler *cameraHandlerA = new MyCameraHandler("camera A");

cameraA->enableTasks(TaskId::DisplayTask);

cameraA->requestFrame(cameraHandlerA, FrameId::Disparity);

int c = 0;
while (c!= 'x')
{
c = getchar();
}
}
````
`mycamerahandler.cpp` also needs to implement `handleRawFrame` and `processFrame`, but the processing logic of `processFrame` is different.
According to the required implementation function - disparity data pseudo-color rendering, select the disparity data whose `FrameId` is `Disparity` for processing.
After calling `convertDisparity2FloatFormat` combined with the number of decimal places occupied by the disparity data (obtained by `getDisparityBitNum`) to convert the disparity data to `float` type, the pseudo-color rendering of the disparity data can be completed through the interface `convertDisparity2RGB`.
The rendered data is not displayed or stored in the Demo, which can be modified according to individual needs.

````C++
void MyCameraHandler::processFrame(int frameId, const unsigned char *image,int64_t time, int width, int height, int frameFormat)
{
switch(frameId) {
case FrameId::Disparity:
{
int bitNum = DisparityConvertor::getDisparityBitNum(frameFormat);
static unsigned char *rgbBuf = new unsigned char[width * height * 3];
static float *floatData = new float[width * height];

DisparityConvertor::convertDisparity2FloatFormat(image, width, height, bitNum, floatData);
DisparityConvertor::convertDisparity2RGB(floatData, width, height, kMinValidDisparityValue, kMaxValidDisparityValue, rgbBuf);
}
break;
default:
break;
}
}
````
### GetCameraParamDemo - get camera calibration parameters
Acquisition of camera calibration parameters without requesting image data. In `main.cpp`, create an instance and complete the initialization, confirm that the connection is successful, wait for 3s (data loading), and call the `requestStereoCameraParameters` interface to obtain the camera's calibration parameters from the device side. The `dumpCameraParams` function only prints the requested parameters, which can be modified according to personal needs.
````C++
#include <chrono>
#include <thread>
#include <cstdio>

#include "stereocamera.h"
#include "protocol.h"
#include "calibrationparams.h"

void dumpCameraParams(const StereoCalibrationParameters &params)
{
printf("************ Camera params ************\n");
printf("Focus: %e pixel\n", params.focus);
printf("Optical center X: %e pixel\n", params.cx);
printf("Optical center Y: %e pixel\n", params.cy);
printf("R-vector roll: %e rad\n", params.RRoll);
printf("R-vector pitch: %e rad\n", params.RPitch);
printf("R-vector yaw: %e rad\n", params.RYaw);
printf("Translation x: %e mm\n", params.Tx);
printf("Translation y: %e mm\n", params.Ty);
printf("Translation z: %e mm\n", params.Tz);
printf("****************************************\n");
}

int main(int argc, char *argv[])
{
StereoCamera *camera = StereoCamera::connect("192.168.1.251");

while (!camera->isConnected()) {
printf("connecting...\n");
std::this_thread::sleep_for(std::chrono::seconds(1));
}

std::this_thread::sleep_for(std::chrono::seconds(3)); // wait for rtdb load

StereoCalibrationParameters params;
camera->requestStereoCameraParameters(params);

dumpCameraParams(params);

int c = 0;
while (c!= 'x')
{
c = getchar();
}
}
````

### GetCompoundDemo——The obstacle information is merged with the left image to draw the overlay effect
After creating an instance in `main.cpp` to complete the initialization and successfully connecting the device through the network, set the running task `DisplayTask | ObstacleTask` through `enableTasks`, and call `requestFrame` to request image data `Compound`. `ObstacleTask` is the basis for the operation of the obstacle extraction algorithm, and `Compound` is the left calibration map data superimposed with the processing results of the obstacle algorithm.
````C++
#include <iostream>
#include <string>

#include "stereocamera.h"
#include "frameid.h"
#include "taskiddef.h"
#include "mycamerahandler.h"
#include "calibrationparams.h"

int main(int argc, char *argv[])
{
StereoCamera *cameraA = StereoCamera::connect("192.168.1.251");
MyCameraHandler *cameraHandlerA = new MyCameraHandler("camera A");

cameraA->enableTasks(TaskId::DisplayTask | TaskId::ObstacleTask);

cameraA->requestFrame(cameraHandlerA, FrameId::Compound);

int c = 0;
while (c!= 'x')
{
c = getchar();
}
}
````
The processing of `Compound` is implemented in the `processFrame` function of `mycamerahandler.cpp`, and the parameters are slightly different from the processing of parallax data. First convert the grayscale data to RGB format through `imageGrayToRGB`, in order to realize the color rendering of the final obstacle frame. Then call the `paintObstacle` interface to complete the drawing of the overlay effect. The `extended` used is essentially the extraction result data for the stripped obstacles. The sample code does not display or store the processed data, and can be modified according to individual needs.

````C++
#include <iostream>
#include <string>

#include "mycamerahandler.h"
#include "satpext.h"
#include "frameid.h"
#include "roadwaypainter.h"
#include "obstaclepainter.h"

MyCameraHandler::MyCameraHandler(std::string name):
mName(name)
{}

void MyCameraHandler::handleRawFrame(const RawImageFrame *rawFrame)
{
// std::cout << mName
// << ", got image, id: " << rawFrame->frameId
// << " , time stamp: " << rawFrame->time
// << std::endl;

processFrame(rawFrame->frameId, (char*)rawFrame + sizeof(RawImageFrame), (char*)rawFrame + rawFrame->dataSize + sizeof(RawImageFrame), rawFrame->time, rawFrame->width, rawFrame->height);
}

void MyCameraHandler::processFrame(int frameId, char *image, char *extended, int64_t time, int width, int height)
{
switch(frameId) {
case FrameId::Compound:
{
static unsigned char *rgbBuf = new unsigned char[width * height * 3];
RoadwayPainter::imageGrayToRGB((unsigned char*)image, rgbBuf, width, height);
ObstaclePainter::paintObstacle(extended, rgbBuf, width, height, true, false);
}
break;
default:
break;
}
}
````

### GetLaneExtDemo——The lane line information is merged with the left image to draw the overlay effect

After creating an instance in `main.cpp` to complete the initialization and successfully connecting the device through the network, set the running task `DisplayTask | LaneTask` through `enableTasks`, and call `requestFrame` to request the image data `LaneExt`. `LaneTask` is the basis for the operation of the lane line recognition algorithm, and `LaneExt` is the left calibration map data superimposed with the processing results of the lane line algorithm.
````C++
#include <iostream>
#include <string>

#include "stereocamera.h"
#include "frameid.h"
#include "taskiddef.h"
#include "mycamerahandler.h"

int main(int argc, char *argv[])
{
StereoCamera *cameraA = StereoCamera::connect("192.168.1.251");
MyCameraHandler *cameraHandlerA = new MyCameraHandler("camera A");

cameraA->enableTasks(TaskId::DisplayTask | TaskId::LaneTask);

cameraA->requestFrame(cameraHandlerA, FrameId::LaneExt);

int c = 0;
while (c!= 'x')
{
c = getchar();
}
}
````
The processing of `LaneExt` is implemented in the `processFrame` function of `mycamerahandler.cpp`. First convert the grayscale data to RGB format through `imageGrayToRGB`, in order to realize the color rendering of the final obstacle frame. Then call the `paintRoadway` interface to complete the drawing of the overlay effect. The `extended` used is essentially the stripped lane line result data. In addition to drawing, it can also be parsed and printed according to requirements. The sample code does not display or store the processed data, and can be modified according to individual needs.

````C++
#include <iostream>
#include <string>

#include "mycamerahandler.h"
#include "satpext.h"
#include "frameid.h"
#include "LdwDataInterface.h"
#include "roadwaypainter.h"

MyCameraHandler::MyCameraHandler(std::string name):
mName(name)
{}

void MyCameraHandler::handleRawFrame(const RawImageFrame *rawFrame)
{
// std::cout << mName
// << ", got image, id: " << rawFrame->frameId
// << " , time stamp: " << rawFrame->time
// << std::endl;

processFrame(rawFrame->frameId, (char*)rawFrame + sizeof(RawImageFrame), (char*)rawFrame + rawFrame->dataSize + sizeof(RawImageFrame), rawFrame->time, rawFrame->width, rawFrame->height);
}

void MyCameraHandler::processFrame(int frameId, char *image, char *extended, int64_t time, int width, int height)
{
switch(frameId) {
case FrameId::LaneExt:
{
FrameDataExtHead *header = reinterpret_cast<FrameDataExtHead *>(extended);
LdwDataPack *ldwDataPack = (LdwDataPack *)header->data;
std::cout << "ldw degree is: " << ldwDataPack->roadway.left_Lane.left_Boundary.degree << std::endl;

static unsigned char *rgbBuf = new unsigned char[width * height * 3];
RoadwayPainter::imageGrayToRGB((unsigned char*)image, rgbBuf, width, height);
mIsLaneDetected = RoadwayPainter::paintRoadway(header->data, rgbBuf, width, height);
}
break;
default:
break;
}
}
````

### DisplayFramesDemo - Display image data (left, right, disparity)
After creating an instance in `main.cpp` to complete the initialization and successfully connecting the device through the network, call `requestFrame` to request image data `CalibLeftCamera` `CalibRightCamera` `Disparity`. You can choose to request one or more image data. It is worth noting that this sample code relies on the OpenCV environment.
````C++
#include <iostream>
#include <opencv2/highgui.hpp>

#include "stereocamera.h"
#include "framemonitor.h"
#include "frameid.h"
#include "taskiddef.h"

/**
* @brief main
* @note the sdk base on qt
* @note on linux x86 desktop platform, there are some problems to use opencv_highgui module without qt event loop
* @note here, calling functions of highgui in qt event loop is correct, called by invokeInLoopThread() by class StereoCamera
*/

int main(int argc, char *argv[])
{
StereoCamera *camera = StereoCamera::connect("192.168.1.251");
std::unique_ptr<FrameMonitor> frameMonitor(new FrameMonitor);

camera->enableTasks(TaskId::DisplayTask);
camera->requestFrame(frameMonitor.get(), FrameId::CalibLeftCamera | FrameId::CalibRightCamera | FrameId::Disparity);

std::function<int()> draw_func = [&frameMonitor]() -> int {
const cv::Mat &leftFrame = frameMonitor->getFrameMat(FrameId::CalibLeftCamera);
const cv::Mat &rightFrame = frameMonitor->getFrameMat(FrameId::CalibRightCamera);
const cv::Mat &disparity = frameMonitor->getFrameMat(FrameId::Disparity);

if (!leftFrame.empty()) {
cv::imshow("Left", leftFrame);
}
        
if (!rightFrame.empty()) {
cv::imshow("Right", rightFrame);
}

if (!disparity.empty()) {
cv::imshow("Disparity", disparity);
}

return cv::waitKey(80);
};

camera->invokeInLoopThread([]{
cv::namedWindow("Left");
cv::namedWindow("Right");
cv::namedWindow("Disparity");
});

// main thread loop for drawing images
while (true) {
frameMonitor->waitForFrames(); // wait for frames ready

int key = 0;
camera->invokeInLoopThread([&key, &draw_func]{
key = draw_func();
});

if (key == 27) {
// press Esc to close
break;
}
}

camera->invokeInLoopThread([]{
cv::destroyAllWindows();
});

return 0;
}
````
The processing of image data is implemented in the `processFrame` function of `framemonitor.cpp`. By judging `FrameId`, image data with different IDs can be differentiated. The `loadFrameData2Mat` function implements loading `Frame` into `Mat`, and the main function calls `cv::imshow` to implement the image display function. It supports color rendering of parallax data, and also supports displaying as grayscale images, which can be achieved by modifying Demo.

````C++
void FrameMonitor::processFrame(const RawImageFrame *rawFrame)
{
switch (rawFrame->frameId) {
case FrameId::Disparity:
{
// only for FrameFormat::Disparity16, bitNum = 5
std::lock_guard<std::mutex> lock(mMutex);
loadFrameData2Mat(rawFrame, mDisparityMat);

// std::cout << "update disparity mat" << std::endl;
mFrameReadyFlag = true;
mFrameReadyCond.notify_one();
}
break;
case FrameId::CalibLeftCamera:
{
std::lock_guard<std::mutex> lock(mMutex);
loadFrameData2Mat(rawFrame, mLeftMat);

// std::cout << "update left mat" << std::endl;
mFrameReadyFlag = true;
mFrameReadyCond.notify_one();
}
break;
case FrameId::CalibRightCamera:
{
std::lock_guard<std::mutex> lock(mMutex);
loadFrameData2Mat(rawFrame, mRightMat);

// std::cout << "update right mat" << std::endl;
mFrameReadyFlag = true;
mFrameReadyCond.notify_one();
}
break;
}
}
````

````C++
void FrameMonitor::loadFrameData2Mat(const RawImageFrame *frameData, cv::Mat &dstMat)
{
int width = frameData->width;
int height = frameData->height;
const unsigned char *imageData = frameData->image;

switch (frameData->format) {
case FrameFormat::Disparity16:
{
// DisparityConvertor::convertDisparity2FloatFormat(imageData, width, height, 5, mDisparityFloatData.get());
// DisparityConvertor::convertDisparity2RGB(mDisparityFloatData.get(), width, height, 0, 45, mRgbBuffer.get());
// cv::Mat dispMat(height, width, CV_8UC3, mRgbBuffer.get());
// cv::resize(dispMat, dstMat, dstMat.size());

cv::Mat dispMat(height, width, CV_16U, (void*)imageData);
cv::resize(dispMat, dstMat, dstMat.size());
cv::normalize(dstMat, dstMat, 0, 255, cv::NORM_MINMAX, CV_8U);
}
break;
case FrameFormat::Gray:
{
cv::Mat grayMat(height, width, CV_8UC1, (void*)imageData);
cv::resize(grayMat, dstMat, dstMat.size());
}
break;
case FrameFormat::YUV422:
{
YuvToRGB::YCbYCr2Rgb(imageData, (char *)mRgbBuffer.get(), width, height);
cv::Mat yuv422Mat(height, width, CV_8UC3, mRgbBuffer.get());
cv::resize(yuv422Mat, dstMat, dstMat.size());
cv::cvtColor(dstMat, dstMat, CV_RGB2BGR);
}
break;
case FrameFormat::YUV422Plannar:
{
YuvToRGB::YCbYCrPlannar2Rgb(imageData, (char *)mRgbBuffer.get(), width, height);
cv::Mat yuv422PlannarMat(height, width, CV_8UC3, mRgbBuffer.get());
cv::resize(yuv422PlannarMat, dstMat, dstMat.size());
cv::cvtColor(dstMat, dstMat, CV_RGB2BGR);
}
break;
}
}
````
### GetMotionDataDemo - Get IMU data
The acceleration, angular velocity range, and acquisition frequency can be set by modifying the sample code. At present, `motion data` is sent from the device side as the additional data of the left calibration map, so it is necessary to request the left calibration map.
````C++
#include <iostream>
#include <string>
#include <chrono>
#include <thread>
#include "stereocamera.h"
#include "frameid.h"
#include "taskiddef.h"
#include "mycamerahandler.h"

int main(int argc, char *argv[])
{
StereoCamera *cameraA = StereoCamera::connect("192.168.1.251");
MyCameraHandler *cameraHandlerA = new MyCameraHandler("camera A");

bool isConnected = false;
while (!isConnected) {
printf("connecting...\n");
isConnected = cameraA->isConnected();
std::this_thread::sleep_for(std::chrono::seconds(1));
}

//get imu parameters
std::cout << "accel range:" << cameraA->getImuAccelRange() << std::endl;
std::cout << "rotation range:" << cameraA->getImuRotationRange() << std::endl;
std::cout << "frequence::" << cameraA->getImuReadFrequence() << std::endl;

//set imu parameters
cameraA->setImuAccelRange(4); //2,4,8,16;default is 4
cameraA->setImuRotationRange(500); //250,500,1000,2000;default is 500
cameraA->setImuReadFrequence(100); //10-100Hz;default is 100Hz

cameraA->requestFrame(cameraHandlerA, FrameId::CalibLeftCamera);
cameraA->enableMotionData(true);
MotionData data;
while (true) {
std::this_thread::sleep_for(std::chrono::milliseconds(80));
int size = cameraHandlerA->getMotionListSize();
for (int index = 0;index < size;index++) {
cameraHandlerA->readMotionData(data);
std::cout << "accel x:" << data.accelX << "g" << std::endl;
std::cout << "accel y:" << data.accelY << "g" << std::endl;
std::cout << "accel z:" << data.accelZ << "g" << std::endl;
std::cout << "gyro x:" << data.gyroX << "deg/s" << std::endl;
std::cout << "gyro y:" << data.gyroY << "deg/s" << std::endl;
std::cout << "gyro z:" << data.gyroZ << "deg/s" << std::endl;
std::cout << "time stamp:" << data.timestamp << "ms" << std::endl;
}
}
}

````

### PointCloudDemo - Demo integrating PCL
PCL is integrated to support 3D reconstruction using PCL after obtaining parallax data from the device. This sample code depends on the OpenCV environment
````C++
#include <iostream>
#include <string>
#include <chrono>
#include <thread>

#include "stereocamera.h"
#include "frameid.h"
#include "taskiddef.h"
#include "mycamerahandler.h"
#include "protocol.h"
#include "calibrationparams.h"
#include "pcviewer.h"

int main(int argc, char *argv[])
{
StereoCamera *cameraA = StereoCamera::connect("192.168.1.251");
MyCameraHandler *cameraHandlerA = new MyCameraHandler("camera A");

while (!cameraA->getProtocol()->isConnected()) {
std::cout << "connecting..." << std::endl;
std::this_thread::sleep_for(std::chrono::seconds(1));
}

std::this_thread::sleep_for(std::chrono::seconds(3));

cameraA->enableTasks(TaskId::DisplayTask);

StereoCalibrationParameters params;
cameraA->requestStereoCameraParameters(params);
cameraHandlerA->setStereoCalibParams(params);
cameraA->requestFrame(cameraHandlerA, FrameId::Disparity);

std::shared_ptr<PCViewer> viewer(new PCViewer(true));

// main loop for showing point cloud
while (!viewer->wasStopped()) {
auto cloud = cameraHandlerA->getCloud();
viewer->update(cloud);
}

std::cout << "stopped" << std::endl;
viewer->close();

cameraA->disconnectFromServer();
delete cameraA;
delete cameraHandlerA;

return 0;
}

````
### GetDeviceInfoDemo - get device information
Access to device information without requesting image data. In `main.cpp`, create an instance and complete the initialization, confirm that the connection is successful, wait for 3s (data loading), and call the `requestDeviceInfo` interface to obtain the device-related information from the device side. The `dumpDeviceInfo` function only prints the requested device information, which can be modified according to personal needs.
````C++
#include <chrono>
#include <thread>
#include <iostream>
#include <fstream>
#include <string>
#include "stereocamera.h"
#include "protocol.h"
#include "deviceinfo.h"

#ifdef _WINDOWS
#include <direct.h>
#else
#include <unistd.h>
#endif

using namespace std;

string readTxt(string file)
{
ifstream infile;
infile.open(file.data()); //Connect the file stream object to the file
//assert(infile.is_open()); //If it fails, output an error message and terminate the program

string sdkVersion;
while (getline(infile, sdkVersion))
{
infile.close(); //Close the file input stream
return sdkVersion;
}
}

void dumpDeviceInfo(const DeviceInfo &deviceInfo, string externSdkVersion)
{
cout << "************ Camera params ************" << endl;
printf("Device model\n");
cout << deviceInfo.HardwareVersion << endl;
printf("Device serial number\n");
cout << deviceInfo.SerialNum << endl;
printf("Firmware type\n");
cout << deviceInfo.ProductCode << endl;
printf("Firmware version\n");
cout << deviceInfo.AdasVersion << endl;
printf("Platform version\n");
cout << deviceInfo.PlatformVersion << endl;
printf("Camera version\n");
cout << deviceInfo.CameraFirmwareVersion << endl;
printf("sdk version\n");
cout << deviceInfo.SdkVersion << endl;
printf("External SDK version\n");
cout << externSdkVersion << endl;
printf("obstacle algorithm version\n");
cout << deviceInfo.ObsVersion << endl;
printf("Lane line algorithm version\n");
cout << deviceInfo.LaneVersion << endl;
printf("Device status\n");
cout << deviceInfo.InitialSetupStatus << endl;
printf("cpu temperature\n");
cout << deviceInfo.CPUTemperature << endl;
}

#ifdef _WINDOWS
#else
string getDirectory()
{
char abs_path[1024];
int cnt = readlink("/proc/self/exe", abs_path, 1024);//Get the absolute path of the executable program
if (cnt < 0 || cnt >= 1024)
{
return NULL;
}

for (int i = cnt; i >= 0; --i)
{
if (abs_path[i] == '/')
{
abs_path[i + 1] = '\0';
break;
}
}

string path(abs_path);

return path;
}
#endif

int main(int argc, char *argv[])
{
StereoCamera *camera = StereoCamera::connect("192.168.1.251");

while (!camera->isConnected()) {
printf("connecting...\n");
std::this_thread::sleep_for(std::chrono::seconds(1));
}

std::this_thread::sleep_for(std::chrono::seconds(3)); // wait for rtdb load
string externSdkVersion;
#ifdef _WINDOWS
char cwd[256];
_getcwd(cwd, 256);
char* path;
if ((path = _getcwd(NULL, 0)) == NULL) {
cerr << "Error message : _getcwd error" << endl;
}
else {
string sdkVersion;
sdkVersion = "//sdkVersion";
sdkVersion = path + sdkVersion;
externSdkVersion = readTxt(sdkVersion);
}
#else
string sdkVersion;
sdkVersion = "//sdkVersion";
string path;
path = getDirectory();
sdkVersion = path + sdkVersion;
externSdkVersion = readTxt(sdkVersion);
#endif

DeviceInfo deviceInfo;
camera->requestDeviceInfo(deviceInfo);
dumpDeviceInfo(deviceInfo, externSdkVersion);

while (true)
{
camera->requestDeviceInfo(deviceInfo);
cout << "cpu temperature" << deviceInfo.CPUTemperature << endl;
std::this_thread::sleep_for(std::chrono::seconds(2));
}

int c = 0;
while (c!= 'x')
{
c = getchar();
}
}
````


# Interface Description

## Software interface definition

### StereoCamera class
#### Internet connection
##### Function name: connect()
Establish a connection with the device with the specified IP.
Parameter Name | Parameter Type | Input/Output | Description
:-: | :-: | :-: | :-:
addr | const char* | input | StereoCamera class object pointer

##### Function name: disconnectFromServer()
Disconnect from the network with the device.

##### Function name: isConnected()
Get the connection status and return it as `bool`, `true` is connected.

#### Calibration data request
function name|parameter name|parameter type|input/output|function
:-: | :-: | :-:| :-: | :-:
requestStereoCameraParameters() | params | StereoCalibrationParameters | output | request binary targeting parameters
requestMonoLeftCameraParameters() | params | MonoCalibrationParameters | output | request monocular left camera calibration parameters
requestMonoRightCameraParameters() | params | MonoCalibrationParameters | output | request monocular right camera calibration parameters

#### Image data request
##### Function name: requestFrame()
Request raw data or algorithmically processed conforming data.
parameter name | parameter type | description
:-: | :-: | :-:
frameHandler | FrameHandler | handler for processing image data, user-defined
frameIds | uint32_t | Unique ID of the required data

#### Rotation Matrix Request
##### Function name: requestRotationMatrix()
Request the rotation matrix used for the conversion between the image coordinate system and the world coordinate system. The return value is of type `bool`, indicating whether the rotation matrix is successfully obtained.
Parameter Name | Parameter Type | Input/Output | Description
:-: | :-: | :-: | :-:
params | RotationMatrix | output | rotation matrix

#### Firmware upgrade
#### Function name: updateFirmware()
Specify the upgrade package for device firmware upgrade.
Parameter Name | Type | Input/Output | Description
:-: | :-: | :-: | :-:
path | const char* | input | upgrade package absolute path with file name
Return value type: `int`
Return value description: upgrade package upload result
return value | description
:-: | :-:
0 | No exception found
-1 | Invalid file address
-2 | service busy (file uploading or upgrading)
-3 | Failed to open file
1 ~ 9 | Camera related anomalies (the specific error code returned is the same as the definition of the camera fault code in the anomaly detection)
19 | Equipment high temperature

#### Frame rate settings
#### Function name: setFrameRate()
Set the image frame rate. After the device restarts, the default frame rate will be restored. For products containing algorithm-related functions, the reduction of the frame rate will affect the algorithm. The return value is of type `bool`, indicating whether the frame rate is set successfully. In the disconnected state, setting the frame rate will fail.
parameter name | parameter type | description
:-: | :-: | :-:
rate | float | Frame rate (fps), user defined

The frame rate setting has certain limitations, which are closely related to the maximum frame rate that the current camera and platform can provide. The following table briefly describes the corresponding relationship:
| Maximum frame rate | Settable frame rate |
| :------------: | :----: |
| 12.5 | 12.5, 6.25, 4.17, 3.125, 2.5, 2.08, 1.79, 1.5625 |
| 15 | 15.0, 7.5, 5.0, 3.75, 3.0, 2.5, 2.14, 1.875 |
| 25 | 25.0, 12.5, 8.33, 6.25, 5.0, 4.17, 3.57, 3.125, 2.78, 2.5, 2.27, 2.08, 1.92, 1.79, 1.67, 1.5625 |
| 30 | 30.0, 15,0, 10.0, 7.5, 6.0, 5.0, 4.29, 3.75, 3.33, 3.0, 2.72, 2.5, 2.3, 2.14, 2.0, 1.875 |

#### Frame rate acquisition
#### Function name: getFrameRate()
Get the frame rate of the device image. The return value is of type `bool`, indicating whether the frame rate is obtained successfully. In the disconnected state, getting the frame rate will fail.
parameter name | parameter type | description
:-: | :-: | :-:
rate | float &| The current image frame rate of the device (fps)

#### Cross-thread execution
#### Function name: invokeInLoopThread()
Execute the incoming function inside the Qt eventloop thread.
parameter name | parameter type | description
:-: | :-: | :-:
method | std::function<void()> | function to be executed (function object with no return value and no incoming parameters)

#### Get ambient light brightness
#### Function name: getAmbientLight()
Get the current ambient light brightness of the device, and the return value indicates whether the acquisition is successful.
parameter name | parameter type | description
:-: | :-: | :-:
lightness | int & | The current ambient light level of the device

| lightness | Representation |
| :------------: | :----: |
|-1|Invalid/Unknown|
| 1|Daytime|
|2|Dusk/Evening|
|3|Night|

#### Get camera image status (whether abnormal)
#### Function name: getSmudgeStatus()
Get the current state of the camera image, the return value indicates whether the acquisition was successful
parameter name | parameter type | description
:-: | :-: | :-:
status | int & | image status

| status | Representation|
| :------------: | :----: |
|-1|Invalid/Unknown|
|0|normal|
|1|Not normal|

#### IMU related
#### Function name: setImuAccelRange()
Set the acceleration range, no return value.
parameter name | parameter type | description
:-: | :-: | :-:
value | int | Acceleration range in g (9.8m/s^2)
Support setting parameters: 2, 4, 8, 16, the default value is 4

#### Function name: setImuRotationRange()
Set the angular velocity range, no return value.
parameter name | parameter type | description
:-: | :-: | :-:
value | int | Angular velocity range in deg/s
Support setting parameters: 250, 500, 1000, 2000, the default value is 500

#### Function name: setImuReadFrequence()
Set the reading frequency of attitude data, no return value.
parameter name | parameter type | description
:-: | :-: | :-:
value | int | frequency in Hz
Support setting parameters: an integer between 10-100, the default value is 100

#### Function name: getImuAccelRange()
Get the acceleration range, the return value is the acceleration range, the unit is g (9.8m/s^2).
#### Function name: getImuRotationRange()
Get the angular velocity range, the return value is the angular velocity range, in deg/s.
#### Function name: getImuReadFrequence()
Get the reading frequency of attitude data, the return value is the reading frequency, the unit is HZ.
#### Function name: enableMotionData()
Get attitude data enable, no return value.
parameter name | parameter type | description
:-: | :-: | :-:
enable | bool | yes/no get attitude data
By modifying this flag bit, the acquisition of IMU data can be enabled/disabled.

### SATP class
function name | return value type | role
:-: | :-: | :-:
getProtocol() | Protocol * | Get the protocol name

### FrameHandler class
#### data processing
function name|parameter name|parameter type|parameter input/output|function
:-: | :-: | :-: | :-: | :-:
handleRawFrame() | rawFrame | RawImageFrame | input | After getting data information, call processing logic

### DisparityConvertor class
#### convertDisparity2FloatFormat()
It is used to integrate and convert the parallax image data obtained after the request, and store it in a `float` type variable
Parameter Name | Type | Input/Output | Description
:-: | :-: | :-: | :-:
src | const unsigned char* | input | pointer to parallax image data
dest | float* | output | converted data pointer
width | int | input | image width (unit: pixels)
height | int | input | image height (unit: pixels)
bitNum | int | input | The number of bits occupied by the fractional part of the disparity data
#### convertDisparity2RGB()
Renders the combined disparity data of type `float` in pseudocolor. Accurately render the disparity value within a certain range, the interval (minDsp, maxDisp) is recommended (0, 45)
Parameter Name | Type | Input/Output | Description
:-: | :-: | :-: | :-:
disparity | const float* | input | integrated float type disparity data pointer
rgbBug | unsigned char* | output | pointer to hold pseudocolor rendered image data
width | int | input | image width (unit: pixels)
height | int | input | image height (unit: pixels)
minDisp | int | input | lower rendering disparity value
maxDisp | int | input | upper limit of rendering disparity value
#### getDisparityBitNum()
According to the received disparity data frame format, return the floating-point number occupied by the fractional part of the disparity data as an `int` type, which is used for disparity data processing.
Parameter Name | Type | Input/Output | Description
:-: | :-: | :-: | :-:
format | int | input | frame format
#### generateLookUpTableX()
Generate a mapping table between the disparity data and the distance value in the X direction.
Parameter Name | Type | Input/Output | Description
:-: | :-: | :-: | :-:
width | int | input | image width (unit: pixels)
bitNum | int | input | The number of bits occupied by the fractional part of the disparity data
baseline | float | input | camera parameters: baseline (unit: mm)
cx | int | input | camera parameters: x coordinate of optical center (unit: pixel)
lookUpTableX | float* | output | The mapping table of the correspondence between the disparity data and the distance value in the X direction in the camera coordinate system
#### generateLookUpTableY()
Generate a mapping table of the correspondence between the disparity data and the distance value in the Y direction, which will be used to obtain the distance between all the pixels of the entire image in the Y direction of the camera coordinate system and the coordinate origin.
Parameter Name | Type | Input/Output | Description
:-: | :-: | :-: | :-:
height | int | input | image height (unit: pixels)
bitNum | int | input | The number of bits occupied by the fractional part of the disparity data
baseline | float | input | camera parameters: baseline (unit: mm)
cy | int | input | camera parameters: optical center y coordinate (unit: pixel)
lookUpTableY | float* | output | The mapping table of the correspondence between the disparity data and the distance value in the Y direction in the camera coordinate system
#### generateLookUpTableZ()
Generate the mapping table of the correspondence between the disparity data and the Z-direction distance value, which will be used to obtain the distance between all the pixels of the entire image and the coordinate origin in the Z-direction of the camera coordinate system.
Parameter Name | Type | Input/Output | Description
:-: | :-: | :-: | :-:
bitNum | int | input | The number of bits occupied by the fractional part of the disparity data
baseline | float | input | camera parameters: baseline (unit: mm)
foucs | float | input | camera parameters: focal length (unit: pixels)
lookUpTableZ | float* | output | The mapping table of the correspondence between the disparity data and the Z-direction distance value in the camera coordinate system
#### getWholeXDistanceByLookupTable()
Look up the table to obtain the distance (unit: mm) of all pixels of the entire image in the X direction of the camera coordinate system from the coordinate origin.
Parameter Name | Type | Input/Output | Description
:-: | :-: | :-: | :-:
src | const unsigned char* | input | parallax image data
width | int | input | image width (unit: pixels)
height | int | input | image height (unit: pixels)
bitNum | int | input | The number of bits occupied by the fractional part of the disparity data
lookUpTableX | float* | input | The mapping table of the correspondence between the disparity data and the distance value in the X direction in the camera coordinate system
distanceX | float* | output | pointer to the distance value of the full image in the X direction
#### getWholeYDistanceByLookupTable()
Look up the table to obtain the distance (unit: mm) of all pixels in the whole image from the coordinate origin in the Y direction of the camera coordinate system.
Parameter Name | Type | Input/Output | Description
:-: | :-: | :-: | :-:
src | const unsigned char* | input | parallax image data
width | int | input | image width (unit: pixels)
height | int | input | image height (unit: pixels)
bitNum | int | input | The number of bits occupied by the fractional part of the disparity data
lookUpTableY | float* | input | The mapping table of the correspondence between the disparity data and the distance value in the Y direction in the camera coordinate system
distanceY | float* | output | pointer to the distance value of the full image in the Y direction
#### getWholeZDistanceByLookupTable()
Look up the table to obtain the distance (unit: mm) of all pixels of the entire image in the Z direction of the camera coordinate system from the coordinate origin.
Parameter Name | Type | Input/Output | Description
:-: | :-: | :-: | :-:
src | const unsigned char* | input | parallax image data
width | int | input | image width (unit: pixels)
height | int | input | image height (unit: pixels)
lookUpTableZ | float* | input | The mapping table of the correspondence between the disparity data and the Z-direction distance value in the camera coordinate system
distanceZ | float* | output | pointer to the distance value of the full image in the Z direction
#### getPointDisparityValue()
According to the position coordinates (posX, posY) of the pixel point in the parallax image, get the parallax data value corresponding to the point, and return it in `float` type (unit: pixel).
Parameter Name | Type | Input/Output | Description
:-: | :-: | :-: | :-:
src | const unsigned char* | input | parallax image data
width | int | input | image width (unit: pixels)
height | int | input | image height (unit: pixels)
bitNum | int | input | The number of bits occupied by the fractional part of the disparity data
posX | int | input | The coordinate of the pixel in the X direction of the full image (unit: pixel)
posY | int | input | The coordinate of the pixel in the Y direction of the full image (unit: pixel)
#### getPointXYZDistance()
Get the distance (unit: mm) of the pixel point in the three directions of X, Y, and Z to the coordinate origin (the center of the left camera lens).
Parameter Name | Type | Input/Output | Description
:-: | :-: | :-: | :-:
src | const unsigned char* | input | parallax image data
width | int | input | image width (unit: pixels)
height | int | input | image height (unit: pixels)
bitNum | int | input | The number of bits occupied by the fractional part of the disparity data
baseline | float | input | camera parameters: baseline (unit: mm)
focus | float | input | camera parameters: focal length (unit: pixels)
cx | int | input | camera parameters: x coordinate of optical center (unit: pixel)
cy | int | input | camera parameters: optical center y coordinate (unit: pixel)
posX | int | input | pixel coordinates in the x-direction of the full image (unit: pixels)
posY | int | input | The y-coordinate of the pixel in the full image (unit: pixel)
xDistance | float | output | pixel distance in X direction
yDistance | float | output | pixel distance in Y direction
zDistance | float | output | pixel distance in Z direction
#### getWholeXDistance()
Get the distance (unit: mm) of all pixels of the entire image in the X direction of the camera coordinate system from the coordinate origin.
Parameter Name | Type | Input/Output | Description
:-: | :-: | :-: | :-:
disparity | const float* | input | integrated full-scale disparity data
width | int | input | image width (unit: pixels)
height | int | input | image height (unit: pixels)
baseline | float | input | camera parameters: baseline (unit: mm)
cx | int | input | camera parameters: x coordinate of optical center (unit: pixel)
xDistance | float* | output | full image distance in X direction

#### getWholeYDistance()
Get the distance (unit: mm) of all pixels in the whole image from the coordinate origin in the Y direction of the camera coordinate system.
Parameter Name | Type | Input/Output | Description
:-: | :-: | :-:
disparity | const float* | input | integrated full-scale disparity data
width | int | input | image width (unit: pixels)
height | int | input | image height (unit: pixels)
baseline | float | input | camera parameters: baseline (unit: mm)
cy | int | input | camera parameters: optical center y coordinate (unit: pixel)
yDistance | float* | output | full image distance in Y direction

#### getWholeZDistance()
Get the distance (unit: mm) of all pixels of the entire image in the Z direction of the camera coordinate system from the coordinate origin.
Parameter Name | Type | Input/Output | Description
:-: | :-: | :-: | :-:
disparity | const float* | input | integrated full-scale disparity data
width | int | input | image width (unit: pixels)
height | int | input | image height (unit: pixels)
baseline | float | input | camera parameters: baseline (unit: mm)
focus | float | input | camera parameters: focal length (unit: pixels)
zDistance | float* | output | full-scale image distance value pointer in Z direction

### RoadwayPainter class
#### imageGrayToRGB()
Convert the single-channel grayscale image to three-channel RGB for subsequent color effects of obstacles or lane lines.
Parameter Name | Type | Input/Output | Description
:-: | :-: | :-: | :-:
gray | const unsigned char* | input | single-channel grayscale image
rgb | const unsigned char* | output | three-channel RGB
width | int | input | image width (unit: pixels)
height | int | input | image height (unit: pixels)
#### paintRoadway()
This function will fuse the lane line information and modify the three-channel RGB image data to achieve the purpose of drawing the lane line information onto the image. After calling this function, the `_rgbImageData` data can be processed later. The return value is of type `bool`, indicating whether there are lane lines identified in the image.
Parameter Name | Type | Input/Output | Description
:-: | :-: | :-: | :-:
_roadwayParam | void* | input | lane line processing result data
_rgbImageData | unsigned char* | output | three-channel RGB image data
_width | int | input | image width (unit: pixels)
_height | int | input | image height (unit: pixels)
maskMode | bool | input | whether to draw in a mixed color mode (true: no false: yes)

### ObstaclePainter class
#### paintObstacle()
This function will fuse the obstacle information and modify the three-channel RGB image data to achieve the purpose of drawing the obstacle information onto the image. After calling this function, the `_rgbImageData` data can be processed later. The return value is of type `bool`, whether the drawing result is successful.
Parameter Name | Type | Input/Output | Description
:-: | :-: | :-: | :-:
_obstacleParam | void* | input | obstacle processing result data
_rgbImageData | unsigned char* | output | three-channel RGB image data
_width | int | input | image width (unit: pixels)
_height | int | input | image height (unit: pixels)
showDetials | bool | input | whether to show obstacle details
singleObs | bool | input | whether to draw only the nearest obstacles in the warning area

### YuvToRGB class
Determine the format used by the image data by judging the `format member` in the `RawImageFrame` structure, as defined in frameformat.h.
#### YCbYCr2Rgb()
Convert `YUV422` format image data to three-channel RGB data.
Parameter Name | Type | Input/Output | Description
:-: | :-: | :-: | :-:
src | const unsigned char* | input | YUV422 image data
dest | char* | output | three-channel RGB image data
width | int | input | image width (unit: pixels)
height | int | input | image height (unit: pixels)

#### YCbYCrPlannar2Rgb()
Convert `YUV422planar` format image data to three-channel RGB data.
Parameter Name | Type | Input/Output | Description
:-: | :-: | :-: | :-:
src | const unsigned char* | input | YUV422planar image data
dest | char* | output | three-channel RGB image data
width | int | input | image width (unit: pixels)
height | int | input | image height (unit: pixels)