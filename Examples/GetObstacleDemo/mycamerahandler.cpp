#include <iostream>
#include <string>

#include "mycamerahandler.h"
#include "satpext.h"
#include "frameid.h"
#include "frameext.h"
#include "obstacleData.h"



MyCameraHandler::MyCameraHandler(std::string name):
    mName(name)
{}

void MyCameraHandler::handleRawFrame(const RawImageFrame *rawFrame)
{
    // put you image processing logic here.    eg:
    processFrame(rawFrame->frameId, (char*)rawFrame + sizeof(RawImageFrame),rawFrame->time,
                 rawFrame->width, rawFrame->height);

}

void MyCameraHandler::processFrame(int frameId, char *obstacleParam, int64_t time, int width, int height)
{
    switch(frameId)
    {
    case FrameId::Obstacle:
    {
         int blockNum = reinterpret_cast<int*>(obstacleParam)[0];
         printf("block num: %d\n",blockNum);
         if(blockNum>0){
              OutputObstacles *pOutputObstacles = reinterpret_cast<OutputObstacles*> (reinterpret_cast<int*>(obstacleParam) + 2);
              std::cout<<"timestamp: "<<time<<std::endl;
              printf("car speed: %3f\n",pOutputObstacles[0].currentSpeed);
              printf("frame rate: %3f\n",pOutputObstacles[0].frameRate);
              for(int i = 0; i < blockNum; i++)
              {
                  float distance = pOutputObstacles[i].avgDistanceZ;
                  printf("averageDistanceZ: %.2fm\n",distance);
                  switch(pOutputObstacles[i].classLabel) {
                  case 4:
                  {
                      printf("left continuous obstacle\n");
                  }
                      break;
                  case 5:
                  {
                      printf("right continuous obstacle\n");
                  }
                      break;
                  default:
                  {
                      printf("classify obstacle for front collision: ");
                      if(pOutputObstacles[i].stateLabel == 1)
                      {
                          printf("nearest obstacle in warning area\n");
                      }
                      else if (pOutputObstacles[i].stateLabel == 2) {
                          printf("other obstacles in warning area\n");
                      }
                      else if (pOutputObstacles[i].stateLabel == 3) {
                          printf("obstacles out of warning area\n");
                      }
                      printf("tracking obstacle ID : %d\n",pOutputObstacles[i].trackId);
                      printf("track frame numbers: %d\n",pOutputObstacles[i].trackFrameNum);
                      printf("obstacle width: %.2fm\n",pOutputObstacles[i].fuzzy3DWidth);
                      printf("obstacle height: %.2fm\n",(pOutputObstacles[i].real3DUpY - pOutputObstacles[i].real3DLowY));
                      printf("SpeedZ: %.2fm/s\n",pOutputObstacles[i].fuzzyRelativeSpeedZ);
                      printf("SpeedX: %.2fm/s\n",pOutputObstacles[i].fuzzyRelativeSpeedCenterX);
                      printf("TTCZ: %.2fs\n",pOutputObstacles[i].fuzzyCollisionTimeZ);
                      printf("DistZ: %.2fm\n",pOutputObstacles[i].fuzzyRelativeDistanceZ);
                      printf("obstacle type: ");
                      switch(pOutputObstacles[i].obstacleType)
                     {
                     case 1:
                         printf("vehicle\n");
                         break;
                     case 2:
                         printf("pedestrian\n");
                         break;
                     default:
                         printf("others\n");
                         break;
                     }

                  }
                      break;
                   }
                  printf("\n");
              }
          }

    }
        break;
    default:
        break;
    }

}
