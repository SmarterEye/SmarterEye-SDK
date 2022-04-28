#include <iostream>
#include <string>
#include <chrono>
#include <thread>
#include <memory>
#include "dataviewer.h"


int main(int argc, char *argv[])
{

    DataViewer *viewer;
    viewer = new DataViewer();

    while(1){
        std::this_thread::sleep_for(std::chrono::seconds(10));
    }

    std::cout << "stopped" << std::endl;

    return 0;
}
