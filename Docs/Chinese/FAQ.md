# FAQ

### 版本更新 

|  上线日期  | 版本号 |    更新内容     |
| :--------: | :----: | :-------------: |
| 2022.05.20 |  v0.1  |      初版       |
| 2022.11.30 |  v0.2  | 增加imu相关说明 |

​                                           

### 开源SDK使用

1. SDK下载地址

   ```
   https://github.com/SmarterEye/SmarterEye-SDK 
   ```

2. SDK适用平台

   ```
   当前版本为开源SDK，提供全部源代码，适用于Windows10，Ubuntu18.04/20.04以及x64，ARM等多种系统环境进行测试和开发，可以根据不同的运行环境进行编译和运行
   ```

3. SDK 提供哪些相机功能接口

   ```
   SDK支持通过千兆网口获取原始彩色/灰度图像数据、视差数据、车道线和障碍物算法结果数据以及相机标定数据等，可以通过调用相关SDK接口实现二次开发
   ```

4. SDK Demo提供哪些功能

   ```
   SDK Demo当前实现显示或者保存左图/视差图、视差转深度图（显示）、视差转三维数据（保存）等功能，具体见SDK文档（<sdk>/Docs）
   ```

5. SDK运行环境配置

   ```
   参考SDK文档，位于SDK文件夹Docs ;如果没有相关的配置，则无法使用SDK
   ```

6. 如何编译生成SDK库

   ```
   使用Qt Creator打开SDK文件夹下release.pro文件，配置编译器，执行编译生成sdk demo运行依赖库，位于Runtime/Bin
   ```
   
7. 如何编译 SDK Demo

   ```
    Demo工程由Cmake构建，
   （1）Ubuntu下通过打开终端执行cmake .. & make；也可以使用支持Cmake的Qt Creator打开        CMakeLists编译，生成的可执行程序位于Runtime/Bin下，可以直接运行。
   （2）Window10 下可以通过cmake-gui软件配置生成Visual Studio工程文件，再通过VS编译，生成的可执行程序位于Runtime/Bin/Release（debug）下，需要拷贝至Bin下才可以执行，否则会报错（提示找不到动态库）；也可以使用支持Cmake的Qt Creator打开CmakeLists编译，生成的可执行程序位于Runtime/Bin下，可以直接运行。
   ```

8. Ubuntu下使用脚本编译SDK库和demo

   ```
   （1）执行脚本编译，进入scripts文件夹，打开终端执行./build.sh, <sdk>下生成SdkRelease_v0.x.x 及其压缩文件SdkRelease_v0.x.x.tar.gz; SdkRelease_v0.x.x/Runtime/Bin文件夹包含生成的sdk库和demo可执行程序
   （2）将库文件的路径添加到环境变量中，eg: export LD_LIBRARY_PATH=/home/zkhy/SmarterEye-SDK/SdkRelease_v0.5.1/Runtime/Bin
   ```

9. 编译PointCloudDemo报错

   ```
   （1）缺少SDK库,编译sdk依赖库
   （2）缺少PCL依赖库，安装VTK
   ```

10. 编译显示qt相关问题

    ```
      报错的原因是安装的Qt有问题，建议在CMakeLists里指定Qt的位置，添加：
      set(CMAKE_PREFIX_PATH /opt/Qt5.12.2/5.12.2/gcc_64)
    ```

11. 如何通过sdk获取右彩图

    ```
      当前相机设备不支持右彩图输出，默认输出左相机彩图，右相机灰度图；当前通过sdk只支持获取左图和视差 
      图，不支持获取右图
    ```



### 安装标定

1. FieldHelper工具软件的使用功能

   ```
   （1）安装标定
   （2）标准固件更新
   ```

2. 相机检测不到障碍物，或者检测的效果比较差，或者测距误差较大

   ```
   出现这种情况，首先可能是相机没有进行安装标定，再者可能是标定效果不好，建议重新规范标定，同时姿态学习的时候建议使用地面标定。标定方法可以参考产品安装手册。
   ```

3. SmarterEye或者FieldHelper无法连接相机设备

   ```
   （1）修改本地IP地址，使前三个字段与相机设备IP保持一致，最后一位不同
   （2）打开powershell，或者win+R，输入cmd，打开命令行，ping相机设备IP地址，如果ping的通，则表示pc与设备网络连接正常
   （3）打开FieldHelper，登录以后，点击”输入连接设备的IP地址“，输入设备IP地址
   （4）如果ping不通，考虑相机设备IP不正确，建议找回
   ```

4. FieldHelper安装配置显示没有障碍物过滤项

      ```
      账号权限问题，确保使用的账号是“安装人员”账号属性
      ```

5. FieldHelper工具软件更新固件显示网络连接出现故障

      ```
      （1）建议更新网络，确保网络连接稳定后软件重启
      （2）更换软件版本
      ```
      
6. 相机标定时不出现黄线或者靶标标定黄线在最上面

      ```
      电脑用户名为中文会出现此情况，原因为安装工具在标定的过程中依赖的一些文件放在以登录用户个人路径下，如果路径中有中文可能会导致资源寻找错误问题，需要更换一台英文用户名的电脑
      ```



### CAN/GPS/imu

1. 通过can输出哪些信息？

   ```
   相机设备通过can输出障碍物和车道线检测结果的相关数据，具体见can通讯协议。当前can通讯为单向传输模式，不支持请求-应答交互方式
   ```
   
2. 相机设备不发送can报文信息，或者只发送ID为0x79F报文信息

   ```
   （1）安装标定后，设备才会发送can报文信息
   （2）默认只发送0x79F，设备持续获取车辆信息（0x7B0）后，才会发送其他报文信息
   （3）相机设备遇到障碍物后，才会发送障碍物报文信息，报文信息格式具体格式见can协议文档
   ```

3. can报文格式高低位如何定义？

   ```
   can采用intel格式的编码方式，低字节在前，高字节在后，比如说对于can ID:0x7A1的报文信息，若0x123表示障碍物宽度，0x124表示障碍物高度，那么第2/3/4字节分别为0x23/0x41/0x12
   ```

4. 是否支持修改can通讯波特率 ？

   ```
   支持，FieldHelper（1.8.22版本及以后）工具软件在安装标定的时候，支持修改can通讯波特率。波特率支持设置为125Kbps，250Kbps，500Kbps，1000Kbps.
   ```

5. 相机设备imu坐标系xyz轴指向？

   ```
   部分相机设备包含imu（具体以产品使用说明为准），以相机的安装位置为参考，向前为+z，向右为+y，向上为+x；相机安装标定以后，通过运行<sdk> GetMotionDataDemo可以输出imu数据.
   ```

   

### 开源项目支持

   1. 当前SDK支持哪些开源项目?

      ```
支持Ubuntu下的ROS
      ```

2. 如何编译生成SDK库

   ```
   使用Cmake构建和编译，生成ROS wrapper依赖库,位于_output如果没有生成SDK库，在编译ros wrapper的时候会报错
   ```

3. 如何编译Ros wrapper

   ```
   在Ubuntu下，进入wrapper/ros, 打开终端执行catkin_make构建和编译,生成节点
   ```

4. Ubuntu下执行roscore报错

   ```
   报错的原因可能有
   （1）已经打开了一个终端，执行了roscore
   （2）当前路径含有中文，需要切换到纯英文路径
   （3）出现错误：Unable to contact my own server at...，打开~/.bashrc文件，添加或修改环境变量ROS_HOSTNAME和ROS_MASTER，即改为：
   export ROS_HOSTNAME=localhost
   export ROS_MASTER_URI=http://localhost:11311
    修改并保存~/.bashrc文件后，再重新启动下该文件，再运行roscore即可正常运行
   ```

5. Ubuntu下编译ros wrapper显示对cv::Mat::Mat()，或者cv::Mat::clone const 未定义的引用

   ```
   报错的原因是找不到Opencv函数库，检查Opencv的配置
   ```

6. Ubuntu下编译ros src显示CV_RGB2BGR未声明

   ```
     报错的原因是安装了opencv4，定位到对应的源代码所在的文件，添加头文件”#include <opencv2/imgproc/types_c.h>“
   ```

7. ros wrapper 产生哪些topic以及service

   ```
   编译成功后，可执行 roslaunch zkhy_stereo_d zkhy_stereo.launch 来使用wrapper程序，产生/zkhy_stereo/disparity等topic和/zkhy_stereo/get_camera_params等service，具体见sdk文档
   ```

8. Ubuntu下运行ros display.launch只显示左彩图和右灰度图

   ```
    当前相机只支持输出左彩图和右灰度图
   ```

   

### 相机功能与算法相关

1. 相机输出的图像像素格式

   ```
   左相机可输出YUV422Plannar格式的彩图，右相机输出Gray格式的灰度图，以及Disparity16/DisparityDens16格式的视差图
   ```

2. 相机的基线，焦距，帧率以及分辨率是多少？

   ```
   常规相机基线12cm，焦距4cm，帧率12.5fps，分辨率1280*720，不同机型配置参数不一定完全相同，具体参考技术手册
   ```

3. 如何通过相机获得检测物体像素点的距离值？

   ```
   可以参考SDK下 Disparity2Real3dDemo，这里提供了通过视差数据转换为三维数据方法，表示像素点在实际空间坐标，其中x表示横向距离，y表示垂直距离，z表示纵向距离
   ```

4. 如何获取相机障碍物以及车道线算法检测结果

   ```
   相机标定以后，可以输出算法检测结果
   （1）通过相机sdk接口可以获取，可以参考相关demo
   （2）通过can通讯获取，可以参考can通讯协议
   （3）外接小屏幕，直接显示输出结果
   ```

5. 面匹配与边缘匹配相机版本的区别

   ```
   面匹配后只能当作点云相机使用，无法运行我们障碍物算法，我们的算法只能运行在边缘匹配方式下
   ```

6. 通过sdk接口获取到的相机左图，与实时采集的时间差是多少?

   ```
   一般在200ms左右
   ```

   

   

     

     

     

     