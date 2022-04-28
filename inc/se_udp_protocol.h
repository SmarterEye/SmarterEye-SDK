#ifndef SE_UDP_PROTOCOL_H
#define SE_UDP_PROTOCOL_H
#include <stdint.h>

#pragma warning (disable:4200)

#define SE_BROADCAST_PORT      52500
#define SE_MONITOR_PORT        52600

enum SECommandType{
    SECommandType_Command,          /**< Packet for command data. The command is used to find/connect to the device. */
    SECommandType_ACK,              /**< Packet for perception data. */
    SECommandType_Message,          /**< Packet for status reporting from device . */
};
enum SECommandID{
    SECommandID_Sync,               /**< Try to connect to the device. */
    SECommandID_Broadcast,          /**< The device Broadcast it's IP/port and other information. */
    SECommandID_Query,              /**< Query the information from device. Not implement now. */
    SECommandID_Heartbeat,          /**< Heartbeat packet*/
    SECommandID_Disconnect,         /**< Disconnect from device. */
    SECommandID_RequirePerception,  /**< Require perception data from device. */
    SECommandID_RequireImage,       /**< Require image data from device. */
};

enum SEDataType {
    SEDataType_Perception,          /**< Perception data packet. */
    SEDataType_Image,               /**< Reserved for future. */
    SEDataType_IMU,                 /**< Reserved for future. */
};

enum SEWorkingState {
    SEWorkingState_Normal,
    SEWorkingState_Abnormal
};
#pragma pack(1)

typedef struct {
    uint8_t version;                /**< Packet protocol version. */
    uint8_t rsvd;                   /**< Reserved. */
    uint16_t length;                /**< Data length, not include the header. */
    uint16_t seq_num;               /**< Sequence number, reserved for future. */
    uint8_t cmd_type;               /**< One of SECommandType value. */
    uint8_t cmd_id;                 /**< One of SECommandID value. */
    uint8_t data[0];                /**< Command specificed data. */
} SECommandPacket;


/** data packet. */
typedef struct {
    uint16_t version;               /**< Packet protocol version. */
    uint16_t data_type;             /**< One of SEDataType value. */
    uint16_t length;                /**< Data length, not include the header. */
    uint16_t seq_num;               /**< Sequence number, reserved for future. */
    uint64_t timestamp;             /**< UTC format timestamp. */
    uint8_t data[0];                /**< Datatype specificed data. */
} SEDataPacket;

/**
 * Data format for SECommandID_Sync command.
 */
typedef struct {
    uint32_t ip_addr;               /**< IP address of the device. */
    uint16_t cmd_port;              /**< UDP port of the command connection. */
    uint16_t perception_port;       /**< UDP port of the peception result connection. */
    uint16_t image_port;            /**< UDP port of the image data connection. */
    uint16_t rsvd[5];
} SESyncRequest;

/** 
 * The information of broadcast device. Used by SECommandID_Broadcast command.
 */
typedef struct {
    uint32_t ip_addr;             /**< Device ip. */
    uint16_t dev_type;            /**< Device type, reserved for future. */
    char device_info[128];
} SEBroadcastDeviceInfo;

/**
 * The body of SECommandID_Heartbeat response.
 */
typedef struct {
    uint8_t ret_code;        /**< Return code. */
    uint8_t state;           /**< Working state. */
    uint16_t reserved;
    uint32_t error_code;     /**< Error code. */
} SEHeartbeatResponse;

/**
 * The body of SECommandID_Query response.
 */
typedef struct {
    uint8_t ret_code;        /**< Return code. */
    uint8_t state;           /**< Working state. */
    uint16_t len;
    char info[0];            /**< information string. */
} SEQueryResponse;

typedef enum {
    SEMotionState_Unknown = 0,
    SEMotionState_Moving = 1,
    SEMotionState_Stationary = 2
}SEMotionState;

typedef enum  {
    SEDataSource_Unknown =0,
    SEDataSource_Vision_Only = 1,
    SEDataSource_Radar_Olny = 2,
    SEDataSource_Vision_Radar = 3
}SEPerceptDataSource;

typedef struct {
    float x;
    float y;
} SEPoint2f;
typedef struct {
    float x;
    float y;
    float z;
} SEPoint3f;
typedef struct  {
  float type = 0.f;
  float conf = 0.f;
  SEPoint2f pt;
}SEKeyPoint2f;

typedef struct {
    float x;          // x coordinate of the top-left corner
    float y;          // y coordinate of the top-left corner
    float width;      // width of the rectangle
    float height;     // height of the rectangle
} SERect;

typedef struct {
    SEPoint2f lower_lt, lower_lb;
    SEPoint2f lower_rb, lower_rt;
    SEPoint2f upper_lt, upper_lb;
    SEPoint2f upper_rb, upper_rt;
}SEBox3D2f;
typedef struct {
    SEPoint3f lower_lt, lower_lb;
    SEPoint3f lower_rb, lower_rt;
    SEPoint3f upper_lt, upper_lb;
    SEPoint3f upper_rb, upper_rt;
}SEBox3D3f;
/**
 * Perception result data type.
 */
typedef enum {
    SEPercetionType_Obstacle,       /*Obstacle data*/
    SEPercetionType_Lane,           /*Lane data*/
    SEPercetionType_Freespace,      /**/
    SEPercetionType_TSR,            /*Traffic sign recognition data.*/
    SEPercetionType_TFL,            /*Traffic light data.*/
    SEPercetionType_Flatness,       /*Flatness data*/
} SEPercetionType;

/**
 * Data structure for perception result. There may be multi perception result in one data packet.
 */
typedef struct {
    uint16_t type;                  /*One of SEPercetionType value.*/
    uint16_t length;                /*Data length, include this header.*/
    char data[0];                   /*Type specificed data.*/
}SEPercetion;

/**
 * Data structure for one obstacle. 
 */
typedef struct {
//
    uint8_t valid;
    uint8_t source;
    uint8_t b_cipv_mcp;                     // obstacle cipv/mcp flag
    uint8_t replaced;

    uint32_t id;                        // obstacle id
    int32_t age;                        // track frames
    int32_t life_time;
    uint32_t type;                       // obstacle classification 0--VEHICLE_REAR, 1--VEHICLE_FULL, 2--PEDESTRIAN, 3--CYCLIST, 100--GENERAL

    float type_confidence;
    float obs_confidence;

    //imagespace infor
    float img_width_var = 0;
    float img_height_var = 0;
    float img_position_var[4] = { 0 };

    SERect img_rect;
    SEBox3D3f img_bbox;

    //worldspace infor
    int curr_lane;
    int traversable;

    float ttc;
    float hmw;
    float yaw;
    float width;
    float height;
    float length;
    float conf_yaw;
    float sigma_yaw;
    float sigma_width;
    float sigma_height;
    float sigma_length;
    float sigma_acc[4];
    float sigma_vel[4];
    float sigma_position[4];
    float vel[3];               // vehicle coordinate system,[0][1][2] save relative speed x/y/z;
    float acc[3];               // vehicle coordinate system,[0][1][2] save relative accleration x/y/z;
    SEPoint3f position;         // obstacle center position in the vehicle coordinate system

    //vehcle infor
    uint32_t veh_type;
    uint32_t veh_classification;
    uint32_t veh_light;
    SEKeyPoint2f key_point;

    //ped
    uint32_t ped_pose;
    uint32_t ped_age;
    uint32_t ped_pos_neg;
    uint32_t ped_occlusion;
    uint32_t ped_orientation;

    //cyclelist infor
    uint32_t cyc_orientation;
    uint32_t cyc_occlusion;
    uint32_t cyc_pos_neg;



}SEObstacle;

/**
 * Data structure for obstacle list. Used for SEPercetionType_Obstacle.
 */
typedef struct {
    uint64_t timestamp;             /**< UTC format timestamp. */
    int32_t obs_num;
    SEObstacle obstacles[0];
} SEObstacles;

/**
 * Data structure for one lane line. 
 */
typedef struct {
    int32_t id;
    int32_t life_time;
    float coeffs[4];
    SEPoint3f start_point;
    SEPoint3f end_point;
    int32_t type;
    int32_t color;
    int32_t source;
    float dist_to_front_wheel;
    float width;
//    int32_t type_sibling;
    float confidence;
    float sigma_coeffs[16];
    int32_t side_position;
}SELaneLine;

/**
 * Data structure for lane. used for SEPercetionType_Lane.
 */
typedef struct {
    uint64_t timestamp;
    uint32_t line_num;
    SELaneLine lines[0];
}SELane;

/**
 * Data structure for one point of freespace. 
 */
typedef struct {
    uint32_t pointType;
    float lateral;
    float longitudinal;
    float u;
    float v;
}SEFreeSpacePoint;

/**
 * Data structure for freespace. used for SEPercetionType_Freespace.
 */
typedef struct {
    uint64_t timestamp;
    uint32_t point_num;
    SEFreeSpacePoint points[0];
}SEFreeSpace;

/**
 * Data structure for one point of flatness.
 */
typedef struct {
    SEPoint2f leftWheelHightPoint;
    SEPoint2f rightWheelHightPoint;
}SEFlatnessPoint;

/**
 * Data structure for flatness. used for SEPercetionType_Flatness.
 */
typedef struct {
    uint64_t timestamp;
    float samplingInterval;
    uint32_t point_num;
    uint32_t type;
    float confidence;
    SEFlatnessPoint points[0];
}SEFlatness;

/**
 * Data structure for one traffic sign. 
 */
typedef struct {
    int32_t id;
    int32_t age;
    int32_t life_time;

    float confidence;
    uint32_t sign_type;

    SERect img_rect;

    float width;
    float height;
    SEPoint3f position;

}SETSRData;

/**
 * Data structure for traffic sign. used by SEPercetionType_TSR.
 */
typedef struct  {

    uint64_t timestamp;
    int32_t sign_num;
    SETSRData signs[];

}SETSR;

/**
 * Data structure for one traffic light. 
 */
typedef struct {
    int32_t id;
    int32_t age;
    int32_t life_time;

    float confidence;
    uint32_t color;
    uint32_t shapeType;

    SERect img_rect;

    float width;
    float height;
    SEPoint3f position;

}SETFLData;

/**
 * Data structure for traffic light. used for SEPercetionType_TFL.
 */
typedef struct {
    uint64_t timestamp;
    int32_t light_num;
    SETFLData lights[];
}SETFL;

struct ImageFreg
{
    uint16_t imageSeq;
    uint16_t total;
    uint32_t offset;
};

struct ImageFrame
{
    uint32_t dataSize;
    uint64_t  timestamp;
    uint16_t frameId;
    uint16_t format;
    uint16_t width;
    uint16_t height;
    uint8_t  image[0];
};

#pragma pack()

#endif // SE_UDP_PROTOCOL_H
