#pragma once
#include <stdint.h>
#include "vision_service_direct_types_enum.h"

//I.########################Others->Vision########################

struct ImageFrame
{
	int rows;
	int cols;
	int chns;
	int step;
	//Keep same as opencv: CV_8U=0  CV_8S=1  CV_16U=2  CV_16S=3  CV_32S=4  CV_32F=5  CV_64F=6
	int type;
	char *data;
	int64_t timestamp;
	int64_t timestamp_trigger;
	MemoryType memory;
	ColorFormat color = COLOR_GRAY;
	//For digital zoom
	int roi_x = -1;
	int roi_y = -1;
	int roi_w = -1;
	int roi_h = -1;
	//For optical zoom
	float opt_scale = -1;
	//Reserved
	int flags = -1;
	char *data2;
};

struct MonoParam
{
	CalibMeans means;
	int rows;
	int cols;
	double srcParam[9];
	double dstParam[4];
	double errs[128];
};

struct BinoParam
{
	CalibMeans means;
	int srcRows;
	int srcCols;
	int dstRows;
	int dstCols;
	double srcParam1[9];
	double srcParam2[9];
	double dstParam1[4];
	double dstParam2[4];
	double trans[3];
	double rvec[3];
	double quat[4];//xyzw
	double rmat[9];
	double euler[3];
	double rvec1[3];
	double quat1[4];//xyzw
	double rmat1[9];
	double euler1[3];
	double rvec2[3];
	double quat2[4];//xyzw
	double rmat2[9];
	double euler2[3];
	double errs[128];
};

struct Cam2ImuParam
{
	CalibMeans means;
	double trans[3];
	double rvec[3];
	double quat[4];//xyzw
	double rmat[9];
	double euler[3];
	double errs[128];
};

struct FirmwareID
{
	RobotModel rmodel;
	char rcId[16];
	char fcsId[16];
	char gimbalId[16];
	char camVersion[4];
};

struct GimbalCamInfo 
{

};

//100HZ
struct AttFrame
{
	float imu_gyro[3];
	float imu_accel[3];
    float uav_quat[4];  // q_wxyz
    int64_t timestamp;
};

//50HZ
struct NedFrame
{
	float coord[3];
	float speed[3];
	//0:both invalid   1:coord valid   2:speed valid   3:both valid
	int valid;
	int64_t timestamp;
};

//5HZ
struct GpsFrame
{
	int gps_time;							//unit: s
	double gps_lat;							//unit: deg
	double gps_lon;							//unit: deg
	double gps_alt;							//unit: deg
	float ned_vx;							//unit: m/s
	float ned_vy;							//unit: m/s
	float ned_vz;							//unit: m/s
	float accuracy_hor;						//unit: m
	float accuracy_ver;						//unit: m
	float accuracy_vel;						//unit: m
	//0:no fix   1:dead reckoning only   2:2D fix   3:3D fix   4:GNSS+Dead Reckoning   5:Time only fix
	int GNSS_fix_type;
	int satellite_num;
	int64_t timestamp;
};

//100HZ
struct GimbalFrame
{
	int state;
	float rpy[3];						//from motor and valid in only one mode
	float speed[3];						//rad/s
	float quat[4];						//from fusion
	int64_t timestamp;
};

//20HZ
struct SonarFrame
{
	short state; 						//信号处理状态
	short amplitude;    				//回波幅值
	short precision;					//阈值等级/测量噪声估计
    int valid_distance;                 //有效测量距离，单位：毫米
    int64_t timestamp;
};

struct RadarFrame { int reservved[16]; };

struct LidarFrame { int reservved[16]; };

struct LEDFrame { int brightness; };

//10HZ
struct RCFrame
{
	short button;						//遥控器按键指令
	short mode;							//当前飞行器模式，1024：ATTI模式；1689：IOC模式；359：GPS模式
	short gimbal_pitch;					//云台pitch拨轮角度归一化值，[-100,100]
	short roll;							//roll角度归一化值，[-100,100]
	short pitch;						//pitch角度归一化值，[-100,100]
	short yaw;							//yaw角度归一化值，[-100,100]
	short thrust;						//油门归一化值，[-100,100]
	int64_t timestamp;   				//视觉时间戳，单位：ns
};

//5HZ
struct FSFrame
{
	uint8_t armed;
	uint8_t ground_condition;
	uint8_t battery_percentage;
	uint8_t flight_mode;				//见<<alink通信协议>> 表4.1.1.3 Flight mode定义
	int64_t timestamp;
};

//1HZ
struct FCHeartbeatFrame
{
	uint8_t type; 						//设备类型
	uint32_t alarm_status1; 				//告警状态1，见<<alink通信协议>> 表4.1.1.1 告警状态1比特定义
	uint32_t alarm_status2; 				//告警状态2，见<<alink通信协议>> 表4.1.1.2 告警状态2比特定义
	uint8_t flight_mode; 				//见<<alink通信协议>> 表4.1.1.3 Flight mode定义
};

//100HZ
struct FCVisionSwitchFrame
{
	bool enable_vio;
	bool enable_head_soa;
	bool enable_rear_soa;
	bool enable_bottom_soa;
	bool enable_right_soa;
	bool enable_left_soa;
	bool enable_top_soa;
	bool enable_prior_soa;
	bool enable_secure_landing;
	bool enable_precise_landing;
	bool enable_mapping_planning;
};

struct NoFlyFrame
{
	int64_t timestamp;
	int32_t max_height;
	int32_t max_heigth_limited;
	uint32_t status;
};

struct VisionMissionStatusFrame
{
	uint32_t timestamp;
	int32_t mission_type;
	uint32_t status1;
	uint32_t status2;
	uint32_t data_u32[4];
	uint8_t data_u8[4];
};

//II.########################Vision->Others########################

union FlightControlCMD
{
	union 
	{
		struct
		{
			float roll_angle_speed;
			float pitch_angle_speed;
			float yaw_angle_speed;
			float normalized_thrust;
		}thrust;
		struct
		{
			float roll_angle_speed;
			float pitch_angle_speed;
			float yaw_angle_speed;
			float vertical_velocity;
		}attitude;
		struct
		{
			float back_forward_velocity;
			float left_right_velocity;
			float vertical_velocity;
			float yaw_rate;
		}velocity;
	}param;
	int32_t data[4];
};
