#ifndef IMU_PARAMETERS_H
#define IMU_PARAMETERS_H

typedef struct{
    unsigned short int year;
    unsigned char month;
    unsigned char date;
    unsigned char hour;
    unsigned char min;
    unsigned char sec;
}UtcTime;

typedef struct {
    int work;
    int sonar_1;
    int sonar_2;
    int sonar_3;
    int sonar_4;
    int sonar_5;
    int sonar_6;
    int sonar_7;
    int sonar_8;
}SonarData;

typedef struct{
    int work;
}SonarState;

typedef struct{
    int clear;
}OdometerState;

typedef  struct{
    float pitch;
    float roll;
    float yaw;
    float linear_acc_x;
    float linear_acc_y;
    float linear_acc_z;
    float angular_vel_x;
    float angular_vel_y;
    float angular_vel_z;
//    float bar_altitude;   //unit : m
//    float magnetic_angle;
}IMUSensorData;

////NMEA 0183 协议解析后数据存放结构体
//typedef struct
//{
//    unsigned char svnum;					      //可见卫星数
//    nmea_slmsg slmsg[12];		//最多12颗卫星
//    nmea_utc_time utc;			//UTC时间
//    unsigned int latitude;				//纬度 分扩大100000倍,实际要除以100000
//    unsigned char nshemi;					//北纬/南纬,N:北纬;S:南纬
//    unsigned int longitude;		  //经度 分扩大100000倍,实际要除以100000
//    unsigned char ewhemi;					//东经/西经,E:东经;W:西经
//    unsigned char gpssta;					//GPS状态:0,未定位;1,非差分定位;2,差分定位;6,正在估算.
//    unsigned char posslnum;				//用于定位的卫星数,0~12.
//    unsigned char possl[12];				//用于定位的卫星编号
//    unsigned char fixmode;					//定位类型:1,没有定位;2,2D定位;3,3D定位
//    unsigned short int pdop;					  //位置精度因子 0~500,对应实际值0~50.0
//    unsigned short int hdop;					  //水平精度因子 0~500,对应实际值0~50.0
//    unsigned short int vdop;					  //垂直精度因子 0~500,对应实际值0~50.0

//    int altitude;			 	//海拔高度,放大了10倍,实际除以10.单位:0.1m
//    unsigned short int speed;					//地面速率,放大了1000倍,实际除以10.单位:0.001公里/小时
//}nmea_msg;

typedef  struct{
    UtcTime uct_time;
    unsigned char satellite_num;
    float altitude;     //unit : m
    float ground_speed;   //unit: m/s
    unsigned int latitude;         //纬度 分扩大100000倍,实际要除以100000
    unsigned char nshemi;	     //北纬/南纬,N:北纬;S:南纬
    unsigned int longitude;	     //经度 分扩大100000倍,实际要除以100000
    unsigned char ewhemi;	     //东经/西经,E:东经;W:西经
}GPSData;

typedef  struct {
    int intf; // 0(undefined),1(json),2(ros)
    int mode; // 0(ctrl), 1(trac)
//    int gear; // (0,1,2?)()
//    int pid; // 0(off), 1(on)
}INTFMode;

typedef struct{
    int plate_type; //0(unknow), 1(rov), 2(mobile)
    int imu_num; // 0(no)
    int track_num; //
    int ultra_num; //
    int arm_num; //
    int head_num; //
    int serv_num; //
}MODULEConfig;

#endif // IMU_PARAMETERS_H
