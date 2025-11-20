//
// Created by ljh on 2024/8/15.
//

/**@file LWD3Type.h
* 
* @brief Definition of 3D camera data types and other data types.
* 
* @copyright  Copyright(C)2024-2026 Percipio All Rights Reserved.
* 
**/

#ifndef LW_D3_C_TYPE_H
#define LW_D3_C_TYPE_H


#if defined(_WIN32)
    #ifdef LW_INTERFACE_EXPORT
        #define LW_API __declspec(dllexport)
    #else
        #define LW_API __declspec(dllimport)
    #endif
#else
    #define LW_API __attribute__((visibility("default")))
#endif

#ifdef __cplusplus
    #define LW_C_API extern "C" LW_API
#else
    #define LW_C_API LW_API
#endif


#include <cstdint>


//******************************************* 内部专用 *******************************************************************//
//#define LW_INTERNAL_API
#ifdef  LW_INTERNAL_API

/// @brief 文件类型。
enum LWFileType : uint32_t
{
    LW_DRNU_HIGH        = 0x00, ///< 高频DRNU标定文件。
    LW_DRNU_LOW         = 0x01, ///< 低频DRNU标定文件。
    LW_BENDING_LUT      = 0x02, ///< 弯曲标定文件。
    LW_DISTORTION_LUT   = 0x03, ///< 畸变标定文件。
    LW_OFFSET_COMP      = 0x04, ///< 平整度标定文件。
    LW_PALLET_ARG       = 0x05, ///< 托盘参数文件。

    LW_OTHER            = 0xf0  ///< 其它自定义文件。
};

/// @brief Binning模式。
enum LWBinningMode : uint32_t
{
    LW_1X1BINNING = 0x00,
    LW_2X2BINNING = 0x01,
    LW_4X4BINNING = 0x02
};

/// @brief 用于温度标定。
struct LWTemperatureParams
{
    double coef1 = 0;
    double coef2 = 0;
    double temp1 = 0;
    double temp2 = 0;
};
#endif //LW_INTERNAL_API


/// @brief 设备句柄描述符，前四个字节为远端IPv4地址后四个字节为本机IPv4地址。
typedef uint64_t LWDeviceHandle;

///@brief 函数返回码，用于告知函数的执行结果。
typedef enum : uint32_t
{
    LW_RETURN_OK                    = 0x00,	///< 执行成功。
    LW_RETURN_COMMAND_UNDEFINED     = 0x03,	///< 命令未定义。
    LW_RETURN_COMMAND_ERROR         = 0x04,	///< 命令结构错误
    LW_RETURN_ARG_OUT_OF_RANGE      = 0x05,	///< 参数设置超范围
    LW_RETURN_FILE_LENGTH_ERROR     = 0x06,	///< 文件大小与实际传输大小不一致
    LW_RETURN_FILE_MD5_ERROR        = 0x07,	///< 文件MD5校验失败

    LW_RETURN_TIMEOUT               = 0x20,	///< 执行超时。
    LW_RETURN_NETWORK_ERROR         = 0x21,	///< 网络错误，欲知详情请调用“LWGetReturnCodeDescriptor”函数。
    LW_RETURN_UNINITIALIZED         = 0x22, ///< SDK还未进行资源初始化（须调用“LWInitializeResources”函数来初始化资源）。
    LW_RETURN_UNOPENED              = 0x23, ///< 设备未打开（须调用“LWOpenDevice”函数来打开设备）。
    LW_RETURN_HANDLE_MISMATCH       = 0x24, ///< 传入的设备句柄无效，请检查该句柄是否是“LWFindDevices”函数调用返回的设备句柄。
    LW_RETURN_FILE_OPEN_ERROR       = 0x25, ///< 文件打开失败。
    LW_RETURN_NOT_SUPPORTED         = 0x26, ///< 当前设备尚不支持该功能。
    LW_RETURN_VERSION_ERROR         = 0x27, ///< 协议版本不匹配。
    LW_RETURN_OUT_OF_MEMORY         = 0x28, ///< 传入的数据缓存大小不足。
    LW_RETURN_TYPE_NOT_EXIST        = 0x29, ///< 类型错误，不存在该类型或是不支持该类型。
    LW_RETURN_TYPE_INPUT_ERROR      = 0x2a, ///< 数据类型错误，请传入正确类型的数据（例如：“LWSavePointCloudAsPCDFile”函数只能传入点云数据）。
    LW_RETURN_THREAD_QUIT_TIMEOUT   = 0x2b, ///< 线程退出超时。
    LW_RETURN_DATA_TYPE_MISMATCH    = 0x2c, ///< 无法获取该类型数据，请设置正确的数据接受类型。
    LW_RETURN_DATA_NOT_UPDATED      = 0x2d, ///< 数据接受缓存区未更新数据，在获取数据之前请先成功调用“LWGetFrameReady”函数。
    LW_RETURN_FILE_NOT_EXIST        = 0x2e, ///< 文件不存在。

    LW_RETURN_FIRMWARE_UPDATE_FAIL  = 0x30, ///< 设备固件更新失败。

    LW_RETURN_CUSTOM_ERROR          = 0xfa, ///< 自定义错误，欲知详情请调用“LWGetReturnCodeDescriptor”函数。

    LW_RETURN_UNDEFINED_ERROR       = 0xff, ///< 未定义错误。

} LWReturnCode;

/// @brief 数据帧类型。
typedef enum : uint32_t
{
    LW_TYPE_UNDEFINED       = 0B00000000,   ///< 类型未定义。

    LW_DEPTH_FRAME          = 0B00000001,   ///< 深度数据类型，单位：毫米。
    LW_AMPLITUDE_FRAME      = 0B00000010,   ///< 幅度数据类型。
    LW_IR_FRAME             = 0B00000100,   ///< IR数据类型。
    LW_POINTCLOUD_FRAME     = 0B00001000,   ///< 点云数据类型，z-轴数据对应深度值。
    LW_RGB_FRAME            = 0B00010000,   ///< RGB数据类型。
    LW_RGB_TO_DEPTH_FRAME   = 0B00100000,   ///< RGB数据映射到深度后的RGB数据类型，其分辨率对齐到深度数据（与深度数据的分辨率一致）。
    LW_DEPTH_TO_RGB_FRAME   = 0B01000000,   ///< 深度数据映射到RGB后的深度数据类型，其分辨率对齐到RGB数据（与RGB数据的分辨率一致）。
    LW_D2R_POINTCLOUD_FRAME = 0B10000000,   ///< 深度数据映射到RGB后的点云数据类型，z-轴数据对应深度值，其点数对齐到RGB像素数（与RGB数据的像素数一致）。

} LWFrameType;

/// @brief 数据帧的像素格式，用以正确的解析数据帧。
typedef enum : uint32_t
{
    LW_PIXEL_FORMAT_UCHAR       = 0x00,	///< 每像素为无符号字符型（unsigned char）。
    LW_PIXEL_FORMAT_USHORT      = 0x01,	///< 每像素为无符号短整型（unsigned short）。
    LW_PIXEL_FORMAT_RGB888      = 0x02,	///< 每像素为三通道无符号字符型(详见：“LWRGB888Pixel”结构体)。
    LW_PIXEL_FORMAT_VECTOR3F    = 0x03,	///< 每像素为三通道浮点型(详见：“LWVector3f”结构体)。

} LWPixelFormat;

/// @brief 传感器类型。
typedef enum : uint32_t
{
    LW_TOF_SENSOR = 0x01,	///< TOF传感器。
    LW_RGB_SENSOR = 0x02,	///< RGB传感器。

} LWSensorType;

/// @brief 曝光模式。
typedef enum : uint32_t
{
    LW_EXPOSURE_AUTO    = 0x00, ///< 自动曝光，由设备根据外部环境自行调整曝光时间。
    LW_EXPOSURE_MANUAL  = 0x01, ///< 手动曝光，以设定的曝光时间为准，不再进行自动调整。

} LWExposureMode;

/// @brief TOF传感器工作的频率模式。
typedef enum : uint32_t
{
    LW_FREQUENCY_DUAL   = 0x00, ///< 双频模式，用于远距离测距。
    LW_FREQUENCY_SINGLE = 0x01,	///< 单频模式，用于近距离测距。

} LWFrequencyMode;

/// @brief 设备的触发模式（工作模式），须在开启数据流之前进行设置。
typedef enum : uint32_t
{
    LW_TRIGGER_ACTIVE       = 0x00,	///< 主动模式（连续触发）。当开启数据流时，设备会按照指定帧率发送数据。
    LW_TRIGGER_SOFT         = 0x01,	///< 软触发模式。当开启数据流时，每调用一次“LWSoftTrigger”函数设备便会发送一帧数据。建议将帧率设置为最大帧率，至少不得低于触发的频率。
    LW_TRIGGER_HARD         = 0x02,	///< 硬触发模式。当开启数据流时，设备每检测到一次外部信号（电平信号）便会发送一帧数据。建议将帧率设置为最大帧率，至少不得低于触发的频率。
    LW_TRIGGER_HARD_FILTER  = 0x03,	///< 带滤波参数的硬触发模式（信号的持续时间和间隔）。根据设定的信号滤波参数，设备每检测到一次外部信号（电平信号）便会发送一帧数据。建议将帧率设置为最大帧率，至少不得低于触发的频率。

} LWTriggerMode;

/// @brief RGB传感器可发送的数据格式。
typedef enum : uint32_t
{
    LW_MJPEG        = 0x00, ///< JPG格式，由其自身的特性，它所占用的网络带宽很小，对设备帧率没影响。
    LW_YUV422_YUYV  = 0x01, ///< YUV格式，由于是无损压缩，它虽能完好的还原出原图像，但占用的网络带宽很大（相比JPG格式），且对设备的帧率影响很大（使得帧率无法超过5帧）。
    LW_YVU420_NV12  = 0x02, ///< YUV格式，由于是无损压缩，它虽能完好的还原出原图像，但占用的网络带宽很大（相比JPG格式），且对设备的帧率影响很大（使得帧率无法超过5帧）。

} LWRgbTransferFormat;

/// @brief 通用滤波参数结构。
typedef struct
{
    bool	enable;		///< 使能开关。
    int32_t	threshold;	///< 滤波阈值。
    int32_t	k1;	        ///< 扩展位，仅对部分滤波有效。
    int32_t	k2;	        ///< 扩展位，仅对部分滤波有效。

} LWFilterParam;

/// @brief 网络配置信息。
typedef struct
{
    char	type;			///< IPv4地址的类型, 0x00表示为动态IP地址, 0x01表示为静态IP地址。
    char    ip[32];			///< IPv4地址。
    char    netmask[32];	///< 子网掩码。
    char    gateway[32];	///< 网关（保留字段，无须关注）。
    char    mac[32];		///< MAC地址（保留字段，无须关注）。
    char    reserved[96];	///< 保留字段。

} LWNetworkInfo;

/// @brief 版本号信息。
typedef struct
{
    int32_t major;      ///< 主版本号。
    int32_t minor;      ///< 次版本号。
    int32_t patch;      ///< 修订版本号。
    int32_t reserved;   ///< 编译版本号。

} LWVersionInfo;

/// @brief 温度结构信息。
typedef struct
{
    float	laser1;	///< 激光器1的温度。
    float	laser2;	///< 激光器2的温度。
    float	chip;	///< 芯片的温度。

} LWTemperature;

/// @brief 时间戳结构信息。
typedef struct
{
    int64_t tv_sec;		///< 秒数。
    int64_t tv_usec;	///< 微秒数。

} LWTimeStamp;

/// @brief 相机内参结构信息。
typedef struct
{
    float fx;	///< x轴向上的焦距长度，它是相机焦距f在x方向上的像素表示，以像素为单位。
    float fy;	///< y轴向上的焦距长度，它是相机焦距f在y方向上的像素表示，以像素为单位。
    float cx;	///< 相机光轴（也称为主点或光心点）在图像坐标系中x轴上的偏移量，以像素为单位。
    float cy;	///< 相机光轴（也称为主点或光心点）在图像坐标系中y轴上的偏移量，以像素为单位。
    float k1;	///< 径向畸变系数1，主要的径向畸变系数。
    float k2;	///< 径向畸变系数2，更高阶的畸变系数，用于更精确地描述畸变情况。
    float k3;	///< 径向畸变系数3，更高阶的畸变系数，用于更精确地描述畸变情况。
    float p1;	///< 切向畸变系数1，用于描述由于镜头装配不完全对称导致的图像畸变。切向畸变表现为图像边缘的偏移。
    float p2;	///< 切向畸变系数2，用于描述由于镜头装配不完全对称导致的图像畸变。切向畸变表现为图像边缘的偏移。

} LWSensorIntrinsicParam;

/// @brief 相机外参结构信息。
typedef struct
{
    float rotation[3][3];	///< 旋转矩阵。
    float translation[3];	///< 平移矩阵。

} LWSensorExtrinsicParam;

/// @brief 点云的点信息，单位：毫米。
typedef struct
{
    float x;    ///< 点在三维空间坐标系里的x坐标值。
    float y;    ///< 点在三维空间坐标系里的y坐标值。
    float z;    ///< 点在三维空间坐标系里的z坐标值（相对于镜头平面的垂直距离即深度值）。

} LWVector3f;

/// @brief RGB图像的像素信息
typedef struct
{
    uint8_t r;	///< 红色分量值。
    uint8_t g;	///< 绿色分量值。
    uint8_t b;	///< 蓝色分量值。

} LWRGB888Pixel;

/// @brief 为了保持"LWFrameData"结构体一致性的情况下能够进行相应扩展，于是新增该“变体”结构体，后续版本的新增数据结构（必须与帧数据息息相关）均在此结构体里进行添加。
/// 注：仅添加与帧数据相关联的数据结构。
typedef struct
{


} LWVariant;

/// @brief 数据帧结构信息。
typedef struct
{
    uint16_t		width;			///< 帧数据的宽度（列数），由于数据是基于像素的，因此也可以看作是水平分辨率。
    uint16_t		height;			///< 帧数据的高度（行数），由于数据是基于像素的，因此也可以看作是垂直分辨率。
    uint32_t		frameIndex;		///< 帧数据的编号，它是从1开始递增的，每次新开启数据流时都会对其进行重置。
    uint32_t		bufferSize;		///< 帧数据域的大小（字节数），即“pFrameData”指针指向的内存区域大小。
    uint32_t		elemSize;		///< 帧数据里一个像素所占用的字节数，例如：深度数据是用“unsigned short”表示的，因此该值是2，对于点云数据则是12。
    uint32_t		total;		    ///< 帧数据总的像素数，即数据宽度与高度的乘积（width * height）。
    LWFrameType		frameType;		///< 帧数据的类型，详见“LWFrameType”所枚举的类型。
    LWPixelFormat	pixelFormat;	///< 帧数据的像素类型，详见“LWPixelFormat”所枚举的类型。
    LWTemperature   temperature;    ///< 设备采集该帧数据时，设备的温度信息（仅TOF数据附带温度信息，如果是RGB数据则请忽略）。
    LWTimeStamp		timestamp;		///< 帧数据的时间戳。
    char*           pFrameData;		///< 指向帧数据域的指针。注：在使用时请先转换为对应的数据类型，比如使用点云数据时则需进行类型转换---> auto ptr = (LWVector3f*)frameData.pFrameData。

    LWVariant*      pVariant;       ///< 指向“LWVariant”数据结构的指针。

} LWFrameData;

#endif //LW_D3_C_TYPE_H
