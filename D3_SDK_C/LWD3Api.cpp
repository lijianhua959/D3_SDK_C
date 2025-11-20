//
// Created by ljh on 2024/8/15.
//
#include "LWD3Api.h"
#include "UpdateToolTask.h"

#include <fstream>
#include <vector>
#include <string>
#include <thread>
#include <chrono>
#include <mutex>
#include <atomic>
#include <condition_variable>
#include <map>
#include <unordered_set>
#include <openssl/evp.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>

#include "turbojpeg.h"

#if defined(_WIN32)
#include <WinSock2.h>
#include <WS2tcpip.h>
#pragma comment(lib, "Ws2_32.lib")
#define MSG_NOSIGNAL (0)
#define PATH_SEPARATOR '\\'
#elif defined(__linux__)
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <ifaddrs.h>
#include <arpa/inet.h>
#define SOCKET int
#define INVALID_SOCKET (-1)
#define SOCKET_ERROR   (-1)
#define PATH_SEPARATOR '/'
#endif


///< 循环队列最大节点数
#define LOOP_QUEUE_SIZE 5
///< 控制端口的端口号
#define COMMAND_PORT 50660
///< 数据端口的端口号
#define DATA_PORT 50661
///< 控制端口的数据接收缓冲区大小
#define COMMAND_MAX_SIZE 512
///< 命令控制协议版本
#define PROTOCOL_VERSION 0x02
///< 数据映射线程数
#define THREAD_POOL_SIZE 6
///< RGBD投影冗余区域数据过滤阈值[mm]
#define RGBD_PROJ_REDUNDANCY 25
///< RGB传感器的垂直方向像素数。
#define RGB_MAX_PIX_ROWS 1944
///< RGB传感器的水平方向像素数。
#define RGB_MAX_PIX_COLS 2592
///< RGB传感器总像素数。
#define RGB_MAX_PIX_NUMBER  RGB_MAX_PIX_ROWS*RGB_MAX_PIX_COLS
///< The maximum value of the z-coordinate of the 3D point cloud.
#define PCD_MAX_VALUE 60000
///< The vertical pixel count of the TOF sensor.
#define TOF_MAX_PIX_ROWS 480
///< The horizontal pixel count of the TOF sensor.
#define TOF_MAX_PIX_COLS 640
///< The total number of pixels of the TOF sensor.
#define TOF_MAX_PIX_NUMBER  TOF_MAX_PIX_ROWS*TOF_MAX_PIX_COLS
// 日志文件的最大尺寸（字节）
#define LOG_FILE_MAX_SAVE_SIZE 100 * 1024 * 1024
///< 命令集
#define C_Start							0x0000
#define C_Stop							0x0001
#define C_SoftTrigger					0x0002
#define C_Discovery						0x0003
#define C_SaveConfig					0x0004
#define C_DeleteConfig					0x0005
#define C_RecoveryDefaultConfigure		0x0006
#define C_DropTest						0x00fc
#define C_StartEntrucking				0x00fd
#define C_StopEntrucking				0x00fe
#define C_DeviceReboot					0x00ff
#define C_SetDataAddress				0x0100
#define C_SetTriggerMode				0x0101
#define C_SetHDR						0x0102
#define C_SetFrameRate					0x0103
#define C_SetIntegralTime				0x0104
#define C_SetTimeFilter					0x0105
#define C_SetFlyPixeFilter				0x0106
#define C_SetConfidenceFilter			0x0107
#define C_SetKalmanFilter				0x0108
#define C_SetSpatialFilter				0x0109
#define C_SetNetworkModel				0x010a
#define C_HeartbeatAddress				0x010b
#define C_SetRecvDataType				0x010c
#define C_SetRecvDataRows				0x010d
#define C_SetTimeMedianFilter			0x010e
#define C_SetIRGMMGain					0x0110
#define C_SetExposureValue				0x0111
#define C_SetMeanFilter					0x0112
#define C_SetFrequencyModel				0x0113
#define C_SetSingleIntegral				0x011f
#define C_SetBinning					0x0120
#define C_SetTempCompensate				0x0121
#define C_SetCameraIntrinsicArg			0x0122
#define C_SetSN							0x0123
#define C_SetDRNU						0x0124
#define C_SetLaserWorkFrequency			0x0125
#define C_SetDistortion					0x0126
#define C_SetTempCoefficient			0x0127
#define C_SetDelayHardTriggerTime		0x0128
#define C_SetRgbCameraExtrinsicArg		0x0129
#define C_SetRgbCameraIntrinsicArg		0x012a
#define C_SetRgbCameraExposureMode		0x012b
#define C_SetRgbCameraExposureTime		0x012c
#define C_SetRgbCameraBrightness		0x012d
#define C_SetRgbCameraContrastRatio		0x012e
#define C_SetRgbCameraResolutionRatio	0x012f
#define C_SetRgbCameraDataFormat		0x0130
#define C_SetRgbCameraGain				0x0132
#define C_SetRgbCameraGamma				0x0133
#define C_SetCameraNumber				0x013f
#define C_SetLaserEnableStatus			0x0140
#define C_GetDeviceStatus				0x0200
#define C_GetVersion					0x0301
#define C_GetTimeInfo					0x0302
#define C_GetFrameRate					0x0303
#define C_GetTriggerMode				0x0304
#define C_GetHDR						0x0305
#define C_GetIntegralTime				0x0306
#define C_GetTimeFilter					0x0307
#define C_GetFlyPixeFilter				0x0308
#define C_GetConfidenceFilter			0x0309
#define C_GetKalmanFilter				0x030a
#define C_GetSpatialFilter				0x030b
#define C_GetNetworkModel				0x030c
#define C_GetIntegralModel				0x030d
#define C_GetTimeMedianFilter			0x030e
#define C_GetDeviceType					0x030f
#define C_GetIRGMMGain					0x0310
#define C_GetTimeStamp					0x0311
#define C_GetFrequencyModel				0x0313
#define C_GetBinning					0x0320
#define C_GetTempCompensate				0x0321
#define C_GetTofCameraIntrinsicArg		0x0322
#define C_GetRgbCameraExtrinsicArg		0x0329
#define C_GetRgbCameraIntrinsicArg		0x032a
#define C_GetRgbCameraExposureMode		0x032b
#define C_GetRgbCameraExposureTime		0x032c
#define C_GetRgbCameraBrightness		0x032d
#define C_GetRgbCameraContrastRatio		0x032e
#define C_GetDelayHardTriggerTime		0x032f
#define C_GetRecvDataRows				0x0325
#define C_GetChipTemperature			0x0326
#define C_GetSN							0x0327
#define C_GetImageInfo					0x0328
#define C_GetRgbCameraResolutionRatio	0x0330
#define C_GetRgbCameraDataFormat		0x0331
#define C_GetRgbCameraGain				0x0332
#define C_GetRgbCameraGamma				0x0333
#define C_GetCameraNumber				0x033f
#define C_GetLaserEnableStatus			0x0340
#define C_FileArgTransfer				0x0500
#define C_FileDataTransfer				0x0501
#define C_FileAckTransfer				0x0502
#define C_OperateCommand				0x0f00

// 日志写入宏定义。
#define LOG_ERROR_OUT(...)	gLogger.writeLog(" ERROR ", __FILE__, __FUNCTION__, __LINE__, __VA_ARGS__);
#define LOG_INFO_OUT(...)	gLogger.writeLog(" INFO ", __FILE__, __FUNCTION__, __LINE__, __VA_ARGS__);

/// ******************************************* 内部工具函数 *******************************************


// 计算文件MD5哈希值的函数。
static LWReturnCode calculateMD5(const std::string& fileName, unsigned char* md5, unsigned int& md5_len)
{
	// 打开文件
	std::ifstream file(fileName, std::ios::binary | std::ios::in);
	if (!file.is_open()) return LW_RETURN_FILE_OPEN_ERROR;

	// 创建上下文
	EVP_MD_CTX* mdctx = EVP_MD_CTX_new();
	EVP_DigestInit_ex(mdctx, EVP_md5(), NULL);
	char buffer[10240];
	while (file.good())
	{
		file.read(buffer, 10240);
		EVP_DigestUpdate(mdctx, buffer, file.gcount());
	}
	file.close();

	// 计算并获取结果
	EVP_DigestFinal_ex(mdctx, md5, &md5_len);

	// 清理并释放资源
	EVP_MD_CTX_free(mdctx);

	return LW_RETURN_OK;
}

// 截取完整路径中的文件名。
static std::string getFileName(const char* str)
{
	// 截取文件名
	int begin = 0;
	int len = 0;
	for (int i = 0, n = strlen(str); i < n; ++i)
	{
		if ((str[i] == '\\') || (str[i] == '/'))
		{
			begin = i + 1;
			len = 0;
		}
		else {
			++len;
		}
	}

	return std::string(str + begin, len);
}

// 将传入的字节流转换为十六进制字符串。
static std::string bytesToHexString(const void* data, uint32_t length, char ch = '\0')
{
    if (data == nullptr) return "";
    auto bytes = (const uint8_t*)data;
    std::string buff = "0x";
    int32_t high;
    int32_t low;

    for (uint32_t i = 0; i < length; i++)
    {
        if (i != 0 && ch != '\0') buff += ch;
        high = bytes[i] / 16;
        low = bytes[i] % 16;
        buff += char((high < 10) ? ('0' + high) : ('a' + high - 10));
        buff += char((low < 10) ? ('0' + low) : ('a' + low - 10));
    }

    return buff;
}

// 当特定的套接字函数指示发生了错误时，应立即调用此函数以检索失败的函数调用的扩展错误代码描述信息。
static std::string getNetworkLastError()
{

#if defined(_WIN32)
    auto error_code = WSAGetLastError();
	if (error_code == 0) return "";
    char error[256];
    FormatMessageA(FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_ARGUMENT_ARRAY, NULL, error_code, NULL, error, 255, NULL);
    return error;
#else
	if (errno == 0) return "";
    return strerror(errno);
#endif

}

// 设置网络数据接收超时。
static bool setNetworkTimeout(SOCKET& sck, uint32_t t)
{

#if defined(_WIN32)
    return setsockopt(sck, SOL_SOCKET, SO_RCVTIMEO, (const char*)(&t), sizeof(t)) == 0;
#else
    timeval tv{ t / 1000, t % 1000 * 1000 };
    return setsockopt(sck, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) == 0;
#endif

}

// 关闭套接字。
static bool closeSocket(SOCKET& sck)
{
    if (sck == INVALID_SOCKET) return true;

#if defined(_WIN32)
    if (closesocket(sck) == SOCKET_ERROR) return false;
#else
    if (shutdown(sck, SHUT_RDWR) == SOCKET_ERROR || close(sck) == SOCKET_ERROR) return false;
#endif

    sck = INVALID_SOCKET;
    return true;
}

// 建立TCP连接。
static bool connectToServer(SOCKET& sck, sockaddr_in& addr, uint32_t timeout, bool alive = true)
{
    timeval tv{ timeout / 1000U, timeout % 1000U * 1000U };
	fd_set writefds;
    FD_ZERO(&writefds);
    FD_SET(sck, &writefds);

#if defined(_WIN32)
	// 设为非阻塞模式
    u_long iMode = 1;
    if (ioctlsocket(sck, FIONBIO, &iMode) == SOCKET_ERROR) return false;
	// TCP连接
    connect(sck, (sockaddr*)&addr, sizeof(addr));
	// 等待连接直至超时
    auto ret = select(0, NULL, &writefds, NULL, &tv);
    if (ret == 0 || ret == SOCKET_ERROR) return false;
	// 设回阻塞模式
    iMode = 0;
    if (ioctlsocket(sck, FIONBIO, &iMode) == SOCKET_ERROR) return false;
	// 心跳检测
    if (alive)
    {
        // 启用自动心跳检测机制
        BOOL    keep_alive = 1;
        setsockopt(sck, SOL_SOCKET, SO_KEEPALIVE, (const char*)&keep_alive, sizeof(keep_alive));
        // 首次检测的时间间隔
        DWORD   keep_idle = 1;
        setsockopt(sck, IPPROTO_TCP, TCP_KEEPIDLE, (const char*)&keep_idle, sizeof(keep_idle));
        // 每次检测的时间间隔
        DWORD   keep_interval = 1;
        setsockopt(sck, IPPROTO_TCP, TCP_KEEPINTVL, (const char*)&keep_interval, sizeof(keep_interval));
        // 检测失败的最大次数
        DWORD   keep_count = 2;
        setsockopt(sck, IPPROTO_TCP, TCP_KEEPCNT, (const char*)&keep_count, sizeof(keep_count));
    }

#else
	// 设为非阻塞模式
	auto flags = fcntl(sck, F_GETFL, 0);
	if (fcntl(sck, F_SETFL, flags | O_NONBLOCK) < 0) return false;
	// TCP连接
	if (connect(sck, (sockaddr *)&addr, sizeof(addr)) == SOCKET_ERROR)
	{
		if (errno != EINPROGRESS) return false;
		// 等待连接直至超时
		auto rc = select(sck + 1, NULL, &writefds, NULL, &tv);
		if (rc <= 0)
		{
			// 超时或错误
			if (rc == 0) errno = ETIMEDOUT;
			return false;
		}
		// 检查套接字错误
		int error;
		socklen_t len = sizeof(error);
		if (getsockopt(sck, SOL_SOCKET, SO_ERROR, &error, &len) == SOCKET_ERROR) return false;
		if (error != 0)
		{
			// 连接失败
			errno = error;
			return false;
		}
	}
	// 恢复阻塞模式（可选）
	if (fcntl(sck, F_SETFL, flags) < 0) return false;
	// 心跳检测
    if (alive)
    {
        // 启用心跳检测机制
        int keep_alive = 1;
        setsockopt(sck, SOL_SOCKET, SO_KEEPALIVE, (void*)&keep_alive, sizeof(keep_alive));
        // 首次检测的时间间隔
        int keep_idle = 1;
        setsockopt(sck, SOL_TCP, TCP_KEEPIDLE, (void*)&keep_idle, sizeof(keep_idle));
        // 每次检测的时间间隔
        int keep_interval = 1;
        setsockopt(sck, SOL_TCP, TCP_KEEPINTVL, (void*)&keep_interval, sizeof(keep_interval));
        // 检测失败的最大次数
        int keep_count = 2;
        setsockopt(sck, SOL_TCP, TCP_KEEPCNT, (void*)&keep_count, sizeof(keep_count));
    }

#endif

    return true;
}

// 获取本机所有IP地址并建立对应的具有广播功能的套结字。
std::map<uint32_t, SOCKET> gSocketMap;
static bool getLocalAddrInfo(std::vector<sockaddr_in>& sockaddr_list)
{
	int bOpt = true;

#if defined(_WIN32)
    char  hostname[255];
    if (gethostname(hostname, sizeof(hostname)) == SOCKET_ERROR) return false;

    struct addrinfo hints;
    struct addrinfo* result = NULL;
    ZeroMemory(&hints, sizeof(hints));
    hints.ai_family = AF_INET;
    if (getaddrinfo(hostname, NULL, &hints, &result) != 0) return false;
    for (addrinfo* ptr = result; ptr != NULL; ptr = ptr->ai_next)
    {
		auto local_addr = *((sockaddr_in*)(ptr->ai_addr));
		local_addr.sin_port = 0;
		sockaddr_list.push_back(local_addr);
		if (gSocketMap.find(local_addr.sin_addr.s_addr) == gSocketMap.end())
		{
			auto _socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
			if ((bind(_socket, (sockaddr*)(&local_addr), sizeof(local_addr)) != SOCKET_ERROR)
				&& (setsockopt(_socket, SOL_SOCKET, SO_BROADCAST, (char*)&bOpt, sizeof(bOpt)) != SOCKET_ERROR))
			{
				gSocketMap[local_addr.sin_addr.s_addr] = _socket;
			}
		}
    }
    if (result) freeaddrinfo(result);
#else
    auto loop_ip = htonl(INADDR_LOOPBACK);
    struct ifaddrs* ifhead = nullptr;
    if (getifaddrs(&ifhead) == SOCKET_ERROR) return false;
    for (ifaddrs* ifpoint = ifhead; ifpoint != nullptr; ifpoint = ifpoint->ifa_next)
    {
        if (ifpoint->ifa_addr == nullptr) continue;
//#if defined(__aarch64__)
//        if (ifpoint->ifa_name[0] != 'e') continue;
//#endif
        if (ifpoint->ifa_addr->sa_family == AF_INET)
        {
            auto local_addr = *(sockaddr_in*)ifpoint->ifa_addr;
            if (local_addr.sin_addr.s_addr != loop_ip)
            {
				local_addr.sin_port = 0;
                sockaddr_list.push_back(local_addr);
				if (gSocketMap.find(local_addr.sin_addr.s_addr) == gSocketMap.end())
				{
					auto _socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
					if ((bind(_socket, (sockaddr*)(&local_addr), sizeof(local_addr)) != SOCKET_ERROR)
						&& (setsockopt(_socket, SOL_SOCKET, SO_BROADCAST, (char*)&bOpt, sizeof(bOpt)) != SOCKET_ERROR))
					{
						gSocketMap[local_addr.sin_addr.s_addr] = _socket;
					}
				}
            }
        }
    }
    if (ifhead) freeifaddrs(ifhead);

#endif

    return true;

}

// 创建RGB相机的畸变校准表（由算法提供）。
static void CreateRGBCameraCalibrationTable(const LWSensorIntrinsicParam& rgb_intrinsic, float d2rScale, cv::Size size_, cv::Mat& rgbCalibMap1, cv::Mat& rgbCalibMap2, cv::Mat& rMatrixR2D, cv::Mat& tMatrixR2D, cv::Mat& rMatrixD2R, cv::Mat& tMatrixD2R)
{
    // 内参矩阵初始化-用于rgb图像畸变校正
    cv::Mat camera_matrix = cv::Mat(3, 3, CV_32F, cv::Scalar::all(0));
    camera_matrix.at<float>(0, 0) = rgb_intrinsic.fx;
    camera_matrix.at<float>(0, 2) = rgb_intrinsic.cx;
    camera_matrix.at<float>(1, 1) = rgb_intrinsic.fy;
    camera_matrix.at<float>(1, 2) = rgb_intrinsic.cy;
    camera_matrix.at<float>(2, 2) = 1.0f;

    //径向畸变参数-用于rgb图像畸变校正
    cv::Mat dist_matrix = cv::Mat(1, 5, CV_32F, cv::Scalar::all(0));
    dist_matrix.at<float>(0, 0) = rgb_intrinsic.k1;
    dist_matrix.at<float>(0, 1) = rgb_intrinsic.k2;
    dist_matrix.at<float>(0, 2) = 0.0f;
    dist_matrix.at<float>(0, 3) = 0.0f;
    dist_matrix.at<float>(0, 4) = rgb_intrinsic.k3;

    //重构新内参
    cv::Mat new_camera_matrix = cv::getOptimalNewCameraMatrix(camera_matrix, dist_matrix, size_, 0, size_, nullptr);

    // 得到重构后保留FOV的映射表 每个像素下标用32位浮点型盛放
    cv::initUndistortRectifyMap(camera_matrix, dist_matrix, cv::Mat(), new_camera_matrix, size_, CV_32F, rgbCalibMap1, rgbCalibMap2);

    // 外参矩阵初始化及赋值—旋转矩阵&平移矩阵
    rMatrixR2D = new_camera_matrix * rMatrixR2D;
    tMatrixR2D = new_camera_matrix * tMatrixR2D;

    //
    new_camera_matrix.at<float>(0, 0) /= d2rScale;
    new_camera_matrix.at<float>(0, 2) /= d2rScale;
    new_camera_matrix.at<float>(1, 1) /= d2rScale;
    new_camera_matrix.at<float>(1, 2) /= d2rScale;

    rMatrixD2R = new_camera_matrix * rMatrixD2R;
    tMatrixD2R = new_camera_matrix * tMatrixD2R;
}

// 网络异常检测回调函数。
void(*networkAbnormalCallback)(LWDeviceHandle handle, const char* error, void* pUserData) = nullptr;
void* pUserData1 = nullptr;

// 帧数据刷新回调函数。
void(*frameReadyCallback)(LWDeviceHandle handle, void* pUserData) = nullptr;
void* pUserData2 = nullptr;


/// ******************************************* 内部数据结构类型 *******************************************

#if defined(_WIN32)
/// @brief 通过进程启动对 Winsock DLL 的使用。
class WinsockDllAutoManager
{
public:
	WinsockDllAutoManager()
	{
		WSADATA wsaData;
		auto ret = WSAStartup(MAKEWORD(2, 2), &wsaData);
		if (ret != 0)
		{
			char error[256];
			FormatMessageA(FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_ARGUMENT_ARRAY, NULL, ret, NULL, error, 255, NULL);
			throw std::runtime_error(error);
		}
	}

	~WinsockDllAutoManager()
	{
		WSACleanup();
	}

private:

} gWinsockDllAutoManager;
#endif

/// @brief 用于浮点数的大小端转换。
union FloatUint32 {
	float f;
	uint32_t u;
};

/// @brief 日志类，以本地网络通信的方式进行日志异步写入操作。
class Logger
{
public:
	Logger(Logger&& obj) = delete;
	Logger(const Logger& obj) = delete;
	Logger& operator=(Logger&& obj) = delete;
	Logger& operator=(const Logger& obj) = delete;

	Logger()
	{
		// 打开日志文件
		mFileHandle.open("logger.txt", std::ios::app);
		if (mFileHandle.fail()) throw std::runtime_error("日志文件打开失败！");
		// 获取日志文件大小
		struct stat file_info {};
		stat("logger.txt", &file_info);
		bufLen = file_info.st_size;
		// 初始化套接字
		sendSocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
		recvSocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
		// 设置接收地址为本机回环地址
		recvAddr.sin_family = AF_INET;
		recvAddr.sin_addr.s_addr = inet_addr("127.0.0.1");
		recvAddr.sin_port = 0;
		// 绑定地址
		socklen_t addrlen = sizeof(sockaddr_in);
		if ((bind(recvSocket, (sockaddr*)&recvAddr, sizeof(recvAddr)) != SOCKET_ERROR)
			&& (getsockname(recvSocket, (sockaddr*)&recvAddr, &addrlen) != SOCKET_ERROR))
		{
			std::thread{ &Logger::writeFile, this }.detach();
			std::this_thread::sleep_for(std::chrono::milliseconds(20));
		}
		else
		{
			// 获取日期和时间
			std::stringstream ss;
			auto t0 = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
			ss << std::put_time(localtime(&t0), "%Y-%m-%d %H:%M:%S");

			mFileHandle << ss.str() << " ERROR " << getNetworkLastError();
		}
	}

	~Logger() 
	{
		closeSocket(sendSocket);
		closeSocket(recvSocket);

		std::this_thread::sleep_for(std::chrono::milliseconds(10));
		mFileHandle.close();
	}

	// 写日志
	void writeLog(
		const char* loglevel,	// Log级别
		const char* fileName,	// 函数所在文件名
		const char* function,	// 函数名
		int lineNumber,			// 行号
		const char* format,		// 格式化
		...)					// 变量
	{
		logBuffer.clear();

		// 获取日期和时间
		std::stringstream ss;
		auto t0 = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
		ss << std::put_time(localtime(&t0), "%Y-%m-%d %H:%M:%S");
		logBuffer += ss.str();

		//	日志级别
		logBuffer += loglevel;

		// 日志辅助信息
		logBuffer += std::string(strrchr(fileName, PATH_SEPARATOR) + 1) + " " + function + " " + std::to_string(lineNumber) + " ";

		// 日志正文
		if (format[0] != '\0')
		{
			va_list ap;
			va_start(ap, format);
			vsnprintf(logInfo, 1024, format, ap);
			va_end(ap);

			logBuffer += std::string(logInfo);
		}

		// 写入日志内容
		sendto(sendSocket, logBuffer.c_str(), logBuffer.size(), 0, (sockaddr*)&recvAddr, sizeof(recvAddr));
	}

private:
	void writeFile()
	{
		char buffer[1024];
		decltype(recvfrom(recvSocket, nullptr, 0, 0, nullptr, nullptr)) ret = 0;
		
		while (true)
		{
			ret = recvfrom(recvSocket, buffer, 1024, 0, nullptr, nullptr);
			if (ret < 1) break;

			mFileHandle << std::string(buffer, ret) << std::endl;
			mFileHandle.flush();

			bufLen += ret;
			if (bufLen > LOG_FILE_MAX_SAVE_SIZE)
			{
				mFileHandle.close();
				if (rename("logger.txt", "logger_old.txt") == -1) throw std::runtime_error("日志文件重命名失败！");

				mFileHandle.open("logger.txt", std::ios::ate);
				if (mFileHandle.fail()) throw std::runtime_error("日志文件打开失败！");

				bufLen = 0;
			}
		}
		
		mFileHandle.close();
	}

	// 
	sockaddr_in	recvAddr;
	SOCKET		sendSocket = INVALID_SOCKET;
	SOCKET		recvSocket = INVALID_SOCKET;
	// 日志正文信息
	char logInfo[1024]{};
	// 存储log的buffer
	std::string logBuffer;
	// 文件句柄
	std::ofstream mFileHandle;
	// 日志尺寸
	size_t bufLen = 0;

} gLogger;


/// @brief 用于设备初始化时的临时授权自动管理，因为设备初始化会调用一系列与设备相关的API，所以需要在某个API出现错误的时候能够恢复初始状态（之所以这样做是避免在每个错误点执行相同的错误处理，如果那样的话会成倍的增加重复代码，显得代码结构很臃肿）。
class ProvisionalAuthority
{
public:
	explicit ProvisionalAuthority(std::atomic<bool>& val) : flag(val)
	{
		flag.store(true);
	}

	~ProvisionalAuthority()
	{
		if(isRestore) flag.store(false);
	}

	void maintain() 
	{
		isRestore = false;
	}

private:
	bool	isRestore = true;
	std::atomic<bool>&	flag;
};

/// @brief 用于简单的自动内存管理。
class AutoMemoryManager
{
public:
	explicit AutoMemoryManager(int len) : bufLen(len)
	{
		buffer = new char[len];
	}

	~AutoMemoryManager()
	{
		 delete[] buffer;
	}

	int size() const
	{
		return bufLen;
	}

	char* data()
	{
		return buffer;
	}

	unsigned char* u_data()
	{
		return (unsigned char*)buffer;
	}

private:
	int   bufLen = 0;
	char* buffer = nullptr;

};

/// @brief 线程计数器，主要用于保证所有线程能够正常退出。
class ThreadCounter
{
public:
	explicit ThreadCounter(std::atomic<int>& _count) : count(_count)
	{
		count.fetch_add(1);
	}

	~ThreadCounter()
	{
		count.fetch_sub(1);
	}

private:
	std::atomic<int>& count;

};

/// @brief 数据元节点。
class DataNode
{
public:
	explicit DataNode(int len) : size(len)
	{
		buffer = new char[len];
		variant = new LWVariant;
	}

	~DataNode()
	{
		delete[] buffer;
		delete variant;
	}

	unsigned char* data()
	{
		return (unsigned char*)buffer;
	}

	DataNode(DataNode&& obj) = delete;
	DataNode(const DataNode& obj) = delete;
	DataNode& operator=(const DataNode& obj) = delete;


public:
	bool			update = false;
	uint32_t	    mark = 0;			///< 用于后续数据对齐的查询标志。
	uint32_t		serial = 0;			///< 帧序号。
	uint32_t		size = 0;			///< 帧数据大小。
	LWTemperature	temperature{};		///< 设备温度信息（仅TOF数据携带此信息）。
	LWTimeStamp		time{};				///< 帧数据的时间戳。
	LWVariant*		variant = nullptr;	///< 帧数据的扩展部分。

private:
	char* buffer = nullptr;				///< 帧数据的数据域首地址。

};

/// @brief 非线程安全循环队列。注：由于读、写线程的执行机制，该队列不会出现由同步而引发的问题。
class LoopQueueNCP
{
public:
	LoopQueueNCP(int _len, int _size) : size(_size)
	{
		for (int i = 0; i < size; i++) data[i] = new DataNode(_len);
	}

	~LoopQueueNCP()
	{
		for (int i = 0; i < size; i++) delete data[i];
	}

	/// @brief 重置队列数据。
	void clear()
	{
		writeIndex = 0;
		writeCounter = 0;

		for (int i = 0; i < size; i++)
		{
			data[i]->update = false;
			data[i]->serial = 0;
		}
	}

	/// @brief 入队（实际上是与指定节点进行替换）。
	/// @param[in,out] obj 替换值
	void enqueue(DataNode*& obj)
	{
		writeIndex = writeCounter++ % size;
		obj->update = true;
		std::swap(data[writeIndex], obj);
	}

	/// @brief 获取最新节点。
	/// @return 最新节点
	DataNode* latestNode() const
	{
		data[writeIndex]->mark = writeIndex;
		return data[writeIndex];
	}

	/// @brief 出队（实际上是与指定节点进行替换）。
	/// @param[in] index 节点索引
	/// @param obj 替换对象
	void dequeue(uint32_t index, DataNode*& obj)
	{
		obj->update = false;
		std::swap(data[index], obj);
	}

	/// @brief 根据序列号查找对应的节点。
	/// @param num 序列号
	/// @return 节点指针
	DataNode* find(uint32_t num) const
	{
		for (int32_t i = 0, k = int32_t(writeIndex) - 1; i < size; ++i, --k)
		{
			if (k < 0) k = size - 1;
			if (data[k]->serial == num)
			{
				data[k]->mark = k;
				return data[k]->update ? data[k] : nullptr;
			}
		}

		return nullptr;
	}

private:
	int32_t		size = 0;
	uint32_t	writeIndex = 0;
	uint32_t	writeCounter = 0;
	DataNode* data[LOOP_QUEUE_SIZE]{};
};

/// @brief 用于收发的命令数据帧结构。
class CommandFrame
{
public:
	explicit CommandFrame(uint16_t _command = 65535) {
		_command = htons(_command);
		memcpy(buffer + 6, &_command, 2);
	}

	/// @brief 数据长度不得大于 COMMAND_MAX_SIZE。
	/// @param _buf 
	/// @param _len 
	CommandFrame(char* _buf, uint16_t _len) {
		if(_len > COMMAND_MAX_SIZE)  _len = COMMAND_MAX_SIZE;
		memcpy(buffer, _buf, _len);
	}

	CommandFrame& operator=(const CommandFrame& obj) {
		if (this != &obj) memcpy(buffer, obj.buffer, COMMAND_MAX_SIZE);

		return *this;
	}

	/// @brief 设置命令的版本协议。
	/// @param[in] version 协议版本
	void setVersion(uint8_t version) {
		buffer[4] = version;
	}

	/// @brief 获取命令的版本协议。
	/// @return 协议版本
	uint8_t getVersion() {
		return buffer[4];
	}

	/// @brief 设置命令类别。
	/// @param[in] type 类别
	void setCommandType(uint8_t type) {
		buffer[5] = type;
	}

	/// @brief 获取命令类别。
	/// @return 类别
	uint8_t getCommandType() {
		return buffer[5];
	}

	/// @brief 设置命令码。
	/// @param[in] command 命令码
	void setCommand(uint16_t command) {
		command = htons(command);
		memcpy(buffer + 6, &command, 2);
	}

	/// @brief 获取命令码。
	/// @return 命令码
	uint16_t getCommand() {
		return ntohs(*(uint16_t*)(buffer + 6));
	}

	/// @brief 设置命令的总分包数。
	/// @param[in] number 总包数
	void setTotalSerialNumber(uint16_t number) {
		number = htons(number);
		memcpy(buffer + 8, &number, 2);
	}

	/// @brief 获取命令的总分包数。
	/// @return 总包数
	uint16_t getTotalSerialNumber() {
		return ntohs(*(uint16_t*)(buffer + 8));
	}

	/// @brief 设置当前的命令分包号。
	/// @param[in] _number 分包号
	void setCurrentSerialNumber(uint16_t _number) {
		_number = htons(_number);
		memcpy(buffer + 10, &_number, 2);
	}

	/// @brief 获取当前命令的分包号。
	/// @return 分包号
	uint16_t getCurrentSerialNumber() {
		return ntohs(*(uint16_t*)(buffer + 10));
	}

	/// @brief 设置命令的数据域，数据长度不得大于 COMMAND_MAX_SIZE。
	/// @param[in] _data 数据域地址
	/// @param[in] _length 数据的长度
	void setArgField(const void* _data, uint16_t _length) {
		auto length = htons(_length);
		memcpy(buffer + 12, &length, 2);
		if (_data != nullptr) memcpy(buffer + 14, _data, _length);
		memcpy(buffer + 14 + _length, footer, 4);
	}

	/// @brief 获取命令的数据域。
	/// @return 数据域地址
	char* getArgField() {
		return (char*)(buffer + 14);
	}

	/// @brief 获取数据域的长度。
	/// @return 数据域长度
	uint16_t getArgFieldLength() {
		return ntohs(*(uint16_t*)(buffer + 12));
	}

	/// @brief 获取命令的总长度。
	/// @return 命令长度
	uint16_t size() {
		return getArgFieldLength() + 18;
	}

	/// @brief 判断命令是否是指定的命令。
	/// @param[in] _command 命令码
	/// @return 真值
	bool isCommand(uint16_t _command) {
		return getCommand() == _command
			&& memcmp(buffer, header, 4) == 0
			&& memcmp(buffer + 14 + getArgFieldLength(), footer, 4) == 0;
	}

	/// @brief 判断其是否是命令格式。
	/// @return 真值
	bool isCommandFormat() {
		return (memcmp(buffer, header, 4) == 0) && (memcmp(buffer + 14 + getArgFieldLength(), footer, 4) == 0);
	}

	/// @brief 获取命令地址。
	/// @return 命令地址
	char* data() {
		return (char*)buffer;
	}

	/// @brief 
	/// @return 命令帧的缓存区大小
	int32_t maxBuffer()
	{
		return COMMAND_MAX_SIZE;
	}

protected:
	uint8_t   header[4] = { 0xA5, 0x5A, 0xAA, 0x55 }; ///< 帧协议头
	uint8_t   footer[4] = { 0x5A, 0xA5, 0x55, 0xAA }; ///< 帧协议尾
	uint8_t   buffer[COMMAND_MAX_SIZE] = {
			0xA5, 0x5A, 0xAA, 0x55,     //0  帧头
			PROTOCOL_VERSION,           //4  协议版本
			0x00,                       //5  命令类别
			0x00, 0x00,                 //6  命令码
			0x00, 0x01,                 //8  总分包数
			0x00, 0x01,                 //10 当前包号
			0x00, 0x00,                 //12 数据域长度
			0x5A, 0xA5, 0x55, 0xAA      //14 帧尾
	};

};

/// @brief 设备句柄，用于一系列设备操作。
class DeviceHandle
{
public:
	explicit DeviceHandle(LWDeviceHandle _handle);
	DeviceHandle(DeviceHandle&& obj) = delete;
	DeviceHandle(const DeviceHandle& obj) = delete;
	DeviceHandle& operator=(DeviceHandle&& obj) = delete;
	DeviceHandle& operator=(const DeviceHandle& obj) = delete;
	~DeviceHandle();

    LWReturnCode OpenDevice();
	LWReturnCode CloseDevice();
	LWReturnCode ReconnectDevice(uint32_t msec);
	LWReturnCode RebootDevice();
	LWReturnCode SaveConfigureInfo();
	LWReturnCode RemoveConfigureInfo();
	LWReturnCode RestoreFactoryConfigureInfo();
	LWReturnCode StartStream();
	LWReturnCode StopStream();
	LWReturnCode HasRgbModule(bool& value);
	LWReturnCode SoftTrigger();
	LWReturnCode SetTriggerMode(LWTriggerMode mode);
	LWReturnCode GetTriggerMode(LWTriggerMode& mode);
	LWReturnCode SetExposureMode(LWSensorType sensorType, LWExposureMode mode);
	LWReturnCode GetExposureMode(LWSensorType sensorType, LWExposureMode& mode);
	LWReturnCode SetFrequencyMode(LWFrequencyMode mode);
	LWReturnCode GetFrequencyMode(LWFrequencyMode& mode);
	LWReturnCode SetHDRModeEnable(bool enable);
	LWReturnCode GetHDRModeEnable(bool& enable);
	LWReturnCode SetTransformRgbToDepthEnable(bool enable);
	LWReturnCode SetTransformDepthToRgbEnable(bool enable);
	LWReturnCode SetFrameRate(int32_t value);
	LWReturnCode GetFrameRate(int32_t& value);
	LWReturnCode SetExposureTime(LWSensorType sensorType, const int32_t* exposureTimeArray, int32_t arraySize);
	LWReturnCode GetExposureTime(LWSensorType sensorType, int32_t* exposureTimeArray, int32_t arraySize, int32_t& filledCount);
	LWReturnCode SetResolution(LWSensorType sensorType, int32_t width, int32_t height);
    LWReturnCode GetResolution(LWSensorType type, int32_t& width, int32_t& height);
	LWReturnCode SetTimeFilterParams(const LWFilterParam& param);
    LWReturnCode GetTimeFilterParams(LWFilterParam& param);
	LWReturnCode SetFlyingPixelsFilterParams(const LWFilterParam& param);
    LWReturnCode GetFlyingPixelsFilterParams(LWFilterParam& param);
	LWReturnCode SetConfidenceFilterParams(const LWFilterParam& param);
    LWReturnCode GetConfidenceFilterParams(LWFilterParam& param);
	LWReturnCode SetSpatialFilterParams(const LWFilterParam& param);
    LWReturnCode GetSpatialFilterParams(LWFilterParam& param);
	LWReturnCode SetTimeMedianFilterParams(const LWFilterParam& param);
    LWReturnCode GetTimeMedianFilterParams(LWFilterParam& param);
	LWReturnCode SetIRGMMGain(int32_t value);
    LWReturnCode GetIRGMMGain(int32_t& gain);
	LWReturnCode SetRgbSensorGain(int32_t value);
	LWReturnCode GetRgbSensorGain(int32_t& value);
	LWReturnCode SetRgbSensorGamma(int32_t value);
	LWReturnCode GetRgbSensorGamma(int32_t& value);
	LWReturnCode SetRgbSensorBrightness(int32_t value);
	LWReturnCode GetRgbSensorBrightness(int32_t& value);
	LWReturnCode SetRgbSensorContrastRatio(int32_t value);
	LWReturnCode GetRgbSensorContrastRatio(int32_t& value);
	LWReturnCode SetNetworkInfo(const LWNetworkInfo& info);
	LWReturnCode GetNetworkInfo(LWNetworkInfo& info);
	LWReturnCode SetRgbDataTransportFormat(LWRgbTransferFormat format);
	LWReturnCode GetRgbDataTransportFormat(LWRgbTransferFormat& format);
	LWReturnCode SetDeviceNumber(int32_t value);
	LWReturnCode GetDeviceNumber(int32_t& value);
	LWReturnCode SetHardTriggerFilterParams(int32_t t1, int32_t t2);
	LWReturnCode GetHardTriggerFilterParams(int32_t& t1, int32_t& t2);
	LWReturnCode GetIntrinsicParam(LWSensorType sensorType, LWSensorIntrinsicParam& intrinsicParam);
	LWReturnCode GetExtrinsicParam(LWSensorType sensorType, LWSensorExtrinsicParam& extrinsicParam);
	LWReturnCode GetDeviceSN(char* _sn_, int32_t bufferLen);
	LWReturnCode GetDeviceType(char* type, int32_t bufferLen);
	LWReturnCode GetTimeStamp(LWTimeStamp& t);
	LWReturnCode GetDeviceVersion(LWVersionInfo& fv, LWVersionInfo& dv);
	LWReturnCode GetFrameReady();
	LWReturnCode UpdateFirmware(const char* filename);

#ifdef LW_INTERNAL_API
	LWReturnCode SendFile(const char* fullname, LWFileType type);
	LWReturnCode SetDeviceSN(const char* _sn_, int size);
	LWReturnCode SendOperateCommand(const char* comstr, int size);
	LWReturnCode SetBinningMode(LWBinningMode mode);
	LWReturnCode SetDRNU(bool enable);
	LWReturnCode SetDistortionCalibration(LWSensorType sensorType, bool enable);
	LWReturnCode SetLaserWorkFrequency(const uint8_t* arr, int size);
	LWReturnCode SetAutoExposureDefaultValue(uint16_t val);
	LWReturnCode SetIntrinsicParam(LWSensorType sensorType, const LWSensorIntrinsicParam& para);
	LWReturnCode SetExtrinsicParam(LWSensorType sensorType, const LWSensorExtrinsicParam& para);
	LWReturnCode SetTemperatureCompensation(bool enable);
	LWReturnCode GetTemperatureCompensation(bool& enable);
	LWReturnCode SetTemperatureParams(const LWTemperatureParams& val);
	LWReturnCode SetLaserEnableStatus(uint32_t flag);
	LWReturnCode GetLaserEnableStatus(uint32_t& flag);
	LWReturnCode SetDataSyncEnable(bool enable);
#endif //LW_INTERNAL_API

private:
	void commandRecvThread();
	void rawDataRecvThread();
	void tofDataHandleThread();
	void rgbDataHandleThread();
	void dataSyncThread();
	void rgbToDepthThread(int threadID, int srcStartRow, int srcEndRow);
	void depthToRgbThread(int threadID, int srcStartRow, int srcEndRow);

	LWReturnCode ExecuteCommand(CommandFrame& command);


public:
    bool				isPro = false;
	bool				isReady = false;
	bool				rgbDistortionEnable = true;
	bool				dataSyncEnable = true;

	std::atomic<bool>	isR2DEnable{ false };
	std::atomic<bool>	isD2REnable{ false };
	std::atomic<bool>	hasRgbModule{ false };

	int32_t			tofWidth = TOF_MAX_PIX_COLS;
	int32_t			tofHeight = TOF_MAX_PIX_ROWS;
	int32_t			tofPixels = TOF_MAX_PIX_NUMBER;
	int32_t			rgbWidth = RGB_MAX_PIX_COLS;
	int32_t			rgbHeight = RGB_MAX_PIX_ROWS;
	int32_t			rgbPixels = RGB_MAX_PIX_NUMBER;
	uint32_t		timeout = 3000;
	int32_t			irGMMGain = 255;
	int32_t		    r2dRunCount = 0;
	int32_t		    d2rRunCount = 0;
	uint32_t		r2dFlag = 0;
	uint32_t		d2rFlag = 0;
	uint32_t		r2dArgRenew = 0;
	uint32_t		d2rArgRenew = 0;

	SOCKET			broadcastSocket = INVALID_SOCKET;

    sockaddr_in		remoteAddr{};
    sockaddr_in		localAddr{};
    sockaddr_in		recvAddr{};

	LWSensorIntrinsicParam	tofInArg{};
	LWSensorIntrinsicParam	rgbInArg{};
	LWSensorExtrinsicParam	rgbOutArg{};

    std::string     sn;
    std::string     describe;

	DataNode*		tofNode;
	DataNode*		rgbNode;
	DataNode*		tofCutNode;
	DataNode*		rgbCutNode;

	AutoMemoryManager pDepth{ TOF_MAX_PIX_NUMBER * 2 };
	AutoMemoryManager pAmp{ TOF_MAX_PIX_NUMBER * 2 };
	AutoMemoryManager pGra{ TOF_MAX_PIX_NUMBER };
	AutoMemoryManager pPot{ TOF_MAX_PIX_NUMBER * 12 };
	AutoMemoryManager pTPot{ RGB_MAX_PIX_NUMBER * 12 };
	AutoMemoryManager pRgb{ RGB_MAX_PIX_NUMBER * 3 };
	AutoMemoryManager pR2D{ TOF_MAX_PIX_NUMBER * 3 };

	std::mutex				socketMutex;
	std::condition_variable	socketNotify;
	std::mutex				r2dMutex;
	std::condition_variable r2dNotify;
	std::mutex				d2rMutex;
	std::condition_variable d2rNotify;

	cv::Mat         d2rOutputImg;
	cv::Mat			d2rInputImg;

private:
	std::atomic<bool>	aliveEnable{false};
	std::atomic<bool>	openEnable{ false };
	std::atomic<bool>	connectEnable{false};
	std::atomic<bool>	tofReadEnable{false};
	std::atomic<bool>	rgbReadEnable{false};
	std::atomic<bool>	readEnable{false};
	std::atomic<bool>	readyEnable{false};
	std::atomic<bool>	isCommandReply{false};

	CommandFrame		commandFrame;

	std::atomic<int>	threadCounter{0};

    float               d2rScale    = 2.5f;

	LWFilterParam		spatialFilter{ true, 3 };
	LWFilterParam		timeFilter{ false, 3 };
	LWFilterParam		timeMedianFilter{ false, 5, 100};
	LWFilterParam		flyingPixelsFilter{ true, 5 };
	LWFilterParam		confidenceFilter{ true, 5 };
	LWDeviceHandle      handle = 0;
	LWTriggerMode       triggerMode = LW_TRIGGER_ACTIVE;
	LWRgbTransferFormat rgbTFormat = LWRgbTransferFormat::LW_MJPEG;
	LWTimeStamp		    rgbTimeStamp{};
	LWTimeStamp		    tofTimeStamp{};

	DataNode*		tofRecvNode;
	DataNode*		rgbRecvNode;
	DataNode*		tofSwapNode;
	DataNode*		rgbSwapNode;
	DataNode*		tofHandleNode;
	DataNode*		rgbSrcNode;
	DataNode*		rgbDstNode;

	LoopQueueNCP	tofDataReadQueue{ TOF_MAX_PIX_NUMBER * 4, 4 };
	LoopQueueNCP	rgbDataReadQueue{ RGB_MAX_PIX_NUMBER * 3, 4 };

	SOCKET			commandSocket	= INVALID_SOCKET;
	SOCKET			dataSocket		= INVALID_SOCKET;

	cv::Mat			rMatrixR2D{ cv::Size(3, 3), CV_32FC1 };
	cv::Mat			tMatrixR2D{ cv::Size(1, 3), CV_32FC1 };
	cv::Mat			rMatrixD2R{ cv::Size(3, 3), CV_32FC1 };
	cv::Mat			tMatrixD2R{ cv::Size(1, 3), CV_32FC1 };
	cv::Mat			rgbCalibMap1;
	cv::Mat			rgbCalibMap2;

	std::mutex				nrMutex;

	std::mutex				tofWriteMutex;
	std::condition_variable tofWriteNotify;
	std::mutex				rgbWriteMutex;
	std::condition_variable rgbWriteNotify;
	std::mutex				dataReadMutex;
	std::condition_variable dataReadNotify;
	std::mutex				syncDataMutex;
	std::condition_variable syncDataNotify;

};

/// @brief 用于全局变量的自动化管理。
class AutoInitGlobalResources
{
public:
	AutoInitGlobalResources()
	{
		initEnable.store(false);
	}

	~AutoInitGlobalResources()
	{

	}

public:
	std::atomic<bool>	initEnable;
	std::string			errorInfo;

	std::map<std::string, DeviceHandle*>	strDeviceMap;
	std::map<LWDeviceHandle, DeviceHandle*> numDeviceMap;
	

} gGlobal;

DeviceHandle::DeviceHandle(LWDeviceHandle _handle) : handle(_handle)
{
	aliveEnable.store(true);

	tofNode			= new DataNode{ TOF_MAX_PIX_NUMBER * 4 };
	rgbNode			= new DataNode{ RGB_MAX_PIX_NUMBER * 3 };
	tofRecvNode		= new DataNode{ TOF_MAX_PIX_NUMBER * 4 };
	rgbRecvNode		= new DataNode{ RGB_MAX_PIX_NUMBER * 3 };
	tofSwapNode		= new DataNode{ TOF_MAX_PIX_NUMBER * 4 };
	rgbSwapNode		= new DataNode{ RGB_MAX_PIX_NUMBER * 3 };
	tofHandleNode	= new DataNode{ TOF_MAX_PIX_NUMBER * 4 };
	rgbSrcNode		= new DataNode{ RGB_MAX_PIX_NUMBER * 3 };
	rgbDstNode		= new DataNode{ RGB_MAX_PIX_NUMBER * 3 };
	tofCutNode		= new DataNode{ TOF_MAX_PIX_NUMBER * 4 };
	rgbCutNode		= new DataNode{ RGB_MAX_PIX_NUMBER * 3 };

	std::thread{ &DeviceHandle::commandRecvThread, this }.detach();
	std::thread{ &DeviceHandle::rawDataRecvThread, this }.detach();
	std::thread{ &DeviceHandle::tofDataHandleThread, this }.detach();
	std::thread{ &DeviceHandle::rgbDataHandleThread, this }.detach();
	std::thread{ &DeviceHandle::dataSyncThread, this }.detach();
}

DeviceHandle::~DeviceHandle()
{
	openEnable.store(false);
	aliveEnable.store(false);
	connectEnable.store(false);
	isR2DEnable.store(false);
	isD2REnable.store(false);
	closeSocket(commandSocket);
	closeSocket(dataSocket);
	std::this_thread::sleep_for(std::chrono::milliseconds(10));

	r2dNotify.notify_all();
	d2rNotify.notify_all();
	socketNotify.notify_all();
	tofWriteNotify.notify_all();
	rgbWriteNotify.notify_all();
	dataReadNotify.notify_all();
	std::this_thread::sleep_for(std::chrono::milliseconds(15));

	auto t = std::chrono::steady_clock::now();
	while (threadCounter.load() > 0)
	{
		if (timeout < std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - t).count())
		{
			LOG_ERROR_OUT("<%s>, Thread exit timeout.", sn.c_str());
			break;
		}
	}

	delete tofNode;
	delete rgbNode;
	delete tofRecvNode;
	delete rgbRecvNode;
	delete tofSwapNode;
	delete rgbSwapNode;
	delete tofHandleNode;
	delete rgbSrcNode;
	delete rgbDstNode;
	delete tofCutNode;
	delete rgbCutNode;
}

void DeviceHandle::commandRecvThread()
{
	ThreadCounter _tc_{ threadCounter };

	while (aliveEnable.load())
	{
		if (recv(commandSocket, commandFrame.data(), COMMAND_MAX_SIZE, 0) > 0)
		{
			if (commandFrame.isCommandFormat())
			{
				isCommandReply.store(true);
				socketNotify.notify_all();
			}

			continue;
		}
		if (!aliveEnable.load()) break;

		std::unique_lock<std::mutex>	lock{ socketMutex };
		if (connectEnable.load())
		{
			connectEnable.store(false);
			auto einfo = getNetworkLastError();

			if (networkAbnormalCallback != nullptr)
				networkAbnormalCallback(handle, einfo.c_str(), pUserData1);

			LOG_ERROR_OUT("<%s>, The network connection has been disconnected.---%s", sn.c_str(), einfo.c_str());
		}
		socketNotify.wait(lock, [this] {	return  connectEnable.load() || !aliveEnable.load(); });
	}
}

void DeviceHandle::rawDataRecvThread()
{
	ThreadCounter _tc_{ threadCounter };

	uint32_t			magic = 2774182485U;
	uint32_t			serial{};
	LWTimeStamp			time{};
	AutoMemoryManager	recvBuf{ 18000000 };

	auto*	recv_buf	= recvBuf.data();
	auto*	u_recv_buf	= recvBuf.u_data();

    decltype(recv(0, nullptr, 0, 0)) recv_len	= 0;
	decltype(recv(0, nullptr, 0, 0)) read_len	= 0;
	decltype(recv(0, nullptr, 0, 0)) frame_len	= 0;
	decltype(recv(0, nullptr, 0, 0)) data_len	= 0;

	bool	determine	= true;
	while (aliveEnable.load())
	{
		data_len	= 0;
		determine	= true;
		
		while (true)
		{
			recv_len = recv(dataSocket, recv_buf + data_len, 65536, 0);
			if (recv_len > 0)
			{
				if (determine)
				{
					if ((magic != *(uint32_t*)recv_buf))
					{
						if(data_len != 0) data_len = 0;
						continue;
					}

					// 获取帧数据基本信息
					time.tv_sec		= *(uint32_t*)(recv_buf + 6);
					time.tv_usec	= *(uint32_t*)(recv_buf + 12);
					serial			= *(uint32_t*)(recv_buf + 32);
					// 帧数据长度
					read_len = ntohl(*(uint32_t*)(recv_buf + 28));
					// 帧长度
					frame_len = read_len + 60;
					// 判定数据解析是否正常
					if (frame_len > 11000000)
					{
						if (data_len != 0) data_len = 0;
						continue;
					}
					// RGB数据传输格式解析
					if ((u_recv_buf[5] == 0x06) 
						&& (u_recv_buf[4] > 0x02)
						&& (rgbTFormat != LWRgbTransferFormat(u_recv_buf[25])))
					{
						rgbTFormat = LWRgbTransferFormat(u_recv_buf[25]);
					}

					determine = false;
				}

				data_len += recv_len;
				if (data_len < frame_len) continue;

				// 数据类型处理
				if (u_recv_buf[5] == 0x05)
				{
					memcpy(tofRecvNode->data(), recv_buf + 56, read_len);
					tofRecvNode->temperature.laser1 = float(u_recv_buf[21] << 8 | u_recv_buf[22]) / 10.0f;
					tofRecvNode->temperature.laser2 = float(u_recv_buf[23] << 8 | u_recv_buf[24]) / 10.0f;
					tofRecvNode->temperature.chip = uint8_t(recv_buf[25]);
					tofRecvNode->serial = serial;
					tofRecvNode->time = time;
					tofRecvNode->size = read_len;

					tofWriteMutex.lock();
					std::swap(tofRecvNode, tofSwapNode);
					tofReadEnable.store(true);
					tofWriteMutex.unlock();
					tofWriteNotify.notify_all();
				}
				else if (u_recv_buf[5] == 0x06)
				{
					memcpy(rgbRecvNode->data(), recv_buf + 56, read_len);
					rgbRecvNode->serial = serial;
					rgbRecvNode->time = time;
					rgbRecvNode->size = read_len;

					rgbWriteMutex.lock();
					std::swap(rgbRecvNode, rgbSwapNode);
					rgbReadEnable.store(true);
					rgbWriteMutex.unlock();
					rgbWriteNotify.notify_all();
				}

				data_len -= frame_len;
				if (data_len > 0)
				{
					memcpy(recv_buf, recv_buf + frame_len, data_len);
				}
				determine = true;

				continue;
			}
			
			break;
		}

		if (!aliveEnable.load()) break;

		std::unique_lock<std::mutex>	lock{ socketMutex };
		if (connectEnable.load())
		{
			connectEnable.store(false);
			auto einfo = getNetworkLastError();

			if (networkAbnormalCallback != nullptr) 
				networkAbnormalCallback(handle, einfo.c_str(), pUserData1);

			LOG_ERROR_OUT("<%s>, The network connection has been disconnected.---%s", sn.c_str(), einfo.c_str());
		}

		tofReadEnable.store(false);
		rgbReadEnable.store(false);
		tofTimeStamp.tv_sec = 0;
		tofTimeStamp.tv_usec = 0;
		rgbTimeStamp.tv_sec = 0;
		rgbTimeStamp.tv_usec = 0;
		socketNotify.wait(lock, [this]{	return  connectEnable.load() || !aliveEnable.load(); });

	}
}

void DeviceHandle::tofDataHandleThread()
{
	ThreadCounter _tc_{ threadCounter };

	uint16_t* dis_ptr;
	uint16_t* amp_ptr;

	uint32_t	index1 = 0;
	uint32_t	count1 = 0;
	cv::Mat		matArray[5];

	uint8_t		last_threshold = 0;
	uint16_t	last_TRSD_JUMP = 0;
	uint16_t	analysis_flag = 0;
	uint16_t    index2 = 0;
	uint32_t    count2 = 0;
	cv::Mat		Depth_local[9];
	cv::Mat		Depth_local_time_mid;

	while (true)
	{
		{
			std::unique_lock<std::mutex> lock{ tofWriteMutex };
			tofWriteNotify.wait(lock, [this] { return tofReadEnable.load() || !aliveEnable.load(); });
			if (!aliveEnable.load())  break;

			tofReadEnable.store(false);
			std::swap(tofHandleNode, tofSwapNode);
		}

		dis_ptr = (uint16_t*)tofHandleNode->data();
		amp_ptr = dis_ptr + tofPixels;

		// 置信度滤波
		if (confidenceFilter.enable)
		{
			auto threshold = uint16_t(confidenceFilter.threshold);
			for (uint32_t i = 0; i < tofPixels; ++i)
			{
				if (amp_ptr[i] < threshold) dis_ptr[i] = 65300;
			}
		}

		// 空间滤波
		cv::Mat src{ tofHeight, tofWidth, CV_16UC1, dis_ptr };
		if (spatialFilter.enable)
		{
			if (spatialFilter.threshold < 6)
			{
				cv::medianBlur(src, src, spatialFilter.threshold);
			}
			else if (spatialFilter.threshold > 5)
			{
				auto cal_times = (spatialFilter.threshold - 1) / 2 - 1;
				while (cal_times-- > 0)
				{
					cv::medianBlur(src, src, 5);
				}
			}
		}

		// 时间中值滤波
		if (timeMedianFilter.enable)
		{
			uint8_t threshold = timeMedianFilter.threshold;

			uint16_t TIME_WIN_SIZE = threshold;
			uint16_t TIME_WIN_MID = floor(TIME_WIN_SIZE / 2);
			auto TRSD_JUMP = uint16_t(timeMedianFilter.k1);
			uint16_t SPATIAL_FILT_SIZE = 5;
			cv::Mat Direction_MASK = cv::Mat::zeros(tofHeight, tofWidth, CV_16U);
			cv::Mat TimeFilt_DOA = cv::Mat::zeros(tofHeight, tofWidth, CV_16U);

			if ((threshold != last_threshold) || (TRSD_JUMP != last_TRSD_JUMP)) {
				index2 = 0;
				count2 = 0;
				analysis_flag = 0;
			}
			if (TIME_WIN_SIZE <= index2) {
				index2 = 0;
			}
			cv::Mat Depth_local_end{ tofHeight, tofWidth, CV_16UC1, dis_ptr };
			cv::medianBlur(Depth_local_end, Depth_local_end, SPATIAL_FILT_SIZE);
			Depth_local[index2] = Depth_local_end.clone();

			if (TIME_WIN_SIZE - 1 <= index2) {
				analysis_flag = 1;
			}
			if (analysis_flag == 1) {

				cv::Mat TimeFilt_data(TIME_WIN_SIZE, tofPixels, CV_16U);

				for (int page = 0; page < TIME_WIN_SIZE; page++) {
					cv::Mat tmp_row = Depth_local[page].reshape(1, 1);
					tmp_row.copyTo(TimeFilt_data.row(page));
				}
				cv::sort(TimeFilt_data, TimeFilt_data, cv::SORT_EVERY_COLUMN);
				cv::Mat tmp_depth = TimeFilt_data.row(TIME_WIN_MID).reshape(1, tofHeight);
				tmp_depth.copyTo(Depth_local_time_mid);

				uint16_t last_index = (index2 - 1) < 0 ? TIME_WIN_SIZE + (index2 - 1) : (index2 - 1);
				cv::Mat DIFF(tofHeight, tofWidth, CV_32S);
				cv::subtract(Depth_local[index2], Depth_local[last_index], DIFF, cv::noArray(), CV_32S);
				Direction_MASK.setTo(1, DIFF >= TRSD_JUMP);
				Direction_MASK.setTo(2, DIFF <= -TRSD_JUMP);

				Depth_local_end.copyTo(TimeFilt_DOA, Direction_MASK == 1);

				uint16_t DOWNSAMPLE_SIZE = 4;
				cv::Mat Direction_MASK_down = cv::Mat::zeros(tofHeight / DOWNSAMPLE_SIZE, tofWidth / DOWNSAMPLE_SIZE, CV_16U);
				auto* MASK_down_Ptr = reinterpret_cast<uint16_t*>(Direction_MASK_down.data);
				for (int r = 0; r < Direction_MASK_down.rows; r++) {
					for (int c = 0; c < Direction_MASK_down.cols; c++, MASK_down_Ptr++) {
						auto* MASK_Ptr = reinterpret_cast<uint16_t*>(Direction_MASK.data);
						MASK_Ptr += r * tofWidth * DOWNSAMPLE_SIZE + c * DOWNSAMPLE_SIZE;

						for (int j = 0; j < DOWNSAMPLE_SIZE; j++) {
							for (int kk = 0; kk < DOWNSAMPLE_SIZE; kk++) {
								if (*MASK_Ptr == 2) {
									*MASK_down_Ptr += 1;
								}
								MASK_Ptr++;
							}
							MASK_Ptr += (tofWidth - DOWNSAMPLE_SIZE);
						}
					}
				}

				uint16_t TRSD_NUM = 30;
				uint16_t TRSD_NUMINWIN = 8;
				cv::Mat Direction_MASK_down_flag = Direction_MASK_down.clone();
				Direction_MASK_down_flag.convertTo(Direction_MASK_down_flag, CV_8U);
				Direction_MASK_down_flag.setTo(0, Direction_MASK_down_flag <= TRSD_NUMINWIN);
				Direction_MASK_down_flag.setTo(255, Direction_MASK_down_flag > TRSD_NUMINWIN);
				cv::Mat Label;
				int num = cv::connectedComponents(Direction_MASK_down_flag, Label, 4, CV_16U);
				std::unordered_map<int, std::vector<int>> Cluster_Index;
				auto* Label_ptr = reinterpret_cast<uint16_t*>(Label.data);
				for (int row = 0; row < Label.rows; row++) {
					for (int col = 0; col < Label.cols; col++, Label_ptr++) {
						if (*Label_ptr > 0) {
							Cluster_Index[*Label_ptr].push_back(row);
							Cluster_Index[*Label_ptr].push_back(col);
						}
					}
				}

				cv::Mat Mask_repair = cv::Mat::zeros(tofHeight, tofWidth, CV_8U);
				for (int id = 1; id <= num; id++) {
					if (Cluster_Index[id].size() / 2 > TRSD_NUM) {
						for (int j = 0; j < (int)Cluster_Index[id].size(); j += 2) {
							uchar* Mask_repair_ptr = Mask_repair.data;
							Mask_repair_ptr += Cluster_Index[id][j] * tofWidth * DOWNSAMPLE_SIZE + Cluster_Index[id][j + 1] * DOWNSAMPLE_SIZE;
							for (int row_offset = 0; row_offset < DOWNSAMPLE_SIZE; row_offset++) {
								for (int col_offset = 0; col_offset < DOWNSAMPLE_SIZE; col_offset++) {
									*Mask_repair_ptr = 1;
									Mask_repair_ptr++;
								}
								Mask_repair_ptr += (tofWidth - DOWNSAMPLE_SIZE);
							}
						}
					}
				}

				Depth_local_end.copyTo(TimeFilt_DOA, (Direction_MASK == 2) & (Mask_repair == 1));

				cv::Mat Mask_smaller_3JUMP = abs(DIFF) <= 3 * TRSD_JUMP;
				cv::Mat Mask_repair0 = (Direction_MASK == 2) & (Mask_repair == 0);
				Depth_local_end.copyTo(TimeFilt_DOA, (Mask_repair0) & (Mask_smaller_3JUMP));

				Depth_local_time_mid.copyTo(TimeFilt_DOA, Mask_repair0 & (~Mask_smaller_3JUMP));
				Depth_local_end.copyTo(TimeFilt_DOA, TimeFilt_DOA >= 60000);

				Mask_repair = cv::Mat::zeros(tofHeight, tofWidth, CV_8U);
				int MID_IDX = (index2 - TIME_WIN_MID) < 0 ? TIME_WIN_SIZE + (index2 - TIME_WIN_MID) : (index2 - TIME_WIN_MID);
				for (int i = 1; i <= TIME_WIN_MID; i++) {
					int TMP_IDX = (MID_IDX + i) > (TIME_WIN_SIZE - 1) ? (MID_IDX + i - TIME_WIN_SIZE) : (MID_IDX + i);
					cv::Mat diff = abs(Depth_local[TMP_IDX] - Depth_local[MID_IDX]);
					Mask_repair.setTo(1, diff > TRSD_JUMP);
				}

				Depth_local_end.copyTo(TimeFilt_DOA, (Direction_MASK == 0) & (Mask_repair == 1));
				Depth_local_time_mid.copyTo(TimeFilt_DOA, (Direction_MASK == 0) & (Mask_repair == 0));

				Depth_local_end.copyTo(TimeFilt_DOA, Depth_local_end >= 60000);

				if (DOWNSAMPLE_SIZE + 1 > 5) {
					cv::medianBlur(TimeFilt_DOA, TimeFilt_DOA, 5);
				}
				else {
					cv::medianBlur(TimeFilt_DOA, TimeFilt_DOA, DOWNSAMPLE_SIZE + 1);
				}

				double guass_sigma = 1.0;
				auto guass_size = int32_t(2.0 * ceil(2 * guass_sigma) + 1);
				cv::Mat guass_Mat;
				cv::GaussianBlur(TimeFilt_DOA, guass_Mat, cv::Size(guass_size, guass_size), guass_sigma);
				cv::Mat guass_mask = ((TimeFilt_DOA > 60000) | (TimeFilt_DOA == 0));
				cv::Mat guass_kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(guass_size, guass_size));
				cv::Mat guass_mask_dilate;
				cv::dilate(guass_mask, guass_mask_dilate, guass_kernel);
				TimeFilt_DOA.copyTo(guass_Mat, guass_mask_dilate);

				double Max_Depth, Min_Depth;
				cv::minMaxLoc(guass_Mat, &Min_Depth, &Max_Depth, nullptr, nullptr, ~guass_mask);
				cv::Mat Gray_Depth = (guass_Mat - Min_Depth) / (Max_Depth - Min_Depth) * 255;
				Gray_Depth.convertTo(Gray_Depth, CV_8U);
				cv::Mat grad_x, grad_y, grad;
				cv::Sobel(Gray_Depth, grad_x, CV_16S, 1, 0, 3);
				cv::Sobel(Gray_Depth, grad_y, CV_16S, 0, 1, 3);
				cv::convertScaleAbs(abs(grad_x) + abs(grad_y), grad);
				grad.setTo(0, guass_mask_dilate);

				cv::Mat Edge_Mask;
				cv::threshold(grad, Edge_Mask, 0, 255, cv::THRESH_OTSU);
				guass_Mat.copyTo(TimeFilt_DOA, ~Edge_Mask);

				auto* data_ptr = reinterpret_cast<uint16_t*>(TimeFilt_DOA.data);
				memcpy(dis_ptr, data_ptr, TimeFilt_DOA.total() * sizeof(uint16_t));
			}

			index2++;
			count2++;
			last_threshold = threshold;
			last_TRSD_JUMP = TRSD_JUMP;

		}
		else if (count2 > 0) {
			count2 = 0;
			index2 = 0;
			analysis_flag = 0;
		}

		// 飞点滤波
		if (flyingPixelsFilter.enable)
		{
			uint8_t threshold = flyingPixelsFilter.threshold;
			cv::Mat gauss_mat;

			cv::GaussianBlur(src, gauss_mat, cv::Size(3, 3), threshold * 0.1);

			cv::Mat edges_x, edges_y, edges;
			cv::Sobel(gauss_mat, edges_x, CV_16S, 1, 0, 1);
			cv::Sobel(gauss_mat, edges_y, CV_16S, 0, 1, 1);
			cv::convertScaleAbs(edges_x + edges_y, edges);

			cv::Mat compare_mat;
			cv::compare(edges, (64u - threshold) << 2, compare_mat, cv::CMP_LT);
			compare_mat /= 255;
			compare_mat.convertTo(compare_mat, CV_16UC1);

			cv::Mat dst_mat;
			cv::multiply(compare_mat, src, dst_mat);
			cv::medianBlur(dst_mat, src, 3);
		}

		// 时域滤波
		if (timeFilter.enable)
		{
			uint8_t threshold = timeFilter.threshold;
			if (threshold <= index1)
			{
				index1 = 0;
				count1 = 0;
				for (uint8_t i = 0; i < threshold; ++i)
					matArray[i] = cv::Mat::zeros(tofHeight, tofWidth, CV_16UC1);
			}
			matArray[index1] = src.clone();
			cv::Mat weight_mat = cv::Mat::zeros(tofHeight, tofWidth, CV_16UC1);
			cv::Mat sum_mat = cv::Mat::zeros(tofHeight, tofWidth, CV_16UC1);
			cv::Mat temp_mat, more_than_zero, less_than_sp, add_weight, now_dis, sp_mat;
			for (uint8_t i = 0; i < threshold; i++)
			{
				if (matArray[i].empty()) matArray[i] = cv::Mat::zeros(tofHeight, tofWidth, CV_16UC1);
				cv::compare(matArray[i], 0, more_than_zero, cv::CMP_GT);
				cv::compare(matArray[i], 0x8000, less_than_sp, cv::CMP_LT);
				add_weight = (more_than_zero & less_than_sp) / 255;
				add_weight.convertTo(add_weight, CV_16UC1);

				if (i == index1)
				{
					add_weight *= threshold;
					now_dis = src.clone();
				}

				cv::multiply(add_weight, matArray[i], temp_mat);
				sum_mat += temp_mat;
				weight_mat += add_weight;
			}

			cv::divide(sum_mat, weight_mat, src);
			cv::compare(src, 0, sp_mat, cv::CMP_EQ);
			sp_mat /= 255;
			sp_mat.convertTo(sp_mat, CV_16UC1);
			cv::multiply(sp_mat, now_dis, now_dis);
			src += now_dis;
			index1 = ++count1 % threshold;
		}
		else if (count1 > 0)
		{
			index1 = 0;
			count1 = 0;
		}

		dataReadMutex.lock();
		readEnable.store(true);
		tofDataReadQueue.enqueue(tofHandleNode);
		dataReadMutex.unlock();
		dataReadNotify.notify_all();
	}

}

void DeviceHandle::rgbDataHandleThread()
{
	ThreadCounter _tc_{ threadCounter };

	auto  tjHandle = tjInitDecompress();

	while (true)
	{
		{
			std::unique_lock<std::mutex> lock{ rgbWriteMutex };
			rgbWriteNotify.wait(lock, [this] {  return rgbReadEnable.load() || !aliveEnable.load(); });
			if (!aliveEnable.load()) break;

			rgbReadEnable.store(false);
			std::swap(rgbSrcNode, rgbSwapNode);
		}

		cv::Mat img(rgbHeight, rgbWidth, CV_8UC3, rgbDstNode->data());
		if (rgbTFormat == LWRgbTransferFormat::LW_MJPEG)
		{
			tjDecompress2(tjHandle, rgbSrcNode->data(), rgbSrcNode->size, rgbDstNode->data(), rgbWidth, rgbWidth * 3, rgbHeight, TJPF_RGB, TJFLAG_NOREALLOC);
		}
		else if (rgbTFormat == LWRgbTransferFormat::LW_YUV422_YUYV)
		{
			cv::cvtColor(
				cv::Mat(rgbHeight, rgbWidth, CV_8UC2, rgbSrcNode->data()),
				img,
				cv::COLOR_YUV2RGB_YUYV
			);
		}
		else if (rgbTFormat == LWRgbTransferFormat::LW_YVU420_NV12)
		{
			cv::cvtColor(
				cv::Mat(int32_t(rgbHeight * 1.5), rgbWidth, CV_8UC1, rgbSrcNode->data()),
				img,
				cv::COLOR_YUV2RGB_NV12
			);
		}
		else
			continue;
		
		cv::flip(img, img, -1);
		if(rgbDistortionEnable) cv::remap(img, img, rgbCalibMap1, rgbCalibMap2, cv::INTER_LINEAR);

		rgbDstNode->serial = rgbSrcNode->serial;
		rgbDstNode->time = rgbSrcNode->time;
		rgbDstNode->size = rgbWidth * rgbHeight * 3;

		dataReadMutex.lock();
		readEnable.store(true);
		rgbDataReadQueue.enqueue(rgbDstNode);
		dataReadMutex.unlock();
		dataReadNotify.notify_all();
	}

	tjDestroy(tjHandle);
}

void DeviceHandle::dataSyncThread()
{
	ThreadCounter _tc_{ threadCounter };

	while (true)
	{
		std::unique_lock<std::mutex> tof_lock{ dataReadMutex };
		dataReadNotify.wait(tof_lock, [this] { return readEnable.load() || !aliveEnable.load(); });
		if (!aliveEnable.load()) break;

		readEnable.store(false);

		// 获取TOF和RGB最新数据
		auto tof_node = tofDataReadQueue.latestNode();
		auto rgb_node = rgbDataReadQueue.latestNode();

		if (dataSyncEnable && hasRgbModule.load())
		{
			// 帧数据的序列号必须相等
			if (tof_node->serial != rgb_node->serial)
			{
				if (tof_node->serial < rgb_node->serial)
				{
					rgb_node = rgbDataReadQueue.find(tof_node->serial);
					if (rgb_node == nullptr) continue;
				}
				else
				{
					tof_node = tofDataReadQueue.find(rgb_node->serial);
					if (tof_node == nullptr) continue;
				}
			}

			// 时间戳必须比前一帧的大，以防止帧回跳
			if (tofTimeStamp.tv_sec > tof_node->time.tv_sec) continue;
			if (tofTimeStamp.tv_sec < tof_node->time.tv_sec || tofTimeStamp.tv_usec < tof_node->time.tv_usec)
			{
				tofTimeStamp = tof_node->time;

				syncDataMutex.lock();
				tofDataReadQueue.dequeue(tof_node->mark, tofNode);
				rgbDataReadQueue.dequeue(rgb_node->mark, rgbNode);
				readyEnable.store(true);
				syncDataMutex.unlock();
				syncDataNotify.notify_all();

				if (frameReadyCallback != nullptr) frameReadyCallback(handle, pUserData2);

				continue;
			}
		}
		else
		{
			syncDataMutex.lock();
			if (tof_node->update)
			{
				// 时间戳必须比前一帧的大，防止帧回跳
				if ((tof_node->time.tv_sec > tofTimeStamp.tv_sec)
					|| ((tof_node->time.tv_sec == tofTimeStamp.tv_sec) && (tof_node->time.tv_usec > tofTimeStamp.tv_usec)))
				{
					tofTimeStamp = tof_node->time;
					tofDataReadQueue.dequeue(tof_node->mark, tofNode);
				}
			}
			else {
				tofNode->update = false;
			}

			if (rgb_node->update)
			{
				// 时间戳必须比前一帧的大，防止帧回跳
				if ((rgb_node->time.tv_sec > rgbTimeStamp.tv_sec)
					|| ((rgb_node->time.tv_sec == rgbTimeStamp.tv_sec) && (rgb_node->time.tv_usec > rgbTimeStamp.tv_usec)))
				{
					rgbTimeStamp = rgb_node->time;
					rgbDataReadQueue.dequeue(rgb_node->mark, rgbNode);
				}
			}
			else {
				rgbNode->update = false;
			}

			readyEnable.store(true);
			syncDataMutex.unlock();
			syncDataNotify.notify_all();

			if (frameReadyCallback != nullptr) frameReadyCallback(handle, pUserData2);

			continue;
		}
	}
}

void DeviceHandle::rgbToDepthThread(int threadID, int srcStartRow, int srcEndRow)
{
	ThreadCounter _tc_{ threadCounter };

	int voxelSize = 4;
	int extend_size = 8;
	int startRow = 0;
	int endRow = 0;
	int shift_ptr = threadID != 0 ? extend_size * tofWidth : 0;

	if (threadID == 0)
	{
		endRow = srcEndRow + extend_size;
	}
	else if (threadID == THREAD_POOL_SIZE - 1)
	{
		startRow = srcStartRow - extend_size;
		endRow = srcEndRow;
	}
	else
	{
		startRow = srcStartRow - extend_size;
		endRow = srcEndRow + extend_size;
	}

	int xmin = 0;
	int ymin = 0;
	int xmax = rgbWidth;
	int ymax = rgbHeight;
	int n = (endRow - startRow) * tofWidth;    //tof分段对应像素点数量

	cv::Mat T_rpt = cv::repeat(tMatrixR2D, 1, n);
	cv::Mat tempXYZ(3, n, CV_32FC1, cv::Scalar::all(0));
	cv::Mat xPixel(1, n, CV_32FC1, cv::Scalar::all(0));
	cv::Mat yPixel(1, n, CV_32FC1, cv::Scalar::all(0));
	auto* ptrX = (float*)(xPixel.data);
	auto* ptrY = (float*)(yPixel.data);

	uint32_t idFlag = 1U << threadID;   // 线程标识
	while (isR2DEnable.load())
	{
		{// 线程运行条件判定
			std::unique_lock<std::mutex> lock{ r2dMutex };
			r2dNotify.wait(lock, [&idFlag, this] { return (r2dFlag & idFlag) > 0 || !isR2DEnable.load(); });
			if (!isR2DEnable.load()) break;
			if (r2dArgRenew & idFlag)
			{
				xmax = rgbWidth;
				ymax = rgbHeight;
				T_rpt = cv::repeat(tMatrixR2D, 1, n);
				r2dArgRenew &= ~idFlag;
			}
		}

		// 先计算tof相机点云
		auto tXYZ1 = (float*)tempXYZ.row(0).data;
		auto tXYZ2 = (float*)tempXYZ.row(1).data;
		auto tXYZ3 = (float*)tempXYZ.row(2).data;
		auto tRow = float(startRow) - tofInArg.cy;
		auto tCol = -tofInArg.cx;
		auto ptrDepth = (uint16_t*)(tofCutNode->data()) + tofWidth * startRow;
		for (int i = startRow; i < endRow; ++i)
		{
			tCol = -tofInArg.cx;
			for (int j = 0; j < tofWidth; ++j, tCol += 1.0f, ++ptrDepth, ++tXYZ1, ++tXYZ2, ++tXYZ3)
			{
				if (*ptrDepth < PCD_MAX_VALUE)
				{
					*tXYZ1 = tCol / tofInArg.fx * float(*ptrDepth);
					*tXYZ2 = tRow / tofInArg.fy * float(*ptrDepth);
					*tXYZ3 = *ptrDepth;
				}
			}
			tRow += 1.0f;
		}

		// 再转换至RGB坐标系
		tempXYZ = rMatrixR2D * tempXYZ + T_rpt;
		cv::divide(tempXYZ.row(0), tempXYZ.row(2), xPixel);
		cv::divide(tempXYZ.row(1), tempXYZ.row(2), yPixel);

		// 20240727-修改后，把均值计算放至后续判定条件里
		// 像素占据情况统计
		cv::Mat occupMap = cv::Mat(std::ceil(float(ymax - ymin) / float(voxelSize)), std::ceil(float(xmax - xmin) / float(voxelSize)), CV_16U, cv::Scalar::all(0));
		cv::Mat occupMapZ = cv::Mat(std::ceil(float(ymax - ymin) / float(voxelSize)), std::ceil(float(xmax - xmin) / float(voxelSize)), CV_32F, cv::Scalar::all(0));
		ptrX = (float*)(xPixel.data);
		ptrY = (float*)(yPixel.data);
		auto ptrZ = (float*)(tempXYZ.row(2).data);
		for (int i = 0; i < xPixel.cols; i++, ++ptrX, ++ptrY, ++ptrZ)
		{
			int idxX = std::floor((*ptrX - float(xmin)) / float(voxelSize));
			int idxY = std::floor((*ptrY - float(ymin)) / float(voxelSize));
			if (idxX >= 0 && idxY >= 0 && idxX < occupMap.cols && idxY < occupMap.rows && *ptrZ>0 && *ptrZ < PCD_MAX_VALUE)
				//if (idxX >= 0 && idxY >= 0 && idxX < occupMap.cols && idxY < occupMap.rows)
			{
				occupMap.at<uint16_t>(idxY, idxX) = occupMap.at<uint16_t>(idxY, idxX) + 1;
				occupMapZ.at<float>(idxY, idxX) += (*ptrZ);
			}
		}

		//创建结果储存
		ptrX = (float*)(xPixel.data) + shift_ptr;
		ptrY = (float*)(yPixel.data) + shift_ptr;
		ptrZ = (float*)(tempXYZ.row(2).data) + shift_ptr;
		ptrDepth = (uint16_t*)(tofCutNode->data()) + tofWidth * srcStartRow;
		auto ptrR2D = (LWRGB888Pixel*)pR2D.data() + tofWidth * srcStartRow;
		auto ptrRGB = (LWRGB888Pixel*)rgbCutNode->data();
		for (int row = srcStartRow; row < srcEndRow; ++row)
		{
			for (int col = 0; col < tofWidth; ++col, ++ptrDepth, ++ptrX, ++ptrY, ++ptrZ, ++ptrR2D)
			{
				if (*ptrDepth < PCD_MAX_VALUE && *ptrDepth>0)
				{
					auto x_ = int(*ptrX);
					auto y_ = int(*ptrY);

					int idxX = std::floor(float(x_ - xmin) / float(voxelSize));
					int idxY = std::floor(float(y_ - ymin) / float(voxelSize));
					// 20240727-根据数据精度设置低于均值+σ以下的数据才是RGB坐标系下的目标物，否则就是其投影区冗余数据
					if (idxX >= 0 && idxY >= 0 && idxX < occupMap.cols && idxY < occupMap.rows)
					{
						float tmp_average_Z = occupMapZ.at<float>(idxY, idxX) / float(occupMap.at<uint16_t>(idxY, idxX));
						if (x_ > -1 && y_ > -1 && x_ < rgbWidth && y_ < rgbHeight && ((*ptrZ) <= (tmp_average_Z + RGBD_PROJ_REDUNDANCY)))
						{
							int index = y_ * rgbWidth + x_;
							*ptrR2D = ptrRGB[index];

							continue;
						}
					}
				}
			}
		}

		r2dMutex.lock();
		r2dFlag &= ~idFlag;
		if (--r2dRunCount == 0)
		{// 所有线程跑完一帧 
			r2dNotify.notify_all();
		}
		r2dMutex.unlock();
	}
}

void DeviceHandle::depthToRgbThread(int threadID, int srcStartRow, int srcEndRow)
{
	ThreadCounter _tc_{ threadCounter };

	int voxelSize = 2;
	int extend_size = 8;
	int startRow = 0;
	int endRow = 0;
	int shift_ptr = threadID != 0 ? extend_size * tofWidth : 0;

	if (threadID == 0)
	{
		endRow = srcEndRow + extend_size;
	}
	else if (threadID == THREAD_POOL_SIZE - 1)
	{
		startRow = srcStartRow - extend_size;
		endRow = srcEndRow;
	}
	else
	{
		startRow = srcStartRow - extend_size;
		endRow = srcEndRow + extend_size;
	}

	int xmin = 0;
	int ymin = 0;
	int xmax = int(float(rgbWidth) / d2rScale);
	int ymax = int(float(rgbHeight) / d2rScale);
	int n = (endRow - startRow) * tofWidth;    //tof分段对应像素点数量

	cv::Mat T_rpt = cv::repeat(tMatrixD2R, 1, n);
	cv::Mat tempXYZ(3, n, CV_32FC1, cv::Scalar::all(0));
	cv::Mat xPixel(1, n, CV_32SC1, cv::Scalar::all(0));
	cv::Mat yPixel(1, n, CV_32SC1, cv::Scalar::all(0));
	auto* ptrX = (float*)(xPixel.data);
	auto* ptrY = (float*)(yPixel.data);

	uint32_t idFlag = 1U << threadID;   // 线程标识
	while (isD2REnable.load())
	{
		{// 线程运行条件判定
			std::unique_lock<std::mutex> lock{ d2rMutex };
			d2rNotify.wait(lock, [&idFlag, this] { return (d2rFlag & idFlag) > 0 || !isD2REnable.load(); });
			if (!isD2REnable.load()) break;
			if (d2rArgRenew & idFlag)
			{
				xmax = int(float(rgbWidth) / d2rScale);
				ymax = int(float(rgbHeight) / d2rScale);
				T_rpt = cv::repeat(tMatrixD2R, 1, n);
				d2rArgRenew &= ~idFlag;
			}
		}

		// 对 tempXYZ 变量赋初值
		auto tXYZ1 = (float*)tempXYZ.row(0).data;
		auto tXYZ2 = (float*)tempXYZ.row(1).data;
		auto tXYZ3 = (float*)tempXYZ.row(2).data;
		auto tRow = float(startRow) - tofInArg.cy;
		auto tCol = -tofInArg.cx;
		auto ptrDepth = (uint16_t*)(tofCutNode->data()) + tofWidth * startRow;
		for (int i = startRow; i < endRow; ++i)
		{
			tCol = -tofInArg.cx;
			for (int j = 0; j < tofWidth; ++j, ++ptrDepth, ++tXYZ1, ++tXYZ2, ++tXYZ3)
			{
				if (*ptrDepth < PCD_MAX_VALUE && *ptrDepth>0)
				{
					*tXYZ1 = tCol / tofInArg.fx * float(*ptrDepth);
					*tXYZ2 = tRow / tofInArg.fy * float(*ptrDepth);
					*tXYZ3 = *ptrDepth;
				}
				tCol += 1.0f;
			}
			tRow += 1.0f;
		}

		// 对 tempXYZ 变量的值做相应变换
		tempXYZ = rMatrixD2R * tempXYZ + T_rpt;
		cv::divide(tempXYZ.row(0), tempXYZ.row(2), xPixel);
		cv::divide(tempXYZ.row(1), tempXYZ.row(2), yPixel);

		// 20240727-修改为累加后除以计数
		// 像素占据情况统计
		cv::Mat occupMap = cv::Mat(std::ceil(float(ymax - ymin) / float(voxelSize)), std::ceil(float(xmax - xmin) / float(voxelSize)), CV_16U, cv::Scalar::all(0));
		cv::Mat occupMapZ = cv::Mat(std::ceil(float(ymax - ymin) / float(voxelSize)), std::ceil(float(xmax - xmin) / float(voxelSize)), CV_32F, cv::Scalar::all(0));
		cv::Mat occupMinZ = cv::Mat(std::ceil(float(ymax - ymin) / float(voxelSize)), std::ceil(float(xmax - xmin) / float(voxelSize)), CV_16U, cv::Scalar::all(0));
		ptrX = (float*)(xPixel.data);
		ptrY = (float*)(yPixel.data);
		ptrDepth = (uint16_t*)(tofCutNode->data()) + tofWidth * startRow;
		auto ptrZ = (float*)(tempXYZ.row(2).data);
		for (int i = 0; i < xPixel.cols; i++, ++ptrX, ++ptrY, ++ptrZ, ++ptrDepth)
		{
			int idxX = std::floor((*ptrX - float(xmin)) / float(voxelSize));
			int idxY = std::floor((*ptrY - float(ymin)) / float(voxelSize));
			if (idxX >= 0 && idxY >= 0 && idxX < occupMinZ.cols && idxY < occupMinZ.rows && *ptrZ>0 && *ptrZ < PCD_MAX_VALUE)
			{
				if (occupMap.at<uint16_t>(idxY, idxX) == 0)
				{
					occupMinZ.at<uint16_t>(idxY, idxX) = *ptrDepth;
				}
				else
				{
					if (*ptrDepth < occupMinZ.at<uint16_t>(idxY, idxX))
					{
						occupMinZ.at<uint16_t>(idxY, idxX) = *ptrDepth;
					}
				}
				occupMap.at<uint16_t>(idxY, idxX) += 1;
				occupMapZ.at<float>(idxY, idxX) += *ptrZ;
			}
		}

		//创建结果储存
		ptrX = (float*)(xPixel.data) + shift_ptr;
		ptrY = (float*)(yPixel.data) + shift_ptr;
		ptrZ = (float*)(tempXYZ.row(2).data) + shift_ptr;
		ptrDepth = (uint16_t*)(tofCutNode->data()) + tofWidth * srcStartRow;
		auto ptrDst = (uint16_t*)d2rInputImg.data;
		for (int row = srcStartRow; row < srcEndRow; ++row)
		{
			for (int col = 0; col < tofWidth; ++col, ++ptrDepth, ++ptrX, ++ptrY)
			{
				if (*ptrDepth < PCD_MAX_VALUE)
				{
					auto x_ = int(*ptrX);
					auto y_ = int(*ptrY);

					int idxX = std::floor(float(x_ - xmin) / float(voxelSize));
					int idxY = std::floor(float(y_ - ymin) / float(voxelSize));
					// 根据数据精度设置低于均值+σ以下的数据才是RGB坐标系下的目标物，否则就是其投影区冗余数据
					if (idxX >= 0 && idxY >= 0 && idxX < occupMap.cols && idxY < occupMap.rows)
					{
						auto tmp_average_Z = uint16_t(occupMapZ.at<float>(idxY, idxX) / float(occupMap.at<uint16_t>(idxY, idxX)));
						if (x_ > -1 && y_ > -1 && x_ < xmax && y_ < ymax)
						{
							int index = y_ * xmax + x_;
							ptrDst[index] = *ptrDepth;
							if (*ptrDepth > (tmp_average_Z + RGBD_PROJ_REDUNDANCY))
							{
								ptrDst[index] = occupMinZ.at<uint16_t>(idxY, idxX);
							}
							continue;
						}
					}
				}
			}
		}

		d2rMutex.lock();
		d2rFlag &= ~idFlag;
		if (--d2rRunCount == 0)
		{// 所有线程跑完一帧 
			// fill empty area
			ptrDst = (uint16_t*)d2rInputImg.data;
			for (int row = 1, index; row < tofHeight - 1; ++row)
			{
				for (int col = 1; col < tofWidth - 1; ++col)
				{
					index = row * tofWidth + col;
					if (*(ptrDst + index) == 0)
					{
						int cnt = 0;
						int tmpIndex = 0;
						int tmpVal = 0;
						for (int locR = row - 1; locR <= row + 1; locR++)
						{
							for (int locC = col - 1; locC <= col + 1; locC++)
							{
								tmpIndex = locR * tofWidth + locC;
								if (*(ptrDst + tmpIndex) > 0)
								{
									cnt++;
									tmpVal += ptrDst[tmpIndex];
								}
							}
						}
						if (cnt > 6) // 当该像素邻域存在超20个点有效值时，将这些点的均值赋值给他
						{
							ptrDst[index] = tmpVal / cnt;
						}
					}

				}
			}

			// 缩放图像大小
			cv::resize(d2rInputImg, d2rOutputImg, cv::Size(rgbWidth, rgbHeight), 0, 0, cv::INTER_NEAREST);

			d2rNotify.notify_all();
		}
		d2rMutex.unlock();
	}
}

LWReturnCode DeviceHandle::ExecuteCommand(CommandFrame& command)
{
	std::unique_lock<std::mutex> lock{ socketMutex };

	isCommandReply.store(false);
	if (send(commandSocket, command.data(), command.size(), MSG_NOSIGNAL) != SOCKET_ERROR)
	{
		if (socketNotify.wait_for(lock, std::chrono::milliseconds{ timeout }, [this] { return isCommandReply.load() || !connectEnable.load() || !aliveEnable.load(); }))
		{
			if (isCommandReply.load())
			{
				if (commandFrame.isCommand(command.getCommand()))
				{
					command = commandFrame;
					if (command.getCommandType() < 0x03) return LW_RETURN_OK;

					return  LWReturnCode(command.getCommandType());
				}

				gGlobal.errorInfo = "Non response command.";
				return LW_RETURN_CUSTOM_ERROR;
			}
		}
		else
			return LW_RETURN_TIMEOUT;
	}

	gGlobal.errorInfo = getNetworkLastError();
	return LW_RETURN_NETWORK_ERROR;
}

LWReturnCode DeviceHandle::OpenDevice()
{
	LOG_INFO_OUT("<%s>", sn.c_str());

    if(!connectEnable.load())
    {
		// 清理连接
		closeSocket(dataSocket);
		closeSocket(commandSocket);

		// 休眠，以规避设备占用问题
		std::this_thread::sleep_for(std::chrono::milliseconds(500));

		// 探测设备占用情况
		SOCKET _socket = gSocketMap[localAddr.sin_addr.s_addr];
		setNetworkTimeout(_socket, timeout);
		CommandFrame command{ C_Discovery };
		if ((sendto(_socket, command.data(), command.size(), 0, (sockaddr*)&remoteAddr, sizeof(remoteAddr)) != SOCKET_ERROR)
			&& (recvfrom(_socket, command.data(), command.maxBuffer(), 0, nullptr, nullptr) != SOCKET_ERROR))
		{
			if (!command.isCommand(C_Discovery)) return LW_RETURN_COMMAND_ERROR;
			auto lik = ntohl(*((uint32_t*)(command.getArgField() + 4)));
			if (lik != 0)
			{
				char buffer[128];
				auto ipPtr = (unsigned char*)&lik;
				snprintf(buffer, 128, "The device is already occupied.(%u.%u.%u.%u)", ipPtr[0], ipPtr[1], ipPtr[2], ipPtr[3]);
				gGlobal.errorInfo = std::string(buffer);
				LOG_ERROR_OUT("<%s>, %s", sn.c_str(), gGlobal.errorInfo.c_str());
				return LW_RETURN_CUSTOM_ERROR;
			}
		}
		else
		{
			gGlobal.errorInfo = getNetworkLastError();
			LOG_ERROR_OUT("<%s>, %s", sn.c_str(), gGlobal.errorInfo.c_str());
			return LW_RETURN_NETWORK_ERROR;
		}
		
		// 建立命令端口连接
		commandSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
		int bOpt = true;
		if ((setsockopt(commandSocket, SOL_SOCKET, SO_REUSEADDR, (char*)&bOpt, sizeof(bOpt)) == SOCKET_ERROR)
			|| (bind(commandSocket, (sockaddr*)(&localAddr), sizeof(localAddr)) == SOCKET_ERROR)
			|| !connectToServer(commandSocket, remoteAddr, timeout))
		{
			auto error_str = getNetworkLastError();
			gGlobal.errorInfo = describe.empty() ? (error_str.empty() ? "Control socket connection timeout!" : error_str) : describe;
			LOG_ERROR_OUT("<%s>, %s", sn.c_str(), gGlobal.errorInfo.c_str());
			return LW_RETURN_CUSTOM_ERROR;
		}
		// 建立数据端口连接
		dataSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
		if ((setsockopt(dataSocket, SOL_SOCKET, SO_REUSEADDR, (char*)&bOpt, sizeof(bOpt)) == SOCKET_ERROR)
			|| (bind(dataSocket, (sockaddr*)(&localAddr), sizeof(localAddr)) == SOCKET_ERROR)
			|| !connectToServer(dataSocket, recvAddr, timeout))
		{
			auto error_str = getNetworkLastError();
			gGlobal.errorInfo = error_str.empty() ? "Data socket connection timeout!" : error_str;
			LOG_ERROR_OUT("<%s>, %s", sn.c_str(), gGlobal.errorInfo.c_str());
			return LW_RETURN_CUSTOM_ERROR;
		}

        connectEnable.store(true);
        socketNotify.notify_all();
    }
	ProvisionalAuthority val{ openEnable };
	
	// 资源初始化
    auto RC = GetResolution(LWSensorType::LW_RGB_SENSOR, rgbWidth, rgbHeight);
    if (hasRgbModule.load())
    {
		rgbPixels = rgbWidth * rgbHeight;

        RC = GetIntrinsicParam(LWSensorType::LW_RGB_SENSOR, rgbInArg);
        if (RC != LW_RETURN_OK) return RC;

        RC = GetExtrinsicParam(LWSensorType::LW_RGB_SENSOR, rgbOutArg);
        if (RC != LW_RETURN_OK) return RC;

        auto ptr = (float*)&rgbOutArg;
        auto ptr1 = (float*)rMatrixR2D.data;
        auto ptr2 = (float*)tMatrixR2D.data;
        auto ptr3 = (float*)rMatrixD2R.data;
        auto ptr4 = (float*)tMatrixD2R.data;
        for (int i = 0; i < 9; i++, ++ptr)
        {
            *ptr1++ = *ptr;
            *ptr3++ = *ptr;
        }
        for (int i = 0; i < 3; i++, ++ptr)
        {
            *ptr2++ = *ptr;
            *ptr4++ = *ptr;
        }

		d2rArgRenew = 0xFF;
		r2dArgRenew = 0xFF;
        CreateRGBCameraCalibrationTable(rgbInArg, d2rScale, cv::Size(rgbWidth, rgbHeight), rgbCalibMap1, rgbCalibMap2, rMatrixR2D, tMatrixR2D, rMatrixD2R, tMatrixD2R);
    }

	RC = GetResolution(LWSensorType::LW_TOF_SENSOR, tofWidth, tofHeight);
	if (RC != LW_RETURN_OK) return RC;
	tofPixels = tofWidth * tofHeight;
	d2rInputImg.create(cv::Size(tofWidth, tofHeight), CV_16UC1);

    RC = GetTriggerMode(triggerMode);
    if (RC != LW_RETURN_OK) return RC;

    RC = GetIntrinsicParam(LWSensorType::LW_TOF_SENSOR, tofInArg);
    if (RC != LW_RETURN_OK) return RC;

    RC = GetTimeFilterParams(timeFilter);
    if (RC != LW_RETURN_OK) return RC;

    RC = GetFlyingPixelsFilterParams(flyingPixelsFilter);
    if (RC != LW_RETURN_OK) return RC;

    RC = GetConfidenceFilterParams(confidenceFilter);
    if (RC != LW_RETURN_OK) return RC;

    RC = GetSpatialFilterParams(spatialFilter);
    if (RC != LW_RETURN_OK) return RC;

    RC = GetTimeMedianFilterParams(timeMedianFilter);
    if (RC != LW_RETURN_OK) return RC;
	
    RC = GetIRGMMGain(irGMMGain);
    if (RC != LW_RETURN_OK) return RC;

	val.maintain();

    return  LW_RETURN_OK;
}

LWReturnCode DeviceHandle::CloseDevice()
{
	LOG_INFO_OUT("<%s>", sn.c_str());

	openEnable.store(false);

	if (connectEnable.load())
	{
		connectEnable.store(false);

		closeSocket(commandSocket);
		closeSocket(dataSocket);
	}

	return LW_RETURN_OK;
}

LWReturnCode DeviceHandle::ReconnectDevice(uint32_t t)
{
	LOG_INFO_OUT("<%s>, timeout: %u", sn.c_str(), t);

	if (!openEnable.load()) return LW_RETURN_UNOPENED;
	if (connectEnable.load()) return LW_RETURN_OK;

	// 清理连接
	closeSocket(dataSocket);
	closeSocket(commandSocket);

	// 休眠，以规避设备占用问题
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));

	// 检测是否被占用
	CommandFrame command{ C_Discovery };
	SOCKET _socket = gSocketMap[localAddr.sin_addr.s_addr];
	setNetworkTimeout(_socket, t);
	if ((sendto(_socket, command.data(), command.size(), 0, (sockaddr*)&remoteAddr, sizeof(remoteAddr)) != SOCKET_ERROR)
		&& (recvfrom(_socket, command.data(), command.maxBuffer(), 0, nullptr, nullptr) != SOCKET_ERROR))
	{
		if (!command.isCommand(C_Discovery)) return LW_RETURN_COMMAND_ERROR;
		auto lik = ntohl(*((uint32_t*)(command.getArgField() + 4)));
		if (lik != 0)
		{
			char buffer[128];
			auto ipPtr = (unsigned char*)&lik;
			snprintf(buffer, 128, "The device is already occupied.(%u.%u.%u.%u)", ipPtr[0], ipPtr[1], ipPtr[2], ipPtr[3]);
			gGlobal.errorInfo = std::string(buffer);
			LOG_ERROR_OUT("<%s>, %s", sn.c_str(), gGlobal.errorInfo.c_str());
			return LW_RETURN_CUSTOM_ERROR;
		}
	}
	else
	{
		gGlobal.errorInfo = getNetworkLastError();
		LOG_ERROR_OUT("<%s>, %s", sn.c_str(), gGlobal.errorInfo.c_str());
		return LW_RETURN_NETWORK_ERROR;
	}

	// 建立新连接
	commandSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	dataSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	int bOpt = true;
	if ((setsockopt(commandSocket, SOL_SOCKET, SO_REUSEADDR, (char*)&bOpt, sizeof(bOpt)) != SOCKET_ERROR)
		&& (setsockopt(dataSocket, SOL_SOCKET, SO_REUSEADDR, (char*)&bOpt, sizeof(bOpt)) != SOCKET_ERROR)
		&& (bind(commandSocket, (sockaddr*)(&localAddr), sizeof(localAddr)) != SOCKET_ERROR)
		&& (bind(dataSocket, (sockaddr*)(&localAddr), sizeof(localAddr)) != SOCKET_ERROR)
		&& connectToServer(commandSocket, remoteAddr, t)
		&& connectToServer(dataSocket, recvAddr, t))
	{
		connectEnable.store(true);
		socketNotify.notify_all();

		return LW_RETURN_OK;
	}

	auto error_str = getNetworkLastError();
	gGlobal.errorInfo = error_str.empty() ? "Device reconnection timeout!" : error_str;
	LOG_ERROR_OUT("<%s>, %s", sn.c_str(), gGlobal.errorInfo.c_str());

	return LW_RETURN_CUSTOM_ERROR;
}

LWReturnCode DeviceHandle::RebootDevice()
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>", sn.c_str());

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	CommandFrame command{ C_DeviceReboot };

	auto ret = ExecuteCommand(command);
	if (ret == LW_RETURN_OK)
	{
		openEnable.store(false);
		connectEnable.store(false);

		closeSocket(commandSocket);
		closeSocket(dataSocket);
	}

	return ret;
}

LWReturnCode DeviceHandle::SaveConfigureInfo()
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>", sn.c_str());

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	CommandFrame command{ C_SaveConfig };

	return ExecuteCommand(command);
}

LWReturnCode DeviceHandle::RemoveConfigureInfo()
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>", sn.c_str());

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	CommandFrame command{ C_DeleteConfig };

	auto ret = ExecuteCommand(command);
	if (ret == LW_RETURN_OK)
	{
		openEnable.store(false);
		connectEnable.store(false);

		closeSocket(commandSocket);
		closeSocket(dataSocket);
	}

	return ret;
}

LWReturnCode DeviceHandle::RestoreFactoryConfigureInfo()
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>", sn.c_str());

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	CommandFrame command{ C_RecoveryDefaultConfigure };

	auto ret = ExecuteCommand(command);
	if (ret == LW_RETURN_OK)
	{
		openEnable.store(false);
		connectEnable.store(false);

		closeSocket(commandSocket);
		closeSocket(dataSocket);
	}

	return ret;
}

LWReturnCode DeviceHandle::StartStream()
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>", sn.c_str());

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	CommandFrame command{ C_Start };
	readyEnable.store(false);

	return ExecuteCommand(command);
}

LWReturnCode DeviceHandle::StopStream()
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>", sn.c_str());

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	CommandFrame command{ C_Stop };

	return ExecuteCommand(command);
}

LWReturnCode DeviceHandle::HasRgbModule(bool& value)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>", sn.c_str());

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	CommandFrame command{ C_GetImageInfo };

	auto RC = ExecuteCommand(command);
	if (RC == LW_RETURN_OK)
	{
		value = bool(command.getArgField()[0]);
	}

	return RC;
}

LWReturnCode DeviceHandle::SoftTrigger()
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	//LOG_INFO_OUT("<%s>", sn.c_str());

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	if (triggerMode != LW_TRIGGER_SOFT)
	{
		gGlobal.errorInfo = "The sensor is currently not in soft trigger mode.";
		LOG_ERROR_OUT("<%s>, %s", sn.c_str(), gGlobal.errorInfo.c_str());
		return LW_RETURN_CUSTOM_ERROR;
	}

	CommandFrame command{ C_SoftTrigger };

	return ExecuteCommand(command);
}

LWReturnCode DeviceHandle::SetTriggerMode(LWTriggerMode mode)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>, Mode: %u", sn.c_str(), mode);

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	CommandFrame command{ C_SetTriggerMode };

	uint8_t val = mode;
	command.setArgField(&val, 1);
	auto RC = ExecuteCommand(command);
	if (RC == LW_RETURN_OK)
	{
		triggerMode = mode;
	}

	return RC;
}

LWReturnCode DeviceHandle::GetTriggerMode(LWTriggerMode& mode)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>", sn.c_str());

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	CommandFrame command{ C_GetTriggerMode };

	auto RC = ExecuteCommand(command);
	if (RC == LW_RETURN_OK)
	{
		mode = LWTriggerMode(command.getArgField()[0]);
		triggerMode = mode;
	}

	return RC;
}

LWReturnCode DeviceHandle::SetExposureMode(LWSensorType sensorType, LWExposureMode mode)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>, SensorType: %u, Mode: %u", sn.c_str(), sensorType, mode);

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	CommandFrame command;

	if (sensorType == LWSensorType::LW_TOF_SENSOR)
	{
        command = CommandFrame{ C_SetIntegralTime };

		uint16_t data[2] = {0, 0};
		if (mode == LW_EXPOSURE_AUTO)
		{
			command.setArgField(&data[0], 2);
		}
		else
		{
			data[0] = htons(1000);
			data[1] = htons(800);
			command.setArgField(&data, 4);
		}
	}
	else if (sensorType == LWSensorType::LW_RGB_SENSOR)
	{
		command = CommandFrame{ C_SetRgbCameraExposureMode };

		uint8_t val = (mode == LW_EXPOSURE_AUTO) ? 0 : 1;
		command.setArgField(&val, 1);
	}
	else
	{
		return LW_RETURN_TYPE_NOT_EXIST;
	}

	return ExecuteCommand(command);
}

LWReturnCode DeviceHandle::GetExposureMode(LWSensorType sensorType, LWExposureMode& mode)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>", sn.c_str());

	if (!openEnable.load()) return LW_RETURN_UNOPENED;
	
	CommandFrame command;
	if (sensorType == LWSensorType::LW_TOF_SENSOR)
	{
		command.setCommand(C_GetIntegralModel);
	}
	else if (sensorType == LWSensorType::LW_RGB_SENSOR)
	{
		command.setCommand(C_GetRgbCameraExposureMode);
	}
	else
	{
		return LW_RETURN_TYPE_NOT_EXIST;
	}

	auto RC = ExecuteCommand(command);
	if (RC == LW_RETURN_OK)
	{
		mode = command.getArgField()[0] == 0x00 ? LW_EXPOSURE_AUTO : LW_EXPOSURE_MANUAL;
	}

	return RC;

}

LWReturnCode DeviceHandle::SetFrequencyMode(LWFrequencyMode mode)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>, Mode: %u", sn.c_str(), mode);

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	CommandFrame command{ C_SetFrequencyModel };

	uint8_t val = mode;
	command.setArgField(&val, 1);

	return ExecuteCommand(command);
}

LWReturnCode DeviceHandle::GetFrequencyMode(LWFrequencyMode& mode)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>", sn.c_str());

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	CommandFrame command{ C_GetFrequencyModel };

	auto RC = ExecuteCommand(command);
	if (RC == LW_RETURN_OK)
	{
		mode = LWFrequencyMode(command.getArgField()[0]);
	}

	return RC;
}

LWReturnCode DeviceHandle::SetHDRModeEnable(bool enable)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>, Enable: %d", sn.c_str(), enable);

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	CommandFrame command{ C_SetHDR };

	command.setArgField(&enable, 1);

	return ExecuteCommand(command);
}

LWReturnCode DeviceHandle::GetHDRModeEnable(bool& enable)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>", sn.c_str());

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	CommandFrame command{ C_GetHDR };

	auto RC = ExecuteCommand(command);
	if (RC == LW_RETURN_OK)
	{
		enable = command.getArgField()[0];
	}

	return RC;
}

LWReturnCode DeviceHandle::SetTransformRgbToDepthEnable(bool enable)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>, Enable: %d", sn.c_str(), enable);

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	if (enable)
	{
		if (!isR2DEnable.load())
		{
			isR2DEnable.store(true);

			int start_row = 0;
			int end_cow = 0;
			int step = tofHeight / THREAD_POOL_SIZE;
			for (int i = 0; i < THREAD_POOL_SIZE; i++)
			{
				start_row = i * step;
				end_cow = (i < THREAD_POOL_SIZE - 1) ? (start_row + step) : tofHeight;
				std::thread{ &DeviceHandle::rgbToDepthThread, this, i, start_row, end_cow }.detach();
			}
		}
	}
	else
	{
		if (isR2DEnable.load())
		{
			isR2DEnable.store(false);
			r2dNotify.notify_all();
		}
	}

	return LW_RETURN_OK;
}

LWReturnCode DeviceHandle::SetTransformDepthToRgbEnable(bool enable)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>, Enable: %d", sn.c_str(), enable);

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	if (enable)
	{
		if (!isD2REnable.load())
		{
			isD2REnable.store(true);

			int start_row = 0;
			int end_cow = 0;
			int step = tofHeight / THREAD_POOL_SIZE;
			for (int i = 0; i < THREAD_POOL_SIZE; i++)
			{
				start_row = i * step;
				end_cow = (i < THREAD_POOL_SIZE - 1) ? (start_row + step) : tofHeight;
				std::thread{ &DeviceHandle::depthToRgbThread, this, i, start_row, end_cow }.detach();
			}
		}
	}
	else
	{
		if (isD2REnable.load())
		{
			isD2REnable.store(false);
			d2rNotify.notify_all();
		}
	}

	return LW_RETURN_OK;
}

LWReturnCode DeviceHandle::SetFrameRate(int32_t value)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>, Value: %d", sn.c_str(), value);

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	if (tofHeight == TOF_MAX_PIX_ROWS)
	{
		if (value > 20 || value < 1)
		{
			return LW_RETURN_ARG_OUT_OF_RANGE;
		}
		if (value == 20) value = 25;
	}
	else
	{
		if (value > 30 || value < 1)
		{
			return LW_RETURN_ARG_OUT_OF_RANGE;
		}
	}

	CommandFrame command{ C_SetFrameRate };
	auto val = uint8_t(value);
	command.setArgField(&val, 1);

	return ExecuteCommand(command);
}

LWReturnCode DeviceHandle::GetFrameRate(int32_t& value)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>", sn.c_str());

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	CommandFrame command{ C_GetFrameRate };

	auto RC = ExecuteCommand(command);
	if (RC == LW_RETURN_OK)
	{
		value = uint8_t(command.getArgField()[0]);
	}

	return RC;
}

LWReturnCode DeviceHandle::SetExposureTime(LWSensorType sensorType, const int32_t* exposureTimeArray, int32_t arraySize)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	std::string str;
	for (int i = 0; i < arraySize; ++i) str += (std::to_string(exposureTimeArray[i]) + " ");
	LOG_INFO_OUT("<%s>, SensorType: %u, Value: %s", sn.c_str(), sensorType, str.c_str());

	if (!openEnable.load()) return LW_RETURN_UNOPENED;
	if (arraySize < 1 || arraySize > 2)
	{
		gGlobal.errorInfo = "The number of parameters exceeds the range.";
		return LW_RETURN_CUSTOM_ERROR;
	}

	uint16_t data[10];
	CommandFrame command;

	if (sensorType == LWSensorType::LW_TOF_SENSOR)
	{
		if (exposureTimeArray[0] == 0)
		{
			command.setCommand(C_SetSingleIntegral);
			return ExecuteCommand(command);
		}

		for (int i = 0; i < arraySize; i++)
		{
			if (exposureTimeArray[i] > 4000 || exposureTimeArray[i] < 0) return LW_RETURN_ARG_OUT_OF_RANGE;
			data[i] = htons(uint16_t(exposureTimeArray[i]));
		}

		if (arraySize == 1)
		{
			arraySize = 2;
			data[1] = htons(uint16_t(exposureTimeArray[0] * 0.9));
		}

		command.setCommand(C_SetIntegralTime);
	}
	else if (sensorType == LWSensorType::LW_RGB_SENSOR)
	{
		for (int i = 0; i < arraySize; i++)
		{
			if (exposureTimeArray[i] > 10000 || exposureTimeArray[i] < 1) return LW_RETURN_ARG_OUT_OF_RANGE;
			data[i] = htons(uint16_t(exposureTimeArray[i]));
		}

		command.setCommand(C_SetRgbCameraExposureTime);
	}
	else
	{
		return LW_RETURN_TYPE_NOT_EXIST;
	}

	command.setArgField(data, arraySize * 2);

	return ExecuteCommand(command);
}

LWReturnCode DeviceHandle::GetExposureTime(LWSensorType sensorType, int32_t* exposureTimeArray, int32_t arraySize, int32_t& filledCount)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>, SensorType: %u", sn.c_str(), sensorType);

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	if (arraySize < 1)
	{
		gGlobal.errorInfo = "The number of parameters exceeds the range.";
		return LW_RETURN_CUSTOM_ERROR;
	}
	
	CommandFrame command;
	if (sensorType == LWSensorType::LW_RGB_SENSOR)
	{
		command.setCommand(C_GetRgbCameraExposureTime);
	}
	else if (sensorType == LWSensorType::LW_TOF_SENSOR)
	{
		command.setCommand(C_GetIntegralTime);
	}
	else
	{
		return LW_RETURN_TYPE_NOT_EXIST;
	}

	auto RC = ExecuteCommand(command);
	if (RC == LW_RETURN_OK)
	{
		int len = command.getArgFieldLength() / 2;
		if (arraySize < len) return LW_RETURN_OUT_OF_MEMORY;

		auto data = (uint16_t*)(command.getArgField());
		for (int i = 0; i < len; i++)
		{
			exposureTimeArray[i] = ntohs(data[i]);
		}

		filledCount = len;
	}

	return RC;
}

LWReturnCode DeviceHandle::SetResolution(LWSensorType sensorType, int32_t width, int32_t height)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>, SensorType: %u, Width: %d, Height: %d", sn.c_str(), sensorType, width, height);

	if (!openEnable.load()) return LW_RETURN_UNOPENED;
	return LW_RETURN_NOT_SUPPORTED;

}

LWReturnCode DeviceHandle::GetResolution(LWSensorType type, int32_t &width, int32_t &height)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>, SensorType: %u", sn.c_str(), type);

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

    if (type == LWSensorType::LW_RGB_SENSOR)
    {
        CommandFrame command{ C_GetImageInfo };

        auto RC = ExecuteCommand(command);
        if (RC == LW_RETURN_OK)
        {
            auto data = command.getArgField();

            hasRgbModule.store(bool(data[0]));

            if (!hasRgbModule.load()) return LW_RETURN_TYPE_NOT_EXIST;

            width = ntohs(*((uint16_t*)(data + 1)));
            height = ntohs(*((uint16_t*)(data + 3)));
        }

        return RC;
    }
    else if (type == LWSensorType::LW_TOF_SENSOR)
    {
		CommandFrame command{ C_GetBinning };

		auto RC = ExecuteCommand(command);
		if (RC == LW_RETURN_OK)
		{
			auto data = command.getArgField();

			if (data[0] == 0x00)
			{
				width = 640;
				height = 480;
			}
			else if (data[0] == 0x01)
			{
				width = 320;
				height = 240;
			}
			else if (data[0] == 0x02)
			{
				width = 160;
				height = 120;
			}
			else
			{
				gGlobal.errorInfo = "获取的\"Binning\"模式不在解析范围!";
				return LW_RETURN_CUSTOM_ERROR;
			}
		}

		return RC;
    }

    return LW_RETURN_NOT_SUPPORTED;
}

LWReturnCode DeviceHandle::SetTimeFilterParams(const LWFilterParam& param)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>, Value: [%d, %d]", sn.c_str(), param.enable, param.threshold);

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	if (param.threshold < 1 || param.threshold > 5) return LW_RETURN_ARG_OUT_OF_RANGE;

	uint8_t arg[2] = { param.enable, (uint8_t)param.threshold };
	CommandFrame command{ C_SetTimeFilter };
	command.setArgField(&arg, 2);

	auto ret = ExecuteCommand(command);
	if (ret == LW_RETURN_OK)
	{
		timeFilter = param;
	}

	return ret;
}

LWReturnCode DeviceHandle::GetTimeFilterParams(LWFilterParam& param)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>", sn.c_str());

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

    CommandFrame command{ C_GetTimeFilter };

    auto RC = ExecuteCommand(command);
    if (RC == LW_RETURN_OK)
    {
        auto ptr = command.getArgField();
		param.enable = bool(ptr[0]);
		param.threshold = uint8_t(ptr[1]);
		param.k1 = 0;
		param.k2 = 0;
    }

    return RC;
}

LWReturnCode DeviceHandle::SetFlyingPixelsFilterParams(const LWFilterParam& param)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>, Value: [%d, %d]", sn.c_str(), param.enable, param.threshold);

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	if (param.threshold < 1 || param.threshold > 64) return LW_RETURN_ARG_OUT_OF_RANGE;

	uint8_t arg[2] = { param.enable, (uint8_t)param.threshold };
	CommandFrame command{ C_SetFlyPixeFilter };
	command.setArgField(&arg, 2);

	auto ret = ExecuteCommand(command);
	if (ret == LW_RETURN_OK)
	{
		flyingPixelsFilter = param;
	}

	return ret;
}

LWReturnCode DeviceHandle::GetFlyingPixelsFilterParams(LWFilterParam& param)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>", sn.c_str());

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

    CommandFrame command{ C_GetFlyPixeFilter };

    auto RC = ExecuteCommand(command);
    if (RC == LW_RETURN_OK)
    {
        auto ptr = command.getArgField();
		param.enable = bool(ptr[0]);
		param.threshold = uint8_t(ptr[1]);
		param.k1 = 0;
		param.k2 = 0;
    }

    return RC;
}

LWReturnCode DeviceHandle::SetConfidenceFilterParams(const LWFilterParam& param)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>, Value: [%d, %d]", sn.c_str(), param.enable, param.threshold);

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	if (param.threshold < 1 || param.threshold > 150) return LW_RETURN_ARG_OUT_OF_RANGE;

	uint8_t arg[2] = { param.enable, (uint8_t)param.threshold };
	CommandFrame command{ C_SetConfidenceFilter };
	command.setArgField(&arg, 2);

	auto ret = ExecuteCommand(command);
	if (ret == LW_RETURN_OK)
	{
		confidenceFilter = param;
	}

	return ret;
}

LWReturnCode DeviceHandle::GetConfidenceFilterParams(LWFilterParam& param)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>", sn.c_str());

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

    CommandFrame command{ C_GetConfidenceFilter };

    auto RC = ExecuteCommand(command);
    if (RC == LW_RETURN_OK)
    {
        auto ptr = command.getArgField();
		param.enable = bool(ptr[0]);
		param.threshold = uint8_t(ptr[1]);
		param.k1 = 0;
		param.k2 = 0;
    }

    return RC;
}

LWReturnCode DeviceHandle::SetSpatialFilterParams(const LWFilterParam& param)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>, Value: [%d, %d]", sn.c_str(), param.enable, param.threshold);

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	if (std::unordered_set<int>{3, 5, 7}.count(param.threshold) == 0) return LW_RETURN_ARG_OUT_OF_RANGE;
	//if (param.threshold != 3 && param.threshold != 5 && param.threshold != 7) return LW_RETURN_ARG_OUT_OF_RANGE;

	uint8_t arg[2] = { param.enable, (uint8_t)param.threshold };
	CommandFrame command{ C_SetSpatialFilter };
	command.setArgField(&arg, 2);

	auto ret = ExecuteCommand(command);
	if (ret == LW_RETURN_OK)
	{
		spatialFilter = param;
	}

	return ret;
}

LWReturnCode DeviceHandle::GetSpatialFilterParams(LWFilterParam& param)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>", sn.c_str());

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

    CommandFrame command{ C_GetSpatialFilter };

    auto RC = ExecuteCommand(command);
    if (RC == LW_RETURN_OK)
    {
        auto ptr = command.getArgField();
		param.enable = bool(ptr[0]);
		param.threshold = uint8_t(ptr[1]);
		param.k1 = 0;
		param.k2 = 0;
    }

    return RC;
}

LWReturnCode DeviceHandle::SetTimeMedianFilterParams(const LWFilterParam& param)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>, Value: [%d, %d, %d]", sn.c_str(), param.enable, param.threshold, param.k1);

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	//if (param.k1 < 1 || param.k1 > 10000) return LW_RETURN_ARG_OUT_OF_RANGE;
	if (std::unordered_set<int>{3, 5, 7, 9}.count(param.threshold) == 0) return LW_RETURN_ARG_OUT_OF_RANGE;
	//if (param.threshold != 3 && param.threshold != 5 && param.threshold != 7 && param.threshold != 9) return LW_RETURN_ARG_OUT_OF_RANGE;

	uint8_t arg[2] = { param.enable, (uint8_t)param.threshold };
	CommandFrame command{ C_SetTimeMedianFilter };
	command.setArgField(&arg, 2);

	auto ret = ExecuteCommand(command);
	if (ret == LW_RETURN_OK)
	{
		timeMedianFilter = param;
		timeMedianFilter.k1 = 100;
	}

	return ret;
}

LWReturnCode DeviceHandle::GetTimeMedianFilterParams(LWFilterParam& param)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>", sn.c_str());

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

    CommandFrame command{ C_GetTimeMedianFilter };

    auto RC = ExecuteCommand(command);
    if (RC == LW_RETURN_OK)
    {
        auto ptr = command.getArgField();
		param.enable = bool(ptr[0]);
		param.threshold = uint8_t(ptr[1]);
		param.k1 = timeMedianFilter.k1;
		param.k2 = 0;
    }

    return RC;
}

LWReturnCode DeviceHandle::SetIRGMMGain(int32_t value)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>, Value: %d", sn.c_str(), value);

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	if (value < 0 || value > 255) return LW_RETURN_ARG_OUT_OF_RANGE;

	irGMMGain = value;

	uint8_t val = value;
	CommandFrame command{ C_SetIRGMMGain };
	command.setArgField(&val, 1);

	return ExecuteCommand(command);
}

LWReturnCode DeviceHandle::GetIRGMMGain(int32_t &gain)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>", sn.c_str());

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

    CommandFrame command{ C_GetIRGMMGain };

    auto RC = ExecuteCommand(command);
    if (RC == LW_RETURN_OK)
    {
        gain = uint8_t(command.getArgField()[0]);
    }

    return RC;
}

LWReturnCode DeviceHandle::SetRgbSensorGain(int32_t value)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>, Value: %d", sn.c_str(), value);

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	if (value > 1023 || value < 0) return LW_RETURN_ARG_OUT_OF_RANGE;

	CommandFrame command{ C_SetRgbCameraGain };

	uint16_t var = value;
	var = htons(var);
	command.setArgField(&var, 2);

	return ExecuteCommand(command);
}

LWReturnCode DeviceHandle::GetRgbSensorGain(int32_t& value)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>", sn.c_str());

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	CommandFrame command{ C_GetRgbCameraGain };

	auto RC = ExecuteCommand(command);
	if (RC == LW_RETURN_OK)
	{
		value = ntohs(*((unsigned short*)command.getArgField()));
	}

	return RC;
}

LWReturnCode DeviceHandle::SetRgbSensorGamma(int32_t value)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>, Value: %d", sn.c_str(), value);

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	if (value > 300 || value < 64) return LW_RETURN_ARG_OUT_OF_RANGE;

	CommandFrame command{ C_SetRgbCameraGamma };

	uint16_t var = htonl(value);
	command.setArgField(&var, 2);

	return ExecuteCommand(command);
}

LWReturnCode DeviceHandle::GetRgbSensorGamma(int32_t& value)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>", sn.c_str());

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	CommandFrame command{ C_GetRgbCameraGamma };

	auto RC = ExecuteCommand(command);
	if (RC == LW_RETURN_OK)
	{
		value = ntohs(*((unsigned short*)command.getArgField()));
	}

	return RC;
}

LWReturnCode DeviceHandle::SetRgbSensorBrightness(int32_t value)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>, Value: %d", sn.c_str(), value);

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	if (value > 64 || value < -64) return LW_RETURN_ARG_OUT_OF_RANGE;

	CommandFrame command{ C_SetRgbCameraBrightness };

	uint32_t var = htonl(value);
	command.setArgField(&var, 4);

	return ExecuteCommand(command);
}

LWReturnCode DeviceHandle::GetRgbSensorBrightness(int32_t& value)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>", sn.c_str());

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	CommandFrame command{ C_GetRgbCameraBrightness };

	auto RC = ExecuteCommand(command);
	if (RC == LW_RETURN_OK)
	{
		value = ntohl(*((int32_t*)command.getArgField()));
	}

	return RC;
}

LWReturnCode DeviceHandle::SetRgbSensorContrastRatio(int32_t value)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>, Value: %d", sn.c_str(), value);

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	if (value > 95 || value < 0) return LW_RETURN_ARG_OUT_OF_RANGE;

	CommandFrame command{ C_SetRgbCameraContrastRatio };

	uint8_t var = value;
	command.setArgField(&var, 1);

	return ExecuteCommand(command);
}

LWReturnCode DeviceHandle::GetRgbSensorContrastRatio(int32_t& value)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>", sn.c_str());

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	CommandFrame command{ C_GetRgbCameraContrastRatio };

	auto RC = ExecuteCommand(command);
	if (RC == LW_RETURN_OK)
	{
		value = (uint8_t)command.getArgField()[0];
	}

	return RC;
}

LWReturnCode DeviceHandle::SetNetworkInfo(const LWNetworkInfo& info)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>, Type: %d, IP: %s, Netmask: %s", sn.c_str(), info.type, info.ip, info.netmask);

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	CommandFrame command{ C_SetNetworkModel };

	uint8_t data[9]{};
	data[0] = info.type;
	if (info.type == 0x01)
	{
		if (inet_pton(AF_INET, info.ip, data + 1) != 1 || inet_pton(AF_INET, info.netmask, data + 5) != 1)
		{
			gGlobal.errorInfo = getNetworkLastError();
			return LW_RETURN_NETWORK_ERROR;
		}
	}
	command.setArgField(data, 9);

	return ExecuteCommand(command);
}

LWReturnCode DeviceHandle::GetNetworkInfo(LWNetworkInfo& info)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>", sn.c_str());

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	CommandFrame command{ C_GetNetworkModel };

	auto RC = ExecuteCommand(command);
	if (RC == LW_RETURN_OK)
	{
		auto data = command.getArgField();
		info.type = data[0];
		if (data[0] == 0x01)
		{
			auto ret = inet_ntop(AF_INET, data + 1, info.ip, 16);
			if (ret == nullptr) return LW_RETURN_NETWORK_ERROR;

			ret = inet_ntop(AF_INET, data + 5, info.netmask, 16);
			if (ret == nullptr) return LW_RETURN_NETWORK_ERROR;
		}
	}

	return RC;
}

LWReturnCode DeviceHandle::SetRgbDataTransportFormat(LWRgbTransferFormat format)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>, Format: %u", sn.c_str(), format);

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	CommandFrame command{ C_SetRgbCameraDataFormat };

	uint8_t val = format;

	command.setArgField(&val, 1);

	return ExecuteCommand(command);
}

LWReturnCode DeviceHandle::GetRgbDataTransportFormat(LWRgbTransferFormat& format)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>", sn.c_str());

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	CommandFrame command{ C_GetRgbCameraDataFormat };

	auto RC = ExecuteCommand(command);
	if (RC == LW_RETURN_OK)
	{
		format = LWRgbTransferFormat(command.getArgField()[0]);
	}

	return RC;
}

LWReturnCode DeviceHandle::SetDeviceNumber(int32_t value)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>, Value: %d", sn.c_str(), value);

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	if (value > 255 || value < 1) return LW_RETURN_ARG_OUT_OF_RANGE;

	CommandFrame command{ C_SetCameraNumber };

	uint8_t var = value;
	command.setArgField(&var, 1);

	return ExecuteCommand(command);
}

LWReturnCode DeviceHandle::GetDeviceNumber(int32_t& value)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>", sn.c_str());

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	CommandFrame command{ C_GetCameraNumber };

	auto RC = ExecuteCommand(command);
	if (RC == LW_RETURN_OK)
	{
		value = uint8_t(command.getArgField()[0]);
	}

	return RC;
}

LWReturnCode DeviceHandle::SetHardTriggerFilterParams(int32_t t1, int32_t t2)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>, Value: [%d, %d]", sn.c_str(), t1, t2);

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	if (t1 < 1000 || t2 < 50 || t1 > 65535 || t2 > 65535) return LW_RETURN_ARG_OUT_OF_RANGE;

	CommandFrame command{ C_SetDelayHardTriggerTime };

	uint16_t data[2] = { htons((uint16_t)t1),  htons((uint16_t)t2) };
	command.setArgField(data, 4);

	return ExecuteCommand(command);
}

LWReturnCode DeviceHandle::GetHardTriggerFilterParams(int32_t& t1, int32_t& t2)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>", sn.c_str());

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	CommandFrame command{ C_GetDelayHardTriggerTime };

	auto RC = ExecuteCommand(command);
	if (RC == LW_RETURN_OK)
	{
		auto ptr = (unsigned short*)command.getArgField();
		t1 = ntohs(ptr[0]);
		t2 = ntohs(ptr[1]);
	}

	return RC;
}

LWReturnCode DeviceHandle::GetIntrinsicParam(LWSensorType sensorType, LWSensorIntrinsicParam& intrinsicParam)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>, SensorType: %u", sn.c_str(), sensorType);

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	CommandFrame command;

	if (sensorType == LWSensorType::LW_TOF_SENSOR)
	{
		command = CommandFrame{ C_GetTofCameraIntrinsicArg };
	}
	else if (sensorType == LWSensorType::LW_RGB_SENSOR)
	{
		command = CommandFrame{ C_GetRgbCameraIntrinsicArg };
	}
	else
	{
		return LW_RETURN_TYPE_NOT_EXIST;
	}

	auto RC = ExecuteCommand(command);
	if (RC == LW_RETURN_OK)
	{
		auto ptr1 = (double*)command.getArgField();
		auto ptr2 = (float*)&intrinsicParam;
		for (int i = 0; i < 7; i++)
		{
			*ptr2++ = float(*ptr1++);
		}
	}

	return RC;
}

LWReturnCode DeviceHandle::GetExtrinsicParam(LWSensorType sensorType, LWSensorExtrinsicParam& extrinsicParam)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>, SensorType: %u", sn.c_str(), sensorType);

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	CommandFrame command;

	if (sensorType == LWSensorType::LW_RGB_SENSOR)
	{
		command = CommandFrame{ C_GetRgbCameraExtrinsicArg };
	}
	else
	{
		return LW_RETURN_TYPE_NOT_EXIST;
	}

	auto RC = ExecuteCommand(command);
	if (RC == LW_RETURN_OK)
	{
		auto ptr1 = (double*)command.getArgField();
		auto ptr2 = (float*)&extrinsicParam;
		for (int i = 0; i < 12; i++)
		{
			*ptr2++ = float(*ptr1++);
		}
	}

	return RC;
}

LWReturnCode DeviceHandle::GetDeviceSN(char* _sn_, int32_t bufferLen)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>", sn.c_str());

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	CommandFrame command{ C_GetSN };

	auto RC = ExecuteCommand(command);
	if (RC != LW_RETURN_OK) return RC;
	auto len = command.getArgFieldLength();
	if (len > bufferLen) return LW_RETURN_OUT_OF_MEMORY;

	auto _sn = std::string(command.getArgField(), len);

	if (sn != _sn)
	{
		gGlobal.strDeviceMap[_sn] = gGlobal.strDeviceMap[sn];
		gGlobal.numDeviceMap[handle] = gGlobal.strDeviceMap[sn];
		gGlobal.strDeviceMap.erase(sn);
		sn = _sn;
	}

	memcpy(_sn_, _sn.c_str(), len);
	if (len != bufferLen) _sn_[len] = '\0';
	return LW_RETURN_OK;

}

LWReturnCode DeviceHandle::GetDeviceType(char* type, int32_t bufferLen)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>", sn.c_str());

	if (!openEnable.load()) return LW_RETURN_UNOPENED;
	//return LW_RETURN_NOT_SUPPORTED;

	CommandFrame command{ C_GetDeviceType };

	auto RC = ExecuteCommand(command);
	if (RC != LW_RETURN_OK) return RC;
	auto len = command.getArgFieldLength();
	if (len > bufferLen) return LW_RETURN_OUT_OF_MEMORY;

	memcpy(type, command.getArgField(), len);
	if (len != bufferLen) type[len] = '\0';
	return LW_RETURN_OK;
}

LWReturnCode DeviceHandle::GetTimeStamp(LWTimeStamp& t)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>", sn.c_str());

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	CommandFrame command{ C_GetTimeStamp };

	auto RC = ExecuteCommand(command);
	if (RC == LW_RETURN_OK)
	{
		auto ptr = command.getArgField();
		t.tv_sec = *((uint32_t*)ptr);
		t.tv_usec = *((uint32_t*)(ptr + 4));
	}

	return RC;
}

LWReturnCode DeviceHandle::GetDeviceVersion(LWVersionInfo& fv, LWVersionInfo& dv)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>", sn.c_str());

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	CommandFrame command{ C_GetVersion };

	auto RC = ExecuteCommand(command);
	if (RC == LW_RETURN_OK)
	{
		auto data = (uint8_t*)command.getArgField();

		fv.major = data[0];
		fv.minor = data[1];
		fv.patch = data[2];
		fv.reserved = data[3];

		dv.major = data[4];
		dv.minor = data[5];
		dv.patch = data[6];
		dv.reserved = data[7];
	}

	return RC;
}

LWReturnCode DeviceHandle::GetFrameReady()
{
	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	std::unique_lock<std::mutex> lock{ syncDataMutex };
	if (syncDataNotify.wait_for(lock, std::chrono::milliseconds{ timeout }, [this] { return readyEnable.load() || !aliveEnable.load(); }))
	{
		if (!aliveEnable.load()) return LW_RETURN_UNINITIALIZED;
		if (rgbNode->update) std::swap(rgbNode, rgbCutNode);
		if (tofNode->update) std::swap(tofNode, tofCutNode);
		isReady = readyEnable.load();
		readyEnable.store(false);

		return LW_RETURN_OK;
	}

	return LW_RETURN_TIMEOUT;
}


LWReturnCode DeviceHandle::UpdateFirmware(const char* filename)
{
	std::unique_lock<std::mutex> lock{ syncDataMutex };

	LOG_INFO_OUT("<%s>", sn.c_str());

	UpdateToolTask task(inet_ntoa(remoteAddr.sin_addr), 22, "root", "lv-zgyfjch", filename, 3000);
	UpdateResultType result = task.processor();

	if (result == RESULT_OK)
	{
		return LW_RETURN_OK;
	}
	else
	{
		gGlobal.errorInfo = result.desc;
		return LW_RETURN_FIRMWARE_UPDATE_FAIL;
	}
}

#ifdef LW_INTERNAL_API
LWReturnCode DeviceHandle::SendFile(const char* fullname, LWFileType type)
{
	LOG_INFO_OUT("<%s>, FileType: %d, Filename: %s", sn.c_str(), type, fullname);

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	// 判定文件是否存在
	//if(access(fullname, 0) != 0) return LW_RETURN_FILE_NOT_EXIST;

	// 获取文件状态信息
	struct stat statbuf {};
	stat(fullname, &statbuf);

	// 数据初始化
	LWReturnCode		returncode = LW_RETURN_NETWORK_ERROR;
	uint32_t			retransmit = 3;//重传次数
	uint32_t 			file_size = statbuf.st_size;
	AutoMemoryManager 	dat{ 65535 };
	auto 				buf = dat.u_data();
	CommandFrame 		command{ C_FileArgTransfer };

	command.setVersion(0x01);
	buf[0] = type; // 文件类型
	uint32_t val32 = htonl(file_size);
	memcpy(buf + 1, &val32, 4);				// 文件长度
	calculateMD5(fullname, buf + 5, val32); // 文件DM5信息
	if (type != LW_OTHER)
	{
		command.setArgField(buf, 21);
	}
	else
	{
		auto fileName = getFileName(fullname);
		memcpy(buf + 21, fileName.data(), fileName.size()); // 文件名
		command.setArgField(buf, 21 + fileName.size());
	}

	// 打开文件
	FILE* fp = fopen(fullname, "rb");
	if (fp == nullptr) return LW_RETURN_FILE_OPEN_ERROR;

	// 建立网络通讯
	sockaddr_in send_addr = remoteAddr;
	send_addr.sin_port = htons(64110);
	SOCKET _socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if ((bind(_socket, (sockaddr*)(&localAddr), sizeof(localAddr)) == SOCKET_ERROR) || !setNetworkTimeout(_socket, timeout))
	{
		goto LABE1;
	}

	// 发送数据
	for (uint32_t i = 0; i < retransmit; i++)
	{
		if ((sendto(_socket, command.data(), command.size(), 0, (sockaddr*)&send_addr, sizeof(send_addr)) == SOCKET_ERROR) 
			|| (recvfrom(_socket, command.data(), command.maxBuffer(), 0, nullptr, nullptr) == SOCKET_ERROR))
		{
			continue;
		}

		if (command.isCommand(C_FileArgTransfer))
		{
			uint16_t count = 1;
			uint32_t residue_size = file_size;
			uint32_t block_size = file_size < 50000u ? file_size : 50000u;

			command.setCommand(C_FileDataTransfer);
			command.setTotalSerialNumber((file_size % block_size > 0) ? (file_size / block_size + 1) : (file_size / block_size));

			while (fread(buf, block_size, 1, fp) != 0)
			{
				command.setCurrentSerialNumber(count++);
				command.setArgField(buf, block_size);

				for (uint32_t ii1 = 1;; ii1++)
				{
					if ((sendto(_socket, command.data(), command.size(), 0, (sockaddr*)&send_addr, sizeof(send_addr)) == SOCKET_ERROR) 
						|| (recvfrom(_socket, command.data(), command.maxBuffer(), 0, nullptr, nullptr) == SOCKET_ERROR))
					{
						if (ii1 > retransmit)
							goto LABE1;
						continue;
					}

					if (command.isCommand(C_FileDataTransfer))
						break;

					if (ii1 > retransmit)
					{
						returncode = LW_RETURN_COMMAND_ERROR;
						goto LABE2;
					}
				}

				residue_size -= block_size;

				if (residue_size < 1)
					break;

				if (residue_size < block_size)
					block_size = residue_size;
			}

			// 校验文件信息
			command.setCommand(C_FileAckTransfer);
			command.setCurrentSerialNumber(1);
			command.setTotalSerialNumber(1);
			command.setArgField(nullptr, 0);
			for (uint8_t ii2 = 1;; ii2++)
			{
				if ((sendto(_socket, command.data(), command.size(), 0, (sockaddr*)&send_addr, sizeof(send_addr)) == SOCKET_ERROR) 
					|| (recvfrom(_socket, command.data(), command.maxBuffer(), 0, nullptr, nullptr) == SOCKET_ERROR))
				{
					if (ii2 > retransmit)
						goto LABE1;
					continue;
				}

				if (command.isCommand(C_FileAckTransfer))
				{
					returncode = command.getCommandType() == 0x02 ? LW_RETURN_OK : LWReturnCode(command.getCommandType());
					goto LABE2;
				}

				if (ii2 > retransmit)
				{
					returncode = LW_RETURN_COMMAND_ERROR;
					goto LABE2;
				}
			}
		}

	}

LABE1:
	gGlobal.errorInfo = getNetworkLastError();

LABE2:
	fclose(fp);
	closeSocket(_socket);

	return returncode;

}

LWReturnCode DeviceHandle::SetDeviceSN(const char* sn_, int size)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>, SN: %s", sn.c_str(), sn_);

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	CommandFrame command{ C_SetSN };
	command.setArgField(sn_, size);

	auto sn_str = std::string(sn_, size);
	auto ret = ExecuteCommand(command);
	if ((ret == LW_RETURN_OK) && (sn != sn_str))
	{
		gGlobal.strDeviceMap[sn_str] = gGlobal.strDeviceMap[sn];
		gGlobal.numDeviceMap[handle] = gGlobal.strDeviceMap[sn];
		gGlobal.strDeviceMap.erase(sn);
		sn = sn_str;
	}
	return ret;
}

LWReturnCode DeviceHandle::SendOperateCommand(const char* comstr, int size)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>, Value: %s", sn.c_str(), comstr);

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	CommandFrame command{ C_OperateCommand };

	command.setArgField(comstr, size + 1);

	return ExecuteCommand(command);
}

LWReturnCode DeviceHandle::SetBinningMode(LWBinningMode mode)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>, Mode: %u", sn.c_str(), mode);

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	CommandFrame command{ C_SetBinning };

	uint8_t val = mode;
	command.setArgField(&val, 1);

	return ExecuteCommand(command);
}

LWReturnCode DeviceHandle::SetDRNU(bool enable)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>, Enable: %d", sn.c_str(), enable);

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	CommandFrame command{ C_SetDRNU };

	command.setArgField(&enable, 1);

	return ExecuteCommand(command);
}

LWReturnCode DeviceHandle::SetDistortionCalibration(LWSensorType sensorType, bool enable)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>, Enable: %d", sn.c_str(), enable);

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	if (sensorType == LWSensorType::LW_TOF_SENSOR)
	{
		CommandFrame command{ C_SetDistortion };

		command.setArgField(&enable, 1);

		return ExecuteCommand(command);
	}
	else if (sensorType == LWSensorType::LW_RGB_SENSOR)
	{
		rgbDistortionEnable = enable;

		return LW_RETURN_OK;
	}
	else
	{
		return LW_RETURN_TYPE_NOT_EXIST;
	}
	
}

LWReturnCode DeviceHandle::SetLaserWorkFrequency(const uint8_t* arr, int size)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	std::string str;
	for (int i = 0; i < size; ++i) str += std::to_string(arr[i]) + " ";
	LOG_INFO_OUT("<%s>, Value: %s", sn.c_str(), str.c_str());

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	if (size < 1)
	{
		gGlobal.errorInfo = "The number of parameters exceeds the range.";
		return LW_RETURN_CUSTOM_ERROR;
	}

	CommandFrame command{ C_SetLaserWorkFrequency };

	command.setArgField(arr, size);

	return ExecuteCommand(command);
}

LWReturnCode DeviceHandle::SetAutoExposureDefaultValue(uint16_t val)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>, Value: %u", sn.c_str(), val);

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	CommandFrame command{ C_SetExposureValue };

	command.setArgField(&val, 2);

	return ExecuteCommand(command);
}

LWReturnCode DeviceHandle::SetIntrinsicParam(LWSensorType sensorType, const LWSensorIntrinsicParam& para)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	std::string str;
	auto* ptr = (float*)&para;
	for (int i = 0; i < 7; ++i) str += std::to_string(ptr[i]) + " ";
	LOG_INFO_OUT("<%s>, SensorType: %d, Value: %s", sn.c_str(), sensorType, str.c_str());

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	CommandFrame command;

	if (sensorType == LWSensorType::LW_TOF_SENSOR)
	{
		command = CommandFrame{ C_SetCameraIntrinsicArg };
	}
	else if (sensorType == LWSensorType::LW_RGB_SENSOR)
	{
		command = CommandFrame{ C_SetRgbCameraIntrinsicArg };
	}
	else
	{
		return LW_RETURN_TYPE_NOT_EXIST;
	}

	double val[7];
	for (int i = 0; i < 7; i++) val[i] = ptr[i];
	command.setArgField(val, 56);

	return ExecuteCommand(command);
}

LWReturnCode DeviceHandle::SetExtrinsicParam(LWSensorType sensorType, const LWSensorExtrinsicParam& para)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	std::string str;
	auto* ptr = (float*)&para;
	for (int i = 0; i < 12; ++i) str += std::to_string(ptr[i]) + " ";
	LOG_INFO_OUT("<%s>, SensorType: %d, Value: %s", sn.c_str(), sensorType, str.c_str());

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	CommandFrame command;

	if (sensorType == LWSensorType::LW_RGB_SENSOR)
	{
		command = CommandFrame{ C_SetRgbCameraExtrinsicArg };
	}
	else
	{
		return LW_RETURN_TYPE_NOT_EXIST;
	}

	double val[12];
	for (int i = 0; i < 12; i++) val[i] = ptr[i];
	command.setArgField(val, 96);

	return ExecuteCommand(command);
}

LWReturnCode DeviceHandle::SetTemperatureCompensation(bool enable)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>, Enable: %d", sn.c_str(), enable);

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	CommandFrame command{ C_SetTempCompensate };

	command.setArgField(&enable, 1);

	return ExecuteCommand(command);
}

LWReturnCode DeviceHandle::GetTemperatureCompensation(bool& enable)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>", sn.c_str());

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	CommandFrame command{ C_GetTempCompensate };

	auto RC = ExecuteCommand(command);
	if (RC == LW_RETURN_OK)
	{
		enable = command.getArgField()[0];
	}

	return RC;
}

LWReturnCode DeviceHandle::SetTemperatureParams(const LWTemperatureParams& para)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	std::string str;
	auto* ptr = (double*)&para;
	for (int i = 0; i < 4; ++i) str += std::to_string(ptr[i]) + " ";
	LOG_INFO_OUT("<%s>, Value: %s", sn.c_str(), str.c_str());

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	CommandFrame command{ C_SetTempCoefficient };

	command.setArgField(&para, sizeof(para));

	return ExecuteCommand(command);
}

LWReturnCode DeviceHandle::SetLaserEnableStatus(uint32_t flag)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>, Status: %u", sn.c_str(), flag);

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	CommandFrame command{ C_SetLaserEnableStatus };

	uint8_t val = flag;
	command.setArgField(&val, 1);

	return ExecuteCommand(command);
}

LWReturnCode DeviceHandle::GetLaserEnableStatus(uint32_t& flag)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>", sn.c_str());

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	CommandFrame command{ C_GetLaserEnableStatus };

	auto RC = ExecuteCommand(command);
	if (RC == LW_RETURN_OK)
	{
		flag = uint8_t(command.getArgField()[0]);
	}

	return RC;
}

LWReturnCode DeviceHandle::SetDataSyncEnable(bool enable)
{
	dataSyncEnable = enable;

	return LW_RETURN_OK;
}

#endif //LW_INTERNAL_API

/// ******************************************* API接口定义 *******************************************

std::mutex notReentryMutex;//不可重入锁（为了实时监控，命令的收发不在同一线程）


LWReturnCode LWInitializeResources()
{
	std::lock_guard<std::mutex> guard{ notReentryMutex };

	LOG_INFO_OUT("");

	gGlobal.initEnable.store(true);

    return LW_RETURN_OK;
}

LWReturnCode LWCleanupResources()
{
	std::lock_guard<std::mutex> guard{ notReentryMutex };

	LOG_INFO_OUT("");

	gGlobal.initEnable.store(false);

    for (auto & iter : gGlobal.strDeviceMap)
    {
        delete iter.second;
    }
	gGlobal.strDeviceMap.clear();
    gGlobal.numDeviceMap.clear();

    return LW_RETURN_OK;
}

LWReturnCode LWFindDevices(LWDeviceHandle *handleList, int32_t listCount, int32_t *filledCount)
{
	std::lock_guard<std::mutex> guard{ notReentryMutex };

	LOG_INFO_OUT("");

	if (!gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED; 
	if (listCount < 1)
	{
		gGlobal.errorInfo = "The number of parameters exceeds the range.";
		return LW_RETURN_CUSTOM_ERROR;
	}

	gGlobal.numDeviceMap.clear();
    *filledCount = 0;

    std::vector<sockaddr_in> sockaddr_list;
	if (!getLocalAddrInfo(sockaddr_list))
	{
		gGlobal.errorInfo = getNetworkLastError();
		LOG_ERROR_OUT("%s", gGlobal.errorInfo.c_str());
		return LW_RETURN_NETWORK_ERROR;
	}

    std::mutex _mutex;
    std::vector<std::thread> thread_list;
    for (auto& local_addr : sockaddr_list)
    {
		thread_list.emplace_back([&local_addr, &_mutex, &handleList, &listCount, &filledCount]()->void {
			//
			sockaddr_in to_addr{};
			to_addr.sin_family = AF_INET;
			to_addr.sin_port = htons(COMMAND_PORT);
			to_addr.sin_addr.s_addr = INADDR_BROADCAST;

			CommandFrame command{ C_Discovery };
			command.setVersion(0x03);
			SOCKET _socket = gSocketMap[local_addr.sin_addr.s_addr];
			if ((sendto(_socket, command.data(), command.size(), 0, (sockaddr*)&to_addr, sizeof(to_addr)) == SOCKET_ERROR)
				|| (!setNetworkTimeout(_socket, 3000))
				)
			{
				gGlobal.errorInfo = getNetworkLastError();
				LOG_ERROR_OUT("%s", gGlobal.errorInfo.c_str());
				return;
			}

			sockaddr_in from_addr{};
			socklen_t addr_len = sizeof(from_addr);
			while (true)
			{
				if (recvfrom(_socket, command.data(), command.maxBuffer(), 0, (sockaddr*)&from_addr, &addr_len) == SOCKET_ERROR) break;
				// 过滤掉IP为：66.66.66.66 的地址  
				if (from_addr.sin_addr.s_addr == 1111638594U) continue;

				// 创建设备句柄对象
				if (command.isCommand(C_Discovery))
				{
					if (command.getVersion() > 0x01)
					{
						auto reply = (uint8_t*)(command.getArgField());

						uint32_t devIP = from_addr.sin_addr.s_addr;
						uint32_t locIP = local_addr.sin_addr.s_addr;
						uint32_t devMrak = ntohl(*((uint32_t*)(reply + 13)));
						
						if(command.getArgFieldLength() < 31) continue;
						std::string sn(command.getArgField() + 17, 13);
						if (sn.find("D3") == std::string::npos) continue;

						LWDeviceHandle id = 0;
						auto idPtr = (char*)&id;
						memcpy(idPtr, &devIP, 4);
						memcpy(idPtr + 4, &locIP, 4);

						_mutex.lock();
						if (gGlobal.strDeviceMap.find(sn) == gGlobal.strDeviceMap.end())
						{
							gGlobal.strDeviceMap[sn] = new DeviceHandle(id);
						}
						gGlobal.numDeviceMap[id] = gGlobal.strDeviceMap[sn];
						if (*filledCount < listCount)
						{
							handleList[*filledCount] = id;
							++(*filledCount);
						}
						_mutex.unlock();

						auto handle = gGlobal.numDeviceMap[id];
						handle->sn = sn;
						handle->describe.clear();
						handle->localAddr = local_addr;
						handle->remoteAddr = from_addr;
						handle->recvAddr = from_addr;
						handle->recvAddr.sin_port = htons(DATA_PORT);

						if (reply[8] == 0x01)
						{
							if ((devIP & devMrak) != (locIP & devMrak))
							{
								char buffer[256];
								auto ipPtr1 = (unsigned char*)&locIP;
								auto ipPtr2 = (unsigned char*)&devIP;
								snprintf(buffer, 256, "The remote IP and the local IP are not in the same network segment and need to be adjusted to the same network segment. Local IP: %u.%u.%u.%u, Remote IP: %u.%u.%u.%u",
									ipPtr1[0], ipPtr1[1], ipPtr1[2], ipPtr1[3], ipPtr2[0], ipPtr2[1], ipPtr2[2], ipPtr2[3]);
								handle->describe = std::string(buffer);
							}
						}
						else
						{
							auto devip_c = (uint8_t*)(&devIP);
							if (((devip_c[0] == 0xA9) && (devip_c[1] == 0xFE)) && ((reply[3] != 0xA9) || (reply[2] != 0xFE)))
							{
								handle->describe = "The remote device has enabled DHCP protocol, but the computer has a static IP and needs to be adjusted to match it.";
							}
						}
                    }
					else
					{
						LOG_ERROR_OUT("The SDK does not match the device firmware version.");
					}
                }
            }
		});
    }

    for (auto& var : thread_list)
    {
        var.join();
    }

    return LW_RETURN_OK;
}

LWReturnCode LWOpenDevice(LWDeviceHandle handle)
{
	std::lock_guard<std::mutex> guard{ notReentryMutex };

	if (!gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
    if (gGlobal.numDeviceMap.find(handle) == gGlobal.numDeviceMap.end())
        return LW_RETURN_HANDLE_MISMATCH;

    return gGlobal.numDeviceMap[handle]->OpenDevice();
}

LWReturnCode LWCloseDevice(LWDeviceHandle handle)
{
	std::lock_guard<std::mutex> guard{ notReentryMutex };

	if (!gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (gGlobal.numDeviceMap.find(handle) == gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return gGlobal.numDeviceMap[handle]->CloseDevice();
}

LWReturnCode LWReconnectDevice(LWDeviceHandle handle, uint32_t t)
{
	std::lock_guard<std::mutex> guard{ notReentryMutex };

	if (!gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (gGlobal.numDeviceMap.find(handle) == gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return gGlobal.numDeviceMap[handle]->ReconnectDevice(t);
}

LWReturnCode LWRebootDevice(LWDeviceHandle handle)
{
	if (!gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (gGlobal.numDeviceMap.find(handle) == gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return gGlobal.numDeviceMap[handle]->RebootDevice();
}

LWReturnCode LWSaveConfigureInfo(LWDeviceHandle handle)
{
	if (!gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (gGlobal.numDeviceMap.find(handle) == gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return gGlobal.numDeviceMap[handle]->SaveConfigureInfo();
}

LWReturnCode LWRemoveConfigureInfo(LWDeviceHandle handle)
{
	if (!gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (gGlobal.numDeviceMap.find(handle) == gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return gGlobal.numDeviceMap[handle]->RemoveConfigureInfo();
}

LWReturnCode LWRestoreFactoryConfigureInfo(LWDeviceHandle handle)
{
	if (!gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (gGlobal.numDeviceMap.find(handle) == gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return gGlobal.numDeviceMap[handle]->RestoreFactoryConfigureInfo();
}

LWReturnCode LWStartStream(LWDeviceHandle handle)
{
	if (!gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (gGlobal.numDeviceMap.find(handle) == gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return gGlobal.numDeviceMap[handle]->StartStream();
}

LWReturnCode LWStopStream(LWDeviceHandle handle)
{
	if (!gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (gGlobal.numDeviceMap.find(handle) == gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return gGlobal.numDeviceMap[handle]->StopStream();
}

LWReturnCode LWHasRgbModule(LWDeviceHandle handle, bool* value)
{
	if (!gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (gGlobal.numDeviceMap.find(handle) == gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return gGlobal.numDeviceMap[handle]->HasRgbModule(*value);
}

LWReturnCode LWSoftTrigger(LWDeviceHandle handle)
{
	if (!gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (gGlobal.numDeviceMap.find(handle) == gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return gGlobal.numDeviceMap[handle]->SoftTrigger();
}

LWReturnCode LWSetTimeout(LWDeviceHandle handle, uint32_t t)
{
	if (!gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (gGlobal.numDeviceMap.find(handle) == gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	gGlobal.numDeviceMap[handle]->timeout = t;

	return LW_RETURN_OK;
}

LWReturnCode LWSetTriggerMode(LWDeviceHandle handle, LWTriggerMode mode)
{
	if (!gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (gGlobal.numDeviceMap.find(handle) == gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return gGlobal.numDeviceMap[handle]->SetTriggerMode(mode);
}

LWReturnCode LWGetTriggerMode(LWDeviceHandle handle, LWTriggerMode* mode)
{
	if (!gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (gGlobal.numDeviceMap.find(handle) == gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return gGlobal.numDeviceMap[handle]->GetTriggerMode(*mode);
}

LWReturnCode LWSetExposureMode(LWDeviceHandle handle, LWSensorType sensorType, LWExposureMode mode)
{
	if (!gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (gGlobal.numDeviceMap.find(handle) == gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return gGlobal.numDeviceMap[handle]->SetExposureMode(sensorType, mode);
}

LWReturnCode LWGetExposureMode(LWDeviceHandle handle, LWSensorType sensorType, LWExposureMode* mode)
{
	if (!gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (gGlobal.numDeviceMap.find(handle) == gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return gGlobal.numDeviceMap[handle]->GetExposureMode(sensorType, *mode);
}

LWReturnCode LWSetFrequencyMode(LWDeviceHandle handle, LWFrequencyMode mode)
{
	if (!gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (gGlobal.numDeviceMap.find(handle) == gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return gGlobal.numDeviceMap[handle]->SetFrequencyMode(mode);
}

LWReturnCode LWGetFrequencyMode(LWDeviceHandle handle, LWFrequencyMode* mode)
{
	if (!gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (gGlobal.numDeviceMap.find(handle) == gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return gGlobal.numDeviceMap[handle]->GetFrequencyMode(*mode);
}

LWReturnCode LWSetHDRModeEnable(LWDeviceHandle handle, bool enable)
{
	if (!gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (gGlobal.numDeviceMap.find(handle) == gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return gGlobal.numDeviceMap[handle]->SetHDRModeEnable(enable);
}

LWReturnCode LWGetHDRModeEnable(LWDeviceHandle handle, bool* enable)
{
	if (!gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (gGlobal.numDeviceMap.find(handle) == gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return gGlobal.numDeviceMap[handle]->GetHDRModeEnable(*enable);
}

LWReturnCode LWSetTransformDepthToRgbEnable(LWDeviceHandle handle, bool enable)
{
	if (!gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (gGlobal.numDeviceMap.find(handle) == gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

    return gGlobal.numDeviceMap[handle]->SetTransformDepthToRgbEnable(enable);
}

LWReturnCode LWSetTransformRgbToDepthEnable(LWDeviceHandle handle, bool enable)
{
	if (!gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (gGlobal.numDeviceMap.find(handle) == gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

    return gGlobal.numDeviceMap[handle]->SetTransformRgbToDepthEnable(enable);
}

LWReturnCode LWSetFrameRate(LWDeviceHandle handle, int32_t value)
{
	if (!gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (gGlobal.numDeviceMap.find(handle) == gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return gGlobal.numDeviceMap[handle]->SetFrameRate(value);
}

LWReturnCode LWGetFrameRate(LWDeviceHandle handle, int32_t* value)
{
	if (!gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (gGlobal.numDeviceMap.find(handle) == gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return gGlobal.numDeviceMap[handle]->GetFrameRate(*value);
}

LWReturnCode LWSetExposureTime(LWDeviceHandle handle, LWSensorType sensorType, const int32_t* etArray, int32_t arraySize)
{
	if (!gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (gGlobal.numDeviceMap.find(handle) == gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return gGlobal.numDeviceMap[handle]->SetExposureTime(sensorType, etArray, arraySize);
}

LWReturnCode LWGetExposureTime(LWDeviceHandle handle, LWSensorType sensorType, int32_t* etArray, int32_t arraySize, int32_t* filledCount)
{
	if (!gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (gGlobal.numDeviceMap.find(handle) == gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return gGlobal.numDeviceMap[handle]->GetExposureTime(sensorType, etArray, arraySize, *filledCount);
}

LWReturnCode LWSetTimeFilterParams(LWDeviceHandle handle, LWFilterParam param)
{
	if (!gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (gGlobal.numDeviceMap.find(handle) == gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return gGlobal.numDeviceMap[handle]->SetTimeFilterParams(param);
}

LWReturnCode LWGetTimeFilterParams(LWDeviceHandle handle, LWFilterParam* param)
{
	if (!gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (gGlobal.numDeviceMap.find(handle) == gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return gGlobal.numDeviceMap[handle]->GetTimeFilterParams(*param);
}

LWReturnCode LWSetTimeMedianFilterParams(LWDeviceHandle handle, LWFilterParam param)
{
	if (!gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (gGlobal.numDeviceMap.find(handle) == gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return gGlobal.numDeviceMap[handle]->SetTimeMedianFilterParams(param);
}

LWReturnCode LWGetTimeMedianFilterParams(LWDeviceHandle handle, LWFilterParam* param)
{
	if (!gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (gGlobal.numDeviceMap.find(handle) == gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return gGlobal.numDeviceMap[handle]->GetTimeMedianFilterParams(*param);
}

LWReturnCode LWSetSpatialFilterParams(LWDeviceHandle handle, LWFilterParam param)
{
	if (!gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (gGlobal.numDeviceMap.find(handle) == gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return gGlobal.numDeviceMap[handle]->SetSpatialFilterParams(param);
}

LWReturnCode LWGetSpatialFilterParams(LWDeviceHandle handle, LWFilterParam* param)
{
	if (!gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (gGlobal.numDeviceMap.find(handle) == gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return gGlobal.numDeviceMap[handle]->GetSpatialFilterParams(*param);
}

LWReturnCode LWSetFlyingPixelsFilterParams(LWDeviceHandle handle, LWFilterParam param)
{
	if (!gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (gGlobal.numDeviceMap.find(handle) == gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return gGlobal.numDeviceMap[handle]->SetFlyingPixelsFilterParams(param);
}

LWReturnCode LWGetFlyingPixelsFilterParams(LWDeviceHandle handle, LWFilterParam* param)
{
	if (!gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (gGlobal.numDeviceMap.find(handle) == gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return gGlobal.numDeviceMap[handle]->GetFlyingPixelsFilterParams(*param);
}

LWReturnCode LWSetConfidenceFilterParams(LWDeviceHandle handle, LWFilterParam param)
{
	if (!gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (gGlobal.numDeviceMap.find(handle) == gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return gGlobal.numDeviceMap[handle]->SetConfidenceFilterParams(param);
}

LWReturnCode LWGetConfidenceFilterParams(LWDeviceHandle handle, LWFilterParam* param)
{
	if (!gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (gGlobal.numDeviceMap.find(handle) == gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return gGlobal.numDeviceMap[handle]->GetConfidenceFilterParams(*param);
}

LWReturnCode LWSetIRGMMGain(LWDeviceHandle handle, int32_t value)
{
	if (!gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (gGlobal.numDeviceMap.find(handle) == gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return gGlobal.numDeviceMap[handle]->SetIRGMMGain(value);
}

LWReturnCode LWGetIRGMMGain(LWDeviceHandle handle, int32_t* value)
{
	if (!gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (gGlobal.numDeviceMap.find(handle) == gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return gGlobal.numDeviceMap[handle]->GetIRGMMGain(*value);
}

LWReturnCode LWSetRgbSensorGain(LWDeviceHandle handle, int32_t value)
{
	if (!gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (gGlobal.numDeviceMap.find(handle) == gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return gGlobal.numDeviceMap[handle]->SetRgbSensorGain(value);
}

LWReturnCode LWGetRgbSensorGain(LWDeviceHandle handle, int32_t* value)
{
	if (!gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (gGlobal.numDeviceMap.find(handle) == gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return gGlobal.numDeviceMap[handle]->GetRgbSensorGain(*value);
}

LWReturnCode LWSetRgbSensorGamma(LWDeviceHandle handle, int32_t value)
{
	if (!gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (gGlobal.numDeviceMap.find(handle) == gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return gGlobal.numDeviceMap[handle]->SetRgbSensorGamma(value);
}

LWReturnCode LWGetRgbSensorGamma(LWDeviceHandle handle, int32_t* value)
{
	if (!gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (gGlobal.numDeviceMap.find(handle) == gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return gGlobal.numDeviceMap[handle]->GetRgbSensorGamma(*value);
}

LWReturnCode LWSetRgbSensorBrightness(LWDeviceHandle handle, int32_t value)
{
	if (!gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (gGlobal.numDeviceMap.find(handle) == gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return gGlobal.numDeviceMap[handle]->SetRgbSensorBrightness(value);
}

LWReturnCode LWGetRgbSensorBrightness(LWDeviceHandle handle, int32_t* value)
{
	if (!gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (gGlobal.numDeviceMap.find(handle) == gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return gGlobal.numDeviceMap[handle]->GetRgbSensorBrightness(*value);
}

LWReturnCode LWSetRgbSensorContrastRatio(LWDeviceHandle handle, int32_t value)
{
	if (!gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (gGlobal.numDeviceMap.find(handle) == gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return gGlobal.numDeviceMap[handle]->SetRgbSensorContrastRatio(value);
}

LWReturnCode LWGetRgbSensorContrastRatio(LWDeviceHandle handle, int32_t* value)
{
	if (!gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (gGlobal.numDeviceMap.find(handle) == gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return gGlobal.numDeviceMap[handle]->GetRgbSensorContrastRatio(*value);
}

LWReturnCode LWSetNetworkInfo(LWDeviceHandle handle, LWNetworkInfo info)
{
	if (!gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (gGlobal.numDeviceMap.find(handle) == gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return gGlobal.numDeviceMap[handle]->SetNetworkInfo(info);
}

LWReturnCode LWGetNetworkInfo(LWDeviceHandle handle, LWNetworkInfo* info)
{
	if (!gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (gGlobal.numDeviceMap.find(handle) == gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return gGlobal.numDeviceMap[handle]->GetNetworkInfo(*info);
}

LWReturnCode LWSetRgbDataTransportFormat(LWDeviceHandle handle, LWRgbTransferFormat format)
{
	if (!gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (gGlobal.numDeviceMap.find(handle) == gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return gGlobal.numDeviceMap[handle]->SetRgbDataTransportFormat(format);
}

LWReturnCode LWGetRgbDataTransportFormat(LWDeviceHandle handle, LWRgbTransferFormat* format)
{
	if (!gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (gGlobal.numDeviceMap.find(handle) == gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return gGlobal.numDeviceMap[handle]->GetRgbDataTransportFormat(*format);
}

LWReturnCode LWSetDeviceNumber(LWDeviceHandle handle, int32_t value)
{
	if (!gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (gGlobal.numDeviceMap.find(handle) == gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return gGlobal.numDeviceMap[handle]->SetDeviceNumber(value);
}

LWReturnCode LWGetDeviceNumber(LWDeviceHandle handle, int32_t* value)
{
	if (!gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (gGlobal.numDeviceMap.find(handle) == gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return gGlobal.numDeviceMap[handle]->GetDeviceNumber(*value);
}

LWReturnCode LWSetHardTriggerFilterParams(LWDeviceHandle handle, int32_t t1, int32_t t2)
{
	if (!gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (gGlobal.numDeviceMap.find(handle) == gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return gGlobal.numDeviceMap[handle]->SetHardTriggerFilterParams(t1, t2);
}

LWReturnCode LWGetHardTriggerFilterParams(LWDeviceHandle handle, int32_t* t1, int32_t* t2)
{
	if (!gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (gGlobal.numDeviceMap.find(handle) == gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return gGlobal.numDeviceMap[handle]->GetHardTriggerFilterParams(*t1, *t2);
}

LWReturnCode LWSetResolution(LWDeviceHandle handle, LWSensorType sensorType, int32_t width, int32_t height)
{
	if (!gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (gGlobal.numDeviceMap.find(handle) == gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return gGlobal.numDeviceMap[handle]->SetResolution(sensorType, width, height);
}

LWReturnCode LWGetResolution(LWDeviceHandle handle, LWSensorType sensorType, int32_t* width, int32_t* height)
{
	if (!gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (gGlobal.numDeviceMap.find(handle) == gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return gGlobal.numDeviceMap[handle]->GetResolution(sensorType, *width, *height);
}

LWReturnCode LWGetIntrinsicParam(LWDeviceHandle handle, LWSensorType sensorType, LWSensorIntrinsicParam* intrinsicParam)
{
	if (!gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (gGlobal.numDeviceMap.find(handle) == gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return gGlobal.numDeviceMap[handle]->GetIntrinsicParam(sensorType, *intrinsicParam);
}

LWReturnCode LWGetExtrinsicParam(LWDeviceHandle handle, LWSensorType sensorType, LWSensorExtrinsicParam* extrinsicParam)
{
	if (!gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (gGlobal.numDeviceMap.find(handle) == gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return gGlobal.numDeviceMap[handle]->GetExtrinsicParam(sensorType, *extrinsicParam);
}

LWReturnCode LWGetDeviceSN(LWDeviceHandle handle, char* sn, int32_t bufferLen)
{
	if (!gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (gGlobal.numDeviceMap.find(handle) == gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return gGlobal.numDeviceMap[handle]->GetDeviceSN(sn, bufferLen);
}

LWReturnCode LWGetDeviceType(LWDeviceHandle handle, char* type, int32_t bufferLen)
{
	if (!gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (gGlobal.numDeviceMap.find(handle) == gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return gGlobal.numDeviceMap[handle]->GetDeviceType(type, bufferLen);
}

LWReturnCode LWGetTimeStamp(LWDeviceHandle handle, LWTimeStamp* t)
{
	if (!gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (gGlobal.numDeviceMap.find(handle) == gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return gGlobal.numDeviceMap[handle]->GetTimeStamp(*t);
}

LWReturnCode LWGetLibVersion(LWVersionInfo *version)
{
	LOG_INFO_OUT("");

	if (!gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;

	version->major = MAJOR_VERSION;
	version->minor = MINOR_VERSION;
	version->patch = PATCH_VERSION;
	version->reserved = REVISION_VERSION;

	return LW_RETURN_OK;
}

LWReturnCode LWGetDeviceVersion(LWDeviceHandle handle, LWVersionInfo* fv, LWVersionInfo* dv)
{
	if (!gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (gGlobal.numDeviceMap.find(handle) == gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return gGlobal.numDeviceMap[handle]->GetDeviceVersion(*fv, *dv);
}

LWReturnCode LWRegisterNetworkMonitoringCallback(void(*pCallback)(LWDeviceHandle handle, const char* error, void* pUserData), void* pUserData)
{
	std::lock_guard<std::mutex> guard{ notReentryMutex };

	LOG_INFO_OUT("");

	if (!gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;

	pUserData1 = pUserData;
	networkAbnormalCallback = pCallback;

	return LW_RETURN_OK;
}

LWReturnCode LWRegisterFrameReadyCallback(void(*pCallback)(LWDeviceHandle handle, void* pUserData), void* pUserData)
{
	std::lock_guard<std::mutex> guard{ notReentryMutex };

	LOG_INFO_OUT("");

	if (!gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;

	pUserData2 = pUserData;
	frameReadyCallback = pCallback;

	return LW_RETURN_OK;
}

const char *LWGetReturnCodeDescriptor(LWReturnCode code)
{
	switch (code)
	{
	case LW_RETURN_OK:
		gGlobal.errorInfo = "Execution successful.";
		break;

	case LW_RETURN_TIMEOUT:
		gGlobal.errorInfo = "Execution timeout.";
		break;

	case LW_RETURN_NETWORK_ERROR:
		break;

	case LW_RETURN_DATA_NOT_UPDATED:
		gGlobal.errorInfo = "The data receiving cache has not been updated. Please successfully call the 'LWGetFrameReady' function before retrieving the data.";
		break;

	case LW_RETURN_ARG_OUT_OF_RANGE:
		gGlobal.errorInfo = "Parameter settings are out of range.";
		break;

	case LW_RETURN_UNINITIALIZED:
		gGlobal.errorInfo = "The resource has not been initialized.";
		break;

	case LW_RETURN_UNOPENED:
		gGlobal.errorInfo = "The device is not turned on.";
		break;

	case LW_RETURN_COMMAND_UNDEFINED:
		gGlobal.errorInfo = "The command is undefined.The device does not yet have the ability to process this command.";
		break;

	case LW_RETURN_OUT_OF_MEMORY:
		gGlobal.errorInfo = "The incoming data buffer is out of memory.";
		break;

	case LW_RETURN_COMMAND_ERROR:
		gGlobal.errorInfo = "The command structure is incorrect.";
		break;

	case LW_RETURN_TYPE_NOT_EXIST:
		gGlobal.errorInfo = "Type error means that the type does not exist.";
		break;

	case LW_RETURN_TYPE_INPUT_ERROR:
		gGlobal.errorInfo = "Type input error.";
		break;

	case LW_RETURN_FIRMWARE_UPDATE_FAIL:
		break;

	case LW_RETURN_THREAD_QUIT_TIMEOUT:
		gGlobal.errorInfo = "Thread exit timeout.";
		break;

	case LW_RETURN_DATA_TYPE_MISMATCH:
		gGlobal.errorInfo = "Unable to retrieve data of this type, please set the correct data acceptance type.";
		break;

	case LW_RETURN_FILE_NOT_EXIST:
		gGlobal.errorInfo = "The file does not exist.";
		break;

	case LW_RETURN_FILE_LENGTH_ERROR:
		gGlobal.errorInfo = "The file length is incorrect.";
		break;

	case LW_RETURN_FILE_MD5_ERROR:
		gGlobal.errorInfo = "File MD5 verification failed.";
		break;

	case LW_RETURN_HANDLE_MISMATCH:
		gGlobal.errorInfo = "The device handle is not in the search list.";
		break;

	case LW_RETURN_FILE_OPEN_ERROR:
		gGlobal.errorInfo = "File opening failed.";
		break;

	case LW_RETURN_NOT_SUPPORTED:
		gGlobal.errorInfo = "The current device is not supported.";
		break;

	case LW_RETURN_VERSION_ERROR:
		gGlobal.errorInfo = "Protocol version mismatch.";
		break;

	case LW_RETURN_CUSTOM_ERROR:
		break;

	case LW_RETURN_UNDEFINED_ERROR:
		gGlobal.errorInfo = " Undefined error.";
		break;

	default:
		gGlobal.errorInfo = "The return code exceeds the predetermined range.";
		break;
	}

	return gGlobal.errorInfo.c_str();
}

LWReturnCode LWGetFrameReady(LWDeviceHandle handle)
{
	if (!gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (gGlobal.numDeviceMap.find(handle) == gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return gGlobal.numDeviceMap[handle]->GetFrameReady();
}

LWReturnCode LWGetFrame(LWDeviceHandle handle, LWFrameData *frame, LWFrameType type)
{
	if (!gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (gGlobal.numDeviceMap.find(handle) == gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	if (!gGlobal.numDeviceMap[handle]->isReady) return LW_RETURN_DATA_NOT_UPDATED;

	switch (type)
	{
	case LW_DEPTH_FRAME:
	{
		memcpy(gGlobal.numDeviceMap[handle]->pDepth.data(), gGlobal.numDeviceMap[handle]->tofCutNode->data(), gGlobal.numDeviceMap[handle]->tofPixels * 2);

		frame->width		= gGlobal.numDeviceMap[handle]->tofWidth;
		frame->height		= gGlobal.numDeviceMap[handle]->tofHeight;
		frame->frameType	= type;
		frame->elemSize		= 2;
		frame->total		= gGlobal.numDeviceMap[handle]->tofPixels;
		frame->pixelFormat	= LW_PIXEL_FORMAT_USHORT;
		frame->frameIndex	= gGlobal.numDeviceMap[handle]->tofCutNode->serial;
		frame->bufferSize	= gGlobal.numDeviceMap[handle]->tofPixels * 2;
		frame->pFrameData	= gGlobal.numDeviceMap[handle]->pDepth.data();
		frame->timestamp	= gGlobal.numDeviceMap[handle]->tofCutNode->time;
		frame->temperature	= gGlobal.numDeviceMap[handle]->tofCutNode->temperature;
		frame->pVariant		= nullptr;

		return LW_RETURN_OK;
	}

	case LW_AMPLITUDE_FRAME:
	{
		memcpy(gGlobal.numDeviceMap[handle]->pAmp.data(), gGlobal.numDeviceMap[handle]->tofCutNode->data() + gGlobal.numDeviceMap[handle]->tofPixels * 2, gGlobal.numDeviceMap[handle]->tofPixels * 2);

		frame->width		= gGlobal.numDeviceMap[handle]->tofWidth;
		frame->height		= gGlobal.numDeviceMap[handle]->tofHeight;
		frame->frameType	= type;
		frame->elemSize		= 2;
		frame->total		= gGlobal.numDeviceMap[handle]->tofPixels;
		frame->pixelFormat	= LW_PIXEL_FORMAT_USHORT;
		frame->frameIndex	= gGlobal.numDeviceMap[handle]->tofCutNode->serial;
		frame->bufferSize	= gGlobal.numDeviceMap[handle]->tofPixels * 2;
		frame->pFrameData	= gGlobal.numDeviceMap[handle]->pAmp.data();
		frame->timestamp	= gGlobal.numDeviceMap[handle]->tofCutNode->time;
		frame->temperature	= gGlobal.numDeviceMap[handle]->tofCutNode->temperature;
		frame->pVariant		= nullptr;

		return LW_RETURN_OK;
	}

	case LW_IR_FRAME:
	{
		auto amp_ptr	= (uint16_t*)(gGlobal.numDeviceMap[handle]->tofCutNode->data() + gGlobal.numDeviceMap[handle]->tofPixels * 2);
		auto gra_ptr	= (uint8_t*)gGlobal.numDeviceMap[handle]->pGra.data();
		auto threshold	= gGlobal.numDeviceMap[handle]->irGMMGain;
		for (uint32_t i = 0, val; i < gGlobal.numDeviceMap[handle]->tofPixels; ++i, ++amp_ptr, ++gra_ptr)
		{
			val = (int)(*amp_ptr / 128.0 * threshold);
			*gra_ptr = (val < 255) ? val : 255;
		}

		frame->width		= gGlobal.numDeviceMap[handle]->tofWidth;
		frame->height		= gGlobal.numDeviceMap[handle]->tofHeight;
		frame->frameType	= type;
		frame->elemSize		= 1;
		frame->total		= gGlobal.numDeviceMap[handle]->tofPixels;
		frame->pixelFormat	= LW_PIXEL_FORMAT_UCHAR;
		frame->frameIndex	= gGlobal.numDeviceMap[handle]->tofCutNode->serial;
		frame->bufferSize	= gGlobal.numDeviceMap[handle]->tofPixels;
		frame->pFrameData	= gGlobal.numDeviceMap[handle]->pGra.data();
		frame->timestamp	= gGlobal.numDeviceMap[handle]->tofCutNode->time;
		frame->temperature	= gGlobal.numDeviceMap[handle]->tofCutNode->temperature;
		frame->pVariant		= nullptr;

		return LW_RETURN_OK;
	}

	case LW_POINTCLOUD_FRAME:
	{
		auto dis_ptr	= (uint16_t*)(gGlobal.numDeviceMap[handle]->tofCutNode->data());
		auto pot_ptr	= (LWVector3f*)(gGlobal.numDeviceMap[handle]->pPot.data());
		auto col		= -gGlobal.numDeviceMap[handle]->tofInArg.cx;
		auto row		= -gGlobal.numDeviceMap[handle]->tofInArg.cy;
		for (uint32_t i = 0; i < gGlobal.numDeviceMap[handle]->tofHeight; ++i)
		{
			col = -gGlobal.numDeviceMap[handle]->tofInArg.cx;
			for (uint32_t j = 0; j < gGlobal.numDeviceMap[handle]->tofWidth; ++j, ++pot_ptr, ++dis_ptr)
			{
				if (*dis_ptr < PCD_MAX_VALUE)
				{
					pot_ptr->x = col / gGlobal.numDeviceMap[handle]->tofInArg.fx * float(*dis_ptr);
					pot_ptr->y = row / gGlobal.numDeviceMap[handle]->tofInArg.fy * float(*dis_ptr);
					pot_ptr->z = *dis_ptr;
				}
				else
				{
					pot_ptr->x = 0;
					pot_ptr->y = 0;
					pot_ptr->z = 0;
				}
                col += 1.0f;
			}
            row += 1.0f;
		}

		frame->width		= gGlobal.numDeviceMap[handle]->tofWidth;
		frame->height		= gGlobal.numDeviceMap[handle]->tofHeight;
		frame->frameType	= type;
		frame->elemSize		= 12;
		frame->total		= gGlobal.numDeviceMap[handle]->tofPixels;
		frame->pixelFormat	= LW_PIXEL_FORMAT_VECTOR3F;
		frame->frameIndex	= gGlobal.numDeviceMap[handle]->tofCutNode->serial;
		frame->bufferSize	= gGlobal.numDeviceMap[handle]->tofPixels * 12;
		frame->pFrameData	= gGlobal.numDeviceMap[handle]->pPot.data();
		frame->timestamp	= gGlobal.numDeviceMap[handle]->tofCutNode->time;
		frame->temperature	= gGlobal.numDeviceMap[handle]->tofCutNode->temperature;
		frame->pVariant		= nullptr;

		return LW_RETURN_OK;
	}

	case LW_RGB_FRAME:
	{
		if (!gGlobal.numDeviceMap[handle]->hasRgbModule.load())
		{
			gGlobal.errorInfo = "This device does not have an RGB module and cannot obtain RGB image data.";
			return LW_RETURN_CUSTOM_ERROR;
		}

		memcpy(gGlobal.numDeviceMap[handle]->pRgb.data(), gGlobal.numDeviceMap[handle]->rgbCutNode->data(), gGlobal.numDeviceMap[handle]->rgbCutNode->size);

		frame->width		= gGlobal.numDeviceMap[handle]->rgbWidth;
		frame->height		= gGlobal.numDeviceMap[handle]->rgbHeight;
		frame->frameType	= type;
		frame->elemSize		= 3;
		frame->total		= gGlobal.numDeviceMap[handle]->rgbPixels;
		frame->pixelFormat	= LW_PIXEL_FORMAT_RGB888;
		frame->frameIndex	= gGlobal.numDeviceMap[handle]->rgbCutNode->serial;
		frame->bufferSize	= gGlobal.numDeviceMap[handle]->rgbCutNode->size;
		frame->pFrameData	= gGlobal.numDeviceMap[handle]->pRgb.data();
		frame->timestamp	= gGlobal.numDeviceMap[handle]->rgbCutNode->time;
		frame->temperature	= {};
		frame->pVariant		= nullptr;

		return LW_RETURN_OK;
	}

	case LW_RGB_TO_DEPTH_FRAME:
	{
		if (!gGlobal.numDeviceMap[handle]->hasRgbModule.load())
		{
			gGlobal.errorInfo = "This device does not have an RGB module and cannot obtain data of this type.";
			return LW_RETURN_CUSTOM_ERROR;
		}

		if (!gGlobal.numDeviceMap[handle]->isR2DEnable.load())
		{
			gGlobal.errorInfo = "The transform function from rgb to depth data must be enabled. Namely: LWSetTransformRgbToDepthEnable(true).";
			return LW_RETURN_CUSTOM_ERROR;
		}

		std::unique_lock<std::mutex> lock{ gGlobal.numDeviceMap[handle]->r2dMutex };
		memset(gGlobal.numDeviceMap[handle]->pR2D.data(), 0, gGlobal.numDeviceMap[handle]->pR2D.size());
		gGlobal.numDeviceMap[handle]->r2dRunCount	= THREAD_POOL_SIZE;
		gGlobal.numDeviceMap[handle]->r2dFlag		= 0xFFFFFFFF;
		gGlobal.numDeviceMap[handle]->r2dNotify.notify_all();
		if (gGlobal.numDeviceMap[handle]->r2dNotify.wait_for(lock, std::chrono::milliseconds{ gGlobal.numDeviceMap[handle]->timeout }, [handle] { return gGlobal.numDeviceMap[handle]->r2dRunCount == 0 || !gGlobal.initEnable.load(); }))
		{
			if (!gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;

			frame->width		= gGlobal.numDeviceMap[handle]->tofWidth;
			frame->height		= gGlobal.numDeviceMap[handle]->tofHeight;
			frame->frameType	= type;
			frame->elemSize		= 3;
			frame->total		= gGlobal.numDeviceMap[handle]->tofPixels;
			frame->pixelFormat	= LW_PIXEL_FORMAT_RGB888;
			frame->frameIndex	= gGlobal.numDeviceMap[handle]->rgbCutNode->serial;
			frame->bufferSize	= gGlobal.numDeviceMap[handle]->tofPixels * 3;
			frame->pFrameData	= gGlobal.numDeviceMap[handle]->pR2D.data();
			frame->timestamp	= gGlobal.numDeviceMap[handle]->rgbCutNode->time;
			frame->temperature	= {};
			frame->pVariant		= nullptr;

			return LW_RETURN_OK;
		}

		return LW_RETURN_TIMEOUT;
	}

	case LW_DEPTH_TO_RGB_FRAME:
	{
		if (!gGlobal.numDeviceMap[handle]->hasRgbModule.load())
		{
			gGlobal.errorInfo = "This device does not have an RGB module and cannot obtain data of this type.";
			return LW_RETURN_CUSTOM_ERROR;
		}

		if (!gGlobal.numDeviceMap[handle]->isD2REnable.load())
		{
			gGlobal.errorInfo = "The transform function from depth to rgb data must be enabled. Namely: LWSetTransformDepthToRgbEnable(true).";
			return LW_RETURN_CUSTOM_ERROR;
		}

		std::unique_lock<std::mutex> lock{ gGlobal.numDeviceMap[handle]->d2rMutex };
		gGlobal.numDeviceMap[handle]->d2rInputImg.setTo(0);
		gGlobal.numDeviceMap[handle]->d2rRunCount = THREAD_POOL_SIZE;
		gGlobal.numDeviceMap[handle]->d2rFlag = 0xFFFFFFFF;
		gGlobal.numDeviceMap[handle]->d2rNotify.notify_all();
		if (gGlobal.numDeviceMap[handle]->d2rNotify.wait_for(lock, std::chrono::milliseconds{ gGlobal.numDeviceMap[handle]->timeout }, [handle] { return gGlobal.numDeviceMap[handle]->d2rRunCount == 0 || !gGlobal.initEnable.load(); }))
		{
			if (!gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;

			frame->width		= gGlobal.numDeviceMap[handle]->rgbWidth;
			frame->height		= gGlobal.numDeviceMap[handle]->rgbHeight;
			frame->frameType	= type;
			frame->elemSize		= 2;
			frame->total		= gGlobal.numDeviceMap[handle]->rgbPixels;
			frame->pixelFormat	= LW_PIXEL_FORMAT_USHORT;
			frame->frameIndex	= gGlobal.numDeviceMap[handle]->tofCutNode->serial;
			frame->bufferSize	= gGlobal.numDeviceMap[handle]->rgbPixels * 2;
			frame->pFrameData	= (char*)gGlobal.numDeviceMap[handle]->d2rOutputImg.data;
			frame->timestamp	= gGlobal.numDeviceMap[handle]->tofCutNode->time;
			frame->temperature	= gGlobal.numDeviceMap[handle]->tofCutNode->temperature;
			frame->pVariant		= nullptr;

			return LW_RETURN_OK;
		}

		return LW_RETURN_TIMEOUT;
	}

	case LW_D2R_POINTCLOUD_FRAME:
	{
		if (!gGlobal.numDeviceMap[handle]->hasRgbModule.load())
		{
			gGlobal.errorInfo = "This device does not have an RGB module and cannot obtain data of this type.";
			return LW_RETURN_CUSTOM_ERROR;
		}

		if (!gGlobal.numDeviceMap[handle]->isD2REnable.load())
		{
			gGlobal.errorInfo = "The transform function from depth to rgb data must be enabled. Namely: LWSetTransformDepthToRgbEnable(true).";
			return LW_RETURN_CUSTOM_ERROR;
		}

		std::unique_lock<std::mutex> lock{ gGlobal.numDeviceMap[handle]->d2rMutex };
		gGlobal.numDeviceMap[handle]->d2rInputImg.setTo(0);
		gGlobal.numDeviceMap[handle]->d2rRunCount = THREAD_POOL_SIZE;
		gGlobal.numDeviceMap[handle]->d2rFlag = 0xFFFFFFFF;
		gGlobal.numDeviceMap[handle]->d2rNotify.notify_all();
		if (!gGlobal.numDeviceMap[handle]->d2rNotify.wait_for(lock, std::chrono::milliseconds{ gGlobal.numDeviceMap[handle]->timeout }, [handle] { return gGlobal.numDeviceMap[handle]->d2rRunCount == 0; }))
		{
			return LW_RETURN_TIMEOUT;
		}

		auto dis_ptr = (uint16_t*)(gGlobal.numDeviceMap[handle]->d2rOutputImg.data);
		auto pot_ptr = (LWVector3f*)(gGlobal.numDeviceMap[handle]->pTPot.data());
		auto col = -gGlobal.numDeviceMap[handle]->rgbInArg.cx;
		auto row = -gGlobal.numDeviceMap[handle]->rgbInArg.cy;
		for (uint32_t i = 0; i < gGlobal.numDeviceMap[handle]->rgbHeight; ++i)
		{
			col = -gGlobal.numDeviceMap[handle]->rgbInArg.cx;
			for (uint32_t j = 0; j < gGlobal.numDeviceMap[handle]->rgbWidth; ++j, ++pot_ptr, ++dis_ptr)
			{
				if (*dis_ptr < PCD_MAX_VALUE)
				{
					pot_ptr->x = col / gGlobal.numDeviceMap[handle]->rgbInArg.fx * float(*dis_ptr);
					pot_ptr->y = row / gGlobal.numDeviceMap[handle]->rgbInArg.fy * float(*dis_ptr);
					pot_ptr->z = *dis_ptr;
				}
				else
				{
					pot_ptr->x = 0;
					pot_ptr->y = 0;
					pot_ptr->z = 0;
				}
				col += 1.0f;
			}
			row += 1.0f;
		}

		frame->width		= gGlobal.numDeviceMap[handle]->rgbWidth;
		frame->height		= gGlobal.numDeviceMap[handle]->rgbHeight;
		frame->frameType	= type;
		frame->elemSize		= 12;
		frame->total		= gGlobal.numDeviceMap[handle]->rgbPixels;
		frame->pixelFormat	= LW_PIXEL_FORMAT_VECTOR3F;
		frame->frameIndex	= gGlobal.numDeviceMap[handle]->tofCutNode->serial;
		frame->bufferSize	= gGlobal.numDeviceMap[handle]->rgbPixels * 12;
		frame->pFrameData	= gGlobal.numDeviceMap[handle]->pTPot.data();
		frame->timestamp	= gGlobal.numDeviceMap[handle]->tofCutNode->time;
		frame->temperature	= gGlobal.numDeviceMap[handle]->tofCutNode->temperature;
		frame->pVariant		= nullptr;

		return LW_RETURN_OK;
	}

	default:
		break;
	}

	return LW_RETURN_TYPE_NOT_EXIST;
}

LWReturnCode LWSavePointCloudAsPCDFile(const char* filename, const LWFrameData* frame, bool binary_mode)
{
	if ((frame->frameType != LW_POINTCLOUD_FRAME) 
		&& (frame->frameType != LW_D2R_POINTCLOUD_FRAME)) 
		return LW_RETURN_TYPE_INPUT_ERROR;

	std::ofstream file;

	binary_mode ? file.open(filename, std::ios::binary) : file.open(filename);
	if (!file.is_open()) return LW_RETURN_FILE_OPEN_ERROR;

	auto N = frame->total;
	file << "# .PCD v0.7 - Point Cloud Data file format\n";
	file << "VERSION 0.7\n";
	file << "FIELDS x y z\n";
	file << "SIZE 4 4 4\n";
	file << "TYPE F F F\n";
	file << "COUNT 1 1 1\n";
	file << "WIDTH " << frame->width << "\n";
	file << "HEIGHT " << frame->height << "\n";
	file << "VIEWPOINT 0 0 0 1 0 0 0\n";
	file << "POINTS " << N << "\n";

	if (binary_mode)
	{
		file << "DATA binary\n";
		file.write(frame->pFrameData, frame->bufferSize);
	}
	else
	{
		file << "DATA ascii\n";
		auto ptr_data = (LWVector3f*)frame->pFrameData;

		for (uint32_t i = 0; i < N; ++i, ++ptr_data)
		{
			file << ptr_data->x << " ";
			file << ptr_data->y << " ";
			file << ptr_data->z << "\n";
		}
	}

	file.close();

	return LW_RETURN_OK;
}

LWReturnCode LWSavePointCloudAsPLYFile(const char* filename, const LWFrameData* frame, bool binary_mode)
{
	if ((frame->frameType != LW_POINTCLOUD_FRAME)
		&& (frame->frameType != LW_D2R_POINTCLOUD_FRAME))
		return LW_RETURN_TYPE_INPUT_ERROR;

	std::ofstream file;

	binary_mode ? file.open(filename, std::ios::binary) : file.open(filename);
	if (!file.is_open()) return LW_RETURN_FILE_OPEN_ERROR;

	uint32_t _size_ = frame->total;

	file << "ply\n";
	binary_mode ? (file << "format binary_little_endian 1.0\n") : (file << "format ascii 1.0\n");
	file << "comment This is a point cloud file.\n";
	file << "element vertex " << _size_ << "\n";
	file << "property float x\n";
	file << "property float y\n";
	file << "property float z\n";
	file << "end_header\n";

	if (binary_mode)
	{
		file.write(frame->pFrameData, frame->bufferSize);
	}
	else
	{
		auto ptr_data = (float*)frame->pFrameData;
		for (uint32_t i = 0; i < _size_; ++i)
		{
			file << *ptr_data++ << " ";
			file << *ptr_data++ << " ";
			file << *ptr_data++ << "\n";
		}
	}

	file.close();

	return LW_RETURN_OK;
}

LWReturnCode LWSaveDataAsCSVFile(const char* filename, const LWFrameData* frame)
{
	std::ofstream file;
	file.open(filename);
	if (!file.fail())
	{
		switch (frame->frameType)
		{
		case LW_DEPTH_FRAME:
		case LW_AMPLITUDE_FRAME:
		case LW_DEPTH_TO_RGB_FRAME:
		{
			auto* ptr = (uint16_t*)frame->pFrameData;
			for (int32_t row = 0; row < frame->height; ++row)
			{
				for (int32_t col = 0; col < frame->width; ++col, ++ptr)
				{
					if (col > 0) file << ",";
					file << *ptr;
				}
				file << "\n";
			}

			file.close();
			return LW_RETURN_OK;
		}

		case LW_POINTCLOUD_FRAME:
		case LW_D2R_POINTCLOUD_FRAME:
		{
			auto* ptr = (LWVector3f*)frame->pFrameData;
			for (int32_t i = 0, n = frame->total; i < n; ++i, ++ptr)
			{
				file << ptr->x << ",";
				file << ptr->y << ",";
				file << ptr->z << "\n";
			}

			file.close();
			return LW_RETURN_OK;
		}

		case LW_RGB_FRAME:
		case LW_RGB_TO_DEPTH_FRAME:
		{
			auto* ptr = (LWRGB888Pixel*)frame->pFrameData;

			for (int32_t row = 0; row < frame->height; ++row)
			{
				for (int32_t col = 0; col < frame->width; ++col, ++ptr)
				{
					if (col > 0) file << ",";
					file << int(ptr->r);
					file << "," << int(ptr->g);
					file << "," << int(ptr->b);
				}

				file << "\n";
			}

			file.close();
			return LW_RETURN_OK;
		}

		case LW_IR_FRAME:
		{
			auto* ptr = (uint8_t*)frame->pFrameData;
			for (int32_t row = 0; row < frame->height; ++row)
			{
				for (int32_t col = 0; col < frame->width; ++col, ++ptr)
				{
					if (col > 0) file << ",";
					file << int(*ptr);
				}
				file << "\n";
			}

			file.close();
			return LW_RETURN_OK;
		}

		default:
			return LW_RETURN_TYPE_INPUT_ERROR;
		}

	}

	return LW_RETURN_FILE_OPEN_ERROR;
}

LWReturnCode LWSaveRgbAsImageFile(const char* filename, const LWFrameData* frame)
{
	if ((frame->frameType & LW_RGB_FRAME) || (frame->frameType & LW_RGB_TO_DEPTH_FRAME))
	{
		cv::Mat src(frame->height, frame->width, CV_8UC3, frame->pFrameData);
		cv::Mat dst;
		cv::cvtColor(src, dst, cv::COLOR_RGB2BGR);

		if (cv::imwrite(filename, dst)) return LW_RETURN_OK;

		gGlobal.errorInfo = "The cv::imwrite function failed to execute.";

		return LW_RETURN_CUSTOM_ERROR;
	}

	return LW_RETURN_TYPE_INPUT_ERROR;
}

LWReturnCode LWUpdateFirmware(LWDeviceHandle handle, const char* filename)
{
	if (!gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (gGlobal.numDeviceMap.find(handle) == gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return gGlobal.numDeviceMap[handle]->UpdateFirmware(filename);
}

LWReturnCode LWUpdateFirmware1(const char* ip, const char* filename)
{
	LOG_INFO_OUT("<%s>", ip);
	if (!gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;

	UpdateToolTask task(ip, 22, "root", "lv-zgyfjch", filename, 3000);
	UpdateResultType result = task.processor();

	if (result == RESULT_OK)
	{
		return LW_RETURN_OK;
	}
	else
	{
		gGlobal.errorInfo = result.desc;
		return LW_RETURN_FIRMWARE_UPDATE_FAIL;
	}
}


#ifdef LW_INTERNAL_API

LWReturnCode LWSendFile(LWDeviceHandle handle, const char* fullname, LWFileType type)
{
	if (!gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (gGlobal.numDeviceMap.find(handle) == gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return gGlobal.numDeviceMap[handle]->SendFile(fullname, type);
}

LWReturnCode LWSetDeviceSN(LWDeviceHandle handle, const char* sn, int size)
{
	if (!gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (gGlobal.numDeviceMap.find(handle) == gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return gGlobal.numDeviceMap[handle]->SetDeviceSN(sn, size);
}

LWReturnCode LWSendOperateCommand(LWDeviceHandle handle, const char* comstr, int size)
{
	if (!gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (gGlobal.numDeviceMap.find(handle) == gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return gGlobal.numDeviceMap[handle]->SendOperateCommand(comstr, size);
}

LWReturnCode LWSetBinningMode(LWDeviceHandle handle, LWBinningMode mode)
{
	if (!gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (gGlobal.numDeviceMap.find(handle) == gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return gGlobal.numDeviceMap[handle]->SetBinningMode(mode);
}

LWReturnCode LWSetDRNU(LWDeviceHandle handle, bool enable)
{
	if (!gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (gGlobal.numDeviceMap.find(handle) == gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return gGlobal.numDeviceMap[handle]->SetDRNU(enable);
}

LWReturnCode LWSetDistortionCalibration(LWDeviceHandle handle, LWSensorType sensorType, bool enable)
{
	if (!gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (gGlobal.numDeviceMap.find(handle) == gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return gGlobal.numDeviceMap[handle]->SetDistortionCalibration(sensorType, enable);
}

LWReturnCode LWSetLaserWorkFrequency(LWDeviceHandle handle, const uint8_t* arr, int size)
{
	if (!gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (gGlobal.numDeviceMap.find(handle) == gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return gGlobal.numDeviceMap[handle]->SetLaserWorkFrequency(arr, size);
}

LWReturnCode LWSetAutoExposureDefaultValue(LWDeviceHandle handle, uint16_t val)
{
	if (!gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (gGlobal.numDeviceMap.find(handle) == gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return gGlobal.numDeviceMap[handle]->SetAutoExposureDefaultValue(val);
}

LWReturnCode LWSetIntrinsicParam(LWDeviceHandle handle, LWSensorType sensorType, LWSensorIntrinsicParam para)
{
	if (!gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (gGlobal.numDeviceMap.find(handle) == gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return gGlobal.numDeviceMap[handle]->SetIntrinsicParam(sensorType, para);
}

LWReturnCode LWSetExtrinsicParam(LWDeviceHandle handle, LWSensorType sensorType, LWSensorExtrinsicParam para)
{
	if (!gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (gGlobal.numDeviceMap.find(handle) == gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return gGlobal.numDeviceMap[handle]->SetExtrinsicParam(sensorType, para);
}

LWReturnCode LWSetTemperatureCompensation(LWDeviceHandle handle, bool enable)
{
	if (!gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (gGlobal.numDeviceMap.find(handle) == gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return gGlobal.numDeviceMap[handle]->SetTemperatureCompensation(enable);
}

LWReturnCode LWGetTemperatureCompensation(LWDeviceHandle handle, bool* enable)
{
	if (!gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (gGlobal.numDeviceMap.find(handle) == gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return gGlobal.numDeviceMap[handle]->GetTemperatureCompensation(*enable);
}

LWReturnCode LWSetTemperatureParams(LWDeviceHandle handle, LWTemperatureParams val)
{
	if (!gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (gGlobal.numDeviceMap.find(handle) == gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return gGlobal.numDeviceMap[handle]->SetTemperatureParams(val);
}

LWReturnCode LWSetLaserEnableStatus(LWDeviceHandle handle, uint32_t flag)
{
	if (!gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (gGlobal.numDeviceMap.find(handle) == gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return gGlobal.numDeviceMap[handle]->SetLaserEnableStatus(flag);
}

LWReturnCode LWGetLaserEnableStatus(LWDeviceHandle handle, uint32_t* flag)
{
	if (!gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (gGlobal.numDeviceMap.find(handle) == gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return gGlobal.numDeviceMap[handle]->GetLaserEnableStatus(*flag);
}

LWReturnCode LWSetDataAlignEnable(LWDeviceHandle handle, bool enable)
{
	if (!gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (gGlobal.numDeviceMap.find(handle) == gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return gGlobal.numDeviceMap[handle]->SetDataSyncEnable(enable);
}


#endif