
#include <iostream>
#include "LWD3Api.h"
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>


// 不要做耗时处理， 尽量轻便
void frameReadyCallback(LWDeviceHandle handle, void* pUserData)
{
	auto ret = LWGetFrameReady(handle);
	if (ret != LW_RETURN_OK)
	{
		printf("LWGetFrameReady function call failed: %s\n", LWGetReturnCodeDescriptor(ret));
		
		// 根据实际情况做相应处理

		return ;
	}
}
// 不要做耗时处理， 尽量轻便
void networkMonitoringCallback(LWDeviceHandle handle, const char* error, void* pUserData)
{
	// When the device does not have an RGB module, rgb is a null pointer.
	if (pUserData != nullptr)
	{
		// 
	}

	/// Do some custom operations after disconnection.
	printf("\n\nDevice Handle: %llu\nError: %s\n\n", handle, error);
	/// 
}


int main(int argc, char* argv[])
{
	auto ret = LWInitializeResources();
	if (ret != LW_RETURN_OK)
	{
		printf("LWInitializeResources function call failed: %s\n\n", LWGetReturnCodeDescriptor(ret));
		return 0;
	}

	// 注册网络回调函数
	LWRegisterNetworkMonitoringCallback(networkMonitoringCallback, nullptr);

	////printf("固件更新中......\n");
	////ret = LWUpdateFirmware1("192.168.1.200", "/home/ljh/data/updata_enc_v1.0.31.sh");
	//ret = LWUpdateFirmware1("192.168.1.205", "C:/Users/12267/Desktop/DATA/update_packets/updata_enc_v1.0.32.sh");
	//if (ret != LW_RETURN_OK)
	//{
	//	printf("LWUpdateFirmware function call failed: %s\n\n", LWGetReturnCodeDescriptor(ret));
	//	return 0;
	//}
	//printf("固件更新完成\n");
	//return 0;

	LWDeviceHandle handleList[5];
	LWDeviceInfo deviceInfoList[5];
	int32_t findCount = 0;
	ret = LWGetDeviceInfoList(deviceInfoList, 5, &findCount);
	if (ret != LW_RETURN_OK)
	{
		printf("LWFindDevices function call failed: %s\n\n", LWGetReturnCodeDescriptor(ret));
		return 0;
	}
	if (findCount < 1)
	{
		printf("No device found.");
		return 0;
	}

	for (int i = 0; i < findCount; i++)
	{
		printf("设备编号: %d, \n设备 SN: %s, 设备 Type: %s, \n设备 IP: %s, 本机 IP: %s, \n设备句柄: %llu\n\n",
			i,
			deviceInfoList[i].sn,
			deviceInfoList[i].type,
			deviceInfoList[i].ip,
			deviceInfoList[i].local_ip,
			deviceInfoList[i].handle
		);

		handleList[i] = deviceInfoList[i].handle;
	}

	int index = 0;
	if (findCount > 1)
	{
		std::cout << "Please enter the index value: ";
		std::cin >> index;
	}

	////printf("固件更新中......\n");
	////ret = LWUpdateFirmware1("192.168.1.200", "/home/ljh/data/updata_enc_v1.0.31.sh");
	//ret = LWUpdateFirmware1(deviceInfoList[index].ip, "C:/Users/12267/Desktop/DATA/update_packets/updata_enc_v1.0.28.sh");
	//if (ret != LW_RETURN_OK)
	//{
	//	printf("LWUpdateFirmware function call failed: %s\n\n", LWGetReturnCodeDescriptor(ret));
	//	return 0;
	//}
	//printf("固件更新完成\n");
	//return 0;

	// 打开设备
	ret = LWOpenDevice(handleList[index]);
	if (ret != LW_RETURN_OK)
	{
		printf("\nLWOpenDevice function call failed: %s\n\n", LWGetReturnCodeDescriptor(ret));
		return 0;
	}

	// 显示设备版本信息
	LWVersionInfo dv;
	LWVersionInfo fv;
	ret = LWGetDeviceVersion(handleList[index], &fv, &dv);
	if (ret != LW_RETURN_OK)
	{
		printf("LWGetDeviceVersion function call failed: %s\n\n", LWGetReturnCodeDescriptor(ret));
		return 0;
	}
	printf("\n\n固件版本: %d.%d.%d.%d\n\n", fv.major, fv.minor, fv.patch, fv.reserved);
	printf("驱动版本: %d.%d.%d.%d\n\n", dv.major, dv.minor, dv.patch, dv.reserved);

	// 发送畸变文件
	//ret = LWSendFile(handleList[0], "C:/Users/12267/Desktop/DATA/DrnuLut_high.txt", LW_DRNU_HIGH);
	//if (ret != LW_RETURN_OK)
	//{
	//	printf("LWSendFile function call failed: %s\n\n", LWGetReturnCodeDescriptor(ret));
	//}
	//return 0;

	//// 网络配置
	//LWNetworkInfo ninfo;
	//ninfo.type = 0x01;
	//memcpy(ninfo.ip, "192.168.1.205", 13);
	//memcpy(ninfo.netmask, "255.255.255.0", 13);
	//ret = LWSetNetworkInfo(handleList[index], ninfo);
	//if (ret != LW_RETURN_OK)
	//{
	//	printf("LWSetNetworkInfo function call failed: %s\n\n", LWGetReturnCodeDescriptor(ret));
	//	return 0;
	//}
	//_sleep(25000);
	//ret = LWOpenDevice(handleList[index]);
	//if (ret != LW_RETURN_OK)
	//{
	//	printf("\nLWOpenDevice function call failed: %s\n\n", LWGetReturnCodeDescriptor(ret));
	//	return 0;
	//}
	//LWNetworkInfo ninfo;
	//ret = LWGetNetworkInfo(handleList[index], &ninfo);
	//if (ret != LW_RETURN_OK)
	//{
	//	printf("LWGetNetworkInfo function call failed: %s\n\n", LWGetReturnCodeDescriptor(ret));
	//	return 0;
	//}
	//printf("IP: %s, Type: %u, Mask: %s\n", ninfo.ip, ninfo.type, ninfo.netmask);

	ret = LWSetExposureMode(handleList[index], LW_TOF_SENSOR, LWExposureMode::LW_EXPOSURE_MANUAL);
	if (ret != LW_RETURN_OK)
	{
		printf("LWSetExposureMode function call failed: %s\n\n", LWGetReturnCodeDescriptor(ret));
		return 0;
	}

	ret = LWSetFrameRate(handleList[index], 10);
	if (ret != LW_RETURN_OK)
	{
		printf("LWSetFrameRate function call failed: %s\n\n", LWGetReturnCodeDescriptor(ret));
		return 0;
	}

	int earr[2] = {1500, 1200};
	ret = LWSetExposureTime(handleList[index], LW_TOF_SENSOR, earr, 2);
	if (ret != LW_RETURN_OK)
	{
		printf("LWGetExposureTime function call failed: %s\n\n", LWGetReturnCodeDescriptor(ret));
		return 0;
	}
	printf("Set Exposure Time: %d, %d\n", earr[0], earr[1]);

	//ret = LWSetTransformDepthToRgbEnable(handleList[index], true);
	//if (ret != LW_RETURN_OK)
	//{
	//	printf("LWSetTransformDepthToRgbEnable function call failed: %s\n\n", LWGetReturnCodeDescriptor(ret));
	//}

	//ret = LWSetTransformRgbToDepthEnable(handleList[index], true);
	//if (ret != LW_RETURN_OK)
	//{
	//	printf("LWSetTransformRgbToDepthEnable function call failed: %s\n\n", LWGetReturnCodeDescriptor(ret));
	//}

	ret = LWSetTriggerMode(handleList[index], LW_TRIGGER_ACTIVE);
	if (ret != LW_RETURN_OK)
	{
		printf("LWSetTriggerMode function call failed: %s\n\n", LWGetReturnCodeDescriptor(ret));
		return 0;
	}

	ret = LWSetSpatialFilterParams(handleList[index], LWFilterParam{ false, 3 });
	if (ret != LW_RETURN_OK)
	{
		printf("LWSetSpatialFilterParams function call failed: %s\n\n", LWGetReturnCodeDescriptor(ret));
		return 0;
	}

	ret = LWSetTimeFilterParams(handleList[index], LWFilterParam{ false, 3, 100 });
	if (ret != LW_RETURN_OK)
	{
		printf("LWSetTimeFilterParams function call failed: %s\n\n", LWGetReturnCodeDescriptor(ret));
		return 0;
	}

	ret = LWSetTimeMedianFilterParams(handleList[index], LWFilterParam{ false, 3, 100 });
	if (ret != LW_RETURN_OK)
	{
		printf("LWSetTimeMedianFilterParams function call failed: %s\n\n", LWGetReturnCodeDescriptor(ret));
		return 0;
	}

	ret = LWSetFlyingPixelsFilterParams(handleList[index], LWFilterParam{ true, 5 });
	if (ret != LW_RETURN_OK)
	{
		printf("LWSetFlyingPixelsFilterParams function call failed: %s\n\n", LWGetReturnCodeDescriptor(ret));
		return 0;
	}

	ret = LWSetConfidenceFilterParams(handleList[index], LWFilterParam{ true, 5 });
	if (ret != LW_RETURN_OK)
	{
		printf("LWSetConfidenceFilterParams function call failed: %s\n\n", LWGetReturnCodeDescriptor(ret));
		return 0;
	}

	ret = LWStartStream(handleList[index]);
	if (ret != LW_RETURN_OK)
	{
		printf("LWStartStream function call failed: %s\n\n", LWGetReturnCodeDescriptor(ret));
		return 0;
	}

	LWFrameData frame;
	int64_t t0 = 0;
	int64_t count = 0;
	int64_t err_count = 0;
	while (true)
	{
		ret = LWGetFrameReady(handleList[index]);
		if (ret != LW_RETURN_OK)
		{
			printf("LWGetFrameReady function call failed: %s\n", LWGetReturnCodeDescriptor(ret));
			if (++err_count > 3) break;
			continue;
		}
		err_count = 0;

		//ret = LWGetFrame(handleList[index], &frame, LWFrameType::LW_RGB_TO_DEPTH_FRAME);
		//ret = LWGetFrame(handleList[index], &frame, LWFrameType::LW_DEPTH_TO_RGB_FRAME);
		//ret = LWGetFrame(handleList[index], &frame, LWFrameType::LW_IR_FRAME);
		//ret = LWGetFrame(handleList[index], &frame, LWFrameType::LW_AMPLITUDE_FRAME);
		//ret = LWGetFrame(handleList[index], &frame, LWFrameType::LW_RGB_FRAME);
		//ret = LWGetFrame(handleList[index], &frame, LWFrameType::LW_POINTCLOUD_FRAME);
		ret = LWGetFrame(handleList[index], &frame, LWFrameType::LW_DEPTH_FRAME);
		if (ret != LW_RETURN_OK)
		{
			printf("LWGetFrame function call failed: %s\n\n", LWGetReturnCodeDescriptor(ret));
			continue;
		}

		// 帧率显示
		if (t0 != frame.timestamp.tv_sec)
		{
			printf("\nFPS: %lld ", count / (frame.timestamp.tv_sec - t0));

			t0 = frame.timestamp.tv_sec;
			count = 0;
		}
		++count;

		// 帧信息显示
		printf("\nframe index: %u, timestamp: %lld.%lld, chip: %.2f℃, laser1: %.2f℃, laser2: %.2f℃, frameType: %u, width: %u, height: %u, bufferSize: %u, elemSize: %u, total: %u",
			frame.frameIndex, 
			frame.timestamp.tv_sec, frame.timestamp.tv_usec, 
			frame.temperature.chip, frame.temperature.laser1, frame.temperature.laser2,
			frame.frameType, 
			frame.width, frame.height, 
			frame.bufferSize, 
			frame.elemSize, 
			frame.total);

		//// IR图像显示
		//cv::Mat img(frame.height, frame.width, CV_8UC1, frame.pFrameData);
		//cv::namedWindow("img", 0);
		//cv::resizeWindow("img", 640, 480);
		//cv::imshow("img", img);
		//cv::waitKey(30);

		// Depth图显示
		cv::Mat dst;
		cv::Mat src(frame.height, frame.width, CV_16UC1, frame.pFrameData);
		src.convertTo(dst, CV_8UC1, 255.0 / 6000);
		cv::namedWindow("dst", 0);
		cv::resizeWindow("dst", 640, 480);
		cv::imshow("dst", dst);
		cv::waitKey(30);
		
		//// RGB图像显示
		//cv::Mat img(frame.height, frame.width, CV_8UC3, frame.pFrameData);
		//cv::cvtColor(img, img, cv::COLOR_RGB2BGR);
		//cv::namedWindow("img", 0);
		//cv::resizeWindow("img", 640, 480);
		//cv::imshow("img", img);
		//cv::waitKey(30);

		//// 保存图像
		//cv::imwrite("C:/Users/12267/Desktop/kimg.bmp", img);
		//cv::imwrite("C:/Users/12267/Desktop/img1.png", img);
		//cv::imwrite("C:/Users/12267/Desktop/kimg.jpg", img);
		
		//// 保存数据
		//ret = LWSaveDataAsCSVFile("C:/Users/12267/Desktop/dep.csv", &frame);
		//if (ret != LW_RETURN_OK)
		//{
		//	printf("LWGetFrame function call failed: %s\n\n", LWGetReturnCodeDescriptor(ret));
		//	//break;
		//}

		if (frame.frameIndex > 50) break;
	}

	ret = LWStopStream(handleList[index]);
	if (ret != LW_RETURN_OK)
	{
		printf("\nLWStopStream function call failed: %s\n\n", LWGetReturnCodeDescriptor(ret));
		return 0;
	}

	ret = LWCloseDevice(handleList[index]);
	if (ret != LW_RETURN_OK)
	{
		printf("LWCloseDevice function call failed: %s\n\n", LWGetReturnCodeDescriptor(ret));
		return 0;
	}

	LWCleanupResources();

	printf("\n退出\n");

	return 0;
}
