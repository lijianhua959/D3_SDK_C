
#include <iostream>
#include "LWD3Api.h"
#include <windows.h>
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
	printf("Device Handle: %llu\nError: %s\n", handle, error);
	/// 
}

void showMat_Thre(cv::Mat input, int Thre, int w = 640, int h = 480) {
	//// 将 CV_32F 映射到 CV_8U 范围
	//cv::Mat mappedMat, tempMat;
	//tempMat = input.clone();
	//tempMat.setTo(0, tempMat > Thre);
	//cv::normalize(tempMat, mappedMat, 0, 255, cv::NORM_MINMAX, CV_8U);
	////tempMat.convertTo(mappedMat, CV_8UC1, 255.0 / 6000);

	//// 创建灰度图像
	//cv::Mat grayscaleImage(mappedMat.size(), CV_8U);

	// 复制数据到灰度图像
	//mappedMat.copyTo(grayscaleImage);

	cv::Mat mappedMat;
	cv::normalize(input, mappedMat, 0, 255, cv::NORM_MINMAX, CV_8UC1);

	cv::namedWindow("Grayscale Visualization", 0);
	cv::resizeWindow("Grayscale Visualization", w, h);

	// 显示可视化结果
	cv::imshow("Grayscale Visualization", mappedMat);
	cv::waitKey(20);

	return;
}


int main(int argc, char* argv[])
{
	auto ret = LWInitializeResources();
	if (ret != LW_RETURN_OK)
	{
		printf("LWInitializeResources function call failed: %s\n\n", LWGetReturnCodeDescriptor(ret));
		return 0;
	}

	LWDeviceHandle handleList[5];
	LWDeviceInfo deviceInfoList[5];
	int32_t findCount = 0;
	ret = LWGetDeviceInfoList(deviceInfoList, 5, &findCount);
	if (ret != LW_RETURN_OK)
	{
		printf("LWFindDevices function call failed: %s\n\n", LWGetReturnCodeDescriptor(ret));
		system("pause");
		return 0;
	}
	if (findCount < 1)
	{
		printf("No device found.");
		system("pause");
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

	//printf("固件更新中......\n");
	//ret = LWUpdateFirmware1("192.168.1.200", "C:/Users/12267/Desktop/DATA/update_packets/updata_enc_v1.0.29.sh");
	//if (ret != LW_RETURN_OK)
	//{
	//	printf("\LWUpdateFirmware function call failed: %s\n\n", LWGetReturnCodeDescriptor(ret));
	//	return 0;
	//}
	//printf("固件更新完成\n");
	//return 0;

	int index = 0;
	if (findCount > 1)
	{
		std::cout << "Please enter the index value: ";
		std::cin >> index;
		ret = LWOpenDevice(handleList[index]);
	}
	else
	{
		ret = LWOpenDevice(handleList[index]);
	}
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

	char sn[20];
	char ty[20] = {};
	ret = LWGetDeviceSN(handleList[index], sn, 20);
	if (ret != LW_RETURN_OK)
	{
		printf("LWGetDeviceSN function call failed: %s\n\n", LWGetReturnCodeDescriptor(ret));
		return 0;
	}
	//ret = LWGetDeviceType(handleList[index], ty, 20);
	//if (ret != LW_RETURN_OK)
	//{
	//	printf("\LWGetDeviceType function call failed: %s\n\n", LWGetReturnCodeDescriptor(ret));
	//	//return 0;
	//}
	//printf("\n设备SN号：%s, 型号：%s\n\n", sn, ty);


	// 发送畸变文件
	//// ret_code = LWSendFile(handleList[0], "D:\\Data\\2511A2D310011\\Bending_LUT.txt", LW_BENDING_LUT);
	//// ret_code = LWSendFile(handleList[0], "C:\\Users\\l\\Documents\\2506ABDM40044\\DrnuLut_high.txt", LW_DRNU_HIGH);
	//ret = LWSendFile(handleList[0], "C:/Users/12267/Desktop/DATA/DrnuLut_high.txt", LW_DRNU_HIGH);
	//if (ret != LW_RETURN_OK)
	//{
	//	printf("LWSendFile function call failed: %s\n\n", LWGetReturnCodeDescriptor(ret));
	//	//return 0;
	//}
	//return 0;

	ret = LWSetFrameRate(handleList[index], 10);
	if (ret != LW_RETURN_OK)
	{
		printf("LWSetFrameRate function call failed: %s\n\n", LWGetReturnCodeDescriptor(ret));
		return 0;
	}

	LWNetworkInfo ninfo;
	ret = LWGetNetworkInfo(handleList[index], &ninfo);
	//ninfo.type = 0x00;
	//memcpy(ninfo.ip, "192.168.1.200", 13);
	//memcpy(ninfo.netmask, "255.255.255.0", 13);
	//ret = LWSetNetworkInfo(handleList[index], ninfo);
	if (ret != LW_RETURN_OK)
	{
		printf("LWGetNetworkInfo function call failed: %s\n\n", LWGetReturnCodeDescriptor(ret));
		return 0;
	}
	printf("IP: %s, Type: %u, Mask: %s\n", ninfo.ip, ninfo.type, ninfo.netmask);

	int earr[2] = {1500, 1200};
	ret = LWSetExposureTime(handleList[index], LW_TOF_SENSOR, earr, 2);
	if (ret != LW_RETURN_OK)
	{
		printf("LWGetExposureTime function call failed: %s\n\n", LWGetReturnCodeDescriptor(ret));
		return 0;
	}
	printf("Set Exposure Time: %d, %d\n", earr[0], earr[1]);

	LWExposureMode emd;
	ret = LWGetExposureMode(handleList[index], LW_TOF_SENSOR, &emd);
	if (ret != LW_RETURN_OK)
	{
		printf("LWGetExposureMode function call failed: %s\n\n", LWGetReturnCodeDescriptor(ret));
		return 0;
	}
	printf("\n设备曝光模式：%d\n\n", emd);

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

	ret = LWStartStream(handleList[index]);
	if (ret != LW_RETURN_OK)
	{
		printf("LWStartStream function call failed: %s\n\n", LWGetReturnCodeDescriptor(ret));
		return 0;
	}

	LWFrameData frame;
	int64_t t0 = 0;
	int64_t fps = 0;
	int count = 0;
	while (true)
		//while (true)
	{
		ret = LWGetFrameReady(handleList[index]);
		if (ret != LW_RETURN_OK)
		{
			printf("LWGetFrameReady function call failed: %s\n", LWGetReturnCodeDescriptor(ret));
			//break;
		}

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
			//break;
		}

		if (t0 != frame.timestamp.tv_sec)
		{
			printf("\ndata frame rate: %lld", fps / (frame.timestamp.tv_sec - t0));

			t0 = frame.timestamp.tv_sec;
			fps = 0;
		}
		++fps;

		//printf("\n(%u, %u)\
		//		\nframeIndex: %u \
		//		\nbufferSize: %u \
		//		\nelemSize: %u \
		//		\ntotal: %u \
		//		\nframeType: %u\n\n \ ", frame.width, frame.height, frame.frameIndex, frame.bufferSize, frame.elemSize, frame.total, frame.frameType);

		cv::Mat img(frame.height, frame.width, CV_16UC1, frame.pFrameData);
		showMat_Thre(img, 7500);

		//cv::Mat img(frame.height, frame.width, CV_8UC3, frame.pFrameData);
		//cv::namedWindow("img", 0);
		//cv::resizeWindow("img", 640, 480);
		//cv::imshow("img", img);
		//cv::waitKey(5);
	}

	ret = LWStopStream(handleList[index]);
	if (ret != LW_RETURN_OK)
	{
		printf("LWStopStream function call failed: %s\n\n", LWGetReturnCodeDescriptor(ret));
		return 0;
	}

	ret = LWCloseDevice(handleList[index]);
	if (ret != LW_RETURN_OK)
	{
		printf("LWCloseDevice function call failed: %s\n\n", LWGetReturnCodeDescriptor(ret));
		return 0;
	}

	ret = LWCleanupResources();
	if (ret != LW_RETURN_OK)
	{
		printf("LWCleanupResources function call failed: %s\n\n", LWGetReturnCodeDescriptor(ret));
	}

	printf("\n退出\n");

	system("pause");

	return 0;
}
