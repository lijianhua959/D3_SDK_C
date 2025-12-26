# CMakeList.txt: D3_SDK_C 的 CMake 项目，在此处包括源代码并定义
# 项目特定的逻辑。
#

project("d3_c_sdk")


if(WIN32)# Windows 平台

	# 引入第三方库
	if (CMAKE_SIZEOF_VOID_P EQUAL 8)
		if(CMAKE_BUILD_TYPE STREQUAL "Debug")
			set(OpenCV_DIR D:/ThirdPartyLib/opencv/x64d/x64/vc17/staticlib)
			set(Update_ROOT_DIR D:/ThirdPartyLib/Firmware_Update/x64d)
		else()
			set(OpenCV_DIR D:/ThirdPartyLib/opencv/x64/x64/vc17/staticlib)
			set(Update_ROOT_DIR D:/ThirdPartyLib/Firmware_Update/x64)
	endif()
		set(libjpeg_turbo_DIR D:/ThirdPartyLib/libjpeg_turbo/x64)
		set(OPENSSL_ROOT_DIR D:/ThirdPartyLib/openssl-3.4.0/x64)
	else()
		set(OpenCV_DIR D:/ThirdPartyLib/opencv/x86/x86/vc17/staticlib)
		set(libjpeg_turbo_DIR D:/ThirdPartyLib/libjpeg_turbo/x86)
		set(OPENSSL_ROOT_DIR D:/ThirdPartyLib/openssl-3.4.0/x86)
		set(Update_ROOT_DIR D:/ThirdPartyLib/Firmware_Update/x86)
	endif()

	find_package(OpenCV REQUIRED)
	find_package(OpenSSL REQUIRED)

	include_directories(${libjpeg_turbo_DIR}/include)
	include_directories(${OPENSSL_INCLUDE_DIR})
	include_directories(${Update_ROOT_DIR}/include)


	# 将源代码添加到此项目的动态库。
	add_library (d3_c_sdk SHARED "LWD3Api.cpp" "LWD3Api.h" "LWD3Type.h" "resource.h" "Version.rc" )

	# 链接库
	target_link_libraries(d3_c_sdk  
	${OpenCV_LIBS} 
	${libjpeg_turbo_DIR}/lib/turbojpeg-static.lib 
	${OPENSSL_CRYPTO_LIBRARIES}
	crypt32.lib
	${Update_ROOT_DIR}/lib/libssh2.lib 
	${Update_ROOT_DIR}/lib/firmware_update.lib 
	)

	# 将生成的动态库复制到可执行程序的目录下
	add_custom_command(TARGET d3_c_sdk POST_BUILD COMMAND "${CMAKE_COMMAND}" -E copy d3_c_sdk.dll "../")


	if (CMAKE_VERSION VERSION_GREATER 3.12)
		set_property(TARGET d3_c_sdk PROPERTY CXX_STANDARD 20)
	endif()

	# TODO: 如有需要，请添加测试并安装目标。

elseif(UNIX) # Linux 平台


else() # 其他平台

	message(STATUS "Current platform: Unknown")

endif()

