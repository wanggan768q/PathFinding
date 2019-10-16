LOCAL_PATH := $(call my-dir)
include $(CLEAR_VARS) 

LOCAL_MODULE := PathFinding
LOCAL_CPPFLAGS := -O2

LOCAL_C_INCLUDES += $(LOCAL_PATH)/include

LOCAL_SRC_FILES += DetourAlloc.cpp\
                   DetourCommon.cpp\
                   DetourInit.cpp\
                   DetourNavMesh.cpp\
                   DetourNavMeshBuilder.cpp\
                   DetourNavMeshQuery.cpp\
                   DetourNode.cpp\
                   PathFinding.cpp

include $(BUILD_SHARED_LIBRARY)