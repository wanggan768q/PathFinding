#ifndef PATHFINDING_H_
#define PATHFINDING_H_

#include <iostream>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>

#include "NavMesh.h"

#include "DetourCommon.h"
#include "DetourInit.h"

/*
typedef void(* DebugCallback) (const char *str);

extern "C" void RegisterDebugCallback(DebugCallback callback);
*/
extern "C" bool LoadNavData(unsigned char *data);

extern "C" void UnLoadNavData();

extern "C" bool FindNavPath(float *start, float *end, int jflag, float *&outarr, int &len);

extern "C" bool Raycast(float *start, float *end, int jflag, float *&outarr, float *&edgeDir);

extern "C" bool IsPosInBlock(float *jpos);

extern "C" void ClearIntPtr(void *pBuffer);

extern "C" void FindNearestPoly(float *start, float *end, int flag, int &nearRef, float *&pt);

extern "C" bool IsWalkable(float *start, int flag);

extern "C" float GetPolyHeight(float *point, int flag);

extern "C" void SetPolyPickExtern(float x, float y, float z);
#endif