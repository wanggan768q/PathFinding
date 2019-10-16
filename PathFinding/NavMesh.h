/*
 * NavMesh.h
 *
 *  Created on: 2013-10-14
 *      Author: Lufeng
 */

#ifndef NAVMESH_H_
#define NAVMESH_H_

#define _SILENCE_STDEXT_HASH_DEPRECATION_WARNINGS

#include <hash_map>
#include <iterator>
#include <utility>

#include "DetourNavMesh.h"
#include "DetourNavMeshQuery.h"

using namespace std;

// 寻路用到的最大值数据
static const int MAX_POLYS = 512;
static const int MAX_SMOOTH = 4096;

// 导航网格数据，从.nav文件中读取到的数据存在这里。
hash_map<int, dtNavMesh*> navMeshs;

// 用于寻路的对象, 提供寻路算法和Dijkstra寻路算法。 portId_sn -> navMeshQuery
hash_map<int, dtNavMeshQuery*> navMeshQuerys;

// 坐标中心扩展坐标轴单位距离
float m_polyPickExt[3] = { 0.1, 500, 0.1 };
float m_polyPickExt1[3] = { 2, 4000, 2 };
#endif /* NAVMESH_H_ */
