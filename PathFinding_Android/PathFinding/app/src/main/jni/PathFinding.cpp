#define EXPORTBUILD
#include "PathFinding.h"
#include <sstream>
dtNavMesh *navMesh = NULL;
dtNavMeshQuery *navMeshQuery = NULL;

//void  SetDebugFunction(FuncPtr fp)
//{
//	Debug = fp;
//}

//DebugCallback gDebugCallback;
//
//void RegisterDebugCallback(DebugCallback callback)
//{
//	if (callback)
//	{
//		gDebugCallback = callback;
//	}
//}

//void LogToUnity(const char *msg)
//{
//	if (gDebugCallback)
//	{
//		gDebugCallback(msg);
//	}
//}

bool LoadNavData(unsigned char *data)
{
    UnLoadNavData();

    navMesh = dtLoadNavMesh(data);
    // 初始化查询对象
    navMeshQuery = dtAllocNavMeshQuery();
    if (!navMeshQuery)
    {
        cerr << "不能创建寻路查询对象Could not create Detour navMeshQuery, stageInfo:" << endl;
        return false;
    }

    // 初始化寻路查询对象，如果错误 直接返回
    dtStatus status = navMeshQuery->init(navMesh, 3072);
    if (dtStatusFailed(status))
    {
        cerr << "不能初始化查询对象Could not init Detour navMesh query, stageInfo:" << endl;
        return false;
    }

    //// 缓存到map中
    //navMeshQuerys.insert(std::pair<int, dtNavMeshQuery*>(jstage, navMeshQuery));
    return true;
}

void UnLoadNavData()
{
    if (navMesh != NULL)
    {
        dtFreeNavMesh(navMesh);
        navMesh = NULL;
    }

    if (navMeshQuery != NULL)
    {
        dtFreeNavMeshQuery(navMeshQuery);
        navMeshQuery = NULL;
    }
}

bool FindNavPath(float *start, float *end, int jflag, float *&outarr, int &len)
{
    bool findPath = true;
    try
    {
        // 起始位置,终点位置
        float m_spos[3], m_epos[3];

        // 格式转换，从jfloatArray转为jfloat
        /*float* start = env->GetFloatArrayElements(start, JNI_FALSE);
        float* end = env->GetFloatArrayElements(end, JNI_FALSE);*/

        if (start == NULL || end == NULL)
        {
            std::cerr << "out of memory!" << std::endl;
            //return NULL;
        }

        // 拷贝数据，转为float数组
        dtVcopy(m_spos, start);
        dtVcopy(m_epos, end);

        /************************************************************************/
        /* 释放资源                                                                     */
        /************************************************************************/
        /*env->ReleaseFloatArrayElements(jstart, start, 0);
        env->ReleaseFloatArrayElements(jend, end, 0);*/

        // 过滤条件
        dtQueryFilter m_filter;
        m_filter.setIncludeFlags((unsigned int)jflag);
        m_filter.setExcludeFlags(0xffffffff ^ jflag);

        // 距离起始坐标和终点坐标最近的polygon多边形的id
        dtPolyRef m_startRef = 0;
        dtPolyRef m_endRef = 0;
        // 查找距离最近的多边形
        navMeshQuery->findNearestPoly(m_spos, m_polyPickExt, &m_filter, &m_startRef, 0);
        navMeshQuery->findNearestPoly(m_epos, m_polyPickExt, &m_filter, &m_endRef, 0);

        // 如果没找到起点
        if (!m_startRef)
        {
            std::cerr << "stageInfo:"
                      << ", Could not find startPoly: " << m_startRef << ", from startPos(" << m_spos[0] << "," << m_spos[2] << ") to endPos(" << m_epos[0] << "," << m_epos[2] << ")" << std::endl;
            //return NULL;
        }
        // 如果没找到终点, 说明终点在阻挡区域, raycast找直线到终点被阻挡的点, 重新找多边形
        if (!m_endRef)
        {
            // raycast找直线到终点被阻挡的点
            float *endBlock = NULL;
            float *edgeDir = NULL;
            Raycast(start, end, jflag, endBlock, edgeDir);
            float *end = endBlock;
            if (end == NULL)
            {
                std::cerr << "out of memory!" << std::endl;
                //return NULL;
            }

            dtVcopy(m_epos, end);
            delete[] end;
            end = NULL;
            // 释放资源
            //env->ReleaseFloatArrayElements(endBlock, end, 0);

            navMeshQuery->findNearestPoly(m_epos, m_polyPickExt, &m_filter, &m_endRef, 0);
            findPath = false;
            // 找起点随机能到的附近的点
            //navMeshQuery->findRandomPointAroundCircle(m_startRef, m_spos, 2, &m_filter, frand, &m_endRef, m_epos);
            if (!m_endRef)
            {
                std::cerr << "stageInfo:"
                          << ", Could not find endPoly: " << m_endRef << ", from startPos(" << m_spos[0] << "," << m_spos[2] << ") to endPos(" << m_epos[0] << "," << m_epos[2] << ")" << std::endl;
                //return NULL;
            }
        }

        // 记录找到的路径上的多边形id的数组，起点->终点
        dtPolyRef m_polys[MAX_POLYS];
        int m_npolys;                        // 多边形个数
        float m_straightPath[MAX_POLYS * 3]; // 直线路径上坐标数组
        unsigned char m_straightPathFlags[MAX_POLYS];
        dtPolyRef m_straightPathPolys[MAX_POLYS]; //找到的直线路径上多边形id的数组
        float path[MAX_SMOOTH * 3];
        int m_nstraightPath = 0; // 找到的直线距离坐标个数

        // 查询起点多边形->终点多边形的路径
        navMeshQuery->findPath(m_startRef, m_endRef, m_spos, m_epos, &m_filter, m_polys, &m_npolys, MAX_POLYS);

        // 如果不为0找到了 则找直线路径
        if (m_npolys)
        {
            // 找直线路径
            navMeshQuery->findStraightPath(m_spos, m_epos, m_polys, m_npolys, m_straightPath, m_straightPathFlags, m_straightPathPolys, &m_nstraightPath, MAX_POLYS);
            int pos = 0;
            for (int i = 0; i < m_nstraightPath * 3;)
            {
                path[pos++] = m_straightPath[i++];
                path[pos++] = m_straightPath[i++];
                path[pos++] = m_straightPath[i++];
            }
            // append the end point
            path[pos++] = m_epos[0];
            path[pos++] = m_epos[1];
            path[pos++] = m_epos[2];
        }

        /* 将float数组转为jfloatArray作为结果返回 */
        int arrayLen = m_nstraightPath * 3; // 数组长度
        len = arrayLen;
        outarr = new float[arrayLen];
        for (int i = 0; i < arrayLen; i++)
        {
            outarr[i] = path[i];
        }

        if (outarr == NULL)
        {
            std::cerr << "out of memory!" << std::endl;
            //return NULL;
        }

        //env->SetFloatArrayRegion(resultArr, 0, arrayLen, path);
    } /*
     catch (runtime_error e) {
     std::cout << e.what() << "*****出错了something wrong!" << std::endl;
     }
     catch (exception err) {
     std::cout << err.what() << "*****出错了something wrong!" << std::endl;
     }*/
    catch (...)
    {
        findPath = false;
        std::cerr << "Java native interface call c++ findPath() method  error" << std::endl;
    }

    return findPath;
}

bool Raycast(float *start, float *end, int jflag, float *&outarr, float *&oEdgeDir)
{
    float *resultArr = NULL;
    float edgeDir[3] = {0};
    try
    {
        // 起始位置,终点位置
        float m_spos[3], m_epos[3];

        // 格式转换，从jfloatArray转为jfloat
        /*jfloat* start = env->GetFloatArrayElements(jstart, JNI_FALSE);
        jfloat* end = env->GetFloatArrayElements(jend, JNI_FALSE);*/

        if (start == NULL || end == NULL)
        {
            std::cerr << "out of memory!" << std::endl;
            //return NULL;
        }

        // 拷贝数据，转为float数组
        dtVcopy(m_spos, start);
        dtVcopy(m_epos, end);

        //// 释放资源                                                                     */
        //env->ReleaseFloatArrayElements(jstart, start, 0);
        //env->ReleaseFloatArrayElements(jend, end, 0);

        // 过滤条件
        dtQueryFilter m_filter;
        m_filter.setIncludeFlags((unsigned int)jflag);
        m_filter.setExcludeFlags(0xffffffff ^ jflag);

        // 距离起始坐标和终点坐标最近的polygon多边形的id
        dtPolyRef m_startRef = 0;

        // 查找距离最近的多边形
        navMeshQuery->findNearestPoly(m_spos, m_polyPickExt, &m_filter, &m_startRef, 0);

        // 如果没找到
        if (!m_startRef)
        {
            std::cerr << "stageInfo:"
                      << ", Could not find startPos:(" << m_spos[0] << "," << m_spos[2] << "), startPoly:" << m_startRef << std::endl;
            //return NULL;
        }

        float hitNormal[3];
        float t;
        dtPolyRef path[MAX_POLYS];
        int pathCount;
        // raycast判断是否阻挡
        navMeshQuery->raycast(m_startRef, m_spos, m_epos, &m_filter, &t, hitNormal, path, &pathCount, MAX_POLYS, edgeDir);

        int const arrayLen = 3; // 数组长度
        float result[arrayLen];

        // 如果t=FLT_MAX则起点到终点无阻挡，能通过, 如果t=0则起点在阻挡区域, 返回终点坐标
        // 否则0<t<1.0则有阻挡，hitPoint = startPos + (endPos - startPos) * t
        // 以上是官网API附加说明，这里根据raycast的方法注释The hit parameter. (FLT_MAX if no wall hit.)简化处理
        if (t == FLT_MAX)
        {
            dtVcopy(result, m_epos);
        }
        // 否则0<t<1.0则有阻挡，hitPoint = startPos + (endPos - startPos) * t
        else if (0 < t && t < 1)
        {
            for (int i = 0; i < arrayLen; ++i)
                result[i] = m_spos[i] + (m_epos[i] - m_spos[i]) * t;
        }
        // 如果t=0则起点在阻挡区域, 返回终点坐标
        else if (t == 0)
        {
            // 找终点所在多边形
            dtPolyRef m_endRef = 0;
            navMeshQuery->findNearestPoly(m_epos, m_polyPickExt, &m_filter, &m_endRef, 0);

            // 如果终点在阻挡，返回起点，否则返回终点
            if (!m_endRef)
            {
                dtVcopy(result, m_spos);
            }
            else
            {
                dtVcopy(result, m_epos);
            }
        }

        /*
        std::cout << "t:" << t << ", FLT_MAX:" << FLT_MAX << ", pathCount:" << pathCount << std::endl;
        std::cout << "hitNormal:" << hitNormal[0] << ", " << hitNormal[1] << ", " << hitNormal[2] << std::endl;
        std::cout << "result:" << result[0] << ", " << result[1] << ", " << result[2] << std::endl;
        for (int i = 0; i < pathCount; i+=3)
        {
        std::cout << i / 3 << "path:" << path[0] << ", " << path[1] << ", " << path[2] << std::endl;
        }
        */

        /* 将float数组转为jfloatArray作为结果返回 */
        //resultArr = new float[arrayLen];
        outarr = new float[arrayLen];
        dtVcopy(outarr, result);
        if (result == NULL)
        {
            std::cerr << "out of memory!" << std::endl;
            //return NULL;
        }
        //env->SetFloatArrayRegion(resultArr, 0, arrayLen, result);
    } /*
     catch (runtime_error e) {
     std::cout << e.what() << "***** something wrong!" << std::endl;
     }
     catch (exception err) {
     std::cout << err.what() << "***** something wrong!" << std::endl;
     }*/
    catch (...)
    {
        std::cerr << "Java native interface call c++ Raycast() method error" << std::endl;
    }

    oEdgeDir = new float[3];
    memcpy(oEdgeDir, edgeDir, sizeof(float) * 3);
    return true;
    //return resultArr;
}

/**
* 判断某坐标是否在阻挡区域内
*/
bool IsPosInBlock(float *jpos)
{
    // 距离该坐标最近的polygon多边形的id
    dtPolyRef posRef = 0;
    try
    {
        // 格式转换
        float pos[3];
        float *tmp = jpos;
        if (tmp == NULL)
        {
            std::cerr << "out of memory!" << std::endl;
            return true;
        }

        dtVcopy(pos, tmp);
        //清理
        delete[] jpos;
        // 过滤条件
        dtQueryFilter m_filter;
        m_filter.setIncludeFlags(0xffff);
        m_filter.setExcludeFlags(0);

        // 查找距离最近的多边形
        navMeshQuery->findNearestPoly(pos, m_polyPickExt, &m_filter, &posRef, 0);

    } /*
     catch (runtime_error e) {
     std::cout << e.what() << "***** something wrong!" << std::endl;
     }
     catch (exception err) {
     std::cout << err.what() << "***** something wrong!" << std::endl;
     }*/
    catch (...)
    {
        //std::cerr << "Java native interface call c++ isPosBlock() method  error" << std::endl;
    }

    return posRef == 0;
}

void ClearIntPtr(void *pBuffer)
{
    if (NULL != pBuffer)
    {
        delete pBuffer;
        pBuffer = NULL;
    }
}

void FindNearestPoly(float *start, float *end, int flag, int &nearRef, float *&pt)
{
    float m_spos[3];
    dtVcopy(m_spos, start);
    dtQueryFilter m_filter;
    m_filter.setIncludeFlags((unsigned int)flag);
    m_filter.setExcludeFlags(0xffff ^ flag);
    //dtPolyRef m_startRef = 0;
    pt = new float[3];
    navMeshQuery->findNearestPoly(m_spos, m_polyPickExt, &m_filter, (dtPolyRef *)&nearRef, pt);
}

bool IsWalkable(float *start, int flag)
{
    float m_spos[3];
    dtVcopy(m_spos, start);
    dtQueryFilter m_filter;
    m_filter.setIncludeFlags((unsigned int)flag);
    m_filter.setExcludeFlags(0xffffffff ^ flag);
    dtPolyRef m_startRef = 0;
    navMeshQuery->findNearestPoly(m_spos, m_polyPickExt, &m_filter, &m_startRef, NULL);
    if (m_startRef == 0)
    {
        return false;
    }
    return navMeshQuery->isValidPolyRef(m_startRef, &m_filter);
}

float GetPolyHeight(float *point, int flag)
{
    float m_spos[3];
    dtVcopy(m_spos, point);
    dtQueryFilter m_filter;
    m_filter.setIncludeFlags((unsigned int)flag);
    m_filter.setExcludeFlags(0xffffffff ^ flag);
    dtPolyRef m_startRef = 0;
    navMeshQuery->findNearestPoly(m_spos, m_polyPickExt, &m_filter, &m_startRef, NULL);

    float height = 0.f;
    if (m_startRef == 0)
    {
        return height;
    }
    navMeshQuery->getPolyHeight(m_startRef, m_spos, &height);
    return height;
}

void SetPolyPickExtern(float x, float y, float z)
{
    m_polyPickExt[0] = x;
    m_polyPickExt[1] = y;
    m_polyPickExt[2] = z;
}
