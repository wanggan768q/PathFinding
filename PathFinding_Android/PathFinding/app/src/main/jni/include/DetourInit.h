#pragma once
#include "DetourNavMesh.h"

#pragma pack(push, 4)

typedef void (*FuncPtr)(const char *);

struct NavMeshSetHeader
{
    int magic;
    int version;
    int numTiles;
    dtNavMeshParams params;
};

struct NavMeshTileHeader
{
    dtTileRef tileRef;
    int dataSize;
};
#pragma pack(pop)

/*
enum SamplePolyAreas
{
    SAMPLE_POLYAREA_GROUND,
    SAMPLE_POLYAREA_WATER,
    SAMPLE_POLYAREA_ROAD,
    SAMPLE_POLYAREA_DOOR,
    SAMPLE_POLYAREA_GRASS,
    SAMPLE_POLYAREA_JUMP,
};
enum SamplePolyFlags
{
    SAMPLE_POLYFLAGS_WALK		= 0x01,		// Ability to walk (ground, grass, road)
    SAMPLE_POLYFLAGS_SWIM		= 0x02,		// Ability to swim (water).
    SAMPLE_POLYFLAGS_DOOR		= 0x04,		// Ability to move through doors.
    SAMPLE_POLYFLAGS_JUMP		= 0x08,		// Ability to jump.
    SAMPLE_POLYFLAGS_DISABLED	= 0x10,		// Disabled polygon
    SAMPLE_POLYFLAGS_BLOCK1		= 0x20,
    SAMPLE_POLYFLAGS_BLOCK2		= 0x40,
    SAMPLE_POLYFLAGS_BLOCK3		= 0x80,
    SAMPLE_POLYFLAGS_BLOCK4		= 0x100,
    SAMPLE_POLYFLAGS_BLOCK5		= 0x200,
    SAMPLE_POLYFLAGS_ALL		= SAMPLE_POLYFLAGS_WALK | SAMPLE_POLYFLAGS_SWIM	| SAMPLE_POLYFLAGS_DOOR,// All abilities.
};
*/
struct dtNavPath
{
public:
    enum
    {
        MAX_POLYS = 128
    };

public:
    float spos[3];
    float epos[3];

    float straightPath[MAX_POLYS * 3];
    unsigned char straightPathFlags[MAX_POLYS];
    dtPolyRef straightPathPolys[MAX_POLYS];
    int nstraightPath;
};

extern dtNavMesh *dtLoadNavMesh(const unsigned char *path);
extern void dtSaveNavMesh(const char *path, const dtNavMesh *mesh);
extern FuncPtr Debug;