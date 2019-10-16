#include "DetourInit.h"
#include <cstdio>
#include <cstring>
#include <sstream>
static const int NAVMESHSET_MAGIC = 'M' << 24 | 'S' << 16 | 'E' << 8 | 'T'; //'MSET';
static const int NAVMESHSET_VERSION = 1;

dtNavMesh *dtLoadNavMesh(const unsigned char *ndata)
{
    NavMeshSetHeader *header = new NavMeshSetHeader();
    //data.read()
    const unsigned char *pos = ndata;
    memcpy(header, pos, sizeof(NavMeshSetHeader));
    pos += sizeof(NavMeshSetHeader);
    if (header->magic != NAVMESHSET_MAGIC)
    {
        return 0;
    }
    if (header->version != NAVMESHSET_VERSION)
    {
        return 0;
    }

    dtNavMesh *mesh = dtAllocNavMesh();
    if (!mesh)
    {
        return 0;
    }

    dtStatus status = mesh->init(&header->params);
    if (dtStatusFailed(status))
    {
        return 0;
    }
    // Read tiles.
    for (int i = 0; i < header->numTiles; ++i)
    {
        NavMeshTileHeader *tileHeader = new NavMeshTileHeader();
        memcpy(tileHeader, pos, sizeof(NavMeshTileHeader));
        pos += sizeof(NavMeshTileHeader);

        if (!tileHeader->tileRef || !tileHeader->dataSize)
            break;

        unsigned char *data = (unsigned char *)dtAlloc(tileHeader->dataSize, DT_ALLOC_PERM);
        if (!data)
        {
            break;
        }
        memset(data, 0, tileHeader->dataSize);
        memcpy(data, pos, tileHeader->dataSize);
        pos += tileHeader->dataSize;

        mesh->addTile(data, tileHeader->dataSize, DT_TILE_FREE_DATA, tileHeader->tileRef, 0);
        delete tileHeader;
    }

    delete header;
    return mesh;
}

void dtSaveNavMesh(const char *path, const dtNavMesh *mesh)
{
    if (!mesh)
        return;

    FILE *fp = fopen(path, "wb");
    if (!fp)
        return;

    // Store header.
    NavMeshSetHeader header;
    header.magic = NAVMESHSET_MAGIC;
    header.version = NAVMESHSET_VERSION;
    header.numTiles = 0;
    for (int i = 0; i < mesh->getMaxTiles(); ++i)
    {
        const dtMeshTile *tile = mesh->getTile(i);
        if (!tile || !tile->header || !tile->dataSize)
            continue;
        header.numTiles++;
    }
    memcpy(&header.params, mesh->getParams(), sizeof(dtNavMeshParams));
    fwrite(&header, sizeof(NavMeshSetHeader), 1, fp);

    // Store tiles.
    for (int i = 0; i < mesh->getMaxTiles(); ++i)
    {
        const dtMeshTile *tile = mesh->getTile(i);
        if (!tile || !tile->header || !tile->dataSize)
            continue;

        NavMeshTileHeader tileHeader;
        tileHeader.tileRef = mesh->getTileRef(tile);
        tileHeader.dataSize = tile->dataSize;
        fwrite(&tileHeader, sizeof(tileHeader), 1, fp);

        fwrite(tile->data, tile->dataSize, 1, fp);
    }
    fclose(fp);
}

FuncPtr Debug = NULL;