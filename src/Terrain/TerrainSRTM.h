#pragma once

#include <QtCore/QObject>
#include <QtPositioning/QGeoCoordinate>
#include "TerrainQueryAirMap.h"
#include <QString>

// used to pack structures
#define PACKED __attribute__((__packed__))

#define LATLON_TO_M_INV 89.83204953368922
#define DEG_TO_RAD      (M_PI / 180.0f)

// MAVLink sends 4x4 grids
#define TERRAIN_GRID_MAVLINK_SIZE 4

// a 2k grid_block on disk contains 8x7 of the mavlink grids.  Each
// grid block overlaps by one with its neighbour. This ensures that
// the altitude at any point can be calculated from a single grid
// block
#define TERRAIN_GRID_BLOCK_MUL_X 7
#define TERRAIN_GRID_BLOCK_MUL_Y 8

// this is the spacing between 32x28 grid blocks, in grid_spacing units
#define TERRAIN_GRID_BLOCK_SPACING_X ((TERRAIN_GRID_BLOCK_MUL_X-1)*TERRAIN_GRID_MAVLINK_SIZE)
#define TERRAIN_GRID_BLOCK_SPACING_Y ((TERRAIN_GRID_BLOCK_MUL_Y-1)*TERRAIN_GRID_MAVLINK_SIZE)

// giving a total grid size of a disk grid_block of 32x28
#define TERRAIN_GRID_BLOCK_SIZE_X (TERRAIN_GRID_MAVLINK_SIZE*TERRAIN_GRID_BLOCK_MUL_X)
#define TERRAIN_GRID_BLOCK_SIZE_Y (TERRAIN_GRID_MAVLINK_SIZE*TERRAIN_GRID_BLOCK_MUL_Y)


class TerrainSRTM : public QObject {
    Q_OBJECT
public:
    TerrainSRTM(QObject *parent = nullptr);
    ~TerrainSRTM();
    static TerrainSRTM* instance();
    bool getAltitudesForCoordinates (const QList<QGeoCoordinate>& coordinates, QList<double>& altitudes, bool& error);
private:
    QString srtm_directory;
    int grid_spacing;

    // inverse of LOCATION_SCALING_FACTOR
    static constexpr float LOCATION_SCALING_FACTOR_INV = LATLON_TO_M_INV;

    void offset_latlng(QGeoCoordinate &coordinate, float ofs_north, float ofs_east);
    float longitude_scale(int32_t lat);
    int32_t limit_lattitude(int32_t lat);
    int32_t wrap_longitude(int64_t lon);
    int east_blocks(QGeoCoordinate coordinate);

    /*
    a grid block is a structure in a local file containing height
    information. Each grid block is 2048 in size, to keep file IO to
    block oriented SD cards efficient
    */
    struct PACKED grid_block {
        // bitmap of 4x4 grids filled in from GCS (56 bits are used)
        uint64_t bitmap;

        // south west corner of block in degrees*10^7
        int32_t lat;
        int32_t lon;

        // crc of whole block, taken with crc=0
        uint16_t crc;

        // format version number
        uint16_t version;

        // grid spacing in meters
        uint16_t spacing;

        // heights in meters over a 32*28 grid
        int16_t height[TERRAIN_GRID_BLOCK_SIZE_X][TERRAIN_GRID_BLOCK_SIZE_Y];

        // indices info 32x28 grids for this degree reference
        uint16_t grid_idx_x;
        uint16_t grid_idx_y;

        // rounded latitude/longitude in degrees.
        int16_t lon_degrees;
        int8_t lat_degrees;
    };

    /*
        grid_block for disk IO, aligned on 2048 byte boundaries
    */
    union grid_io_block {
        struct grid_block block;
        uint8_t buffer[2048];
    };

    int find_block_index(QGeoCoordinate coordinate);
};

class TerrainSRTMQuery : public TerrainQueryInterface {
    Q_OBJECT

public:
    TerrainSRTMQuery(QObject* parent = nullptr);

    // Overrides from TerrainQueryInterface
    void requestCoordinateHeights(const QList<QGeoCoordinate>& coordinates) final;
    void requestPathHeights(const QGeoCoordinate& fromCoord, const QGeoCoordinate& toCoord) final;
    void requestCarpetHeights(const QGeoCoordinate& swCoord, const QGeoCoordinate& neCoord, bool statsOnly) final;
};
