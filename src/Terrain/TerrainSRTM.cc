#include "TerrainSRTM.h"
#include "TerrainTileManager.h"
#include <QFile>
#include <QDataStream>
#include <QString>
#include "QGCLoggingCategory.h"
#include <QtNumeric>

QGC_LOGGING_CATEGORY(TerrainSRTMQueryLog, "qgc.terrainsrtm.terrainquery")
Q_GLOBAL_STATIC(TerrainSRTM, s_terrainSRTM)

TerrainSRTM* TerrainSRTM::instance(void)
{
    return s_terrainSRTM();
}

TerrainSRTM::TerrainSRTM(QObject *parent)
    : QObject(parent)
{
    srtm_directory = "/Users/oleksii/Downloads/TerrainSRTM1";
    grid_spacing = 30;
}

TerrainSRTM::~TerrainSRTM()
{

}

float TerrainSRTM::longitude_scale(int32_t lat)
{
    float scale = cosf(lat * (1.0e-7 * DEG_TO_RAD));
    return scale > 0.01 ? scale : 0.01;
}

/*
  limit lattitude to -90e7 to 90e7
 */
int32_t TerrainSRTM::limit_lattitude(int32_t lat)
{
    if (lat > 900000000L) {
        lat = 1800000000LL - lat;
    } else if (lat < -900000000L) {
        lat = -(1800000000LL + lat);
    }
    return lat;
}

/*
  wrap longitude for -180e7 to 180e7
 */
int32_t TerrainSRTM::wrap_longitude(int64_t lon)
{
    if (lon > 1800000000L) {
        lon = int32_t(lon-3600000000LL);
    } else if (lon < -1800000000L) {
        lon = int32_t(lon+3600000000LL);
    }
    return int32_t(lon);
}

// extrapolate latitude/longitude given distances (in meters) north and east
void TerrainSRTM::offset_latlng(QGeoCoordinate &coordinate, float ofs_north, float ofs_east)
{
    int32_t lat = coordinate.latitude()*10*1000*1000L;
    int32_t lng = coordinate.longitude()*10*1000*1000L;
    const int32_t dlat = ofs_north * LOCATION_SCALING_FACTOR_INV;
    const int64_t dlng = (ofs_east * LOCATION_SCALING_FACTOR_INV) / longitude_scale(lat+dlat/2);
    lat += dlat;
    coordinate.setLatitude((float)limit_lattitude(lat)/(10*1000*1000L));
    coordinate.setLongitude((float)wrap_longitude(dlng+lng)/(10*1000*1000L));
}

int TerrainSRTM::east_blocks(QGeoCoordinate coordinate)
{
    coordinate.setLatitude((int)coordinate.latitude());
    coordinate.setLongitude((int)coordinate.longitude());
    QGeoCoordinate coordinate2(coordinate);
    coordinate2.setLongitude(coordinate2.longitude() + 1);

    offset_latlng(coordinate2,0,2*grid_spacing*TERRAIN_GRID_BLOCK_SIZE_Y);
    return coordinate.distanceTo(coordinate2) / (grid_spacing*TERRAIN_GRID_BLOCK_SPACING_Y);
}

int TerrainSRTM::find_block_index(QGeoCoordinate coordinate)
{
    QGeoCoordinate coordinate2(coordinate);
    coordinate.setLatitude((int)coordinate.latitude());
    coordinate.setLongitude((int)coordinate.longitude());
    coordinate2.setLatitude(coordinate.latitude());


    return coordinate.distanceTo(coordinate2) / (grid_spacing*TERRAIN_GRID_BLOCK_SPACING_Y);
}

/// Either returns altitudes from cache or queues database request
///     @param[out] error true: altitude not returned due to error, false: altitudes returned
/// @return true: altitude returned (check error as well), false: database query queued (altitudes not returned)
bool TerrainSRTM::getAltitudesForCoordinates(const QList<QGeoCoordinate>& coordinates, QList<double>& altitudes, bool& error)
{
    for (const QGeoCoordinate& coordinate: coordinates) {
        QGeoCoordinate ref(coordinate);
        ref.setLatitude((int)qAbs(coordinate.latitude()));
        ref.setLongitude((int)qAbs(coordinate.longitude()));
        QString path = QString("%1/%2%3%4%5.DAT")
            .arg(srtm_directory)
            .arg(coordinate.latitude() <0?'S':'N')
            .arg((int)ref.latitude(), 2, 10, QChar('0'))
            .arg(coordinate.longitude() <0?'W':'E')
            .arg((int)ref.longitude(), 3, 10, QChar('0'));

        QFile file(path);
        if (!file.open(QIODevice::ReadOnly)) {
            error = true;
            return false;
        }

        QGeoCoordinate coordinate2;
        coordinate2.setLatitude(coordinate.latitude());
        coordinate2.setLongitude(ref.longitude());
        uint idx_x = ref.distanceTo(coordinate2) / grid_spacing;
        coordinate2.setLatitude(ref.latitude());
        coordinate2.setLongitude(coordinate.longitude());
        uint idx_y = ref.distanceTo(coordinate2) / grid_spacing;

        uint grid_idx_x = idx_x  / TERRAIN_GRID_BLOCK_SPACING_X;
        uint grid_idx_y = idx_y  / TERRAIN_GRID_BLOCK_SPACING_Y;

        idx_x = idx_x % TERRAIN_GRID_BLOCK_SPACING_X;
        idx_y = idx_y % TERRAIN_GRID_BLOCK_SPACING_Y;

        uint blocknum = east_blocks(coordinate) * grid_idx_x + grid_idx_y;


        QDataStream stream(&file);

        if (stream.skipRawData(blocknum * sizeof(union grid_io_block)) == -1) {
            error = true;
            return false;
        }

        grid_block block;
        if (stream.readRawData((char *)&block,sizeof(block)) == -1) {
            error = true;
            return false;
        }

        file.close();


        altitudes.push_back(block.height[idx_x][idx_y]);
    }
    error = false;
    return true;
}

TerrainSRTMQuery::TerrainSRTMQuery(QObject* parent)
    : TerrainQueryInterface(parent)
{

}

void TerrainSRTMQuery::requestCoordinateHeights(const QList<QGeoCoordinate>& coordinates)
{
    if (coordinates.length() == 0) {
        return;
    }

    QList<double> altitudes;
    bool error;
    TerrainSRTM::instance()->getAltitudesForCoordinates(coordinates,altitudes,error);

    bool success = true;
    emit coordinateHeightsReceived(success, altitudes);
}

void TerrainSRTMQuery::requestPathHeights(const QGeoCoordinate& fromCoord, const QGeoCoordinate& toCoord)
{
    QList<QGeoCoordinate> coordinates;
    double distanceBetween;
    double finalDistanceBetween;

    coordinates = TerrainTileManager::pathQueryToCoords(fromCoord, toCoord, distanceBetween, finalDistanceBetween);

    bool error;
    QList<double> altitudes;
    TerrainSRTM::instance()->getAltitudesForCoordinates(coordinates,altitudes,error);

    bool success = true;
    emit pathHeightsReceived(success, distanceBetween, finalDistanceBetween, altitudes);
}

void TerrainSRTMQuery::requestCarpetHeights(const QGeoCoordinate& swCoord, const QGeoCoordinate& neCoord, bool statsOnly)
{
    // TODO
    Q_UNUSED(swCoord);
    Q_UNUSED(neCoord);
    Q_UNUSED(statsOnly);
    qWarning() << "Carpet queries are currently not supported from SRTM data";
}