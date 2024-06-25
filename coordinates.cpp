#include "coordinates.h"
#include <cmath>

const double EARTH_RADIUS = 6371.0;

double calculate_bearing(std::vector<double> coord1, std::vector<double> coord2) {
    double lat1 = coord1[0] * M_PI / 180.0;
    double lon1 = coord1[1] * M_PI / 180.0;
    double lat2 = coord2[0] * M_PI / 180.0;
    double lon2 = coord2[1] * M_PI / 180.0;

    double delta_lon = lon2 - lon1;

    double x = std::cos(lat2) * std::sin(delta_lon);
    double y = std::cos(lat1) * std::sin(lat2) - std::sin(lat1) * std::cos(lat2) * std::cos(delta_lon);

    double bearing = std::atan2(x, y);
    bearing = bearing * 180.0 / M_PI;
    bearing = std::fmod((bearing + 360.0), 360.0);
    bearing = std::fmod((bearing + 180.0), 360.0);

    return bearing;
}

double haversine_distance(std::vector<double> coord1, std::vector<double> coord2) {
    double lat1 = coord1[0] * M_PI / 180.0;
    double lon1 = coord1[1] * M_PI / 180.0;
    double lat2 = coord2[0] * M_PI / 180.0;
    double lon2 = coord2[1] * M_PI / 180.0;

    double dlat = lat2 - lat1;
    double dlon = lon2 - lon1;

    double a = std::sin(dlat / 2) * std::sin(dlat / 2) +
               std::cos(lat1) * std::cos(lat2) * std::sin(dlon / 2) * std::sin(dlon / 2);
    double c = 2 * std::atan2(std::sqrt(a), std::sqrt(1 - a));

    double distance = EARTH_RADIUS * c;
    return distance;
}

double translate_coordinates(std::vector<double> coord, double bearing, double distance) {
    double lat = coord[0] * M_PI / 180.0;
    double lon = coord[1] * M_PI / 180.0;
    double bearing_rad = bearing * M_PI / 180.0;

    double new_lat = std::asin(std::sin(lat) * std::cos(distance / EARTH_RADIUS) +
                               std::cos(lat) * std::sin(distance / EARTH_RADIUS) * std::cos(bearing_rad));
    double new_lon = lon + std::atan2(std::sin(bearing_rad) * std::sin(distance / EARTH_RADIUS) * std::cos(lat),
                                      std::cos(distance / EARTH_RADIUS) - std::sin(lat) * std::sin(new_lat));
    double results[2];
    
    results[0] = new_lat * 180.0 / M_PI;
    results[1] = new_lon * 180.0 / M_PI;


    return results[2];
}