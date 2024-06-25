#ifndef COORDINATES_H
#define COORDINATES_H

#include <vector>

double calculate_bearing(const std::vector<double>& coord1, const std::vector<double>& coord2);
double haversine_distance(const std::vector<double>& coord1, const std::vector<double>& coord2);
std::vector<double> translate_coordinates(const std::vector<double>& coord, double bearing, double distance);

#endif // COORDINATES_H
