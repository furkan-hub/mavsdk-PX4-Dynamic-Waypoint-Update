#ifndef COORDINATES_H
#define COORDINATES_H

#include <vector>

double calculate_bearing(std::vector<double> coord1, std::vector<double> coord2);
double haversine_distance(std::vector<double> coord1, std::vector<double> coord2);
double translate_coordinates(std::vector<double> coord, double bearing, double distance);

#endif // COORDINATES_H
