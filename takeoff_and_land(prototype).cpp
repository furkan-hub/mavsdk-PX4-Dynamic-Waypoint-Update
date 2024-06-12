#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/mission_raw/mission_raw.h>
#include <iostream>
#include <thread>
#include <chrono>
#include "coordinates.h"

using namespace mavsdk;
using namespace std::this_thread;
using namespace std::chrono;

bool debug = false;

std::shared_ptr<System> connect_to_pixhawk(const std::string& connection_string) {
    auto mavsdk = std::make_shared<Mavsdk>();
    auto connection_result = mavsdk->add_any_connection(connection_string);

    if (connection_result != ConnectionResult::Success) {
        std::cerr << "Connection failed: " << connection_result_str(connection_result) << std::endl;
        return nullptr;
    }

    std::shared_ptr<System> system;
    bool connected = false;
    mavsdk->subscribe_on_new_system([&mavsdk, &system, &connected]() {
        const auto systems = mavsdk->systems();
        if (!systems.empty()) {
            system = systems[0];
            connected = true;
        }
    });

    std::cout << "Waiting for drone to connect..." << std::endl;
    for (int i = 0; i < 10; ++i) {
        if (connected) {
            break;
        }
        sleep_for(seconds(1));
    }

    if (!connected) {
        std::cerr << "Connection timeout" << std::endl;
        return nullptr;
    }

    std::cout << "Connected to " << connection_string << std::endl;
    return system;
}

std::vector<double> get_gps_position(std::shared_ptr<Telemetry> telemetry) {
    auto position = telemetry->position();
    return {position.latitude_deg, position.longitude_deg, position.absolute_altitude_m};
}

std::pair<double, double> set_home_position(std::shared_ptr<Telemetry> telemetry, double lat, double lon) {
    auto home = telemetry->home();
    home.latitude_deg = lat;
    home.longitude_deg = lon;
    return {lat, lon};
}

void clear_mission(std::shared_ptr<MissionRaw> mission_raw) {
    mission_raw->clear_mission();
    std::cout << "Mission cleared" << std::endl;
}

std::vector<MissionRaw::MissionItem> get_current_mission(std::shared_ptr<MissionRaw> mission_raw) {
    auto mission_pair = mission_raw->download_mission();
    auto mission_items = mission_pair.second;
    std::cout << "Mission count received: " << mission_items.size() << std::endl;
    return mission_items;
}

std::pair<double, double> get_home_position(std::shared_ptr<Telemetry> telemetry) {
    auto home = telemetry->home();
    return {home.latitude_deg, home.longitude_deg};
}

void upload_mission(std::shared_ptr<MissionRaw> mission_raw, const std::vector<MissionRaw::MissionItem>& mission_items) {
    mission_raw->upload_mission(mission_items);
    std::cout << "Mission uploaded successfully" << std::endl;
}

void update_waypoints(std::shared_ptr<Telemetry> telemetry, std::shared_ptr<MissionRaw> mission_raw, const std::vector<double>& new_point, double threshold) {
    auto waypoints = get_current_mission(mission_raw);
    auto home = get_home_position(telemetry);
    std::vector<std::pair<double, double>> waypoints_coords;
    for (const auto& wp : waypoints) {
        waypoints_coords.push_back({wp.x / 1e7, wp.y / 1e7});
    }

    double bearing = calculate_bearing(new_point, {home.first, home.second});
    double distance = haversine_distance({home.first, home.second}, new_point);
    double distance_meters = distance * 1000;

    std::cout << distance << ", " << bearing << std::endl;

    if (distance_meters >= threshold) {
        for (size_t i = 0; i < waypoints_coords.size(); ++i) {
            auto point = waypoints_coords[i];
            auto new_coords = translate_coordinates({point.first, point.second}, bearing, distance);
            waypoints[i].x = static_cast<int32_t>(new_coords[0] * 1e7);
            waypoints[i].y = static_cast<int32_t>(new_coords[1] * 1e7);
            std::cout << "Updated waypoint " << i << " to new coordinates: " << waypoints[i].x << ", " << waypoints[i].y << std::endl;
        }
    }

    sleep_for(seconds(1));
    upload_mission(mission_raw, waypoints);
}

int main() {
    auto mavsdk1 = std::make_shared<Mavsdk>();
    auto mavsdk2 = std::make_shared<Mavsdk>();

    std::string connection_string = "udp://:14540";  // Değiştirmeniz gerekebilir
    std::string connection_string2 = "tcp://:5772";

    auto drone = connect_to_pixhawk(connection_string);
    auto ship = connect_to_pixhawk(connection_string2);

    if (!drone || !ship) {
        return 1;
    }

    auto telemetry_drone = std::make_shared<Telemetry>(drone);
    auto telemetry_ship = std::make_shared<Telemetry>(ship);
    auto mission_raw = std::make_shared<MissionRaw>(drone);

    while (true) {
        auto ship_coords = get_gps_position(telemetry_ship);
        std::vector<double> new_point = {ship_coords[0], ship_coords[1]};
        double threshold = 25;  // metre

        update_waypoints(telemetry_drone, mission_raw, new_point, threshold);
        set_home_position(telemetry_drone, ship_coords[0], ship_coords[1]);
        sleep_for(seconds(5));  // 10 saniye bekle, ardından waypoint güncellemesini tekrar dene
    }

    return 0;
}



    // vector<double> coord1 = {37.7749, -122.4194}; // San Francisco, CA
    // vector<double> coord2 = {34.0522, -118.2437}; // Los Angeles, CA

    // double bearing = calculate_bearing(coord1, coord2);
    // cout << "Bearing: " << bearing << " degrees" << endl;

    // double distance = haversine_distance(coord1, coord2);
    // cout << "Distance: " << distance << " km" << endl;

    // double bearing_to_translate = 90.0;   // Örnek olarak 90 derece
    // double distance_to_translate = 100.0; // Örnek olarak 100 km

    // vector<double> new_coord = translate_coordinates(coord1, bearing_to_translate, distance_to_translate);
    // cout << "New Coordinates: [" << new_coord[0] << ", " << new_coord[1] << "]" << endl;
