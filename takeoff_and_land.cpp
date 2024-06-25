#include <chrono>
#include <cstdint>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/mission/mission.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>
#include <iostream>
#include <future>
#include <memory>
#include <thread>
#include <cmath>
#include <vector>
#include "coordinates.h"

using namespace mavsdk;
using namespace std;

using chrono::seconds;
using this_thread::sleep_for;

// Home pozisyonu ayarlamak için gerekli değişkenler
float home_latitude = 47.3977419;
float home_longitude = 8.2455938;
float home_altitude = -480;
float drone2_pos[2];
float drone2_attitude[3];
float distance_treshold = 10;
float distance_diff = 0;
float drone1_homepos[2];
float bearing = 0;

// -180 +180 derece olan yaw açısını 0-360 arasında normalize eder
double normalizeAngle(double angle) {
    // 360 dereceye göre mod alarak normalize et
    angle = fmod(angle, 360.0);

    // Negatif değerleri pozitife çevir
    if (angle < 0) {
        angle += 360.0;
    }

    return angle;
}

float radian_to_degree(float radian) {
    return radian * (180.0 / M_PI);
}

// Home noktası güncelleme fonksiyonu
void update_home(MavlinkPassthrough& mavlink_passthrough, float home_latitude, float home_longitude, float home_altitude) {
    MavlinkPassthrough::CommandLong command{};
    command.target_sysid = mavlink_passthrough.get_target_sysid();
    command.target_compid = mavlink_passthrough.get_target_compid();
    command.command = MAV_CMD_DO_SET_HOME;
    command.param1 = 0; // Use specified location
    command.param2 = 0; // Unused
    command.param3 = 0; // Unused
    command.param4 = 0; // Unused
    command.param5 = home_latitude;
    command.param6 = home_longitude;
    command.param7 = home_altitude;

    mavlink_passthrough.send_command_long(command);

    cout << "Home noktası güncellendi.\n";
}

void update_waypoints(Mission& mission, vector<Mission::MissionItem>& waypoints, double distance_diff, double bearing) {
    vector<pair<double, double>> waypoints_coords;

    // Waypoint'lerin koordinatlarını al
    for (const auto& wp : waypoints) {
        waypoints_coords.emplace_back(wp.latitude_deg, wp.longitude_deg);
    }

    // Her waypoint'i bağımsız olarak güncelle
    for (size_t i = 0; i < waypoints_coords.size(); ++i) {
        auto [x, y] = waypoints_coords[i];
        vector<double> point = {x, y};

        // Waypoint'in kendi konumuna göre dönüşüm uygula
        vector<double> new_coords = translate_coordinates(point, bearing, distance_diff);

        waypoints[i].latitude_deg = new_coords[0];
        waypoints[i].longitude_deg = new_coords[1];
    }

    Mission::MissionPlan mission_plan;
    mission_plan.mission_items = waypoints;

    auto upload_result = mission.upload_mission(mission_plan);

    if (upload_result != Mission::Result::Success) {
        cerr << "Waypoint güncellenirken hata: " << upload_result << '\n';
    } else {
        cout << "Waypointler başarıyla güncellendi.\n";
    }
}

int main() {
    Mavsdk mavsdk{Mavsdk::Configuration{Mavsdk::ComponentType::GroundStation}};
    ConnectionResult connection_result = mavsdk.add_any_connection("udp://:14540"); // bağlantı objesi
    ConnectionResult connection_result2 = mavsdk.add_any_connection("tcp://127.0.0.1:5772"); // bağlantı objesi

    if (connection_result != ConnectionResult::Success) {
        cerr << "Bağlantı 1 başarısız " << connection_result << '\n';
        return 1;
    }

    if (connection_result2 != ConnectionResult::Success) {
        cerr << "Bağlantı 2 başarısız: " << connection_result2 << '\n';
        return 1;
    }

    // Wait until systems are found
    this_thread::sleep_for(chrono::seconds(2));

    auto systems = mavsdk.systems();

    if (systems.size() < 2) {
        cerr << "Yeterli sayıda sistem bulunamadı.\n";
        return 1;
    }

    shared_ptr<System> system1 = systems.at(0);
    shared_ptr<System> system2 = systems.at(1);

    if (!system1->is_connected() || !system2->is_connected()) {
        cerr << "Sistem bağlı değil\n";
        return 1;
    }
    cout << "Araçlar Hazır...\n";

    // Gerekli objeler
    Telemetry telemetry1(system1);
    Telemetry telemetry2(system2);

    Action action1(system1);
    Action action2(system2);

    Mission mission1(system1);
    Mission mission2(system2);

    MavlinkPassthrough mavlink_passthrough1(system1);
    MavlinkPassthrough mavlink_passthrough2(system2);

    // Telemetri rate ayarlama
    auto set_rate_result1 = telemetry1.set_rate_position(1.0);
    if (set_rate_result1 != Telemetry::Result::Success) {
        cerr << "Oran ayarlama başarısız: " << set_rate_result1 << '\n';
        return 1;
    }

    auto set_rate_result2 = telemetry2.set_rate_position(1.0);
    if (set_rate_result2 != Telemetry::Result::Success) {
        cerr << "Oran ayarlama başarısız: " << set_rate_result2 << '\n';
        return 1;
    }

    // Hedef gemi için posizyon bilgisi yayını
    telemetry2.subscribe_position([](Telemetry::Position position) {
        cout << "Drone 2 - Yükseklik: " << position.relative_altitude_m << " m, "
             << "Enlem: " << position.latitude_deg << " derece, "
             << "Boylam: " << position.longitude_deg << " derece" << endl;
        drone2_pos[0] = position.latitude_deg;
        drone2_pos[1] = position.longitude_deg;
    });

    // Hedef gemi için euler açıları bilgileri
    telemetry2.subscribe_attitude_euler([](Telemetry::EulerAngle euler_angle) {
        float roll_deg = radian_to_degree(euler_angle.roll_deg);
        float pitch_deg = radian_to_degree(euler_angle.pitch_deg);
        float yaw_deg = normalizeAngle(radian_to_degree(euler_angle.yaw_deg));
        drone2_attitude[0] = roll_deg;
        drone2_attitude[1] = pitch_deg;
        drone2_attitude[2] = yaw_deg;
        cout << "Drone 2 - Roll: " << drone2_attitude[0] << " derece, "
             << "Pitch: " << drone2_attitude[1] << " derece, "
             << "Yaw: " << drone2_attitude[2] << " derece" << endl;
    });

    // Drone 1 home pozisyonunu alma
    telemetry1.subscribe_home([](Telemetry::Position home_position) {
        cout << "Drone 1 Home - Enlem: " << home_position.latitude_deg
             << ", Boylam: " << home_position.longitude_deg
             << ", Yükseklik: " << home_position.relative_altitude_m << " m" << endl;
        drone1_homepos[0] = home_position.latitude_deg;
        drone1_homepos[1] = home_position.longitude_deg;
    });

    while (true) {
        sleep_for(seconds(1));

        vector<double> drone2_pos_vector = {drone2_pos[0], drone2_pos[1]};
        vector<double> drone1_homepos_vector = {drone1_homepos[0], drone1_homepos[1]};

        distance_diff = haversine_distance(drone2_pos_vector, drone1_homepos_vector); // home noktası ve hareketli platform arası mesafe
        bearing = calculate_bearing(drone2_pos_vector, drone1_homepos_vector); // bearing açısı hesabı
        cout << distance_diff * 1000 << " Meter" << endl;
        cout << bearing << " degrees" << endl;

        if (distance_diff * 1000 >= distance_treshold) {
            update_home(mavlink_passthrough1, drone2_pos[0], drone2_pos[1], home_altitude); // drone için home noktası güncellemesi

            auto mission_items_result = mission1.download_mission();
            if (mission_items_result.first == Mission::Result::Success) {
                auto mission_items = mission_items_result.second.mission_items; // MissionPlan içindeki mission_items'e eriş
                update_waypoints(mission1, mission_items, distance_diff, bearing); // Waypoint'leri güncelle
            }
        }
    }

    return 0;
}
