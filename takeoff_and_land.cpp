#include <chrono>
#include <cstdint>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/mission_raw/mission_raw.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>
#include <iostream>
#include <future>
#include <memory>
#include <thread>
#include <cmath>
#include <vector>
#include <iomanip> // For std::setprecision
#include "coordinates.h"

using namespace mavsdk;
using namespace std;

using chrono::seconds;
using this_thread::sleep_for;

// Home pozisyonu ayarlamak için gerekli değişkenler
double home_latitude = 47.3977419;
double home_longitude = 8.2455938;
float home_altitude = -480;
double drone2_pos[2];
float drone2_attitude[3];
float distance_treshold = 10;
float distance_diff = 0;
double drone1_homepos[2];
float bearing = 0;

// -180 +180 derece olan yaw açısını 0-360 arasında normalize eder
double normalizeAngle(double angle) {
    angle = fmod(angle, 360.0);
    if (angle < 0) {
        angle += 360.0;
    }
    return angle;
}

float radian_to_degree(float radian) {
    return radian * (180.0 / M_PI);
}

// Home noktası güncelleme fonksiyonu
void update_home(MavlinkPassthrough& mavlink_passthrough, double home_latitude, double home_longitude, float home_altitude) {
    MavlinkPassthrough::CommandLong command{};
    command.target_sysid = mavlink_passthrough.get_target_sysid();
    command.target_compid = mavlink_passthrough.get_target_compid();
    command.command = MAV_CMD_DO_SET_HOME;
    command.param1 = 0; // Use specified location
    command.param2 = 0; // Unused
    command.param3 = 0; // Unused
    command.param4 = 0; // Unused
    command.param5 = home_latitude * 1e7; // converting to int32 required by MAVLink
    command.param6 = home_longitude * 1e7; // converting to int32 required by MAVLink
    command.param7 = home_altitude;

    mavlink_passthrough.send_command_long(command);

    cout << "Home noktası güncellendi: " << fixed << setprecision(8) << home_latitude << ", " << home_longitude << endl;
}

void update_waypoints(MissionRaw& mission_raw, vector<MissionRaw::MissionItem>& waypoints, double distance_diff, double bearing) {
    vector<pair<double, double>> waypoints_coords;

    // Waypoint'lerin koordinatlarını al ve int'ten double'a çevir
    for (const auto& wp : waypoints) {
        waypoints_coords.emplace_back(wp.x / 1e7, wp.y / 1e7); // converting from int to double
    }

    // Her waypoint'i bağımsız olarak güncelle
    for (size_t i = 0; i < waypoints_coords.size(); ++i) {
        auto [x, y] = waypoints_coords[i];
        vector<double> point = {x, y};

        // Waypoint'in kendi konumuna göre dönüşüm uygula
        vector<double> new_coords = translate_coordinates(point, bearing, distance_diff);

        waypoints[i].x = static_cast<int32_t>(new_coords[0] * 1e7); // converting back to int32
        waypoints[i].y = static_cast<int32_t>(new_coords[1] * 1e7); // converting back to int32

        // Debug: Yeni koordinatları yazdır
        cout << "Güncellenmiş waypoint " << i << ": " << fixed << setprecision(8) << new_coords[0] << ", " << new_coords[1] << endl;
    }

    auto upload_result = mission_raw.upload_mission(waypoints);

    if (upload_result != MissionRaw::Result::Success) {
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

    // Sistemler bulunana kadar bekle
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

    MissionRaw mission_raw1(system1);
    MissionRaw mission_raw2(system2);

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

    // Hedef gemi için pozisyon bilgisi yayını
    telemetry2.subscribe_position([](Telemetry::Position position) {
        cout << "Drone 2 - Yükseklik: " << position.relative_altitude_m << " m, "
             << "Enlem: " << fixed << setprecision(8) << position.latitude_deg << " derece, "
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
        cout << "Drone 2 - Roll: " << fixed << setprecision(2) << drone2_attitude[0] << " derece, "
             << "Pitch: " << drone2_attitude[1] << " derece, "
             << "Yaw: " << drone2_attitude[2] << " derece" << endl;
    });

    // Drone 1 home pozisyonunu alma
    telemetry1.subscribe_home([](Telemetry::Position home_position) {
        cout << "Drone 1 Home - Enlem: " << fixed << setprecision(8) << home_position.latitude_deg
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

            auto mission_items_result = mission_raw1.download_mission();
            if (mission_items_result.first == MissionRaw::Result::Success) {
                auto mission_items = mission_items_result.second; // MissionPlan içindeki mission_items'e eriş
                update_waypoints(mission_raw1, mission_items, distance_diff, bearing); // Waypoint'leri güncelle
            }
        }
    }

    return 0;
}
