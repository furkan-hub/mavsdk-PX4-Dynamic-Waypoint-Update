//
// AISOFT MAVSDK ship operate mavsdk module
// 2024
// Furkan Doğan
//

#include <chrono>
#include <cstdint>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <iostream>
#include <future>
#include <memory>
#include <thread>
#include "coordinates.h"

using namespace mavsdk;
using namespace std;

using chrono::seconds;
using this_thread::sleep_for;

// Mevcut pozisyonu alir ve GPS koordinatlarini döndurur.
vector<double> get_gps_position(shared_ptr<Telemetry> telemetry) {
    auto position = telemetry->position();
    return {position.latitude_deg, position.longitude_deg, position.absolute_altitude_m};
}

int main()
{
    // MAVSDK yapilandirmasi ile yeni bir MAVSDK nesnesi olusturuluyor.
    Mavsdk mavsdk{Mavsdk::Configuration{Mavsdk::ComponentType::GroundStation}};
    // Baglantiyi baslatma (UDP uzerinden)
    ConnectionResult connection_result = mavsdk.add_any_connection("udp://:14540");

    // Baglanti kontrolu
    if (connection_result != ConnectionResult::Success)
    {
        cerr << "Baglanti basarisiz: " << connection_result << '\n';
        return 1;
    }

    // İlk autopilot sistemi aliniyor.
    auto system = mavsdk.first_autopilot(3.0);
    if (!system)
    {
        cerr << "Sistem beklenirken zaman asimina ugradi\n";
        return 1;
    }
    cout << "Arac Hazir...\n";
    
    // Eklentileri yaratma
    auto telemetry = Telemetry{system.value()};
    auto action = Action{system.value()};

    // Drone'un yukseklik bilgisini saniyede bir kez alma.
    const auto set_rate_result = telemetry.set_rate_position(1.0);
    if (set_rate_result != Telemetry::Result::Success)
    {
        cerr << "Oran ayarlama basarisiz: " << set_rate_result << '\n';
        return 1;
    }

    // Pozisyon bilgisi aboneligi.
    telemetry.subscribe_position([](Telemetry::Position position) {
        cout << "Yukseklik: " << position.relative_altitude_m << " m, "
             << "Enlem: " << position.latitude_deg << " derece, "
             << "Boylam: " << position.longitude_deg << " derece" << endl;
             
    });

    // Surekli çalisan döngu
    while (true) {
        // Diger görevleri isleyin veya mesgul dönguyu önlemek için uyuyun
        this_thread::sleep_for(chrono::seconds(1));
    }

    return 0;
}
