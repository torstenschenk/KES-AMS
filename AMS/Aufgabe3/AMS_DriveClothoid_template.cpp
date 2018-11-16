/// Beuth Hochschule für Technik Berlin
/// Autonome Mobile Systeme
/// Prof. Dr.-Ing. Volker Sommer

#include "AMS_Robot.hpp"

using namespace AMS;
using namespace PlayerCc; // dtor, rtod, limit, normalize

void DriveRobot(AMS_Robot* robotp, double L1, double LK, double L2, int turndir)
{
    double sb;           				// Beschleunigungsweg (= Bremsweg)
    double tb;           				// Beschleunigungszeit
    double tc;           				// Fahrtzeit mit vmax
    double dt;           				// aktuelle Bewegungszeit
    double t0;           				// Startzeitpunkt der Bewegung
    double t0K;          				// Startzeitpunkt der Kurvenfahrt
    double tbK;          				// Zeitdauer bis max. Winkelgeschwindigkeit erreicht wird
    double w;            				// aktuelle vorzeichenrichtige Winkelgeschwindigkeit
    double vmax = robotp->get_vmax();   // Maximale Bahngeschwindigkeit
    double vacc = robotp->get_vacc();   // Bahnbeschleunigung
    double wmax = robotp->get_wmax();   // maximale Winkelgeschwindigkeit

    robotp->init_push_mode();  // data shall be read from message queue

    /********************* Fügen Sie ab hier eigenen Quellcode ein **********************/

    sb = pow(vmax,2)/(2*vacc);
    tb = vmax / vacc;
    tc = (L1+LK+L2 - 2*sb)/vmax;
    tbK = (LK/vmax);
    t0K = (L1 - sb) / vmax;
    double tmp = 0.0; // helper, no special use

    /******************** Ende des zusätzlich eingefügten Quellcodes ********************/

    // gleichmäßige Beschleunigung während tb
    robotp->wait_for_new_data(); // aktuelle Daten vom Roboter holen
    t0 = robotp->get_t(); // Startzeitpunkt
    robotp->log.info("Accelerating.");
    do {
        robotp->wait_for_new_data();
        dt = robotp->get_t() - t0; // Bewegungszeit updaten
        robotp->set_speed(vacc*dt, 0);
    }
    while( dt < tb);

    // gleichförmige Bewegung mit maximaler Bahngeschwindigkeit während tc
    // Währenddessen Befahren der Klothoiden
    robotp->set_speed(vmax, 0);
    robotp->log.info("Driving with constant speed.");
    do {
        robotp->wait_for_new_data();
        dt = robotp->get_t() - t0 - tb; // Bewegungszeit updaten
        /********************* Fügen Sie ab hier eigenen Quellcode ein **********************/

        if ( dt > t0K && dt < t0K+tbK)
        {
            tmp = (dt-t0K) * wmax / tbK;
            // cout << "tmp = " << tmp  << endl;
            robotp->set_speed(vmax,tmp*turndir);
        }
        else if(dt > t0K +tbK && dt < t0K+2*tbK)
        {
            tmp = (  tbK-(dt-t0K - tbK)) * wmax / tbK;
            // cout << "tmp = " << tmp  << endl;
            robotp->set_speed(vmax,tmp*turndir);
        }
        else
            robotp->set_speed(vmax,0);

        /******************** Ende des zusätzlich eingefügten Quellcodes ********************/
    }
    while( dt < tc);

    // Abbremsen während tb
    robotp->log.info("Decelerating.");
    do {
        robotp->wait_for_new_data();
        dt = robotp->get_t() - t0 - tb - tc; // aktuelle Bremszeit
        robotp->set_speed(vmax-vacc*dt, 0);
    }
    while( dt < tb);

    robotp->stop(); // Stoppen
}
