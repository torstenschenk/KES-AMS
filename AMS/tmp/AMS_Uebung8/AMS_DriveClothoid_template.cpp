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
    w = 0; //Winkelgeschwindigkeit ist 0
    sb = pow(vmax,2) / (2*vacc); //Beschleunigungsweg ist der Quotient aus maximaler Bahngeschwindigkeit hoch und dem Doppeltem der Bahnbeschleunigung
    tb = vmax/vacc; //Beschleunigungszeit ist die maximale Bahngeschwindigkeit durch die  Bahnbeschleunigung
    tc =((L1-sb)+ 2*LK + (L2-sb) )/ vmax; // Erste gerade Trajektorienlänge -Beschleunigungsweg + die Länge der Krümmung mal 2 + zweite gerade Trajektorienlänge - Beschleunigungswrg duch maximale Bahngeschwindigkeit
    tbK =LK /vmax; //Zeitddauer bis maximale Winkelgeschwindigkeit erreicht wird ist der Quatient aus Länge der Krümmung durch die maximale Bahngeschwindigkeit

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
    t0K= ((L1-sb) /vmax) + tb;
    do {
        /********************* Fügen Sie ab hier eigenen Quellcode ein **********************/
        robotp->wait_for_new_data();
        dt = robotp->get_t() - t0 -tb; //aktuelle Zeit - Startzeit - Beschleunigungszeit

       // robotp->set_speed(vmax, (dt<t0K) ? 0 : (dt>t0K+2*tbK) ? 0: (dt< t0K+tbK) ? wmax * (dt - t0K) * turndir: wmax * turndir /(dt - t0K -tbK));
        if (dt >= t0K-tb && dt <= (t0K+tbK-tb)){ // kleiner als: Startzeitpunkt der Kurvenfahrt +Zeitdauer bis max. Winkelgeschwindigkeit erreicht wird - Beschleunigungszeit
            w = (wmax/tbK)*(dt -t0K +tb); //  maximale Winkelgeschwindigkeit/Zeitdauer bis max. Winkelgeschwindigkeit erreicht wird * (Zeitlänge- Startzeitpunkt der Kurvenfahrt + Beschleunigungszeit)
            robotp->set_speed(vmax,turndir*w);

        } else if(dt >= (t0K+tbK-tb) && dt <= (t0K+2*tbK-tb)){
            w = wmax-(wmax/tbK)*(dt -t0K -tbK+tb);
            robotp->set_speed(vmax,turndir*w);

        }else{
            robotp->set_speed(vmax,0);
        }


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
