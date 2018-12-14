/// Beuth Hochschule für Technik Berlin
/// Autonome Mobile Systeme
/// Prof. Dr.-Ing. Volker Sommer

#include "AMS_Robot.hpp"
#include "newmat/newmatio.h" // Input/output für Newmat

using namespace AMS;
using namespace PlayerCc;
using namespace std;
namespace po = boost::program_options;

AMS_Robot robot;

#define PI 3.1415927

static double degtorad(double deg) {
    return deg*PI/180.0;
}
static double radtodeg(double rad) {
    return rad*180.0/PI;
}

int main(int argc, char **argv)
{
    double *scan;                   // Zeiger auf aktuellen Scan
    int LUT[360];                   // Look-Up-Table mit Indizes zu allen relevanten Messwerten
    double MaxRange;                // Maximale Messdistanz des Entfernungssensors in m
    double sigma_r;                 // Standardabweichung des Entfernungssensors in m
    int N=0;                        // Anzahl der relevanten Messungen im Scan
    double PhiR;                    // Normalenwinkel der Regressionsgeraden vom Roboter aus
    double dR;                      // Normalenabstand der Regressionsgeraden von der Roboterposition
    double var_rho=0;               // Varianz des Abstands der Messpunkte von der Regressionsgeraden
    int i;                          // Laufvariable, gleichzeitig Winkel in deg
    double x, y, theta;             // Roboterkoordinaten
    int Phi;                        // globaler Winkel der gefundenen Wand
    double d;                       // globaler Abstand der gefundenen Wand
    double xi[360];                      // x-Koordinate des aktuellen Messpunktes
    double yi[360];                      // y-Koordinate des aktuellen Messpunktes
    double m_x=0;                   // Mittelwert der x-Koordinaten der Messpunkte
    double m_y=0;                   // Mittelwert der y-Koordinaten der Messpunkte
    double var_x=0;                 // Varianz der x-Koordinaten der Messpunkte
    double var_y=0;                 // Varianz der y-Koordinaten der Messpunkte
    double cov_xy=0;                // Kovarianz zwischen x- und y-Kordinaten der Messpunkte

    // Roboter initialisieren
	if( !(robot.read_config(argc, argv) && robot.connect()) ) {
		robot.log.notice("Call with -h to see the available options.");
		return -1;
	}

    robot.init_push_mode();  // Daten von "robot" aus Warteschlange lesen
    robot.wait_for_new_data();

    robot.get_scan(scan, MaxRange, sigma_r); // Messwerte, max. Messdistanz und radiale Standardabweichung auslesen

    /********************* Fügen Sie ab hier eigenen Quellcode ein **********************/

    // Bestimmug der Anzahl der relevanten Messwerte und Besetzen der Look-Up-Table
    for ( i = 0; i < 360; ++i)
        LUT[i] = -1;

    for ( i = 0; i < 360; ++i) {
        if ( *(scan+i) > 0)
            LUT[N++] = i;
    }

    // Rücksprung, falls keine Messwerte vorhanden sind
    if ( N == 0 ) {
        cout << "ERR: scanner did not return scan data!" << endl;
        return 1;
    }

    // Berechnung des Normalenwinkels der Regressionsgeraden
    for ( i = 0; i < N; ++i) {
        double ri = *(scan+LUT[i]);
        xi[i] = ri * cos(degtorad(i));
        yi[i] = ri * sin(degtorad(i));
    }

    for ( i = 0; i < N; ++i) {
        m_x += xi[i];
        m_y += yi[i];
    }

    m_x /= N;
    m_y /= N;

    for ( i = 0; i < N; ++i) {
        var_x  += xi[i]*xi[i];
        var_y  += yi[i]*yi[i];
        cov_xy =+ xi[i]*yi[i];
    }
    var_x  = var_x  / N - m_x;
    var_y  = var_y  / N - m_y;
    cov_xy = cov_xy / N - m_x*m_y;

    PhiR = atan2( -2*cov_xy, var_y - var_x);

    // Berechnung des Normalenabstands der Regressionsgeraden
    dR = m_x* cos(PhiR) + m_y * sin(PhiR);

    // Korrektur bei negativem Normalenabstand
    if ( dR < 0 ) {
        if ( PhiR < 0 )
            PhiR += PI;
        else
            PhiR -= PI;
    }

    // Berechnung des mittleren quadratischen Fehlers mit Rücksprung bei zu großem Fehler
    for ( i = 0; i < N; ++i) {
        var_rho += pow( (xi[i]*cos(PhiR) + yi[i]*sin(PhiR)) - dR , 2);
    }
    var_rho /= N;

    // Schwelle: var_rho < 0.002

    // Globale Koordinaten der gefundenen linearen Kontur bestimmen


    /******************** Ende des zusätzlich eingefügten Quellcodes ********************/

    while(1); // Endlosschleife
    return 0;
}
