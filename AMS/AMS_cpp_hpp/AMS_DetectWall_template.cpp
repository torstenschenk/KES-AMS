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
    double xi;                      // x-Koordinate des aktuellen Messpunktes
    double yi;                      // y-Koordinate des aktuellen Messpunktes
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

    // Bestimmung der Anzahl der relevanten Messwerte und Besetzen der Look-Up-Table

    // Rücksprung, falls keine Messwerte vorhanden sind

    // Berechnung des Normalenwinkels der Regressionsgeraden

    // Berechnung des Normalenabstands der Regressionsgeraden

    // Korrektur bei negativem Normalenabstand

    // Berechnung des mittleren quadratischen Fehlers mit Rücksprung bei zu großem Fehler

    // Globale Koordinaten der gefundenen linearen Kontur bestimmen

    /******************** Ende des zusätzlich eingefügten Quellcodes ********************/

    while(1); // Endlosschleife
    return 0;
}
