/// Beuth Hochschule für Technik Berlin
/// Autonome Mobile Systeme
/// Prof. Dr.-Ing. Volker Sommer

#include "AMS_Robot.hpp"
#include "newmat/newmatio.h" // Input/output für Newmat
#include <stdexcept>

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
    int N=0;                     // Anzahl der relevanten Messungen im Scan
    double PhiR;                    // Normalenwinkel der Regressionsgeraden vom Roboter aus
    double dR;                      // Normalenabstand der Regressionsgeraden von der Roboterposition
    double var_rho=0;               // Varianz des Abstands der Messpunkte von der Regressionsgeraden
    int i;                          // Laufvariable, gleichzeitig Winkel in deg
    double x, y, theta;             // Roboterkoordinaten
    double Phi;                        // globaler Winkel der gefundenen Wand
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
    for(i=0; i< 360; i++){
        if((scan[i])>0){
            LUT[N++] = i;
        }
    }
     // Rücksprung, falls keine Messwerte vorhanden sind
    if(N==0){
        throw invalid_argument("N is zero");
    }

    // Berechnung des Normalenwinkels der Regressionsgeraden

    for(i=0; i<N; i++){
        xi = (scan [LUT[i]]) * cos(LUT[i] * (M_PI/180.));
        yi = (scan [LUT[i]]) * sin(LUT[i] * (M_PI/180.));
        m_x += xi;
        m_y += yi;
        var_x += pow(xi,2);
        var_y += pow(yi,2);
        cov_xy += xi * yi;
    }
    m_x /= N;
    m_y /= N;
    var_x = (1.0/N) * var_x - pow(m_x,2);
    var_y = (1.0/N) * var_y - pow(m_y,2);
    cov_xy = (cov_xy/N) - m_x * m_y;

    // Berechnung des Normalenabstands der Regressionsgeraden
    PhiR = 0.5 * atan2((-2) * cov_xy , var_y - var_x);
    dR = m_x * cos(PhiR) + m_y * sin(PhiR);


    // Korrektur bei negativem Normalenabstand
    if(dR < 0){
        dR = fabs(dR);
        PhiR = PhiR + M_PI;
    }
    // Berechnung des mittleren quadratischen Fehlers mit Rücksprung bei zu großem Fehler
    for (i = 0; i<N; i++){
        xi = (scan [LUT[i]]) * cos(LUT[i] * (M_PI/180));
        yi = (scan [LUT[i]]) * sin(LUT[i] * (M_PI/180));
        var_rho += pow((xi * cos(PhiR) + yi * sin(PhiR) - dR),2);
    }
    var_rho /= N;

    /*if(sqrt(var_rho) > 2*sigma_r){
    robot.log.info("var_rho = %lf", var_rho);
        throw invalid_argument("var_rho too big");
    }*/


    // Globale Koordinaten der gefundenen linearen Kontur bestimmen
    theta = robot.get_theta();
    x = robot.get_x();
    y = robot.get_y();

    robot.log.notice("theta ist: %lf", theta);
    robot.log.notice("x ist: %lf", x);
    robot.log.notice("y ist: %lf", y);

    Phi = (theta + PhiR);
    //Phi = (theta + PhiR) * 180./M_PI;
    d = dR + (x * cos(theta + PhiR) ) + (y * sin(theta + PhiR));

    if(d < 0){
        Phi = Phi + M_PI;
        d = fabs(d);
    }
    robot.log.notice("robot.x is: %lf . robot.y is: %lf ", robot.get_x() , robot.get_y());
    robot.log.notice("PhiR ist: %lf . dR ist: %lf ",PhiR * 180./M_PI,dR);
    robot.log.notice("Phi ist: %lf . d ist: %lf ",Phi* 180./M_PI,d);
    /******************** Ende des zusätzlich eingefügten Quellcodes ********************/

    while(1); // Endlosschleife
    return 0;
}
