/// Beuth Hochschule für Technik Berlin
/// Autonome Mobile Systeme
/// Prof. Dr.-Ing. Volker Sommer

#include "AMS_KalmanFilter.hpp"
#include <iostream>
#include "newmat/newmatio.h"

using namespace AMS;
using namespace PlayerCc;
using namespace std;

KalmanFilter::KalmanFilter(AMS_Robot* robotpointer)
{
    // Dimension von P festlegen und P initialisieren
    P.ReSize(3,3);
    P = 0.0;
    // Zeiger auf Roboterobjekt als Attribut speichern
    this->robotp = robotpointer;
    b = 0.27;     // Abstand der Räder vom kinematischen Zentrum des Roboters [m]
    ks = 0.001;   // Schlupfkonstante zur Berechnung der Varianz der Roboterbewegung in [m]
    red = 0;
    green = 155;
    blue = 0;
    // Kinematik initialisieren
    D.ReSize(2,2);  // Dimension festlegen
    D << 0.5   <<  0.5
      << 0.5/b << -0.5/b;
    // Simulierten Schlupffaktor des Roboters setzen
    robotp->set_slip_const(0.001);
    // Simulierten Messfehler des Entfernungssensors setzen (erst für Übungsaufgabe 8 relevant)
    robotp->set_sigma_ranger(0.04);
    pt_count = 360;         // Anzahl der Ellipsenpunkte festlegen
    ellipse = new player_point_2d_t[pt_count+1]; // Feld zum Speichern der Fehlerellipse reservieren
}

void KalmanFilter::PlotEllipse(double xm, double ym)
{
    double alpha;              // Parameter zum Zeichnen der Ellipse
    SymmetricMatrix P1(2);     // Kopie von P als symmetrische Matrix für Eigenwertberechnung
    Matrix T(2,2);             // Matrix mit den othogonalen Eigenvektoren von P
    DiagonalMatrix L(2);       // Diagonalmatrix mit den Eigenwerten von P
    ColumnVector xys(2);       // Vektor mit jeweils aktuellem Ellipsenpunkt (xs,ys) in Hauptachsenform (lokale Koordinaten)
    ColumnVector xy(2);        // Vektor mit jeweils aktuellem Ellipsenpunkt (x,y) in globalen Koordinaten

    /********************* Fügen Sie ab hier eigenen Quellcode ein **********************/

    // Schleife zur Berechnung der Ellipse in Parameterform und zum Speichern im Array "ellipse"

    /******
    Folie 6 letzte Seite
    *******/

    P1 << P(1, 1) << P(1, 2) << P(2, 2);

    cout << "P: " << endl;
    cout << P << endl;
    cout << "P1: " << endl;
    cout << P1 << endl;

    EigenValues(P1, L, T);

    /****
    Folie 6 Seite 17
    *****/

    double sigma_x = sqrt(L(1));
    double sigma_y = sqrt(L(2));
    cout << "L:" << endl;
    cout << L << endl;
    ColumnVector Exy(2);
    Exy << xm << ym;

    for (alpha = 0; alpha < pt_count + 1; alpha++) {
        cout << "alpha: " << alpha << endl;
        xys << sigma_x * cos(alpha * M_PI / 180)
            << sigma_y * sin(alpha * M_PI / 180);
        cout << "T: " << T << endl;
        cout << "xys: " << xys << endl;
        cout << "Exy: " << Exy << endl;


        xy = T * xys + Exy;

        ellipse[(int)alpha] = {xy(1), xy(2)};
    }



    /******************** Ende des zusätzlich eingefügten Quellcodes ********************/

    robotp->graphmap->Color(red,green,blue,0);  // Farben im Roboterobjekt setzen
	robotp->graphmap->DrawPolyline(ellipse, pt_count+1); // Ellipse zeichnen
}

void KalmanFilter::PredictCov(double theta, double delta, double phi)
{
    ColumnVector u(2);          // Eingangsvektor mit delta und phi
    ColumnVector u_rl(2);       // Eingangsvektor mit Weginkrementen für rechtes und linkes Rad
    Matrix A(3,3);              // Systemmatrix des linearisierten Zustandsmodells
    Matrix B(3,2);              // Eingangsmatrix des linearisierten Zustandsmodells
    DiagonalMatrix Q_rl(2);     // Kovarianzmatrix für Wegstrecken des rechten und linken Rades
    Matrix Q(2,2);              // Kovarianzmatrix für Eingangsdaten delta und phi

    /********************* Fügen Sie ab hier eigenen Quellcode ein **********************/

    // Eingangsgrößen in Vektor speichern
    /*
    Speichern	Sie	die	übergebenen	Werte	für	δ	und	φ	im	Vektor	u	und	ermitteln	Sie	dar-
aus	mit	der	Inversen	von	D	die	Weginkremente	der	beiden	Räder	im	Vektor	u rl .	Be-
setzen	Sie	damit	die	Kovarianzmatrix	Q rl 	und	bestimmen	Sie	daraus	mit	D	die	Kova-
rianzmatrix	der	Eingangsgrößen	Q.
    */

    /******
    Folie 7 - Seite 25
    *******/

    u(1) = delta;
    u(2) = phi;

    // Rückrechnung von delta und phi auf Wegstrecken der beiden Räder
    u_rl = D.i() * u;

    // Varianzinkrement der Räder proportional zu Radwegen und abhängig von ks vorgeben
    Q_rl << u_rl(1) * ks << u_rl(2) * ks;

    // Varianzinkrement für delta und phi berechnen
    Q = D * Q_rl * D.t();

    // System- und Eingangsmatrix für aktuellen Schritt bestimmen
    /******
    Folie Seite 23
    *******/

    A << 1 << 0 << -delta * sin(theta + (phi/2))
      << 0 << 1 << delta * cos(theta + (phi/2))
      << 0 << 0 << 1;
    B << cos(theta + (phi/2)) << -delta / 2 * sin(theta + (phi/2))
      << sin(theta + (phi/2)) << delta / 2 * cos(theta + (phi/2))
      << 0 << 1;
    // Prädiktion der Kovarianzmatrix

    P = A * P * A.t() + B * Q * B.t();

    /******************** Ende des zusätzlich eingefügten Quellcodes ********************/

    // Ausgabe der Kovarianzmatrix
    cout << setw(12) << setprecision(5) << P << endl;
}

/// Die folgende Methode ist erst für Übungsaufgabe 8 relevant
/*
bool KalmanFilter::Correction(double& x, double& y, double& theta)
{
    double PhiR;                // Normalenwinkel einer erkannten Wand relativ zum Roboter
    double dR;                  // Normalenabstand einer erkannten Wand vom Roboter
    int Phi;                    // globale Ausrichtung der aktuell erfassten Geraden
    double d;                   // globaler Abstand der aktuellen Geraden
    SymmetricMatrix R(2);       // Kovarianzmatrix der aktuellen Messung
    Matrix S(2,2);              // Kovarianzmatrix der Innovation
    Matrix H(2,3);              // Messmatrix
    ColumnVector y_(2);         // Messvektor mit x_phi und theta
    Matrix K(3,2);              // Kalman-Gain Matrix
    IdentityMatrix E(3);        // Einheitsmatrix
    ColumnVector x_(3);         // Vektor für Roboterzustand

    // Auswertung des Scans und Rücksprung, falls keine Wand detektiert wurde
    if( !robotp->detect_wall(PhiR, dR, R) ) {
        printf("Keine Wand gefunden\n");
        return false;
    }

    //********************* Fügen Sie ab hier eigenen Quellcode ein **********************

    // Globale Koordinaten der gefundenen Kontur bestimmen für Suche in Karte
    Phi = ;
    d = ;

    // Messmatrix abhängig von der Ausrichtung der Kontur festlegen

    // Kovarianzmatrix der Innovation bestimmen
    S = ;

    // Suchen nach der aktuell gefundenen Geraden in der Karte durch Aufruf der Methode search() aus WallMap

    // Bestimmung der Messdaten für den Korrekturschritt
    y_(1) = ;
    y_(2) = ;

    // Alternative Bestimmung der Messdaten, falls gemessene Kontur zwischen Roboter und Koordinatenursprung liegt
    // Dies kann aufgrund einer zu großer Abweichung (> 0.5 rad) zwischen gemessenem und prädiziertem theta erkannt

    // Kalman Gain für aktuellen Schritt berechnen
    K = ;
    // Zustand mit Messung korrigieren; vorher Roboterzustand in Vektor x_ speichern
    x_ = ;
    // Kovarianzmatrix korrigieren
    P =;

    //******************** Ende des zusätzlich eingefügten Quellcodes ********************

    return true;
}
*/
