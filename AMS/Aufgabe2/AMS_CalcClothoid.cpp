/// Beuth Hochschule für Technik Berlin
/// Autonome Mobile Systeme
/// Prof. Dr.-Ing. Volker Sommer

#include "AMS_Robot.hpp"
#include <newmat/newmatap.h>    // Linear algebra


using namespace AMS;

AMS_Robot robot;                // Roboterobjekt

int main(int argc, char **argv)
{
    ColumnVector P0(2);         // Startpunkt der Bahnkurve
    ColumnVector P1(2);         // Scheitelpunkt des krümmungsstetigen Übergangs
    ColumnVector P2(2);         // Endpunkt der Bahnkurve
    ColumnVector V01(2);        // Vektor vom Punkt 0 zum Punkt 1
    ColumnVector V12(2);        // Vektor vom Punkt 1 zum Punkt 2
    ColumnVector Pc0(2);        // Startpunkt des krümmungsstetigen Übergangs
    ColumnVector Pc1(2);        // Endpunkt des krümmungsstetigen Übergangs
    double Rmin;                // Kurvenradius im Scheitelpunkt der Bahnkurve (=Kehrwert der max. Krümmung)
    double L01, L12;            // Längen der Vektoren V01 und V12
    double theta;               // Winkel der Tangente an die Klothoide
    double thetaL;              // Tangentenwinkel der Klothoiden am Scheitelpunkt
    double L;                   // Länge der Klothoide
    double k;                   // Krümmungsparameter für Klothoide
    double xL, yL;              // Parameter für Klothoide
    double delta;               // Abstand zwischen dem Beginn der Klothoiden und Punkt P1
    double L1, L2;              // Länge der Geradenstücke vor und hinter der Klothoiden
    double phi1, phi2;          // Richtungswinkel der Geradenstücke vor und hinter der Klothoiden
    double s, xs=0, ys=0;       // Hilfsparameter zum Zeichnen
    double px, py;              // Plotkoordinaten
    const double ds = 0.01;     // Abstand der Punkte beim Plotten der Trajektorie

    // Roboter initialisieren
	if( !(robot.read_config(argc, argv) && robot.connect()) ) {
		robot.log.notice("Call with -h to see the available options.");
		return -1;
	}

    // Eckpunkte für krümmungsstetigen Übergang vorgeben
    P0(1) =  0.0;     // Startpunkt x-Koordinate
	P0(2) =  2.0;     // Startpunkt y-Koordinate
	P1(1) =  3.0;     // Scheitelpunkt x-Koordinate
	P1(2) =  1.0;     // Scheitelpunkt y-Koordinate
	P2(1) =  2.5;     // Endpunkt x-Koordinate
	P2(2) = -3.0;     // Endpunkt y-Koordinate


    robot.init_push_mode();

    /********************* Fügen Sie ab hier eigenen Quellcode ein **********************/

	// Minimalen Krümmungsradius aus maximaler Bahn und Winkelgeschwindigkeit berechnen
	Rmin = robot.get_vmax() / robot.get_wmax();

    // Vektoren zwischen den Punkten P0 und P1 sowie P1 und P2 festlegen und deren Längen berechnen
    V01 = P1 - P0;
  //  cout << "V01 = "<< V01(0) << " , "<< V01(1) << endl;
    V12 = P2 - P1;
   // cout << "V12 = "<< V12(0) << " , "<< V12(1) << endl;
    L01 = V01.NormFrobenius();
    cout << "L01 = "  << L01 << endl;
    L12 = V12.NormFrobenius();
    cout << "L12 = "  << L12 << endl;

    // Tangentenwinkel im Scheitelpunkt mittels Skalarprodukt und inverser Cosinusfunktion berechnen
    // z-Komponente des Vektorproduktes liefert zusätzlich das Vorzeichen von thetaL
    thetaL =  acos( DotProduct(V01, V12) / (L01 * L12) ) /2;
    cout << "DotProduct(V01, V12) = " << DotProduct(V01, V12);
    cout << "thetaL = " << thetaL << endl;
    if (( V01(1)*V12(2)-V12(1)*V01(2) ) < 0)
        thetaL =  thetaL * -1;

      cout << "thetaL 2 = " << thetaL << endl;

    // Parameter für krümmungsstetigen Übergang mit Klothoide berechnen
    L = fabs(2 * thetaL * Rmin);
    cout << "L" << L << endl;
    xL = L * ( 1 - (pow(thetaL,2) / 10) + (pow(thetaL,4) / 216) );
    yL = L * ( (thetaL / 3) - (pow(thetaL,3) / 42) + (pow(thetaL,5) / 1320)  );

    cout << "xL and yL = " << xL << " , " << yL << endl;
    delta = (yL * tan(thetaL) ) + xL;
    cout << "delta = " << delta << endl;

    // Berechnung des Start- und des Endpunktes der Klothoiden
    Pc0 = P0 + (V01 / L01) * (L01 - delta);
    Pc1 = P1 + (V12 / L12) * delta;

    /******************** Ende des zusätzlich eingefügten Quellcodes ********************/

    // Abbruch, falls delta_x die Länge des Vektors V01 oder die des Vektors V12 übersteigt.
    // In diesem Fall kann der krümmungsstetige Übergang mit dem vorgegebenem Radius nicht berechnet werden
    if( delta > L01 || delta > L12 ) {
        robot.log.errorStream() << "Fehler: Gewählter minimaler Radius nicht möglich.\n";
        return -1;
        }

    // Zeichnen des ersten Geradenstücks bis zum Beginn der Klothoide
    L1 = (Pc0-P0).NormFrobenius();
    phi1 = atan2(V01(2),V01(1)); // Orientierung des Geradenstücks, d.h. des Vektors V01 berechnen
    for( s=0; s<L1; s+=ds) {
        px = P0(1) + s*cos(phi1);
        py = P0(2) + s*sin(phi1);
        robot.draw_point(px, py, 0, 255, 0);  // Punkt zeichnen
    }

    // Zeichnen der Klothoiden
    k = 0.5/(thetaL*Rmin*Rmin);    // Parameter der Klothoide (Vorzeichen von thetaL bestimmt Krümmungsrichtung)
  /*  for( s=0; s<=L*2; s+=ds ) {    // Schleife über Punkte auf Klothoide
        if( s<=L )
            theta =      // 1. Hälfte: Zunahme der Krümmung
        else
            theta = // 2. Hälfte: Abnahme der Krümmung
        xs +=
        ys += */
        /********************* Fügen Sie ab hier eigenen Quellcode ein **********************/
        // Koordinatentransformation mit Berücksichtigung des Startpunktes Pc0 und der Richtung phi1
   /*     px =
        py =

        /******************** Ende des zusätzlich eingefügten Quellcodes ********************/
 /*       robot.draw_point(px, py, 255, 0, 255);
   }*/

    // Zeichnen des zweiten Geradenstücks hinter der Klothoide
    phi2 = atan2(V12(2),V12(1));
    L2 = (P2-Pc1).NormFrobenius();
    for( s=0; s<L2; s+=ds) {
        px = Pc1(1) + s*cos(phi2);
        py = Pc1(2) + s*sin(phi2);
        robot.draw_point(px, py, 0, 255, 0);
    }
    // Ausgabe der berechneten Teillängen
	robot.log.info("Length of 1st straight line: %.2lf m.", L1);
	robot.log.info("Length of clothoide: %.2lf m.", L*2);
	robot.log.info("Length of 2nd straight line: %.2lf m.", L2);

    while(1); // Endlosschleife
}
