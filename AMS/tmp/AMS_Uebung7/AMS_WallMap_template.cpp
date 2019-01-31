/// Beuth Hochschule für Technik Berlin
/// Autonome Mobile Systeme
/// Prof. Dr.-Ing. Volker Sommer

#include "AMS_Robot.hpp"
#include "AMS_WallMap.hpp"

using namespace PlayerCc;
using namespace std;

// Im Konstruktor: Hinderniskonturen einlesen und in Vektor-Attribut speichern
WallMap::WallMap() {
    ifstream infile;
    wall walli;
    infile.open("AMS_Walls.txt");
    while( infile >> walli.phi >> walli.dist >> walli.Q11 >> walli.Q22 >> walli.Q12 ) {
//        printf("%3d %3.0lf %1.3lf %le %le %le\n", Walls.size(), walli.phi, walli.dist, walli.Q11, walli.Q22, walli.Q12);
        Walls.push_back(walli);
    }
    infile.close();
}
void WallMap::out() {
    uint i;
    for( i=0; i<Walls.size(); i++ )
        printf("%3d: phi=%4.0lf  dist=%5.2lf\n", i+1, Walls.at(i).phi, Walls.at(i).dist);
}
// In search(): Suchen nach gespeicherten Kontur mit minimalem Abstand zu übergebener Geraden
// rsq_max enthält hierbei den maximal erlaubten quadratischen Abstand zwischen der aktuellen Geraden mit Kartenkontur
bool WallMap::search(int& phi, double& dist, double rsq_max, Matrix S) {
    uint32_t i;                     // Schleifenindex
    ColumnVector nue(2);             // enthält die jeweils aktuelle Innovation
    Matrix S_inv(2,2);               // Inverse der Kovarianzmatrix der Innovation für Mahalanobis-Metrik
    ColumnVector rsq(1);             // aktueller Abstandswert der Innovation
    double rsq_temp;                // Geringster temporärer Abstand zwischen Gerade aus Karte und aktueller Messung
    int i_temp = -1;                 // Index der Geraden mit temporär geringstem Abstand von aktueller Messung

    /********************* Fügen Sie ab hier eigenen Quellcode ein **********************/

    // Invertieren der Kovarianzmatrix

    // Initialisiseren von rsq_temp mit rsq_max

    // Schleife über alle gespeicherten Konturen
    //   Darin jeweils Berechnung der aktuellen Innovation nue und des Abstands rsq mittels Mahalanobis-Metrik
    //   und speichern des geringsten Abstandes sowie des zugehörigen Indizes in rsq_temp und i_temp

    // Rücksprung mit false, falls keine Übereinstimmung mit Kartenkontur gefunden wurde

    // Setzen der Rückgabeparameter phi und dist und Rücksprung mit true

    /******************** Ende des zusätzlich eingefügten Quellcodes ********************/
}
