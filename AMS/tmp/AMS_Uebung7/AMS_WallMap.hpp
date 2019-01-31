/// Beuth Hochschule für Technik Berlin
/// Autonome Mobile Systeme
/// Prof. Dr.-Ing. Volker Sommer

// Hilfsklasse zum Laden der Konturen in "AMS_Walls.txt" in einen Vektor und zum Suchen darin nach aktueller Kontur
#include <iostream>
#include <fstream>
#include <vector>
#include "newmat/newmatio.h"
using namespace std;

struct wall {
    double phi;
    double dist;
    double Q11;
    double Q22;
    double Q12;
};

class WallMap {
private:
    vector<wall> Walls;
public:
    WallMap();
    void out();
    bool search(int& phi, double& dist, double rsq_max, Matrix S);
};
