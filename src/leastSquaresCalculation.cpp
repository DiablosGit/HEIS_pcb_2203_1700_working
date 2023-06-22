#include <Arduino.h>
#include "globals.h"
#include "BasicLinearAlgebra.h" //Bibliothek für Matrizenrechnung

float phase;       //phase angle == phase?
float i_amplitude; //current amplitude
float u_amplitude; //voltage amplitude
float i_phase;
float u_phase;
float z;           //impedance
float delta_phase; //Phasenversatz

void leastSquares() //berechnet die Amplituden und Phasen von Strom und Spannung
//Messwerte sind in globalen Variablen gespeichert und werden nicht übergeben

{ 
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //algorithmus für Strom
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  int i;
  int k;
  int j = 0;
  double tol = 1e-12;     //Toleranz für Änderung der Parameter (s) für nächste Iteration
  double sabs = 1.0;      //Startwert für s, dass Berechnung anfängt, >tol 
  int iter = 25;          // maximale Anzahl der Berechnungsdurchläufe 
  int p = 1;
  const int m = arraySizeMeasurements;
  //Startwerte für Referenzsignal für ersten Berechnungsdurchlauf
  BLA::Matrix<3, 1> a = {0.5,   //amplitude
                        0.5, //phase
                        1.5};  //offset
  BLA::Matrix<m, 3> D;       //Def. Matrix D, enthält Ableitung des Residuumsvektors nach den einzelnen Parametern
  BLA::Matrix<3, m> DT;      //Def. MAtrix DT, DT = D transponiert
  BLA::Matrix<3, 3> A;       //Def. Matrix A, A = DT*D
  BLA::Matrix<3, 3> L;       //Def. Matrix L, L dient zur Lösung der Gleichung A*x = B; es gilt: A = L transponiert * L; ist eine Dreiecksmatrix
  BLA::Matrix<3, 1> b;       //Def. Vektor b, b = DT*r
  BLA::Matrix<m, 1> r;       //Def. r, r ist der Residuumsvektor der die Differenz aus der gesuchten Funktion und den Messwerten darstellt
  BLA::Matrix<3, 1> c;       //Def. c, Hilfsvektor aus Zwischenschritt zur Lösung von A*x = B bzw. zur Berechnung von s
  BLA::Matrix<3, 1> s;       //Def. s, Inkrement zur Neuberechnung der Parameter für nächsten Berechnungsschritt
  L.Fill(0);                 //Füllen der Matrix L mit nullen, da sonst Berechnung  nicht möglich


  while ((sabs > tol) and (p < iter)) {   //Durchführen der Berechnung solange die Toleranzgrenze nicht unter- und die maximale Berechnungsschritte nicht überschritten ist 
    
    for (i = 0; i < m; i++) {  //Aufstellen von r und D
      r(i, 0) = a(0, 0) * sin(2 * PI * frequency * timeCurrentMatrix(0, i)/1.0E6 + a(1, 0)) + a(2, 0) - adcCurrentMatrix(0,i)/ 4095.0 * 3.3 / 0.02;
      D(i, 0) = sin(2 * PI * frequency * timeCurrentMatrix(0, i)/1.0E6 + a(1, 0));    //Ableitung von r nach a1
      D(i, 1) = a(0, 0) * cos(2 * PI * frequency * timeCurrentMatrix(0, i)/1.0E6 + a(1, 0));    //Ableitung von r nach a2
      D(i, 2) = 1;    //Ableitung von r nach a3
    }

    DT = ~D;    
    A = DT * D;
    b = DT * r;

    //Cholesky-Zerlegung; 1. Berechnung von L 2. Berechnung von c 3. Berechnung von s
    for (i = 0; i <= 2; i++){
      double Summe1 = 0;    //Summe1 initialisieren mit 0
      double Summe2 = 0;    //Summe2 initialisieren mit 0
      for (k = 0; k <= i - 1; k++){
          Summe1 = Summe1 + L(k, i) * L(k, i);    //Summe 1 berechnen
      }

      L(i, i) = sqrt(A(i, i) - Summe1);   //Diagonaleinträge von L berechnen 
      if (L(i, i) == 0) {
          L(i, i) = 1 * 10 ^ -2;
      }
      for (j = i + 1; j <= 2; j++) {
        for (k = 0; k <= i - 1; k++) {
          Summe2 = Summe2 + L(k, j) * L(k, i);     //Summe2 berechnen
        }
        
        L(i, j) = (1 / L(i, i)) * (A(i, j) - Summe2);     //Nebenelemente von L berechnen
      }
    }
    //c berechnen als Zwischenschritt
    c(0, 0) = (b(0, 0) / L(0, 0));      
    c(1, 0) = (b(1, 0) - L(0, 1) * c(0, 0)) / L(1, 1);
    c(2, 0) = (b(2, 0) - L(0, 2) * c(0, 0) - L(1, 2) * c(1, 0)) / L(2, 2);

    //s berechnen 
    s(2, 0) = (c(2, 0) / L(2, 2));
    s(1, 0) = (c(1, 0) - L(1, 2) * s(2, 0)) / L(1, 1);
    s(0, 0) = (c(0, 0) - L(0, 2) * s(2, 0) - L(0, 1) * s(1, 0)) / L(0, 0);

    //a neu berechnen für nächsten Durchlauf 
    a = a - s;

    //Betrag von s berechnen für Prüfung mit Toleranzgrenze
    sabs = sqrt(s(0, 0) * s(0, 0) + s(1, 0) * s(1, 0) + s(2, 0) * s(2, 0));
    p++;
    //Serial.print("Sabs:");Serial.println(sabs);Serial.print("Amplitude");Serial.print("\t");Serial.print(a(0,0),5);Serial.print("\n");Serial.print("Phase");Serial.print("\t");Serial.print(a(1,0),5);Serial.print("\n");Serial.print(a(2,0),5);
  
  } //Ende while schleife
  

  
  double Amplitude, Phase;

  //Unterscheidung: wenn Amplitude negativ ist, Betrag davon nehmen und Phase um PI verschieben 
  if (a(0, 0) < 0) {
    Amplitude = sqrt(a(0, 0) * a(0, 0));
    Phase = a(1, 0) + PI;
  }
  //Unterscheidung: wenn Amplitude positiv ist, bleiben werte 
  if (a(0, 0) > 0) {
    Amplitude = a(0, 0);
    Phase = a(1, 0);
  }

  //Verschiebung der Phase in den Bereich von -PI bis +PI
  while (Phase > PI) {
    //Serial.println("größer als PI");
    Phase = Phase - 2 * PI;
  }

  while (Phase < -PI) {
    //Serial.println("größer als PI");
    Phase = Phase + 2 * PI;
  }
  delay(200);
  Serial.println("##### Ergebnisse für Strom ###############");
  Serial.print("Amplitude: ");Serial.println(Amplitude,5);
  Serial.print("Phase: ");Serial.println(Phase,5);


  //Speichere Messwerte
  i_amplitude = Amplitude;
  i_phase = Phase;


 ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //algorithmus für Spannung
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   j = 0;
   sabs = 1.0;      //müssen für nächsten Durchlauf wieder auf Startwerte gesetzt werden
   p = 1;
  // const int m = arraySizeMeasurements;
  // //Startwerte für Referenzsignal für ersten Berechnungsdurchlauf
  a = {0.5,   //amplitude
      0.5, //phase
      1.5};  //offset
  // BLA::Matrix<m, 3> D;
  // BLA::Matrix<3, m> DT;
  // BLA::Matrix<3, 3> A;
  // BLA::Matrix<3, 3> L;
  // BLA::Matrix<3, 1> b;
  // BLA::Matrix<m, 1> r;
  // BLA::Matrix<3, 1> c;
  // BLA::Matrix<3, 1> s;
  L.Fill(0);  
  // Serial.println("Start");

  while ((sabs > tol) and (p < iter)) {
    //Berechnung mit realen Messwerten
    for (i = 0; i < m; i++) {
      r(i, 0) = a(0, 0) * sin(2 * PI * frequency * timeVoltageMatrix(0, i)/1.0E6 + a(1, 0)) + a(2, 0) - adcVoltageMatrix(0,i)/ 4095.0 * 3.3 ;
      D(i, 0) = sin(2 * PI * frequency * timeVoltageMatrix(0, i)/1.0E6 + a(1, 0));
      D(i, 1) = a(0, 0) * cos(2 * PI * frequency * timeVoltageMatrix(0, i)/1.0E6 + a(1, 0));
      D(i, 2) = 1;
    }

    DT = ~D;
    A = DT * D;
    b = DT * r;

    for (i = 0; i <= 2; i++){
      double Summe1 = 0;
      double Summe2 = 0;
      for (k = 0; k <= i - 1; k++){
          Summe1 = Summe1 + L(k, i) * L(k, i);
      }
      //Serial.print("Summe1:");Serial.println(Summe1);

      L(i, i) = sqrt(A(i, i) - Summe1);
      if (L(i, i) == 0) {
          L(i, i) = 1 * 10 ^ -2;
      }
      for (j = i + 1; j <= 2; j++) {
        for (k = 0; k <= i - 1; k++) {
          Summe2 = Summe2 + L(k, j) * L(k, i);
        }
        //Serial.print("Summe2:");Serial.println(Summe2);
        L(i, j) = (1 / L(i, i)) * (A(i, j) - Summe2);
      }
    }

    c(0, 0) = (b(0, 0) / L(0, 0));
    c(1, 0) = (b(1, 0) - L(0, 1) * c(0, 0)) / L(1, 1);
    c(2, 0) = (b(2, 0) - L(0, 2) * c(0, 0) - L(1, 2) * c(1, 0)) / L(2, 2);

    s(2, 0) = (c(2, 0) / L(2, 2));
    s(1, 0) = (c(1, 0) - L(1, 2) * s(2, 0)) / L(1, 1);
    s(0, 0) = (c(0, 0) - L(0, 2) * s(2, 0) - L(0, 1) * s(1, 0)) / L(0, 0);
    //Serial.println("c und s berechnet");
    a = a - s;

    sabs = sqrt(s(0, 0) * s(0, 0) + s(1, 0) * s(1, 0) + s(2, 0) * s(2, 0));
    p++;
    //Serial.print("Sabs:");Serial.println(sabs);Serial.print("Amplitude");Serial.print("\t");Serial.print(a(0,0),5);Serial.print("\n");Serial.print("Phase");Serial.print("\t");Serial.print(a(1,0),5);Serial.print("\n");Serial.print(a(2,0),5);
  
  } //Ende while schleife

  
  Amplitude = 0; Phase = 0;
  if (a(0, 0) < 0) {
    Amplitude = sqrt(a(0, 0) * a(0, 0));
    Phase = a(1, 0) + PI;
  }
  if (a(0, 0) > 0) {
    Amplitude = a(0, 0);
    Phase = a(1, 0);
  }
  while (Phase > PI) {
    //Serial.println("größer als PI");
    Phase = Phase - 2 * PI;
  }

  while (Phase < -PI) {
    //Serial.println("größer als PI");
    Phase = Phase + 2 * PI;
  }
  delay(200);
  Serial.println("##### Ergebnisse für Spannung ###############");
  Serial.print("Amplitude: ");Serial.println(Amplitude,5);
  Serial.print("Phase: ");Serial.println(Phase,5); 
  //Speichere Messwerte
  u_amplitude = Amplitude;
  u_phase = Phase;

  Serial.println("##### Impedanz und Phase ###############");
  z = u_amplitude / i_amplitude;
  Serial.print("Impedanz: "); Serial.println(z,5);
  delta_phase = u_phase - i_phase;
  if (delta_phase <= 0)
  {
    delta_phase = delta_phase * -1.0;
  }
  Serial.print("Phase: ");Serial.println(delta_phase,5);
}


