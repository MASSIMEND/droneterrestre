///motore a
//int potenziometro; //potenziometro collegato al piedino A0
int valorePWM;
//int enA = 6;//  motore engine A=enA=piedino 6 tramite segnale modulato pwm=976hz (possibile modif tramite timer 0), meglio non toccare
int enA = 9;//motore engine A=enA=piedino 9 tramite segnale modulato pwm=488hz (possibile modif tramite timer 1)
int in1 = 7;//avanti RUOTA-A=in1
int in2 = 8;// INDIETRO RUOTA-A=in2
//int i=80;// la ruota inizia a muoversi a 100 utilizzato con cilo for next
float t=0; //tempo
/// Variables //////////////////////////////////////////////////////////////////////////////////////////////////////////////
int encoder_pin = 2;             //Pin 2,  encoder       
  float  rpm = 0;           // ROTAZIONI PER MINUTO
  
float velocity = 0;                 //Veloc  [Km/h]
float velocitym = 0;               //Veloc  [m/s]
float a,b;
volatile int pulses = 0;       // NUMERO DI PULSAZIONI AL SECONDO O FRAZIONI DI SECONDO
unsigned long timeold = 0;  // variabile utilizzata per aggiornare il tempo 
unsigned int pulsesperturn = 50; // Número di fori dell'encoder rotellina
const int wheel_diameter = 86;   // Diametro della rotellina encoder[mm]
static volatile unsigned long debounce = 0; // contatore  per antirimbalzo impulsi
int tempocampionPID=10;//IMPORTANTE// intervallo ciclico di temPo in millis di attivazione algorit. PID
//----------------------------------PID----------------------------------

#include <PID_v1.h>
double Setpoint, Input, Output;// variabili per memorizzare i valori

//double Kp=80, Ki=20, Kd=1;
float Kp=180, Ki=110, Kd=500; //in questo caso abbiamo messo float
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
//----------------------------------PID----------------------------------

//------------------------ Config del Arduino -----------------------------------------
void setup() {

   

//----------------------------------PID----------------------------------

//initialize the variables we're linked to
  Input = velocitym;      //
  Setpoint = 2.5;         //VAL.GRANDEZZA (VELOCITA/GIRI/PULs)CHE VOGLIAMO MANTENERE, in questo caso metri/s

  //turn the PID on
  myPID.SetMode(AUTOMATIC);
myPID.SetSampleTime(tempocampionPID);//impostiamo ogni quanti millis l'algoritmo PID interviene
//----------------------------------PID----------------------------------
  
  Serial.begin(28800);
 //imposta i motori come output
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  // IMPOSTAZIONE-----------MOTORE VERSO
 digitalWrite(8, HIGH);// ruota A avanti abilitato
 digitalWrite(7, LOW);

  //...................Programmma ruota........................//
  pinMode(encoder_pin, INPUT); // Configuración del pin nº2
 //attachInterrupt(digitalPinToInterrupt(2),counter,RISING);// Configurazione dell'interrupt al pedino 2, variabile volatile COUNTER, MODO rising
 attachInterrupt(0, counter, RISING); // configuarazione alternativa a qella sopra del piedino dell'interrupt e del modo riing
   pulses = 0;
   rpm = 0;
   timeold = 0;
 //---------------------------------EXCEL PARALLAX-COMANDI-INIZIO per Foglio excel------------------
 Serial.println("CLEARDATA"); // CANCELLA IL FOGLIO ELETTRONICO
Serial.println("LABEL,nonusato, Secondi, Pulses, RPM, Valore PWM,Veloc. [m/s] ,Veloc. [Km/h]");
//---------------------------------------------------------------------------------------------------

}
void loop() {
  //----------------------------------PID----------------------------------
Input = velocitym;// come variabile di ingresso abbiamo la velocita in m/s ottenuto calcolando il numero di inpulsi nella frazione di secondo(0.20 secondi)
  myPID.Compute();
 valorePWM=Output;// l'output della elaborazione con algoritmo PID è un valore PWM(0-255) che verrà
                  // assegnata alla variabile valorePWM per essere utilizzata dal motore
//----------------------------------PID----------------------------------
  
//valorePWM=i; //utilizzta solo nel ciclo for next automatico senza PID
  analogWrite(enA, valorePWM);    // invia al motore il valore PWM ottenuto dal PID
//Serial.print("DATA,TIME,");Serial.print(millis()/1000);Serial.print(",");Serial.print(pulses,DEC);
 
  //...............INIZIO  SENSORE RUOTA....................... //
   t=millis();
 t=t/1000;
if (millis() - timeold >= 200){  // si attiva ogni 0,20 secondi
      noInterrupts(); //disattiva l'interruzione del piedino dove si ricevono gli impulsi ruotino per permettere il calcolo della velocità
      a=(60000 / pulsesperturn );
      b=(millis() - timeold);
      rpm = (a*pulses)/ b; // calcolo dei giri del ruotino forato per ogni minuto
      //rpm = (60 * 1000 / pulsesperturn )/ (mmillis() - timeold)* pulses; // Calculamos las revoluciones por minuto
      velocity = rpm * 3.1416 * wheel_diameter * 60 / 1000000; // Calcolo della velocità [Km/h] 
      velocitym = (velocity*10)/36;// calcolo velocità in m/s
      
//----------------------trasferimento dati al foglio excel----------------------------------    
      
  Serial.print("DATA,TIME,");Serial.print(t,2);Serial.print(",");Serial.print(pulses,DEC);Serial.print(",");Serial.print(rpm,2);Serial.print(",");Serial.print(valorePWM);Serial.print(",");Serial.print(velocitym,2);Serial.print(",");Serial.println(velocity,2);
//--------------------------------------------------------------------------------------------
timeold = millis(); // Memorizzaimo il tempo attuale per iniziare nuovamente la procedura di contare gli impulsi
         pulses = 0;  // INIZIALIZZIAMO NUOVAMENTE IL CONTATORE DI IMPULSI DELLA ROTELLINA
         interrupts(); // RIABILITIAMO IL PIEDINO DI INTERRUPT
  }
 }
// ....................FINE DEL PROGRAMMA PRINCIPALE................//
// .....................INIZIO CONTA IMPULSI.......................//
void counter(){ 
  if(  digitalRead (encoder_pin) && (micros()-debounce > 100) && digitalRead (encoder_pin) ) { 
// Verificare nuovamente che l'encoder emetta un buon segnale e quindi verificare che il tempo sia superiore a 100 microsecondi e controlla nuovamente che il segnale sia corretto.
        debounce = micros(); // memorizza il tempo in microsecondi aspettati per evitare impulsi non desiderati 
        pulses++;}  // Somma il numero di impulsi considerati affidabili
        //else ; 
        } 

