#include <PID_v1.h>
#include<Servo.h>

Servo ESC; //crear motor 1
Servo ESC1;//crear motor 2

const int sampleRate = 0.5;                                           // 0.5 millisecond de actualizacion

double Setpoint1, Input1, Output1;                                       //variables de PID
double consKp1=0.3276,consKi1=0.0088,consKd1=0.1;                                    //Declara constantes con valores inciales     
//double consKp1=0.9,consKi1=0.1,consKd1=0.3; 
PID myPID_B1(&Input1, &Output1, &Setpoint1,consKp1,consKi1,consKd1, DIRECT);   //Crear objeto PID

double Setpoint2, Input2, Output2;                                       //variables de PID
double consKp2=0.9,consKi2=0.01,consKd2=0;                                    //Declara constantes con valores inciales     
PID myPID_B2(&Input2, &Output2, &Setpoint2,consKp2,consKi2,consKd2, DIRECT);   //Crear objeto PID


char mssg;                                                             //variable para guardar mensaje de python    
int vel=880;                                                              //amplitud del pulso motor 1
int vel1;                                                             //amplitud del pulso motor 2
int led=8;int D=0;
int angulo= 0;
 
void setup(){
  
  pinMode(led, OUTPUT);
  Serial.begin(9600);
  myPID_B1.SetMode(AUTOMATIC);                                           //Setea valores en auto
  myPID_B1.SetSampleTime(sampleRate);                                    //Tiempo de actualizacion
  myPID_B2.SetMode(AUTOMATIC);                                           //Setea valores en auto
  myPID_B2.SetSampleTime(sampleRate);                                    //Tiempo de actualizacion

   mssg = Serial.read(); //leemos el serial
      
      if(mssg=='A'){angulo=8;}
      if(mssg=='B'){angulo=10;}
      if(mssg=='C'){angulo=20;}
      if(mssg=='D'){angulo=30;}
      if(mssg=='E'){angulo=40;}
      if(mssg=='F'){angulo=50;}
      if(mssg=='G'){angulo=60;}
      if(mssg=='H'){angulo=70;}
      if(mssg=='I'){angulo=80;}
      if(mssg=='J'){angulo=90;}
      Input1=angulo;
      if(Input1>=0){
        digitalWrite(led,HIGH);
      }
 //inicializa set point Brushless 1----------------------------------
  Setpoint1=0;                                                      //Setpoint// Valor de potenciometro no de velocidad NO CONFUNDIR!!
  myPID_B1.SetOutputLimits(880,1100);
  Input1=angulo+872;
  Output1=0;
  //------------------------------------------------------------------


  
  //inicializa set point Brushless 2----------------------------------
  Setpoint2=911;                                                      //Setpoint// Valor de potenciometro no de velocidad NO CONFUNDIR!!
  myPID_B2.SetOutputLimits(880,1200);
  Input2=analogRead(A0)-31;
  Output2=0;
  //------------------------------------------------------------------
  
  //--------------------Inicializacion de motores-----------------------//
  ESC.attach(6);
  ESC1.attach(9);
                      //Activar el ESC
  ESC.writeMicroseconds(0); //1000 = 1ms
  ESC1.writeMicroseconds(0);
                 //Cambia el 1000 anterior por 2000 si
                 //tu ESC se activa con un pulso de 2ms
  delay(3000);   //Esperar 5 segundos para hacer la activacion
  //Iniciar puerto serial
  ESC.writeMicroseconds (2000);
  ESC1.writeMicroseconds(2000);
 
  Serial.setTimeout(10);
  //--------------------Inicializacion de motores-----------------------//
  
  mssg = Serial.read(); //leemos el serial
  angulo= 5;


}

void loop()
{
  //-----------------------------LECTURA ÁNGULO EJE XZ------------------------------------// 
  //Serial.println(mssg); 
    mssg = Serial.read(); //leemos el serial
      
      if(mssg=='A'){angulo=8;}
      if(mssg=='B'){angulo=10;}
      if(mssg=='C'){angulo=20;}
      if(mssg=='D'){angulo=30;}
      if(mssg=='E'){angulo=40;}
      if(mssg=='F'){angulo=50;}
      if(mssg=='G'){angulo=60;}
      if(mssg=='H'){angulo=70;}
      if(mssg=='I'){angulo=80;}
      if(mssg=='J'){angulo=90;}
      Input1=angulo;
      if(Input1>=0){
        digitalWrite(led,HIGH);
      }


       
  //-----------------------------CONTROL MOTOR 1 ESC------------------------------------//
    delay(700);

    double er1= abs(Setpoint1-Input1);                         //Error de PID
    myPID_B1.Compute(); //Nuevo Output
    myPID_B1.SetTunings(consKp1,consKi1,consKd1);                //Setear constantantemente Kp,Ki,Kd
    //Serial.println(consKp1);
    //Serial.println(consKi1);
    //Serial.println(consKd1);

    ESC.writeMicroseconds(Output1);
   
  //-----------------------------CONTROL MOTOR 2 ESC1------------------------------------// 
  Input2=analogRead(A0)+180;                               //Lectura de potenciometro
        delay(700);
      Serial.print("Valor de A0 es ");
  Serial.println(Input2);
      Serial.print("Valor de OUTPUT es ");
  Serial.println(Output2);
  double er2= abs(Setpoint2-Input2);                         //Error de PID
  //Serial.println("Error es: ");
 // Serial.println(er);
    myPID_B2.Compute(); //Nuevo Output
    myPID_B2.SetTunings(consKp2,consKi2,consKd2);                //Setear constantantemente Kp,Ki,Kd
    Serial.println(consKp2);
    Serial.println(consKi2);
    Serial.println(consKd2);
    Serial.println("yeeeah");//LOGRO FUNCIONAR!!!!!

      ESC1.writeMicroseconds(Output2);
   if(Output2<=Setpoint2-80 and Output2>=Setpoint2+80){
     ESC1.writeMicroseconds(1);
    }
    
 //-----------------------------CONTROL MOTOR 1------------------------------------//    
    
}

