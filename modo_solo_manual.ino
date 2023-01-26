          /************************LIBRERIAS***************/

#include <pt.h>                           //Libreria para el uso de hilos en arduino ,se decarga por afuero al ide de arduino ,visitar  https://roboticsbackend.com/arduino-protothreads-tutorial/
#include <LiquidCrystal_I2C.h>            //Libreria para LCD por I2C 
#include <Adafruit_ADS1015.h>             //libreria para el conversor A/D adafruit hay que decargar una version mas vieja 1.0.x.x que la ultima para que el ADA funcione
#include <TimerOne.h>                     //Libreria para utilizat e timer 1 del arduino el cual se usa para el acccionamiento del motor sin interferir en el programa
        /**************************************************/


        /********************DEFINICION DE PINES DEL ARDUINO *********************/
#define DIR_PIN          7              //Pin 7 establece la direccio0n del giro del motor e el driver 
#define STEP_PIN         6              //Pin 6 dda el paso para el accionamiento del motor tambien se conceta al driver
#define ENABLE_PIN       4              // habilitacion del driover ,no lo utilizo aun
#define carrera          2              // Interrupcion que finaliza el promgrama en caso de emergencia-STOP 
#define PIN_MAN          3              // Pin 3 de arduino ,señal de iterrupccccion que cambia el modo de fiunciomaineto del programa (NO ESTA EN USO)
#define INC_PIN          8              // Pin 8 de arduino pulsador que accionandolo aumenta presion --PULL
#define LOW_PIN          9              // simil anterior pero disminuye presion
        /************************************************************************/


        /****************** CREACION DE HILOS DEL PROGRAMA**********************/
struct pt hilo1; // hilo1 del multitasking para funcion de on presiones
struct pt hilo5; // hilo5 del multitasking para accionamiento manual
        /**********************************************************************/


        /*************************ESTRUCTURA PARA LA PANTALLA LCD*************/
LiquidCrystal_I2C lcd (0x27, 16, 2); // DIR, columnas , filas
        /*********************************************************************/

        /********Configuracion del conversor ADS mas variables ***************/
        
Adafruit_ADS1115 ads;                 // Inicializacion de ADC como objeto ads
const float multiplier = 0.0001875F;  //Ver manual de adafruit,constante de calibracion
int16_t adc0;                         //variuable de 16 bit la cual adquiere presion de forma analogica pra la camara interna 
int16_t adc1;                         // simil anterior pero par ala camara externa  
        /**********************************************************************/

        /*************variables utilizadas en el encoder***********************/
float grados;           //grados deplazados
float counter = 0;      //contador
        /*********************************************************************/

        /********VARIABLES GLOBALES UTILIZADAS A LO LARGO DEL PRGRAMA*********/

int  Pe=200;                     //variables de presiones seteadas por usuario como limites
int  Pi=500;

int lectura_presion_ext;        //variables globales para asignar lectura de presion
int lectura_presion_int;
int lectura_presion_int_A;
int lectura_presion_int_B;

int delta_Pc = 20;              //variables para los delta de carga y Fatiga
int delta_PF = 50;

int duty = 1;

static long t=0;                //conteo de tiempo restante
static long T=0;
static long tiempo_restante;

bool ejecutando;                //variable de control para iniciar y parar ensayoboo
bool ejecutando2;
bool ejecutando3;

int Pausa_ruptura=50;           //calibracion del tiempo en milisegundos que interrumpe el ensayo para retomar el resto de los hilos
int Pausa_fatiga=50;
int Muestreo_mediciones=100;

bool estado;                   //variable para el cambio de estado

    /**********************************************************************/


void setup() {
  
  /*****************Seteo de la comunicacion Serial***********************/
  Serial.begin(115200);       //inicio de comunicacion coon una frecuencia mayor a 9600 par auna mejor adquisison de datos
  /***********************************************************************/

  /****************Configuracion en inicciasion dela pantalla  LCD********/
  lcd.init();                 //inicializa el LCD en modo de libreria I2C
  lcd.backlight();            // habilita iluminacion posterior de LCD
  lcd.clear();                // limpia pantalla
  /***********************************************************************/

  /***************************Seteo del Timer 1***************************/
  Timer1.attachInterrupt(Step);   //LLamado a la funcion de iterrrupccion del timer
  Timer1.initialize(250);         // frecuancia en la que ocurre la interrupccioencargado de medir las per
  Timer1.stop();                  // para interrupccccon del timer(Se apaga el motor)
  /***********************************************************************/

  /*************Asignacion de valores a los pines del arduino*************/
  pinMode(2,INPUT_PULLUP);                                    // Pin 2 como iterrupcion para cdetener el programa 
  attachInterrupt(digitalPinToInterrupt(2),parada,CHANGE);    //Llamado a la funcion de interrupcccion del programa llamada parada, si como esta
  
  pinMode(3,INPUT_PULLUP);                                    // pin 3 par ael cambio de estado (No utilizado en solo manual)
 
  pinMode(STEP_PIN,   OUTPUT);                                // CONFIGURACION del los pines que van al driver 
  pinMode(DIR_PIN,    OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(DIR_PIN, LOW);

  pinMode(INC_PIN,INPUT_PULLUP);                              //CONFIGURACION DE los pines que van a los pulsadores
  pinMode(LOW_PIN,INPUT_PULLUP);
  digitalWrite(INC_PIN, HIGH);
  digitalWrite(LOW_PIN, HIGH);
  /************************************************************************/

  /****************************CREACION DE HILOS***************************/
  PT_INIT(&hilo1); //crea el hilo1
  PT_INIT(&hilo5); //crea el hilo5
  /************************************************************************/
}

void parada(){                //FUNCION de la parada, del programa obiamente
  while(1){};
  }


void Step() {               // Funcion del pulso que accciona el motor
  
    digitalWrite(STEP_PIN, HIGH);
    digitalWrite(STEP_PIN, LOW);
    
}


  /******************************MEDICION DE PRESIONES****************************************/
void medicion_presiones(struct pt *pt)  // Hilo encaragaddo de medir las presiones
{ 
  static long T;                        // varaibles de tiempo para entara y salir del hilo
  static long Taux3;
  PT_BEGIN(pt);                         //inicia el hilo1 (medicion de presiones)
  
  //lcd.clear();                        // visualizacion de varables de precion en la pnatalla del LCD
  lcd.setCursor(0,0);                   
  lcd.print("Pe=");
  lcd.setCursor(0,1);
  lcd.print("Pi=");

  adc0 = ads.readADC_SingleEnded(0);        //lectura de los valores obetendos por el ADC
  adc1 = ads.readADC_SingleEnded(1);
  
  float Voltage_int=adc0*multiplier*1.008;  // conversion de los valores obetenuidos por el ADC a presiones 
  float Voltage_ext=adc1*multiplier*1.008;
  int P_int=250*Voltage_int-250;
  int P_ext=250*Voltage_ext-250;
   
   do
  { 
     grados=counter/2;                 //declaracion de valoresm del encoder
    int degress=grados/2.844/260;
    T=millis();
    Taux3=T-t;
    
    lectura_presion_int = P_int;        //asignacion de variables
    lectura_presion_ext = P_ext;

    // variables impresas en el visor serie para seguir las mediciones
    Serial.print(lectura_presion_ext);  //pe
    Serial.print("    "); 
    Serial.print(lectura_presion_int);  //pi
    Serial.print("    "); 
    Serial.println(degress); // Impresion de encoder en pantalla

    //simil anterior pero para el LCD
    
    lcd.setCursor(3,0);
    lcd.print(lectura_presion_ext);
    lcd.setCursor(3,1);
    lcd.print(lectura_presion_int);

    lcd.setCursor(8,0);
    lcd.print("DP=");
    lcd.print(degress);
    lcd.print("    ");
    
    
                        //cerrando archivo
    PT_WAIT_WHILE(pt, (millis()-T)<Muestreo_mediciones);
    
    //Reinicio del LCD
    lcd.setCursor(3,0);
    lcd.print("    ");
    lcd.setCursor(3,1);
    lcd.print("    ");

  }while(true);
  PT_END(pt); //finaliza el hilo1
}

/*********************************************************************************************/



/**************************Hilo de funcionamineto manual del programa*************************/
void manual (struct pt *pt)
{
  static long Taux9;
  PT_BEGIN(pt) //inicio del hilo5 ensayo de fatiga
 
 ejecutando=HIGH;

  do//comienza carga de presiones hasta que la camara externa llegue a Pe def por Us
  {
    Taux9=millis();
if(digitalRead(INC_PIN) == LOW){              //si se accciona el pin de icrematar presion se activa
      motor(HIGH,LOW);
      //Serial.println("Aumentado presion");
      //delay(100);
    }
  else if(digitalRead(LOW_PIN) == LOW){       //simil anterior pero para disminuir la presion
      motor(HIGH,HIGH);
      //Serial.println("disminuyendo presion ");
      //delay(100);
      }
  else{
    motor(LOW,LOW);                       //en caso que no se actibven ningun pulsador el programa mantiene apagado al motor
    //Serial.println("APAGADO");
    //delay(100);
    }
   PT_WAIT_WHILE(pt, (millis()-Taux9)<Pausa_fatiga);
  }while ((ejecutando == HIGH));//cuando el cronometro marca 0 sale del bucle y termina el ensayo
 
  do // al finalizar el tiempo o el motor de la camara interna no puede compensar mas la presion se vuelve al punto de inicio
  {
   Taux9=millis();
  // motor_ext (Velocidad_mot_pp_ext,1);
   PT_WAIT_WHILE(pt, (millis()-Taux9)<Pausa_fatiga);
  }while(lectura_presion_ext != 0);
 
  Serial.println("FIN DE ENSAYO");
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("FIN DE ENSAYO");


  PT_END(pt); //finaliza el hilo5
}
/***********************************************************************************/

/******************************ACCIONAMIENTO DEL MOTOR******************************/
void motor(bool estado, bool sentido)           //funcion que se le enian los parametros para el accionado del motor , estado iundica si esta predido o apagado y giro mel sentido de giro
{ 
  if(estado == HIGH){                           //si esta encendido activa al motor
    digitalWrite(DIR_PIN, sentido);             // kle asigna la direccion de giro del motor
    Timer1.start();
    }
    else{                                       //si estado==LOW apaga el motor
      Timer1.stop();
      }
}
/************************************************************************************/


/************************PROGRAMA PRINCIPAL******************************************/
void loop() {
  
   medicion_presiones(&hilo1);        //LLAmado al hilo de lectura de presiones
   manual(&hilo5);                    //llamado al hilo de nacccionamiento manual del motor 
}
/***********************************************************************************/
