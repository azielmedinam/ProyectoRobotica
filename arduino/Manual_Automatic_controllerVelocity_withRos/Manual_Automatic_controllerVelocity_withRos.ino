//ROS Part
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#define W_MAX 15.7   // Velocidad maxima del motor (rad/s

//Bluetooth Part
#include <SoftwareSerial.h>

//Motor Derecha
#define MOTR_IN1 3   // Pin de dirección de motor 1 (Left)
#define MOTR_IN2 2   // Pin de dirección de motor 1 (Left)
#define MOTR_EN 6    // Enable para motor 1 (Left)

#define ENCR_A 19      // Pin A del encoder motor 1
#define ENCR_B 18      // Pin B del encoder motor 1

// Motor Izquierda
#define MOTL_IN1 4   // Pin de dirección de motor 1 (Left)
#define MOTL_IN2 5   // Pin de dirección de motor 1 (Left)
#define MOTL_EN 7   // Enable para motor 1 (Left)

#define ENCL_A 21     // Pin A del encoder motor 1
#define ENCL_B 20     // Pin B del encoder motor 1

//ROS VARIABLES
ros::NodeHandle nh;
float wheelbase = 0.43;  // Replace with the actual value
float radio=0.06;

//CONTROLLER VARIABLES
  //Cuentas del encoder RIGHT
long encoderCountR = 0;
long prevEncoderCountR = 0;
long totalCountsR = 0;
int aPrevStateR = 0;

  //Cuentas del encoder LEFT
long encoderCountL = 0;
long prevEncoderCountL = 0;
long totalCountsL = 0;
int aPrevStateL = 0;

  //Valores RPM, rad/s RIGHT
float wRefR = 0.0;
float wR = 0.0, rpmR = 0.0;

  //Valores RPM, rad/s LEFT
float wRefL = 0.0;
float wL = 0.0, rpmL = 0.0;

  //Valores tiempo
float prevT = 0, cambioT = 0, tiempo = 0;

  //Variables del controlador
float dt=0.02;
float Ts;
float kp, ki, kd;

  //RIGHT
float q0_r, q1_r, q2_r;
float u_r[2] = {0.0, 0.0}; // u_1[0] salida actual   u[1] salida anterior
float e_r[3] = {0.0, 0.0, 0.0};  //e[0] error actual    e[1] anterior  e[2] dos veces anterior
float retroR; //retroalimentacion

  //LEFT
float q0_l, q1_l, q2_l;
float u_l[2] = {0.0, 0.0}; // u_1[0] salida actual   u[1] salida anterior
float e_l[3] = {0.0, 0.0, 0.0};  //e[0] error actual    e[1] anterior  e[2] dos veces anterior
float retroL; //retroalimentacion

//BLUETOOTH VARIABLES
  //Coordinates Variables
SoftwareSerial btSerial(10, 11); // RX, TX
String valorStr = "";
int x;
int y;
bool boton = true;
int ejeXvalor;
int ejeYvalor;
int ejeYvelocidadAdelante;
int ejeYvelocidadAtras;
int ejeXvelocidadDerecha;
int ejeXvelocidadIzquierda;

//ROS TOPICS
// Define global variables for wheel speeds
float wr = 0.0;  // Right wheel speed (rad/s)
float wl = 0.0;  // Right wheel speed (rad/s)

// Publishers for wheel speeds
std_msgs::Float32 wl_msg;
ros::Publisher wl_pub("/wl", &wl_msg);

std_msgs::Float32 wr_msg;
ros::Publisher wr_pub("/wr", &wr_msg);

// Callback function to process received Twist messages
void cmdVelCallback(const geometry_msgs::Twist& twist_msg) {
  // Extract linear and angular components from Twist message
  float linear_vel = twist_msg.linear.x;   // Linear velocity (m/s)
  float angular_vel = twist_msg.angular.z;  // Angular velocity (rad/s)

  // Calculate wheel speeds using differential drive equations
  wl = ((2*linear_vel) - (angular_vel * wheelbase)) / (2.0*radio);
  wr = ((2*linear_vel) + (angular_vel * wheelbase)) / (2.0*radio);

  wRefL = wl;
  wRefR = wr;
  
  // Publish wheel speeds
  wl_msg.data = wl;
  wl_pub.publish(&wl_msg);

  wr_msg.data = wr;
  wr_pub.publish(&wr_msg);
}

ros::Subscriber<geometry_msgs::Twist> cmdVelSub("/cmd_vel", &cmdVelCallback);

void setup() {
  //Velocidad serial 
  Serial.begin(57600);

  //Velocidad serial bluetooth
  btSerial.begin(9600);
  
  //setup RIGHT
  pinMode(MOTR_IN1, OUTPUT);
  pinMode(MOTR_IN2, OUTPUT);
  pinMode(MOTR_EN, OUTPUT);
  pinMode(ENCR_A, INPUT);
  pinMode(ENCR_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCR_A), countEncoder_r, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCR_B), countEncoder_r, CHANGE);
  aPrevStateR = digitalRead(ENCR_A);

  //setup LEFT
  pinMode(MOTL_IN1, OUTPUT);
  pinMode(MOTL_IN2, OUTPUT);
  pinMode(MOTL_EN, OUTPUT);
  pinMode(ENCL_A, INPUT);
  pinMode(ENCL_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCL_A), countEncoder_l, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCL_B), countEncoder_l, CHANGE);
  aPrevStateL = digitalRead(ENCL_A);
  
  //Variables del controlador
  kp = 4.4;
  ki = 30;
  kd = 0.01;
  Ts = 0.02; 
 
  q0_r= kp+(ki*Ts/2)+(kd/Ts);
  q1_r= -kp+(ki*Ts/2)-(2.0*kd/Ts);
  q2_r= kd/Ts;
  q0_l= kp+(ki*Ts/2)+(kd/Ts);
  q1_l= -kp+(ki*Ts/2)-(2.0*kd/Ts);
  q2_l= kd/Ts;
  
  // Initialize ROS node
  nh.initNode();
  nh.subscribe(cmdVelSub);

  // Advertise the wheel speed publishers
  nh.advertise(wl_pub);
  nh.advertise(wr_pub);
}

void loop() {
  tiempo = millis();
  joystick_read();

  if (x < 485 || x > 505 || y < 485 || y > 505) {
     // Si los valores están fuera del rango, respondemos al joystick
     manual_move(); // Asumiendo que esta función maneja el control manual
  } else {
      // Si los valores están dentro del rango, funcionamos de manera autónoma
     automatic_move(); // Asumiendo que esta función maneja el control autónomo
   }
  nh.spinOnce();
}

//CONTROLLER FUNCTIONS
void countEncoder_r() {
  //Guarda valores del canal A y B del enconder del motor de la derecha
  int encA = digitalRead(ENCR_A);
  int encB = digitalRead(ENCR_B);

  if (encA !=  aPrevStateR) {
    if (encB != encA){
       //Cuenta hacia adelante
      encoderCountR++;
    }else{
      //Cuenta hacia atrás
      encoderCountR--;
    }
    aPrevStateR = encA;
  }
}

void countEncoder_l() {
  //Guarda valores del canal A y B del enconder del motor de la derecha
  int encA = digitalRead(ENCL_A);
  int encB = digitalRead(ENCL_B);

  if (encA !=  aPrevStateL) {
    if (encB != encA){
       //Cuenta hacia adelante
      encoderCountL++;
    }else{
      //Cuenta hacia atrás
      encoderCountL--;
    }
    aPrevStateL = encA;
  }
}

//MANUAL MOVE FUNCTIONS
/*
  * Funciones Carro: Adelante, Atras, Derecha, Izquierda
*/
void adelante(int vel){ 
  digitalWrite(MOTR_IN1,HIGH);
  digitalWrite(MOTR_IN2,LOW);
  digitalWrite(MOTL_IN1,HIGH);
  digitalWrite(MOTL_IN2,LOW);
  analogWrite(MOTR_EN,vel);
  analogWrite(MOTL_EN,vel);
}

void atras(int vel){ 
  digitalWrite(MOTR_IN1,LOW);
  digitalWrite(MOTR_IN2,HIGH);
  digitalWrite(MOTL_IN1,LOW);
  digitalWrite(MOTL_IN2,HIGH);
  analogWrite(MOTR_EN,vel);
  analogWrite(MOTL_EN,vel);
}

void derecha(int vel){ 
  digitalWrite(MOTR_IN1,HIGH);
  digitalWrite(MOTR_IN2,LOW);
  digitalWrite(MOTL_IN1,LOW);
  digitalWrite(MOTL_IN2,HIGH);
  analogWrite(MOTR_EN,vel);
  analogWrite(MOTL_EN,vel);
}

void izquierda(int vel){ 
  digitalWrite(MOTR_IN1,LOW);
  digitalWrite(MOTR_IN2,HIGH);
  digitalWrite(MOTL_IN1,HIGH);
  digitalWrite(MOTL_IN2,LOW);
  analogWrite(MOTR_EN,vel);
  analogWrite(MOTL_EN,vel);
}

void parar(){ 
  digitalWrite(MOTR_IN1,LOW);
  digitalWrite(MOTR_IN2,LOW);
  digitalWrite(MOTL_IN1,LOW);
  digitalWrite(MOTL_IN2,LOW);
  analogWrite(MOTR_EN,0);
  analogWrite(MOTL_EN,0);
}

//Manual move (loop)
void joystick_read(){
  // Lectura serial de datos del joystick por bluetooth
  while (btSerial.available()) {
    char c = btSerial.read();
    if (c == ',') {
      x = valorStr.toInt();
      //Serial.println(x); // Mostrar el valor entero en el Monitor Serial.
      valorStr = "";
    } else if (c == '.') {  // Fin de línea indica el final del número.
      y = valorStr.toInt();
      //Serial.println(y); // Mostrar el valor entero en el Monitor Serial.
      valorStr = "";
    } else if (c == '\n') { 
      
      boton = false;
      valorStr = "";
    } else {
      valorStr += c;
    }
  }
  ejeXvalor = x;
  ejeYvalor = y;
}

void manual_move(){
  Serial.println("Modo manual");
   // Mapea los valores del joystick a la velocidad de los motores
  ejeYvelocidadAdelante = map(ejeYvalor, 512, 1023, 0, 255);
  ejeYvelocidadAtras = map(ejeYvalor, 512, 0, 0, 255);
  ejeXvelocidadDerecha = map(ejeXvalor, 512, 1023, 0, 255);
  ejeXvelocidadIzquierda = map(ejeXvalor, 512, 0, 0, 255);
  
  // Movimiento de motores
  if(ejeYvalor > 550){
    adelante(ejeYvelocidadAdelante);
  }
  else if(ejeYvalor < 474){
    atras(ejeYvelocidadAtras);
  }
  else if(ejeXvalor > 550){
    derecha(ejeXvelocidadDerecha);
  }
  else if(ejeXvalor < 474){
    izquierda(ejeXvelocidadIzquierda);
  }
  else if(ejeYvalor < 550 && ejeYvalor > 474 && ejeXvalor < 550 && ejeXvalor > 474){
    parar();
  }
}

void automatic_move(){
  Serial.println("Modo automatico");
  if (cambioT >= 29) {
    Serial.println("Cuenta 29");
    prevT = tiempo;
    
    //CUENTAS DEL ENCODER

    //RIGHT
    totalCountsR = encoderCountR - prevEncoderCountR;
    rpmR = ((totalCountsR * 60.0)/0.029) / (30.0 * 70.0);
    wR = (2.0*3.1416*rpmR)/ 60.0;
    prevEncoderCountR = encoderCountR;

    //LEFT
    totalCountsL = encoderCountL - prevEncoderCountL;
    rpmL = ((totalCountsL * 60.0)/0.029) / (30.0 * 70.0);
    wL = (2.0*3.1416*rpmL)/ 60.0;
    prevEncoderCountL = encoderCountL;
    
    // INICIO DE AREA DE CONTROL

    //RIGHT
    retroR = wR;
    //calcular la señal de error
    e_r[0] = wRefR - retroR;
    //calcular la ecuación en diferencias
    u_r[0]=q0_r*e_r[0]+q1_r*e_r[1]+q2_r*e_r[2]+u_r[1];
    //limitar la señal de control
    if(u_r[0] > 255.0) u_r[0] = 255;
    if(u_r[0] < -255.0) u_r[0] = -255;
    //corrimiento para mover las muestras
    e_r[2] = e_r[1];
    e_r[1] = e_r[0];
    u_r[1] = u_r[0];

    //LEFT 
    retroL = wL;
    //calcular la señal de error
    e_l[0] = wRefL - retroL;
    //calcular la ecuación en diferencias
    u_l[0]=q0_l*e_l[0]+q1_l*e_l[1]+q2_l*e_l[2]+u_l[1];
    //limitar la señal de control
    if(u_l[0] > 255.0) u_l[0] = 255;
    if(u_l[0] < -255.0) u_l[0] = -255;
    //corrimiento para mover las muestras
    e_l[2] = e_l[1];
    e_l[1] = e_l[0];
    u_l[1] = u_l[0];
    
    // FIN DE AREA DE CONTROL
  
    //SALIDAS DE VOLTAJE PWM

    //RIGHT
    if ((int)u_r[0]<0){
      analogWrite(MOTR_EN,(int)(-u_r[0]));
      digitalWrite(MOTR_IN1, LOW); // Establece la dirección del motor
      digitalWrite(MOTR_IN2, HIGH);
    }
    if ((int)u_r[0]>=0){
      analogWrite(MOTR_EN,(int)(u_r[0]));
      digitalWrite(MOTR_IN1, HIGH); // Establece la dirección del motor
      digitalWrite(MOTR_IN2, LOW);
    }

    //LEFT
    if ((int)u_l[0]<0){
      analogWrite(MOTL_EN,(int)(-u_l[0]));
      digitalWrite(MOTL_IN1, HIGH); // Establece la dirección del motor
      digitalWrite(MOTL_IN2, LOW);
    }
    if ((int)u_l[0]>=0){
      analogWrite(MOTL_EN,(int)(u_l[0]));
      digitalWrite(MOTL_IN1, LOW); // Establece la dirección del motor
      digitalWrite(MOTL_IN2, HIGH);
    }
   
//    Serial.print("Referencia_R: ");
//    Serial.println(wRefR);
//    Serial.print("Velocidad actual_R: ");
//    Serial.println(retroR);
//    Serial.print("Error R: ");
//    Serial.println(e_r[0]);
//    Serial.print("Señal de control R: ");
//    Serial.println(u_r[0]);
//
//    Serial.print("Referencia_L: ");
//    Serial.println(wRefL);
//    Serial.print("Velocidad actual_L: ");
//    Serial.println(retroL);
//    Serial.print("Error L: ");
//    Serial.println(e_l[0]);
//    Serial.print("Señal de control L: ");
//    Serial.println(u_l[0]);
  }
   cambioT = tiempo - prevT;
}
