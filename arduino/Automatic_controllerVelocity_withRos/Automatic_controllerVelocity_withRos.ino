//ROS Part
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#define W_MAX 15.7   // Velocidad maxima del motor (rad/s)

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

// Bocina
#define BOCINA 8

// Notes of the melody:
#define NOTE_E4  330
#define NOTE_F4  349
#define NOTE_G4  392
#define NOTE_A4  440
#define NOTE_B4  494
#define NOTE_C5  523
#define NOTE_D5  587
#define NOTE_E5  659

// Duration of each note (in milliseconds):
#define DUR_1  200  // 1 beat
#define DUR_2  400  // 2 beats

int noteDurations;

//ROS variables
ros::NodeHandle nh;
float wheelbase = 0.43;  // Replace with the actual value
float radio=0.06;

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

//ROS topics
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

void soundCallback(const std_msgs::Int32& int_msg){
  int state = int_msg.data;
  /*int melody[] = {
    NOTE_E4, NOTE_G4, NOTE_E5, NOTE_D5, NOTE_C5, 
    NOTE_B4, NOTE_A4, NOTE_G4, NOTE_F4, NOTE_G4
  };
  
  int noteDurations[] = {
    DUR_1, DUR_1, DUR_1, DUR_1, DUR_1, 
    DUR_1, DUR_1, DUR_1, DUR_1, DUR_2
  };
  if(state == 1){
    for (int thisNote = 0; thisNote < 10; thisNote++) {
      int noteDuration = noteDurations[thisNote];
      tone(BOCINA, melody[thisNote], noteDuration);
      int pauseBetweenNotes = noteDuration * 1.30;
      delay(pauseBetweenNotes);
      noTone(BOCINA);
    }
  } else {
    noTone(BOCINA);
  }*/
   if (state == 1) {
    // Encender la bocina
    tone(BOCINA, 1000);  // Puedes ajustar la frecuencia según sea necesario
  } else {
    // Apagar la bocina
    noTone(BOCINA);
  }

}

ros::Subscriber<geometry_msgs::Twist> cmdVelSub("/cmd_vel", &cmdVelCallback);
ros::Subscriber<std_msgs::Int32> soundSub("/sound", &soundCallback);

void setup() {
  Serial.begin(57600);
  
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

  //bocina
  pinMode(BOCINA, OUTPUT);
  
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
  nh.subscribe(soundSub);

  // Advertise the wheel speed publishers
  nh.advertise(wl_pub);
  nh.advertise(wr_pub);
}

void loop() {
  tiempo = millis();
  if (cambioT >= 29) {
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
   
    Serial.print("Referencia_R: ");
    Serial.println(wRefR);
    Serial.print("Velocidad actual_R: ");
    Serial.println(retroR);
    Serial.print("Error R: ");
    Serial.println(e_r[0]);
    Serial.print("Señal de control R: ");
    Serial.println(u_r[0]);

    Serial.print("Referencia_L: ");
    Serial.println(wRefL);
    Serial.print("Velocidad actual_L: ");
    Serial.println(retroL);
    Serial.print("Error L: ");
    Serial.println(e_l[0]);
    Serial.print("Señal de control L: ");
    Serial.println(u_l[0]);
  }
  cambioT = tiempo - prevT;
  nh.spinOnce();
}

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
