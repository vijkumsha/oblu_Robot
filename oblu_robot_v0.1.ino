// include the library code:
#include <SoftwareSerial.h>
#include <math.h>

#define packet_size 68
#define track_error_percent 5
#define theta_error_percent 7
// Mototrs//
#define motor1p 22
#define motor1m 26
#define motor2p 30
#define motor2m 34
unsigned int stepsw = 0;
double sin_phi, cos_phi;
double delta [4] = {0.0, 0.0, 0.0, 0.0};
double final_data [5];
double distance, distance1, track_length, heading = 0.0;
double dx [5];
double x_sw [5];
int bytedata, bytedata_usb;
byte ack [5] = {0, 1, 2, 3, 4};
int line[packet_size];
uint8_t header[4];
int newbyte = -1;
int x [4];
int y [4];
int z [4];
int h [4];
int steps[2];
bool flag_f, flag_b = false;
unsigned int j, k, l, package_number, package_number_old, datacount = 0;
int nturn;
float theta;
double track;
//double data_array[] = {0.5, 90, 0.5, -90, 0.5, 90, 0.5, -90, 0.5, 90, 0.5, 180, 0.5, -90, 0.5, 90, 0.5,  -90, 0.5, 90, 0.5, -90, 0.5, 0};//stair
//double data_array[] = {0.0, 45, 1.414, 135, 1, 135, 1.414, -135, 1, 0}; // cross
//double data_array[] = {1,180,1,180,0.01,0}; // 0x3f800000, 0x43340000, 0x3f800000, 0x00000000 //3F 80 00 00 43 34 00 00 3F 80 00 00 00 00 00 00 
double data_array[]= {1, -72, 1, -72, 1, -72, 1, -72, 1, -144, 1.618, 144, 1.618, 144, 1.618, 144, 1.618, 144, 1.618, 0};//pentagon
int data_array1[]= {};
int r, count;
typedef unsigned char uchar;

const int w_cmd[3] = {0x34, 0x00, 0x34};
//int serialdata(byte data);
void setup() {
  pinMode(motor1p, OUTPUT);
  pinMode(motor1m, OUTPUT);
  pinMode(motor2p, OUTPUT);
  pinMode(motor2m, OUTPUT);
  // initialize the serial communications:
  Serial3.begin(115200);
  Serial2.begin(115200);
  Serial.begin(115200);
   for (int i = 0; i < packet_size; i++) {
    line[i] = '\0';
  }
}

void loop() {
   // when characters arrive over the serial port...
  if (Serial3.available()) {
    // wait a bit for the entire message to arrive
    delay(100);
    // read all the available characters
    while (Serial3.available() > 0) {
       bytedata = Serial3.read();
      if (newbyte == -1) {
        if (bytedata == 0xA0) {
          datacount = 0;
        }
        if (bytedata == 0xD4) {
          datacount++;
          newbyte = 0;
          break;
        }
      }
      else {
        if (datacount) {
          serialdata(bytedata);
        }
      }
    }
  }
}
void serialdata(int data) {
  line[j] = data;
  j++;
  if (j == 63) {
    //////////////////////////////////header assign///////////////
    for (int i = 0; i < 4; i++) {
      header[i] = line[i];
    }
    //////////////////////////////////// payload //////////////////////////
    for (int i = 0; i < 4; i++) { /////////////x 4 byte////////////
      x[i] = line[i + 4];
      dx[0] = bytesToFloat(x[0], x[1], x[2], x[3]);
    }
    for (k = 0; k < 4; k++) { //////////////y 4 byte////////
      y[k] = line[k + 8];
      dx[1] = bytesToFloat(y[0], y[1], y[2], y[3]);
    }
    for (k = 0; k < 4; k++) { /////////////z 4 byte////////////////
      z[k] = line[k + 12];
        dx[2] =  bytesToFloat(z[0], z[1], z[2], z[3]);
    }
    for (k = 0; k < 4; k++) { /////////////Heading 4 byte////////////////
      h[k] = line[k + 16];
        dx[3] =  bytesToFloat(h[0], h[1], h[2], h[3]);
    }
    for (k = 0; k < 1; k++) { /////////////step count 2 byte////////////////
      steps[k] = line[k + 61];
    }
    int package_number1 = header[1];
    int package_number2 = header[2];
    //////////////////////////////// Create ack for next step///////////////////////////////
    ack[0] = createAck(ack, package_number1, package_number2); // Acknowledgement created
    ack[1] = createAck(ack, package_number1, package_number2); // Acknowledgement created
    ack[2] = createAck(ack, package_number1, package_number2); // Acknowledgement created
    ack[3] = createAck(ack, package_number1, package_number2); // Acknowledgement created
    ack[4] = createAck(ack, package_number1, package_number2); // Acknowledgement created
    package_number = package_number1 * 256 + package_number2;  //PACKAGE NUMBER ASSIGNED
    if (package_number_old != package_number) {
      stepsw++;
      stepwise_dr_tu();
      package_number_old = package_number;
    }
     motorcontrols();
    j = 0;
  }
}
byte createAck(byte ack [], int packet_number_1, int packet_number_2){
  ack[0] = 0x01; // 1st byte
  ack[1] = (byte)packet_number_1; // 2nd byte
  ack[2] = (byte)packet_number_2; // 3rd byte
  ack[3] = (byte)((1 + packet_number_1 + packet_number_2 - (1 + packet_number_1 + packet_number_2) % 256) / 256); // 4th byte – Quotient of {(1+P1+P2) div 256}
  ack[4] = (byte)((1 + packet_number_1 + packet_number_2) % 256); // 5th byte- Remainder of {(1+P1+P2)div 256
  return ack[4];
}

float bytesToFloat(uchar b0, uchar b1, uchar b2, uchar b3){
  union {
    float f;
    uchar b[4];
  } u;
  u.b[3] = b0;
  u.b[2] = b1;
  u.b[1] = b2;
  u.b[0] = b3;
  return u.f;
}
void stepwise_dr_tu()
{
  sin_phi = (float) sin(x_sw[3]);
  cos_phi = (float) cos(x_sw[3]);
  delta[0] = cos_phi * dx[0] - sin_phi * dx[1];
  delta[1] = sin_phi * dx[0] + cos_phi * dx[1];
  delta[2] = dx[2];
  delta[3] += dx[3] * 57.3;
  x_sw[0] += delta[0];
  x_sw[1] += delta[1];
  x_sw[2] += delta[2];
  x_sw[3] += dx[3];
  final_data[0] = x_sw[0];
  final_data[1] = x_sw[1];
  final_data[2] = x_sw[2];
  final_data[3] = delta[3];
  distance1 = sqrt((delta[0] * delta[0] + delta[1] * delta[1] + delta[2] * delta[2]));
  heading += dx[3] * 57.3; 
  if(flag_f){
    track_length += distance1;
    distance += distance1;
  }
  if(flag_b){
    track_length -= distance1;
    distance -= distance1;
  }
}
void move_forward(int mdelay){
    digitalWrite(motor1p, HIGH);
    digitalWrite(motor1m, LOW);
    digitalWrite(motor2p, HIGH);
    digitalWrite(motor2m, LOW);
    delay(mdelay);
    stop_m();
    flag_f = true;
    flag_b = false;
}
void move_backward(int mdelay){
    digitalWrite(motor1p, LOW); 
    digitalWrite(motor1m, HIGH);
    digitalWrite(motor2p, LOW);
    digitalWrite(motor2m, HIGH);
    delay(mdelay);
    stop_m();
    flag_b = true;
    flag_f = false;
}
void turn_left(int mdelay){
    digitalWrite(motor1p, HIGH);
    digitalWrite(motor1m, LOW);
    digitalWrite(motor2p, LOW);
    digitalWrite(motor2m, HIGH);
    delay(mdelay);
    stop_m();
    flag_f = false;
    flag_b = false;
}
void turn_right(int mdelay){
    digitalWrite(motor1p, LOW); 
    digitalWrite(motor1m, HIGH);
    digitalWrite(motor2p, HIGH);
    digitalWrite(motor2m, LOW);
    delay(mdelay);
    stop_m();
    flag_f = false;
    flag_b = false;
}
void stop_m(){
    digitalWrite(motor1p, LOW);
    digitalWrite(motor1m, LOW); 
    digitalWrite(motor2p, LOW);
    digitalWrite(motor2m, LOW);
}
void motorcontrols() {
   
   bool turn = false;
   float dis_error = 0;
   float theta_error = 0;
   int forw_delay, back_delay = 0;
   int left_delay, right_delay = 0;
   
  track = data_array[nturn];
  theta = (float) data_array[nturn+1];

  double delta_track = fabs(track - track_length);
  float delta_theta = fabs(theta - heading);
  //if (nturn==0){
     dis_error = 0.015;
     theta_error = 3;
 /* }
  else {
     dis_error = (track * track_error_percent)/200;
     theta_error = (theta * theta_error_percent)/200;
  }
*/

  if (delta_track < 0.2){
        forw_delay = 300;
  }
   else if (delta_track >= 0.2 && delta_track < 0.5){
        forw_delay = 500;
  }
  else if (delta_track >= 0.5 && delta_track < 1){
        forw_delay = 700;
  }
   else if (delta_track >= 1){
        forw_delay = 800;
  }
  back_delay = forw_delay;
  
  if (delta_theta < 15){
        left_delay = 80;
  }
   else if (delta_theta >= 15 && delta_theta < 30){
        left_delay = 140;
  }
  else if (delta_theta >= 30 && delta_track < 60){
        left_delay = 160;
  }
   else if (delta_theta > 60){
        left_delay = 180;
  }
  right_delay = left_delay;
  
  if ((track_length >= (track - dis_error)) && (track_length <= (track + dis_error)))
     turn = true;
  else 
     turn = false;

  if (!turn){
     if (heading >= 3-1.5) {
       turn_left(40);
     }
     else if ((heading <= (-3+1.5))) {
       turn_right(40);
     }
     else if ((track_length < (track - dis_error)) ) {
       move_forward(forw_delay);
     }
     else if ((track_length >= (track + dis_error))) {
       move_backward(100);
     }
  }
  else {
    if(theta == 0){
      stop_m();
      j = 0;
      nturn = 0;
      datacount = 0;
      newbyte = -1;
    }
    else if (heading < (theta - theta_error)) {
     turn_right(right_delay);
    }
    else if (heading > (theta + theta_error)) {
     turn_left(left_delay);
    }
    else{                                                                                              
     track_length = 0;
     heading = 0;
     nturn=nturn+2;
    }
  }
}




