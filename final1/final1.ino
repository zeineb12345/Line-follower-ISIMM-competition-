//Les moteurs:
#define RightMotor1 10 // forward (IN1) ymin
#define RightMotor2 11 // backward (IN2)
#define LeftMotor1 5 // forward (IN3) ysar
#define LeftMotor2 6 // backward (IN4)
int zone=1;
int speed = 120;
int x,y,time_started;
bool done=false;
bool west_S2=false;
bool west_S = false;
float motorCoeff = 0.98;
//Les capteurs:
int Sensor[]={A5,A4,A3,A2,A1,A0}; //A5 the one on the far left and A0 the one on the far right
char sensorValChar[7] = {"\0"}; // a string representing the current values of each sensor ( we have 6 sensors in our case)
int sensorVal[6]; //a table respresenting the current values of each sensot (integers)
long thershold[6];
int penderations[6] = {-3,-2,-1,1,2,3};
int penderationsRight[6] = {-3,-2,-1,1,6,11};
int penderations2[6] = {-10,-5,-1,1,3,4};
int penderationsRight2[6] = {-3,-2,-1,1,8,14};
//int penderationsRight[6] = {-9,-6,-1,1,2,3};
int tabChoice = 1; // choix des penderations (entre penderationsRight (1) et penderations (0) )

//PID related:
float Kp = 23 ;
float Kd =3;
float error = 0;
float previousError = 0;


//Autres pins :
const int LED_PIN = 12;
const int BUTTON_PIN = 8;
unsigned long time;




//-----------------------------------All the functions we used-------------------------------------------------



/*function calibrage which will run once (in the setup) before the robot starts working on the map
its main goal is to determine the thershold for each sensor which will help us find out if the sensors are reading
black or white later*/
void calibrage()
{
  //led turns on indicating calibrage started
  digitalWrite(LED_PIN, HIGH);


  //each sensor has a diffrent threshold for more precision
  long moyWhite[]={0,0,0,0,0,0};
  long moyBlack[]={0,0,0,0,0,0};
  
  int n = 100; //number of times we're gonna read the values for each sensor for each color
  Serial.println("begin reading white");
  //reading n white values for each sensor
  for(int i = 1 ; i <= n ; i++){
   for(int j = 0 ; j<6 ; j++)
      moyWhite[j] += analogRead(Sensor[j]);
    
    delay(20);
  }
  Serial.println("end of reading white");

  digitalWrite(LED_PIN, LOW);
  delay(3000);
  digitalWrite(LED_PIN, HIGH);

  Serial.println("begin of reading black");
  //reading n*6 black values
  for(int i = 1 ; i <= n ; i++){
    for(int j = 0 ; j<6 ; j++)
      moyBlack[j] += analogRead(Sensor[j]);
    delay(20);
  }
  digitalWrite(LED_PIN, LOW);

  Serial.println("end of reading black");

  //determening threshold for each sensor
  for(int i = 0 ; i<6 ; i++){
    thershold[i] = (moyBlack[i] + moyWhite[i]) / (n*2);
    Serial.println(thershold[i]);
    }
  
}

//running each motor either forward or backwards
void runMotors(int left, int right)
{
  left = checkMotorVal(left*motorCoeff);
  right = checkMotorVal(right);
  //running left motor
  if(left>=0) //foraward
  {
    analogWrite(LeftMotor1, left);
    analogWrite(LeftMotor2, 0 );
  }
  else { //backward
    analogWrite(LeftMotor1, 0);
    analogWrite(LeftMotor2, abs(left) );
  }

  //running right motor
  if(right>=0) //forward
  {
    analogWrite(RightMotor1, right);
    analogWrite(RightMotor2, 0 );
  }
  else { //backward
    analogWrite(RightMotor1, 0);
    analogWrite(RightMotor2, abs(right));
  }


}

//Making sure the value never gets out of the range[-255,255]
int checkMotorVal(int a){
  if(a>220)
    return(220);
  else if(a<-220)
    return(-220);
  else
    return(a);
}

void readSensors(){
  int n = 4; // n is the number of readings we need to caculate their average and then consider the average as one reading
  
  for(int i = 0 ; i <= 5 ; i++) // intialisation to 0 (to calculate average later)
    sensorVal[i] = 0;
  
  for(int i = 1 ; i <= n ; i++)// for each ith sensor we read if it's in black (1) or white (-1) (using analogToDigital function) and add it to sensorVal[i]
  {
    for(int j=0 ; j<6 ; j++)
    sensorVal[j] += analogToDigital( analogRead(Sensor[j]) ,thershold[j] );
    //serial.println(sensorVal[j])
  }

  //finding out each sensor's average value
  
    for(int i=0 ; i<6 ; i++)
      sensorVal[i] = sensorVal[i]>0; // if the average sensor value is + then (readings =1 > =0 ) so we return 1, else we return 0 (either readings = 0 > =1 )
  
  //updating sensorValChar for later use
  for(int i=0 ; i<6 ; i++){
    if(sensorVal[i]==0)
      sensorValChar[i] ='0'; 
    else
      sensorValChar[i] ='1';
    }
}

//Function that returns -1 if reading is <threshold (white) and 1 if reading is >=threshold (black)
int analogToDigital(int reading, int threshold){ 
  if(reading < threshold)
    return -1; //white
  else
    return 1; //black
}

//calculating error for PID
void calculErreur()
{ 
  error = 0;
  if(tabChoice == 0)
    for(int i = 0 ; i <= 5 ; i++)
      error += (sensorVal[i] == 1)*penderations[i]; //sensor = 1 means sensor detects black
  else if(tabChoice == 2)
    for(int i = 0 ; i <= 5 ; i++)
      error += (sensorVal[i] == 1)*penderations2[i]; //sensor = 1 means sensor detects black
  else if (tabChoice == 1)
   for(int i = 0 ; i <= 5 ; i++)
      error += (sensorVal[i] == 1)*penderationsRight[i]; //sensor = 1 means sensor detects black;
  else
    for(int i = 0 ; i <= 5 ; i++)
      error += (sensorVal[i] == 1)*penderationsRight2[i]; //sensor = 1 means sensor detects black;

}

//Calcul PID pour determiner les vitesses des moteurs
int calculPID(){
    calculErreur();
    if(!strcmp(sensorValChar,"000000")){
      error = previousError*2;
    }
    float output;
    output=Kp*error;
    output+= Kd*(error-previousError);
    previousError = error;
    return output;
}

//Running motors using the PID correction we just calculated
void RunPID(int speedMoy){//running pid
    int consigne = calculPID();
      runMotors(speedMoy+consigne , speedMoy-consigne);
  }



//setup
void setup() {
  //defining input and output
  pinMode(LeftMotor1,OUTPUT);
  pinMode(LeftMotor2,OUTPUT);
  pinMode(RightMotor1,OUTPUT);
  pinMode(RightMotor2,OUTPUT);
  
  pinMode(BUTTON_PIN, INPUT_PULLUP );
  pinMode(LED_PIN, OUTPUT);

  for(int i=0 ;i<6;i++)
    pinMode(Sensor[i], INPUT);
  
  calibrage(); //calibrage in white first then calibrage in black
  while(digitalRead(BUTTON_PIN) == LOW){};
  
  delay(300);
  time = millis();
  runMotors(120, 120);
  delay(1300);
}

void loop() {
  
  readSensors();
  RunPID(speed);
  if (millis()-time >=8500)
  {
    tabChoice=2;
  }
  int x_g=sensorVal[0]+sensorVal[1]+sensorVal[2];
  int x_d=sensorVal[3]+sensorVal[4]+sensorVal[5];
  if (millis()-time >=11500 && (!strcmp(sensorValChar,"111001")||(x_g>=2 && x_d>=1))&& !done){
    digitalWrite(LED_PIN,HIGH);
    runMotors(0,0);
    delay(100);
    //dawrou 9odeem (chwaya imin)
    runMotors( 100, 90);
    delay(700);
    runMotors( 0, 0);
    delay(100);
    //dawrou ysar
    runMotors( 0, 120);
    delay(600);
    runMotors( 0, 0);
    delay(100);
    runMotors( 90, 90);
    delay(200);
    runMotors( 0, 120);
    delay(500);
    done = true;
    digitalWrite(LED_PIN,LOW);
  }
  if(millis()-time>16500 && !strcmp(sensorValChar,"111111")){
    runMotors( 0, 0);
    delay(300);
    runMotors( 155, 135);
    delay(370);
    runMotors( 0, 0);
    delay(60000);
  }
  x=sensorVal[0]+sensorVal[1]+sensorVal[2]+sensorVal[3];
  if(x>=2 && millis()-time>=18000 && !west_S)
  { 
    digitalWrite(LED_PIN,HIGH);
    time_started=millis();
    west_S = true;
  }
  y=sensorVal[5]+sensorVal[4]+sensorVal[3]+sensorVal[2];
  
  if(y>=2 && west_S && millis()-time_started>700 && !west_S2)
  {
    tabChoice = 3;
    west_S2=true;
    runMotors( 110, 80);
    delay(600);
  }

}