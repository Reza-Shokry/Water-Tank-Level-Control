const int pumpSpeed =11;
const int pumpOnOff1 =12;
const int pumpOnOff2 =13;
const int trigPin = 4;  
const int echoPin = 5; 
unsigned long start,end,time;
float target=9.5;
float I = 0; 
float duration, distance,lastDistance,filteredDistance,lastfilteredDistance,PID,P,D;
int analogWS;


struct KalmanFilter {
    float q;  // واریانس فرآیند
    float r;  // واریانس اندازه‌گیری
    float x;  // مقدار تخمین زده شده
    float p;  // واریانس تخمین
    float k;  // بهره کالمن
    
};

void kalmanInit(KalmanFilter &kf, float q, float r, float initial_value) {
    kf.q = q;
    kf.r = r;
    kf.x = initial_value;
    kf.p = 1.0;
    kf.k = 0.0;
}

float kalmanUpdate(KalmanFilter &kf, float measurement) {
    
    kf.p = kf.p + kf.q;

    // به روزرسانی بهره کالمن
    kf.k = kf.p / (kf.p + kf.r);

    //  به روزرسانی مقدار تخمین زده شده
    kf.x = kf.x + kf.k * (measurement - kf.x);

    // به روزرسانی واریانس تخمین
    kf.p = (1 - kf.k) * kf.p;

    return kf.x;
}

KalmanFilter kf;

void setup() {
  start = millis();
  pinMode(pumpOnOff1, OUTPUT);
  pinMode(pumpOnOff2, OUTPUT);
  pinMode(pumpSpeed, OUTPUT);
     
  // Turn off motors - Initial state
  digitalWrite(pumpOnOff1, LOW);
  digitalWrite(pumpOnOff2, LOW);
  
  pinMode(trigPin, OUTPUT);  
	pinMode(echoPin, INPUT);  
	Serial.begin(9600);  
  lastDistance = 12;
  lastfilteredDistance = 12 ;

  kalmanInit(kf, 1.0, 8, 0.0);
  target = target +0.25; 

}

void loop() {
  // Clears the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2000);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  // Calculating the distance
  distance = duration * 0.034 / 2;

  filteredDistance = kalmanUpdate(kf,distance);    

  digitalWrite(pumpOnOff1, HIGH);
  digitalWrite(pumpOnOff2,LOW);   

  end = millis();
  time = end -start;
  start = millis();  
  P = (filteredDistance-target);
  I = I + (filteredDistance-target) * time*1000;
  D = (distance - lastDistance) / time*1000;
  float  DF = ( filteredDistance - lastfilteredDistance)/time*1000;
  lastDistance = distance; 
  lastfilteredDistance = filteredDistance ; 
  PID= 97 *P + 18 *DF + 3 *I ;  
  if (PID > 255) {
  PID = 255;
  }  
  else if (PID<5) {
  digitalWrite(pumpOnOff1, LOW);
  digitalWrite(pumpOnOff2,LOW);
  PID = 0;
  }
  
  digitalWrite(pumpSpeed, int (PID)) ;

  Serial.print("DF:");
  Serial.print(DF);
  Serial.print("\t");
  Serial.print("P:");
  Serial.print(P);
  Serial.print("\t");
  Serial.print("PID:");
  Serial.print(PID);
  Serial.print("\t");
  Serial.print("distance:");
  Serial.println(distance); 
        
}
