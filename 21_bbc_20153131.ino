// Arduino pin assignment
#define PIN_IR A0
#define PIN_LED 9
#define PIN_SERVO 10

#define _DUTY_MIN 1430 // servo full clockwise position
#define _DUTY_NEU 1630 // servo neutral position (90 degree)
#define _DUTY_MAX 1830 // servo full counterclockwise position
#define _DIST_10 80
#define _DIST_45 339
#define  ALPHA 0.9 // EMA weight of new sample (range: 0 to 1). Setting this value to 1 effectively disables EMA filter.
#define INTERVAL 150  // servo update interval
#define SERVO_MS_DUTY 1.55  //10 duty당 소요시간 ms
#define CALI_10_45    350.0 / (_DIST_45 - _DIST_10)


#include <Servo.h>

Servo myservo;
float alpha, volt_ema;
int duty_target;
int current_duty;
int delta_duty;
unsigned move_ms;
unsigned sleep_ms ;
unsigned next_move_ms;
int move_count;
int move_dir;

  void setup() {

  pinMode(PIN_LED,OUTPUT);
  digitalWrite(PIN_LED, 1);
  myservo.attach(PIN_SERVO);
  myservo.writeMicroseconds(_DUTY_NEU);
  delay(1000*2); //중립대기
  alpha = ALPHA;
  volt_ema=0.0;
  
  current_duty=_DUTY_NEU;
  move_count=30;
  delta_duty= (_DUTY_MAX-_DUTY_NEU)/move_count;
  move_ms=delta_duty*SERVO_MS_DUTY/10;
  sleep_ms =1;
  next_move_ms=0; //시작하면 바로 대기없이 서보에 명령을 주기위해 0으로 함

  Serial.begin(57600);
}

float ir_distance(void){ // return value unit: mm
  float val;  
  float v[3];
  float max_v=-1;
  float min_v=9999;
  float volt;
  int i;
  int max_n;
  int min_n;
 
//   float volt = float(analogRead(PIN_IR));
//  다점측정으로 3점을 읽고 읽어들인 신호중 max, min 값을 제거하고 중위값을 사용
// 그 중위값으로 또 EMA 필터를 사용
//   volt = float(analogRead(PIN_IR));


   for (i=0;i<3;i++){
   v[i] = float(analogRead(PIN_IR));
   if(v[i]>max_v){
      max_v=v[i];
      max_n=i;
   }
   if(v[i]<min_v){
      min_v=v[i];
      min_n=i;
   }
  }
  
  switch(max_n){
  case 0:
    if(min_n==1){
      volt=v[2];
    }
    else{
      volt=v[1];
    }
    break;
  case 1:
    if(min_n==0){
      volt=v[2];
    }
    else{
      volt=v[0];
    }
    break;
  case 2:
    if(min_n==0){
      volt=v[1];
    }
    else{
      volt=v[0];
    }
    break;
  }
  
  
  if(volt_ema==0.0){
    volt_ema=volt;
  }
  volt_ema = alpha*volt + (1-alpha)*volt_ema;
  val = ((6762.0/(volt_ema-9.0))-4.0) * 10.0;
  return val;
} // 이중보정된 적외선 센서측정값


void loop() {
  float raw_dist; // val 값이 들어옴
  float dist_cali ; // 거리측정자 상의 실제거리와의 보정
  unsigned long now_ms;

    raw_dist = ir_distance(); // val 값이 들어옴
    dist_cali = 100 + 350.0 / (_DIST_45 - _DIST_10) * (raw_dist - _DIST_10); // 거리측정자 상의 실제거리와의 보정
    if(dist_cali<277){
      if(move_dir==1){
        //위로 움직이던 중이면 서보 모터 이동시간은 기다리지만 추가 sleep 시간을 무시한다.
      }
      move_dir=0; //아래로 움직인다.
    } else {
      if(move_dir==0){
        //아래로 움직이던 중이면 서보 모터 이동시간은 기다리지만 추가 sleep 시간을 무시한다.
      }
      move_dir=1; //위로 움직인다.
    }
     now_ms=millis();

  /*  if( now_ms< next_move_ms){
  Serial.print("wait:");
  Serial.print(now_ms);
  Serial.print(",until:");
  Serial.println(next_move_ms);
      return;
  
    }
 */
    //서보모터 이동가능한 시간이 된경우

    if(move_dir==1){
      //위로 올라감. duty 값을 작게 함.
       current_duty=current_duty-delta_duty;
    } else {
      //아래로 내려감. duty 값을 크게 함.
      current_duty=current_duty+delta_duty;
    }

  Serial.print(",delta_duty:");
  Serial.print(delta_duty);
  Serial.print(",dist_cali:");
  Serial.print(dist_cali);
  Serial.print(",current_duty:");
  Serial.println(current_duty);

    if((current_duty >= _DUTY_MIN) &&   (current_duty <= _DUTY_MAX)){
      myservo.writeMicroseconds(current_duty);  
      next_move_ms=now_ms+move_ms+sleep_ms;

    } else {
      if(current_duty < _DUTY_MIN){
        current_duty=_DUTY_MIN;
        
      }
      if(current_duty > _DUTY_MAX){
        current_duty=_DUTY_MAX;
        
      }
    } 
} // loop 종료
