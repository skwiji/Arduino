#include <Servo.h>

/////////////////////////////
// Configurable parameters //
/////////////////////////////

// Arduino pin assignment
#define PIN_LED 9        
#define PIN_SERVO 10    
#define PIN_IR A0     

// Framework setting
#define _DIST_TARGET 255    // 목표하는 위치
#define _DIST_MIN 100   // 거리의 최소값 100mm
#define _DIST_MAX 410   // 거리의 최대값 410mm

// Distance sensor
#define _DIST_ALPHA 0.8   // 적외선 센서 EMA필터값

// Servo range
#define _DUTY_MIN 1500 // 높이 21cm
#define _DUTY_NEU 1625 // 높이 19.6cm
#define _DUTY_MAX 1850 // 높이 18.2cm

// Servo speed control
#define _SERVO_ANGLE 4 // 최대 가동범위에 따른 _DUTY_MAX와 _DUTY_MIN 에 해당하는 각차이(목표 서보 회전각) / 위아래 2.0degree 씩. 높이차이 중립기준 1.4cm
#define _SERVO_SPEED 600 // 서보 속도를 degree/sec 로 설정

// Event periods
#define _INTERVAL_DIST 20   // 적외선센서 업데이트 주기
#define _INTERVAL_SERVO 20  // 서보 업데이트 주기
#define _INTERVAL_SERIAL 100  // 시리얼 플로터 갱신 속도

// PID parameters
#define _KP 1.3  // P 이득 비율
#define _KI 0.0  // I 이득 비율
#define _KD 28  // D 이득 비율

//filter
#define LENGTH 30
#define k_LENGTH 8

// Servo instance
Servo myservo; // 서보 객체 생성

//////////////////////
// global variables //
//////////////////////

// Distance sensor
float dist_target; // location to send the ball
float dist_raw, dist_ema; // 거리와 노이즈 필터 적용 후 거리를 저장하기 위한 변수
const float coE[] = {-0.0000066, 0.0054495, -0.4109230, 89.3011111};

// Event periods
unsigned long last_sampling_time_dist, last_sampling_time_servo,
last_sampling_time_serial;  // 각 이벤트별 업데이트를 위해 측정하는 시간

bool event_dist, event_servo, event_serial; // 이벤트별 이번 루프에서 업데이트 여부

// Servo speed control
int duty_chg_per_interval; // 서보속도 제어를 위한 변수 선언
int duty_target, duty_curr; // 목표duty, 현재입력할duty

// PID variables
float error_curr, error_prev, control, pterm, dterm, iterm; // PID 제어를 위한 현재 오차, 이전오차, 컨트롤, p 값, d 값, i 값 변수 선언

// ir_distansce_filter
int a, b; // unit: mm
int correction_dist, iter;
float dist_list[LENGTH], sum, alpha;
float ir_noise_filtered_dist;


void setup() {
// initialize GPIO pins for LED and attach servo 
  pinMode(PIN_LED, OUTPUT); // 핀 LED 활성화
  myservo.attach(PIN_SERVO); // 서보 구동을 위한 서보 초기화

// initialize global variables
duty_target, duty_curr = _DUTY_NEU; // duty_target, duty_curr 초기화
last_sampling_time_dist, last_sampling_time_servo, last_sampling_time_serial = 0; // 샘플링 타임 변수 초기화
dist_raw, dist_ema = _DIST_MIN; // dist 변수 초기화
pterm = iterm = dterm = 0; // pid 제어값에서 우선 d 만 사용, term값들 초기화

// move servo to neutral position

  myservo.writeMicroseconds(_DUTY_NEU);

// initialize serial port
  Serial.begin(57600);

// convert angle speed into duty change per interval.
  duty_chg_per_interval = (_DUTY_MAX - _DUTY_MIN)/(float)(_SERVO_ANGLE) * (_SERVO_SPEED /1000.0)*_INTERVAL_SERVO;

//ir_sensor variables
  a = 80;
  b = 277;
  correction_dist = 0;
  iter = 0; sum = 0;
  alpha = _DIST_ALPHA;
  
}

// 전압 물리량 변환 코드
float ir_distance(void){ // return value unit: mm
    float val;
    float volt = float(analogRead(PIN_IR));
    val = ((6762.0/(volt - 9.0)) - 4.0) * 10.0;
    return val;
}

// 적외선 센서 잡음 필터링(노이즈) 
float ir_distance_noise_filter(){ 
    sum = 0;
    iter = 0;
    while (iter < LENGTH)
    {
      dist_list[iter] = 100 + 300.0 / (b - a) * (ir_distance() - a);
      sum += dist_list[iter];
      iter++;
    }
  
    for (int i = 0; i < LENGTH-1; i++){
      for (int j = i+1; j < LENGTH; j++){
        if (dist_list[i] > dist_list[j]) {
          float tmp = dist_list[i];
          dist_list[i] = dist_list[j];
          dist_list[j] = tmp;
        }
      }
    }
    
    for (int i = 0; i < k_LENGTH; i++) {
      sum -= dist_list[i];
    }
    for (int i = 1; i <= k_LENGTH; i++) {
      sum -= dist_list[LENGTH-i];
    }
  
    float dist_cali = sum/(LENGTH-2*k_LENGTH);
    
    return alpha*dist_cali + (1-alpha)*dist_ema;
}  

void loop() {

    // indentation 수정(space 4칸)
    /////////////////////
    // Event generator //
    /////////////////////
    unsigned long time_curr = millis(); // 이벤트 업데이트 주기 계산을 위한 현재 시간
    // 이벤트 주기가 돌아올때까지 현재시간과 비교하며 기다리도록 함.
    
    if(time_curr >= last_sampling_time_dist + _INTERVAL_DIST) {
        last_sampling_time_dist += _INTERVAL_DIST;
        event_dist = true;
    }
    if(time_curr >= last_sampling_time_servo + _INTERVAL_SERVO) {
        last_sampling_time_servo += _INTERVAL_SERVO;
        event_servo = true;
    }
    if(time_curr >= last_sampling_time_serial + _INTERVAL_SERIAL) {
        last_sampling_time_serial += _INTERVAL_SERIAL;
        event_serial = true;
    }

    ////////////////////
    // Event handlers //
    ////////////////////

    if(event_dist) {
        event_dist = false; // 업데이트 대기
        // 적외선 센서 redading 값 보정(거리와의 차이) return value unit: mm
        // calibrate distance reading from the IR sensor
        float x = ir_distance_noise_filter();
        dist_raw = coE[0] * pow(x, 3) + coE[1] * pow(x, 2) + coE[2] * x + coE[3];
                       
        // PID control logic
        error_curr = _DIST_TARGET - dist_raw; // 오차 계산
        pterm = error_curr; // p값은 오차
        dterm = error_curr - error_prev;
        control = _KP * pterm + _KI * iterm +  _KD * dterm;
        error_prev = error_curr;
        
        // duty_target = f(duty_neutral, control)
         duty_target = _DUTY_NEU + ( control); // 
 
        // keep duty_target value within the range of [_DUTY_MIN, _DUTY_MAX]
        if(duty_target < _DUTY_MIN)  // 양극값 넘어가는 경우 극값으로 제한
        {
            duty_target = _DUTY_MIN;
        }

        if(duty_target > _DUTY_MAX)
        {
            duty_target = _DUTY_MAX;
        }
    }
  
    if(event_servo) {
        event_servo = false; // 업데이트 대기
        // adjust duty_curr toward duty_target by duty_chg_per_interval
        if(duty_target > duty_curr) {
            duty_curr += duty_chg_per_interval;
        if(duty_target < duty_curr ) duty_curr = duty_target;
    }
    else {
        duty_curr -= duty_chg_per_interval;
        if(duty_curr < duty_target) duty_curr = duty_target;
    }

        // update servo position
        myservo.writeMicroseconds((int)duty_curr);
        event_servo = false; // 모든 작업이 끝나면 이벤트를 다시 false로
    }
  
    if(event_serial) {
    event_serial = false; // 이벤트 주기가 왔다면 다시 false로 만들고 이벤트를 수행
    Serial.print("dist_ir:");
    Serial.print(dist_raw);
    Serial.print(",pterm:");
    Serial.print(map(pterm,-1000,1000,510,610));
    Serial.print(",dterm:");
    Serial.print(map(dterm,-1000,1000,510,610));
    Serial.print(",duty_target:");
    Serial.print(map(duty_target,1000,2000,410,510));
    Serial.print(",duty_curr:");
    Serial.print(map(duty_curr,1000,2000,410,510));
    Serial.println(",Min:100,Low:200,dist_target:255,High:310,Max:410");

        event_serial = false;
    }
    
} // loop out
