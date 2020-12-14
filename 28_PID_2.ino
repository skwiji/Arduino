#include <Servo.h>

/////////////////////////////
// Configurable parameters //
/////////////////////////////

// Arduino pin assignment
#define PIN_LED 9        
#define PIN_SERVO 10    
#define PIN_IR A0     

// Framework setting
#define _DIST_TARGET 255   // 목표하는 위치
#define _DIST_MIN 100   // 거리의 최소값 100mm  // 실제로 스토퍼에 끝까지 댔을때 92mm까지 이동
#define _DIST_MAX 410   // 거리의 최대값 410mm  // 실제로 스토퍼에 끝까지 댔을때 423mm까지 이동

// Distance sensor
#define _DIST_ALPHA 0.35   // 적외선 센서 EMA필터값

// Servo range
#define _DUTY_MIN 1500 // 높이 21.6cm
#define _DUTY_NEU 1665 // 높이 19.5cm
#define _DUTY_MAX 1850 // 높이 17.4cm

// Servo speed control
#define _SERVO_ANGLE 6 // 최대 가동범위에 따른 _DUTY_MAX와 _DUTY_MIN 에 해당하는 각차이(목표 서보 회전각) / 위아래 2.0degree 씩. 높이차이 중립기준 1.4cm
#define _SERVO_SPEED 1000 // 서보 속도를 degree/sec 로 설정

// Event periods
#define _INTERVAL_DIST 30   // 적외선센서 업데이트 주기
#define _INTERVAL_SERVO 20  // 서보 업데이트 주기
#define _INTERVAL_SERIAL 100  // 시리얼 플로터 갱신 속도
#define DELAY_MICROS  1500 // 필터에 넣을 샘플값을 측정하는 딜레이

// PID parameters
#define _KP 1.8  // P 이득 비율
#define _KI 1.12  // I 이득 비율
#define _KD 52.0 // D 이득 비율
#define _ITERM_MAX 28.0

// Servo instance
Servo myservo; // 서보 객체 생성

//////////////////////
// global variables //
//////////////////////

// Distance sensor
float dist_target = _DIST_TARGET; // location to send the ball
float dist_raw, dist_ema; // 거리와 노이즈 필터 적용 후 거리를 저장하기 위한 변수
float samples_num = 3;
const float coE[] = {-0.0000028, 0.0017685, 0.6657775, 30.9043937};

// Event periods
unsigned long last_sampling_time_dist, last_sampling_time_servo,
last_sampling_time_serial;  // 각 이벤트별 업데이트를 위해 측정하는 시간

bool event_dist, event_servo, event_serial; // 이벤트별 이번 루프에서 업데이트 여부

// Servo speed control
int duty_chg_max; // maximum speed, i.e., duty difference per interval (unit: us/interval)
int duty_chg_per_interval; // current speed (unit: us/interval)
int duty_chg_adjust; // duty accelration per interval during ramp up/down period (unit: us/interval^2)
int duty_target, duty_curr; // 목표duty, 현재입력할duty

// PID variables
float error_curr, error_prev, control, pterm, dterm, iterm; // PID 제어를 위한 현재 오차, 이전오차, 컨트롤, p 값, d 값, i 값 변수 선언

// ir_distansce_filter
int a, b; // unit: mm
float alpha;
float filtered_dist; // 노이즈필터를 완료한 거리측정값


void setup() {
// initialize GPIO pins for LED and attach servo 
  pinMode(PIN_LED, OUTPUT); // 핀 LED 활성화
  myservo.attach(PIN_SERVO); // 서보 구동을 위한 서보 초기화

// initialize global variables
duty_target, duty_curr = _DUTY_NEU; // duty_target, duty_curr 초기화
last_sampling_time_dist, last_sampling_time_servo, last_sampling_time_serial = 0; // 샘플링 타임 변수 초기화
dist_raw = _DIST_MIN; // dist 변수 초기화
dist_ema = 0;
pterm = iterm = dterm = 0;

// move servo to neutral position

  myservo.writeMicroseconds(_DUTY_NEU);

// initialize serial port
  Serial.begin(57600);

// convert angle speed into duty change per interval.
  duty_chg_per_interval = (_DUTY_MAX - _DUTY_MIN)/(float)(_SERVO_ANGLE) * (_SERVO_SPEED /1000.0)*_INTERVAL_SERVO;

//ir_sensor variables
  alpha = _DIST_ALPHA;

  error_prev=0;
  
}

// 전압 물리량 변환 코드
float ir_distance(void){ // return value unit: mm
    float val;
    float volt = float(analogRead(PIN_IR));
    val = ((6762.0/(volt - 9.0)) - 4.0) * 10.0;
    return val;
}

// 적외선 센서 잡음 필터링(노이즈) 

float under_noise_filter(void){ // 아래로 떨어지는 형태의 스파이크를 제거해주는 필터
  int currReading;
  int largestReading = 0;
  for (int i = 0; i < samples_num; i++) {
    currReading = ir_distance();
    if (currReading > largestReading) { largestReading = currReading; }
    // Delay a short time before taking another reading
    delayMicroseconds(DELAY_MICROS);
  }
  return largestReading;
}

float noise_filtered_ir_distance(void){ // 아래로 떨어지는 형태의 스파이크를 제거 후, 위로 치솟는 스파이크를 제거하고 EMA필터를 적용함.
  // under_noise_filter를 통과한 값을 upper_nosie_filter에 넣어 최종 값이 나옴.
  int currReading;
  int lowestReading = 1024;
  for (int i = 0; i < samples_num; i++) {
    currReading = under_noise_filter();
    if (currReading < lowestReading) { lowestReading = currReading; }
  }
  // ema 필터 추가
  dist_ema = _DIST_ALPHA*lowestReading + (1-_DIST_ALPHA)*dist_ema;
  return dist_ema;
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
        // 적외선 센서 redading 값 거리 보정(실제거리와의 차이) return value unit: mm
        // calibrate distance reading from the IR sensor
        filtered_dist = noise_filtered_ir_distance();
        float x = filtered_dist;
        dist_raw = coE[0] * pow(x, 3) + coE[1] * pow(x, 2) + coE[2] * x + coE[3];
                       
        // PID control logic
        error_curr = _DIST_TARGET - dist_raw; // 오차 계산
        pterm = _KP * error_curr; // p값은 오차
        dterm = _KD * (error_curr - error_prev);
        iterm += _KI * error_curr;
        /*
        if(iterm > _ITERM_MAX) iterm = _ITERM_MAX;
        if(iterm < - _ITERM_MAX) iterm = - _ITERM_MAX; 
        */
        if(iterm > _ITERM_MAX) iterm = 0;
        if(iterm < - _ITERM_MAX) iterm = - 0; 
        
        control = pterm + dterm + iterm;
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
    Serial.print("IR:");
    Serial.print(dist_raw);
    Serial.print(",T:");
    Serial.print(dist_target);
    Serial.print(",P:");
    Serial.print(map(pterm,-1000,1000,510,610));
    Serial.print(",D:");
    Serial.print(map(dterm,-1000,1000,510,610));
    Serial.print(",I:");
    Serial.print(map(iterm,-1000,1000,510,610));
    Serial.print(",DTT:");
    Serial.print(map(duty_target,1000,2000,410,510));
    Serial.print(",DTC:");
    Serial.print(map(duty_curr,1000,2000,410,510));
    Serial.println(",-G:245, +G:265, m:0, M:800");

        event_serial = false;
    }
    
} // loop out
