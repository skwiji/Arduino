// Arduino pin assignment
#define PIN_LED 9
#define PIN_TRIG 12
#define PIN_ECHO 13

#define MEDIAN_N 30 // 중위수필터의 최근 N개의 샘플을 저장할 때의 N 지정 N = 3, 10, 30-

// configurable parameters
#define SND_VEL 346.0 // sound velocity at 24 celsius degree (unit: m/s)
#define INTERVAL 25 // sampling interval (unit: ms)
#define _DIST_MIN 100 // minimum distance to be measured (unit: mm)
#define _DIST_MAX 300 // maximum distance to be measured (unit: mm)

// global variables
float timeout; // unit: us
float dist_min, dist_max, dist_raw, dist_med; // unit: mm
float median;
unsigned long last_sampling_time; // unit: ms
float scale; // used for pulse duration to distance conversion
float values[MEDIAN_N];
float values_sort[MEDIAN_N];

void setup() {
// initialize GPIO pins
  pinMode(PIN_LED,OUTPUT);
  pinMode(PIN_TRIG,OUTPUT);
  digitalWrite(PIN_TRIG, LOW);
  pinMode(PIN_ECHO,INPUT);

// initialize USS related variables
  dist_min = _DIST_MIN; 
  dist_max = _DIST_MAX;
  timeout = (INTERVAL / 2) * 1000.0; // precalculate pulseIn() timeout value. (unit: us)
  dist_raw = 0.0; // raw distance output from USS (unit: mm)
  scale = 0.001 * 0.5 * SND_VEL;
  median=0;

// initialize serial port
  Serial.begin(57600);

// initialize last sampling time
  last_sampling_time = 0;
}

void loop() {
// wait until next sampling time. 
// millis() returns the number of milliseconds since the program started. Will overflow after 50 days.
  if(millis() < last_sampling_time + INTERVAL) return;

// get a distance reading from the USS
  dist_raw = USS_measure(PIN_TRIG,PIN_ECHO);
   median=median_filter(dist_raw);

// output the read value to the serial port
  Serial.print("Min:0,");
  Serial.print("raw:");
  Serial.print(dist_raw);
  Serial.print(",");
  Serial.print("median:");
  Serial.print(map(median,0,400,100,500));
  Serial.print(",");
  Serial.println("Max:500");
  

// turn on the LED if the distance is between dist_min and dist_max
  if(dist_raw < dist_min || dist_raw > dist_max) {
    analogWrite(PIN_LED, 255);
  }
  else {
    analogWrite(PIN_LED, 0);
  }

// update last sampling time
  last_sampling_time += INTERVAL;
}

// get a distance reading from USS. return value is in millimeter.
float USS_measure(int TRIG, int ECHO)
{
  float reading;
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  reading = pulseIn(ECHO, HIGH, timeout) * scale; // unit: mm
  if(reading < dist_min || reading > dist_max) reading = 0.0; // return 0 when out of range.
  return reading;
}


int compare(const void *a, const void *b)    // 오름차순 비교 함수 구현
{
    float num1 = *(float *)a;    // void 포인터를 int 포인터로 변환한 뒤 역참조하여 값을 가져옴
    float num2 = *(float *)b;    // void 포인터를 int 포인터로 변환한 뒤 역참조하여 값을 가져옴

    if (num1 < num2)    // a가 b보다 작을 때는
        return -1;      // -1 반환
    
    if (num1 > num2)    // a가 b보다 클 때는
        return 1;       // 1 반환
    
    return 0;    // a와 b가 같을 때는 0 반환
}

float median_filter(float last_value)
{
  float median_value;
  int i;
  int mid_n;


  i=1;
  while(i<MEDIAN_N){
      values[i-1]=values[i];
      values_sort[i-1]=values[i];
      i++;
  }
  values[MEDIAN_N-1]=last_value;
  values_sort[MEDIAN_N-1]=last_value;

  qsort(values_sort,MEDIAN_N,sizeof(float),compare);



  if((MEDIAN_N %2)==0){
      mid_n=MEDIAN_N/2;
      median_value=(values_sort[mid_n-1]+values_sort[mid_n])/2;
  }
  else {
      mid_n=MEDIAN_N/2;
      median_value=values_sort[mid_n];
  }
  
  return median_value;
}
