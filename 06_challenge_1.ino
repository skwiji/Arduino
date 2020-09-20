#define PIN_LED 7

int period;
int on_time;
int off_time;

void set_period(int new_period){

  period=new_period;
  
}

void set_duty(int duty){
  int unit_period;
  
  unit_period=period/100;
  on_time=duty*unit_period; // duty 단계에 따른 밝기 단계를 맞추기 위해 한 period 내에서 얼만큼 led를 킬 것인지
  off_time=period-on_time; // 한 period 내에서 on_time 시간을 제외한 나머지 시간

  digitalWrite(PIN_LED, 0); // led 켬
  delayMicroseconds(on_time);

  digitalWrite(PIN_LED, 1); // led 끔
  delayMicroseconds(off_time);

}

void setup() {
  pinMode(PIN_LED, OUTPUT);

  period=100;

// period 설정 10000, 1000, 100 중 사용할 코드 사용하고 나머지는 주석처리, 시작 밝기 단계 0
  set_period(10000);
  set_duty(0);
/*
  set_period(1000);
  set_duty(0);

  set_period(100);
  set_duty(0);
*/

}

void loop() {
  int current_duty;
  int loop_count;
  int count_interval;
  int period_count;
  int current_count;


    //50만us  사용
    loop_count=500000/period;
 
   count_interval=100/loop_count;
 
    if(count_interval<1){
      count_interval=1;
 
   }

    period_count=loop_count/100;
    
    if(period_count<1){
      period_count=1;
  
    }
    
    //50만  us  사용, duty 0~100 까지 늘리면서 밝기 올릴때
    for(current_duty=count_interval;current_duty<=100;current_duty+=count_interval){
      current_count=0;
      while(current_count<period_count){
        set_duty(current_duty);
        current_count++;
      }
    }
    
    //50만  us  사용, duty 100~0 까지 줄이면서 밝기 올릴때
    for(current_duty=100-count_interval;current_duty>=0;current_duty-=count_interval){
      current_count=0;
      while(current_count<period_count){
        set_duty(current_duty);
        current_count++;
      }
    }
}
