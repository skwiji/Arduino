#define PIN_LED 7
unsigned int count, toggle;

void setup() {
  pinMode(PIN_LED, OUTPUT);
  Serial.begin(115200); // Initialize serial port
  while (!Serial) {
    ; // wait for serial port to connect.
 }
  Serial.println("Hello World!");
  count = toggle = 0;
  digitalWrite(PIN_LED, toggle); // turn on LED. 
}

void loop() {
  Serial.println(++count);
  delay(1000); // 켜진상태로 1초 대기
 
  //지금 카운트는 1이다.
  //1일때는 1회 켜러면 2보다 작을때
  //5일때는 5회 켜려면 6보다 작을때로 설정
  
  while(count<6){


    toggle = toggle_state(toggle); // toggle LED value. //현재 토글이 1이므로 0으로 만들어서 led  를 끄고 100 ms 대기
    digitalWrite(PIN_LED, toggle); // update LED status. // 0을 write해서 led 를 끈다.
    delay(100); // wait for 100 milliseconds //100ms 대기, 1초에 5회 깜박이려면 '꺼짐100ms + 켜짐 100ms' 를 다섯번 반복

    toggle = toggle_state(toggle); // toggle LED value. //현재 토글이 0이므로 1으로 만들어서 led  를 켜고 100 ms 대기
    digitalWrite(PIN_LED, toggle); // update LED status. // 1을 write해서 led 를 켠다.
    delay(100); // wait for 100 milliseconds //100ms 대기, 1초에 5회 깜박이려면 '꺼짐100ms + 켜짐 100ms' 를 다섯번 반복

    count++;
  }
  
//끈상태로 만든다.
  toggle = toggle_state(toggle); // toggle LED value. // 현재 토글이 1이므로 0으로 만들어서 led  를 끄고
  digitalWrite(PIN_LED, toggle); // update LED status. // 0을 write해서 led 를 끈다.
 
 
  while (1) {  
    ;
    } 
}

int toggle_state(int toggle) {
  
  return (! toggle);
  
}
