#include <Wire.h>
#include <DFRobot_SIM7000.h>

#define PIN_TX 7
#define PIN_RX 8
SoftwareSerial mySerial(PIN_RX, PIN_TX);
DFRobot_SIM7000 sim7000(&mySerial);
char                 buff[100];

#define DEBUG
#define WINDOWING 60           // 윈도우 칸 개수 = 60개
#define MOVING_WINDOWING 1000  // 1000msec = 1sec 간격으로 윈도잉
#define NUMBER_OF_COUNTER 1
#define NUMBER_OF_PACKET 1

volatile int counts[NUMBER_OF_COUNTER] = { 0 };          // GM 카운터의 카운트를 저장할 배열
int readings[NUMBER_OF_COUNTER][WINDOWING] = { { 0 } };  // 마지막 60초 동안의 카운트를 저장할 배열
int total[NUMBER_OF_COUNTER] = { 0 };                    // 60초 동안의 총 카운트를 저장할 배열
unsigned long prevTime = 0;                              // 이전 시간을 저장할 변수
int readIndex = 0;                                       // readings 배열의 현재 인덱스
volatile bool impulseDetected = false;                   // 인터럽트 감지 플래그
volatile int lastImpulseCount = 0;                       // 마지막 튜브 임펄스 카운트 저장

void TUBE_IMPULSE1() {
  counts[0]++;
  impulseDetected = true;        // 인터럽트 감지 시 플래그 설정
  lastImpulseCount = counts[0];  // 마지막 카운트 값을 저장
}



void setup() {
  Serial.begin(9600);
  mySerial.begin(19200);
  pinMode(2, INPUT);
  attachInterrupt(digitalPinToInterrupt(2), TUBE_IMPULSE1, FALLING);  // define external interrupts

  Serial.println("Turn ON SIM7000......");
  if (sim7000.turnON()) {  //Turn ON SIM7000
    Serial.println("Turn ON !");
  }

  Serial.println("Set baud rate......");
  while (1) {
    if (sim7000.setBaudRate(19200)) {
      Serial.println("Set baud rate:19200");
      break;
    } else {
      Serial.println("Failed to set baud rate");
      delay(1000);
    }
  }

  // Check SIM card
  if (sim7000.checkSIMStatus()) {
    Serial.println("SIM card is ready.");
  } else {
    Serial.println("SIM card error.");
    while (1)
      ;
  }
  Serial.println("Init positioning function......");
  while (1) {
    if (sim7000.initPos()) {
      Serial.println("Positioning function initialized");
      break;
    } else {
      Serial.println("Fail to init positioning function");
      delay(1000);
    }
  }

  // Set network mode to LTE Cat-M1 (eNB)
  if (sim7000.setNetMode(sim7000.eNB)) {
    Serial.println("Network mode set to LTE.");
  } else {
    Serial.println("Failed to set network mode.");
    while (1)
      ;
  }

  //Attach to the network
  if (sim7000.attacthService()) {
    Serial.println("Attached to the network.");
  } else {
    Serial.println("Failed to attach to the network.");
    while (1)
      ;
  }

  if (sim7000.httpInit(DFRobot_SIM7000::eGPRS)) {
    Serial.println("HTTP initialized.");
  } else {
    Serial.println("Failed to initialize HTTP.");
    while (1)
      ;
  }
  //cnr
  // if (sim7000.openNetwork(sim7000.eTCP, "223.171.45.112", 8000)) {
  //   Serial.println("Connected to the server.");
  // } else {
  //   Serial.println("Failed to connect to the server.");
  //   while (1)
  //     ;
  // }

  //ipr400
    if (sim7000.openNetwork(sim7000.eTCP, "223.171.53.193", 6000)) {
    Serial.println("Connected to the server.");
  } else {
    Serial.println("Failed to connect to the server.");
    while (1)
      ;
  }

  for (int i = 0; i < NUMBER_OF_COUNTER; i++) {
    counts[i] = 0;
    total[i] = 0;
    for (int j = 0; j < WINDOWING; j++) {
      readings[i][j] = 0;
    }
  }
  readIndex = 0;
  prevTime = millis();
}

volatile bool dataToSendFlag = false;
unsigned long lastSendTime = 0;
bool waitingForResponse = false;

void loop() {
  // 인터럽트 감지 로직
  if (impulseDetected) {
    impulseDetected = false;
    dataToSendFlag = true;  // 데이터 전송 플래그 설정
  }

  // CPM 계산 로직
  unsigned long curTime = millis();
  if (curTime - prevTime >= MOVING_WINDOWING) {
    prevTime = curTime;
    for (int i = 0; i < NUMBER_OF_COUNTER; i++) {
      total[i] -= readings[i][readIndex];
      readings[i][readIndex] = counts[i];
      counts[i] = 0;
      total[i] += readings[i][readIndex];
    }
    readIndex++;
    if (readIndex >= WINDOWING) readIndex = 0;

    if (dataToSendFlag) {
      sendData();              // 데이터 전송 함수 호출
      dataToSendFlag = false;  // 데이터 전송 후 플래그 초기화
    }
  }
 
  Serial.println("Getting position......");
  if (sim7000.getPosition()) {  //Get the current position
    Serial.print(" Longitude : ");
    Serial.println(sim7000.getLongitude());  //Get longitude
    Serial.print(" Latitude : ");
    Serial.println(sim7000.getLatitude());  //Get latitude
  } else {
    Serial.println("Wrong data try again");
  }
}

void sendData() {
  // CPM 및 Tube Impulse 값을 문자열로 변환
  //String dataToSend = "CPM: " + String(total[0]) + ", Tube Impulse: " + String(lastImpulseCount) + "\n";
  String dataToSend = "cpm" + String(total[0])  + "lat" + sim7000.getLatitude() + "lon" + sim7000.getLongitude() + "\n";
  char charBuf[dataToSend.length() + 1];  //\n을 포함하기 위한 길이 1추가
  dataToSend.toCharArray(charBuf, sizeof(charBuf));
  sim7000.send(charBuf);

 
  // memset(recvBuf, 0, sizeof(recvBuf));

  // if (sim7000.recv(recvBuf, sizeof(recvBuf))) {
  //   // 수신 데이터 처리
  //   Serial.println("Received data:");
  //   Serial.println(recvBuf);
  // } else {
  //   Serial.println("No data received.");
  // }
}
