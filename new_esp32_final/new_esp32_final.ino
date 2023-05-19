
#include <Arduino.h>
#include <WiFi.h>
#include <WebSocketsClient.h>
#include <ArduinoJson.h>

#define BTSerial Serial2
#define sz 10
// SETTINGS

const char* ssid = "214scienceedu"; //해당 지점의 와이파이 설정
const char* password = "@ESedu214"; 

const char* serverAddress = "server.greenseed.or.kr"; //연결하고자 하는 서버의 ip주소
const int serverPort = 443;//연결하고자 하는 서버의 포트번호 
const char* stompEndpoint = "/test";

// base URL for SockJS (websocket) connection
// The complete URL will look something like this(cf. http://sockjs.github.io/sockjs-protocol/sockjs-protocol-0.3.3.html#section-36):
// ws://<serverAddress>:<serverPort>/<ws_baseurl>/<3digits>/<randomstring>/websocket
// For the default config of Spring's SockJS/STOMP support, the default base URL is "/socketentry/".
const char* ws_baseurl            = "/device/"; // don't forget leading and trailing "/" !!!





String data_rv_raw; // 수신받을 센서 데이터 buffer
//String data_rv; // 수신받을 센서 데이터 buffer

WebSocketsClient webSocket;
StaticJsonDocument<200> ACK; // 수신받을 json 데이터 buffer
bool toSend = false; //데이터를 서버로 보낼지 결정할 boolean 변수, setup()에서 최종적으로 연결이 완료되면 자동으로 true로 세팅해 바로 데이터를 보낼 수 있게 함
String list[sz+1] = {"hum","tur","ph","dust","dox","co2","lux","hum_EARTH","pre","temp","mac"};
float values[sz]; //hum - 0, tur - 1, ph - 2, dust - 3, dox - 4, co2 - 5, lux - 6, hum_EARTH - 7, pre - 8, temp - 9 <-- 각 센서 값 별로 할당된 배열의 인덱스 ex. 미세먼지 센서의 값은 value[3]에 할당되어야 함
String delim = "\\\"";
int comma[sz];//블루투스에서 받은 문자열 중 ','들의 인덱스를 저장하는 배열
String sub[sz];//블루투스에서 받은 문자열을 필드별로 분해한 실수값 저장하는 배열 (이후 실수화 해줄것들)

int test[sz];
  
//ESP32의 MAC 주소
String MAC = WiFi.macAddress();     

/*
String DATE; // 측정 시각
int HUM; // 습도 센서
float TUR; // 탁도 센서
float PH; // ph 센서
float DUST; // 미세먼지 센서
float DO; // 산소포화 센서
float CO2; // 이산화탄소 센서
float LUX; // 조도 센서
int HUM_EARTH; // 토양습도 센서
int PRE; // 기압 센서
*/

//-------------------------------------------------------------------------------------CHOISW_START
  //블루투스로 부터 받은 메시지 파싱 & JSON메시지로 인코딩 & 서버로 전송
  void measure_data() { 
  
    //배열 초기화, 현재 연결되지 않은 센서의 값은 무의미한 값을 의미하는 -99999로 전송
    for(int i=0;i<sz;i++)
    {
      values[i] = -99999;
    }
    
    //전송할 데이터 형식 완성을 위한 prefix, postfix
    String st = "[\"SEND\\ndestination:/app/device\\n\\n{"; //웹에서 확인하려면 app/뒷부분을 device로 해주어야 함
    String ed = "}\\u0000\"]";    
  
    //현재 시간을 문자열 형태로 가져와야 함
    //블루투스 메시지 파싱 및 json메시지 index에 맞게 대입
     String str;
     while(BTSerial.available()) {
      //데이터 1줄 가져오기
      str=BTSerial.readStringUntil('*');
      
      //','를 기준으로 substring을 구별해 내기 위해 ','문자 별 인덱스를 comma 배열에 저장
      for(int i =0; i<sz;i++){ 
        if(i == 0)
          comma[i] = str.indexOf(","); 
        else
          comma[i] = str.indexOf(",",comma[i-1]+1);
      }
  
      //comma배열의 값(','값들의 str내 index)을 경계로 substring함수 호출, 각 성분별로 수치 저장 (여전히 수치가 String 자료형인 상태임)
      sub[0] = str.substring(0,comma[0]);
      for(int i =0; i<sz-2;i++)
      {
        sub[i+1] = str.substring(comma[i]+1,comma[i+1]);
      }
      sub[sz-1] = str.substring(comma[sz-2]+1,str.length());
      for(int i =0 ; i< sz;i++)
          values[i] = sub[i].toFloat(); //String데이터를 실수로 바꾸어줌
      if(values[0] <= 0.00)// 잘못된 데이터라면 버림: 평상시 대기의 습도값이 0.00이 나올리 없다. 
         continue;
      Serial.println("유효 데이터");
      for(int i =0;i<sz;i++)
      {
        Serial.print(values[i]);
        Serial.print(' ');
      }
      Serial.println();
    delay(100);
  
    //서버로 전송할 문자열 배열
    char temp[1024] = {0,};  
    //문자열 배열의 인덱스
    int cur = 0;              
    
    //요구되는 문자열 형식으로 encoding 시작, 센서 값만 인덱스에 맞게 넣어주면 문제 X
    for(int i=0;i<st.length();i++)
    {
      temp[cur] = st[i];
      cur++;
    }
    for(int i=0;i<sz+1;i++)
    {
      for(int j=0;j<delim.length();j++)
      {
        temp[cur] = delim[j];
        cur++;
      }
      for(int j=0;j<list[i].length();j++)
      {
        temp[cur] = list[i][j];
        cur++;
      }
      for(int j=0;j<delim.length();j++)
      {
        temp[cur] = delim[j];
        cur++;
      }
      temp[cur] = ':';
      cur++;
  
      if(i < sz)
      {
        String valueString = String(values[i]);
        /*Serial.print("values[");
        Serial.print(i);
        Serial.print("] : ");
        Serial.print(values[i]);
        Serial.print(' ');*/
        for(int j=0;j<delim.length();j++)
        {
          temp[cur] = delim[j];
          cur++;
        }
        for(int j=0;j<valueString.length();j++)
        {
          temp[cur] = valueString[j];
          cur++;
        }
        for(int j=0;j<delim.length();j++)
        {
          temp[cur] = delim[j];
          cur++;
        }
        temp[cur] = ',';
        cur++;
      }
      else
      {
        if(i == sz)
        {
          for(int j=0;j<delim.length();j++)
          {
            temp[cur] = delim[j];
            cur++;
          }
          for(int j=0;j<MAC.length();j++)
          {
            temp[cur] = MAC[j];
            cur++;
          }
          for(int j=0;j<delim.length();j++)
          {
            temp[cur] = delim[j];
            cur++;
          }
          Serial.println();
        }
        /*else if(i == sz+1)
        {
          for(int j=0;j<delim.length();j++)
          {
            temp[cur] = delim[j];
            cur++;
          }
          for(int j=0;j<DATE.length();j++)
          {
            temp[cur] = DATE[j];
            cur++;
          }
          for(int j=0;j<delim.length();j++)
          {
            temp[cur] = delim[j];
            cur++;
          }
          Serial.println();
        }*/
      }
  
    }
    for(int i=0;i<ed.length();i++)
    {
      temp[cur] = ed[i];
      cur++;
    }
    //encoding 끝
  
    //Serial.printf("%s\n",temp);
    webSocket.sendTXT(temp); //웹소켓을 통해 서버로 json데이터 전송 
    }
  }
//----------------------------------------------------------------------------------CHOISW_END

void receive_data(uint8_t * payload) //서버로 부터 넘어오는 데이터 수신
{
  ACK = payload;
  if(ACK["STATUS"] = 200) {
    Serial.println("Server received the correct data!");
    if(ACK["CTR"]=false) {
      Serial.println("Server requested to stop sending data(CTR=false)");
      while(true) {
        delay(1000); // 측정 및 전송 재시작 i)키트의 전원을 껐다 켜기 ii)ESP32의 EN 버튼 누르기
      }
    }
    else if(ACK["REFRESH"]=true) {
      Serial.println("Rebooting sensor(REFRESH=1)");
      ESP.restart();
    }
    else {
      Serial.println("Sending next data...(CTR=True)");
    }
  }
  else if(ACK["STATUS"] = 400) {
    Serial.println("Error occured!(STATUS=400)");
  }
  else {
    Serial.println("Error etc(unknown STATUS)");
  }
}

void webSocketEvent(WStype_t type, uint8_t * payload, size_t length) {

    switch (type) {
        //연결이 끊겼을 때의 처리
        case WStype_DISCONNECTED:
            Serial.printf("[WSc] Disconnected!\n");
            break;
            
        case WStype_CONNECTED:
            {
                Serial.printf("[WSc] Connected to url: %s\n",  payload);
            }
            break;
            
        //handle SockJs+STOMP protocol
        //연결 과정
        case WStype_TEXT:
            {
                String text = (char*) payload;
                Serial.printf("[WSc] get text: %s\n", payload);
                
                if (payload[0] == 'h') {

                    Serial.println("Heartbeat!");

                } else if (payload[0] == 'o') {

                    // on open connection
                    String msg = "[\"CONNECT\\naccept-version:1.1,1.0\\nheart-beat:10000,10000\\nMAC:" + MAC + "\\n\\n\\u0000\"]";
                    char connectMessage[120] = {0,};
                    for(int i=0;i<msg.length();i++)
                    {
                      connectMessage[i] = msg[i];
                    }
                    webSocket.sendTXT(connectMessage);

                } else if (text.startsWith("a[\"CONNECTED")) {

                    //subscribe to some channels
                    //최종 연결 완료
                    String msg = "[\"SUBSCRIBE\\nid:sub-0\\ndestination:/topic/" + MAC + "\\n\\n\\u0000\"]"; 
                    char subscrMessage[120] = {0,};
                    for(int i=0;i<msg.length();i++)
                    {
                      subscrMessage[i] = msg[i];
                    }
                    webSocket.sendTXT(subscrMessage);
                    delay(3000);
                    toSend = true;
                } 
                //------------------------------------------------------------CHOISW_START
                else {
                    //이 부분에서 ESP32로 전송되는 메세지 처리 ex. 보정 정보
                    int cnt =0;
                    String str;
                    while(text[cnt]!='{')
                      cnt++;
                    while(text[cnt]!='}')
                    {
                       str.concat(text[cnt]);
                       cnt++;
                    }
                    Serial.print("TEXT: str: ");
                    Serial.printf("%s\n",str);
                    for(int i=1;i<=cnt;i++)
                    {
                      Serial.print(str[i]);
                      BTSerial.write(str[i]);
                    }
                    Serial.println();
                }
                //--------------------------------------------------------------CHOISW_END
                break;
            }
        case WStype_BIN:
            break;
    }
}
void setup() {
  Serial.begin(115200);
  BTSerial.begin(115200);//블루투스 모듈과 통신 시작 (와이파이의 통신속도 115200에 맞춰 원활한 전송을 위함)
  Serial.println();

  Serial.print("Logging into WLAN: "); Serial.print(ssid); Serial.print(" ...");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
  }
  Serial.println(" success.");
  Serial.print("IP: "); Serial.println(WiFi.localIP());


  // #####################
  // create socket url according to SockJS protocol (cf. http://sockjs.github.io/sockjs-protocol/sockjs-protocol-0.3.3.html#section-36)
  // #####################
  String socketUrl = ws_baseurl;
  socketUrl += random(0, 999);
  socketUrl += "/";
  socketUrl += random(0, 999999); // should be a random string, but this works (see )
  socketUrl += "/websocket";
  // connect to websocket
  webSocket.beginSSL(serverAddress, serverPort, socketUrl);
  webSocket.setExtraHeaders(); // remove "Origin: file://" header because it breaks the connection with Spring's default websocket config
  //webSocket.setExtraHeaders("foo: I am so funny\r\nbar: not"); // some headers, in case you feel funny
  String macHeader = "mac: " + MAC;
  char mac[15];
  for(int i=0;i<macHeader.length();i++) {
    mac[i] = macHeader[i];
  }
  webSocket.setExtraHeaders(mac);// handshake 시점에 인증을 위한 mac 주소 전송
  webSocket.onEvent(webSocketEvent); 
}
//-------------------------------------------------------------CHOISW_START
void loop() {
  webSocket.loop();
  if(toSend == true)
  {
    measure_data();
    delay(100);
    //유효데이터 전송 후 블루투스 수신 버퍼 비우기
    while(Serial.available() > 0){
      Serial.read();
    }
  }
}
//--------------------------------------------------------------CHOISW_END
