// SCS Implementation Left - BPS, Buzzer
// Flags
int rtdmode = 0, brakefault = 0, appsfault = 0, apps_scs = 0;

// Pins utilized in DAQ - 2, 12, 13, 14, 15, 16, 17, 32(can change), 33(can change), 34
// Pins avaliable - 4, 5, 18, 19, 22, 23, 25, 26, 27, 35(input only), 36(input only), 39(input only)

// Pin Definitions
#define buzzer 4
#define RTD_LED 5
#define pwm 23
#define testled 40 // ?
#define apps_led 25 //?
#define bspd_led 36 

#define apps1_pin 26
#define apps2_pin 27
#define BPS_pin 39
#define RTDB_pin 35 
#define bps_scs_pin 18
#define ECU_SCS_pin 19
#define Air_State_pin 22

// Thresholds 12-bit ADC
#define lt1 1450.0   //1.25V 
#define lt2 750.0    //0.75V
#define ht1 3102.27  	//2.5V
#define ht2 1861.36   //1.5V
#define bps_th 930.68 //0.75V

void setup() {
  Serial.begin(115200);
  pinMode(buzzer, OUTPUT);
  pinMode(RTD_LED, OUTPUT);
  pinMode(pwm, OUTPUT);
  pinMode(testled, OUTPUT);
  pinMode(apps_led, OUTPUT);
  pinMode(bspd_led, OUTPUT);
  pinMode(apps1_pin, INPUT);
  pinMode(apps2_pin, INPUT);
  analogReadResolution(12); // Set ADC resolution to 12-bit
  PWM_init();
}

void loop() {
  float a, b, apps1, apps2, apps1_analog, apps2_analog,bps;
  int rtdb, bps_scs, ECU_SCS, Air_State;

  rtdb = analogRead(RTDB_pin);
  bps = analogRead(BPS_pin);
  bps_scs = digitalRead(bps_scs_pin);
  ECU_SCS = digitalRead(ECU_SCS_pin);
  Air_State = digitalRead(Air_State_pin);



  //Print the states of each pin
  Serial.print("RTDB Pin State: ");
  Serial.println(rtdb);
  Serial.print("BPS Pin State: ");
  Serial.println(bps);
  Serial.print("ECU_SCS Pin State: ");
  Serial.println(ECU_SCS);
  Serial.print("Air_State Pin State: ");
  Serial.println(Air_State);

  Serial.print("Value of BPS=");
  Serial.println(bps*3.3/4095);
  Serial.println(bps_th*3.3/4095);
  if (bps >= bps_th && rtdb > 3000 && ECU_SCS == 1 && Air_State == 1 && rtdmode == 0 && brakefault == 0 && appsfault == 0) { // pending - bps scs check
    unsigned long startTime = millis(); 

    while (millis() - startTime <= 2000) { 
      digitalWrite(buzzer, HIGH);
    }

    digitalWrite(buzzer, LOW);
    Serial.println("Buzz... Buzz... Buzz...");
    rtdmode = 1;
  }

  if (rtdmode) { 
    digitalWrite(RTD_LED, HIGH);
    Serial.println("RTD Mode Entered");
    bps = analogRead(BPS_pin);
    apps1_analog = analogRead(apps1_pin);
    apps2_analog = analogRead(apps2_pin);
    Serial.print("Apps1: ");
    apps1 = apps1_analog * 3.6 / 4095;
    Serial.println(apps1);
    apps2 = apps2_analog * 3.6 / 4095;
    Serial.print("Apps2: ");
    Serial.println(apps2);
    a = ((float)(apps1_analog - lt1) / (float)(ht1 - lt1));
    b = ((float)(apps2_analog - lt2) / (float)(ht2 - lt2));

    Serial.print("a: ");
    Serial.println(a);
    Serial.print("b: ");
    Serial.println(b);

    analogWrite(pwm, map(apps1_analog, lt1, ht1, 0, 255));

    while (ECU_SCS == 0) {
      ECU_SCS = digitalRead(ECU_SCS_pin);
      analogWrite(pwm, 0);
      digitalWrite(RTD_LED, LOW);
      rtdmode = 0;
      Serial.println("ECU SCS Check Failed... Exiting RTD Mode");
    }

    

    while (Air_State == 0) {
      Air_State = digitalRead(Air_State_pin);
      analogWrite(pwm, 0);
      digitalWrite(RTD_LED, LOW);
      rtdmode = 0;
      Serial.println("Air State Low... Exiting RTD Mode");
    }

    // Check
    // while (bps_scs == 0) {
    //   bps_scs = digitalRead(bps_scs_pin);
    //   analogWrite(pwm, 0);
    //   digitalWrite(RTD_LED, LOW);
    //   rtdmode = 0;
    //   Serial.println("BPS SCS Check Failed... Exiting RTD Mode");
    // }
    while((apps1_analog > 3500 || apps2_analog > 3500) && rtdmode == 1 && apps_scs==0)
    {
        Serial.println("APPS SCS Check Fault...");
        analogWrite(pwm, 0);
        digitalWrite(RTD_LED, LOW);
        Serial.println("Exiting RTD Mode");
    }

    if (analogRead(apps1_analog) < 3500 && analogRead(apps2_analog) < 3500)
    {
        digitalWrite(RTD_LED, HIGH);
        Serial.println("RTD Mode Entered...");
        apps_scs=0;
        analogWrite(pwm, map(apps1_analog, lt1, ht1, 0, 255));
    }
    bps = analogRead(BPS_pin);
    // Brake fault condition
    if (a > 0.25 && bps >= bps_th && brakefault == 0 && rtdmode == 1) {
      digitalWrite(bspd_led, HIGH);
      unsigned long startTime = millis(); 
      Serial.println("BPS fault... BSPD LED On");
      brakefault = 1;
      while (brakefault == 1) {
        bps = analogRead(BPS_pin);
        apps1_analog = analogRead(apps1_pin);
        a = ((float)(apps1_analog - lt1) / (float)(ht1 - lt1));
        b = ((float)(apps2_analog - lt2) / (float)(ht2 - lt2));

        if (millis() - startTime < 2000 && ((a < 0.25) || bps < bps_th)) { // 500ms plausibility check
          digitalWrite(RTD_LED, HIGH);
          digitalWrite(bspd_led, LOW);
          brakefault = 0;
          Serial.println("RTD Mode Re-entered");
        }
        if (millis() - startTime > 2000) {
          analogWrite(pwm, 0);
          digitalWrite(RTD_LED, LOW);
          Serial.println("Exiting RTD Mode");
          brakefault = 1;
        }
        
        apps1_analog = analogRead(apps1_pin);
        apps2_analog = analogRead(apps2_pin);
        a = ((float)(apps1_analog - lt1) / (float)(ht1 - lt1));
        b = ((float)(apps2_analog - lt2) / (float)(ht2 - lt2));
        if (a > 0.05 || b > 0.05) {
          analogWrite(pwm, 0);
        }
        else if (a < 0.05 || b < 0.05) {
          analogWrite(pwm, map(apps1_analog, lt1, ht1, 0, 255));
          digitalWrite(RTD_LED, HIGH);
          digitalWrite(bspd_led, LOW);
          brakefault = 0;
          Serial.println("BSPD LED Off");
          Serial.println("RTD Mode Re-entered");
        }
      }
      }
    // APPS Fault condition
    if (abs(a - b) > 0.3 && rtdmode == 1) { // 10% implausibility
      unsigned long startTime = millis(); 
      appsfault = 1;
      while (appsfault == 1) {
        apps1_analog = analogRead(apps1_pin);
        apps2_analog = analogRead(apps2_pin);
        digitalWrite(apps_led, HIGH);
        a = ((float)(apps1_analog - lt1) / (float)(ht1 - lt1));
        b = ((float)(apps2_analog - lt2) / (float)(ht2 - lt2));
        if (millis() - startTime <= 200 && (abs(a - b) < 0.3)) { // 100ms condition
          digitalWrite(apps_led, LOW);
          Serial.println("APPS LED Off");
          appsfault = 0;
        }
        if (millis() - startTime > 2000) { // 500ms plausibility check
          analogWrite(pwm, 0);
          digitalWrite(RTD_LED, LOW);
          digitalWrite(apps_led, HIGH);
          appsfault = 1;
          rtdmode = 0;
          Serial.println("Apps fault occurred. RTD LED turned LOW. APPS LED turned HIGH");
          delay(1000);
        }
      }
    }
  }
}

void PWM_init() {
  // for motor controller
  ledcSetup(0, 5000, 8); // Use channel 0, 5000 Hz PWM, 8-bit resolution
  ledcAttachPin(pwm, 0);   
}
