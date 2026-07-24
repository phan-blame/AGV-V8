#include <Wire.h>
#include <LiquidCrystal_I2C.h>

//====================================================
// LCD
//====================================================
LiquidCrystal_I2C lcd(0x27, 20, 4);

//====================================================
// MOTOR LEFT PINS
//====================================================
const uint8_t PWM_LEFT   = 25;
const uint8_t DIR_LEFT   = 18;
const uint8_t EN_LEFT    = 23;
const uint8_t SPEED_LEFT = 34;
const uint8_t ALARM_LEFT = 32;

//====================================================
// MOTOR RIGHT PINS
//====================================================
const uint8_t PWM_RIGHT   = 26;
const uint8_t DIR_RIGHT   = 19;
const uint8_t EN_RIGHT    = 27;
const uint8_t SPEED_RIGHT = 35;
const uint8_t ALARM_RIGHT = 33;

//====================================================
// BUTTON PINS
//====================================================
const uint8_t BTN_MENU  = 13;
const uint8_t BTN_UP    = 12;
const uint8_t BTN_DOWN  = 14;
const uint8_t BTN_ENTER = 4;

#define OBSTACLE_PIN 16

//====================================================
// PWM CONFIG
//====================================================
#define CH_LEFT   0
#define CH_RIGHT  1
#define PWM_FREQ 10000 
#define PWM_RES  8

//====================================================
// MOTOR PARAMETER 
//====================================================
// Đổi từ 4 xuống 2 vì tốc độ thực tế đang bị nhân đôi trên Serial Monitor
const uint8_t PULSE_PER_REV = 8; 

//====================================================
// VARIABLES
//====================================================
volatile uint32_t pulseLeft = 0;
volatile uint32_t pulseRight = 0;

volatile float rpmLeftFilter = 0;
volatile float rpmRightFilter = 0;
const float FILTER_ALPHA = 0.1; 

volatile float setRPM = 1700;    
volatile float pwmLeft = 0;
volatile float pwmRight = 0;

//====================================================
// TUNED PI PARAMETERS
//====================================================
float kp = 0.08;   
float ki = 0.003;  

float errorLeft = 0;
float errorRight = 0;
float integralLeft = 0;
float integralRight = 0;

//====================================================
// LIMIT PWM
//====================================================
const int PWM_MIN = 10;  
const int PWM_MAX = 255;

//====================================================
// STATUS SIGNALS
//====================================================
volatile bool runMotor = false;
bool turnLeft=true;
volatile bool forward = true;

unsigned long lastButton = 0;
const uint16_t BUTTON_DELAY = 150;

TaskHandle_t TaskLCDHandle;
TaskHandle_t TaskControlHandle;

//====================================================
// INTERRUPTS CHỐNG NHIỄU PHẦN MỀM (DEBOUNCE THỜI GIAN THỰC)
//====================================================
volatile unsigned long lastMicrosLeft = 0;
volatile unsigned long lastMicrosRight = 0;
const unsigned long DEBOUNCE_TIME = 600; // Bỏ qua tất cả các xung nhiễu xuất hiện nhanh bất thường dưới 350 micro-giây

void IRAM_ATTR speedLeftISR() { 
    unsigned long now = micros();
    if (now - lastMicrosLeft > DEBOUNCE_TIME) {
        pulseLeft++; 
        lastMicrosLeft = now;
    }
}

void IRAM_ATTR speedRightISR() { 
    unsigned long now = micros();
    if (now - lastMicrosRight > DEBOUNCE_TIME) {
        pulseRight++; 
        lastMicrosRight = now;
    }
}

//====================================================
// DIRECTION CONTROL
//====================================================
void updateDirection() {
    if(forward) {
        digitalWrite(DIR_LEFT, HIGH);
        digitalWrite(DIR_RIGHT, LOW);
    } else {
        digitalWrite(DIR_LEFT, LOW);
        digitalWrite(DIR_RIGHT, HIGH);
    }
}

//====================================================
// BUTTON SCANNER (Core 0)
//====================================================
void checkButton() {
    if(millis() - lastButton < BUTTON_DELAY) return;

    if(digitalRead(BTN_UP) == LOW) {
        setRPM += 100;
        if(setRPM > 3000) setRPM = 3000;
        lastButton = millis();
    }
    if(digitalRead(BTN_DOWN) == LOW) {
        setRPM -= 100;
        if(setRPM < 300) setRPM = 300;
        lastButton = millis();
    }
    if(digitalRead(BTN_MENU) == LOW) {
        forward = !forward;
        updateDirection();
        lastButton = millis();
    }
    if(digitalRead(BTN_ENTER) == LOW) {
        runMotor = !runMotor;
        if(!runMotor) {
            integralLeft = 0; integralRight = 0;
            ledcWrite(CH_LEFT, 0); ledcWrite(CH_RIGHT, 0);
        }
        lastButton = millis();
    }
}

//====================================================
// DRIVER ALARM CHECK
//====================================================
void checkAlarm() {
    if(digitalRead(ALARM_LEFT) == LOW) {
        runMotor = false;
        ledcWrite(CH_LEFT, 0); ledcWrite(CH_RIGHT, 0);
        lcd.clear(); lcd.setCursor(2, 1); lcd.print("LEFT MOTOR ALRM");
        while(1);
    }
    if(digitalRead(ALARM_RIGHT) == LOW) {
        runMotor = false;
        ledcWrite(CH_LEFT, 0); ledcWrite(CH_RIGHT, 0);
        lcd.clear(); lcd.setCursor(2, 1); lcd.print("RIGHT MOTOR ALRM");
        while(1);
    }
}

//====================================================
// TASK 1: LCD (Core 0)
//====================================================
void TaskLCD(void * pvParameters) {
    lcd.init();
    lcd.backlight();
    lcd.clear();
    
    lcd.setCursor(5, 0);  lcd.print("AGV V4");
    lcd.setCursor(2, 1);  lcd.print("ANTI-NOISE V2");
    vTaskDelay(pdMS_TO_TICKS(1000));
    lcd.clear();

    for(;;) {
        checkButton(); 
        
        lcd.setCursor(0, 0);  lcd.print("L:"); lcd.print((int)rpmLeftFilter); lcd.print("   ");
        lcd.setCursor(10, 0); lcd.print("R:"); lcd.print((int)rpmRightFilter); lcd.print("   ");

        lcd.setCursor(0, 1);  lcd.print("SET:"); lcd.print((int)setRPM); lcd.print(" RPM   ");

        lcd.setCursor(0, 2);  lcd.print("PL:"); lcd.print((int)pwmLeft); lcd.print("   ");
        lcd.setCursor(10, 2); lcd.print("PR:"); lcd.print((int)pwmRight); lcd.print("   ");

        lcd.setCursor(0, 3);  lcd.print(runMotor ? "RUN " : "STOP");
        lcd.setCursor(6, 3);  lcd.print(forward ? "FWD " : "REV ");

        int balance = (int)(rpmLeftFilter - rpmRightFilter);
        lcd.setCursor(12, 3); lcd.print("B:");
        if(balance >= 0) lcd.print("+");
        lcd.print(balance); lcd.print("   ");

        vTaskDelay(pdMS_TO_TICKS(150)); 
    }
}

//====================================================
// TASK 2: PI CONTROL (Core 1)
//====================================================
void TaskControl(void * pvParameters) {
    const TickType_t xDelay = pdMS_TO_TICKS(100); 
    TickType_t xLastWakeTime = xTaskGetTickCount();

    float rpmLeftRaw = 0;
    float rpmRightRaw = 0;
    const float Ts = 0.1; 

    for(;;) {
        noInterrupts();
        uint32_t pL = pulseLeft;   pulseLeft = 0;
        uint32_t pR = pulseRight;  pulseRight = 0;
        interrupts();

        // Tính toán tốc độ mechanical RPM chuẩn xác
        rpmLeftRaw = (pL * 60.0) / (PULSE_PER_REV * Ts);
        rpmRightRaw = (pR * 60.0) / (PULSE_PER_REV * Ts);

        // Lọc thông thấp làm mượt
        rpmLeftFilter = FILTER_ALPHA * rpmLeftRaw + (1.0 - FILTER_ALPHA) * rpmLeftFilter;
        rpmRightFilter = FILTER_ALPHA * rpmRightRaw + (1.0 - FILTER_ALPHA) * rpmRightFilter;
        // Trong vòng lặp for(;;) của TaskControl, ngay sau khi tính rpmFilter:
       if (rpmLeftFilter > (setRPM + 500) || rpmRightFilter > (setRPM + 500)) {
         ledcWrite(CH_LEFT, 0); 
          ledcWrite(CH_RIGHT, 0);
         // Tạm dừng một nhịp để chờ tín hiệu ổn định lại
         vTaskDelay(pdMS_TO_TICKS(50));
         continue; 
         }

        checkAlarm();

        if(runMotor && digitalRead(OBSTACLE_PIN)==LOW)
        {
            ledcWrite(CH_LEFT,0);
            ledcWrite(CH_RIGHT,0);
            vTaskDelay(pdMS_TO_TICKS(150));

            forward=false;
            updateDirection();

            ledcWrite(CH_LEFT,130);
            ledcWrite(CH_RIGHT,130);
            vTaskDelay(pdMS_TO_TICKS(700));

            ledcWrite(CH_LEFT,0);
            ledcWrite(CH_RIGHT,0);
            vTaskDelay(pdMS_TO_TICKS(100));

            if(turnLeft){
                // rẽ trái: bánh trái dừng, bánh phải chạy
                ledcWrite(CH_LEFT,0);
                ledcWrite(CH_RIGHT,130);
            }else{
                // rẽ phải: bánh phải dừng, bánh trái chạy
                ledcWrite(CH_LEFT,130);
                ledcWrite(CH_RIGHT,0);
            }
            vTaskDelay(pdMS_TO_TICKS(450));

            ledcWrite(CH_LEFT,0);
            ledcWrite(CH_RIGHT,0);
            vTaskDelay(pdMS_TO_TICKS(100));

            forward=true;
            updateDirection();

            turnLeft=!turnLeft;
            continue;
        }

        if (!runMotor) {
            pwmLeft = 0; pwmRight = 0;
            integralLeft = 0; integralRight = 0;
            ledcWrite(CH_LEFT, 0); ledcWrite(CH_RIGHT, 0);
        } 
        else {
            errorLeft = setRPM - rpmLeftFilter;
            errorRight = setRPM - rpmRightFilter;

            integralLeft += errorLeft * Ts;
            integralRight += errorRight * Ts;

            integralLeft = constrain(integralLeft, -3000, 3000);
            integralRight = constrain(integralRight, -3000, 3000);

            // Tự động ước lượng mức nền PWM áp dựa trên Setpoint mong muốn
            float baseP = map((int)setRPM, 300, 3000, 50, 200);
            
           // PI
pwmLeft  = baseP + (kp * errorLeft)  + (ki * integralLeft);
pwmRight = baseP + (kp * errorRight) + (ki * integralRight);

// Cân bằng 2 bánh
float balance=(rpmLeftFilter-rpmRightFilter)*0.08;

balance=constrain(balance,-40,40);



pwmLeft  -= balance;
pwmRight += balance;

// Giới hạn PWM
pwmLeft  = constrain(pwmLeft,  PWM_MIN, PWM_MAX);
pwmRight = constrain(pwmRight, PWM_MIN, PWM_MAX);

            ledcWrite(CH_LEFT, (int)pwmLeft);
            ledcWrite(CH_RIGHT, (int)pwmRight);
        }
        //================ DEBUG ===================
Serial.print("OBS:");
Serial.print(digitalRead(OBSTACLE_PIN));

Serial.print("  SET:");
Serial.print((int)setRPM);

Serial.print("  L_RPM:");
Serial.print((int)rpmLeftFilter);

Serial.print("  R_RPM:");
Serial.print((int)rpmRightFilter);

Serial.print("  L_PWM:");
Serial.print((int)pwmLeft);

Serial.print("  R_PWM:");
Serial.println((int)pwmRight);

        // Output Serial Monitor
        Serial.print("SET:");    Serial.print((int)setRPM);
        Serial.print(" L_RPM:");  Serial.print((int)rpmLeftFilter);
        Serial.print(" R_RPM:");  Serial.print((int)rpmRightFilter);
        Serial.print(" L_PWM:");  Serial.print((int)pwmLeft);
        Serial.print(" R_PWM:");  Serial.println((int)pwmRight);

        vTaskDelayUntil(&xLastWakeTime, xDelay);
    }
}

//====================================================
// SETUP
//====================================================
void setup() {
    Serial.begin(115200);

    pinMode(DIR_LEFT, OUTPUT);   pinMode(EN_LEFT, OUTPUT);
    pinMode(DIR_RIGHT, OUTPUT);  pinMode(EN_RIGHT, OUTPUT);
    pinMode(SPEED_LEFT, INPUT);  pinMode(SPEED_RIGHT, INPUT);
    pinMode(ALARM_LEFT, INPUT_PULLUP);
    pinMode(ALARM_RIGHT, INPUT_PULLUP);

    pinMode(BTN_MENU, INPUT_PULLUP);  pinMode(BTN_UP, INPUT_PULLUP);
    pinMode(BTN_DOWN, INPUT_PULLUP);  pinMode(BTN_ENTER, INPUT_PULLUP);
    pinMode(OBSTACLE_PIN, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(SPEED_LEFT), speedLeftISR, RISING);
    attachInterrupt(digitalPinToInterrupt(SPEED_RIGHT), speedRightISR, RISING);

    ledcSetup(CH_LEFT, PWM_FREQ, PWM_RES);
    ledcAttachPin(PWM_LEFT, CH_LEFT);
    ledcSetup(CH_RIGHT, PWM_FREQ, PWM_RES);
    ledcAttachPin(PWM_RIGHT, CH_RIGHT);

    ledcWrite(CH_LEFT, 0);  ledcWrite(CH_RIGHT, 0);
    digitalWrite(EN_LEFT, LOW);   digitalWrite(EN_RIGHT, LOW);
    updateDirection();

    xTaskCreatePinnedToCore(TaskLCD, "TaskLCD", 4096, NULL, 1, &TaskLCDHandle, 0);     
    xTaskCreatePinnedToCore(TaskControl, "TaskControl", 4096, NULL, 2, &TaskControlHandle, 1); 
}

void loop() {
    
    vTaskDelete(NULL);
}