#include <Arduino.h>

#include <DW1000Ng.hpp>
#include <DW1000NgUtils.hpp>
#include <DW1000NgTime.hpp>
#include <DW1000NgConstants.hpp>
#include <DW1000NgRanging.hpp>
#include <DW1000NgRTLS.hpp>

#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <WebSerial.h>
#include <esp_now.h>
#include <math.h>

#define SAMPLE_COUNT 10
#define SSID "Your_SSID"
#define PASSPHRASE "Your_Passphrase"

//initial (home) position of robot
#define START_X 0.4f
#define START_Y 0.4f

#define LEFT_IN1 18
#define LEFT_IN2 19
#define RIGHT_IN1 21
#define RIGHT_IN2 22

#define LEFT_HALL_PIN 33
#define RIGHT_HALL_PIN 25

#define US_TRIG_PIN 26
#define US_ECHO_PIN 35

static volatile bool obstacleStop = false;

// Speed limits

#define MAX_DUTY 50   // 50%

#define MAX_SPEED_MPS 0.8f
#define UWB_GATE_MARGIN_M 0.25f
#define MIN_UWB_GATE_M 0.15f

// Motion commands
#define FORWARD 0
#define BACKWARD 1
#define LEFT 2
#define RIGHT 3
#define STOP 4


#define WHEEL_BASE_M        0.219075f   // measure center-to-center wheel spacing
#define METERS_PER_PULSE    0.0008372902f  // calibrate based on wheel/sensor
#define UWB_CORRECTION_GAIN 0.35f   // 0..1, higher = trust UWB more

// Hall timing variables
volatile unsigned long lastLeftPulse = 0;
volatile unsigned long lastRightPulse = 0;
volatile unsigned long leftPeriod = 0;
volatile unsigned long rightPeriod = 0;
volatile unsigned long g_leftPulseCount = 0;
volatile unsigned long g_rightPulseCount = 0;


static constexpr uint8_t PIN_RST = 27;
static constexpr uint8_t PIN_SS  = 4;

// 64-bit EUI (DW1000)
static const char EUI[] = "AA:BB:CC:DD:EE:FF:00:00";

// -------------------- Configs --------------------
device_configuration_t DEFAULT_CONFIG = {
    false,
    true,
    false,
    true,
    false,
    SFDMode::DECAWAVE_SFD,
    Channel::CHANNEL_1,
    DataRate::RATE_850KBPS,
    PulseFrequency::FREQ_16MHZ,
    PreambleLength::LEN_256,
    PreambleCode::CODE_3
};

static frame_filtering_configuration_t TAG_FRAME_FILTER_CONFIG = {
    false,
    false,
    true,
    false,
    false,
    false,
    false,
    false
};

static sleep_configuration_t SLEEP_CONFIG = {
    false,  // onWakeUpRunADC
    false,  // onWakeUpReceive
    false,  // onWakeUpLoadEUI
    true,   // onWakeUpLoadL64Param
    true,   // preserveSleep
    true,   // enableSLP
    false,  // enableWakePIN
    true    // enableWakeSPI
};


typedef struct {
    float x;
    float y;
    uint32_t t_ms;
} PositionPacket;

typedef struct
{
    float x;
    float y;
    uint32_t t_ms;
} PositionSample;

typedef struct {
  float x;
  float y;
} waypoint;



typedef struct {
    float x_m;
    float y_m;
    uint32_t last_uwb_ms;
} RobotPose;

static RobotPose g_robot;


enum WheelDir {
    WHEEL_REV = -1,
    WHEEL_STOP = 0,
    WHEEL_FWD = 1
};

static volatile int8_t g_leftWheelDir = WHEEL_STOP;
static volatile int8_t g_rightWheelDir = WHEEL_STOP;


typedef struct {
    float uwb_x;
    float uwb_y;

    float est_x;
    float est_y;
    float est_heading;

    uint32_t last_left_count;
    uint32_t last_right_count;
    uint32_t last_update_ms;
} PositionEstimator;

static PositionEstimator g_est;

// ---------------- Hall Interrupts ----------------

void IRAM_ATTR leftHallISR() {
    unsigned long now = micros();
    leftPeriod = now - lastLeftPulse;
    lastLeftPulse = now;
    g_leftPulseCount++;
}

void IRAM_ATTR rightHallISR() {
    unsigned long now = micros();
    rightPeriod = now - lastRightPulse;
    lastRightPulse = now;
    g_rightPulseCount++;
}

AsyncWebServer server(80);

static String g_webOutput;
static SemaphoreHandle_t g_webMutex;


waypoint nav_waypoints[] = {
{0.4,0.4},{0.4,0.9},{0.4,1.4},{0.4,1.9},{0.4,2.4},
{0.6668,2.4},{0.6668,1.9},{0.6668,1.4},{0.6668,0.9},{0.6668,0.4}
};

int waypoint_index = 0;
int num_waypoints = sizeof(nav_waypoints) / sizeof(nav_waypoints[0]);



PositionSample posBuffer[SAMPLE_COUNT];
static int g_posHead = 0;   // next write index
static int g_posCount = 0;  // number of valid samples

static SemaphoreHandle_t g_posMutex;

static volatile uint32_t g_blinkRateMs = 200;

// Queue for incoming position packets from ESP-NOW callback
static QueueHandle_t g_posQueue = nullptr;

// Task handles
static TaskHandle_t g_uwbTaskHandle   = nullptr;
static TaskHandle_t g_printTaskHandle = nullptr;


static void initDw1000();
static void initEspNow();
static void printDw1000Info();

static void uwbTask(void *param);
static void printTask(void *param);


static void HeadingTask(void *param);
bool computeHeadingLS(const PositionSample *samples, int n, float &heading, float &speed);
void moveFunction(int action, int speed);

// ESP-NOW callback
static void onReceive(const esp_now_recv_info *info, const uint8_t *incomingData, int len);

// -------------------- Init Helpers --------------------
static void initDw1000() {
#if defined(ESP8266)
    DW1000Ng::initializeNoInterrupt(PIN_SS);
#else
    DW1000Ng::initializeNoInterrupt(PIN_SS, PIN_RST);
#endif
    Serial.println("DW1000Ng initialized ...");

    DW1000Ng::applyConfiguration(DEFAULT_CONFIG);
    DW1000Ng::setTXPower(0x1F1F1F1F);
    DW1000Ng::enableFrameFiltering(TAG_FRAME_FILTER_CONFIG);

    DW1000Ng::setEUI(EUI);
    DW1000Ng::setNetworkId(RTLS_APP_ID);
    DW1000Ng::setDeviceAddress(5);
    DW1000Ng::setAntennaDelay(0);

    DW1000Ng::applySleepConfiguration(SLEEP_CONFIG);

    DW1000Ng::setPreambleDetectionTimeout(15);
    DW1000Ng::setSfdDetectionTimeout(273);
    DW1000Ng::setReceiveFrameWaitTimeoutPeriod(2000);

    Serial.println(F("Committed configuration ..."));
    printDw1000Info();
}

static void printDw1000Info() {
    char msg[128];
    DW1000Ng::getPrintableDeviceIdentifier(msg);
    Serial.print("Device ID: "); Serial.println(msg);
    DW1000Ng::getPrintableExtendedUniqueIdentifier(msg);
    Serial.print("Unique ID: "); Serial.println(msg);
    DW1000Ng::getPrintableNetworkIdAndShortAddress(msg);
    Serial.print("Network ID & Device Address: "); Serial.println(msg);
    DW1000Ng::getPrintableDeviceMode(msg);
    Serial.print("Device mode: "); Serial.println(msg);
}

static void initEspNow() {
    WiFi.mode(WIFI_STA);

    if (esp_now_init() != ESP_OK) {
        Serial.println("ERROR: ESP-NOW init failed");
        // probably add code to retry
        while (true) { delay(1000); }
    }

    esp_now_register_recv_cb(onReceive);

    Serial.println("ESP-NOW initialized.");
}

static float wrapAngle(float a)
{
    while (a > PI)
        a -= 2.0f * PI;

    while (a < -PI)
        a += 2.0f * PI;

    return a;
}

float distanceToWaypoint(float x, float y, waypoint wp)
{
    float dx = wp.x - x;
    float dy = wp.y - y;
    return sqrt(dx*dx + dy*dy);
}

float desiredHeading(float x, float y, waypoint wp)
{
    float dx = wp.x - x;
    float dy = wp.y - y;
    return atan2(dy, dx);
}


bool computeWeightedPosition(float &x_out, float &y_out)
{
    xSemaphoreTake(g_posMutex, portMAX_DELAY);

    if(g_posCount <= 0)
    {
        xSemaphoreGive(g_posMutex);
        return false;
    }

    float sumW = 0.0f;
    float sumX = 0.0f;
    float sumY = 0.0f;

    for(int k = 0; k < g_posCount; k++)
    {
        int idx = (g_posHead - 1 - k + SAMPLE_COUNT) % SAMPLE_COUNT;

        float w = float(g_posCount - k);   // newest gets biggest weight

        sumW += w;
        sumX += posBuffer[idx].x * w;
        sumY += posBuffer[idx].y * w;
    }

    xSemaphoreGive(g_posMutex);

    x_out = sumX / sumW;
    y_out = sumY / sumW;
    return true;
}

void updateDeadReckoning()
{
    uint32_t leftCount, rightCount;
    int8_t leftDir, rightDir;

    noInterrupts();                     //avoid ISR updating the pulse counts
    leftCount = g_leftPulseCount;       //total number of pulses from wheel encoders since boot
    rightCount = g_rightPulseCount;
    leftDir = g_leftWheelDir;           //is the wheel spinning forward or backward
    rightDir = g_rightWheelDir;
    interrupts();                       //re enable interrupts


    int32_t dLeft = (int32_t)leftCount - (int32_t)g_est.last_left_count;            //how many pulses since last run
    int32_t dRight = (int32_t)rightCount - (int32_t)g_est.last_right_count;

    g_est.last_left_count = leftCount;                                          //update counts
    g_est.last_right_count = rightCount;

    if(dLeft == 0 && dRight == 0)                   //ignore if the wheels have not moved since last run
        return;

    float sLeft = float(dLeft) * METERS_PER_PULSE * float(leftDir);         //left wheel travel
    float sRight = float(dRight) * METERS_PER_PULSE * float(rightDir);      //right wheel travel

    

    float dCenter = 0.5f * (sLeft + sRight);            //assume the center has travelled the average of the two wheel's travel
    float dTheta = (sRight - sLeft) / WHEEL_BASE_M;     //apply differential drive thetha calculation

    float thetaMid = g_est.est_heading + 0.5f * dTheta; //assume the robot travelled with heading halfway between the last heading and new heading

    g_est.est_x += dCenter * cosf(thetaMid);        //get x component
    g_est.est_y += dCenter * sinf(thetaMid);        //get y component
    g_est.est_heading = wrapAngle(g_est.est_heading + dTheta);      //ensure angles stay between -pi and pi
}

void correctEstimateWithUWB()
{
    float uwbX, uwbY;

    //compute a weighted average of the last few UWB positions so that 
    //the most recent measurement has the most influence on result
    if(!computeWeightedPosition(uwbX, uwbY))
        return;

    g_est.uwb_x = uwbX;
    g_est.uwb_y = uwbY;

    //update the estimated positions. Gain controls how much of an effect the uwb value has
    g_est.est_x = (1.0f - UWB_CORRECTION_GAIN) * g_est.est_x + UWB_CORRECTION_GAIN * uwbX;
    g_est.est_y = (1.0f - UWB_CORRECTION_GAIN) * g_est.est_y + UWB_CORRECTION_GAIN * uwbY;
}

void initPositionEstimator()
{
    uint32_t now = millis();

    g_est.uwb_x = START_X;
    g_est.uwb_y = START_Y;

    g_est.est_x = START_X;
    g_est.est_y = START_Y;
    g_est.est_heading = 0.0f;

    g_est.last_left_count = 0;
    g_est.last_right_count = 0;
    g_est.last_update_ms = now;
}

void navigateToWaypoint(float x, float y, float heading)
{
    waypoint wp = nav_waypoints[waypoint_index];

    float dist = distanceToWaypoint(x, y, wp);

    if(dist < 0.15)   // reached waypoint
    {
        waypoint_index++;

        if(waypoint_index >= num_waypoints)
        {
            moveFunction(STOP, 0);
            WebSerial.println("all waypoints reached");
            
            return;
        }

        return;
    }

    float desired = desiredHeading(x, y, wp);
    float error = wrapAngle(desired - heading);

    float absError = fabs(error);

    if(absError < 0.15)
    {
        WebSerial.println("moving forward");
        moveFunction(FORWARD, 100);
    }
    else if(error > 0)
    {
        WebSerial.println("moving left");
        moveFunction(LEFT, 50);
    }
    else
    {
        WebSerial.println("moving right");
        moveFunction(RIGHT, 50);
    }
}

void moveFunction(int action, int speed)
{

    if(obstacleStop && action != STOP)
        return;


    int pwm = (speed * MAX_DUTY) / 100;

    switch (action)
    {
        case FORWARD:

            g_leftWheelDir = WHEEL_FWD;
            g_rightWheelDir = WHEEL_FWD;

            analogWrite(LEFT_IN1, pwm);
            analogWrite(LEFT_IN2, 0);

            analogWrite(RIGHT_IN1, pwm);
            analogWrite(RIGHT_IN2, 0);
        break;

        case BACKWARD:

            g_leftWheelDir = WHEEL_REV;
            g_rightWheelDir = WHEEL_REV;

            analogWrite(LEFT_IN1, 0);
            analogWrite(LEFT_IN2, pwm);

            analogWrite(RIGHT_IN1, 0);
            analogWrite(RIGHT_IN2, pwm);
        break;

        case LEFT:

            g_leftWheelDir = WHEEL_REV;
            g_rightWheelDir = WHEEL_FWD;

            analogWrite(LEFT_IN1, 0);
            analogWrite(LEFT_IN2, pwm);

            analogWrite(RIGHT_IN1, pwm);
            analogWrite(RIGHT_IN2, 0);
        break;

        case RIGHT:

            g_leftWheelDir = WHEEL_FWD;
            g_rightWheelDir = WHEEL_REV;

            analogWrite(LEFT_IN1, pwm);
            analogWrite(LEFT_IN2, 0);

            analogWrite(RIGHT_IN1, 0);
            analogWrite(RIGHT_IN2, pwm);
        break;

        case STOP:
        default:

            g_leftWheelDir = WHEEL_STOP;
            g_rightWheelDir = WHEEL_STOP;

            WebSerial.println("stopping");
            analogWrite(LEFT_IN1, 0);
            analogWrite(LEFT_IN2, 0);
            analogWrite(RIGHT_IN1, 0);
            analogWrite(RIGHT_IN2, 0);
        break;
    }
}

static bool acceptPositionMeasurement(const PositionPacket &p)
{
    
    float dx = p.x - g_robot.x_m;
    float dy = p.y - g_robot.y_m;

    float disp = sqrtf(dx*dx + dy*dy);

    uint32_t dt_ms = p.t_ms - g_robot.last_uwb_ms;
    float dt_s = dt_ms * 0.001f;

    float maxAllowed = MAX_SPEED_MPS * dt_s + UWB_GATE_MARGIN_M;

    if(maxAllowed < MIN_UWB_GATE_M)
        maxAllowed = MIN_UWB_GATE_M;

    return disp <= maxAllowed;
}

float readUltrasonicDistance()
{
    digitalWrite(US_TRIG_PIN, LOW);
    delayMicroseconds(3);

    digitalWrite(US_TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(US_TRIG_PIN, LOW);

    long duration = pulseIn(US_ECHO_PIN, HIGH, 30000); // timeout 30ms

    if(duration == 0){
        
        Serial.println("No echo detected");
        return -1;
    }

    float distance = duration * 0.0343 / 2.0; // cm

    return (distance / 100.0); // convert to meters
}

void prefillPositionBuffer()
{   
    uint32_t time = millis();

    for(int i=0;i<SAMPLE_COUNT;i++)
    {
        posBuffer[i].x = START_X;
        posBuffer[i].y = START_Y;
        posBuffer[i].t_ms = time;
    }

    g_robot.x_m = START_X;
    g_robot.y_m = START_Y;
    g_robot.last_uwb_ms = time;

    g_posHead = 0;
    g_posCount = SAMPLE_COUNT;
}

// -------------------- Tasks --------------------


static void uwbTask(void *param) {
    (void)param;

    for (;;) {
        // Put DW1000 in its own deep sleep mode between blinks
        DW1000Ng::deepSleep();

        // Sleep this task for blink rate
        uint32_t localBlinkMs = g_blinkRateMs; // single read of volatile
        vTaskDelay(pdMS_TO_TICKS(localBlinkMs));


        DW1000Ng::spiWakeup();
        DW1000Ng::setEUI(EUI);

        // Do localization
        RangeInfrastructureResult res = DW1000NgRTLS::tagTwrLocalize(1500);

        if (res.success) {
            // Update blink rate (with limits)
            uint32_t newRate = res.new_blink_rate;
            if (newRate < 200)   newRate = 200;    // clamp
            if (newRate > 2000) newRate = 2000;  // clamp
            g_blinkRateMs = newRate;
        }

        //WebSerial.println();
        //WebSerial.print("Heartbeat");
    }
}

// Consumes position packets that come in via ESP-NOW callback
static void PositionUpdateTask(void *param) {
    (void)param;
    
    float heading;
    float speed;

    PositionPacket p;
    for (;;) {
        // Block indefinitely waiting for data 
        if (xQueueReceive(g_posQueue, &p, portMAX_DELAY) == pdTRUE) {
            

            // Reject impossible motion
            if(!acceptPositionMeasurement(p))
            {
                // Ignore bad measurement
                continue;
            }

            g_robot.x_m = p.x;
            g_robot.y_m = p.y;
            g_robot.last_uwb_ms = p.t_ms;


            xSemaphoreTake(g_posMutex, portMAX_DELAY);

            posBuffer[g_posHead].x = p.x;
            posBuffer[g_posHead].y = p.y;
            posBuffer[g_posHead].t_ms = p.t_ms;

            int last = g_posHead;

            g_posHead = (g_posHead + 1) % SAMPLE_COUNT;
            if(g_posCount < SAMPLE_COUNT)
                g_posCount++;

            xSemaphoreGive(g_posMutex);

            correctEstimateWithUWB();

            
            WebSerial.print("Robot position: ");
            WebSerial.print(posBuffer[last].x, 3);
            WebSerial.print(", ");
            WebSerial.println(posBuffer[last].y, 3);
        }


    }
}

static void odometryTask(void *param)
{

    for(;;)
    {
        updateDeadReckoning();

        navigateToWaypoint(
            g_est.est_x,
            g_est.est_y,
            g_est.est_heading
        );

        WebSerial.print("Estimated position: ");
        WebSerial.print(g_est.est_x, 3);
        WebSerial.print(", ");
        WebSerial.print(g_est.est_y, 3);
        WebSerial.print("  heading: ");
        WebSerial.println(g_est.est_heading, 3);

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

static void ultrasonicTask(void *param)
{
    while(true)
    {
        //Serial.print("checking distance ");
        float distance = readUltrasonicDistance();

        if(distance > 0 && distance <= 1.0)
        {
            Serial.print("Obstacle distance: ");
            Serial.print(distance);
            Serial.println(" m");
        }

        if(distance > 0 && distance <= 0.5)
        {
            obstacleStop = true;

            moveFunction(STOP, 0);
        }
        else
        {
            obstacleStop = false;
        }

        vTaskDelay(pdMS_TO_TICKS(80));
    }
}



// -------------------- ESP-NOW Callback --------------------

static void onReceive(const esp_now_recv_info *info, const uint8_t *incomingData, int len) {
    (void)info;

    if (!incomingData || len < (int)sizeof(PositionPacket) || !g_posQueue) {
        return;
    }

    PositionPacket p;
    memcpy(&p, incomingData, sizeof(p));

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xQueueSendFromISR(g_posQueue, &p, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}


bool computeHeadingLS(const PositionSample *samples, int n, float &heading, float &speed)
{
    if (n < 3) return false;

    double tmean = 0;
    double xmean = 0;
    double ymean = 0;

    for (int i = 0; i < n; i++)
    {
        double t = samples[i].t_ms * 0.001;
        tmean += t;
        xmean += samples[i].x;
        ymean += samples[i].y;
    }

    tmean /= n;
    xmean /= n;
    ymean /= n;

    double denom = 0;
    double numx = 0;
    double numy = 0;

    for (int i = 0; i < n; i++)
    {
        double t = samples[i].t_ms * 0.001;
        double dt = t - tmean;

        denom += dt * dt;
        numx += dt * (samples[i].x - xmean);
        numy += dt * (samples[i].y - ymean);
    }

    if (fabs(denom) < 1e-6)
        return false;

    double vx = numx / denom;
    double vy = numy / denom;

    speed = hypot(vx, vy);

    /*
    if (speed < 0.05) // robot barely moving
        return false;
    */

    heading = atan2(vy, vx);

    return true;
}




// -------------------- Setup --------------------
void setup() {
    Serial.begin(115200);

    SPI.begin(14, 12, 13, PIN_SS);
    delay(200);

    pinMode(US_TRIG_PIN, OUTPUT);
    pinMode(US_ECHO_PIN, INPUT);
    
    pinMode(LEFT_IN1, OUTPUT);
    pinMode(LEFT_IN2, OUTPUT);
    pinMode(RIGHT_IN1, OUTPUT);
    pinMode(RIGHT_IN2, OUTPUT);

    pinMode(LEFT_HALL_PIN, INPUT_PULLUP);
    pinMode(RIGHT_HALL_PIN, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(LEFT_HALL_PIN), leftHallISR, RISING);
    attachInterrupt(digitalPinToInterrupt(RIGHT_HALL_PIN), rightHallISR, RISING);

    Serial.println(F("DW1000N RTLS Tag (FreeRTOS)"));

    g_posMutex = xSemaphoreCreateMutex();
    g_webMutex = xSemaphoreCreateMutex();

    // Create queue BEFORE registering callback (so callback has somewhere to send)
    g_posQueue = xQueueCreate(
        10,                    // depth
        sizeof(PositionPacket) // item size
    );
    if (!g_posQueue) {
        Serial.println("ERROR: Failed to create position queue");
        while (true) { delay(1000); }
    }

    for(int i=0;i<SAMPLE_COUNT;i++) {
        posBuffer[i].x = 0;
        posBuffer[i].y = 0;
        posBuffer[i].t_ms = 0;
    }
    
    g_robot.x_m = START_X;
    g_robot.y_m = START_Y;
    g_robot.last_uwb_ms = millis();

    prefillPositionBuffer();
    initPositionEstimator();

    initDw1000();
    initEspNow();

    WiFi.begin(SSID, PASSPHRASE);

    Serial.print("Connecting WiFi");

    if (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.printf("WiFi Failed!\n");
    return;
    }

    Serial.println();
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
    WebSerial.begin(&server);
    server.begin();

    // Create tasks

    xTaskCreatePinnedToCore(
        uwbTask,
        "UWBTask",
        8192,
        nullptr,
        3,                 
        &g_uwbTaskHandle,
        1                  
    );


    xTaskCreatePinnedToCore(
        PositionUpdateTask,
        "PositionUpdateTask",
        4096,
        nullptr,
        2,
        nullptr,
        1
    );


    xTaskCreatePinnedToCore(
        ultrasonicTask,
        "ultrasonicTask",
        4096,
        NULL,
        2,
        NULL,
        1
    );

    xTaskCreatePinnedToCore(
        odometryTask,
        "odometryTask",
        4096,
        nullptr,
        2,
        nullptr,
        1
    );
    

    Serial.println("Setup complete.");
}

void loop() {
    // Nothing here. FreeRTOS tasks run the program.

}