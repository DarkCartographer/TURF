/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

/* 
 * StandardRTLSAnchorMain_TWR.ino
 * 
 * This is an example master anchor in a RTLS using two way ranging ISO/IEC 24730-62_2013 messages
 */

#include <DW1000Ng.hpp>
#include <DW1000NgUtils.hpp>
#include <DW1000NgRanging.hpp>
#include <DW1000NgRTLS.hpp>
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>

// === UWB distance correction functions ===

float correctAnchorA(float m) {
    return -0.0613539660f * m * m + 0.9854327824f * m + 0.2231270241f;
}

float correctAnchorB(float m) {
    return -0.0568271523f * m * m + 0.9687394152f * m + 0.1995927684f;
}

float correctAnchorC(float m) {
    return -0.0729287388f * m * m + 1.0907117325f * m + 0.0702370832f;
}

typedef struct Position {
    double x;
    double y;
} Position;

typedef struct {
    float x;
    float y;
    uint32_t t_ms;
} PositionPacket;

uint8_t robotMAC[] = {0xD8, 0xBC, 0x38, 0xD5, 0xF3, 0x5C};


// connection pins

const uint8_t PIN_RST = 27;
const uint8_t PIN_SS = 4; // spi select pin


// Extended Unique Identifier register. 64-bit device identifier. Register file: 0x01
const char EUI[] = "AA:BB:CC:DD:EE:FF:00:01";

Position position_self = {0,0};
Position position_B = {0,3.0861};
Position position_C = {3.0861,0};

double range_self;
double range_B;
double range_C;

boolean received_B = false;

byte target_eui[8];
byte tag_shortAddress[] = {0x05, 0x00};

byte anchor_b[] = {0x02, 0x00};
uint16_t next_anchor = 2;
byte anchor_c[] = {0x03, 0x00};


String positioning;
volatile unsigned long delaySent = 0;
int16_t sentNum = 0; // todo check int type
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

frame_filtering_configuration_t ANCHOR_FRAME_FILTER_CONFIG = {
    false,
    false,
    true,
    false,
    false,
    false,
    false,
    true /* This allows blink frames */
};

constexpr char WIFI_SSID[] = "tulsiwifi";

int32_t getWiFiChannel(const char *ssid) {
  if (int32_t n = WiFi.scanNetworks()) {
      for (uint8_t i=0; i<n; i++) {
          if (!strcmp(ssid, WiFi.SSID(i).c_str())) {
              return WiFi.channel(i);
          }
      }
  }
  return 0;
}

void setup() {
    // DEBUG monitoring
    Serial.begin(250000);
    Serial.println(F("### DW1000Ng-arduino-ranging-anchorMain ###"));
    pinMode(PIN_SS, OUTPUT);
    digitalWrite(PIN_SS, HIGH);
    SPI.begin(14, 12, 13, PIN_SS);
    // initialize the driver
    #if defined(ESP8266)
    DW1000Ng::initializeNoInterrupt(PIN_SS);
    #else
    DW1000Ng::initializeNoInterrupt(PIN_SS, PIN_RST);
    #endif
    Serial.println(F("DW1000Ng initialized ..."));
    // general configuration
    DW1000Ng::applyConfiguration(DEFAULT_CONFIG);
    DW1000Ng::setTXPower(0x1F1F1F1F);
    
    DW1000Ng::enableFrameFiltering(ANCHOR_FRAME_FILTER_CONFIG);
    
    DW1000Ng::setEUI(EUI);

    DW1000Ng::setPreambleDetectionTimeout(64);
    DW1000Ng::setSfdDetectionTimeout(273);
    DW1000Ng::setReceiveFrameWaitTimeoutPeriod(5000);

    DW1000Ng::setNetworkId(RTLS_APP_ID);
    DW1000Ng::setDeviceAddress(1);
	
    DW1000Ng::setAntennaDelay(16508);
    
    Serial.println(F("Committed configuration ..."));
    // DEBUG chip info and registers pretty printed
    char msg[128];
    DW1000Ng::getPrintableDeviceIdentifier(msg);
    Serial.print("Device ID: "); Serial.println(msg);
    DW1000Ng::getPrintableExtendedUniqueIdentifier(msg);
    Serial.print("Unique ID: "); Serial.println(msg);
    DW1000Ng::getPrintableNetworkIdAndShortAddress(msg);
    Serial.print("Network ID & Device Address: "); Serial.println(msg);
    DW1000Ng::getPrintableDeviceMode(msg);
    Serial.print("Device mode: "); Serial.println(msg);

    WiFi.mode(WIFI_STA);
    int32_t channel = getWiFiChannel(WIFI_SSID);

    esp_wifi_set_promiscuous(true);
    esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
    esp_wifi_set_promiscuous(false);
    
    Serial.println("ESP-NOW initializing");
    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW init failed");
    }
    Serial.println("ESP-NOW init succeeded");

    esp_now_peer_info_t peer{};
    memcpy(peer.peer_addr, robotMAC, 6);
    peer.encrypt = false;

    esp_now_add_peer(&peer);
    Serial.println("Fully Initialized");

}




bool calculatePosition(double &x, double &y) {
    // Use corrected copies, do not overwrite the globals
    const double rA = correctAnchorA(range_self);
    const double rB = correctAnchorB(range_B);
    const double rC = correctAnchorC(range_C);

    /*
    Serial.print("Distance from A: ");
    Serial.println(rA);

    Serial.print("Distance from B: ");
    Serial.println(rB);

    Serial.print("Distance from C: ");
    Serial.println(rC);

    */

    if (rA <= 0.0 || rB <= 0.0 || rC <= 0.0) {
        return false;
    }

    // Anchor coordinates
    const double x1 = position_self.x;
    const double y1 = position_self.y;
    const double x2 = position_B.x;
    const double y2 = position_B.y;
    const double x3 = position_C.x;
    const double y3 = position_C.y;

    // Initial guess from exact algebraic solve
    {
        const double A = (-2.0 * x1) + (2.0 * x2);
        const double B = (-2.0 * y1) + (2.0 * y2);
        const double C = (rA * rA) - (rB * rB)
                       - (x1 * x1) + (x2 * x2)
                       - (y1 * y1) + (y2 * y2);

        const double D = (-2.0 * x2) + (2.0 * x3);
        const double E = (-2.0 * y2) + (2.0 * y3);
        const double F = (rB * rB) - (rC * rC)
                       - (x2 * x2) + (x3 * x3)
                       - (y2 * y2) + (y3 * y3);

        const double det1 = (E * A - B * D);
        const double det2 = (B * D - A * E);

        if (fabs(det1) < 1e-9 || fabs(det2) < 1e-9) {
            return false;
        }

        x = (C * E - F * B) / det1;
        y = (C * D - A * F) / det2;
    }

    // Inverse-distance weights based on corrected measured ranges
    const double wA = 1.0 / fmax(rA*rA, 0.05);
    const double wB = 1.0 / fmax(rB*rB, 0.05);
    const double wC = 1.0 / fmax(rC*rC, 0.05);

    // Gauss-Newton weighted least-squares refinement
    for (int iter = 0; iter < 6; ++iter) {
        const double dx1 = x - x1;
        const double dy1 = y - y1;
        const double dx2 = x - x2;
        const double dy2 = y - y2;
        const double dx3 = x - x3;
        const double dy3 = y - y3;

        const double d1 = sqrt(dx1 * dx1 + dy1 * dy1);
        const double d2 = sqrt(dx2 * dx2 + dy2 * dy2);
        const double d3 = sqrt(dx3 * dx3 + dy3 * dy3);

        if (d1 < 1e-6 || d2 < 1e-6 || d3 < 1e-6) {
            return false;
        }

        // Residuals: predicted distance - measured distance
        const double f1 = d1 - rA;
        const double f2 = d2 - rB;
        const double f3 = d3 - rC;

        // Jacobian rows
        const double j11 = dx1 / d1;
        const double j12 = dy1 / d1;
        const double j21 = dx2 / d2;
        const double j22 = dy2 / d2;
        const double j31 = dx3 / d3;
        const double j32 = dy3 / d3;

        // J^T W J
        double H00 = wA * j11 * j11 + wB * j21 * j21 + wC * j31 * j31;
        double H01 = wA * j11 * j12 + wB * j21 * j22 + wC * j31 * j32;
        double H11 = wA * j12 * j12 + wB * j22 * j22 + wC * j32 * j32;

        // Light damping for stability
        H00 += 1e-6;
        H11 += 1e-6;

        // J^T W f
        const double g0 = wA * j11 * f1 + wB * j21 * f2 + wC * j31 * f3;
        const double g1 = wA * j12 * f1 + wB * j22 * f2 + wC * j32 * f3;

        const double det = H00 * H11 - H01 * H01;
        if (fabs(det) < 1e-12) {
            return false;
        }

        // Solve (J^T W J) * step = -J^T W f
        const double stepX = -( H11 * g0 - H01 * g1) / det;
        const double stepY = -(-H01 * g0 + H00 * g1) / det;

        x += stepX;
        y += stepY;

        // Converged
        if ((stepX * stepX + stepY * stepY) < 1e-8) {
            break;
        }
    }

    return true;
}


void loop() {
    if(DW1000NgRTLS::receiveFrame()){
        size_t recv_len = DW1000Ng::getReceivedDataLength();
        byte recv_data[recv_len];
        DW1000Ng::getReceivedData(recv_data, recv_len);


        if(recv_data[0] == BLINK) {
            DW1000NgRTLS::transmitRangingInitiation(&recv_data[2], tag_shortAddress);
            DW1000NgRTLS::waitForTransmission();

            RangeAcceptResult result = DW1000NgRTLS::anchorRangeAccept(NextActivity::RANGING_CONFIRM, next_anchor);
            if(!result.success) return;
             //delay(30);
            if(result.range != 0){
            range_self = result.range;
            }
            
            /*
            String rangeString = "Range: "; rangeString += range_self; rangeString += " m";
            rangeString += "\t RX power: "; rangeString += DW1000Ng::getReceivePower(); rangeString += " dBm";
            Serial.println(rangeString);
            */

        } else if(recv_data[9] == 0x60) {
            double range = static_cast<double>(DW1000NgUtils::bytesAsValue(&recv_data[10],2) / 1000.0);

            /*
            String rangeReportString = "Range from: "; rangeReportString += recv_data[7];
            rangeReportString += " = "; rangeReportString += range;
            Serial.println(rangeReportString);
            */
            if(received_B == false && recv_data[7] == anchor_b[0] && recv_data[8] == anchor_b[1]) {
                range_B = range;
                received_B = true;
            } else if(received_B == true && recv_data[7] == anchor_c[0] && recv_data[8] == anchor_c[1]){
                range_C = range;
                double x, y;
                if (!calculatePosition(x, y)) {
                    received_B = false;
                    return;
                }

                PositionPacket p;
                p.x = x;
                p.y = y;
                
                

                positioning = "Found position - x: ";
                positioning += x; positioning +=" y: ";
                positioning += y;
                Serial.println(positioning);
                
                
                

                esp_now_send(robotMAC, (uint8_t*)&p, sizeof(p));

                received_B = false;
            } else {
                received_B = false;
            }
        }
    }

    
}