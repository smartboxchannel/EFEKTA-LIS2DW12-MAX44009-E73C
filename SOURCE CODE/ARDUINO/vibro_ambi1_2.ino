// SDK PORT
extern "C" {
#include "app_gpiote.h"
#include "nrf_gpio.h"
}
#define APP_GPIOTE_MAX_USERS 1
#include <LIS2DW12Sensor.h>
#include <MAX44009.h>
//#define MY_DEBUG
#define MY_RADIO_NRF5_ESB
#define MY_DISABLED_SERIAL
int16_t mtwr;
#define MY_TRANSPORT_WAIT_READY_MS (mtwr)
#define MY_NRF5_ESB_PA_LEVEL (NRF5_PA_MAX)
#include <MySensors.h>
#define SN "LUX & VIBRO SENS"
#define SV "1.0"
#define V_SENS_CHILD_ID 1
#define LUX_SENS_CHILD_ID 2
#define WPM_SENS_CHILD_ID 3
#define INTERVAL_R_LUX_CHILD_ID 220
#define LEVEL_SENSIV_V_SENS_CHILD_ID 230
#define ENABLE_WPM_SENS_CHILD_ID 240
#define SIGNAL_Q_ID 253
#define TEMP_CHILD_ID 254 //for any tests
#include <MySensors.h>
MyMessage vibroMsg(V_SENS_CHILD_ID, V_TRIPPED);
MyMessage brightMsg(LUX_SENS_CHILD_ID, V_LEVEL);
MyMessage wpmMsg(WPM_SENS_CHILD_ID, V_LEVEL);
MyMessage conf_wpmMsg(ENABLE_WPM_SENS_CHILD_ID, V_VAR1);
MyMessage conf_vsensMsg(LEVEL_SENSIV_V_SENS_CHILD_ID, V_VAR1);
MyMessage conf_interv_rluxMsg(INTERVAL_R_LUX_CHILD_ID, V_VAR1);
MyMessage tempMsg(TEMP_CHILD_ID, V_VAR1); //for any tests
bool nosleep = 0;
bool button_flag = 0;
bool configMode = 0;
bool wpm_enable = 0;
bool onoff = 1;
bool flag_update_transport_param;
bool flag_sendRoute_parent;
bool flag_no_present;
bool flag_nogateway_mode;
bool flag_find_parent_process;
bool flag_fcount;
bool Ack_TL;
bool Ack_FP;
bool PRESENT_ACK;
byte conf_vibro_set = 1;
byte interval_reading_lux = 10;
byte err_delivery_beat;
byte problem_mode_count;
uint8_t  countbatt = 0;
uint8_t batt_cap;
uint8_t old_batt_cap = 100;
//unsigned long BATT_TIME = 43200000; //12 hours
uint32_t BATT_TIME = 7200000; //12 hours
uint32_t SLEEP_TIME_TEMP = 60000; //1 minute
uint32_t SLEEP_TIME;
uint32_t C_BATT_TIME;
uint32_t oldmillis;
uint32_t newmillis;
uint32_t previousMillis;
uint32_t lightMillisR;
uint32_t configMillis;
uint32_t interrupt_time;
uint32_t SLEEP_TIME_W;
uint32_t axel_time;
int16_t result;
int16_t brightness;
int16_t lastbrightness;
int16_t brightThreshold = 25;
int16_t myid;
int16_t mypar;
int16_t old_mypar = -1;
int16_t master_id;
float Wpm;
float ODR_1Hz6_LP_ONLY = 1.6f;
float ODR_12Hz5 = 12.5f;
float ODR_25Hz = 25.0f;
float ODR_50Hz = 50.0f;
float ODR_100Hz = 100.0f;
float ODR_200Hz = 200.0f;
bool vibro = 1;
static app_gpiote_user_id_t m_gpiote_user_id;
uint32_t PIN_BUTTON1_MASK;
uint32_t AXEL_INT1_MASK;
volatile byte axelInt1Status = 0;
volatile byte buttInt1Status = 0;
uint16_t batteryVoltage;
int16_t linkQuality;
int16_t old_linkQuality;
LIS2DW12Sensor *lis2;
MAX44009 light;



void preHwInit() {
  board_Init();
}



void before() {
  blinky(1, 1, GREEN_LED);
  wait(1000);
  nRF_Init();
  device_Conf();
  happy_init();
}



void setup() {
  interrupt_Init();
  sensors_Init();
  config_Happy_node();
}



void presentation()
{
  if (!sendSketchInfo(SN, SV)) {
    _transportSM.failedUplinkTransmissions = 0;
    sleep(1000);
    wait(50);
    if (!sendSketchInfo(SN, SV)) {
      _transportSM.failedUplinkTransmissions = 0;
    }
  }

  present(V_SENS_CHILD_ID, S_VIBRATION, "STATUS VIBRO", 1);
  wait(2500, C_PRESENTATION, S_VIBRATION);
  CORE_DEBUG(PSTR("MyS: TEST WAIT AFTER PRESENT SENSOR\n"));
  if (PRESENT_ACK == 0) {
    _transportSM.failedUplinkTransmissions = 0;
    sleep(1500);
    wait(50);
    present(V_SENS_CHILD_ID, S_VIBRATION, "STATUS VIBRO", 1);
    wait(2500, C_PRESENTATION, S_VIBRATION);
    CORE_DEBUG(PSTR("MyS: TEST WAIT AFTER PRESENT SENSOR\n"));
    _transportSM.failedUplinkTransmissions = 0;
  } else {
    PRESENT_ACK = 0;
  }

  present(LUX_SENS_CHILD_ID, S_LIGHT_LEVEL, "LUX", 1);
  wait(2500, C_PRESENTATION, S_LIGHT_LEVEL);
  CORE_DEBUG(PSTR("MyS: TEST WAIT AFTER PRESENT SENSOR\n"));
  if (PRESENT_ACK == 0) {
    _transportSM.failedUplinkTransmissions = 0;
    sleep(1500);
    wait(50);
    present(LUX_SENS_CHILD_ID, S_LIGHT_LEVEL, "LUX", 1);
    wait(2500, C_PRESENTATION, S_LIGHT_LEVEL);
    CORE_DEBUG(PSTR("MyS: TEST WAIT AFTER PRESENT SENSOR\n"));
    _transportSM.failedUplinkTransmissions = 0;
  } else {
    PRESENT_ACK = 0;
  }

  present(WPM_SENS_CHILD_ID, S_LIGHT_LEVEL, "W/M^2", 1);
  wait(2500, C_PRESENTATION, S_LIGHT_LEVEL);
  CORE_DEBUG(PSTR("MyS: TEST WAIT AFTER PRESENT SENSOR\n"));
  if (PRESENT_ACK == 0) {
    _transportSM.failedUplinkTransmissions = 0;
    sleep(1500);
    wait(50);
    present(WPM_SENS_CHILD_ID, S_LIGHT_LEVEL, "W/M^2", 1);
    wait(2500, C_PRESENTATION, S_LIGHT_LEVEL);
    CORE_DEBUG(PSTR("MyS: TEST WAIT AFTER PRESENT SENSOR\n"));
    _transportSM.failedUplinkTransmissions = 0;
  } else {
    PRESENT_ACK = 0;
  }

  present(SIGNAL_Q_ID, S_CUSTOM, "SIGNAL QUALITY", 1);
  wait(2500, C_PRESENTATION, S_CUSTOM);
  CORE_DEBUG(PSTR("MyS: TEST WAIT AFTER PRESENT SENSOR\n"));
  if (PRESENT_ACK == 0) {
    _transportSM.failedUplinkTransmissions = 0;
    sleep(1500);
    wait(50);
    present(SIGNAL_Q_ID, S_CUSTOM, "SIGNAL QUALITY", 1);
    wait(2500, C_PRESENTATION, S_CUSTOM);
    CORE_DEBUG(PSTR("MyS: TEST WAIT AFTER PRESENT SENSOR\n"));
    _transportSM.failedUplinkTransmissions = 0;
  } else {
    PRESENT_ACK = 0;
  }

  present(ENABLE_WPM_SENS_CHILD_ID, S_CUSTOM, "ON|OFF WPM", 1);
  wait(2500, C_PRESENTATION, S_CUSTOM);
  CORE_DEBUG(PSTR("MyS: TEST WAIT AFTER PRESENT SENSOR\n"));
  if (PRESENT_ACK == 0) {
    _transportSM.failedUplinkTransmissions = 0;
    sleep(1500);
    wait(50);
    present(ENABLE_WPM_SENS_CHILD_ID, S_CUSTOM, "ON|OFF WPM", 1);
    wait(2500, C_PRESENTATION, S_CUSTOM);
    CORE_DEBUG(PSTR("MyS: TEST WAIT AFTER PRESENT SENSOR\n"));
    _transportSM.failedUplinkTransmissions = 0;
  } else {
    PRESENT_ACK = 0;
  }

  present(LEVEL_SENSIV_V_SENS_CHILD_ID, S_CUSTOM, "SENS LEVEL VIBRO", 1);
  wait(2500, C_PRESENTATION, S_CUSTOM);
  CORE_DEBUG(PSTR("MyS: TEST WAIT AFTER PRESENT SENSOR\n"));
  if (PRESENT_ACK == 0) {
    _transportSM.failedUplinkTransmissions = 0;
    sleep(1500);
    wait(50);
    present(LEVEL_SENSIV_V_SENS_CHILD_ID, S_CUSTOM, "SENS LEVEL VIBRO", 1);
    wait(2500, C_PRESENTATION, S_CUSTOM);
    CORE_DEBUG(PSTR("MyS: TEST WAIT AFTER PRESENT SENSOR\n"));
    _transportSM.failedUplinkTransmissions = 0;
  } else {
    PRESENT_ACK = 0;
  }

  present(INTERVAL_R_LUX_CHILD_ID, S_CUSTOM, "INTERVAL RLUX|MIN", 1);
  wait(2500, C_PRESENTATION, S_CUSTOM);
  CORE_DEBUG(PSTR("MyS: TEST WAIT AFTER PRESENT SENSOR\n"));
  if (PRESENT_ACK == 0) {
    _transportSM.failedUplinkTransmissions = 0;
    sleep(1500);
    wait(50);
    present(INTERVAL_R_LUX_CHILD_ID, S_CUSTOM, "INTERVAL RLUX|MIN", 1);
    wait(2500, C_PRESENTATION, S_CUSTOM);
    CORE_DEBUG(PSTR("MyS: TEST WAIT AFTER PRESENT SENSOR\n"));
    _transportSM.failedUplinkTransmissions = 0;
  } else {
    PRESENT_ACK = 0;
  }

  send(conf_wpmMsg.set(wpm_enable), 1);
  wait(2500, C_SET, V_VAR1);
  CORE_DEBUG(PSTR("MyS: TEST WAIT AFTER SEND CONF SENSOR\n"));
  if (Ack_TL == 0) {
    _transportSM.failedUplinkTransmissions = 0;
    sleep(1500);
    wait(50);
    send(conf_wpmMsg.set(wpm_enable), 1);
    wait(2500, C_SET, V_VAR1);
    CORE_DEBUG(PSTR("MyS: TEST WAIT AFTER SEND CONF SENSOR\n"));
    _transportSM.failedUplinkTransmissions = 0;
  } else {
    Ack_TL = 0;
  }

  send(conf_vsensMsg.set(conf_vibro_set), 1);
  wait(2500, C_SET, V_VAR1);
  CORE_DEBUG(PSTR("MyS: TEST WAIT AFTER SEND CONF SENSOR\n"));
  if (Ack_TL == 0) {
    _transportSM.failedUplinkTransmissions = 0;
    sleep(1500);
    wait(50);
    send(conf_vsensMsg.set(conf_vibro_set), 1);
    wait(2500, C_SET, V_VAR1);
    CORE_DEBUG(PSTR("MyS: TEST WAIT AFTER SEND CONF SENSOR\n"));
    _transportSM.failedUplinkTransmissions = 0;
  } else {
    Ack_TL = 0;
  }

  send(conf_interv_rluxMsg.set(interval_reading_lux), 1);
  wait(2500, C_SET, V_VAR1);
  CORE_DEBUG(PSTR("MyS: TEST WAIT AFTER SEND CONF SENSOR\n"));
  if (Ack_TL == 0) {
    _transportSM.failedUplinkTransmissions = 0;
    sleep(1500);
    wait(50);
    send(conf_interv_rluxMsg.set(interval_reading_lux), 1);
    wait(2500, C_SET, V_VAR1);
    CORE_DEBUG(PSTR("MyS: TEST WAIT AFTER SEND CONF SENSOR\n"));
    _transportSM.failedUplinkTransmissions = 0;
  } else {
    Ack_TL = 0;
  }
}



void loop()
{
  if (flag_update_transport_param == 1) {
    update_Happy_transport();
  }
  if (flag_sendRoute_parent == 1) {
    present_only_parent();
  }

  if (isTransportReady() == true) {
    if (flag_nogateway_mode == 0) {
      if (flag_find_parent_process == 1) {
        find_parent_process();
      }
      if (configMode == 0) {
        if ((axelInt1Status == AXEL_INT1) || (buttInt1Status == PIN_BUTTON1)) {
          nosleep = 1;
          if (axelInt1Status == AXEL_INT1) {
            nosleep = 1;
            send_Axel();
            axelInt1Status = 0;
            nosleep = 0;
            newmillis = millis();
            interrupt_time = newmillis - oldmillis;
            SLEEP_TIME_W = SLEEP_TIME_W - interrupt_time;
            if (SLEEP_TIME_W < 5000) {
              SLEEP_TIME_W = SLEEP_TIME;
              send_Brigh(1);
              countbatt++;
              if (countbatt == C_BATT_TIME) {
                sendBatteryStatus(1);
                countbatt = 0;
              }
            }
          }
          if (buttInt1Status == PIN_BUTTON1) {
            if (digitalRead(PIN_BUTTON1) == 0 && button_flag == 0) {
              button_flag = 1;
              nosleep = 1;
              previousMillis = millis();
              ledsOff();
            }
            if (digitalRead(PIN_BUTTON1) == 0 && button_flag == 1) {
              if ((millis() - previousMillis > 0) && (millis() - previousMillis <= 1750)) {
                if (millis() - lightMillisR > 25) {
                  lightMillisR = millis();
                  onoff = !onoff;
                  digitalWrite(GREEN_LED, onoff);
                }
              }
              if ((millis() - previousMillis > 1750) && (millis() - previousMillis <= 2000)) {
                ledsOff();
              }
              if ((millis() - previousMillis > 2000) && (millis() - previousMillis <= 4000)) {
                if (millis() - lightMillisR > 25) {
                  lightMillisR = millis();
                  onoff = !onoff;
                  digitalWrite(BLUE_LED, onoff);
                }
              }
              if ((millis() - previousMillis > 4000) && (millis() - previousMillis <= 4250)) {
                ledsOff();
              }
              if ((millis() - previousMillis > 4250) && (millis() - previousMillis <= 6250)) {
                if (millis() - lightMillisR > 25)    {
                  lightMillisR = millis();
                  onoff = !onoff;
                  digitalWrite(RED_LED, onoff);
                }
              }
              if ((millis() - previousMillis > 6250) && (millis() - previousMillis <= 6500)) {
                ledsOff();
              }
              if ((millis() - previousMillis > 6500) && (millis() - previousMillis <= 8500)) {
                if (millis() - lightMillisR > 50)    {
                  lightMillisR = millis();
                  onoff = !onoff;
                  digitalWrite(RED_LED, onoff);
                }
              }
              if (millis() - previousMillis > 8500) {
                ledsOff();
              }
            }
            if (digitalRead(PIN_BUTTON1) == 1 && button_flag == 1) {
              if (millis() - previousMillis <= 2000)
              {
                ledsOff();
                send_Brigh(0);
                nosleep = 0;
                button_flag = 0;
                buttInt1Status = 0;
              }
              if ((millis() - previousMillis > 2000) && (millis() - previousMillis <= 4000))
              {
                ledsOff();
                configMode = 1;
                button_flag = 0;
                configMillis = millis();
              }
              if ((millis() - previousMillis > 4250) && (millis() - previousMillis <= 6250))
              {
                ledsOff();
                blinky(2, 2, RED_LED);
                button_flag = 0;
                buttInt1Status = 0;
                presentation();
                nosleep = 0;
              }
              if ((millis() - previousMillis > 6500) && (millis() - previousMillis <= 8500))
              {
                ledsOff();
                blinky(3, 3, RED_LED);
                new_device();
              }
              if (((millis() - previousMillis > 1750) && (millis() - previousMillis <= 2000)) || ((millis() - previousMillis > 4000) && (millis() - previousMillis <= 4250)) || ((millis() - previousMillis > 6250) && (millis() - previousMillis <= 6500)) || ((millis() - previousMillis > 8500)))
              {
                ledsOff();
                blinky(1, 2, GREEN_LED);
                nosleep = 0;
                button_flag = 0;
                buttInt1Status = 0;
              }
            }
          }
        } else {
          SLEEP_TIME_W = SLEEP_TIME;
          send_Brigh(1);
          countbatt++;
          if (countbatt == C_BATT_TIME) {
            sendBatteryStatus(1);
            countbatt = 0;
          }
          nosleep = 0;
        }
      } else {
        if (millis() - configMillis > 20000) {
          blinky(3, 3, GREEN_LED);
          configMode = 0;
          nosleep = 0;
          button_flag = 0;
          buttInt1Status = 0;
        }
      }
    } else {
      if (buttInt1Status == PIN_BUTTON1) {

        if (digitalRead(PIN_BUTTON1) == 0 && button_flag == 0) {
          button_flag = 1;
          nosleep = 1;
          previousMillis = millis();
          ledsOff();
        }

        if (digitalRead(PIN_BUTTON1) == 0 && button_flag == 1) {
          if ((millis() - previousMillis > 0) && (millis() - previousMillis <= 500))
          {
            ledsOff();
          }
          if ((millis() - previousMillis > 500) && (millis() - previousMillis <= 2500))
          {
            lightMillisR = millis();
            onoff = !onoff;
            digitalWrite(BLUE_LED, onoff);
          }
          if ((millis() - previousMillis > 2500) && (millis() - previousMillis <= 2750))
          {
            ledsOff();
          }
          if ((millis() - previousMillis > 2750) && (millis() - previousMillis <= 4750)) {
            if (millis() - lightMillisR > 50) {
              lightMillisR = millis();
              onoff = !onoff;
              digitalWrite(RED_LED, onoff);
            }
          }

          if (millis() - previousMillis > 4750) {
            ledsOff();
            blinky(3, 1, GREEN_LED);
            button_flag = 0;
            nosleep = 0;
            buttInt1Status = 0;
          }
        }

        if (digitalRead(PIN_BUTTON1) == 1 && button_flag == 1) {
          if (millis() - previousMillis <= 500)
          {
            ledsOff();
            button_flag = 0;
            nosleep = 0;
            buttInt1Status = 0;
          }
          if ((millis() - previousMillis > 500) && (millis() - previousMillis <= 2500))
          {
           ledsOff();
            blinky(1, 1, BLUE_LED);
            check_parent();
            button_flag = 0;
            nosleep = 0;
            buttInt1Status = 0; 
          }

          if ((millis() - previousMillis > 2500) && (millis() - previousMillis <= 2750))
          {
           ledsOff();
            button_flag = 0;
            nosleep = 0;
            buttInt1Status = 0; 
          }

          if ((millis() - previousMillis > 2750) && (millis() - previousMillis <= 4750))
          {
            ledsOff();
            blinky(3, 3, RED_LED);
            new_device();
          }

          if (millis() - previousMillis > 4750)
          {
            ledsOff();
            button_flag = 0;
            nosleep = 0;
            buttInt1Status = 0;
          }
        }
      } else {
        check_parent();
      }
    }
  }

  if (_transportSM.failureCounter > 0)
  {
    _transportConfig.parentNodeId = loadState(101);
    _transportConfig.nodeId = myid;
    _transportConfig.distanceGW = loadState(103);
    mypar = _transportConfig.parentNodeId;
    nosleep = 0;
    flag_fcount = 1;
    err_delivery_beat = 5;
    happy_node_mode();
    gateway_fail();
  }

  if (configMode == 0) {
    if (nosleep == 0) {
      oldmillis = millis();
      axelInt1Status = 0;
      buttInt1Status = 0;
      sleep(SLEEP_TIME_W, false);
      nosleep = 1;
    }
  }
}



float GetWpm()
{
  float SunLuxCoef = 0.0079;
  float Wpm_temp = 0;
  Wpm_temp = (float)brightness;
  Wpm_temp *= SunLuxCoef;
  return Wpm_temp;
}



void blinky(uint8_t pulses, uint8_t repit, uint8_t ledColor) {
  for (int x = 0; x < repit; x++) {
    if (x > 0) {
      sleep(300);
    }
    for (int i = 0; i < pulses; i++) {
      if (i > 0) {
        sleep(80);
      }
      digitalWrite(ledColor, LOW);
      sleep(20);
      digitalWrite(ledColor, HIGH);
    }
  }
}



void ledsOff() {
  digitalWrite(RED_LED, HIGH);
  digitalWrite(GREEN_LED, HIGH);
  digitalWrite(BLUE_LED, HIGH);
}



void nRF_Init() {
  NRF_POWER->DCDCEN = 1;
  NRF_NFCT->TASKS_DISABLE = 1;
  NRF_NVMC->CONFIG = 1;
  NRF_UICR->NFCPINS = 0;
  NRF_NVMC->CONFIG = 0;
  NRF_SAADC ->ENABLE = 0;
  NRF_PWM0  ->ENABLE = 0;
  NRF_PWM1  ->ENABLE = 0;
  NRF_PWM2  ->ENABLE = 0;
  NRF_TWIM1 ->ENABLE = 0;
  NRF_TWIS1 ->ENABLE = 0;
  NRF_RADIO->TXPOWER = 8;
}



void sensors_Init() {
  Wire.begin();
  wait(100);
  light.begin();
  wait(100);
  lis2 = new LIS2DW12Sensor (&Wire);
  vibro_Init();
  if (isTransportReady() == true) {
    blinky(3, 1, BLUE_LED);
    wait(200);
    blinky(3, 1, GREEN_LED);
    wait(200);
    blinky(3, 1, RED_LED);
    SLEEP_TIME_W = SLEEP_TIME;
    send_Brigh(0);
    wait(50);
    sendBatteryStatus(0);
    axel_time = millis();
  } else {
    blinky(5, 3, RED_LED);
  }
}



void vibro_Init() {
  if (conf_vibro_set == 1) {
    lis2->ODRTEMP = ODR_1Hz6_LP_ONLY;
  }
  if (conf_vibro_set == 2) {
    lis2->ODRTEMP = ODR_12Hz5;
  }
  if (conf_vibro_set == 3) {
    lis2->ODRTEMP = ODR_25Hz;
  }
  if (conf_vibro_set == 4) {
    lis2->ODRTEMP = ODR_100Hz;
  }
  if (conf_vibro_set == 5) {
    lis2->ODRTEMP = ODR_200Hz;
  }
  lis2->Enable_X();
  wait(10);
  lis2->Enable_Wake_Up_Detection();
  wait(10);
}



void board_Init() {
  pinMode(PIN_BUTTON1, INPUT);
  pinMode(AXEL_INT1, INPUT);
  pinMode(AXEL_INT2, INPUT);
  pinMode(AMBI_INT, INPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(BLUE_LED, OUTPUT);
  ledsOff();
}



void send_Axel() {
  if (millis() - axel_time >= 5000) {
    blinky(6, 1, RED_LED);
    lis2->Disable_Wake_Up_Detection();
    wait(100);
    if (_transportConfig.parentNodeId == 0) {
      if (send(vibroMsg.set(vibro))) {
        wait(100);
        err_delivery_beat = 0;
        if (flag_nogateway_mode == 1) {
          flag_nogateway_mode = 0;
          CORE_DEBUG(PSTR("MyS: NORMAL GATEWAY MODE\n"));
          err_delivery_beat = 0;
        }
      } else {
        _transportSM.failedUplinkTransmissions = 0;
        if (err_delivery_beat < 5) {
          err_delivery_beat++;
        }
        if (err_delivery_beat == 4) {
          if (flag_nogateway_mode == 0) {
            gateway_fail();
            CORE_DEBUG(PSTR("MyS: LOST GATEWAY MODE\n"));
          }
        }
      }
      lis2->Enable_Wake_Up_Detection();
      axel_time = millis();
      nosleep = 0;
    }
    if (_transportConfig.parentNodeId > 0) {
      send(vibroMsg.set(vibro), 1);
      wait(2500, C_SET, V_TRIPPED);
      if (Ack_TL == 1) {
        Ack_TL = 0;
        err_delivery_beat = 0;
        //sleep_flag = 0;
        if (flag_nogateway_mode == 1) {
          flag_nogateway_mode = 0;
          CORE_DEBUG(PSTR("MyS: NORMAL GATEWAY MODE\n"));
          err_delivery_beat = 0;
        }
      } else {
        _transportSM.failedUplinkTransmissions = 0;
        if (err_delivery_beat < 5) {
          err_delivery_beat++;
        }
        if (err_delivery_beat == 4) {
          if (flag_nogateway_mode == 0) {
            gateway_fail();
            CORE_DEBUG(PSTR("MyS: LOST GATEWAY MODE\n"));
          }
        }
      }
      lis2->Enable_Wake_Up_Detection();
      axel_time = millis();
      nosleep = 0;
    }
  } else {
    nosleep = 0;
  }
}



void send_Brigh(bool start) {
  brightness = light.get_lux() * 2;
  wait(10);
  if (start == 1) {
    if (abs(brightness - lastbrightness) >= brightThreshold) {
      if (_transportConfig.parentNodeId == 0) {
        if (send(brightMsg.set(brightness, 0))) {
          err_delivery_beat = 0;
          if (flag_nogateway_mode == 1) {
            flag_nogateway_mode = 0;
            CORE_DEBUG(PSTR("MyS: NORMAL GATEWAY MODE\n"));
            err_delivery_beat = 0;
          }
          lastbrightness = brightness;
          if (wpm_enable == 1) {
            Wpm = GetWpm();
            wait(100);
            send(wpmMsg.set(Wpm, 0));
          }
          wait(50);
          blinky(2, 2, BLUE_LED);
        } else {
          _transportSM.failedUplinkTransmissions = 0;
          if (err_delivery_beat < 5) {
            err_delivery_beat++;
          }
          if (err_delivery_beat == 4) {
            if (flag_nogateway_mode == 0) {
              gateway_fail();
              CORE_DEBUG(PSTR("MyS: LOST GATEWAY MODE\n"));
            }
          }
        }
      }
      if (_transportConfig.parentNodeId > 0) {
        send(brightMsg.set(brightness, 0), 1);
        wait(2500, C_SET, V_LEVEL);
        if (Ack_TL == 1) {
          Ack_TL = 0;
          err_delivery_beat = 0;
          if (flag_nogateway_mode == 1) {
            flag_nogateway_mode = 0;
            CORE_DEBUG(PSTR("MyS: NORMAL GATEWAY MODE\n"));
            err_delivery_beat = 0;
          }
          lastbrightness = brightness;
          if (wpm_enable == 1) {
            Wpm = GetWpm();
            wait(100);
            send(wpmMsg.set(Wpm, 0));
          }
          wait(50);
          blinky(2, 2, BLUE_LED);
        } else {
          _transportSM.failedUplinkTransmissions = 0;
          if (err_delivery_beat < 5) {
            err_delivery_beat++;
          }
          if (err_delivery_beat == 4) {
            if (flag_nogateway_mode == 0) {
              gateway_fail();
              CORE_DEBUG(PSTR("MyS: LOST GATEWAY MODE\n"));
            }
          }
        }
      }
    }
  } else {
    send(brightMsg.set(brightness, 0));
    lastbrightness = brightness;
    if (wpm_enable == 1) {
      Wpm = GetWpm();
      wait(100);
      send(wpmMsg.set(Wpm, 0));
    }
    wait(50);
    blinky(2, 2, BLUE_LED);
  }
}



void interrupt_Init() {
  //***
  //SET
  //NRF_GPIO_PIN_NOPULL
  //NRF_GPIO_PIN_PULLUP
  //NRF_GPIO_PIN_PULLDOWN
  //***
  nrf_gpio_cfg_input(PIN_BUTTON1, NRF_GPIO_PIN_PULLUP);
  nrf_gpio_cfg_input(AXEL_INT1, NRF_GPIO_PIN_NOPULL);
  APP_GPIOTE_INIT(APP_GPIOTE_MAX_USERS);
  PIN_BUTTON1_MASK = 1 << PIN_BUTTON1;
  AXEL_INT1_MASK = 1 << AXEL_INT1;
  //  app_gpiote_user_register(p_user_id, pins_low_to_high_mask, pins_high_to_low_mask, event_handler)
  app_gpiote_user_register(&m_gpiote_user_id, AXEL_INT1_MASK, PIN_BUTTON1_MASK, gpiote_event_handler);
  app_gpiote_user_enable(m_gpiote_user_id);
  axelInt1Status = 0;
  buttInt1Status = 0;
}



void gpiote_event_handler(uint32_t event_pins_low_to_high, uint32_t event_pins_high_to_low)
{
  MY_HW_RTC->CC[0] = (MY_HW_RTC->COUNTER + 2); // Taken from d0016 example code, ends the sleep delay

  if (PIN_BUTTON1_MASK & event_pins_high_to_low) {
    if ((buttInt1Status == 0) && (axelInt1Status == 0)) {
      buttInt1Status = PIN_BUTTON1;
    }
  }
  if (flag_nogateway_mode == 0) {
    if (configMode == 0) {
      if (AXEL_INT1_MASK & event_pins_low_to_high) {
        if ((axelInt1Status == 0) && (buttInt1Status == 0)) {
          axelInt1Status = AXEL_INT1;
        }
      }
    }
  }

  /***
    if ((PIN_BUTTON_MASK & event_pins_low_to_high) || (PIN_BUTTON1_MASK & event_pins_high_to_low))
  ***/
}



void device_Conf() {
  conf_vibro_set = loadState(230);
  if ((conf_vibro_set > 5) || (conf_vibro_set == 0)) {
    conf_vibro_set = 1;
    saveState(230, conf_vibro_set);
  }

  wpm_enable = loadState(240);
  if (wpm_enable > 1) {
    wpm_enable = 0;
    saveState(240, wpm_enable);
  }

  interval_reading_lux  = loadState(220);
  if (interval_reading_lux > 60) {
    interval_reading_lux = 60;
    saveState(230, interval_reading_lux);
  } else if (interval_reading_lux < 1) {
    interval_reading_lux = 1;
    saveState(230, interval_reading_lux);
  }

  SLEEP_TIME = SLEEP_TIME_TEMP * interval_reading_lux;
  C_BATT_TIME = BATT_TIME / SLEEP_TIME;
}



void sendBatteryStatus(bool start) {
  sleep(5000);
  wait(200);
  batteryVoltage = hwCPUVoltage();
  wait(10);
  batt_cap = battery_level_in_percent(batteryVoltage);
  if (start == 1) {
    if (batt_cap < old_batt_cap) {
      sendBatteryLevel(battery_level_in_percent(batteryVoltage), 1);
      wait(2500, C_INTERNAL, I_BATTERY_LEVEL);
      old_batt_cap = batt_cap;
    }
  } else {
    sendBatteryLevel(battery_level_in_percent(batteryVoltage), 1);
    wait(2500, C_INTERNAL, I_BATTERY_LEVEL);
  }

  linkQuality = calculationRxQuality();
  if (linkQuality != old_linkQuality) {
    wait(10);
    sendSignalStrength(linkQuality);
    wait(50);
    old_linkQuality = linkQuality;
  }
}



bool sendSignalStrength(const int16_t level, const bool ack)
{
  return _sendRoute(build(_msgTmp, GATEWAY_ADDRESS, SIGNAL_Q_ID, C_SET, V_VAR1,
                          ack).set(level));
}
int16_t calculationRxQuality() {
  int16_t nRFRSSI_temp = transportGetReceivingRSSI();
  int16_t nRFRSSI = map(nRFRSSI_temp, -85, -40, 0, 100);
  if (nRFRSSI < 0) {
    nRFRSSI = 0;
  }
  if (nRFRSSI > 100) {
    nRFRSSI = 100;
  }
  return nRFRSSI;
}



void happy_init() {
  //hwWriteConfig(EEPROM_NODE_ID_ADDRESS, 255); // ******************** checking the node config reset *************************

  if (hwReadConfig(EEPROM_NODE_ID_ADDRESS) == 0) {
    hwWriteConfig(EEPROM_NODE_ID_ADDRESS, 255);
  }
  if (loadState(100) == 0) {
    saveState(100, 255);
  }
  CORE_DEBUG(PSTR("EEPROM NODE ID: %d\n"), hwReadConfig(EEPROM_NODE_ID_ADDRESS));
  CORE_DEBUG(PSTR("USER MEMORY SECTOR NODE ID: %d\n"), loadState(100));

  if (hwReadConfig(EEPROM_NODE_ID_ADDRESS) == 255) {
    mtwr = 0;
  } else {
    mtwr = 10000;
    no_present();
  }
  CORE_DEBUG(PSTR("MY_TRANSPORT_WAIT_MS: %d\n"), mtwr);

}



void new_device() {
  hwWriteConfig(EEPROM_NODE_ID_ADDRESS, 255);
  saveState(100, 255);
  wdt_enable(WDTO_15MS);
}



void config_Happy_node() {
  if (mtwr == 0) {
    myid = getNodeId();
    saveState(100, myid);
    mypar = _transportConfig.parentNodeId;
    old_mypar = mypar;
    master_id = 0; // *************************** master slave mode is not initialized in this example, ..stub *******************************
    saveState(101, mypar);
    saveState(102, _transportConfig.distanceGW);
  }
  if (mtwr != 0) {
    myid = getNodeId();
    if (myid != loadState(100)) {
      saveState(100, myid);
    }
    if (isTransportReady() == true) {
      mypar = _transportConfig.parentNodeId;
      master_id = 0; // *************************** master slave mode is not initialized in this example, ..stub *******************************
      if (mypar != loadState(101)) {
        saveState(101, mypar);
      }
      if (_transportConfig.distanceGW != loadState(102)) {
        saveState(102, _transportConfig.distanceGW);
      }
      present_only_parent();
    }
    if (isTransportReady() == false)
    {
      no_present();
      flag_fcount = 1;
      err_delivery_beat = 5;
      _transportConfig.nodeId = myid;
      _transportConfig.parentNodeId = loadState(101);
      _transportConfig.distanceGW = loadState(102);
      mypar = _transportConfig.parentNodeId;
      happy_node_mode();
      gateway_fail();
    }
  }
}



void no_present() {
  _coreConfig.presentationSent = true;
  _coreConfig.nodeRegistered = true;
}



void happy_node_mode() {
  _transportSM.findingParentNode = false;
  _transportSM.transportActive = true;
  _transportSM.uplinkOk = true;
  _transportSM.pingActive = false;
  transportSwitchSM(stReady);
  _transportSM.failureCounter = 0;
}



void gateway_fail() {
  flag_nogateway_mode = 1;
  flag_update_transport_param = 0;
  SLEEP_TIME_W = SLEEP_TIME / 2;
  lis2->Disable_Wake_Up_Detection();
}



void find_parent_process() {
  flag_update_transport_param = 1;
  flag_find_parent_process = 0;
  CORE_DEBUG(PSTR("MyS: STANDART TRANSPORT MODE IS RESTORED\n"));
  err_delivery_beat = 0;
  lis2->Enable_Wake_Up_Detection();
}



void update_Happy_transport() {
  CORE_DEBUG(PSTR("MyS: UPDATE TRANSPORT CONFIGURATION\n"));
  mypar = _transportConfig.parentNodeId;
  master_id = 0; // *************************** master slave mode is not initialized in this example, ..stub *******************************
  if (mypar != loadState(101))
  {
    saveState(101, mypar);
  }
  if (_transportConfig.distanceGW != loadState(102))
  {
    saveState(102, _transportConfig.distanceGW);
  }
  present_only_parent();
  wait(50);
  nosleep = 0;
  flag_update_transport_param = 0;
}



void present_only_parent() {
  if (old_mypar != mypar) {
    CORE_DEBUG(PSTR("MyS: SEND LITTLE PRESENT:) WITH PARENT ID\n"));
    if (_sendRoute(build(_msgTmp, 0, NODE_SENSOR_ID, C_INTERNAL, 6).set(mypar))) {
      flag_sendRoute_parent = 0;
      old_mypar = mypar;
    } else {
      flag_sendRoute_parent = 1;
    }
  }
}



void check_parent() {
  _transportSM.findingParentNode = true;
  CORE_DEBUG(PSTR("MyS: SEND FIND PARENT REQUEST, WAIT RESPONSE\n"));
  _sendRoute(build(_msg, 255, NODE_SENSOR_ID, C_INTERNAL, 7).set(""));
  wait(1500, C_INTERNAL, 8);
  if (_msg.sensor == 255) {
    if (mGetCommand(_msg) == 3) {
      if (_msg.type == 8) {
        Ack_FP = 1;
        CORE_DEBUG(PSTR("MyS: PARENT RESPONSE FOUND\n"));
      }
    }
  }
  if (Ack_FP == 1) {
    CORE_DEBUG(PSTR("MyS: FIND PARENT PROCESS\n"));
    Ack_FP = 0;
    transportSwitchSM(stParent);
    flag_nogateway_mode = 0;
    flag_find_parent_process = 1;
    SLEEP_TIME_W = SLEEP_TIME;
    problem_mode_count = 0;
  } else {
    _transportSM.findingParentNode = false;
    CORE_DEBUG(PSTR("MyS: PARENT RESPONSE NOT FOUND\n"));
    _transportSM.failedUplinkTransmissions = 0;
    nosleep = 0;
    if (problem_mode_count < 24) {
      CORE_DEBUG(PSTR("PROBLEM MODE COUNTER: %d\n"), problem_mode_count);
      problem_mode_count++;
      SLEEP_TIME_W = SLEEP_TIME / 100 * 120;
    } else if (problem_mode_count == 24) {
      SLEEP_TIME_W = SLEEP_TIME * 30;
      CORE_DEBUG(PSTR("PROBLEM MODE COUNTER: %d\n"), problem_mode_count);
    }
  }
}



void receive(const MyMessage & message)
{
  if (message.sensor == ENABLE_WPM_SENS_CHILD_ID) {
    if (message.type == V_VAR1) {
      if (mGetCommand(message) == C_SET) {
        if (message.isEcho()) {
          Ack_TL = 1;
        } else {
          wpm_enable = message.getBool();
          saveState(240, wpm_enable);
          wait(10);
          send(conf_wpmMsg.set(wpm_enable));
          wait(50);
          blinky(3, 3, GREEN_LED);
          configMode = 0;
          nosleep = 0;
          button_flag = 0;
          buttInt1Status = 0;
        }
      }
    }
  }

  if (message.sensor == LEVEL_SENSIV_V_SENS_CHILD_ID) {
    if (message.type == V_VAR1) {
      if (mGetCommand(message) == C_SET) {
        if (message.isEcho()) {
          Ack_TL = 1;
        } else {
          conf_vibro_set = message.getByte();
          vibro_Init();
          saveState(230, conf_vibro_set);
          wait(10);
          send(conf_vsensMsg.set(conf_vibro_set));
          wait(50);
          blinky(3, 3, GREEN_LED);
          configMode = 0;
          nosleep = 0;
          button_flag = 0;
          buttInt1Status = 0;
        }
      }
    }
  }

  if (message.sensor == INTERVAL_R_LUX_CHILD_ID) {
    if (message.type == V_VAR1) {
      if (mGetCommand(message) == C_SET) {
        if (message.isEcho()) {
          Ack_TL = 1;
        } else {
          interval_reading_lux = message.getByte();
          SLEEP_TIME = SLEEP_TIME_TEMP * interval_reading_lux;
          C_BATT_TIME = BATT_TIME / SLEEP_TIME;
          saveState(220, interval_reading_lux);
          wait(10);
          send(conf_interv_rluxMsg.set(interval_reading_lux));
          wait(50);
          blinky(3, 3, GREEN_LED);
          configMode = 0;
          nosleep = 0;
          button_flag = 0;
          buttInt1Status = 0;
        }
      }
    }
  }

  if (message.sensor == LUX_SENS_CHILD_ID) {
    if (message.type == V_LEVEL) {
      if (mGetCommand(message) == C_SET) {
        if (message.isEcho()) {
          Ack_TL = 1;
        }
      }
    }
  }

  if (message.sensor == V_SENS_CHILD_ID) {
    if (message.type == V_TRIPPED) {
      if (mGetCommand(message) == C_SET) {
        if (message.isEcho()) {
          Ack_TL = 1;
        }
      }
    }
  }

  if (mGetCommand(message) == 0) {
    PRESENT_ACK = 1;
    CORE_DEBUG(PSTR("MyS: !!!ACK OF THE PRESENTATION IN THE FUNCTION RECEIVE RECEIVED!!!\n"));
  }
}



//send(tempMsg.set(C_BATT_TIME));  //for any tests
