/*
NTC ADS1256 example: continuously reading swipes of configured channels.
Results are shown on serial, syslog, webserver and an influx db.
Influx DB can be created with command influx -execute "create database Ads"
Extended post_influx example to actually show temperatures taken by 8 NTCs with voltage dividers.

Connection table
ESP8266 | ADS1256
========+========
GND     | GND
D5 SCLK | SCLK
D6 MISO | DOUT
D7 MOSI | DIN

Connect ADS1256 5V to ESP8266 board 5V if available (e.g. USB) or use other source.
Other pins (CS, DRDY, LED) as defined below.
*/

#include <Arduino.h>

// 8x24bit ADC
#include <Ads1256.h>

// ADS1256 settings, report interval ~= reportCount * sizeof(ads1256_ains) / SPS
uint8_t gain = 0;  // make sure max ain volts * 2^gain < 2.5V!
bool buffer = true;  // high impedance by using buffering
Ads1256Base::rate_t rate = Ads1256::SPS_10;  // 2.5 - 30k
uint8_t ads1256_ains[] = {0, 1, 2, 3, 4, 5, 6, 7};  // channels to read
const size_t reportCount = 70;  // swipes before posting (70 * 8ch / 10 sps ~= 60s)
const int32_t num_avg = 100;  // weight of old value avg vs new value

// Pins for ESP8266 and ESP32
#if defined(ESP8266)
#define PIN_CS D8
#define PIN_DRDY D1
#define PIN_DB_LED D4
#elif defined(ESP32)
#define PIN_CS 5
#define PIN_DRDY 17
#define PIN_DB_LED 2
#else
#error "No ESP8266 or ESP32, define your pins here!"
#endif

// Web Updater
#include <ESP8266HTTPUpdateServer.h>
#include <ESP8266WebServer.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiClient.h>

// Post to InfluxDB
#include <ESP8266HTTPClient.h>

#define DB_LED_ON LOW
#define DB_LED_OFF HIGH

// Infrastructure
#include <NTPClient.h>
#include <Syslog.h>
#include <WiFiManager.h>
#include <WiFiUdp.h>

// Web status page and OTA updater
#define WEBSERVER_PORT 80

class UpdateStatus : public RequestHandler {
  // Detect start of OTA updates
  public:
    UpdateStatus( ESP8266WebServer &server ) : updating(false) {
      server.addHandler(this);
    }

    bool canHandle(HTTPMethod method, const String& uri) override { 
      if( method == HTTP_POST && uri == "/update" ) {
        updating = true;
        Serial.println("Update starting");
      }
      else {
        noUpdate();
      }
      return false;
    }

    // call e.g. in ISR to check if update is running
    bool isUpdating() { return updating; }

    // call in loop to reset status of canceled update
    void noUpdate() { 
      if( updating ) {
        Serial.println("Update ended");
        updating = false;
      }
    }

  private:
    volatile bool updating;  // volatile for use in ISRs
};

ESP8266WebServer web_server(WEBSERVER_PORT);
UpdateStatus updateStatus(web_server);
ESP8266HTTPUpdateServer esp_updater;


// Post to InfluxDB
WiFiClient client;
HTTPClient http;
int influx_status = 0;
time_t post_time = 0;

// Breathing status LED
const uint32_t ok_interval = 5000;
const uint32_t err_interval = 1000;

uint32_t breathe_interval = ok_interval; // ms for one led breathe cycle
bool enabledBreathing = true;

#ifndef PWMRANGE
#define PWMRANGE 1023
#endif

// Time sync
WiFiUDP ntpUDP;
NTPClient ntp(ntpUDP, NTP_SERVER);
static char start_time[30] = "";

// Syslog
WiFiUDP logUDP;
Syslog syslog(logUDP, SYSLOG_PROTO_IETF);

// ADS1256 DRDY interrupt handling flag
volatile bool ready = false;

// ADS1256 data
Ads1256::Time dly;
Ads1256::Spi spi(PIN_CS);
Ads1256 ads(spi, dly, ready);
char ads1256_id = ' ';
int32_t ads1256_in[8] = {0};
int32_t ads1256_uv[8] = {0};
int32_t ads1256_avg_uv[8] = {0};

/* Class to calculate temperature from measured ntc voltage
 0v   uvin   uvcc
 |      |      |
 +- r1 -+- r2 -+
 |      |
 +- ntc-+

 Considerations
 * uvcc should be a stable voltage bigger than what the adc can measure
 * current through ntc should be low to avoid self heating -> r2 high
 * uvin with unplugged/defect ntc should not be too high for adc: r1/uv = (r1+r2)/uvcc
 * good resolution delta t -> delta v: r1+r2 ~ 1-2/10 r_ntc at nominal t
 * avoid em influence by not measuring at full µV resolution: r1+r2 < 100kohm 
 Example
 * ads1256 adc can measure 2.5V and has vcc 5V -> uv=2500000, uvcc=5000000, 
 * a common ntc has rt = 100 kohm at t = 25°C and beta = 3950
 * good values for r1, r2 could be ~10-20 kohm
*/
class Ntc {
  public:
    Ntc(int32_t uvcc, uint32_t r1, uint32_t r2, int32_t t, uint32_t rt, uint32_t beta) :
      _uvcc(uvcc), _c1(1.0/r1), _c2(1.0/r2), _milli_beta(beta*1000), 
      _beta_over_tk_minus_log_rt(beta/(25 + 273.15) - log(rt)), _r_mo(), _t_mc() {}

    // Calculate ntc resistance and temperature from measured voltage
    // r_ntc = 1/((uvcc/uvin - 1)/r2 - 1/r1);
    // t [°C] = 1/(ln(r_ntc/r_25)/beta + 1/t_25[K]) - 273.15
    void update_uv( size_t chan, int32_t uvin ) {
      if( chan < sizeof(_r_mo)/sizeof(*_r_mo) ) {
        float r = 1/(_c2*(_uvcc/uvin - 1) - _c1);  // µV to ohm...
        _r_mo[chan] = r * 1000;                    // ...to milli ohm to...
        _t_mc[chan] = _milli_beta/(log(r) + _beta_over_tk_minus_log_rt) - 273150;  // ...milli celsius
      }
    }

    const uint32_t *r_mo() { return _r_mo; }  // return resistance array of all channels [mOhm]
    const int32_t *t_mc() { return _t_mc; }   // return temperature array of all channels [m°C]

  private:
    const float _uvcc;  // overall voltage over voltage separator r1,r2 in µV
    const float _c1;    // conductivity 1/r1 in 1/Ohm
    const float _c2;    // conductivity 1/r2 in 1/Ohm

    const uint32_t _milli_beta;        // ntc parameter
    float _beta_over_tk_minus_log_rt;  // precalculate once

    uint32_t _r_mo[8];
    int32_t _t_mc[8];
};

Ntc ntc(5000000, 18000, 22000, 25, 100000, 3950);

// Reset Ads1256 into defined state
void resetAds() {
  Serial.print("Reset Ads1256 ");
  ads.reset();
  if (ads.wait()) {
    Serial.print("done, basic init ");
    ads.buffer(buffer);
    uint8_t cmd = 0b11110000;             // IO -> all output, all GND
    if (ads.clock_out(Ads1256::CO_OFF)    // dont drive external clock out
        && ads.wreg(Ads1256::IO, &cmd, 1) // save power for open pins
        && ads.sps(rate)                  // slow read
        && ads.mux(ads1256_ains[0])       // 1st channel->gnd
        && ads.gain(gain)                 // gain factor = 2^gain
        && ads.standby()) {               // standby
      Serial.println("done: standby");
    } else {
      Serial.println("failed");
    }
  } else {
    Serial.println("failed");
  }
}

// Post data to InfluxDB
void post_data() {
  static const char uri[] = "/write?db=" INFLUX_DB "&precision=s";

  char fmt[] = "ads1256,host=%s id=%c,ver=" VERSION ","
               "in0=%d,in1=%d,in2=%d,in3=%d,in4=%d,in5=%d,in6=%d,in7=%d,"
               "uv0=%d,uv1=%d,uv2=%d,uv3=%d,uv4=%d,uv5=%d,uv6=%d,uv7=%d,"
               "mr0=%u,mr1=%u,mr2=%u,mr3=%u,mr4=%u,mr5=%u,mr6=%u,mr7=%u,"
               "mt0=%d,mt1=%d,mt2=%d,mt3=%d,mt4=%d,mt5=%d,mt6=%d,mt7=%d";
  char msg[sizeof(fmt) + 30 + 4 * 8 * 10];
  snprintf(msg, sizeof(msg), fmt, WiFi.getHostname(), ads1256_id, 
           ads1256_in[0], ads1256_in[1], ads1256_in[2], ads1256_in[3],
           ads1256_in[4], ads1256_in[5], ads1256_in[6], ads1256_in[7],
           ads1256_uv[0], ads1256_uv[1], ads1256_uv[2], ads1256_uv[3],
           ads1256_uv[4], ads1256_uv[5], ads1256_uv[6], ads1256_uv[7],
           ntc.r_mo()[0], ntc.r_mo()[1], ntc.r_mo()[2], ntc.r_mo()[3],
           ntc.r_mo()[4], ntc.r_mo()[5], ntc.r_mo()[6], ntc.r_mo()[7],
           ntc.t_mc()[0], ntc.t_mc()[1], ntc.t_mc()[2], ntc.t_mc()[3],
           ntc.t_mc()[4], ntc.t_mc()[5], ntc.t_mc()[6], ntc.t_mc()[7]);

  http.begin(client, INFLUX_SERVER, INFLUX_PORT, uri);
  http.setUserAgent(PROGNAME);
  influx_status = http.POST(msg);
  String payload = http.getString();
  http.end();

  if (influx_status < 200 || influx_status > 299) {
    breathe_interval = err_interval;
    syslog.logf(LOG_ERR, "Post %s:%d%s status=%d msg='%s' response='%s'",
                INFLUX_SERVER, INFLUX_PORT, uri, influx_status, msg,
                payload.c_str());
  } else {
    breathe_interval = ok_interval;
    post_time = time(NULL);
  };
}

// Standard page
const char *main_page( const char *body ) {
  static const char fmt[] =
      "<html>\n"
      " <head>\n"
      "  <title>" PROGNAME " v" VERSION "</title>\n"
      "  <meta http-equiv=\"expires\" content=\"5\">\n"
      " </head>\n"
      " <body>\n"
      "  <h1>" PROGNAME " v" VERSION "</h1>\n%s"
      "  <table><tr>\n"
      "   <td><form action=\"json\">\n"
      "    <input type=\"submit\" name=\"json\" value=\"JSON\" />\n"
      "   </form></td>\n"
      "   <td><form action=\"breathe\" method=\"post\">\n"
      "    <input type=\"submit\" name=\"breathe\" value=\"Breathe\" />\n"
      "   </form></td>\n"
      "   <td><form action=\"reset\" method=\"post\">\n"
      "    <input type=\"submit\" name=\"reset\" value=\"Reset\" />\n"
      "   </form></td>\n"
      "  </tr></table>\n"
      "  <div>Post firmware image to /update<div>\n"
      "  <div>Influx status: %d<div>\n"
      "  <div>Last update: %s<div>\n"
      " </body>\n"
      "</html>\n";
  static char page[sizeof(fmt) + 500] = "";
  static char curr_time[30];
  time_t now;
  time(&now);
  strftime(curr_time, sizeof(curr_time), "%FT%T%Z", localtime(&now));
  snprintf(page, sizeof(page), fmt, body, influx_status, curr_time);
  return page;
}

// Define web pages for update, reset or for event infos
void setup_webserver() {
  web_server.on("/json", []() {
    static const char fmt[] =
        "{\n"
        " \"meta\": {\n"
        "  \"device\":  \"%s\",\n"
        "  \"program\": \"" PROGNAME "\",\n"
        "  \"version\": \"" VERSION "\",\n"
        "  \"esp_id\":  \"%06x\",\n"
        "  \"ads_id\":  \"%c\",\n"
        "  \"started\": \"%s\",\n"
        "  \"posted\":  \"%s\"\n"
        " },\n"
        " \"ads\": {\n"
        "  \"in\":     [ %9d, %9d, %9d, %9d, %9d, %9d, %9d, %9d ],\n"
        "  \"uv\":     [ %9d, %9d, %9d, %9d, %9d, %9d, %9d, %9d ],\n"
        "  \"uv_avg\": [ %9d, %9d, %9d, %9d, %9d, %9d, %9d, %9d ],\n"
        "  \"r_mo\":   [ %9u, %9u, %9u, %9u, %9u, %9u, %9u, %9u ],\n"
        "  \"t_mc\":   [ %9d, %9d, %9d, %9d, %9d, %9d, %9d, %9d ]\n"
        " }\n"
        "}\n";
    static char msg[sizeof(fmt) + 2 * 22 + 5 * 8 * 10];
    static char inf_time[30];
    strftime(inf_time, sizeof(inf_time), "%FT%T%Z", localtime(&post_time));
    uint32_t esp_id = ESP.getChipId();
    snprintf(msg, sizeof(msg), fmt, WiFi.getHostname(), esp_id, ads1256_id, start_time, inf_time,
             ads1256_in[0], ads1256_in[1], ads1256_in[2], ads1256_in[3],
             ads1256_in[4], ads1256_in[5], ads1256_in[6], ads1256_in[7],
             ads1256_uv[0], ads1256_uv[1], ads1256_uv[2], ads1256_uv[3],
             ads1256_uv[4], ads1256_uv[5], ads1256_uv[6], ads1256_uv[7],
             ads1256_avg_uv[0], ads1256_avg_uv[1], ads1256_avg_uv[2], ads1256_avg_uv[3], 
             ads1256_avg_uv[4], ads1256_avg_uv[5], ads1256_avg_uv[6], ads1256_avg_uv[7],
             ntc.r_mo()[0], ntc.r_mo()[1], ntc.r_mo()[2], ntc.r_mo()[3],
             ntc.r_mo()[4], ntc.r_mo()[5], ntc.r_mo()[6], ntc.r_mo()[7],
             ntc.t_mc()[0], ntc.t_mc()[1], ntc.t_mc()[2], ntc.t_mc()[3],
             ntc.t_mc()[4], ntc.t_mc()[5], ntc.t_mc()[6], ntc.t_mc()[7]);
    web_server.send(200, "application/json", msg);
  });

  // Call this page to reset the ESP
  web_server.on("/reset", HTTP_POST, []() {
    syslog.log(LOG_NOTICE, "RESET");
    web_server.send(200, "text/html",
                    "<html>\n"
                    " <head>\n"
                    "  <title>" PROGNAME " v" VERSION "</title>\n"
                    "  <meta http-equiv=\"refresh\" content=\"7; url=/\"> \n"
                    " </head>\n"
                    " <body>Resetting...</body>\n"
                    "</html>\n");
    delay(200);
    ESP.restart();
  });

  // Index page
  web_server.on("/", []() { 
    web_server.send(200, "text/html", main_page(""));
  });

  // Toggle breathing status led if you dont like it or ota does not work
  web_server.on("/breathe", HTTP_POST, []() {
    enabledBreathing = !enabledBreathing; 
    web_server.send(200, "text/html", main_page(enabledBreathing ? "breathing enabled" : "breathing disabled")); 
  });

  web_server.on("/breathe", HTTP_GET, []() {
    web_server.send(200, "text/html", main_page(enabledBreathing ? "breathing enabled" : "breathing disabled")); 
  });

  // Catch all page
  web_server.onNotFound( []() { 
    web_server.send(404, "text/html", main_page("<h2>page not found</h2>\n")); 
  });

  web_server.begin();

  MDNS.addService("http", "tcp", WEBSERVER_PORT);
  syslog.logf(LOG_NOTICE, "Serving HTTP on port %d", WEBSERVER_PORT);
}

volatile bool have_time = false;  // for breathing

// check ntp status
void check_ntptime() {
  ntp.update();
  if (!have_time && ntp.getEpochTime() > (2000UL - 1970) * 365 * 24 * 60 * 60 ) {
    have_time = true;
    time_t now = time(NULL);
    strftime(start_time, sizeof(start_time), "%FT%T%Z", localtime(&now));
    syslog.logf(LOG_NOTICE, "Booted at %s", start_time);
  }
}

// Status led update
void IRAM_ATTR breathe() {
  static uint32_t start = 0;  // start of last breath
  static uint32_t min_duty = PWMRANGE / 10; // limit min brightness
  static uint32_t max_duty = PWMRANGE / 2;  // limit max brightness
  static uint32_t prev_duty = 0;

  uint32_t now = millis();
  uint32_t elapsed = now - start;
  if (elapsed > breathe_interval) {
    start = now;
    elapsed -= breathe_interval;
  }

  uint32_t duty =
      (max_duty - min_duty) * elapsed * 2 / breathe_interval + min_duty;
  if (duty > max_duty) {
    duty = 2 * max_duty - duty;
  }

  duty = duty * duty / max_duty;

  if (duty != prev_duty) {
    prev_duty = duty;
    analogWrite(PIN_DB_LED, PWMRANGE - duty);
  }
}

// Interrupt handler 
void IRAM_ATTR readyIsr() { 
  ready = true;
  // dont't breathe during ota updates to avoid wdt reset
  if( enabledBreathing && have_time && !updateStatus.isUpdating() ) {
    breathe();
  }
}

// Send ads1256 data to serial and syslog
void slog(const char label[], const int32_t values[], bool as_unsigned = false ) {
  static const char sfmt[] = "%7s %9d, %9d, %9d, %9d, %9d, %9d, %9d, %9d";
  static const char ufmt[] = "%7s %9u, %9u, %9u, %9u, %9u, %9u, %9u, %9u";
  static char msg[120];
  snprintf(msg, sizeof(msg), as_unsigned ? ufmt : sfmt, label, values[0], values[1],
           values[2], values[3], values[4], values[5], values[6], values[7]);
  syslog.logf(LOG_INFO, msg);
  Serial.println(msg);
}

// Startup
void setup() {
  WiFi.mode(WIFI_STA);
  String host(HOSTNAME);
  host.toLowerCase();
  WiFi.hostname(host.c_str());

  pinMode(PIN_DB_LED, OUTPUT);
  digitalWrite(PIN_DB_LED, DB_LED_ON);
  analogWriteRange(PWMRANGE);

  Serial.begin(115200);
  Serial.println("\nStarting " PROGNAME " v" VERSION " " __DATE__ " " __TIME__);

  // Syslog setup
  syslog.server(SYSLOG_SERVER, SYSLOG_PORT);
  syslog.deviceHostname(WiFi.getHostname());
  syslog.appName("Joba1");
  syslog.defaultPriority(LOG_KERN);

  digitalWrite(PIN_DB_LED, DB_LED_OFF);

  WiFiManager wm;
  // wm.resetSettings();
  if (!wm.autoConnect()) {
    Serial.println("Failed to connect WLAN");
    for (int i = 0; i < 1000; i += 200) {
      digitalWrite(PIN_DB_LED, DB_LED_ON);
      delay(100);
      digitalWrite(PIN_DB_LED, DB_LED_OFF);
      delay(100);
    }
    ESP.restart();
    while (true)
      ;
  }

  digitalWrite(PIN_DB_LED, DB_LED_ON);
  char msg[80];
  snprintf(msg, sizeof(msg), "%s Version %s, WLAN IP is %s", PROGNAME, VERSION,
           WiFi.localIP().toString().c_str());
  Serial.println(msg);
  syslog.logf(LOG_NOTICE, msg);

  ntp.begin();

  MDNS.begin(WiFi.getHostname());

  esp_updater.setup(&web_server);
  setup_webserver();

  attachInterrupt(PIN_DRDY, readyIsr, FALLING);

  ads.begin(); // start spi

  // after ESP reset we dont know the ads state -> reset
  // then ads is calibrated, idle, single read mode for channel 0-1 mux, no gain
  resetAds(); // reset ads and set some basic config (SPS, mux, clock out,
              // standby)

  if (ads.wakeup() && ads.wait()) {
    ads.selfcal();
    ads.wait();
    uint8_t id = ads.id();
    static char hex_digits[] = "0123456789abcdef";
    ads1256_id = hex_digits[id];
    Serial.println("Setup done");
  } else {
    Serial.println("Setup failed");
  }
}

// Main loop
void loop() {
  Ads1256::value_t values[sizeof(ads1256_ains)] = {0};
  static size_t swipes = reportCount - 1;  // first report at first values
  static bool first_swipe = true;

  check_ntptime();
  web_server.handleClient();
  updateStatus.noUpdate();

  if (ads.read_swipe(values, ads1256_ains, 0, sizeof(ads1256_ains), first_swipe)) {
    swipes++;
    for (size_t i = 0; i < sizeof(ads1256_ains); i++) {
      size_t ch = ads1256_ains[i];
      ads1256_in[ch] = Ads1256::to_int(values[i]);
      ads1256_uv[ch] = Ads1256::to_microvolts(ads1256_in[ch], gain);
      if( first_swipe ) {
        ads1256_avg_uv[ch] = ads1256_uv[ch];
      }
      else {
        ads1256_avg_uv[ch] = ((int64_t)ads1256_avg_uv[ch] * num_avg + ads1256_uv[ch]) / (num_avg + 1);
      }
      ntc.update_uv(ch, ads1256_avg_uv[ch]);
    }

    first_swipe = false;

    if (swipes == reportCount) {
      slog("ain:", ads1256_in);
      slog("uv:", ads1256_uv);
      slog("uv_avg:", ads1256_avg_uv);
      slog("r_ntc:", (const int32_t *)ntc.r_mo(), true);
      slog("temp:", ntc.t_mc());
      post_data();
      swipes = 0;
    }
  } else {
    syslog.logf(LOG_ERR, "swipe read failed");
    delay(100);
  }
}
