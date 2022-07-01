#include <Arduino.h>

#include <Ads1256.h>
#include <math.h>


// example pins for ESP8266 and ESP32
#if defined(ESP8266)
#define PIN_CS D8
#define PIN_DRDY D1
#elif defined(ESP32)
#define PIN_CS 5
#define PIN_DRDY 17
#else
#error "No ESP8266 or ESP32, define your pins here!"
#endif


volatile bool ready = false;

void IRAM_ATTR readyIsr() {
    ready = true;
}


Ads1256::Time dly;
Ads1256::Spi spi(PIN_CS);
Ads1256 ads(spi, dly, ready);

uint8_t chan = 7;  // first channel
uint8_t gain = 0;  // first gain factor = 2^gain


void resetAds() {
    Serial.print("Reset Ads1256 ");
    ads.reset();
    if( ads.wait() ) {
        Serial.print("done, init ");
        ads.buffer(false);
        uint8_t cmd = 0b11110000; // IO -> all output, all GND
        if( ads.clock_out(Ads1256::CO_OFF)  // dont drive external clock out 
         && ads.wreg(Ads1256::IO, &cmd, 1)  // save power for open pins
         && ads.sps(Ads1256::SPS_100)       // fast read
         && ads.mux(chan)                   // chan->gnd
         && ads.standby() ) {               // standby
            Serial.println("done: standby");
        }
        else {
            Serial.println("failed");
        }
    }
    else {
        Serial.println("failed");
    }
}


void printRegs() {
    uint8_t data[Ads1256::FSC2 + 1] = {0};

    ads.rreg(Ads1256::STATUS, data, sizeof(data));

    Serial.print("(ST MU AD DR IO OFFSET FULL)=(");
    for(size_t i = 0; i < 5; i++) {
        Serial.printf("%02x ", data[i]);
    }

    Ads1256::value_t o, f;
    o.lo = data[5]; 
    o.mid = data[6]; 
    o.hi = data[7]; 
    f.lo = data[8]; 
    f.mid = data[9]; 
    f.hi = data[10]; 
    Serial.printf("%8d %8d) - ", Ads1256::to_int(o), Ads1256::to_int(f));
}


void setup() {
    Serial.begin(115200);
    Serial.println("\nADS1256 bulk " __FILE__ " " __TIMESTAMP__);

    attachInterrupt(PIN_DRDY, readyIsr, FALLING);

    ads.begin();  // start spi

    // after ESP reset we dont know the ads state -> reset
    // then ads is calibrated, idle, single read mode for channel 0-1 mux, no gain
    resetAds();  // reset ads and set some basic config (SPS 30k, mux 0->gnd, clock out off, standby)

    if( ads.wakeup() && ads.wait() ) {
        Serial.println("init done");
        printRegs();
    }
    else {
        Serial.println("init failed");
    }
}


void handleKeypress( uint8_t &chan, uint8_t &gain ) {
    bool gainCal = false;
    bool muxCal = false;
    while( Serial.available() ) {
        int ch = Serial.read();
        if( ch >= '0' && ch <= '6' ) {
            gain = ch - '0';
            gainCal = true;
        }
        else if( ch >= 'a' && ch <= 'h' ) {
            chan = ch - 'a';
            muxCal = true;
        }
    }
    if( gainCal || muxCal ) {
        if( muxCal ) {
            ads.mux(chan);
        }
        if( gainCal ) {
            ads.gain(gain);
        }
        ads.selfcal();
        ads.wait();
    }
}


Ads1256::value_t currValues[200];
Ads1256::value_t lastValues[200];
size_t numValues = sizeof(currValues)/sizeof(*currValues);

void print_stats( uint8_t chan, uint8_t gain, uint32_t elapsed ) {
    Serial.printf("Channel %u with gain factor %u: Elapsed: %u ms -> %u SPS - ", 
        chan, 1 << gain, elapsed, (numValues*1000)/elapsed);
    int32_t minValue = INT32_MAX;
    int32_t maxValue = INT32_MIN;
    int64_t sumValues = 0;
    int64_t sumSquaredValues = 0;
    for( size_t i = 0; i < numValues; i++ ) {
        int32_t currValue = Ads1256::to_microvolts(Ads1256::to_int(lastValues[i]), gain);
        if( currValue < minValue ) minValue = currValue;
        if( currValue > maxValue ) maxValue = currValue;
        sumValues += currValue;
        sumSquaredValues += (int64_t)currValue * currValue;
    }
    Serial.printf("Bulk(%u): min/max/avg/stddev=%10d, %10d, %10d, %10.2f\n", 
        numValues, minValue, maxValue, (int32_t)(sumValues/numValues), 
        sqrt( (sumSquaredValues - (double)(sumValues*sumValues)/numValues) / (numValues - 1) ));
    printRegs();
}


void loop() {
    uint32_t start = millis();

    bool ok = ads.read_bulk(currValues, numValues, true);

    uint32_t elapsed = millis() - start;

    uint8_t lastChan = chan;
    uint8_t lastGain = gain;
  
    handleKeypress(chan, gain);

    memcpy(lastValues, currValues, sizeof(lastValues));

    // this could be done in another task so next bulk_read could start earlier (or better use circular buffer)
    if( ok ) { 
        print_stats(lastChan, lastGain, elapsed);
    }
    else {
        Serial.println("bulk read failed");
        delay(1000);
    }
}
