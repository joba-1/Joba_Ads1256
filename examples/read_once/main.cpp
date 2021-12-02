#include <Arduino.h>

#include <Ads1256.h>


#define PIN_CS 5
#define PIN_DRDY 17


volatile bool ready = false;

void IRAM_ATTR readyIsr() {
    ready = true;
}


Ads1256::Time dly;
Ads1256::Spi spi(PIN_CS);
Ads1256 ads(spi, dly, ready);

uint8_t chan = 0;


void resetAds() {
    Serial.print("Reset Ads1256 ");
    ads.reset();
    if( ads.wait() ) {
        Serial.print("done, init ");
        uint8_t cmd = 0b11110000; // IO -> all output, all GND
        if( ads.clock_out(Ads1256::CO_OFF)  // dont drive external clock out 
         && ads.wreg(Ads1256::IO, &cmd, 1)  // save power for open pins
         && ads.sps(Ads1256::SPS_30K)       // fast read
         && ads.mux(0)                      // 0->gnd
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
    Serial.println("\nADS1256 test " __FILE__ " " __TIMESTAMP__);

    attachInterrupt(PIN_DRDY, readyIsr, FALLING);

    ads.begin();  // start spi

    // after ESP reset we dont know the ads state -> reset
    // then ads is calibrated, idle, single read mode for channel 0-1 mux, no gain
    resetAds();  // reset ads and set some basic config (SPS 30k, mux 0->gnd, clock out off, standby)

    if( ads.wakeup() && ads.wait() ) {
        printRegs();
        Serial.println("init done");
    }
    else {
        Serial.println("init failed");
    }
}


void loop() {
    static uint32_t count = 0;

    int32_t raw = ads.read_once();
    if( raw != INT32_MIN ) { 
        Serial.printf("Channel[%u]: %7d µV\n", chan, Ads1256::to_microvolts(raw));
        if( ++count > 10 ) {
            count = 0;
            if( ++chan > 7 ) chan = 0;
            ads.mux(chan);
            Serial.println();
        }
    }
    else {
        Serial.println("Read once failed");
    }
    delay(200);
}
