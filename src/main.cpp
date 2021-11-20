#include <Arduino.h>

#include <Ads1256.h>


#define PIN_CS 5
#define PIN_DRDY 17


volatile bool ready = false;

void IRAM_ATTR readyIsr() {
    ready = true;
}


ArduinoDelay dly;
ArduinoSpi spi(PIN_CS);
ArduinoAds1256 ads(spi, dly, ready);

uint8_t chan = 0;
        

long long mikrovolt( uint32_t raw ) {
  return ((raw - 1300LL) * 2780000LL) / (4689000LL - 1300LL);
}


void setup() {
    Serial.begin(115200);
    Serial.println("ADS1256 swipe mode " __FILE__ " " __TIMESTAMP__);

    attachInterrupt(PIN_DRDY, readyIsr, FALLING);

    // after reset we are calibrated, idle, single read mode for channel 0-1 mux, no gain
    ads.begin();
    Serial.printf("Ads1256 ID=%u\n", ads.id());
    Serial.printf("One shot 0(g=0): %lld\n", mikrovolt(ads.one_shot(0, 8)));
    Serial.printf("One shot 0(g=1): %lld\n", mikrovolt(ads.one_shot(0, 8, 1)/2));
    Serial.printf("One shot 0(g=2): %lld\n", mikrovolt(ads.one_shot(0, 8, 2)/4));
    Serial.printf("One shot 0(g=3): %lld\n", mikrovolt(ads.one_shot(0, 8, 3)/8));

    Serial.printf("One shot 1(g=0): %lld\n", mikrovolt(ads.one_shot(1)));
    Serial.printf("One shot 1(g=1): %lld\n", mikrovolt(ads.one_shot(1, 8, 1)/2));
    Serial.printf("One shot 1(g=2): %lld\n", mikrovolt(ads.one_shot(1, 8, 2)/4));

    Serial.printf("One shot 6(g=0): %lld\n", mikrovolt(ads.one_shot(6, 8, 0)));
    Serial.printf("One shot 6(g=1): %lld\n", mikrovolt(ads.one_shot(6, 8, 1)/2));
    Serial.printf("One shot 6(g=2): %lld\n", mikrovolt(ads.one_shot(6, 8, 2)/4));
    Serial.printf("One shot 6(g=3): %lld\n", mikrovolt(ads.one_shot(6, 8, 3)/8));
    Serial.printf("One shot 6(g=4): %lld\n", mikrovolt(ads.one_shot(6, 8, 4)/16));
    Serial.printf("One shot 6(g=5): %lld\n", mikrovolt(ads.one_shot(6, 8, 5)/32));
    Serial.printf("One shot 6(g=6): %lld\n", mikrovolt(ads.one_shot(6, 8, 6)/64));

    Serial.printf("One shot 6(g=6): %lld\n", mikrovolt(ads.one_shot()/64));
    Serial.printf("One shot 6(g=6): %lld\n", mikrovolt(ads.one_shot()/64));
    Serial.printf("One shot 6(g=6): %lld\n", mikrovolt(ads.one_shot()/64));

    Ads1256::value_t values[10];
    if( ads.bulk_read(values, 10) ) {
      for( int i=0; i < 10; i++ ) {
        Serial.printf("Bulk(%d): %lld\n", i, mikrovolt(Ads1256::to_int(values[i])/64));
      }
    }
    
    ads.command(Ads1256::RESET);
    ads.wait();
    ads.clock_out(Ads1256::CO_OFF);  // dont drive external clock out 
    uint8_t cmd = 0b11110000;
    ads.wreg(Ads1256::IO, &cmd, 1);  // save power for open pins
    ads.sps(Ads1256::SPS_100);
    ads.auto_calibrate(true);
    ads.mux(chan);  // 0->gnd
    ads.command(Ads1256::SYNC);
    ads.command(Ads1256::WAKEUP);
    if( !ads.rdata() ) Serial.println("rdata failed");    // request next valid data
}

void loop() {
    static uint32_t count = 0;

    Ads1256::value_t value = {};
    if( ads.update(&value) ) {
        int32_t raw = Ads1256::to_int(value);
        Serial.printf("Channel[%u]: %7lld µV\n", chan, mikrovolt(raw));
        if( ++count > 10 ) {
            count = 0;
            if( ++chan > 7 ) chan = 0;
            ads.mux(chan);
            ads.command(Ads1256::SYNC);
            ads.command(Ads1256::WAKEUP);
        }
        delay(200);
        if( !ads.rdata() ) Serial.println("rdata failed");    // request next valid data
    }
}