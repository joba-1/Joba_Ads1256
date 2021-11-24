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


void resetAds() {
    Serial.print("Reset Ads1256 ");
    if( ads.command(Ads1256::RESET) ) {
        ads.wait();

        Serial.print("and init ");
        ads.clock_out(Ads1256::CO_OFF);  // dont drive external clock out 

        uint8_t cmd = 0b11110000; // IO -> all output, all GND
        ads.wreg(Ads1256::IO, &cmd, 1);  // save power for open pins
      
        ads.sps(Ads1256::SPS_30K);
        ads.mux(0);  // 0->gnd
        ads.command(Ads1256::STANDBY);

        Serial.println("done: standby");
    }
    else {
        Serial.println("failed");
    }
}


void printRegs() {
    uint8_t data[Ads1256::FSC2 + 1];

    Serial.println();
    Serial.println("ST MU AD DR IO O0 O1 O2 F0 F1 F2");

    ads.rreg(Ads1256::STATUS, data, sizeof(data));

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
    Serial.printf("%8d %8d\n\n", Ads1256::to_int(o), Ads1256::to_int(f));
}


void setup() {
    Serial.begin(115200);
    // delay(1500);
    Serial.println("\nADS1256 test " __FILE__ " " __TIMESTAMP__);

    Ads1256::value_t v[] = {
        { 0, 0, 0 },
        { 0, 0, 1 },
        { 0, 1, 0 },
        { 1, 0, 0 },
        { 0, 0, 0xff },
        { 0, 0xff, 0 },
        { 0x7f, 0, 0 },
        { 0x7f, 0xff, 0xff },
        { (int8_t)0xff, 0, 0 },
        { (int8_t)0xff, 0xff, 0xff },
        { (int8_t)0x80, 0, 0 } };

    printf("%8s -> %11d %11d %11d %11d %11d %11d %11d gains\n", "raw", 0, 1, 2, 3, 4, 5, 6);
    for( size_t i=0; i<sizeof(v)/sizeof(*v); i++ ) {
        uint32_t raw = Ads1256::to_int(v[i]);
        uint32_t uv0 = Ads1256::to_microvolts(raw, 0, 2500000);
        uint32_t uv1 = Ads1256::to_microvolts(raw, 1, 2500000);
        uint32_t uv2 = Ads1256::to_microvolts(raw, 2, 2500000);
        uint32_t uv3 = Ads1256::to_microvolts(raw, 3, 2500000);
        uint32_t uv4 = Ads1256::to_microvolts(raw, 4, 2500000);
        uint32_t uv5 = Ads1256::to_microvolts(raw, 5, 2500000);
        uint32_t uv6 = Ads1256::to_microvolts(raw, 6, 2500000);
        printf("%8d -> %11d %11d %11d %11d %11d %11d %11d µV\n", raw, uv0, uv1, uv2, uv3, uv4, uv5, uv6);
    }


    attachInterrupt(PIN_DRDY, readyIsr, FALLING);

    // after reset we are calibrated, idle, single read mode for channel 0-1 mux, no gain
    ads.begin();
    resetAds();

    printRegs();
    for( uint8_t in = 0; in < 8; in++ ) {
        for( uint8_t g = 0; g <= 7; g++ ) {
            int32_t raw = ads.one_shot(in, 8, g);
            Serial.printf("One shot channel=%u, gain=%u, hml=(%02x %02x %02x), value=%d\n", in, g, (raw >> 16) & 0xff, (raw >> 8) & 0xff, raw & 0xff, Ads1256::to_microvolts(raw));
            printRegs();
        }
        int32_t raw = ads.one_shot(in, 8, 0);
        Serial.printf("One shot channel=%u, gain=%u, hml=(%02x %02x %02x), value=%d\n", in, 0, (raw >> 16) & 0xff, (raw >> 8) & 0xff, raw & 0xff, Ads1256::to_microvolts(raw));
        printRegs();
        Serial.println();
    }

    resetAds();
    // ads.command(Ads1256::STANDBY);
    ads.command(Ads1256::SYNC);
    ads.command(Ads1256::WAKEUP);
    // if( !ads.rdata() ) Serial.println("rdata failed");    // request next valid data
}


void handleKeypress( uint8_t &chan, uint8_t &gain ) {
    uint8_t newGain = gain;
    bool gainCal = false;
    bool muxCal = false;
    while( Serial.available() ) {
        int ch = Serial.read();
        if( ch >= '0' && ch <= '6' ) {
            newGain = ch - '0';
            gain = 1 << newGain;
            gainCal = true;
        }
        else if( ch >= 'a' && ch <= 'h' ) {
            chan = ch - 'a';
            muxCal = true;
        }
    }
    if( gainCal || muxCal ) {
        ads.command(Ads1256::SDATAC);
        ads.rdata();
        ads.wait();
        if( muxCal ) {
            ads.mux(chan);
        }
        if( gainCal ) {
            ads.gain(newGain);
        }
        ads.command(Ads1256::SELFCAL);
        ads.wait();
    }
}


Ads1256::value_t currValues[10000];
Ads1256::value_t lastValues[10000];
size_t numValues = sizeof(currValues)/sizeof(*currValues);

void loop() {
    static uint8_t gain = 1;

    uint32_t start = millis();

    memset(currValues, 0, sizeof(currValues));
    bool ok = ads.bulk_read(currValues, numValues);
    uint32_t elapsed = millis() - start;

    uint8_t lastChan = chan;
    uint8_t lastGain = gain;
  
    handleKeypress(chan, gain);

    memcpy(lastValues, currValues, sizeof(lastValues));

    // this could be done in another task so next bulk_read could start earlier (or better use circular buffer)
    if( ok ) { 
        Serial.printf("\nChannel %u with gain factor %u: Elapsed: %u ms -> %u SPS\n", lastChan, lastGain, elapsed, (numValues*1000)/elapsed);
        for( int i = 0; i < numValues; i++ ) {
            Serial.printf("Bulk(%5d): (%02x %02x %02x) = %10d raw = %10d µV\n", 
                i, lastValues[i].hi, lastValues[i].mid, lastValues[i].lo, 
                Ads1256::to_int(lastValues[i]), Ads1256::to_microvolts(Ads1256::to_int(lastValues[i]), lastGain));
            if( i == 4 || i == numValues/2 + 2) {
                Serial.println();
                i += numValues/2 - 7;
            }
        }
        ads.command(Ads1256::SDATAC);
        ads.rdata();
        ads.wait();
        printRegs();
    }
    else {
        Serial.println("bulk read failed");
    }
}


void loop_one_shots() {
    static uint32_t count = 0;

    int32_t raw = ads.one_shot();
    if( raw != INT32_MIN ) { 
        Serial.printf("Channel[%u]: %7d µV\n", chan, Ads1256::to_microvolts(raw));
        if( ++count > 10 ) {
            count = 0;
            if( ++chan > 7 ) chan = 0;
            ads.mux(chan);
            ads.command(Ads1256::STANDBY);
            Serial.println();
        }
    }
    delay(200);
}


void loop_single_query() {
    static uint32_t count = 0;

    Ads1256::value_t value = {};
    if( ads.update(&value) ) {
        int32_t raw = Ads1256::to_int(value);
        Serial.printf("Channel[%u]: %7d µV\n", chan, Ads1256::to_microvolts(raw));
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