#include <Arduino.h>

#include <Ads1256.h>
#include <math.h>


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


void check24bit() {
    typedef struct val2uvolt { Ads1256::value_t val; int32_t uvolt[7]; } val2uvolt_t;

    val2uvolt_t vu[] = {  // selected value_t values to print out
        {{ 0, 0, 0 }},
        {{ 0, 0, 1 }},
        {{ 0, 1, 0 }},
        {{ 1, 0, 0 }},
        {{ 0, 0, 0xff }},
        {{ 0, 0xff, 0 }},
        {{ 0x7f, 0, 0 }},
        {{ 0x7f, 0xff, 0xff }},
        {{ (int8_t)0xff, 0, 0 }},
        {{ (int8_t)0xff, 0xff, 0xff }},
        {{ (int8_t)0x80, 0, 0 }} };

    Serial.printf("%30s -> %11d %11d %11d %11d %11d %11d %11d gains\n", "u, i, raw", 0, 1, 2, 3, 4, 5, 6);
    for( uint32_t u=0; u<=0xffffff; u++) {    // possible values from sensor as uint32_t
        int32_t i = (int32_t)(u | ((u & 0x800000) ? 0xff000000 : 0));  // possible values from sensor as int32_t
        Ads1256::value_t value;               // possible values from sensor as value_t
        value.hi = (int8_t)(u >> 16);
        value.mid = (u >> 8) & 0xff;
        value.lo = u & 0xff;
        int32_t raw = Ads1256::to_int(value);  // calculated value as int32_t
        bool isExample = false;
        for( size_t v=0; v < sizeof(vu)/sizeof(*vu); v++ ) {
            if( value.hi == vu[v].val.hi && value.mid == vu[v].val.mid && value.lo == vu[v].val.lo ) {
                isExample = true;
                Serial.printf("0x%06x => %8d == %8d -> ", u, i, raw);
                for( size_t g=0; g < sizeof(vu[0].uvolt)/sizeof(vu[0].uvolt[0]); g++) {
                    vu[v].uvolt[g] = Ads1256::to_microvolts(raw, g, 2500000);
                }
                Serial.printf("%11d %11d %11d %11d %11d %11d %11d µV\n", vu[v].uvolt[0], vu[v].uvolt[1], 
                    vu[v].uvolt[2], vu[v].uvolt[3], vu[v].uvolt[4], vu[v].uvolt[5], vu[v].uvolt[6]);
            }
        }
        if( !isExample && i != raw ) {
            Serial.printf("0x%06x => %8d != %8d -> error\n", u, i, raw);
        }
    }
}


void setup() {
    Serial.begin(115200);
    // delay(1500);
    Serial.println("\nADS1256 test " __FILE__ " " __TIMESTAMP__);

    // check24bit();
 
    attachInterrupt(PIN_DRDY, readyIsr, FALLING);

    ads.begin();  // start spi

    // after ESP reset we dont know the ads state -> reset
    // then ads is calibrated, idle, single read mode for channel 0-1 mux, no gain
    resetAds();  // reset ads and set some basic config (SPS, mux, clock out, standby)

    // on shot measurements for each channel and gain combination
    printRegs();  // show status of all registers
    for( uint8_t in = 0; in < 8; in++ ) {
        for( uint8_t g = 0; g <= 7; g++ ) {
            int32_t raw = ads.read_once(in, 8, g);
            Serial.printf("One shot chan=%u, gain=%u, hml=(%02x %02x %02x), µV=%d\n", in, g, (raw >> 16) & 0xff, (raw >> 8) & 0xff, raw & 0xff, Ads1256::to_microvolts(raw));
            printRegs();
        }
        // to check consistency, try channel 0 a second time
        int32_t raw = ads.read_once(in, 8, 0);
        Serial.printf("One shot chan=%u, gain=%u, hml=(%02x %02x %02x), µV=%d\n", in, 0, (raw >> 16) & 0xff, (raw >> 8) & 0xff, raw & 0xff, Ads1256::to_microvolts(raw));
        Serial.println();
        printRegs();
    }

    resetAds();
    ads.wakeup() && ads.wait();
    printRegs();
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
        if( muxCal ) {
            ads.mux(chan);
        }
        if( gainCal ) {
            ads.gain(newGain);
        }
        ads.selfcal();
        ads.wait();
    }
}


bool first = true;
uint8_t swipeAins[] = { 0, 1, 2, 3, 4, 5, 6, 7 };
uint8_t *swipeAouts = 0;  // measure ain -> gnd
Ads1256::value_t swipeValues[sizeof(swipeAins)];
size_t swipeRepeats = 1000;

void loop_swipe() {
    uint32_t start = millis();
    memset(swipeValues, 0, sizeof(swipeValues));
    size_t count = sizeof(swipeAins);
    if( ads.read_swipe(swipeValues, swipeAins, swipeAouts, count, first) ) {
        first = false;
        for( size_t repeat = 0; repeat < swipeRepeats; repeat++ ) {
            ads.read_swipe(swipeValues, swipeAins, swipeAouts, count, first);
        }
        uint32_t elapsed = millis() - start;
        Serial.printf("Swipe SPS = %u: ", count*swipeRepeats*1000/elapsed);
        for( size_t i = 0; i < count; i++ ) {
            int32_t uv = Ads1256::to_microvolts(Ads1256::to_int(swipeValues[i]));
            Serial.printf("%u = %d, ", swipeAins[i], uv);
        }
        Serial.println();
    }
    else {
        Serial.println("swipe read failed");
        delay(1000);
    }
}


Ads1256::value_t currValues[10000];
Ads1256::value_t lastValues[10000];
size_t numValues = sizeof(currValues)/sizeof(*currValues);

void loop_bulk() {
    static uint8_t gain = 1;

    uint32_t start = millis();

    memset(currValues, 0, sizeof(currValues));
    bool ok = ads.read_bulk(currValues, numValues, true);
    uint32_t elapsed = millis() - start;

    uint8_t lastChan = chan;
    uint8_t lastGain = gain;
  
    handleKeypress(chan, gain);

    memcpy(lastValues, currValues, sizeof(lastValues));

    // this could be done in another task so next bulk_read could start earlier (or better use circular buffer)
    if( ok ) { 
        Serial.printf("Channel %u with gain factor %u: Elapsed: %u ms -> %u SPS - ", 
            lastChan, lastGain, elapsed, (numValues*1000)/elapsed);
        int32_t minValue = INT32_MAX;
        int32_t maxValue = INT32_MIN;
        int64_t sumValues = 0;
        int64_t sumSquaredValues = 0;
        for( int i = 0; i < numValues; i++ ) {
            int32_t currValue = Ads1256::to_microvolts(Ads1256::to_int(lastValues[i]), lastGain);
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
    else {
        Serial.println("bulk read failed");
        delay(1000);
    }
}


void loop_one_shots() {
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
    delay(200);
}


void loop_single_query() {
    static uint32_t count = 0;

    Ads1256::value_t value = {};
    if( ads.rdata(value) ) {
        int32_t raw = Ads1256::to_int(value);
        Serial.printf("Channel[%u]: %7d µV\n", chan, Ads1256::to_microvolts(raw));
        if( ++count > 10 ) {
            count = 0;
            if( ++chan > 7 ) chan = 0;
            ads.mux(chan);
        }
        delay(200);
        if( !ads.sync() ) Serial.println("sync failed");
    }
    else {
        Serial.println("rdata failed");
    }
}


void loop() {
    // loop_swipe();
    loop_bulk();
}