/*
Read voltages from Ads1256 with different parameters for comparison

According to the datasheet the lowest effective resolution is 13.8 bits (during high SPS).
Also measurements are from 0 to 3V if buffering is on, so accuracy is at least 3V/2^13.8 ~= 200 µV
Maximum voltage with highest gain is 3V / 64 ~= 47 mV

So, if I provide the same ~40 mV on all Ain, I should get results within 200 µV no matter what?
This example cycles through all available Ain, gain and SPS, with and without buffering, 
with or without self calibration after changes and with single shot or bulk readings.

Since 40mV is a bit low for my testbed, I dont test gain 16, 32 and 64 and use 300-350 mV
*/

#include <Arduino.h>

#include <Ads1256.h>
#include <math.h>

// Start measurement if Ain0 is in this mV range
#define MV_MIN 300
#define MV_MAX 350


#define PIN_CS    5
#define PIN_DRDY 17
#define PIN_LED   2


// Interrupt flag, set true by falling edge of DRdy
volatile bool ready = false;

void IRAM_ATTR readyIsr() {
    ready = true;
}


Ads1256::Time tick;
Ads1256::Spi spi(PIN_CS);
Ads1256 ads(spi, tick, ready);

Ads1256::value_t value[100];  // free choice of size for bulk reads (full SPS above 1000)
const uint8_t Values = sizeof(value)/sizeof(*value);


void resetAds() {
    Serial.print("Reset Ads1256 ");
    ads.reset();
    if( ads.wait() ) {
        Serial.print("done, init ");
        uint8_t cmd = 0b11110000;  // IO -> all gpio as output, all GND
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

    ads.begin();  // start spi with max allowed speed

    // after ESP reset we dont know the ads state -> reset
    // then ads is calibrated, idle, single read mode for channel 0-1 mux, no gain
    resetAds();  // set some basic config (SPS 30k, mux 0->gnd, clock out off, standby)

    // Wait for voltage that is valid for all gain settings used by the test
    if( ads.wakeup() && ads.wait() ) {
        printRegs();
        Serial.printf("waiting for valid input range %d-%d mV on Ain0\n", MV_MIN, MV_MAX);
        Ads1256::PinOut led(PIN_LED);
        led.toggle();
        int32_t uv = 0;
        uint32_t start = tick.now_ms();
        while( tick.now_ms() - start < 1000 ) {  // stable for at least 1s
            if( ads.sync_wakeup() && ads.wait() && ads.rdata(value[0]) ) {
                uv = Ads1256::to_microvolts(Ads1256::to_int(value[0]));
                Serial.printf("%8d mV \r", uv / 1000);
            }
            if( uv < MV_MIN * 1000 || uv > MV_MAX * 1000 ) {
                start = tick.now_ms();
            }
            tick.delay_ms(10);
        }
        led.toggle();
        Serial.println("init done");
    }
    else {
        Serial.println("init failed");
    }
}


bool analyzeValue( uint8_t gain, size_t &iMin, size_t &iMax, int32_t &uvAvg, int32_t &uvStddev ) {
    int32_t minValue = INT32_MAX;
    int32_t maxValue = INT32_MIN;
    int64_t sumValues = 0;
    int64_t sumSquaredValues = 0;
    size_t count = 0;

    for( size_t i = 0; i < Values; i++ ) {
        int32_t currValue = Ads1256::to_microvolts(Ads1256::to_int(value[i]), gain);
        if( currValue != INT32_MIN && currValue != INT32_MAX ) {
            if( currValue < minValue ) { minValue = currValue; iMin = i; }
            if( currValue > maxValue ) { maxValue = currValue; iMax = i; }
            sumValues += currValue;
            sumSquaredValues += (int64_t)currValue * currValue;
            count++;
        }
    }

    if( !count ) return false;

    uvAvg = (int32_t)((sumValues + count/2)/count); 
    uvStddev = (int32_t)(sqrt( (sumSquaredValues - (double)(sumValues*sumValues)/count) / (count - 1) ) + 0.5);
    return true;
}


void loop() {
    // Arrays of used sps rate enum values and strings
    const Ads1256::rate_t Rate[] = { Ads1256::SPS_30K, Ads1256::SPS_15K, Ads1256::SPS_7K5, 
        Ads1256::SPS_3K75, Ads1256::SPS_2K, Ads1256::SPS_1K, Ads1256::SPS_500, Ads1256::SPS_100,
        Ads1256::SPS_60, Ads1256::SPS_50, Ads1256::SPS_30, Ads1256::SPS_25, Ads1256::SPS_15, 
        Ads1256::SPS_10, Ads1256::SPS_5, Ads1256::SPS_2_5 };
    const char *Sps[] = { "30000", "15000", "7500", "3075", "2000", "1000", "500", "100", "60",
        "50", "30", "25", "15", "10", "5", "2.5" };

    const uint32_t Counts = 5;  // free choice of repeatet measurements with same parameters
    const uint8_t Chans = 8;    // Ain0 ... Ain7
    const uint8_t Gains = 7;    // 2^0 ... 2^6 (factor 1 ... 64)
    const uint8_t Rates = sizeof(Rate)/sizeof(*Rate);

    // Current settings, Default values must match those after resetAds()
    static uint32_t count = 0;
    static uint8_t chan = 0;
    static uint8_t gain = 0;
    static size_t rate = 0;
    static bool buffer = false;

    static bool calibrate = false;
    static bool bulk = false;

    bool ok = false;  // true if value[] contains valid data

    printRegs();
    Serial.printf("count=%u, chan=%u, gain=%2u, rate=%5s, buff=%c, bulk=%c, calib=%c", 
        count, chan, 1<<gain, Sps[rate], buffer ? 'y' : 'n', bulk ? 'y' : 'n', calibrate ? 'y' : 'n');

    uint32_t start = tick.now_ms();
    uint32_t sps;  // measured SPS (compare with rate setting)

    // Do one bulk or one single measurement
    if( bulk ) {
        if( ads.sync_wakeup() && ads.wait() && ads.rdatac(value[0]) ) {
            size_t v = 1;
            while( v < Values - 1 && ads.wait() && ads.read(value[v]) ) {
                ++v;
            }
            ok = (v == Values - 1 && ads.wait() && ads.read(value[Values - 1], true)); 
        }
        sps  = (1000 * Values) / (tick.now_ms() - start + 1);
    }
    else {
        ok = (ads.sync_wakeup() && ads.wait() && ads.rdata(value[0]));
        sps = 1000 / (tick.now_ms() - start + 1);
    }

    // Evaluate measurement result
    if( ok ) {
        if( bulk ) {
            size_t iMin, iMax;
            int32_t uvAvg, uvStddev;
            if( analyzeValue(gain, iMin, iMax, uvAvg, uvStddev) ) {
                int32_t rawMin = Ads1256::to_int(value[iMin]);
                int32_t uvMin = Ads1256::to_microvolts(rawMin, gain);
                int32_t rawMax = Ads1256::to_int(value[iMax]);
                int32_t uvMax = Ads1256::to_microvolts(rawMax, gain);
                Serial.printf(": sps=%5u, min=(%02x,%02x,%02x) = %8d = %5d.%03d mV, "
                    "max=(%02x,%02x,%02x) = %8d = %5d.%03d mV, stddev=%5d.%03d mV, avg=%5d.%03d mV\n", sps,
                    (uint8_t)value[iMin].hi, value[iMin].mid, value[iMin].lo, rawMin, uvMin / 1000, uvMin % 1000,
                    (uint8_t)value[iMax].hi, value[iMax].mid, value[iMax].lo, rawMax, uvMax / 1000, uvMax % 1000,
                    uvStddev / 1000, uvStddev % 1000, uvAvg / 1000, uvAvg % 1000);
            }
            else {
                int32_t raw = Ads1256::to_int(value[0]);
                Serial.printf(": sps=%5u, v_0(%02x,%02x,%02x) = %8d = INVALID!\n", 
                    sps, (uint8_t)value[0].hi, value[0].mid, value[0].lo, raw);
            }
        } 
        else {
            int32_t raw = Ads1256::to_int(value[0]);
            int32_t uv = Ads1256::to_microvolts(raw, gain);
            if( uv != INT32_MAX && uv != INT32_MIN ) {
                Serial.printf(": sps=%5u, hml=(%02x,%02x,%02x) = %8d = %5d.%03d mV\n",
                    sps, (uint8_t)value[0].hi, value[0].mid, value[0].lo, raw, uv / 1000, uv % 1000);
            }
            else {
                Serial.printf(": sps=%5u, hml=(%02x,%02x,%02x) = %8d = INVALID!\n",
                    sps, (uint8_t)value[0].hi, value[0].mid, value[0].lo, raw);
            }
        }
    }
    else {
        Serial.println(" failed");
    }

    // iterate over all permutations
    if( ++count >= Counts ) {
        count = 0;
        if( ++chan >= Chans - 5 ) {  // -5 for testing (only chan 0-2)
            chan = 0;
            if( ++gain >= Gains - 3 ) {  // -3 skips 16, 32 and 64 (no overflow below ~375mV)
                gain = 0;
                if( ++rate >= Rates - 4 ) {  // -4 skips SPS below 25 (boring to wait)
                    rate = 0;
                    buffer = !buffer;
                    if( !buffer ) {
                        bulk = !bulk;
                        if( !bulk ) {
                            calibrate = !calibrate;
                        }
                    }
                    if( !ads.buffer(buffer) ) {
                        Serial.println("Set buffering status failed");
                    }
                }
                if( !ads.sps(Rate[rate]) ) {
                    Serial.println("Set SPS rate failed");
                }
            }
            if( !ads.gain(gain) ) {
            Serial.println("Set gain failed");
            }
        }
        ads.mux(chan);
        if( calibrate ) {
            if( !ads.selfcal() || !ads.wait() ) {
                Serial.println("Self calibration failed");
            }
        }
    }
}
