#ifndef Ads1256_h
#define Ads1256_h

#include <stdint.h>
#include <stdbool.h>


class Ads1256 {
public:
    // Platform independent interface for microseconds
    class Delay {
    public:
        virtual void us( uint16_t us ) = 0;
    };

    // Platform independent interface for SPI
    class Spi {
    public:
        virtual void begin( uint32_t freq ) = 0;

        virtual void start() = 0;
        virtual void transfer(uint8_t *buffer, uint8_t len) = 0;
        virtual void end() = 0;
    };

    // Platform independent interface for gpio pins
    class Pin {
    public:
        Pin( uint8_t id ) : _id(id) {};

        uint8_t id() { return _id; }
        virtual void init() = 0;
        virtual bool get() = 0;

    private:
        uint8_t _id;
    };

    class PinOut {
    public:
        PinOut() {};

        virtual void set() = 0;
        virtual void reset() = 0;
        virtual bool toggle() = 0;
    };

    typedef enum { 
        STATUS, MUX, ADCON, DRATE, IO, OFC0, OFC1, OFC2, FSC0, FSC1, FSC2 
    } register_t;

    typedef enum { 
        WAKEUP, SDATAC=0x0f, SELFCAL=0xf0, SELFOCAL, SELFGCAL, 
        SYSOCAL, SYSGCAL, SYNC=0xfc, STANDBY, RESET
    } command_t;

    typedef enum status {
        IDLE,    // no need to care for DRDY now
        SINGLE,  // wait for one DRDY low -> IDLE
        CONT,    // wait for continuous DRDY lows
        STOP,    // stop continuous mode -> IDLE
        RESTART, // reset from continuous mode -> INIT
        INIT     // recover from reset
    } status_t;

    typedef enum rate {
        SPS_30K =0b11110000, SPS_15K=0b11100000, SPS_7K5=0b11010000, 
        SPS_3K75=0b11000000, SPS_2K =0b10110000, SPS_1K =0b10100001, 
        SPS_500 =0b10010010, SPS_100=0b10000010, SPS_60 =01110010, 
        SPS_50  =0b01100011, SPS_30 =0b01010011, SPS_25 =0b01000011, 
        SPS_15  =0b00110011, SPS_10 =0b00100011, SPS_5  =0b00010011, 
        SPS_2_5 =0b00000011
    } rate_t;

    typedef enum clock_out {
        CO_OFF=0b000000, CO_1=0b010000, CO_2=0b100000, CO_4=0b110000
    } clock_out_t;

    typedef enum detect_current {
        DC_OFF=0b0000, DC_0_5=0b0100, DC_2=0b1000, DC_10=0b1100
    } detect_current_t;

    typedef enum pin {
        PIN_0, PIN_1, PIN_2, PIN_3
    } pin_t;

    typedef struct value {
        uint8_t hi, mid, lo;
    } value_t;

    // Prerequisite: set ready on falling edge of DRDY (too much hassle to put portable interrupt handling in a class...)
    Ads1256( Spi &spi, Delay &delay, volatile bool &ready, uint32_t freq = 7680000UL );

    // state machine
    void begin();  // call to init SPI with max speed if not done elsewhere
    bool update( value_t *value = 0 );  // call in a loop, returns true if new value or reset ready
    void wait();  // return if ready

    // Ads1256 commands with state
    bool command( command_t command );  // if continuous: schedule RESET or SDATAC command for update() else: execute command
    bool rdata();  // schedule rdata for next update(): first with DRDY gets value 
    bool rdatac( value_t &value );  // start continuous mode

    // Ads1256 commands without state
    bool rreg( register_t reg, uint8_t *buffer, uint8_t len ); // read up to 16 bytes
    bool wreg( register_t reg, uint8_t *buffer, uint8_t len ); // write up to 16 bytes 

    // Change settings
    void sps( rate_t rate );
    void clock_out( clock_out_t ratio );
    void detect_current( detect_current_t current );
    void gain( uint8_t power_of_two );
    bool mux( uint8_t ain, uint8_t aout = 8 );
    uint8_t id();
    void auto_calibrate( bool on );
    void buffer( bool on );

    // Read status
    void get_calibration( value_t &offset, value_t &full );
    bool ready();  // DRDY from status register

    // Ads1256 GPIO, alternatively use high level IoPin* classes or low level wreg(IO) with bit mask)
    void io_out( pin_t pin, bool out );
    void io_write( pin_t pin, bool on );
    bool io_read( pin_t pin );

    int32_t one_shot();  // using current settings, from standby -> standby
    int32_t one_shot( uint8_t ain, uint8_t aout = 8, uint8_t gain = 0);  // reset -> standby

    bool bulk_read( value_t *values, uint32_t count );  // Idle -> Idle, using current settings
    // TODO: flag for keep going

    static int32_t to_int( const value_t &value );

private:
    static const uint8_t T6 = 50;
    static const uint8_t T10 = 8;
    static const uint8_t T11a = 4;
    static const uint8_t T11b = 24;

    uint16_t tx_us( uint8_t taus );  // microseconds from T* values

    uint16_t _tau_clkin_ns;  // time of one clock tick (for delays)
    Spi &_spi;  // portable spi handler
    Delay &_delay;  // portable microsecond delays
    volatile bool &_ready;  // externally driven interrupt flag for DRDY
    status_t _status;  // Ads1256 state
};


class IoPinIn : public Ads1256::Pin {
public:
    typedef Ads1256::pin_t pin_t;

    IoPinIn( pin_t id, Ads1256 &ads ) : Ads1256::Pin((uint8_t)id), _ads(ads) {
        init();
    }

    void init() { _ads.io_out((pin_t)id(), false); }  // private?
    bool get() { return _ads.io_read((pin_t)id()); }

protected:
    Ads1256 &_ads;
};

class IoPinOut : public IoPinIn, Ads1256::PinOut {
public:
    IoPinOut( pin_t id, Ads1256 &ads ) : IoPinIn(id, ads) {}

    void init() { _ads.io_out((pin_t)id(), true); }  // private?
    void set() { _ads.io_write((pin_t)id(), true); }
    void reset() { _ads.io_write((pin_t)id(), false); }
    bool toggle() { bool on = get(); if(on) reset(); else set(); return !on; };
};


#ifdef ARDUINO

#include <Arduino.h>
#include <SPI.h>

class ArduinoDelay : public Ads1256::Delay {
public:
    virtual void us( uint16_t us ) {
        delayMicroseconds(us);
    }
};

class ArduinoPin : public Ads1256::Pin {
public:
    ArduinoPin( uint8_t id ) : Ads1256::Pin(id) {
        init();
    }

    void init() { pinMode(id(), INPUT); }
    bool get() { return digitalRead(id()); }
};

class ArduinoPinOut : public ArduinoPin, Ads1256::PinOut {
public:
    ArduinoPinOut( uint8_t id ) : ArduinoPin(id) {
        init();
    }

    void init() { pinMode(id(), OUTPUT); }
    void set() { digitalWrite(id(), HIGH); }
    void reset() { digitalWrite(id(), LOW); }
    bool toggle() { bool on = get(); if(on) reset(); else set(); return !on; };
};

class ArduinoSpi : public Ads1256::Spi {
public:
    ArduinoSpi( uint8_t cs ) : _cs(cs) {
        _cs.set();
    }

    void begin( uint32_t freq ) {
        SPI.begin();
        SPI.beginTransaction(SPISettings(freq, MSBFIRST, SPI_MODE1));
    };

    virtual void start() {
        _cs.reset();     
    };

    virtual void transfer(uint8_t *buffer, uint8_t len) {
        SPI.transferBytes(buffer, buffer, len);
    };
    
    virtual void end() {
        _cs.set();
    };

private:
    ArduinoPinOut _cs;
};

class ArduinoAds1256 : public Ads1256 {
public:
    ArduinoAds1256( ArduinoSpi &spi, ArduinoDelay &delay, volatile bool &ready, uint32_t freq ) :
        Ads1256(spi, delay, ready, freq) {
    }
    ArduinoAds1256( ArduinoSpi &spi, ArduinoDelay &delay, volatile bool &ready ) :
        Ads1256(spi, delay, ready) {
    }
};

#endif


#endif