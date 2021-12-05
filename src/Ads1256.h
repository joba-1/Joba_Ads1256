#ifndef Ads1256_h
#define Ads1256_h

/*
ADS1256 library 

The current implementation uses the Arduino framework, but the goal is to be easily portable.
Testing was done on an ESP32. To port to another µC only the interface classes 
Ads1256Base::Time, Ads1256Base::Spi and Ads1256Base::Pin need to be implemented and
a boolean needs to be set to true on a falling edge of the Ads1256 DRDY line (best via interrupt).

Status: 
* The library is aware of the chip state, e.g. you cannot read data if drdy didn't go low yet to signal valid data
* All chip features are supported, except for data output bit order and auto calibration
  * output bit order is simply not tested yet (don't see the usecase)
  * auto calibration changes the state machine of the chip and would require monitoring registers (too much hassle)
* IO pins of the Ads1256 can be managed via register IO, with io_out()/io_read()/io_write() functions
  or via classes IoPinIn and IoPinOut, your choice.
* There are convenience functions for reading values available
  * read a single value using current settings
  * read a single value, starting with a reset and setting mux and gain
  * bulk readings into a buffer of values from a single channel
  * swipe readings over all given muxes, one after another into a buffer of values
* Performance
  * bulk read: ~29800 SPS
  * swipe read over all channels: ~3440 SPS
*/

#include <stdint.h>
#include <stdbool.h>


class Ads1256Base {
public:
    // Platform independent interface for microseconds
    class Time {
    public:
        virtual void delay_us( uint16_t us ) = 0;
        virtual void delay_ms( uint16_t ms ) = 0;  // yields, if supported
        virtual uint32_t now_ms() = 0;
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
        WAKEUP, RDATA, RDATAC = 0x03, SDATAC = 0x0f, RREG = 0x10, WREG = 0x50, SELFCAL = 0xf0, 
        SELFOCAL, SELFGCAL, SYSOCAL, SYSGCAL, SYNC = 0xfc, STANDBY, RESET
    } command_t;

    typedef enum state {
        DRDY,    // wait for one DRDY low: ready -> true
        IDLE,    // nothing pending
        CONT,    // wait for continuous DRDY lows: ready -> true
        SLEEP    // wait for WAKEUP
    } state_t;

    typedef enum rate {
        SPS_30K  = 0b11110000, SPS_15K = 0b11100000, SPS_7K5 = 0b11010000, 
        SPS_3K75 = 0b11000000, SPS_2K  = 0b10110000, SPS_1K  = 0b10100001, 
        SPS_500  = 0b10010010, SPS_100 = 0b10000010, SPS_60  = 0b01110010, 
        SPS_50   = 0b01100011, SPS_30  = 0b01010011, SPS_25  = 0b01000011, 
        SPS_15   = 0b00110011, SPS_10  = 0b00100011, SPS_5   = 0b00010011, 
        SPS_2_5  = 0b00000011
    } rate_t;

    typedef enum clock_out {
        CO_OFF = 0b000000, CO_1 = 0b010000, CO_2 = 0b100000, CO_4 = 0b110000
    } clock_out_t;

    typedef enum detect_current {
        DC_OFF = 0b0000, DC_0_5 = 0b0100, DC_2 = 0b1000, DC_10 = 0b1100
    } detect_current_t;

    typedef enum pin {
        PIN_0, PIN_1, PIN_2, PIN_3
    } pin_t;

    typedef struct value {
        int8_t hi; uint8_t mid, lo;
    } value_t;

    // Default values that depend on parameters external to the Ads1256
    static const uint32_t FREQUENCY  = 7680000UL;  // clock frequency
    static const int32_t  UV_REF     = 2500000UL;  // reference (micro)voltage 
    static const uint32_t TIMEOUT_MS = 900;        // default timeout (> calibration time at 2.5Hz)

    // Prerequisite: set ready on falling edge of DRDY (too much hassle to put portable interrupt handling in a class...)
    Ads1256Base( Spi &spi, Time &delay, volatile bool &ready, uint32_t freq = FREQUENCY );

    // SPI bus and IRQ line
    void begin();  // call to init SPI with max speed if not done elsewhere
    bool wait( uint32_t timeout_ms = TIMEOUT_MS );  // return true when ready or false if nothing to wait for or timeout

    // Ads1256 commands including verifying ads chip state
    void reset( uint32_t timeout_ms = TIMEOUT_MS );
    bool selfcal();
    bool selfocal();
    bool selfgcal();
    bool sysocal();
    bool sysgcal();
    bool wakeup();
    bool standby();
    bool sync_wakeup();
    bool rdata( value_t &value );   // read single value
    bool rdatac( value_t &value );  // start continuous mode with first value
    bool read( value_t &value, bool last = false );  // read value in continuous mode, stop continuous mode if last
    bool sdatac();  // end continuous read discarding last value 
    bool rreg( register_t reg, uint8_t *buffer, uint8_t len );        // read registers, up to 11 bytes
    bool wreg( register_t reg, const uint8_t *buffer, uint8_t len );  // write registers, up to 11 bytes 

    // Change settings (while chip is idle)
    bool sps( rate_t rate );
    bool clock_out( clock_out_t ratio );
    bool detect_current( detect_current_t current );
    bool gain( uint8_t power_of_two );
    bool mux( uint8_t ain, uint8_t aout = 8 );
    bool auto_calibrate( bool on );
    bool buffer( bool on );

    // Read status (while chip is not in continuous mode)
    uint8_t id();  // 0xff on error
    bool get_calibration( value_t &offset, value_t &full );
    bool ready();  // DRDY from status register

    // Ads1256 GPIO, alternatively use high level IoPin* classes or low level wreg(IO) with bit masks)
    void io_out( pin_t pin, bool out );
    void io_write( pin_t pin, bool on );
    bool io_read( pin_t pin );

    // Convenience command series handling ads state, if possible 
    int32_t read_once( uint32_t timeout_ms = TIMEOUT_MS );  // read once, using current settings
    int32_t read_once( uint8_t ain, uint8_t aout = 8, uint8_t gain = 0, uint32_t timeout_ms = TIMEOUT_MS );  // init everything from reset -> idle
    bool read_bulk( value_t *values, uint32_t count, bool once = true, uint32_t timeout_ms = TIMEOUT_MS );  // Idle -> Idle, using current settings
    bool read_swipe( value_t *values, uint8_t *ains, uint8_t *aouts, uint32_t count, bool first = true, uint32_t timeout_ms = TIMEOUT_MS );  // Idle -> Idle, using current settings

    // Convert signed 24bit struct to signed 32bit
    static int32_t to_int( const value_t &value );
    // AinP voltage relative to AinN (with gain factor = 2^gain and uvRef = VrefP - VrefN) 
    static int32_t to_microvolts( int32_t raw, uint8_t gain = 0, int32_t uvRef = UV_REF );

private:
    // Chip SPI timings in terms of chip frequency [1/fclkin]
    static const uint8_t T6 = 50;    // pg 6: delay between sending command and requesting result
    static const uint8_t T10 = 8;    // pg 6: keep chip selected after last clock
    static const uint8_t T11a = 4;   // pg 6: minimal clock gap between commands RREG, WREG or RDATA
    static const uint8_t T11b = 24;  // pg 6: minimal clock gap between commands RDATAC, SYNC

    uint16_t tx_us( uint8_t taus );             // microseconds from T* values
    void xfer( uint8_t *buffer, uint8_t len );  // spi transfer buffer (bidirectional, sends then reads buffer)
    void command( command_t cmd );              // execute a command, not checking chip state
    bool xcal( command_t cal );                 // calibrate -> DRDY

    uint16_t _tau_clkin_ns;  // time of one clock tick (for delays)
    Spi &_spi;               // portable spi handler interface
    Time &_time;             // portable microsecond delays interface
    volatile bool &_ready;   // externally driven interrupt flag for DRDY
    state_t _state;          // chip status (state engine, see Ads1256.dot)
};


class IoPinIn : public Ads1256Base::Pin {
public:
    typedef Ads1256Base::pin_t pin_t;

    IoPinIn( pin_t id, Ads1256Base &ads ) : Ads1256Base::Pin((uint8_t)id), _ads(ads) {
        init();
    }

    void init() { _ads.io_out((pin_t)id(), false); }  // only needed if pin was reused
    bool get()  { return _ads.io_read((pin_t)id()); }

protected:
    Ads1256Base &_ads;
};

class IoPinOut : public IoPinIn, Ads1256Base::PinOut {
public:
    IoPinOut( pin_t id, Ads1256Base &ads ) : IoPinIn(id, ads) {}

    void init()   { _ads.io_out((pin_t)id(), true); }  // only needed if pin was reused
    void set()    { _ads.io_write((pin_t)id(), true); }
    void reset()  { _ads.io_write((pin_t)id(), false); }
    bool toggle() { bool on = get(); if(on) reset(); else set(); return !on; };
};


#ifdef ARDUINO

#include <Arduino.h>
#include <SPI.h>

class Ads1256 : public Ads1256Base {
public:

    class Time : public Ads1256Base::Time {
    public:
        virtual void delay_us( uint16_t us ) {
            delayMicroseconds(us);
        }

        virtual void delay_ms( uint16_t ms ) {
            delay(ms);
        }

        virtual uint32_t now_ms() {
            return millis();
        }
    };

    class Pin : public Ads1256Base::Pin {
    public:
        Pin( uint8_t id ) : Ads1256Base::Pin(id) {
            init();
        }

        void init() { pinMode(id(), INPUT); }
        bool get()  { return digitalRead(id()); }
    };

    class PinOut : public Pin, Ads1256Base::PinOut {
    public:
        PinOut( uint8_t id ) : Pin(id) {
            init();
        }

        void init()   { pinMode(id(), OUTPUT); }
        void set()    { digitalWrite(id(), HIGH); }
        void reset()  { digitalWrite(id(), LOW); }
        bool toggle() { bool on = get(); if(on) reset(); else set(); return !on; };
    };

    class Spi : public Ads1256Base::Spi {
    public:
        Spi( uint8_t cs ) : _cs(cs) {
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
        PinOut _cs;
    };

    Ads1256( Spi &spi, Time &delay, volatile bool &ready, uint32_t freq ) :
        Ads1256Base(spi, delay, ready, freq) {
    }
    Ads1256( Spi &spi, Time &delay, volatile bool &ready ) :
        Ads1256Base(spi, delay, ready) {
    }
};

#endif


#endif
