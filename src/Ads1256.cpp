#include <Ads1256.h>

Ads1256Base::Ads1256Base( Spi &spi, Time &delay, volatile bool &ready, uint32_t freq ) : 
    _tau_clkin_ns(1000000000UL / freq), 
    _spi(spi),
    _time(delay),
    _ready(ready),  // set to true if new valid values are available
    _state(DRDY) {
}

void Ads1256Base::begin() {
    _spi.begin(1000000000UL/(_tau_clkin_ns * 4UL));  // pg 6
}

void Ads1256Base::xfer( uint8_t *buffer, uint8_t len ) {
    _spi.start();
    _spi.transfer(buffer, len);
    _time.delay_us(tx_us(T10));   // pg 6
    _spi.end();
    _time.delay_us(tx_us(T11a));  // pg 6
}

void Ads1256Base::command( const command_t cmd ) {
    uint8_t buf = (uint8_t)cmd;
    xfer(&buf, 1);
}

bool Ads1256Base::wakeup() {
    if( _state != SLEEP ) return false;
    command(WAKEUP);
    _ready = false;
    _state = DRDY;
    return true;
}

bool Ads1256Base::xcal( command_t cal ) {
    if( !((_state == DRDY && _ready) || _state == IDLE) ) return false;
    command(cal);
    _ready = false;
    _state = DRDY;
    return true;
}

bool Ads1256Base::selfcal() {
    return xcal(SELFCAL);
}

bool Ads1256Base::selfocal() {
    return xcal(SELFOCAL);
}

bool Ads1256Base::selfgcal() {
    return xcal(SELFGCAL);
}

bool Ads1256Base::sysocal() {
    return xcal(SYSOCAL);
}

bool Ads1256Base::sysgcal() {
    return xcal(SYSGCAL);
}

bool Ads1256Base::sync_wakeup() {
    if( !((_state == DRDY && _ready) || _state == IDLE) ) return false;
    command(SYNC);
    _time.delay_us(tx_us(T11b-T11a));  // pg 6
    command(WAKEUP);
    _ready = false;
    _state = DRDY;
    return true;
}

bool Ads1256Base::standby() {
    if( !((_state == DRDY && _ready) || _state == IDLE) ) return false;
    command(STANDBY);
    _state = SLEEP;
    return true;
}

bool Ads1256Base::rreg( register_t reg, uint8_t *buffer, uint8_t len ) { 
    // !(state==idle) && !(state==drdy && ready)
    if( reg + len - 1 > FSC2 || len == 0 || !((_state == DRDY && _ready) || _state == IDLE) ) return false;
    uint8_t cmd[2];
    cmd[0] = reg | 0x10;
    cmd[1] = len - 1;
    _spi.start();
    _spi.transfer(cmd, 2);
    _time.delay_us(tx_us(T6));    // pg 34
    _spi.transfer(buffer, len);
    _time.delay_us(tx_us(T10));   // pg 6
    _spi.end();
    _time.delay_us(tx_us(T11a));  // pg 6
    return true;
}

// protect this: could set autocal and order -> not supported yet
bool Ads1256Base::wreg( register_t reg, const uint8_t *buffer, uint8_t len ) {
    if( reg + len - 1 > FSC2 || len == 0 || !((_state == DRDY && _ready) || _state == IDLE) ) return false;
    uint8_t cmd[3+FSC2];
    cmd[0] = reg | 0x50;
    cmd[1] = len - 1;
    memcpy(&cmd[2], buffer, len);
    xfer(cmd, 2+len);
    return true;
}

bool Ads1256Base::rdata( value_t &value ) {
    if( !(_state == DRDY && _ready) ) return false;
    uint8_t cmd = RDATA;
    value = {0};
    _spi.start();
    _spi.transfer(&cmd, 1);
    _time.delay_us(tx_us(T6));    // pg 6
    _spi.transfer((uint8_t *)&value, 3);
    _time.delay_us(tx_us(T10));   // pg 6
    _spi.end();
    _time.delay_us(tx_us(T11a));  // pg 6
    _state = IDLE;
    return true;
}

bool Ads1256Base::rdatac( value_t &value ) {
    if( !(_state == DRDY && _ready) ) return false;
    uint8_t cmd = RDATAC;
    value = {0};
    _spi.start();
    _spi.transfer(&cmd, 1);
    _time.delay_us(tx_us(T6));    // pg 6
    _spi.transfer((uint8_t *)&value, 3);
    _time.delay_us(tx_us(T10));   // pg 6
    _spi.end();
    _time.delay_us(tx_us(T11b));  // pg 6
    _ready = false;
    _state = CONT;
    return true;
}

bool Ads1256Base::read( value_t &value, bool last ) {
    if( _state != CONT || !_ready ) return false;
    value = {0};
    if( last ) {
        value.lo = SDATAC;
        _state = IDLE;
    }
    _spi.start();
    _spi.transfer((uint8_t *)&value, 3);
    _time.delay_us(tx_us(T10));  // pg 6
    _spi.end();
    _ready = false;
    return true;
}

bool Ads1256Base::sdatac() {
    if( _state != CONT ) return false;
    command(SDATAC);
    _state = IDLE;
    return true;
}

bool Ads1256Base::sps( rate_t rate ) {
    return wreg(DRATE, (uint8_t *)&rate, 1);
}

bool Ads1256Base::clock_out( clock_out_t ratio ) {
    uint8_t adcon;
    if( rreg(ADCON, &adcon, 1) ) {
        if( (adcon & 0b1100000) == ratio ) return true;
        adcon &= ~(0b1100000);
        adcon |= (uint8_t)ratio;
        return wreg(ADCON, &adcon, 1);
    }
    return false;
}

bool Ads1256Base::detect_current( detect_current_t current ) {
    uint8_t adcon;
    if( rreg(ADCON, &adcon, 1) ) {
        if( (adcon & 0b11000) == current ) return true;
        adcon &= ~(0b11000);
        adcon |= (uint8_t)current;
        return wreg(ADCON, &adcon, 1);
    }
    return false;
}

bool Ads1256Base::gain( uint8_t power_of_two ) {
    if( power_of_two <= 7 ) {
        uint8_t adcon;
        if( rreg(ADCON, &adcon, 1) ) {
            if( (adcon & 0b111) == power_of_two ) return true;
            adcon &= ~(0b111);
            adcon |= power_of_two;
            return wreg(ADCON, &adcon, 1);
        }
    }
    return false;
}

bool Ads1256Base::mux( uint8_t ain, uint8_t aout ) {
    if( ain > 8 || aout > 8 || ain == aout ) return false;
    aout |= ain << 4;
    return wreg(MUX, &aout, 1);
}

uint8_t Ads1256Base::id() {
    uint8_t status;
    if( rreg(STATUS, &status, 1) ) return status >> 4;
    return 0xff;
}

// if auto calibrate is on, calibration- and many wreg commands need to manually wait for DRDY
bool Ads1256Base::auto_calibrate( bool on ) {
    uint8_t status;
    if( rreg(STATUS, &status, 1) && on == !(status & 0b100) ) {
        if( on ) {
            status |= 0b100;
        }
        else {
            status &= ~0b100;
        }
        return wreg(STATUS, &status, 1);
    }
    return false;
}

bool Ads1256Base::get_calibration( value_t &offset, value_t &full ) {
    uint8_t buf[6];
    if( rreg(OFC0, buf, sizeof(buf)) ) {
        offset.lo = buf[0];
        offset.mid = buf[1];
        offset.hi = buf[2];
        full.lo = buf[3];
        full.mid = buf[4];
        full.hi = buf[5];
        return true;
    }
    return false;
}

bool Ads1256Base::buffer( bool on ) {
    uint8_t status;
    if( rreg(STATUS, &status, 1) && on == !(status & 0b10) ) {
        if( on ) {
            status |= 0b10;
        }
        else {
            status &= ~0b10;
        }
        return wreg(STATUS, &status, 1);
    }
    return false;
}

bool Ads1256Base::ready() {
    uint8_t status;
    rreg(STATUS, &status, 1);
    return status & 1;
}


void Ads1256Base::io_out( pin_t pin, bool out ) {
    uint8_t io;
    rreg(IO, &io, 1);
    uint8_t mask = 1 << ((uint8_t)pin + 4);
    if( out != !(io & mask) ) {
        if( out ) {
            io &= ~mask;
        }
        else {
            io |= mask;
        }
        wreg(IO, &io, 1);
    }
}

void Ads1256Base::io_write( pin_t pin, bool on ) {
    uint8_t io;
    rreg(IO, &io, 1);
    uint8_t mask = 1 << (uint8_t)pin;
    bool is_out = !(io & (1 << ((uint8_t)pin + 4)));
    if( on != !(io & mask) && is_out ) {
        if( on ) {
            io |= mask;
        }
        else {
            io &= ~mask;
        }
        wreg(IO, &io, 1);
    }
}

bool Ads1256Base::io_read( pin_t pin ) {
    uint8_t io;
    rreg(IO, &io, 1);
    return io & (1 << (uint8_t)pin);
}


int32_t Ads1256Base::to_int( const value_t &value ) {
    return (value.hi << 16) | (value.mid << 8) | value.lo;
}

int32_t Ads1256Base::to_microvolts( int32_t raw, uint8_t gain, int32_t uvRef ) {
    if( raw == 8388607 ) return INT32_MAX;
    if( raw == -8388608 ) return INT32_MIN; 
    // int64_t uvModulatorRange = uvRef * 2;                                 // full voltage range the modulator can measure
    // int64_t uvModulatorIn = (uvModulatorRange * raw) / ((1L << 23) - 1);  // maps raw (-2^23 ... +2^23-1) to VinDiff (0 .. uvFullRange)
    // return uvModulatorIn >> gain;                                         // finally reduce voltage value by gain
    // equivalent calculation:
	return (((int64_t)raw * uvRef) / ((1LL << 22) - 1)) >> gain;  // remains: add AinN if it is not 0 
}

uint16_t Ads1256Base::tx_us( uint8_t taus ) {
    return (taus * _tau_clkin_ns + 999) / 1000;  
}


bool Ads1256Base::wait( uint32_t timeout_ms ) {
    if( _state == SLEEP ) {
        wakeup();
    }
    else if ( _state == IDLE ) {
        sync_wakeup();
    }
    uint32_t start_ms = _time.now_ms();
    while( !_ready ) {
        if( _time.now_ms() - start_ms > timeout_ms ) return false;
        yield();
    }
    return true;
}

void Ads1256Base::reset( uint32_t timeout_ms ) {
    if( (_state == DRDY || _state == CONT) && !_ready ) wait(timeout_ms);
    command(RESET);
    _state = DRDY;
    _ready = false;
}

// assumes not to be in continuous read mode
int32_t Ads1256Base::read_once( uint32_t timeout_ms ) {
    value_t value;
    if( _state != CONT && wait(timeout_ms) && rdata(value) ) {
        return to_int(value);
    }
    return INT32_MIN;
}

// assumes nothing
int32_t Ads1256Base::read_once( uint8_t ain, uint8_t aout, uint8_t gain_power, uint32_t timeout_ms ) {
    if( _state == CONT ) wait(timeout_ms);  // pg 37
    reset();
    if( wait(timeout_ms) && mux(ain, aout) && gain(gain_power) && sync_wakeup() ) {
        return read_once();
    }
    return INT32_MIN;
}

bool Ads1256Base::read_bulk( value_t *values, uint32_t count, bool once, uint32_t timeout_ms ) {
    if( count == 0 ) return true;  // not reading never fails
    if( count == 1 && (_state != CONT || once) ) return false;  // start or end continuous mode needs one reading
    if( count == 2 && (_state != CONT && once) ) return false;  // start and end continuous mode needs two readings
    if( _state != CONT && wait(timeout_ms) && rdatac(*(values++))) {
        --count;
    }
    else {
        return false;  // could not start continuous mode
    }
    while( --count ) {
        if( !wait(timeout_ms) || !read(*(values++)) ) return false;  // read failed
    }
    return wait(timeout_ms) && read(*values, once);  // final read ends continuous mode if once
}

bool Ads1256Base::read_swipe( value_t *values, uint8_t *ains, uint8_t *aouts, uint32_t count, bool first, uint32_t timeout_ms ) {
    if( count == 0 ) return true;  // doing nothing always succeeds
    if( first ) {  // set channel mux for first reading
        if( !mux(ains[0], aouts ? aouts[0] : 8) ) return false;
    }
    for( size_t i = 0; i < count; i++ ) {
        size_t next = i + 1;
        if( next == count ) next = 0;  // cycle, so next call to read_swipe, first can be false
        if( !wait(timeout_ms) ) return false;
        if( !mux(ains[next], aouts ? aouts[next] : 8) ) return false;
        if( !rdata(values[i]) ) return false; 
    }
    return true;
}
