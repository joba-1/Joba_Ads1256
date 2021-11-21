#include <Ads1256.h>

Ads1256::Ads1256( Ads1256::Spi &spi, Ads1256::Delay &delay, volatile bool &ready, uint32_t freq ) : 
    _tau_clkin_ns(1000000000UL / freq), 
    _spi(spi),
    _delay(delay),
    _ready(ready),  // set to true if new valid values are available
    _status(IDLE) {
}

void Ads1256::begin() {
    _spi.begin(1000000000UL/(_tau_clkin_ns * 4UL));  // pg 6
}

bool Ads1256::update( value_t *value ) {
    if( _ready && _status != IDLE ) {
        switch( _status ) {
            case SINGLE:
                if( value ) {
                    value->hi = 0x01;  // RDATA
                    _spi.start();
                    _spi.transfer(&value->hi, 1);
                    _delay.us(tx_us(T6));    // pg 6
                    _spi.transfer((uint8_t *)value, 3);
                    _delay.us(tx_us(T10));   // pg 6
                    _spi.end();
                    _delay.us(tx_us(T11a));  // pg 6
                }
                _status = IDLE;
                break;
            case STOP:
                if( value ) {
                    value->hi = 0x0f;  // SDATAC
                }
                _status = IDLE;
            case CONT:
                if( value ) {
                    _spi.start();
                    _spi.transfer((uint8_t *)value, 3);
                    _delay.us(tx_us(T10));  // pg 6
                    _spi.end();
                }
                break;
            case RESTART:
                command(RESET);
                _status = INIT;
                break;
            case INIT:
                _status = IDLE;
            case IDLE:
                break;
        }
        _ready = false;
        return true;
    }
    return false;
}

void Ads1256::wait() {
    while( !update() ) yield();
}

bool Ads1256::command( const command_t cmd ) {
    if( _status == CONT ) {
        switch( cmd ) {
            case RESET:
                _status = RESTART;
                break;
            case SDATAC:
                _status = STOP;
                break;
            default:
                return false;
        }
    }
    else {  // ! CONT
        if ( cmd == SDATAC ) {
            return false;
        }
        uint8_t buf = (uint8_t)cmd;
        _spi.start();
        _spi.transfer(&buf, 1);
        _delay.us(tx_us(T10));  // pg 6
        _spi.end();
        if( cmd == SYNC ) {
            _delay.us(tx_us(T11b));  // pg 6
        }
        else {
            _delay.us(tx_us(T11a));  // pg 6
            if( cmd != STANDBY && cmd != WAKEUP ) {
                _status = INIT;
                _ready = false;
            }
        }
    }
    return true;
}

bool Ads1256::rdata() {
    if( _status == IDLE || _status == INIT ) {
        _status = SINGLE;
        _ready = false;
        return true;
    }
    return false;
}

bool Ads1256::rdatac( value_t &value ) {
    if( _status != CONT && _status != INIT ) {
        if( _status != STOP && _status != RESTART ) {
            _spi.start();
            value.hi = 0x03;  // RDATAC
            _spi.transfer(&value.hi, 1);
            _delay.us(tx_us(T6));    // pg 6
            _spi.transfer((uint8_t *)&value, 3);
            _delay.us(tx_us(T10));   // pg 6
            _spi.end();
            _delay.us(tx_us(T11b));  // pg 6
            _ready = false;
        }
        _status = CONT;
        return true;
    }
    return false;
}

bool Ads1256::rreg( register_t reg, uint8_t *buffer, uint8_t len ) {
    if( len > 16 ) return false;
    uint8_t cmd[2] = { (uint8_t)reg | 0x10, len - 1 };
    _spi.start();
    _spi.transfer(cmd, sizeof(cmd));
    _delay.us(tx_us(T6));    // pg 6
    _spi.transfer(buffer, len);
    _delay.us(tx_us(T10));   // pg 6
    _spi.end();
    _delay.us(tx_us(T11a));  // pg 6
    return true;
}

bool Ads1256::wreg( register_t reg, uint8_t *buffer, uint8_t len ) {
    if( len > 16 ) return false;
    uint8_t cmd[2] = { (uint8_t)reg | 0x50, len - 1 };
    _spi.start();
    _spi.transfer(cmd, sizeof(cmd));
    _spi.transfer(buffer, len);
    _delay.us(tx_us(T10));   // pg 6
    _spi.end();
    _delay.us(tx_us(T11a));  // pg 6
    return true;
}

void Ads1256::sps( rate_t rate ) {
    wreg(DRATE, (uint8_t *)&rate, 1);
}

void Ads1256::clock_out( clock_out_t ratio ) {
    uint8_t adcon;
    rreg(ADCON, &adcon, 1);
    adcon &= ~(0b1100000);
    adcon |= (uint8_t)ratio;
    wreg(ADCON, &adcon, 1);
}

void Ads1256::detect_current( detect_current_t current ) {
    uint8_t adcon;
    rreg(ADCON, &adcon, 1);
    adcon &= ~(0b11000);
    adcon |= (uint8_t)current;
    wreg(ADCON, &adcon, 1);
}

void Ads1256::gain( uint8_t power_of_two ) {
    if( power_of_two > 7 ) power_of_two = 7;
    uint8_t adcon;
    rreg(ADCON, &adcon, 1);
    adcon &= ~(0b111);
    adcon |= power_of_two;
    wreg(ADCON, &adcon, 1);
}

bool Ads1256::mux( uint8_t ain, uint8_t aout ) {
    if( ain > 8 || aout > 8 || ain == aout ) return false;
    aout |= ain << 4;
    wreg(MUX, &aout, 1);
    return true;
}

uint8_t Ads1256::id() {
    uint8_t status;
    rreg(STATUS, &status, 1);
    return status >> 4;
}

void Ads1256::auto_calibrate( bool on ) {
    uint8_t status;
    rreg(STATUS, &status, 1);
    if( on == !(status & 0b100) ) {
        if( on ) {
            status |= 0b100;
        }
        else {
            status &= ~0b100;
        }
        wreg(STATUS, &status, 1);
    }
}

void Ads1256::get_calibration( value_t &offset, value_t &full ) {
    uint8_t buf[6];
    rreg(OFC0, buf, sizeof(buf));
    offset.lo = buf[0];
    offset.mid = buf[1];
    offset.hi = buf[2];
    full.lo = buf[3];
    full.mid = buf[4];
    full.hi = buf[5];
}

void Ads1256::buffer( bool on ) {
    uint8_t status;
    rreg(STATUS, &status, 1);
    if( on == !(status & 0b10) ) {
        if( on ) {
            status |= 0b10;
        }
        else {
            status &= ~0b10;
        }
        wreg(STATUS, &status, 1);
    }
}

bool Ads1256::ready() {
    uint8_t status;
    rreg(STATUS, &status, 1);
    return status & 1;
}

void Ads1256::io_out( pin_t pin, bool out ) {
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

void Ads1256::io_write( pin_t pin, bool on ) {
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

bool Ads1256::io_read( pin_t pin ) {
    uint8_t io;
    rreg(IO, &io, 1);
    return io & (1 << (uint8_t)pin);
}

int32_t Ads1256::to_int( const value_t &value ) {
    return ((int32_t)value.hi << 16) | (value.mid << 8) | value.lo;
}

uint16_t Ads1256::tx_us( uint8_t taus ) {
    return (taus * _tau_clkin_ns + 999) / 1000;  
}

// assumes to be in standby
int32_t Ads1256::one_shot() {
    if( command(Ads1256::WAKEUP) && rdata() ) {
        rdata();
        value_t value;
        while( !update(&value) ) {
            yield();
        }
        command(Ads1256::STANDBY);
        return to_int(value);
    }
    return INT32_MIN;
}

// assumes not to be in continuous read mode
int32_t Ads1256::one_shot( uint8_t ain, uint8_t aout, uint8_t gain_power) {
    if( command(Ads1256::RESET) ) {
        wait();
        if( mux(ain, aout) ) {
            gain(gain_power);
            if( command(Ads1256::SELFCAL) ) {
                wait();
                if( command(Ads1256::SYNC) && command(Ads1256::WAKEUP) ) {
                    rdata();
                    value_t value;
                    while( !update(&value) ) {
                        yield();
                    }
                    command(Ads1256::STANDBY);
                    return to_int(value);
                }
            }
        }
    }
    return INT32_MIN;
}

// assumes idle or cont
bool Ads1256::bulk_read( value_t *values, uint32_t count, bool once ) {
    if( (_status == CONT && count) || (rdatac(*(values++)) && count--) ) {
        while( count-- > 1 ) {
            while( !update(values) ) {
                yield();
            }
            values++;
        }
        if( !once || command(SDATAC) ) {
            while( !update(values) ) {
                yield();
            }
            return true;
        }
    }
    return false;
}
