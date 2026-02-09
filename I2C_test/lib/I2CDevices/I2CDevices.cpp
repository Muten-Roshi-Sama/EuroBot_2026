#include "I2CDevices.h"

// --- Low-level helpers ---
static bool writeReg(uint8_t addr, uint8_t reg, uint8_t val) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.write(val);
    return Wire.endTransmission() == 0;
}

static int readRegs(uint8_t addr, uint8_t reg, uint8_t *buf, size_t len) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return -1;
    size_t r = Wire.requestFrom((int)addr, (int)len);
    if (r != len) return -1;
    for (size_t i = 0; i < len; ++i) buf[i] = Wire.read();
    return (int)r;
}

static bool pingAddr(uint8_t addr) {
    Wire.beginTransmission(addr);
    return (Wire.endTransmission() == 0);
}

// --- PCF8574 implementation ---
bool PCF8574::begin(uint8_t addr) {
    _addr = addr;
    return pingAddr(_addr);
}

bool PCF8574::read(uint8_t &value) {
    Wire.requestFrom((int)_addr, 1);
    if (Wire.available()) {
        value = Wire.read();
        return true;
    }
    return false;
}

bool PCF8574::write(uint8_t value) {
    Wire.beginTransmission(_addr);
    Wire.write(value);
    return (Wire.endTransmission() == 0);
}

// --- ADXL345 implementation ---
bool ADXL345::begin(uint8_t addr) {
    _addr = addr;
    // check device whoami by reading DEVID (0x00) should be 0xE5
    uint8_t devid = 0;
    if (readRegisters(0x00, &devid, 1) && devid == 0xE5) {
        // set measure bit in POWER_CTL (0x2D)
        writeRegister(0x2D, 0x08);
        // set full resolution, +/-2g by default
        writeRegister(0x31, 0x08);
        delay(10);
        return true;
    }
    return false;
}

bool ADXL345::readAccel(int16_t &ax, int16_t &ay, int16_t &az) {
    uint8_t buf[6];
    if (!readRegisters(0x32, buf, 6)) return false;
    ax = (int16_t)((buf[1] << 8) | buf[0]);
    ay = (int16_t)((buf[3] << 8) | buf[2]);
    az = (int16_t)((buf[5] << 8) | buf[4]);
    return true;
}

bool ADXL345::writeRegister(uint8_t reg, uint8_t val) {
    return writeReg(_addr, reg, val);
}

bool ADXL345::readRegisters(uint8_t reg, uint8_t *buf, size_t len) {
    return readRegs(_addr, reg, buf, len) == (int)len;
}

