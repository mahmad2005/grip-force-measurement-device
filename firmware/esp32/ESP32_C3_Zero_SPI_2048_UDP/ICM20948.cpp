#include "ICM20948.h"

ICM20948::ICM20948()
: m_spi(&SPI),
  m_csPin(255),
  m_settings(1600000, MSBFIRST, SPI_MODE0) {}

bool ICM20948::begin(uint8_t csPin, SPIClass &spi) {
  m_spi = &spi;
  m_csPin = csPin;
  pinMode(m_csPin, OUTPUT);
  digitalWrite(m_csPin, HIGH);
  return true;
}

bool ICM20948::init() {
  // Reset
  selectBank(BANK_0);
  writeReg(REG_PWR_MGMT_1, 0xC1);
  delay(100);

  // Wake & clock
  selectBank(BANK_0);
  writeReg(REG_PWR_MGMT_1, 0x01);

  // Enable gyro+accel
  selectBank(BANK_0);
  writeReg(REG_PWR_MGMT_2, 0x00);

  // ODR align
  selectBank(BANK_2);
  writeReg(REG_ODR_ALIGN_EN, 0x01);

  // Gyro: div=0; DLPF on; ±250 dps
  writeReg(REG_GYRO_SMPLRT_DIV, 0x00);
  writeReg(REG_GYRO_CONFIG_1,   0b00000001);

  // Accel: div=0; DLPF on; ±2 g
  writeReg(REG_ACCEL_SMPLRT_DIV1, 0x00);
  writeReg(REG_ACCEL_SMPLRT_DIV2, 0x00);
  writeReg(REG_ACCEL_CONFIG,      0b00000001);

  // SPI only
  selectBank(BANK_0);
  uint8_t user = readReg(REG_USER_CTRL);
  writeReg(REG_USER_CTRL, user | 0x10);

  // Check WHO_AM_I
  uint8_t who = whoAmI();
  return (who == WHO_AM_I_VALUE);
}

uint8_t ICM20948::whoAmI() {
  selectBank(BANK_0);
  return readReg(REG_WHO_AM_I);
}

void ICM20948::readGyroAccel(int16_t gyro[3], int16_t accel[3]) {
  uint8_t buf[12];
  selectBank(BANK_0);
  readMulti(REG_ACCEL_XOUT_H, buf, sizeof(buf));

  // accel
  accel[0] = (int16_t)((buf[0]  << 8) | buf[1]);
  accel[1] = (int16_t)((buf[2]  << 8) | buf[3]);
  accel[2] = (int16_t)((buf[4]  << 8) | buf[5]);
  // gyro
  gyro[0]  = (int16_t)((buf[6]  << 8) | buf[7]);
  gyro[1]  = (int16_t)((buf[8]  << 8) | buf[9]);
  gyro[2]  = (int16_t)((buf[10] << 8) | buf[11]);
}

void ICM20948::readGyroAccelFloat(float gyro_dps[3], float accel_g[3]) {
  int16_t g[3], a[3];
  readGyroAccel(g, a);
  gyro_dps[0] = g[0] / GYRO_SENS_250DPS;
  gyro_dps[1] = g[1] / GYRO_SENS_250DPS;
  gyro_dps[2] = g[2] / GYRO_SENS_250DPS;
  accel_g[0]  = a[0] / ACCEL_SENS_2G;
  accel_g[1]  = a[1] / ACCEL_SENS_2G;
  accel_g[2]  = a[2] / ACCEL_SENS_2G;
}

// ----------- low-level SPI helpers -----------
void ICM20948::selectBank(uint8_t bank) {
  digitalWrite(m_csPin, LOW);
  m_spi->beginTransaction(m_settings);
  m_spi->transfer(0x7F & 0x7F);      // write
  m_spi->transfer(bank);
  m_spi->endTransaction();
  digitalWrite(m_csPin, HIGH);
}
uint8_t ICM20948::readReg(uint8_t reg) {
  uint8_t v;
  digitalWrite(m_csPin, LOW);
  m_spi->beginTransaction(m_settings);
  m_spi->transfer(reg | 0x80);
  v = m_spi->transfer(0x00);
  m_spi->endTransaction();
  digitalWrite(m_csPin, HIGH);
  return v;
}
void ICM20948::writeReg(uint8_t reg, uint8_t val) {
  digitalWrite(m_csPin, LOW);
  m_spi->beginTransaction(m_settings);
  m_spi->transfer(reg & 0x7F);
  m_spi->transfer(val);
  m_spi->endTransaction();
  digitalWrite(m_csPin, HIGH);
}
void ICM20948::readMulti(uint8_t reg, uint8_t *buf, uint16_t len) {
  digitalWrite(m_csPin, LOW);
  m_spi->beginTransaction(m_settings);
  m_spi->transfer(reg | 0x80);
  for (uint16_t i = 0; i < len; ++i) buf[i] = m_spi->transfer(0x00);
  m_spi->endTransaction();
  digitalWrite(m_csPin, HIGH);
}

// ----------- AK09916 helpers (now class members) -----------
void ICM20948::ak_write(uint8_t reg, uint8_t val) {
  selectBank(BANK_3);
  writeReg(REG_I2C_SLV0_ADDR, (uint8_t)(AK09916_I2C_ADDR & 0x7F)); // write
  writeReg(REG_I2C_SLV0_REG,  reg);
  writeReg(REG_I2C_SLV0_DO,   val);
  writeReg(REG_I2C_SLV0_CTRL, 0x80 | 0x01); // enable, len=1
  delay(50);
}
void ICM20948::ak_read_window(uint8_t onset_reg, uint8_t len) {
  selectBank(BANK_3);
  writeReg(REG_I2C_SLV0_ADDR, (uint8_t)(0x80 | AK09916_I2C_ADDR)); // read
  writeReg(REG_I2C_SLV0_REG,  onset_reg);
  writeReg(REG_I2C_SLV0_CTRL, (uint8_t)(0x80 | (len & 0x0F)));     // enable, length
  delay(50);
}

// ----------- Magnetometer API -----------
bool ICM20948::initMag_AK09916() {
  // I2C master reset+enable
  selectBank(BANK_0);
  uint8_t user = readReg(REG_USER_CTRL);
  writeReg(REG_USER_CTRL, user | 0x02);   // I2C_MST_RST
  delay(100);
  user = readReg(REG_USER_CTRL);
  writeReg(REG_USER_CTRL, user | 0x20);   // I2C_MST_EN
  delay(10);

  // 400kHz I2C, route ODR
  selectBank(BANK_3);
  writeReg(REG_I2C_MST_CTRL, 0x07);       // 400 kHz
  selectBank(BANK_0);
  writeReg(REG_LP_CONFIG, 0x40);          // ODR from I2C_MST_ODR_CONFIG
  selectBank(BANK_3);
  writeReg(REG_I2C_MST_ODR_CONFIG, 0x03); // ~136 Hz

  // AK09916 reset + continuous mode 4 (100 Hz)
  ak_write(AK09916_CNTL3, 0x01);
  delay(10);
  ak_write(AK09916_CNTL2, 0x08);

  // Create continuous read window: start 0x10, len=9 (ST1, HXL..HZH, TMPS, ST2)
  ak_read_window(0x10, 9);

  selectBank(BANK_0);
  return true;
}

void ICM20948::readMagnetometer(int16_t mag[3]) {
  uint8_t buf[9];
  selectBank(BANK_0);
  readMulti(REG_EXT_SLV_SENS_DATA_00, buf, sizeof(buf));
  // HXL,HXH,HYL,HYH,HZL,HZH -> little-endian words
  mag[0] = (int16_t)((buf[2] << 8) | buf[1]);
  mag[1] = (int16_t)((buf[4] << 8) | buf[3]);
  mag[2] = (int16_t)((buf[6] << 8) | buf[5]);
}

void ICM20948::readGyroAccelMag(int16_t g[3], int16_t a[3], int16_t m[3]) {
  readGyroAccel(g, a);
  readMagnetometer(m);
}
