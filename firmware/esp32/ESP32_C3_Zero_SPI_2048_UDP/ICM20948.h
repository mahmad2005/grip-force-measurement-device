#pragma once
#include <Arduino.h>
#include <SPI.h>

class ICM20948 {
public:
  // Public constants (scale factors)
  static constexpr float GYRO_SENS_250DPS   = 131.0f;    // LSB/°/s
  static constexpr float ACCEL_SENS_2G      = 16384.0f;  // LSB/g
  static constexpr float MAG_SENS_uT_PER_LSB= 0.15f;     // AK09916 ~0.15 µT/LSB
  static constexpr uint8_t WHO_AM_I_VALUE   = 0xEA;

  ICM20948();
  bool begin(uint8_t csPin, SPIClass &spi = SPI);
  bool init();                             // gyro±250, accel±2g, DLPF on
  uint8_t whoAmI();

  // Raw reads (int16)
  void readGyroAccel(int16_t gyro[3], int16_t accel[3]);

  // Engineering units
  void readGyroAccelFloat(float gyro_dps[3], float accel_g[3]);

  // ===== Magnetometer (AK09916) =====
  bool initMag_AK09916();                  // enable I2C master + set AK09916 to continuous
  void readMagnetometer(int16_t mag[3]);   // raw counts
  void readGyroAccelMag(int16_t g[3], int16_t a[3], int16_t m[3]);

private:
  // --- low level SPI helpers ---
  void selectBank(uint8_t bank);
  uint8_t readReg(uint8_t reg);
  void writeReg(uint8_t reg, uint8_t val);
  void readMulti(uint8_t reg, uint8_t *buf, uint16_t len);

  // --- AK09916 helpers (as class methods so they can use private stuff) ---
  void ak_write(uint8_t reg, uint8_t val);
  void ak_read_window(uint8_t onset_reg, uint8_t len);

  // --- internal state ---
  SPIClass   *m_spi;
  uint8_t     m_csPin;
  SPISettings m_settings;

  // --- register/bank defs ---
  enum Bank : uint8_t { BANK_0 = 0x00, BANK_1 = 0x10, BANK_2 = 0x20, BANK_3 = 0x30 };

  // BANK 0
  static constexpr uint8_t REG_WHO_AM_I               = 0x00;
  static constexpr uint8_t REG_USER_CTRL              = 0x03;
  static constexpr uint8_t REG_LP_CONFIG              = 0x05;
  static constexpr uint8_t REG_PWR_MGMT_1             = 0x06;
  static constexpr uint8_t REG_PWR_MGMT_2             = 0x07;
  static constexpr uint8_t REG_ACCEL_XOUT_H           = 0x2D; // then 12B: ax..az, gx..gz
  static constexpr uint8_t REG_EXT_SLV_SENS_DATA_00   = 0x3B; // external sensor data window

  // BANK 2
  static constexpr uint8_t REG_GYRO_SMPLRT_DIV        = 0x00;
  static constexpr uint8_t REG_GYRO_CONFIG_1          = 0x01;
  static constexpr uint8_t REG_ODR_ALIGN_EN           = 0x09;
  static constexpr uint8_t REG_ACCEL_SMPLRT_DIV1      = 0x10;
  static constexpr uint8_t REG_ACCEL_SMPLRT_DIV2      = 0x11;
  static constexpr uint8_t REG_ACCEL_CONFIG           = 0x14;

  // BANK 3 (I2C master)
  static constexpr uint8_t REG_I2C_MST_ODR_CONFIG     = 0x00;
  static constexpr uint8_t REG_I2C_MST_CTRL           = 0x01;
  static constexpr uint8_t REG_I2C_SLV0_ADDR          = 0x03;
  static constexpr uint8_t REG_I2C_SLV0_REG           = 0x04;
  static constexpr uint8_t REG_I2C_SLV0_CTRL          = 0x05;
  static constexpr uint8_t REG_I2C_SLV0_DO            = 0x06;

  // AK09916 (inside ICM on aux I2C)
  static constexpr uint8_t AK09916_I2C_ADDR           = 0x0C;
  static constexpr uint8_t AK09916_CNTL2              = 0x31;
  static constexpr uint8_t AK09916_CNTL3              = 0x32;
};
