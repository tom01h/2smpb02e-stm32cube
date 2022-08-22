// Arduion Library for Omron 2SMPB02E
// 2019/1/16: akita11 (akita@ifdl.jp)
// Change to STM32Cube Library
// 2022/8/21: tom01h

#include "Omron2SMPB02E.hpp"

#include <math.h>
#include "main.h"

uint8_t Omron2SMPB02E::read_reg(uint8_t addr)
{
  uint8_t data;
  HAL_I2C_Master_Transmit(i2c_ch, i2c_addr<<1, &addr, 1, 1000);
  HAL_I2C_Master_Receive( i2c_ch, i2c_addr<<1, &data, 1, 1000);
  return(data);
}

void Omron2SMPB02E::write_reg(uint8_t addr, uint8_t data)
{
  uint8_t tmp[2];
  tmp[0] = addr;
  tmp[1] = data;
  HAL_I2C_Master_Transmit(i2c_ch, i2c_addr<<1, tmp, 2, 1000);
}

// read {(@addr):(@addr+1)}, 2's complement
int Omron2SMPB02E::read_reg16(uint8_t addr)
{
  uint16_t d = (read_reg(addr) << 8) | read_reg(addr + 1); // [@(addr):@(addr+1)]
  return(-(d & 0b1000000000000000) | (d & 0b0111111111111111)); // 2's complement
  return(d);
  
}

Omron2SMPB02E::Omron2SMPB02E(I2C_HandleTypeDef * hi2c, uint8_t SDO = 1)
{
  i2c_ch = hi2c;
  i2c_addr = 0x56;
  if (SDO == 0) i2c_addr = 0x70;
}

void Omron2SMPB02E::begin()
{
  write_reg(IO_SETUP, 0x00); // IO_SETUP

  // 補正用パラメータ
  //
  //   a0, b00 : OTP / 16
  //     {19:12}   {11:4}    {3:0}
  // a0  COE_a0_1  COE_a0_0  COE_a0_ex
  // b00 COE_b00_1 COE_b00_0 COE_b00_ex
  //
  //   a1, ... : A + (S * OTP) / 32767
  //        A        S        OTP
  // a1    -6e-03    4.3e-04  [COE_a1_1,COE_a1_0]
  // a2    -1.9e-11  1.2e-10  [COE_a2_1,COE_a2_0]
  // bt1   1.0e-01   9.1e-02  [COE_bt1_1,COE_bt1_0]
  // bt2   1.2e-8    1.2e-06  [COE_bt2_1,COE_bt2_0]
  // bp1   3.3e-02   1.9e-02  [COE_bp1_1,COE_bp1_0]
  // b11   2.1e-07   1.4e-07  [COE_b11_1,COE_b11_0]
  // bp2   -6.3e-10  3.5e-10  [COE_bp2_1,COE_bp2_0]
  // b12   2.9e-13   7.6e-13  [COE_b12_1,COE_b12_0]
  // b21   2.1e-15   1.2e-14  [COE_b21_1,COE_b21_0]
  // bp3   1.3e-16   7.9e-17  [COE_bp3_1,COE_bp3_0]

  a0 = ((uint32_t)read_reg(COE_a0_1) << 12) | ((uint32_t)read_reg(COE_a0_0) << 4) | ((uint32_t)read_reg(COE_b00_a0_ex) & 0x0000000f);
  a0 = -(a0 & (uint32_t)1 << 19) + (a0 & ~((uint32_t)1 << 19)); // 2's complement
  a1 = A_a1 + read_reg16(COE_a1) * S_a1/32767.0;
  a2 = A_a2 * read_reg16(COE_a2) * S_a2/32767.0;

  b00 =((uint32_t)read_reg(COE_b00_1) << 12) | ((uint32_t)read_reg(COE_b00_0) << 4) | ((uint32_t)read_reg(COE_b00_a0_ex) >> 4);
  b00 = -(b00 & (uint32_t)1 << 19) | (b00 & ~((uint32_t)1 << 19)); // 2's complement
  bt1 = A_bt1 + read_reg16(COE_bt1) * S_bt1/32767.0;
  b11 = A_b11 + read_reg16(COE_b11) * S_b11/32767.0;
  bt2 = A_bt2 + read_reg16(COE_bt2) * S_bt2/32767.0;
  b12 = A_b12 + read_reg16(COE_b12) * S_b12/32767.0;
  bp1 = A_bp1 + read_reg16(COE_bp1) * S_bp1/32767.0;
  bp2 = A_bp2 + read_reg16(COE_bp2) * S_bp2/32767.0;
  b21 = A_b21 + read_reg16(COE_b21) * S_b21/32767.0;
  bp3 = A_bp3 + read_reg16(COE_bp3) * S_bp3/32767.0;
  
  set_average(AVG_1, AVG_1);

}

uint8_t Omron2SMPB02E::read_id()
{
  return(read_reg(CHIP_ID)); // CHIP_ID, would be 0x5c
}

void Omron2SMPB02E::reset()
{
  write_reg(RESET, 0xe6); // software reset
}

long Omron2SMPB02E::read_raw_temp()
{
  return((((uint32_t)read_reg(TEMP_TXD2) << 16)
	  | ((uint32_t)read_reg(TEMP_TXD1) <<  8)
	  | ((uint32_t)read_reg(TEMP_TXD0)      )) - ((uint32_t)1 << 23));
}

long Omron2SMPB02E::read_raw_pressure()
{
  return((((uint32_t)read_reg(PRESS_TXD2) << 16)
	  | ((uint32_t)read_reg(PRESS_TXD1) <<  8)
	  | ((uint32_t)read_reg(PRESS_TXD0)      )) - ((uint32_t)1 << 23));
}

// read temperature in [degC]
float Omron2SMPB02E::read_temp()
{
  float t = read_calc_temp() / 256.0;
  return(t);
}

float Omron2SMPB02E::read_calc_temp()
{
  // Tr = a0 + a1 * Dt + a2 * Dt^2 
  // -> temp = Re / 256 [degC]
  //   Dt : raw temperature value from TEMP_TXDx reg.

  float dt = read_raw_temp();
  float temp = a0/16.0 + (a1 + a2 * dt) * dt;
  return(temp);
}

// read pressure in [Pa]
float Omron2SMPB02E::read_pressure()
{
  // Pr = b00 + (bt1 * Tr) + (bp1 * Dp) + (b11 * Dp * Tr) + (bt2 * Tr^2)
  //      + (bp2 * Dp^2) + (b12 * Dp * Tr^2) + (b21 * Dp^2 * Tr) + (bp3 * Dp^3)
  //   Tr : raw temperature from TEMP_TXDx reg.
  //   Dp : raw pressure from PRESS_TXDx reg.
  float prs;

  float dp = read_raw_pressure();
  float tr = read_calc_temp();

  float w;
  float w2;
  // Pr = b00 + {(bt1) + (b11 * Dp) + (bt2 * Tr) + (b12 * Dp * Tr)} * Tr
  //      + [(bp1) + {(bp2) + (b21 * Tr) + (bp3 * Dp)} * Dp] * Dp
  prs = b00/16.0;
  w  =  bt1 + b11 * dp + tr * (bt2 + b12 * dp);
  prs += tr * w;
  w = bp1;
  w2 = bp2 + b21 * tr + bp3 * dp;
  w += dp * w2;
  prs += dp * w;
  
  float P0 = 1013.25;
  height = (powf((P0 / (prs / 100.0)), 1.0 / 5.256) -1) * (tr/256.0 + 273.15) / 0.0065;
  
  return(prs / 100.0);
}

float Omron2SMPB02E::read_height()
{
  return(height);
}

void Omron2SMPB02E::set_average(uint8_t temp_avg, uint8_t pressure_avg)
{
  uint8_t r = read_reg(CTRL_MEAS) & 0x03;
  r = r | (temp_avg << 5);
  r = r | (pressure_avg << 2);
  write_reg(CTRL_MEAS, r);
}

void Omron2SMPB02E::set_mode(uint8_t mode)
{
  uint8_t r = read_reg(CTRL_MEAS) & 0xfc;
  r = r | mode;
  write_reg(CTRL_MEAS, r);
}

uint8_t Omron2SMPB02E::is_busy()
{
  if ((read_reg(DEVICE_STAT) & 0x08) == 0) return(0);
  else return(1); // busy
}

void Omron2SMPB02E::set_filter(uint8_t mode)
{
  write_reg(IIR_CNT, mode);
}
