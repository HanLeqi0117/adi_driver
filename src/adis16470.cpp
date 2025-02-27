// Copyright (c) 2017, Analog Devices Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
// * Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in
//   the documentation and/or other materials provided with the
//   distribution.
//
// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
// INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
// HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
// STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
// OF THE POSSIBILITY OF SUCH DAMAGE.

#include "rclcpp/rclcpp.hpp"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/types.h>
#include <unistd.h>
#include <stdint.h>
#include <string>
#include "adi_driver/adis16465.h"

/**
 * @brief change big endian 2 byte into short
 * @param data Head pointer to the data
 * @retrun converted value
 */
int16_t big_endian_to_short(unsigned char *data)
{
  unsigned char buff[2] = {data[1], data[0]};
  return *reinterpret_cast<int16_t*>(buff);
}

/**
 * @brief change big endian 2 byte into short
 * @param data Head pointer to the data
 * @retrun converted value
 */
void short_to_big_endian(unsigned char *buff, int16_t data)
{
  buff[0] = data >> 8;
  buff[1] = data & 0x00ff;
}

/**
 * @brief Constructor
 */
Adis16465::Adis16465()
  : fd_(-1)
{
}

/**
 * @brief Open device
 * @param device Device file name (/dev/ttyACM*)
 * @retval 0 Success
 * @retval -1 Failure
 */
int Adis16465::openPort(const std::string device)
{
  fd_ = open(device.c_str(), O_RDWR | O_NOCTTY);
  if (fd_ < 0)
  {
    perror("openPort");
    return -1;
  }
  if (tcgetattr(fd_, &defaults_) < 0)
  {
    perror("openPort");
    return -1;
  }
  struct termios config;
  cfmakeraw(&config);
  if (tcsetattr(fd_, TCSANOW, &config) < 0)
  {
    perror("openPort");
    return -1;
  }
  // Set SPI mode
  unsigned char buff[20] = {0};
  buff[0] = 0x5A;
  buff[1] = 0x02;  // Set mode command
  buff[2] = 0x93;  // Set SPI mode default: 0x93
  buff[3] = 5;  // 1MHz clock speed default: 5

  int size = write(fd_, buff, 4);
  if (size != 4)
  {
    perror("openPort");
  }
  if (tcdrain(fd_) < 0)
  {
    perror("openPort");
  }
  size = read(fd_, buff, 2);
  if (size != 2)
  {
    perror("openPort");
    return -1;
  }
  // Check first byte
  if (buff[0] != 0xff)
  {
    perror("openPort");
    return -1;
  }
  return 0;
}

/**
 * @brief Close device
 */
void Adis16465::closePort()
{
  if (tcsetattr(fd_, TCSANOW, &defaults_) < 0)
  {
    perror("closePort");
  }
  close(fd_);
}

/**
 * @param data Product ID (0x4056)
 * @retval 0 Success
 * @retval -1 Failed
 */
int Adis16465::get_product_id(int16_t& pid)
{
  // get product ID
  int r;
  (void) r;
  unsigned char buff[20];

  // Sending data
  buff[0] = 0x61;
  buff[1] = 0x72;
  buff[2] = 0x00;
  int size = write(fd_, buff, 3);
  if (size != 3)
  {
    perror("get_product_id");
    return -1;
  }
  if (tcdrain(fd_) < 0)
  {
    perror("get_product_id");
    return -1;
  }
  size = read(fd_, buff, 3);
  if (size != 3)
  {
    perror("get_product_id");
    return -1;
  }
  // Receiving data
  buff[0] = 0x61;
  buff[1] = 0x00;
  buff[2] = 0x00;
  size = write(fd_, buff, 3);
  if (size != 3)
  {
    perror("get_product_id");
    return -1;
  }
  if (tcdrain(fd_) < 0)
  {
    perror("get_product_id");
    return -1;
  }
  size = read(fd_, buff, 3);
  if (size != 3)
  {
    perror("get_product_id");
    return -1;
  }
  // Convert to short
  pid = big_endian_to_short(&buff[1]);
  return 0;
}

/**
 * @brief Read data from the register
 * @param address Register address
 * @retval 0 Success
 * @retval -1 Failed
 * 
 * - Adress is the first byte of actual address
 * - Actual data at the adress will be returned by next call.
 */
int Adis16465::read_register(char address, int16_t& data)
{
  unsigned char buff[3] = {0x61, (unsigned char)address, 0x00};
  int size = write(fd_, buff, 3);
  if (size != 3)
  {
    perror("read_register");
    return -1;
  }
  if (tcdrain(fd_) < 0)
  {
    perror("read_register");
    return -1;
  }
  size = read(fd_, buff, 3);
  if (size !=3)
  {
    perror("read");
  }
  data = big_endian_to_short(&buff[1]);

  return 0;
}

/**
 * @brief Write data to the register
 * @param address Register address
 * @retval 0 Success
 * @retval -1 Failed
 * 
 * - Adress is the first byte of actual address.
 * - Specify data at the adress.
 */
int Adis16465::write_register(char address, int16_t data)
{
  unsigned char buff[5] = {0x61, 0x00, 0x00, 0x00, 0x00};
  // Set R~/W bit 1
  buff[1] = address | 0x80;
  buff[3] = (address + 1) | 0x80;
  // Set data
  buff[2] = data & 0xff;
  buff[4] = data >> 8;

  int size = write(fd_, buff, sizeof(buff));
  if (size != sizeof(buff))
  {
    perror("write_register");
    return -1;
  }
  if (tcdrain(fd_) < 0)
  {
    perror("write_register");
    return -1;
  }
  unsigned char recv_buff[5] = {0, 0, 0, 0, 0};
  size = read(fd_, recv_buff, sizeof(recv_buff));
  if (size != sizeof(recv_buff))
  {
    perror("write_register");
    return -1;
  }
  if (recv_buff[0] != 0xff)
  {
    perror("write_register: ACK error");
    return -1;
  }
  return 0;
}

/**
 * @brief Update all information by bust read
 * @retval 0 Success
 * @retval -1 Failed
 * 
 * - See burst read function at pp.14 
 * - Data resolution is 16 bit
 */
int Adis16465::update_burst(void)
{
  unsigned char buff[64] = {0};
  // 0x6800: Burst read function
  buff[0] = 0x61;
  buff[1] = 0x68;
  buff[2] = 0x00;
  int size = write(fd_, buff, 24);
  if (size != 24)
  {
    perror("update_burst");
    return -1;
  }
  if (tcdrain(fd_) < 0)
  {
    perror("update_burst");
    return -1;
  }
  size = read(fd_, buff, 30);
  if (size != 30)
  {
    perror("update_burst");
    return -1;
  }
  int16_t diag_stat = big_endian_to_short(&buff[3]);
  if (diag_stat != 0)
  {
    fprintf(stderr, "diag_stat error: %04x\n", (uint16_t)diag_stat);
    return -1;
  }
  // X_GYRO_OUT
  gyro[0] = big_endian_to_short(&buff[5]) * M_PI / 180 / 10.0;
  // Y_GYRO_OUT
  gyro[1] = big_endian_to_short(&buff[7]) * M_PI / 180 / 10.0;
  // Z_GYRO_OUT
  gyro[2] = big_endian_to_short(&buff[9]) * M_PI / 180 / 10.0;
  // X_ACCL_OUT
  accl[0] = big_endian_to_short(&buff[11]) * M_PI / 180 / 10.0;
  // Y_ACCL_OUT
  accl[1] = big_endian_to_short(&buff[13]) * M_PI / 180 / 10.0;
  // Z_ACCL_OUT
  accl[2] = big_endian_to_short(&buff[15]) * M_PI / 180 / 10.0;
  // TEMP_OUT
  temp = big_endian_to_short(&buff[16]) * 0.1;
  return 0;
}

/**
 * @brief update gyro and accel in high-precision read
 */
int Adis16465::update(void)
{
  int16_t gyro_out[3], gyro_low[3], accl_out[3], accl_low[3], temp_out;

  // キャリブレーション
  // write_register((unsigned char)0x40, (int16_t)0x0000);
  // write_register((unsigned char)0x42, (int16_t)0x0000);
  // write_register((unsigned char)0x44, (int16_t)0x0000);
  // write_register((unsigned char)0x46, (int16_t)0x0000);
  // write_register((unsigned char)0x48, (int16_t)0x0000);
  // write_register((unsigned char)0x4a, (int16_t)0x0000);
  // write_register((unsigned char)0x4c, (int16_t)0x0000);
  // write_register((unsigned char)0x4e, (int16_t)0x0000);
  // write_register((unsigned char)0x50, (int16_t)0x0000);
  // write_register((unsigned char)0x52, (int16_t)0x0000);
  // write_register((unsigned char)0x54, (int16_t)0x0000);
  // write_register((unsigned char)0x56, (int16_t)0x0000);  // calibaration: 0x0fa0(9.8)

  read_register(0x04, gyro_low[0]);
  read_register(0x06, gyro_low[0]);
  read_register(0x08, gyro_out[0]);
  read_register(0x0a, gyro_low[1]);
  read_register(0x0c, gyro_out[1]);
  read_register(0x0e, gyro_low[2]);
  read_register(0x10, gyro_out[2]);
  read_register(0x12, accl_low[0]);
  read_register(0x14, accl_out[0]);
  read_register(0x16, accl_low[1]);
  read_register(0x18, accl_out[1]);
  read_register(0x1a, accl_low[2]);
  read_register(0x1c, accl_out[2]);
  read_register(0x00, temp_out);

  // temperature convert
  temp = temp_out * 0.1;
  
 // 32bit convert

  for (int i=0; i < 3; i++)

  {

    //ADIS16465-1, 32-bit GYROSCOPES Sensitivity 10,485,760 LSB/°/sec

    // gyro[i] = ((int32_t(gyro_out[i]) << 16) + int32_t(gyro_low[i])) * M_PI / 180.0 / 10485760.0;

    // accl[i] = ((int32_t(accl_out[i]) << 16) + int32_t(accl_low[i])) * 9.8 / 262144000.0;
    
    // gyro[i] = ((int32_t(gyro_out[i]) << 16) + int32_t(gyro_low[i])) * M_PI / 180.0 / 655360.0;

    // accl[i] = ((int32_t(accl_out[i]) << 16) + int32_t(accl_low[i])) * 9.8 / 52428800.0;

    gyro[i] = ((int32_t(gyro_out[i]) << 16) + int32_t(gyro_low[i])) * M_PI / 180.0 / 655360.0;

    accl[i] = ((int32_t(accl_out[i]) << 16) + int32_t(accl_low[i])) * 9.8 / 52428800.0;
    

    //ADIS16465-2, 32-bit GYROSCOPES Sensitivity 2,621,440 LSB/°/sec

    /*

    gyro[i] = ((int32_t(gyro_out[i]) << 16) + int32_t(gyro_low[i])) * M_PI / 180.0 / 2621440.0;

    accl[i] = ((int32_t(accl_out[i]) << 16) + int32_t(accl_low[i])) * 9.8 / 262144000.0;

    */

    

    //ADIS16465-3, 32-bit GYROSCOPES Sensitivity 655,360 LSB/°/sec

     /*

    gyro[i] = ((int32_t(gyro_out[i]) << 16) + int32_t(gyro_low[i])) * M_PI / 180.0 / 655360 .0;

    accl[i] = ((int32_t(accl_out[i]) << 16) + int32_t(accl_low[i])) * 9.8 / 262144000.0;

    */

    

  }

  return 0;
}

/**
 * @brief set bias estimating time (GLOB_CMD)
 * @retval 0 Success
 * @retval -1 Failed
 */
int Adis16465::set_bias_estimation_time(int16_t tbc)
{
  write_register(0x66, tbc);
  tbc = 0;
  int16_t dummy = 0;
  read_register(0x66, dummy);
  read_register(0x00, tbc);
  RCLCPP_INFO(rclcpp::get_logger("adis16465"), "TBC: %04x", tbc);
  return 0;
}

/**
 * @brief Bias correction update (GLOB_CMD)
 * @retval 0 Success
 * @retval -1 Failed
 */
int Adis16465::bias_correction_update(void)
{
  // Bit0: Bias correction update
  int16_t data = 1;
  write_register(0x68, data);

  return 0;
}

int Adis16465::reset(void)
{
  unsigned char buff[4] = {0xe8, 0x80, 0xe9, 0x00};

  int size = write(fd_, buff, sizeof(buff));
  if (size != sizeof(buff))
  {
    perror("reset_register");
    return -1;
  }
  if (tcdrain(fd_) < 0)
  {
    perror("reset_register");
    return -1;
  }
  unsigned char recv_buff[4] = {0};
  size = read(fd_, recv_buff, sizeof(recv_buff));
  if (size != sizeof(recv_buff))
  {
    perror("reset_register");
    return -1;
  }
  if (recv_buff[0] != 0xff)
  {
    perror("reset_register: ACK error");
    return -1;
  }
  return 0;
}

