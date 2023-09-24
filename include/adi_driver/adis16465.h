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

#ifndef ADI_DRIVER_ADIS16465_H
#define ADI_DRIVER_ADIS16465_H

#include <termios.h>
#include <string>

// ADIS16465のクラス
class Adis16465
{
public:
  //! File descripter for USB-ISS
  // FDの概念について、URL：https://wa3.i-3-i.info/word14383.html
  // Linuxに振られるファイルの番号
  int fd_;
  //! Saved terminal config
  struct termios defaults_;

  // Gyro sensor(x, y, z)
  // ジャイロセンサのデータを収納する配列、doulbe 8Byte
  double gyro[3];
  // Acceleration sensor(x, y, z)
  // 加速度ンサのデータを収納する配列、 doulbe 8Byte
  double accl[3];
  // Temperature sensor
  double temp;

  // コンストラクター
  Adis16465();
  // シリアル通信を始める関数、引数：デバイスファイルのアドレスの文字列
  int openPort(const std::string device);
  // シリアル通信を閉じる関数
  void closePort();
  // 製品IDを取得する関数、引数：データの引用
  int get_product_id(int16_t& data);
  // データを更新する関数(再帰的読み書き)
  int update(void);
  // データを更新する関数(一気読み書き)
  int update_burst(void);
  // データの読み込みを行う関数、引数1：データのアドレス、引数2：データの引用
  int read_register(char address, int16_t& data);
  // データの書き込みを行う関数、引数1：データのアドレス、引数2：データの引用
  int write_register(char address, int16_t data);
  // 誤差を校正する関数
  int bias_correction_update(void);
  int set_bias_estimation_time(int16_t tbc);
  // IMUのリセット
  int reset(void);
};

#endif  // ADI_DRIVER_ADIS16465_H
