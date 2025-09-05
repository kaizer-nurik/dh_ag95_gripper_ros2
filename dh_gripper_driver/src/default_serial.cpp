// Copyright (c) 2023 PickNik, Inc.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the {copyright_holder} nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include "serial_driver/serial_driver.hpp"

#include <dh_gripper_driver/default_serial.hpp>
#include <chrono>
#include <system_error>
#include <vector>
#include <iostream>



namespace dh_gripper_driver
{

DefaultSerial::DefaultSerial()
{
}

DefaultSerial::~DefaultSerial()
{
  if (is_open())
  {
    close();
  }
}
void DefaultSerial::open()
{

  swri_serial_util::SerialConfig port_config(B115200,8,1,swri_serial_util::SerialConfig::Parity::NO_PARITY,false,false,true);
  if (!port_.Open(dev_name,port_config)){
    const auto error_msg = "PORT OPEN ERROR! " + port_.ErrorMsg();
    throw std::runtime_error(error_msg);

  }

  is_opened = true;
}

bool DefaultSerial::is_open() const
{
  return is_opened;
}

void DefaultSerial::close()
{
  port_.Close();
  is_opened = false;
}

std::vector<uint8_t> DefaultSerial::read(size_t size)
{
  std::vector<uint8_t> data;
  
  uint32_t ms32 = static_cast<uint32_t>(std::chrono::duration_cast<std::chrono::milliseconds>(timeout_ms).count());
  swri_serial_util::SerialPort::Result res = port_.ReadBytes(data, size, ms32);

  switch (res)
  {
  case swri_serial_util::SerialPort::Result::TIMEOUT:
  {
    const auto error_msg = "TIMEOUT READ ERROR!";
    throw std::runtime_error(error_msg);
    break;
  }
  case swri_serial_util::SerialPort::Result::INTERRUPTED:
  {
    const auto error_msg = "INTERRUPTED READ ERROR!";
    throw std::runtime_error(error_msg);
    break;
  }
  case swri_serial_util::SerialPort::Result::ERROR:
  {
    const auto error_msg = "READ ERROR!";
    throw std::runtime_error(error_msg);
    break;
  }
  default:
    break;
  }

  // if (data.size() != size)
  // {
  //   const auto error_msg = "Requested " + std::to_string(size) + " bytes, but got " + std::to_string(data.size());
  //   throw std::runtime_error(error_msg);
  // }

  //Try to fixx....
  while (data.size()<size)
  {
    std::vector<uint8_t> data_chunk = this->read(size-data.size());
    data.insert(data.end(), data_chunk.begin(), data_chunk.end());
  }
  

  return data;
}

void DefaultSerial::write(const std::vector<uint8_t>& data)
{
  
  int32_t num_bytes_written = port_.Write(data);
  int e = errno;  
  std::string msg = std::system_category().message(e);

  if (num_bytes_written == -1)
  {
    const auto error_msg = "WRITE ERROR "+ port_.ErrorMsg() + msg;
    throw std::runtime_error(error_msg);
  }
  if (num_bytes_written != data.size())
  {
    const auto error_msg =
        "Attempted to write " + std::to_string(data.size()) + " bytes, but wrote " + std::to_string(num_bytes_written);
    throw std::runtime_error(error_msg);
  }
}


void DefaultSerial::set_port(const std::string& port_name)
{
  dev_name = port_name;
}

std::string DefaultSerial::get_port() const
{
  return dev_name;
}

void DefaultSerial::set_timeout(std::chrono::milliseconds timeout)
{
  timeout_ms = timeout;
}

std::chrono::milliseconds DefaultSerial::get_timeout() const
{
  
  return timeout_ms;
}

void DefaultSerial::set_baudrate(uint32_t baudrate)
{
  baud = baudrate;
}

uint32_t DefaultSerial::get_baudrate() const
{
  return baud;
}


}  // namespace dh_gripper_driver
