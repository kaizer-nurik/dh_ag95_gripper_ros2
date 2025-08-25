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
#include <condition_variable>
#include <mutex>
#include <chrono>
#include <system_error>
#include <vector>
#include <iostream>

#include <boost/asio/buffer.hpp>

namespace dh_gripper_driver
{

DefaultSerial::DefaultSerial() 
{
}

void DefaultSerial::open()
{
  if (port){
    ctx.waitForExit();
    port.reset(); 
  }
  drivers::serial_driver::SerialPortConfig config(baud, fc, pt, sb);
  static constexpr const char * dev_namez = "/dev/robot/dh_ag95_gripper";
  std::cout<< "dev_name "<< dev_namez<<std::endl;
  port = std::make_unique<drivers::serial_driver::SerialPort>(ctx, dev_namez, config);
  port->open();
}

bool DefaultSerial::is_open() const
{
  if(!port){
    return false;
  }
  return port->is_open();
}

void DefaultSerial::close()
{
  ctx.waitForExit();
  port.reset(); 
  port->close();
}

std::vector<uint8_t> DefaultSerial::read(size_t size)
{
  return read_with_timeout(size, timeout_ms);
  std::vector<uint8_t> data(size);
  data.resize(size);
  size_t bytes_read = port->receive(data);
  if (bytes_read != size)
  {
    const auto error_msg = "Requested " + std::to_string(size) + " bytes, but got " + std::to_string(bytes_read);
    throw std::runtime_error(error_msg);
  }
  return data;
}

void DefaultSerial::write(const std::vector<uint8_t>& data)
{
  std::size_t num_bytes_written = port->send(data);
  if (num_bytes_written != data.size())
  {
    const auto error_msg =
        "Attempted to write " + std::to_string(data.size()) + " bytes, but wrote " + std::to_string(num_bytes_written);
    throw std::runtime_error(error_msg);
  }
}


void DefaultSerial::set_port(const std::string& port_name)
{
  dev_name = (char*)port_name.c_str();
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

//-----------------------------------------------------------------------------
// read_with_timeout: uses SerialPort::async_receive(Functor)
// Functor signature is: std::function<void(std::vector<uint8_t>&, const size_t&)>
// -----------------------------------------------------------------------------
std::vector<uint8_t>
DefaultSerial::read_with_timeout(std::size_t size, std::chrono::milliseconds timeout)
{
  if (!port || !port->is_open()) {
    throw std::runtime_error("read_with_timeout(): serial port is not open");
  }
  if (size == 0) {
    return {};
  }

  std::vector<uint8_t> out(size);

  const auto deadline = std::chrono::steady_clock::now() + timeout;

  std::mutex mtx;
  std::condition_variable cv;

  std::size_t offset = 0;

  while (offset < size) {
    bool completed = false;
    std::size_t got = 0;
    std::error_code ec; // (SerialPort async functor doesn’t give EC, we keep this for uniformity)

    // Arm ONE async receive; handler copies into the output and signals.
    port->async_receive(
      [&](std::vector<uint8_t>& buff, const size_t& nbytes)
      {
        // Copy as much as we still need
        std::lock_guard<std::mutex> lk(mtx);
        got = std::min(nbytes, size - offset);
        if (got > 0) {
          std::memcpy(out.data() + offset, buff.data(), got);
          offset += got;
        }
        completed = true;
        cv.notify_one();
      }
    );

    std::unique_lock<std::mutex> lk(mtx);

    // Wait until this chunk completes or the overall deadline hits
    if (!cv.wait_until(lk, deadline, [&]{ return completed; })) {
      // Timeout -> abort the inflight async op the only way we can
      lk.unlock();
      try {
        // If your SerialPort exposes cancel(), prefer that:
        // port->cancel();
        if (port->is_open()) {
          port->close();
        }
      } catch (...) {}
      throw std::system_error(std::make_error_code(std::errc::timed_out),
                              "read_with_timeout(): timed out");
    }

    // Defensive: prevent infinite loop
    if (got == 0) {
      throw std::runtime_error("read_with_timeout(): async_receive returned 0 bytes");
    }
  }

  return out;
}


}  // namespace dh_gripper_driver
