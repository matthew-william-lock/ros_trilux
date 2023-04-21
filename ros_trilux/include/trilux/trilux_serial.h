/*
BSD 3-Clause License

Copyright (c) 2023, Matthew Lock

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its
   contributors may be used to endorse or promote products derived from
   this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef TRILUX_SERIAL_H
#define TRILUX_SERIAL_H

#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <boost/function.hpp>
#include <boost/lexical_cast.hpp>

#include <iostream>

namespace trilux
{
    class TriLuxSerial
    {
    public:
        TriLuxSerial(std::string port_name, int baud) : io(),
                                                        port(io)
        {
            this->connect(port_name, baud);
        };

        ~TriLuxSerial(){};

        /**
         * Setup and connect to serial port.
         * @param portName Serial port name.
         * @param baud Serial baud rate.
         * @return
         */
        bool connect(const std::string &port_name, int baud)
        {
            port.open(port_name);
            port.set_option(boost::asio::serial_port::baud_rate(baud));
            port.set_option(boost::asio::serial_port::flow_control(boost::asio::serial_port::flow_control::none));
            port.set_option(boost::asio::serial_port::parity(boost::asio::serial_port::parity::none));
            port.set_option(boost::asio::serial_port::stop_bits(boost::asio::serial_port::stop_bits::one));

            if (!port.is_open())
            {
                return false;
            }

            this->startReceive();
            this->runner = boost::thread(boost::bind(&boost::asio::io_service::run, &io));

            return true;
        }

        /*!
         * @brief Start receiving data from the serial port
         */
        void startReceive()
        {
            boost::asio::async_read_until(this->port, this->buffer, "\r", boost::bind(&TriLuxSerial::onData, this, _1, _2));
        };

        /*!
         * @brief Callback for when data is received from the serial port
         * @param e Error code
         * @param size Size of data received
         * @note This function is called by the boost::asio::async_read_until function
         */
        void onData(const boost::system::error_code &e, std::size_t size)
        {

            if (e)
            {
                std::cout << "Error: " << e.message() << std::endl;
                return;
            }

            std::istream is(&buffer);
            std::string line;
            std::getline(is, line);

            std::cout << "Received: " << line << std::endl;

            // Start receiving again
            this->startReceive();
        }

    private:
        boost::asio::io_service io; // Hardware i/o service.
        // boost::asio::deadline_timer timer; // Timer without an expiry time.
        boost::asio::serial_port port; // Serial input port
        boost::thread runner;          // Main operation thread
        boost::asio::streambuf buffer; // Input buffer.
    };
}

#endif // TRILUX_SERIAL_H