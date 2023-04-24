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

#include <trilux/trilux_protocol.h>

#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <boost/function.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/split.hpp>

#include <chrono>
#include <iostream>

namespace trilux
{

    // Measurement Struct
    struct TriLuxMeasurement
    {
        double chlorophyll_a;           // [ug/L]
        double nephelometric_turbidity; // [FNU]
        double phycocyanin;             // [ug/L]
        uint32_t measurement_time;        // [s]
        double vin;                     // [V]
        double vref;                    // [V]
        double temp;                    // [C]

        std::string toString() const
        {
            return "Chl: " + std::to_string(chlorophyll_a) + " Neph: " + std::to_string(nephelometric_turbidity) + " Phyc: " + std::to_string(phycocyanin) + " Time: " + std::to_string(measurement_time) + " Vin: " + std::to_string(vin) + " Vref: " + std::to_string(vref) + " Temp: " + std::to_string(temp);
        }
    };

    class TriLuxSerial
    {
    public:
        TriLuxSerial(std::string port_name, int baud, std::function<void(const trilux::TriLuxMeasurement &)> onDatacallback) : io(),
                                                                                                                               timer(io),
                                                                                                                               port(io),
                                                                                                                               callback_(onDatacallback)
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
            // Open the serial port.
            try
            {
                this->port.open(port_name);
            }
            catch (boost::system::system_error &e)
            {
                std::cout << "Error: " << e.what() << std::endl;
                return false;
            }

            // Set options.
            this->port.set_option(boost::asio::serial_port_base::baud_rate(baud));
            this->port.set_option(boost::asio::serial_port_base::character_size(8));
            this->port.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
            this->port.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
            this->port.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));

            // Start the receive thread
            this->runner = boost::thread(boost::bind(&boost::asio::io_service::run, &this->io));

            // Start receiving data
            this->startReceive();

            return true;
        }

        /*!
         * @brief Start receiving data from the serial port
         */
        void startReceive()
        {
            boost::asio::async_read_until(this->port, this->buffer, "\n\r", boost::bind(&TriLuxSerial::onData, this, _1, _2));
        };

        /*!
         * @brief Callback for when data is received from the serial port.
            Typical message format is: "000.348, 004.831, 001.868, 011.83V, 002.48V, 029.26'C"
         * @param e Error code
         * @param size Size of data received
         * @param callback Callback function to call with the decoded data
         * @note This function is called by the boost::asio::async_read_until function
         */
        void onData(const boost::system::error_code &e, std::size_t size)
        {

            // Check for error
            if (e)
            {
                std::cout << "Error: " << e.message() << std::endl;
                return;
            }

            // Get the data
            std::istream is(&buffer);
            std::string line;
            std::getline(is, line);

            auto measurement = this->decodeData(line);
            if (measurement.first)
            {
                std::cout << measurement.second.toString() << std::endl;
                callback_(measurement.second);
            }

            // Start receiving again
            std::cout << "Waiting for data..." << std::endl;
            this->startReceive();
        }

        /**
         * Send data to serial port.
         * @param data Data to send.
         */
        void send(std::string data)
        {
            // Add CR to end of string
            data += "\r";
            boost::asio::write(this->port, boost::asio::buffer(data, data.size()));
        }

        /**
         * Decode data from serial port and return decode success.
         * @param data Data to decode.
         * @return Pair of decode success and decoded data.
         */
        std::pair<bool, trilux::TriLuxMeasurement> decodeData(std::string data)
        {

            // Typical message format is: "000.348, 004.831, 001.868, 011.83V, 002.48V, 029.26'C"
            // I.e. "chlorophyll, chlorophyll_a, nephelometric_turbidity, phycocyanin, vin, vref, temp"

            // Create a TriLuxMeasurement object
            trilux::TriLuxMeasurement measurement;

            // Try catch block to catch any errors
            try
            {
                // Split the string into a vector of strings
                std::vector<std::string> split_data;
                boost::split(split_data, data, boost::is_any_of(","));

                // Convert the strings to floats
                std::vector<float> split_data_float;
                for (std::vector<std::string>::iterator it = split_data.begin(); it != split_data.end(); ++it)
                {
                    // Remove any spaces
                    boost::trim(*it);

                    // Remove any spaces anywhere in the string
                    boost::erase_all(*it, " ");

                    // Remove any "V" or "'C" characters
                    if ((*it).find("V") != std::string::npos)
                    {
                        (*it).erase((*it).find("V"), 1);
                    }
                    if ((*it).find("'C") != std::string::npos)
                    {
                        (*it).erase((*it).find("'C"), 2);
                    }

                    std::string s = *it;

                    // Convert to float
                    split_data_float.push_back(std::stof(s));
                }

                measurement.chlorophyll_a = split_data_float[0];
                measurement.nephelometric_turbidity = split_data_float[1];
                measurement.phycocyanin = split_data_float[2];
                measurement.vin = split_data_float[3];
                measurement.vref = split_data_float[4];
                measurement.temp = split_data_float[5];
                measurement.measurement_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
            }
            // Catch any error
            catch (std::exception &e)
            {
                std::cout << "Error: " << e.what() << std::endl;
                return std::make_pair(false, measurement);
            }

            return std::make_pair(true, measurement);
        }

    private:
        boost::asio::io_service io;        // Hardware i/o service.
        boost::asio::deadline_timer timer; // Timer without an expiry time.
        boost::asio::serial_port port;     // Serial input port
        boost::thread runner;              // Main operation thread
        boost::asio::streambuf buffer;     // Input buffer.

        std::function<void(const trilux::TriLuxMeasurement &)> callback_;
    };
}

#endif // TRILUX_SERIAL_H