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

// DOCUMENTATION FOR THE TRILUX FLOUROMETER IS PROVIDED BY THE MANUFACTURER ON REQUEST

#ifndef TRILUX_PROTOCOL_H
#define TRILUX_PROTOCOL_H

#include <string>

namespace trilux
{

   class TriLuxProtocol
   {
   public:
      TriLuxProtocol(){};
      ~TriLuxProtocol(){};

      /*!
       * @brief Enable or disable continuous measurement mode
       * @param enable True to enable continuous measurement mode, false to disable
       * @return The command string to send to the TriLux
       */
      inline std::string setContinuousMeasurementMsg(bool enable)
      {
         return enable ? "Ru" : "St";
      };

      /*!
       * @brief Reboot the TriLux
       * @return The command string to send to the TriLux
       * @note This command will cause the TriLux to disconnect from the serial port and reconnect after a short delay
       * @note This command will not return a response
       *
       */
      inline std::string rebootMsg()
      {
         return "Reboot";
      }

      /*!
       * @brief Enable or disable VIN reporting
       * @param enable True to enable VIN reporting, false to disable
       * @return The command string to send to the TriLux
       */
      inline std::string enableVinReportingMsg(bool enable)
      {
         return enable ? "Sh vi on" : "Sh vi of";
      }

      /*!
       * @brief Enable or disable Vref reporting
       * @param enable True to enable Vref reporting, false to disable
       * @return The command string to send to the TriLux
       */
      inline std::string enableVrefReportingMsg(bool enable)
      {
         return enable ? "Sh vr on" : "Sh vr of";
      }

      /*!
       * @brief Enable or disable temp reporting
       * @param enable True to enable temp reporting, false to disable
       * @return The command string to send to the TriLux
       */
      inline std::string enableTempReportingMsg(bool enable)
      {
         return enable ? "Sh te on" : "Sh te of";
      }

      /*!
       * @brief Enable or disable analog reporting
       * @param enable True to enable analog reporting, false to disable
       * @return The command string to send to the TriLux
       */
      inline std::string enableAnalogMsg(bool enable)
      {
         return enable ? "An on" : "An of";
      }
   };
}

#endif // TRILUX_PROTOCOL_H