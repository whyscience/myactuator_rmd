#include "myactuator_rmd/can/node.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cerrno>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <system_error>
#include <vector>

#include "myactuator_rmd/can/linux_can.h"
// #include <linux/can/error.h>
// #include <linux/can/raw.h>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>

#include "myactuator_rmd/can/exceptions.hpp"
#include "myactuator_rmd/can/frame.hpp"
#include "myactuator_rmd/can/utilities.hpp"

#include "freertos/FreeRTOS.h"


namespace myactuator_rmd {
  namespace can {

    Node::Node(std::string const& ifname, std::chrono::microseconds const& send_timeout, std::chrono::microseconds const& receive_timeout,
               bool const is_signal_errors)
    {
      //initSocket(ifname);
      //setSendTimeout(send_timeout);
      //setRecvTimeout(receive_timeout);
      //setErrorFilters(is_signal_errors);
      return;
    }

    Node::~Node() {
      //closeSocket();
      return;
    }

    // ESP32SJA1000Class::loopback()
    void Node::setLoopback(bool const is_loopback) {
      if (is_loopback) {
          CAN.loopback();
      } else {
          // Exit loopback mode (you can write another method in ESP32SJA1000Class for this if needed)
          CAN.observe();  // For normal observation mode
      }
    }

    // int ESP32SJA1000Class::filter(int id, int mask)
    void Node::setRecvFilter(std::vector<std::uint32_t> const& can_ids, bool const is_invert) {
      /* std::vector<struct ::can_filter> filters {};
      filters.resize(can_ids.size());
      for (std::size_t i = 0; i < can_ids.size(); ++i) {
        auto const& can_id {can_ids[i]};
        if (is_invert) {
          filters[i].can_id = can_id | CAN_INV_FILTER;;
        } else {
          filters[i].can_id = can_id;
        }
        filters[i].can_mask = CAN_SFF_MASK;
      }
      if (::setsockopt(socket_, SOL_CAN_RAW, CAN_RAW_FILTER, filters.data(), sizeof(::can_filter)*filters.size()) < 0) {
        throw SocketException(errno, std::generic_category(), "Interface '" + ifname_ + "' - Could not configure read filter");
      }
      return; */
      for (const auto& id : can_ids) {
        if (id > 0x7FF) {
            // Extended frame
            CAN.filterExtended(id, 0x1FFFFFFF); // Filter with all bits as mask
        } else {
            // Standard frame
            CAN.filter(id, 0x7FF); // Standard filter
        }
    }
    }

    /*void Node::setSendTimeout(std::chrono::microseconds const& timeout) {
      struct ::timeval const send_timeout {myactuator_rmd::toTimeval(timeout)};
      if (::setsockopt(socket_, SOL_SOCKET, SO_SNDTIMEO, reinterpret_cast<const char*>(&send_timeout), sizeof(struct ::timeval)) < 0) {
        throw SocketException(errno, std::generic_category(), "Interface '" + ifname_ + "' - Error setting socket timeout");
      }
      return;
    }*/

    /*void Node::setRecvTimeout(std::chrono::microseconds const& timeout) {
      struct ::timeval const recv_timeout {myactuator_rmd::toTimeval(timeout)};
      if (::setsockopt(socket_, SOL_SOCKET, SO_RCVTIMEO, reinterpret_cast<const char*>(&recv_timeout), sizeof(struct ::timeval)) < 0) {
        throw SocketException(errno, std::generic_category(), "Interface '" + ifname_ + "' - Error setting socket timeout");
      }
      return;
    }*/

    /*void Node::setErrorFilters(bool const is_signal_errors) {
      // See https://github.com/linux-can/can-utils/blob/master/include/linux/can/error.h
      ::can_err_mask_t err_mask {};
      /* if (is_signal_errors) {
        err_mask = (CAN_ERR_TX_TIMEOUT | CAN_ERR_LOSTARB | CAN_ERR_CRTL | CAN_ERR_PROT | CAN_ERR_TRX | 
                    CAN_ERR_ACK | CAN_ERR_BUSOFF | CAN_ERR_BUSERROR | CAN_ERR_RESTARTED);
      }
      if (::setsockopt(socket_, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &err_mask, sizeof(::can_err_mask_t)) < 0) {
        throw SocketException(errno, std::generic_category(), "Interface '" + ifname_ + "' - Error setting error acknowledgement");
      } #1#
      return;
    }*/

    void canReceiver() {
      // try to parse packet
      int packetSize = CAN.parsePacket();

      if (packetSize) {
        // received a packet
        Serial.print ("Received ");

        if (CAN.packetExtended()) {
          Serial.print ("extended ");
        }

        if (CAN.packetRtr()) {
          // Remote transmission request, packet contains no data
          Serial.print ("RTR ");
        }

        Serial.print ("packet with id 0x");
        Serial.print (CAN.packetId(), HEX);

        if (CAN.packetRtr()) {
          Serial.print (" and requested length ");
          Serial.println (CAN.packetDlc());
        } else {
          Serial.print (" and length ");
          Serial.println (packetSize);

          // only print packet data for non-RTR packets
          while (CAN.available()) {
            Serial.print ((char) CAN.read());
          }
          Serial.println();
        }

        Serial.println();
      } else {
        Serial.println("No CAN frame available");
      }
    }

    Frame Node::read() const {
      Serial.println("Reading CAN frame");
      int packetSize = CAN.parsePacket();
      // wait for packet to be available
      /* while (packetSize == 0)
      {
        vTaskDelay(1);
        packetSize = CAN.parsePacket();
      }
      Serial.println("Got CAN frame"); */

      if (packetSize) {  // Check if a packet is available
        std::uint32_t can_id = CAN.packetId();  // Get the CAN ID
        std::array<std::uint8_t, 8> data;
        /* Serial.print("CAN ID: ");
        Serial.println(can_id, HEX);

        // Read data from the packet
        for (int i = 0; i < CAN.available(); i++) {
            data[i] = CAN.read();
        }

        Serial.print("Data: ");
        for (int i = 0; i < 8; i++) {
            Serial.print(data[i], HEX);
            Serial.print(" ");
        } */

        Serial.print ("Received ");

        if (CAN.packetExtended()) {
          Serial.print ("extended ");
        }

        if (CAN.packetRtr()) {
          // Remote transmission request, packet contains no data
          Serial.print ("RTR ");
        }

        Serial.print ("packet with id 0x");
        Serial.print (CAN.packetId(), HEX);

        if (CAN.packetRtr()) {
          Serial.print (" and requested length ");
          Serial.println (CAN.packetDlc());
        } else {
          Serial.print (" and length ");
          Serial.println (packetSize);

          // only print packet data for non-RTR packets
          while (CAN.available()) {
            Serial.print ((char) CAN.read());
          }
          Serial.println();
        }

        Serial.println();

        // Return a Frame object with the CAN ID and data
        return Frame(can_id, data);
      }
      Serial.println("No CAN frame available");

      // If no packet is available, throw an exception or handle it appropriately
      // throw std::runtime_error("No CAN frame available");
      return Frame(0, {0, 0, 0, 0, 0, 0, 0, 0});
    }

    void Node::write(Frame const& frame) {
      return write(frame.getId(), frame.getData());
    }

    void Node::write(std::uint32_t const can_id, std::array<std::uint8_t,8> const& data) { // write to CAN bus
      /* struct ::can_frame frame {};
      frame.can_id = can_id;
      frame.len = 8;
      std::copy(std::begin(data), std::end(data), std::begin(frame.data));
      if (::write(socket_, &frame, sizeof(struct ::can_frame)) != sizeof(struct ::can_frame)) {
        std::ostringstream ss {};
        ss << frame;
        throw SocketException(errno, std::generic_category(), "Interface '" + ifname_ + "' - Could not write CAN frame '" + ss.str() + "'");
      }
      return; */

      std::size_t dlc = data.size(); // Data length is the size of the array, should be 8

      // Start the CAN packet with CAN ID, DLC, and RTR (false for normal transmission)
      /* if (CAN.beginPacket(can_id, dlc, true) == 0) {
          // Handle error: invalid CAN ID or DLC
          throw std::runtime_error("Failed to begin CAN packet");
      } */
      CAN.beginPacket(can_id);

      //print the id and data
      Serial.print("Sending CAN ID: ");
      Serial.println(can_id, HEX);
      Serial.print("Data: ");
      for (std::size_t i = 0; i < dlc; ++i) {
          Serial.print(data[i], HEX);
          Serial.print(" ");
      }

      // Write the data array to the CAN packet
      for (std::size_t i = 0; i < dlc; ++i) {
          CAN.write(data[i]);
      }

      // Send the packet
      CAN.endPacket();
    }

    /*void Node::initSocket(std::string const& ifname) {//
      /* ifname_ = ifname;
      socket_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
      if (socket_ < 0) {
        throw SocketException(errno, std::generic_category(), "Interface '" + ifname_ + "' - Error creating socket");
      }

      struct ::ifreq ifr {};
      std::strcpy(ifr.ifr_name, ifname.c_str());
      if (::ioctl(socket_, SIOCGIFINDEX, &ifr) < 0) {
        throw SocketException(errno, std::generic_category(), "Interface '" + ifname_ + "' - Error manipulating device parameters");
      }

      struct ::sockaddr_can addr {};
      addr.can_family = AF_CAN;
      addr.can_ifindex = ifr.ifr_ifindex;
      if (::bind(socket_, reinterpret_cast<struct ::sockaddr*>(&addr), sizeof(addr)) < 0) {
        throw SocketException(errno, std::generic_category(), "Interface '" + ifname_ + "' - Error assigning address to socket");
      } #1#
      return;
    }*/

    /*void Node::closeSocket() noexcept {
      ::close(socket_);
      return;
    }*/

  }
}
