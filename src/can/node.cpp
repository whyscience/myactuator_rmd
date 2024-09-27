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

// #include "myactuator_rmd/can/linux_can.h"
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
#include <HardwareSerial.h>

/* valid bits in CAN ID for frame formats */
#define CAN_SFF_MASK 0x000007FFU /* standard frame format (SFF) */
#define CAN_EFF_MASK 0x1FFFFFFFU /* extended frame format (EFF) */
#define CAN_ERR_MASK 0x1FFFFFFFU /* omit EFF, RTR, ERR flags */


namespace myactuator_rmd
{
  namespace can
  {

    Node::Node(std::string const &ifname, std::chrono::microseconds const &send_timeout, std::chrono::microseconds const &receive_timeout,
               bool const is_signal_errors)
    {
      // initSocket(ifname);
      // setSendTimeout(send_timeout);
      // setRecvTimeout(receive_timeout);
      // setErrorFilters(is_signal_errors);
      return;
    }

    Node::~Node()
    {
      // closeSocket();
      return;
    }

    // ESP32SJA1000Class::loopback()
    void Node::setLoopback(bool const is_loopback)
    {
      if (is_loopback)
      {
        CAN0.setNoACKMode(true); // Loopback without ACK
      }
      else
      {
        CAN0.setNoACKMode(false); // Normal mode with ACK
      }
    }

    // int ESP32SJA1000Class::filter(int id, int mask)
    void Node::setRecvFilter(std::vector<std::uint32_t> const &can_ids, bool const is_invert)
    {
      // Reset filters to accept all by default
      twai_filter_config_t twai_filters_cfg = TWAI_FILTER_CONFIG_ACCEPT_ALL();

      // Iterate over can_ids and configure filters
      for (std::size_t i = 0; i < can_ids.size(); ++i)
      {
        uint32_t id = can_ids[i];
        if (is_invert)
        {
          // Implement custom filter logic for inverted filters (not directly supported in TWAI)
          twai_filters_cfg.acceptance_code = ~id;
          twai_filters_cfg.acceptance_mask = CAN_SFF_MASK;
        }
        else
        {
          twai_filters_cfg.acceptance_code = id;
          twai_filters_cfg.acceptance_mask = CAN_SFF_MASK;
        }
      }

      // Reinstall the driver with the new filter settings
      CAN0.disable();
      CAN0.enable(); // This will apply the new filter configuration
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

    void printFrameNode(CAN_FRAME *message)
    {
      Serial.print(message->id, HEX);
      if (message->extended)
        Serial.print(" Extended, len: ");
      else
        Serial.print(" Standard, len: "); // standard frame
      Serial.print(message->length, DEC);
      Serial.print(", ");
      for (int i = 0; i < message->length; i++)
      {
        Serial.print(message->data.byte[i], HEX);
        Serial.print(" ");
      }
      Serial.println();
    }

    Frame Node::read() const
    {
      Serial.println("Reading CAN frame");

      CAN_FRAME rxFrame;
      int time_ms = 0;
      while(!CAN0.available() && timeout_ms > time_ms++) {
        delay(1);
      }
      if (CAN0.available())
      {
        CAN0.read(rxFrame);
        Serial.print("Received frame!  ");
        printFrameNode(&rxFrame);
        std::array<std::uint8_t, 8> data{};
        std::copy(std::begin(rxFrame.data.byte), std::end(rxFrame.data.byte), std::begin(data));
        return Frame(rxFrame.id, data); // Adapted for your Frame class
      }
      else
      {
        Serial.println("Error reading CAN frame");
        return Frame{0, {0, 0, 0, 0, 0, 0, 0, 0}};
      }
    }

    void Node::write(Frame const &frame)
    {
      CAN_FRAME txFrame;
      txFrame.id = frame.getId();
      txFrame.length = frame.getData().size();
      std::copy(frame.getData().begin(), frame.getData().end(), std::begin(txFrame.data.byte));

      if (!CAN0.sendFrame(txFrame))
      {
        Serial.println("Error writing CAN frame");
        throw SocketException(errno, std::generic_category(), "Error writing CAN frame");
      }
    }

    void Node::write(std::uint32_t const can_id, std::array<std::uint8_t, 8> const &data)
    { // write to CAN bus
      Frame frame{can_id, data};
      write(frame);
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
