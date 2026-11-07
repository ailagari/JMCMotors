/*==============================================================================
  JMCLinkUDP.h  -  commands over UDP (Ethernet shield, Controllino, ESP32 WiFi)
==============================================================================
  Works with any Arduino UDP object: EthernetUDP (Controllino, Uno/Mega with
  a W5100/W5500 shield, ESP32 with a W5500) or WiFiUDP (ESP32). Bring the
  network up in the sketch first (Ethernet.begin / WiFi.begin), then:

    EthernetUDP udp;               // or: WiFiUDP udp;
    JMCLinkUDP  pcLink(udp);
    ...
    pcLink.begin(8001, IPAddress(192,168,1,100), 8000);

  One command per datagram. Replies go to the command's sender (IP + source
  port) and to the PC address; events go to the PC address only.

  Sender filter: by default any host may send commands. In a plant, restrict
  it with allowSender(ip) (up to 4 addresses); packets from anyone else are
  dropped silently and counted in rejectedCount().
==============================================================================*/
#ifndef JMCLinkUDP_h
#define JMCLinkUDP_h

#include "JMCLink.h"
#include <Udp.h>
#include <IPAddress.h>

class JMCLinkUDP : public JMCLink {
  public:
    explicit JMCLinkUDP(UDP& udp) : _udp(udp) {}

    // localPort: the port this board listens on. pcIp/pcPort: where events
    // (and a copy of every reply) are sent.
    void begin(uint16_t localPort, IPAddress pcIp, uint16_t pcPort) {
      _pcIp = pcIp;
      _pcPort = pcPort;
      _udp.begin(localPort);
    }

    // Accept commands only from these addresses (call up to 4 times).
    bool allowSender(IPAddress ip) {
      if (_nAllowed >= 4) return false;
      _allowed[_nAllowed++] = ip;
      return true;
    }
    void allowAnySender()           { _nAllowed = 0; }
    uint32_t rejectedCount() const  { return _rejected; }
    IPAddress lastSender() const    { return _sender; }

    int receive(char* buf, size_t cap) override {
      int size = _udp.parsePacket();
      if (size <= 0) return 0;
      IPAddress from = _udp.remoteIP();
      uint16_t  port = _udp.remotePort();
      if (!allowed(from)) { discard(); _rejected++; return 0; }
      if ((size_t)size > cap - 1) { discard(); _sender = from; _senderPort = port; return -1; }
      int n = _udp.read((unsigned char*)buf, cap - 1);
      if (n < 0) n = 0;
      buf[n] = '\0';
      _sender = from;
      _senderPort = port;
      return n;
    }

    void reply(const char* msg, size_t len) override {
      send(_pcIp, _pcPort, msg, len);
      if (_senderPort && !(_sender == _pcIp && _senderPort == _pcPort))
        send(_sender, _senderPort, msg, len);
    }

    void event(const char* msg, size_t len) override {
      send(_pcIp, _pcPort, msg, len);
    }

  private:
    UDP&      _udp;
    IPAddress _pcIp;
    uint16_t  _pcPort = 0;
    IPAddress _sender;
    uint16_t  _senderPort = 0;
    IPAddress _allowed[4];
    uint8_t   _nAllowed = 0;
    uint32_t  _rejected = 0;

    bool allowed(const IPAddress& ip) const {
      if (_nAllowed == 0) return true;
      for (uint8_t i = 0; i < _nAllowed; i++) if (_allowed[i] == ip) return true;
      return false;
    }
    void discard() { while (_udp.available() > 0) _udp.read(); }
    void send(const IPAddress& ip, uint16_t port, const char* msg, size_t len) {
      if (!port) return;
      _udp.beginPacket(ip, port);
      _udp.write((const uint8_t*)msg, len);
      _udp.endPacket();
    }
};

#endif // JMCLinkUDP_h
