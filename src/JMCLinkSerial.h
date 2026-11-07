/*==============================================================================
  JMCLinkSerial.h  -  commands over a serial line (USB, RS-232, Bluetooth ...)
==============================================================================
    JMCLinkSerial pcLink(Serial);   // any Stream; start it in setup()

  Commands: one per line, ended by CR, LF or CRLF.
  Replies and events: each message ends with CR LF. A multi-line message
  (the I, SD and DG reports) uses bare LF between its lines, so a PC program
  can split messages on CR LF and lines on LF.

  Do not use the same port for this link and for the RS-485 drives.
==============================================================================*/
#ifndef JMCLinkSerial_h
#define JMCLinkSerial_h

#include "JMCLink.h"

class JMCLinkSerial : public JMCLink {
  public:
    explicit JMCLinkSerial(Stream& port) : _s(port) {}

    // The line is collected directly in buf, so the caller must pass the
    // same buffer on every call until a line completes (JMCController does).
    int receive(char* buf, size_t cap) override {
      while (_s.available() > 0) {
        int c = _s.read();
        if (c < 0) break;
        if (c == '\r' || c == '\n') {
          if (_n == 0 && !_overflow) continue;          // empty line / CRLF tail
          bool tooLong = _overflow;
          size_t n = _n;
          _n = 0;
          _overflow = false;
          if (tooLong) return -1;
          buf[n] = '\0';
          return (int)n;
        }
        if (_n < cap - 1) buf[_n++] = (char)c;
        else _overflow = true;                           // rejected at end of line
      }
      return 0;
    }

    void reply(const char* msg, size_t len) override { line(msg, len); }
    void event(const char* msg, size_t len) override { line(msg, len); }

  private:
    Stream& _s;
    size_t  _n = 0;
    bool    _overflow = false;

    void line(const char* msg, size_t len) {
      _s.write((const uint8_t*)msg, len);
      _s.write('\r');
      _s.write('\n');
    }
};

#endif // JMCLinkSerial_h
