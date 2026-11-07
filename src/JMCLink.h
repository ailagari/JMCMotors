/*==============================================================================
  JMCLink.h  -  how commands reach the controller and replies leave it
==============================================================================
  The controller does not care whether commands come over Ethernet, WiFi
  or a serial cable. It talks to one JMCLink:

    JMCLinkUDP     (JMCLinkUDP.h)     Ethernet shield / Controllino / ESP32
                                      WiFi - any Arduino UDP object
    JMCLinkSerial  (JMCLinkSerial.h)  USB serial or any Stream, one command
                                      per line

  Write your own by implementing these three functions.
==============================================================================*/
#ifndef JMCLink_h
#define JMCLink_h

#include <Arduino.h>

class JMCLink {
  public:
    // Copy one complete command into buf as a NUL-terminated string.
    // Returns its length, 0 when nothing is waiting, or -1 when a command
    // arrived but was longer than cap-1 and was dropped (the controller
    // answers "ERR:command too long" - a truncated command is never run).
    virtual int receive(char* buf, size_t cap) = 0;

    // The answer to the command being processed.
    virtual void reply(const char* msg, size_t len) = 0;

    // An unsolicited event (limit switch, fault, watchdog ...).
    virtual void event(const char* msg, size_t len) = 0;
};

#endif // JMCLink_h
