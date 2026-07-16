#ifndef ENCODER_COUNTER_H
#define ENCODER_COUNTER_H

#include <Arduino.h>
#include <stdint.h>

class EncoderCounter4x {
public:
  EncoderCounter4x();

  void init(uint8_t pinA, uint8_t pinB, bool invertDir = false);

  long getCount() const;
  void snapshot(long &count, unsigned long &lastEdgeUs) const;

  void resetCount();
  void setCount(long count);

  void onInterruptA();
  void onInterruptB();

  unsigned long getLastEdgeUs() const;

private:
  volatile long count_;
  volatile unsigned long lastEdgeUs_;
  volatile uint8_t prevState_;

  bool invertDir_;

  uint8_t pinA_;
  uint8_t pinB_;

  void processEdge();
};

#endif