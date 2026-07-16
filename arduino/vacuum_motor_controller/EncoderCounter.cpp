#include "EncoderCounter.h"

EncoderCounter4x::EncoderCounter4x()
  : count_(0),
    lastEdgeUs_(0),
    prevState_(0),
    invertDir_(false),
    pinA_(0),
    pinB_(0)
{
}

void EncoderCounter4x::init(uint8_t pinA, uint8_t pinB, bool invertDir) {
  pinA_ = pinA;
  pinB_ = pinB;
  invertDir_ = invertDir;

  pinMode(pinA_, INPUT_PULLUP);
  pinMode(pinB_, INPUT_PULLUP);

  uint8_t a = digitalRead(pinA_);
  uint8_t b = digitalRead(pinB_);

  prevState_ = (a << 1) | b;

  count_ = 0;
  lastEdgeUs_ = 0;
}

long EncoderCounter4x::getCount() const {
  long value;

  noInterrupts();
  value = count_;
  interrupts();

  return value;
}

void EncoderCounter4x::snapshot(long &count, unsigned long &lastEdgeUs) const {
  noInterrupts();
  count = count_;
  lastEdgeUs = lastEdgeUs_;
  interrupts();
}

void EncoderCounter4x::resetCount() {
  noInterrupts();
  count_ = 0;
  interrupts();
}

void EncoderCounter4x::setCount(long count) {
  noInterrupts();
  count_ = count;
  interrupts();
}

void EncoderCounter4x::onInterruptA() {
  processEdge();
}

void EncoderCounter4x::onInterruptB() {
  processEdge();
}

unsigned long EncoderCounter4x::getLastEdgeUs() const {
  unsigned long value;

  noInterrupts();
  value = lastEdgeUs_;
  interrupts();

  return value;
}

void EncoderCounter4x::processEdge() {
  uint8_t a = digitalRead(pinA_);
  uint8_t b = digitalRead(pinB_);

  uint8_t currentState = (a << 1) | b;

  uint8_t index = (prevState_ << 2) | currentState;

  // Same class-code idea:
  // 00 -> 01 -> 11 -> 10 -> 00 = +1
  // 00 -> 10 -> 11 -> 01 -> 00 = -1
  static const int8_t stateTable[16] = {
    0,  1, -1,  0,
   -1,  0,  0,  1,
    1,  0,  0, -1,
    0, -1,  1,  0
  };

  int8_t delta = stateTable[index];

  if (invertDir_) {
    delta = -delta;
  }

  if (delta != 0) {
    count_ += delta;
    lastEdgeUs_ = micros();
  }

  prevState_ = currentState;
}