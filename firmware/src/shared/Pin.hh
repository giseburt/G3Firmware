#ifndef PIN_HH
#define PIN_HH

#include "AvrPort.hh"
#include "Pin.hh"

/// \ingroup HardwareLibraries
class Pin {
private:
        AvrPort port;
        const uint8_t pin_index;
        const uint8_t pin_mask;
        const uint8_t pin_mask_inverted;
public:
        Pin() : port(AvrPort()), pin_index(0), pin_mask(0), pin_mask_inverted(0xff) {}
        Pin(AvrPort& port_in, uint8_t pin_index_in) : port(port_in), pin_index(pin_index_in), pin_mask(_BV(pin_index_in)), pin_mask_inverted(~_BV(pin_index_in)) {}
        bool isNull() const { return port.isNull(); }
        void setDirection(bool out) const { port.setPinDirection(pin_mask,out); }
        bool getValue() const { return port.getPin(pin_mask); }
        void setValue(bool on) const { on ? port.setPinOn(pin_mask) : port.setPinOff(pin_mask_inverted); }
        const uint8_t getPinIndex() const { return pin_index; }
};

static const Pin NullPin;

#endif // PIN_HH
