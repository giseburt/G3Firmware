
#include "Interface.hh"
#include "InterfaceBoard.hh"


// TODO: Make this a proper module.
#if defined HAS_INTERFACE_BOARD

namespace interface {


LiquidCrystal* lcd;
InterfaceBoard* board;

static const Pin InterfaceFooPin = INTERFACE_FOO_PIN;

bool isConnected() {

	// Strategy: Set up the foo pin as an input, turn on pull up resistor,
	// then measure it. If low, then we probably have an interface board.
	// If high, we probably don't.

	InterfaceFooPin.setValue(true);
	InterfaceFooPin.setDirection(false);

	// if we are pulled down, then we have an led attached??
	if (!InterfaceFooPin.getValue()) {
		InterfaceFooPin.setDirection(true);
		InterfaceFooPin.setValue(true);

		return true;
	}
	else {
		InterfaceFooPin.setDirection(true);
		InterfaceFooPin.setValue(false);

		return false;
	}

	return (!InterfaceFooPin.getValue());

}

void init(InterfaceBoard* board_in, LiquidCrystal* lcd_in) {
    board = board_in;
    lcd = lcd_in;
}

void pushScreen(Screen* newScreen) {
        board->pushScreen(newScreen);
}

void popScreen() {
        board->popScreen();
}

void doInterrupt() {
        board->doInterrupt();
}

micros_t getUpdateRate() {
        return board->getUpdateRate();
}

void doUpdate() {
        board->doUpdate();
}


}

#endif
