#ifndef SWITCH_H
#define SWITCH_H

class Switch {
public:
    Switch(int pin);
    void begin();
    bool isOn(); // retourne true si contact = GND

private:
    int pin;
};

#endif
