#ifndef STARTCONTACT_H
#define STARTCONTACT_H

class StartContact {
public:
    StartContact(int pin);
    void begin();
    bool isInserted();   // tirette en place
    bool isRemoved();    // tirette retirée

private:
    int pin;
};

#endif
