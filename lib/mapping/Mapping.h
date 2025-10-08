#ifndef MAPPING_H
#define MAPPING_H

#include <Arduino.h>

class Mapping {
public:
    Mapping();
    void begin();
    void update();
    
    // Example functions
    void scanBoard();        // Scan or map the board
    bool isCellOccupied(int x, int y); // Check if a board cell is occupied
};

#endif
