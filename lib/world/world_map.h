#pragma once

struct Zone {
    float x;
    float y;
    float radius;
};

// Dimensions plateau
const float FIELD_WIDTH = 450;   // mm
const float FIELD_HEIGHT = 1800;  // mm

// Zones de noisettes
const Zone ZONE_A = { 325, 200, 100 };
const Zone ZONE_B = { 275, 500, 75 };
const Zone ZONE_C = { 225, 750, 75 };
const Zone ZONE_D = { 225, 1050, 75 };
const Zone ZONE_E = { 275, 1300, 75 };

// Liste si besoin
const Zone ZONES_NOISETTES[] = { ZONE_A, ZONE_B, ZONE_C, ZONE_D, ZONE_E };
const int NB_ZONES_NOISETTES = 5;
