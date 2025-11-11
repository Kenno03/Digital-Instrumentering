#ifndef SUN_POSITION_H
#define SUN_POSITION_H

#include <stdint.h>

/*
azimuth   - Sun’s compass direction in degrees (0° = North, 90° = East)
elevation - Sun’s height above the horizon in degrees (0° = horizon, 90° = zenith)
*/

typedef struct {
    float azimuth;
    float elevation;
} SunPosition;

SunPosition sun_position(float latitude, float longitude, double JD);

typedef struct {
	uint16_t yy;
	uint8_t MM;
	uint8_t dd;
	uint8_t hh;
	uint8_t mm;
    uint8_t ss;
} datetime;

double utc_to_julian(datetime utc);

#endif /* SUN_POSITION_H */
