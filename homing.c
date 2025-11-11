#include "homing.h"
#include <math.h>

#define DEG2RAD (M_PI / 180.0f)
#define RAD2DEG (180.0f / M_PI)

SunPosition sun_position(float latitude, float longitude, double julian_day) {

    SunPosition sp;

    // Julian centuries since J2000.0
    float T = (julian_day - 2451545.0) / 36525.0;

    // Mean longitude and anomaly of the Sun
    float L0 = fmodf(280.46646f + T * (36000.76983f + T * 0.0003032f), 360.0f);
    float M  = 357.52911f + T * (35999.05029f - 0.0001537f * T);

    // Equation of center
    float C = (1.914602f - T*(0.004817f + 0.000014f*T)) * sinf(DEG2RAD*M)
            + (0.019993f - 0.000101f*T) * sinf(DEG2RAD*2*M)
            + 0.000289f * sinf(DEG2RAD*3*M);

    // True longitude and obliquity
    float lambda = L0 + C;
    float epsilon = 23.439f - 0.00000036f * T;

    // Declination
    float decl = asinf(sinf(DEG2RAD*epsilon) * sinf(DEG2RAD*lambda));

    // Equation of time (in minutes)
    float y = tanf(DEG2RAD*(epsilon/2.0f)) * tanf(DEG2RAD*(epsilon/2.0f));
    float Etime = 4.0f * RAD2DEG * (y*sinf(2*DEG2RAD*L0)
                 - 2*0.016708f*sinf(DEG2RAD*M)
                 + 4*0.016708f*y*sinf(DEG2RAD*M)*cosf(2*DEG2RAD*L0)
                 - 0.5f*y*y*sinf(4*DEG2RAD*L0)
                 - 1.25f*0.016708f*0.016708f*sinf(2*DEG2RAD*M));

    // --- Solar time calculation ---
    double JD0 = floor(julian_day + 0.5) - 0.5;     // Julian day at previous midnight
    double fractional_day = julian_day - JD0;       // fraction of day [0..1)
    float true_solar_time = fractional_day*1440.0f + Etime + 4.0f * longitude;
    true_solar_time = fmodf(true_solar_time, 1440.0f); // wrap to 0-1440 minutes

    float hour_angle = (true_solar_time / 4.0f) - 180.0f;

    // Convert to radians
    float H = DEG2RAD * hour_angle;
    float lat = DEG2RAD * latitude;

    // Elevation and azimuth
    float elev = asinf(sinf(lat)*sinf(decl) + cosf(lat)*cosf(decl)*cosf(H));
    float az = atan2f(-sinf(H), tanf(decl)*cosf(lat) - sinf(lat)*cosf(H));

    sp.elevation = elev * RAD2DEG;
    sp.azimuth = fmodf((az * RAD2DEG + 360.0f), 360.0f);

    return sp;
}

double utc_to_julian(datetime utc) {

	if (utc.M <= 2) {
		utc.Y -= 1;
		utc.M += 12;
	} // Treat Jan, Feb as months 13 and 14 of previous year (simplifies leap-year handling)

	int A = utc.Y / 100;
	int B = 2 - A + (A / 4); // Gregorian correction term

	double day_fraction = (utc.h + (utc.m / 60.0) + (utc.s / 3600.0)) / 24.0;

	double JD = (int)(365.25 * (utc.Y + 4716))
			  + (int)(30.6001 * (utc.M + 1))
			  + utc.D + day_fraction + B - 1524.5; // 1524.5 shifts the epoch so that JD = 0 corresponds to noon Jan 1, 4713 BC

	return JD;
}
