/* sensors.h */

#ifndef SENSORS_H
#define SENSORS_H 1


// Important variables
#define NUM_SENSORS 3
extern uint32_t dist[NUM_SENSORS];

// Subroutines
void initSensors(void);
void restartReading(void);
void stopReading(void);
#endif
