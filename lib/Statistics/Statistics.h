#ifndef Statistics_h
#define Statistics_h

#include "Arduino.h"

const byte STATS_BUF_LENGTH = 50;

class Statistics
{
private:
    byte failsCount;
    int avgMs;
    byte momentMs;
    unsigned int iterationsCount;
    byte maxMs;
    byte buffer[STATS_BUF_LENGTH];
    byte errorsCountBuffer[STATS_BUF_LENGTH];
    byte bufferI;
    long startMs;
    long nextStartMs;
    long endMs;
    long nextEndMs;
    float failRate;

    void calc();

public:
    Statistics();

    void registerIteration();
    void registerTransmissionEnd();

    void registerTransmissionError();

    void registerIterationEnd();

    unsigned int getIterationsCount();
    byte getFailsCount();
    float getFailRate();
    int getAvgMs();
    byte getMaxMs();
    byte getMomentMs();
};

#endif