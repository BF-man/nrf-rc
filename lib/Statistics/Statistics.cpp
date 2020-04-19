#include "Arduino.h"
#include "Statistics.h"

Statistics::Statistics()
{
  failsCount = 0;
  avgMs = 0;
  momentMs = 0;
  iterationsCount = 0;
  maxMs = 0;
  bufferI = 0;
  startMs = millis();
  nextStartMs = millis();
  endMs = millis();
  nextEndMs = millis();
  failRate;
}

void Statistics::registerIteration()
{
  ++iterationsCount;
  startMs = nextStartMs;
  nextStartMs = millis();

  errorsCountBuffer[bufferI] = 0;
};
void Statistics::registerTransmissionEnd(){};

void Statistics::registerTransmissionError()
{
  errorsCountBuffer[bufferI] = 1;
};

void Statistics::registerIterationEnd()
{
  endMs = nextEndMs;
  nextEndMs = millis();
  momentMs = endMs - startMs;

  ++bufferI;
  if (bufferI >= STATS_BUF_LENGTH)
    bufferI = 0;

  calc();
}

void Statistics::calc()
{
  buffer[bufferI] = momentMs;

  avgMs = 0;
  for (int i = 0; i < STATS_BUF_LENGTH; i++)
  {
    avgMs += buffer[i];
    if (buffer[i] > maxMs)
      maxMs = buffer[i];
  }
  avgMs = avgMs / STATS_BUF_LENGTH;

  failsCount = 0;
  for (int i = 0; i < STATS_BUF_LENGTH; i++)
  {
    if (errorsCountBuffer[i] == 1)
      failsCount += 1;
  }

  failRate = (float)(failsCount) / (float)STATS_BUF_LENGTH;
}

unsigned int Statistics::getIterationsCount() { return iterationsCount; };
byte Statistics::getFailsCount() { return failsCount; };
float Statistics::getFailRate() { return failRate; };
int Statistics::getAvgMs() { return avgMs; };
byte Statistics::getMaxMs() { return maxMs; };
byte Statistics::getMomentMs() { return momentMs; };