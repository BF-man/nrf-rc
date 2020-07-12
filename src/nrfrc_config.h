#ifndef nrfrc_config_h
#define nrfrc_config_h

#include "Arduino.h"

const uint8_t NRFRC_CONFIG_RADIO_CHANNEL = 115;
const rf24_datarate_e NRFRC_CONFIG_RADIO_DATA_RATE = RF24_250KBPS;
const bool NRFRC_CONFIG_RADIO_ACK_ENABLED = true;

#endif
