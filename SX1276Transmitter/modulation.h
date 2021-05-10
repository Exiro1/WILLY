

#include "SX1276Lib/enums/enums.h"
#include <cstdint>
#include "sx1276-hal.h"

typedef struct RadioConfig
{
    ModemType type;
    uint32_t freq;
    uint32_t bandwidth;
    uint32_t datarate;//or spreading factor
    uint8_t coderate;
    uint32_t bandwidthAfc;
    uint16_t preambleLen;
    uint16_t symbTimeout;
    bool fixLen;
    uint8_t payloadLen;
    bool crcOn;
    bool freqHopOn ;
    uint8_t hopPeriod;
    bool iqInverted;
    bool rxContinuous;
    uint8_t power;
    uint32_t dev;
    

}RadioConfig_t;


void setRF(RadioConfig_t conf,SX1276MB1xAS *radio,bool onlyTX);
/*
void setRFRX(RadioConfig_t conf,SX1276MB1xAS *radio);

void setRFTX(RadioConfig_t conf,SX1276MB1xAS *radio);
*/