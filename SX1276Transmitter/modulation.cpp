#include "modulation.h"




void setRF(RadioConfig_t conf, SX1276MB1xAS *Radio,bool onlyTX){
    Radio->SetChannel(conf.freq);
    Radio->SetTxConfig(conf.type, conf.power, conf.dev, conf.bandwidth, conf.datarate,
        conf.coderate, conf.preambleLen, conf.fixLen,
        conf.crcOn, 0, conf.hopPeriod,
        conf.iqInverted, 2000);
    if(!onlyTX)
    Radio->SetRxConfig(conf.type, conf.bandwidth, conf.datarate,
            conf.coderate, conf.bandwidthAfc, conf.preambleLen,
        conf.symbTimeout, conf.fixLen, conf.payloadLen,
            conf.crcOn, 0, conf.hopPeriod,
            conf.iqInverted, conf.rxContinuous);
	
        
}







/*
void setRFRX(RadioConfig_t conf, SX1276MB1xAS *Radio) {

	Radio->SetChannel(conf.freq);
    
    //Radio->SetRxConfig();
    Radio->SetRxConfig(conf.type, conf.bandwidth, conf.datarate,
            conf.coderate, conf.bandwidthAfc, conf.preambleLen,
        conf.symbTimeout, conf.fixLen, conf.payloadLen,
            conf.crcOn, 0, conf.hopPeriod,
            conf.iqInverted, conf.rxContinuous);


}

void setRFTX(RadioConfig_t conf, SX1276MB1xAS *Radio) {


    Radio->SetChannel(conf.freq);
    Radio->SetTxConfig(conf.type, conf.power, conf.dev, conf.bandwidth, conf.datarate,
        conf.coderate, conf.preambleLen, conf.fixLen,
        conf.crcOn, 0, conf.hopPeriod,
        conf.iqInverted, 2000);

}
*/