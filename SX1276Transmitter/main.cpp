#include "mbed.h"
#include "main.h"
#include "sx1276-hal.h"
#include "modulation.h"

/* Set this flag to '1' to display debug messages on the console */
#define DEBUG_MESSAGE   1

/* Set this flag to '1' to use the LoRa modulation or to '0' to use FSK modulation */
#define USE_MODEM_LORA  1
#define USE_MODEM_FSK   !USE_MODEM_LORA

#define RF_FREQUENCY                                    868000000 // Hz
#define TX_OUTPUT_POWER                                 14        // 14 dBm


    #define LORA_BANDWIDTH                              2         // [0: 125 kHz,
                                                                  //  1: 250 kHz,
                                                                  //  2: 500 kHz,
                                                                  //  3: Reserved]
    #define LORA_SPREADING_FACTOR                       7         // [SF7..SF12]
    #define LORA_CODINGRATE                             1         // [1: 4/5,
                                                                  //  2: 4/6,
                                                                  //  3: 4/7,
                                                                  //  4: 4/8]
    #define LORA_PREAMBLE_LENGTH                        8         // Same for Tx and Rx
    #define LORA_SYMBOL_TIMEOUT                         5         // Symbols
    #define LORA_FIX_LENGTH_PAYLOAD_ON                  false
    #define LORA_FHSS_ENABLED                           false  
    #define LORA_NB_SYMB_HOP                            4     
    #define LORA_IQ_INVERSION_ON                        false
    #define LORA_CRC_ENABLED                            true


    #define FSK_FDEV                                    25000     // Hz
    #define FSK_DATARATE                                19200     // bps
    #define FSK_BANDWIDTH                               50000     // Hz
    #define FSK_AFC_BANDWIDTH                           83333     // Hz
    #define FSK_PREAMBLE_LENGTH                         5         // Same for Tx and Rx
    #define FSK_FIX_LENGTH_PAYLOAD_ON                   false
    #define FSK_CRC_ENABLED                             true


#define RX_TIMEOUT_VALUE                                3500      // in ms
#define BUFFER_SIZE                                     32        // Define the payload size here

#if( defined ( TARGET_KL25Z ) || defined ( TARGET_LPC11U6X ) )
DigitalOut led( LED2 );
#else
DigitalOut led( LED1 );
#endif

/*
 *  Global variables declarations
 */
typedef enum
{
    LOWPOWER = 0,
    IDLE,

    RX,
    RX_TIMEOUT,
    RX_ERROR,

    TX,
    TX_TIMEOUT,

    CAD,
    CAD_DONE
}AppStates_t;

typedef enum
{
    HS,
    TIME,
    CONFIG,
    END
}UpdateState_t;


UpdateState_t updState = HS;

volatile AppStates_t State = LOWPOWER;

/*!
 * Radio events function pointer
 */
static RadioEvents_t RadioEvents;

/*
 *  Global variables declarations
 */
SX1276MB1xAS Radio( NULL );

const uint8_t ACK[] = "ACK";
const uint8_t TIMESET[] = "TIMESET";
const uint8_t CONFSET[] = "CONFIGSET";

uint16_t BufferSize = BUFFER_SIZE;
uint8_t Buffer[BUFFER_SIZE];

uint8_t frame[16] = {'0','1','2','3','4','5','6','7','8','9','a','b','c','d','e','f'};

int16_t RssiValue = 0.0;
int8_t SnrValue = 0.0;

uint8_t SFvalue[6] = {7,8,9,10,11,12};

RadioConfig conf;

uint32_t freqTemp;
ModemType typeT;
uint8_t bandT;
uint8_t sfT;
uint8_t crT;
uint8_t ptxT;

int main( void ) 
{
    uint8_t i;


    bool isMaster = true;

    debug( "\n\n\r     Transmitter Started \n\n\r" );

    // Initialize Radio driver
    initRadioEvent();

    // verify the connection with the board
    while( Radio.Read( REG_VERSION ) == 0x00  )
    {
        debug( "Radio could not be detected!\n\r", NULL );
        wait( 1 );
    }

    debug_if( ( DEBUG_MESSAGE & ( Radio.DetectBoardType( ) == SX1276MB1LAS ) ), "\n\r > Board Type: SX1276MB1LAS < \n\r" );
    debug_if( ( DEBUG_MESSAGE & ( Radio.DetectBoardType( ) == SX1276MB1MAS ) ), "\n\r > Board Type: SX1276MB1MAS < \n\r" );
    

   conf = (RadioConfig) {
        MODEM_LORA, RF_FREQUENCY, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,LORA_CODINGRATE, 0, 
        LORA_PREAMBLE_LENGTH, LORA_SYMBOL_TIMEOUT,LORA_FIX_LENGTH_PAYLOAD_ON,0,
        LORA_CRC_ENABLED,LORA_FHSS_ENABLED,LORA_NB_SYMB_HOP,LORA_IQ_INVERSION_ON,
        true,TX_OUTPUT_POWER,FSK_FDEV 
    };
    
    
    
    debug_if( DEBUG_MESSAGE, "Starting Update process...\r\n" );

    led = 0;
    updateProcess();
    startProcessing();
    
}

void updateProcess(){
    setRF(conf,&Radio,false);
    wait_ms(500);
    bool end = false;
    Radio.Rx( 4000 );
    while( !end )
    {
        switch( State )
        {
        case RX:
            State = LOWPOWER;
            if( BufferSize > 0 )
            {
                  if(strcmp((char*)Buffer,"HSINIT") == 0){
                     updState = TIME;
                     debug("HandShake received \n");
                     sendFrame(ACK,4);
                  }else 
                  if(updState == TIME){
                     updState = CONFIG;
                     setTime();
                     debug("Time set \n");
                     sendFrame(TIMESET,8);
                  }else         /*    TO IMPLEMENT    */
                  if(updState == CONFIG){
                     updState = END;
                     setConfig();
                     debug("config set \n");
                     sendFrame(CONFSET,10);
                  }else         /*    TO IMPLEMENT    */
                  if(updState == END){
                     freqTemp = atoi((char*)Buffer);
                     debug("OTA CONFIG END");
                     end = true;
                  }else{
                      //error, restarting process
                      updState = HS;
                      Radio.Rx( 4000 );
                  }
                  
            }
            
            break;
        case TX:
            Radio.Rx( 3000 );
            State = LOWPOWER;
            break;
            //si rien reçu, on renvoi le handshake
        case RX_TIMEOUT:
            updState = HS;
            Radio.Rx( 4000 );
            State = LOWPOWER;
            break;
        case RX_ERROR:
            updState = HS;
            Radio.Rx( 4000 );
            State = LOWPOWER;
            break;
        case LOWPOWER:
            break;
        default:
            State = LOWPOWER;
            break;
        }
    }
    conf.freq = freqTemp;
    conf.datarate = sfT;
    conf.coderate = crT;
    conf.power = ptxT;
    conf.bandwidth = bandT;
    
    
    debug("UPDATE END %u %u %u %u %u\n",conf.freq,conf.datarate,conf.coderate,conf.power,conf.bandwidth);
}


void setConfig(){
    typeT = MODEM_LORA;
    if (Buffer[0] == 0)
        typeT = MODEM_FSK;
    bandT = Buffer[1];
    sfT = Buffer[2];
    crT = Buffer[3];
    ptxT = Buffer[4];
    
}
//get info from buffer
void setTime(){
    struct tm myDate;
    myDate.tm_mday = Buffer[0];
    myDate.tm_mon = Buffer[1];  
    myDate.tm_year = Buffer[2];   
    myDate.tm_hour = Buffer[3];
    myDate.tm_min = Buffer[4];
    myDate.tm_sec = Buffer[5];  
    time_t timestamp = mktime( & myDate );    
    set_time(timestamp);
    
    time_t seconds = time(NULL);
    char timet[32];
    strftime(timet, 32, "%c \n", localtime(&seconds));
    debug(" TIME : ");
    debug(timet);
    debug("\n");
}

void startProcessing(){
    wait_ms(500);
    setRF(conf,&Radio,true);
    wait_ms(500);
    sendFrame(frame, 16);
    uint8_t ind = 0;
    uint8_t sfind = 1;
    
    while( 1 )
    {
        switch( State )
        {
        
        case TX:
            State = LOWPOWER;
            led = !led;
            //debug("send \r\n");
            wait_ms(333);
            uint8_t ranBytes[32];
            getRandom(ranBytes,7);
            sendFrame(ranBytes,7);
            
            
            break;
        case TX_TIMEOUT:
            break;
        case LOWPOWER:
            break;
        default:
            State = LOWPOWER;
            break;
        }
    }
    
}

void getRandom(uint8_t *ran,uint8_t size){
    srand(time(NULL));
    for(uint8_t i = 0;i<size;i++){
        *(ran+i) = rand()%(255);
    }
}

void sendFrame(uint8_t* bytes,uint8_t len) {
    debug("sending : ");
    char text[32];
    strcpy(text , (char*)bytes);
    debug(text);
    debug( "\r\n" );
    Radio.Send(bytes, len);
    wait_ms(10);
}

void sendFrame(const uint8_t* bytes,uint8_t len) {
    debug("sending : ");
    char text[32];
    strcpy(text , (char*)bytes);
    debug(text);
    debug( "\r\n" );
    Radio.Send((uint8_t*)text, len);
    wait_ms(10);
}


void initRadioEvent() {
    RadioEvents.TxDone = OnTxDone;
    RadioEvents.RxDone = OnRxDone;
    RadioEvents.RxError = OnRxError;
    RadioEvents.TxTimeout = OnTxTimeout;
    RadioEvents.RxTimeout = OnRxTimeout;
    Radio.Init(&RadioEvents);
}

void OnTxDone( void )
{
    Radio.Sleep( );
    State = TX;
    debug_if( DEBUG_MESSAGE, "> OnTxDone\n\r" );
}

void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr )
{
    Radio.Sleep( );
    BufferSize = size;
    memcpy( Buffer, payload, BufferSize );
    RssiValue = rssi;
    SnrValue = snr;
    State = RX;
    debug_if( DEBUG_MESSAGE, "> OnRxDone\n\r" );
}

void OnTxTimeout( void )
{
    Radio.Sleep( );
    State = TX_TIMEOUT;
    debug_if( DEBUG_MESSAGE, "> OnTxTimeout\n\r" );
}

void OnRxTimeout( void )
{
    Radio.Sleep( );
    Buffer[BufferSize] = 0;
    State = RX_TIMEOUT;
    debug_if( DEBUG_MESSAGE, "> OnRxTimeout\n\r" );
}

void OnRxError( void )
{
    Radio.Sleep( );
    State = RX_ERROR;
    debug_if( DEBUG_MESSAGE, "> OnRxError\n\r" );
}

