/*******************************************************************************
 * Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
 *
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 *
 *******************************************************************************/

#include <Arduino.h>

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <U8x8lib.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// the OLED used
// ux8x(clock, data, reset)
U8X8_SSD1306_128X64_NONAME_SW_I2C u8x8 (/* clock=*/ 22, /* data=*/ 21, /* reset=*/ 16);


// LoRaWAN NwkSKey, network session key
static const PROGMEM u1_t NWKSKEY[16] = { 0xF7, 0xCE, 0xFD, 0x86, 0x3A, 0x6B, 0x0C, 0x3D, 0x05, 0x76, 0xE8, 0x1A, 0xDC, 0xD5, 0x05, 0x07 };

// LoRaWAN AppSKey, application session key
static const u1_t PROGMEM APPSKEY[16] = { 0x8B, 0xDB, 0x05, 0xBE, 0x4A, 0x86, 0xED, 0x6C, 0xF8, 0x3C, 0xF9, 0xA3, 0xDA, 0x3F, 0x05, 0xD8 };

// LoRaWAN end-device address (DevAddr)
static const u4_t DEVADDR = 0x26011245;

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui(u1_t *buf) {}
void os_getDevEui(u1_t *buf) {}
void os_getDevKey(u1_t *buf) {}

#define ONE_WIRE_BUS 13 // GPIO 13
#define LED_BUILTIN 25


OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
DeviceAddress tempSensor;
static float tempC;
int sw = 0;

static uint8_t mydata[16];

static osjob_t sendjob;
static osjob_t triggerjob;
static osjob_t readjob;

void do_temp_read(osjob_t *j);

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 60 * 5;




// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = 18, 
    .rxtx = LMIC_UNUSED_PIN,
    .rst = LMIC_UNUSED_PIN,
    //If DIO2 is not connected use:
    .dio = {/*dio0*/ 26, /*dio1*/ 33, /*dio2*/ LMIC_UNUSED_PIN} 
    //If DIO2 is connected use:
    //.dio = {/*dio0*/ 26, /*dio1*/ 33, /*dio2*/ 32} 
};
void triggerReadTemp()
{
    sensors.requestTemperatures();
    Serial.println(F("Trigger Temps"));
}

void readTemps()
{
    tempC = sensors.getTempC(tempSensor);
    if((tempC > 85.0) || (tempC < -40.00)){ //invalid temperature reading
    Serial.printf("invalid temp reading = %0.2f\n", tempC);
    tempC = -127.0;
    sw = 0;
    return;
    }
    else
    {
      sw = 1;
    }

    Serial.printf("Read temps = %0.2fC\n", tempC);
    #define BUFFMAX 10
    char buf[BUFFMAX];
    int  len = snprintf(buf, BUFFMAX, "=%0.2f", tempC);
    if(len<0){
      snprintf(buf, BUFFMAX, "ERR CONV");
       Serial.printf("Conversion Error\n");
       sw = 0;
    }
    else if(len>=BUFFMAX){ //conversion exceeded buffer length
      snprintf(buf,BUFFMAX,"ERR OF");
      Serial.printf("temp Won't fit in buffer[%d] =%0.2f\n",BUFFMAX,tempC);
      sw = 0;   
    }
 
    u8x8.drawString(0, 0, buf);
}

int encodeTemp(float tempC)
{
    if(sw)
    {
        char buf[8];
        sprintf(buf, ">%0.2f", tempC);
        u8x8.drawString(0, 3, buf);
        Serial.printf("Send Temp: %0.2f\n", tempC);

        s2_t itemp = (s2_t)(tempC * 100);
        if (itemp < 0)
        {
          itemp *= -1;
          itemp |= 0x8000;
        }
        int len = sizeof(itemp);
        for (int i = 0; i < len; i++)
        {
          mydata[i] = itemp & 0xff;
          itemp = itemp >> 8;
        }
    
      return len;
    }
}

void do_temp_trigger(osjob_t *j)
{
    triggerReadTemp();

    // Scehdule the actual read in 2 seconds
    os_setTimedCallback(&readjob, os_getTime() + sec2osticks(2), do_temp_read);
}

void do_temp_read(osjob_t *j)
{
    readTemps();

    os_setTimedCallback(&triggerjob, os_getTime() + sec2osticks(28), do_temp_trigger);
}

void do_send(osjob_t *j)
{
    int len = encodeTemp(tempC);

    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND)
    {
        Serial.println(F("OP_TXRXPEND, not sending"));
    }
    else
    {
        // Prepare upstream data transmission at the next possible time.
        LMIC_setTxData2(1, mydata, len, 0);
        Serial.println(F("Packet queued"));
        digitalWrite(LED_BUILTIN, HIGH);
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void onEvent(ev_t ev)
{
    Serial.print(os_getTime());
    Serial.print(": ");
    switch (ev)
    {
    case EV_SCAN_TIMEOUT:
        Serial.println(F("EV_SCAN_TIMEOUT"));
        break;
    case EV_BEACON_FOUND:
        Serial.println(F("EV_BEACON_FOUND"));
        break;
    case EV_BEACON_MISSED:
        Serial.println(F("EV_BEACON_MISSED"));
        break;
    case EV_BEACON_TRACKED:
        Serial.println(F("EV_BEACON_TRACKED"));
        break;
    case EV_JOINING:
        Serial.println(F("EV_JOINING"));
        break;
    case EV_JOINED:
        Serial.println(F("EV_JOINED"));
        break;
    case EV_RFU1:
        Serial.println(F("EV_RFU1"));
        break;
    case EV_JOIN_FAILED:
        Serial.println(F("EV_JOIN_FAILED"));
        break;
    case EV_REJOIN_FAILED:
        Serial.println(F("EV_REJOIN_FAILED"));
        break;
    case EV_TXCOMPLETE:
        digitalWrite(LED_BUILTIN, LOW);
        Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
        if (LMIC.txrxFlags & TXRX_ACK)
            Serial.println(F("Received ack"));
        if (LMIC.dataLen)
        {
            Serial.println(F("Received "));
            Serial.println(LMIC.dataLen);
            Serial.println(F(" bytes of payload"));
        }
        // Schedule next transmission
        os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
        break;
    case EV_LOST_TSYNC:
        Serial.println(F("EV_LOST_TSYNC"));
        break;
    case EV_RESET:
        Serial.println(F("EV_RESET"));
        break;
    case EV_RXCOMPLETE:
        // data received in ping slot
        Serial.println(F("EV_RXCOMPLETE"));
        break;
    case EV_LINK_DEAD:
        Serial.println(F("EV_LINK_DEAD"));
        break;
    case EV_LINK_ALIVE:
        Serial.println(F("EV_LINK_ALIVE"));
        break;
    default:
        Serial.println(F("Unknown event"));
        break;
    }
}

void setup()
{
    Serial.begin(115200);
    Serial.println(F("Node v0.2"));

    u8x8.begin();
    u8x8.setFont(u8x8_font_courB18_2x3_r);

    sensors.begin();
    sensors.getAddress(tempSensor, 0);
    sensors.setResolution(tempSensor, 10); // 0.25 degrees increments

    // Trigger the initial temperature reading
    triggerReadTemp();

    pinMode(LED_BUILTIN, OUTPUT);
    for (int i = 0; i < 10; i++)
    {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(50);
        digitalWrite(LED_BUILTIN, LOW);
        delay(50);
    }

    // Read at least one reading into the registers and the global variable
    readTemps();

   SPI.begin(5, 19, 27, 18);

    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

// Set static session parameters. Instead of dynamically establishing a session
// by joining the network, precomputed session parameters are be provided.
#ifdef PROGMEM
    // On AVR, these values are stored in flash and only copied to RAM
    // once. Copy them to a temporary buffer here, LMIC_setSession will
    // copy them into a buffer of its own again.
    uint8_t appskey[sizeof(APPSKEY)];
    uint8_t nwkskey[sizeof(NWKSKEY)];
    memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
    memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
    LMIC_setSession(0x1, DEVADDR, nwkskey, appskey);
#else
    // If not running an AVR with PROGMEM, just use the arrays directly
    LMIC_setSession(0x1, DEVADDR, NWKSKEY, APPSKEY);
#endif


    #if defined(CFG_eu868)
    // Set up the channels used by the Things Network, which corresponds
    // to the defaults of most gateways. Without this, only three base
    // channels from the LoRaWAN specification are used, which certainly
    // works, so it is good for debugging, but can overload those
    // frequencies, so be sure to configure the full frequency range of
    // your network here (unless your network autoconfigures them).
    // Setting up channels should happen after LMIC_setSession, as that
    // configures the minimal channel set.
    // NA-US channels 0-71 are configured automatically
    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
    LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band
    // TTN defines an additional channel at 869.525Mhz using SF9 for class B
    // devices' ping slots. LMIC does not have an easy way to define set this
    // frequency and support for class B is spotty and untested, so this
    // frequency is not configured here.

   #elif defined(CFG_us915)
    // NA-US channels 0-71 are configured automatically
    // but only one group of 8 should (a subband) should be active
    // TTN recommends the second sub band, 1 in a zero based count.
    // https://github.com/TheThingsNetwork/gateway-conf/blob/master/US-global_conf.json
    LMIC_selectSubBand(1);
    #endif

    // Disable link check validation
    LMIC_setLinkCheckMode(0);

    // TTN uses SF9 for its RX2 window.
    LMIC.dn2Dr = DR_SF9;

    // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
    // Only use one channel and SF
    forceTxSingleChannelDr();

    // Start job
     do_temp_read(&readjob);
     do_send(&sendjob);
}
// Disables all channels, except for the one defined above, and sets the
  // data rate (SF). This only affects uplinks; for downlinks the default
  // channels or the configuration from the OTAA Join Accept are used.
  //
  // Not LoRaWAN compliant; FOR TESTING ONLY!
  //
  void forceTxSingleChannelDr() {
    // Define the single channel and data rate (SF) to use
   int channel = 0;
   int dr = DR_SF7;
    for(int i=0; i<9; i++) { // For EU; for US use i<71
        if(i != channel) {
            LMIC_disableChannel(i);
        }
    }
    // Set data rate (SF) and transmit power for uplink
    LMIC_setDrTxpow(dr, 14);
}
void loop()
{
    os_runloop_once();
}
