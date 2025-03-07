#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <EEPROM.h>

/////////////////////////////////////////////////////////////////////////////
////                                 I2S                                 ////
/////////////////////////////////////////////////////////////////////////////

#define I2S_CLK_FREQ      160000000  // Hz
#define I2S_24BIT         3     // I2S 24 bit half data
#define I2S_LEFT          2     // I2S RX Left channel

#define I2SI_DATA         12    // I2S data on GPIO12 (D6)
#define I2SI_BCK          13    // I2S clk on GPIO13 (D7)
#define I2SI_WS           14    // I2S select on GPIO14 (D5)

#define SLC_BUF_CNT       8     // Number of buffers in the I2S circular buffer
#define SLC_BUF_LEN       64    // Length of one buffer, in 32-bit words.

/**
 * Convert I2S data.
 */
#define convert(sample) (((int32_t)(sample) >> 13) - 240200)

typedef struct {
  uint32_t blocksize      : 12;
  uint32_t datalen        : 12;
  uint32_t unused         : 5;
  uint32_t sub_sof        : 1;
  uint32_t eof            : 1;
  volatile uint32_t owner : 1;

  uint32_t *buf_ptr;
  uint32_t *next_link_ptr;
} sdio_queue_t;

static sdio_queue_t i2s_slc_items[SLC_BUF_CNT];  // I2S DMA buffer descriptors
static uint32_t *i2s_slc_buf_pntr[SLC_BUF_CNT];  // Pointer to the I2S DMA buffer data
static volatile uint32_t rx_buf_cnt = 0;
static volatile uint32_t rx_buf_idx = 0;
static volatile bool rx_buf_flag = false;

/**
 * @brief ISR triggered when SLC has finished writing to one of the buffers.
 */
void ICACHE_RAM_ATTR slc_isr(void *para) {
  uint32_t status;

  status = SLCIS;
  SLCIC = 0xFFFFFFFF;

  if (status == 0) {
    return;
  }

  if (status & SLCITXEOF) {
    // We have received a frame
    ETS_SLC_INTR_DISABLE();
    sdio_queue_t *finished = (sdio_queue_t*)SLCTXEDA;

    finished->eof = 0;
    finished->owner = 1;
    finished->datalen = 0;

    for (int i = 0; i < SLC_BUF_CNT; i++) {
      if (finished == &i2s_slc_items[i]) {
        rx_buf_idx = i;
      }
    }
    rx_buf_cnt++;
    rx_buf_flag = true;
    ETS_SLC_INTR_ENABLE();
  }
}

/**
 * @brief Set I2S clock.
 * I2S bits mode only has space for 15 extra bits,
 * 31 in total. The
 */
void i2s_set_rate(uint32_t rate) {
  uint32_t i2s_clock_div = (I2S_CLK_FREQ / (rate * 31 * 2)) & I2SCDM;
  uint32_t i2s_bck_div = (I2S_CLK_FREQ / (rate * i2s_clock_div * 31 * 2)) & I2SBDM;

  // RX master mode, RX MSB shift, right first, msb right
  I2SC &= ~(I2STSM | I2SRSM | (I2SBMM << I2SBM) | (I2SBDM << I2SBD) | (I2SCDM << I2SCD));
  I2SC |= I2SRF | I2SMR | I2SRMS | (i2s_bck_div << I2SBD) | (i2s_clock_div << I2SCD);
}

/**
 * @brief Initialise I2S as a RX master.
 */
void i2s_init(void) {
  // Config RX pin function
  PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTDI_U, FUNC_I2SI_DATA);
  PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTCK_U, FUNC_I2SI_BCK);
  PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTMS_U, FUNC_I2SI_WS);

  // Enable a 160MHz clock
  I2S_CLK_ENABLE();

  // Reset I2S
  I2SC &= ~(I2SRST);
  I2SC |= I2SRST;
  I2SC &= ~(I2SRST);

  // Reset DMA
  I2SFC &= ~(I2SDE | (I2SRXFMM << I2SRXFM));

  // Enable DMA
  I2SFC |= I2SDE | (I2S_24BIT << I2SRXFM);

  // Set RX single channel (left)
  I2SCC &= ~((I2STXCMM << I2STXCM) | (I2SRXCMM << I2SRXCM));
  I2SCC |= (I2S_LEFT << I2SRXCM);
  i2s_set_rate(16667);

  // Set RX data to be received
  I2SRXEN = SLC_BUF_LEN;

  // Bits mode
  I2SC |= (15 << I2SBM);

  // Start receiver
  I2SC |= I2SRXS;
}

/**
 * @brief Initialize the SLC module for DMA operation.
 * Counter intuitively, we use the TXLINK here to
 * receive data.
 */
void slc_init(void) {
  for (int x = 0; x < SLC_BUF_CNT; x++) {
    i2s_slc_buf_pntr[x] = (uint32_t *)malloc(SLC_BUF_LEN * 4);
    for (int y = 0; y < SLC_BUF_LEN; y++) i2s_slc_buf_pntr[x][y] = 0;

    i2s_slc_items[x].unused = 0;
    i2s_slc_items[x].owner = 1;
    i2s_slc_items[x].eof = 0;
    i2s_slc_items[x].sub_sof = 0;
    i2s_slc_items[x].datalen = SLC_BUF_LEN * 4;
    i2s_slc_items[x].blocksize = SLC_BUF_LEN * 4;
    i2s_slc_items[x].buf_ptr = (uint32_t *)&i2s_slc_buf_pntr[x][0];
    i2s_slc_items[x].next_link_ptr = (uint32_t *)((x < (SLC_BUF_CNT - 1)) ? (&i2s_slc_items[x + 1]) : (&i2s_slc_items[0]));
  }

  // Reset DMA
  ETS_SLC_INTR_DISABLE();
  SLCC0 |= SLCRXLR | SLCTXLR;
  SLCC0 &= ~(SLCRXLR | SLCTXLR);
  SLCIC = 0xFFFFFFFF;

  // Configure DMA
  SLCC0 &= ~(SLCMM << SLCM);      // Clear DMA MODE
  SLCC0 |= (1 << SLCM);           // Set DMA MODE to 1
  SLCRXDC |= SLCBINR | SLCBTNR;   // Enable INFOR_NO_REPLACE and TOKEN_NO_REPLACE

  // Feed DMA the 1st buffer desc addr
  SLCTXL &= ~(SLCTXLAM << SLCTXLA);
  SLCTXL |= (uint32_t)&i2s_slc_items[0] << SLCTXLA;

  ETS_SLC_INTR_ATTACH(slc_isr, NULL);

  // Enable EOF interrupt
  SLCIE = SLCITXEOF;
  ETS_SLC_INTR_ENABLE();

  // Start transmission
  SLCTXL |= SLCTXLS;
}

/**
 *  @brief Calculate RMS pressure from buffer of raw I2S samples
 *  @param samples Pointer to buffer holding raw samples
 *  @param length Length of buffer
 *  @return Calculated RMS pressure
 */
float calculatePressureRMS(uint32_t* samples, int length) {
  float sum = 0;
  for (int i = 0; i < length; i++) {
    int32_t sampleRaw = convert(samples[i]);  // Convert back to an 18-bit signed number
    float pressure = sampleRaw / 131071.0f;   // 131071 (max value of 18-bit signed number) ~ 1Pa
    sum += pressure * pressure;
  }
  float rms = sqrt(sum / length);
  return rms;
}

/**
 *  @brief Convert RMS pressure into SPL (dB)
 *  @param rms RMS pressure
 *  @return Calculated SPL
 */
float convertToDecibels(float rms) {
  float dbValue = 20.0f * log10(rms / 20.0e-6);
  return dbValue;
}

/**
 *  @brief Wait until a I2S sample buffer is filled and compute SPL
 *  @return Calculated SPL
 */
float getSoundLevel(void) {
  while (!rx_buf_flag); // Wait until a buffer is filled

  float rmsValue = calculatePressureRMS(i2s_slc_buf_pntr[rx_buf_idx], SLC_BUF_LEN);
  float dbValue = convertToDecibels(rmsValue);

  rx_buf_flag = false;

  return dbValue;
}

/////////////////////////////////////////////////////////////////////////////
////                           DATA FROM STM32                           ////
/////////////////////////////////////////////////////////////////////////////

#define PACKET_SOF 0x55   // Start of frame identifier
#define PACKET_EOF 0xff   // End of frame identifier

typedef struct __attribute__((packed)) {
    uint8_t sof;
    float temperature;
    float humidity;
    float pressure;
    float voc;
    float co2;
    float lux;
    uint8_t crc;
    uint8_t eof;
} datapacket_t;

/**
 *  @brief Calculate CRC checksum of databuffer
 *  @param buffer Pointer to databuffer
 *  @param size Length of buffer / part on which CRC applies
 *  @return calculated CRC
 */
static uint8_t calcCRC(uint8_t* buffer, size_t size) {
	uint16_t sum = 0;
	for (size_t i = 0; i < size; i++)
		sum += buffer[i];

	uint8_t crc = sum & 0xff;
	crc += (sum / 0x0100); 	// Add with carry
	crc = 0xFF-crc; 		    // Complement

	return crc;
}

/////////////////////////////////////////////////////////////////////////////
////                           MQTT & WIFI                               ////
/////////////////////////////////////////////////////////////////////////////

#define EEPROM_SIZE 4096

typedef struct __attribute__((packed)) {
  char ssid[35];
  char psk[35];
  char mqtt_server[35];
  uint16_t mqtt_port;
  char mqtt_topic[15];
  char classroom[5];
  uint8_t crc;
} config_t;
config_t cfg;

WiFiClient espClient;
PubSubClient client(espClient);

void read_config(void) {
  Serial.println(F("Reading node configuration..."));
  EEPROM.begin(EEPROM_SIZE);

  EEPROM.get(0, cfg);
  if (calcCRC((uint8_t*)&cfg, sizeof(config_t)-1) != cfg.crc) {
    Serial.println(F("ERROR: CRC error in configuration data"));
    while (1) {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(100);
      digitalWrite(LED_BUILTIN, LOW);
      delay(100);
    }
  }
}

/**
 *  @brief Try to connect to network
 *  WiFi not connected -> LED flashing
 *  WiFi connected -> LED on
 */
void setup_wifi(void) {
  bool led = false;

  Serial.printf("Connecting to '%s'...\n\r", cfg.ssid);
  WiFi.begin(cfg.ssid, cfg.psk);

  while (WiFi.status() != WL_CONNECTED) {
    digitalWrite(LED_BUILTIN, led = !led);
    delay(200);
  }
  digitalWrite(LED_BUILTIN, LOW); // LED on

  Serial.println(F("Connected"));
  Serial.print(F("IP address: "));
  Serial.println(WiFi.localIP());
}

/**
 *  @brief Try to (re)connect to MQTT server
 *  MQTT not connected -> LED flashing 2X
 *  MQTT connected -> LED on
 */
void connect_mqtt() {
  static bool firstTime = true;

  if (firstTime) {
    Serial.println(F("Connecting to MQTT..."));
    firstTime = false;
  } else {
    Serial.println(F("MQTT connection lost, trying to reconnect..."));
  }

  while (!client.connected()) {
    if (client.connect(("node-" + String(cfg.classroom)).c_str())) {
      Serial.println(F("MQTT connected"));
    } else {
      Serial.print(F("MQTT connection failed failed, rc = "));
      Serial.println(client.state());

      // Flash LED 2X
      digitalWrite(LED_BUILTIN, HIGH);
      delay(200);
      digitalWrite(LED_BUILTIN, LOW);
      delay(200);
      digitalWrite(LED_BUILTIN, HIGH);
      delay(200);
      digitalWrite(LED_BUILTIN, LOW);
      delay(200);
      digitalWrite(LED_BUILTIN, HIGH);
      delay(1200);
      digitalWrite(LED_BUILTIN, HIGH);
      delay(200);
      digitalWrite(LED_BUILTIN, LOW);
      delay(200);
      digitalWrite(LED_BUILTIN, HIGH);
      delay(200);
      digitalWrite(LED_BUILTIN, LOW);
      delay(200);
      digitalWrite(LED_BUILTIN, HIGH);
    }
  }

  digitalWrite(LED_BUILTIN, LOW);
}

void setup(void) {
  Serial.begin(115200);  // RX <- STM32, TX -> debug
  while (!Serial);
  Serial.println(F("\nESP8266 starting..."));

  pinMode(I2SI_WS, OUTPUT);
  pinMode(I2SI_BCK, OUTPUT);
  pinMode(I2SI_DATA, INPUT);
  slc_init();
  i2s_init();

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);  // Note: on-board LED on NodeMCU is active low
  read_config();
  setup_wifi();
  client.setServer(cfg.mqtt_server, cfg.mqtt_port);
}

void loop(void) {
  static uint8_t buffer[sizeof(datapacket_t)]; // Buffer used to hold incomming data
  static datapacket_t* as_packet = (datapacket_t *)buffer; // Buffer represented as packet
  static uint8_t* buffer_end = buffer + sizeof(datapacket_t) - 1; // Points to last byte of buffer
  static uint8_t* bptr = buffer; // Pointer to current position within buffer
  static bool expecting_sof = true; // Awaiting start of new packet

  // Maintain MQTT connection
  if (!client.connected()) {
    connect_mqtt();
  }
  client.loop();

  // Process incoming data
  while (Serial.available() > 0) {
    uint8_t cin = Serial.read();

    if (expecting_sof) {
      if (cin == PACKET_SOF) {
        *bptr++ = cin;
        expecting_sof = false;
      }
      continue;
    }

    if (bptr < buffer_end) {
      *bptr++ = cin; // Fill buffer with data
    } else if (bptr == buffer_end) {
      // Expecting EOF
      if (cin == PACKET_EOF) {
        *bptr++ = cin;
        
        uint8_t crc = calcCRC(buffer, sizeof(datapacket_t) - 2); // Don't include CRC and EOF bytes
        if (crc == as_packet->crc) {
          StaticJsonDocument<200> jsonDoc;

          jsonDoc["classID"] = cfg.classroom;
          jsonDoc["temperature"] = as_packet->temperature;
          jsonDoc["humidity"] = as_packet->humidity;
          jsonDoc["pressure"] = as_packet->pressure;
          jsonDoc["co2"] = as_packet->co2;
          jsonDoc["voc"] = as_packet->voc;
          jsonDoc["light"] = as_packet->lux;
          jsonDoc["microphone"] = getSoundLevel();

          char payload[200];
          serializeJson(jsonDoc, payload);

          if (client.publish(cfg.mqtt_topic, payload)) {
            Serial.println(F("Successfully sent payload to MQTT"));
          } else {
            Serial.println(F("Failed to send payload to MQTT"));
          }
        }
      }
      // Reset for the next packet
      bptr = buffer;
      expecting_sof = true;
    }
  }
}