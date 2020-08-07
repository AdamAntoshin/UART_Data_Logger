/*
   RX_OUT (RX) - 11
   TX_IN (RX) - 20 (SDA)
*/

//Libraries
#include "wiring_private.h" // pinPeripheral() function
#include <SPI.h>
#include <SD.h>

//UART parameters
#define USB_BAUD_RATE 115200
#define LOGGER_BAUD_RATE 9600

//Buffer and logging parameters
#define BUFFER_SIZE 250
#define BUFFER_TIMEOUT 2000
#define TIME_STAMP_MAX_LENGTH 30
#define SD_CS 4

//General parameters
#define SD_ERROR_BLINK_DELAY 100
#define SD_ERROR_BLINK_NUM 10
#define STATUS_BLINK_DELAY 1000

//ASCII special characters
#define ASCII_TAB 9
#define ASCII_LF 10
#define ASCII_CR 13

/*
   Feather M0's SAMD core allows wiring multiple
   SERCOMs to certain digital pins to create
   new serial interfaces. Each SERCOM has 4 "pads".
   All of them (0, 1, 2, 3) can be used as RX,
   but only 0 and 2 can be TX.

   For more information, see: https://learn.adafruit.com/using-atsamd21-sercom-to-add-more-spi-i2c-serial-ports/creating-a-new-serial
   For SERCOM pads-to-pins table, see: https://github.com/arduino/ArduinoCore-samd/blob/master/variants/arduino_zero/variant.cpp
*/

//arguments: sercom_num, RX_PIN, TX_PIN, RX_PAD, TX_PAD
Uart RX_OUT (&sercom1, 11, 10, SERCOM_RX_PAD_0, UART_TX_PAD_2);
Uart TX_IN (&sercom3, 20, 6, SERCOM_RX_PAD_0, UART_TX_PAD_2);

//SD File object declaration
File log_file;

//Global constants and variables declaration
const char rx_time_stamp_format[] = "[RX-%07d]:";
const char tx_time_stamp_format[] = "[TX-%07d]:";
uint32_t led_timer, file_create_time;
char log_name_tmp[5] = "LOG_";
char log_name[13];
char rx_out_buff[BUFFER_SIZE], tx_in_buff[BUFFER_SIZE]; //RAM buffers
bool rx_out_buff_flag = false, tx_in_buff_flag = false; //Flags: dump buffers to file?
uint16_t indx = 0; //File index
//unsigned long rx_out_timer, tx_in_timer;

//Link SERCOMs to declared UART channels
void SERCOM1_Handler()
{
  RX_OUT.IrqHandler();
}

void SERCOM3_Handler()
{
  TX_IN.IrqHandler();
}

//Function declarations
void init_IO();
void init_SD();
void generate_log_name();
void open_log_file();
void init_SERCOM();
void init_buffers();
void handle_status_led();
uint32_t return_buff_time();
void handle_rx_out();
void handle_tx_in();
void dump_rx_out_buff();
void dump_tx_in_buff();

//Initialize IO
void init_IO() {
  Serial.begin(USB_BAUD_RATE);
  RX_OUT.begin(LOGGER_BAUD_RATE);
  TX_IN.begin(LOGGER_BAUD_RATE);

  pinMode(LED_BUILTIN, OUTPUT);
}

//Initialize SD card
void init_SD() {
  //If init fails, blink LED and try again
  while (!SD.begin(SD_CS)) {
    int i;
    Serial.println("SD INIT FAILED, TRYING AGAIN");
    for (i = 0; i < SD_ERROR_BLINK_NUM; i++) {
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      delay(SD_ERROR_BLINK_DELAY);
    }
  }
}

//After boot, logger generates new file with incremented index
void generate_log_name() {
  sprintf(log_name, "%s%04d.TXT", log_name_tmp, indx);
  led_timer = millis();

  //Check if file with index exists, and increment index if it does
  while (SD.exists(log_name)) {
    if (millis() - led_timer >= 100) {
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      led_timer = millis();
    }
    Serial.print(log_name);
    Serial.println(" exists...");
    indx++;
    sprintf(log_name, "%s%04d.TXT", log_name_tmp, indx);
  }
}

//Open (create) new log file
void open_log_file() {
  Serial.print("Opening ");
  Serial.println(log_name);

  log_file = SD.open(log_name, FILE_WRITE);

  //If file open fails, restart
  if (!log_file) {
    Serial.println("OPEN FAILED");
    NVIC_SystemReset();
  }

  //Mark file creation time for time stamps
  file_create_time = millis();
}

//Initialize SERCOM pins
void init_SERCOM() {
  // Assign pins 10 & 11 SERCOM functionality
  pinPeripheral(10, PIO_SERCOM);
  pinPeripheral(11, PIO_SERCOM);
  pinPeripheral(20, PIO_SERCOM);
  //pinPeripheral(16, PIO_SERCOM_ALT);
}

//Fill both buffers with null terminator, "clearing" them
void init_buffers() {
  memset(rx_out_buff, '\0', BUFFER_SIZE);
  memset(tx_in_buff, '\0', BUFFER_SIZE);
  }

//Update status led to blink slowly while waiting
void handle_status_led() {
  if (millis() - led_timer >= STATUS_BLINK_DELAY) {
    led_timer = millis();
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }
}

//Return buffer dump time in seconds
uint32_t return_buff_time() {
  return (millis() - file_create_time) / 1000;
  }

//Handle RX_OUT buffer
void handle_rx_out() {
  if (RX_OUT.available() > 0) {
    char c = RX_OUT.read();

    //c is a valid character
    if (c >= 32 && c <= 126) {
      char strChar[2];
      strChar[1] = '\0';
      strChar[0] = c;
      strcat(rx_out_buff, strChar); //Add to buffer string
    }
    //Replace tab with "[TAB]"
    else if (c == ASCII_TAB) {
      strcat(rx_out_buff, "[TAB]");
    }
    //Ignore line feeds
    else if (c == ASCII_LF) {
      //strcat(rx_out_buff, "[LF]");
      //rx_out_buff_flag = true;
      Serial.println("RX LF");
    }
    //Replace carriage return with "[CR]" and set flag, ending the buffer
    else if (c == ASCII_CR) {
      strcat(rx_out_buff, "[CR]");
      rx_out_buff_flag = true;
      Serial.println("RX CR");
    }
    //Replace invalid characters with ASCII hex code
    else {
      char hex_buff[10];
      sprintf(hex_buff, "[0x%02X]", c);
      strcat(rx_out_buff, hex_buff);
    }

    //Check if buffer is near full and set flag accordingly
    if (strlen(rx_out_buff) > BUFFER_SIZE - 7) {
      rx_out_buff_flag = true;
      Serial.println("RF BUFFER FULL");
    }

    //rx_out_timer = millis();
  }
}

//Handle TX_IN buffer
void handle_tx_in() {
  if (TX_IN.available() > 0) {
    char c = TX_IN.read();

    //c is a valid character
    if (c >= 32 && c <= 126) {
      char strChar[2];
      strChar[1] = '\0';
      strChar[0] = c;
      strcat(tx_in_buff, strChar);
    }
    //Replace tab with "[TAB]"
    else if (c == ASCII_TAB) {
      strcat(tx_in_buff, "[TAB]");
    }
    //Ignore line feeds
    else if (c == ASCII_LF) {
      //strcat(tx_in_buff, "[LF]");
      //tx_in_buff_flag = true;
      Serial.println("TX LF");
    }
    //Replace carriage return with "[CR]" and set flag, ending the buffer
    else if (c == ASCII_CR) {
      strcat(tx_in_buff, "[CR]");
      tx_in_buff_flag = true;
      Serial.println("TX CR");
    }
    //Replace invalid characters with ASCII hex code
    else {
      char hex_buff[10];
      sprintf(hex_buff, "[0x%02X]", c);
      strcat(tx_in_buff, hex_buff);
    }

    //Check if buffer is near full and set flag accordingly
    if (strlen(tx_in_buff) > BUFFER_SIZE - 7) {
      tx_in_buff_flag = true;
      Serial.println("TX BUFFER FULL");
    }

    //tx_in_timer = millis();
  }
}

//Dump RX_OUT buffer into text file
void dump_rx_out_buff() {
  uint16_t l; //Number of bytes written to file
  char stamp[TIME_STAMP_MAX_LENGTH];

  //Check if buffer is not empty
  if (rx_out_buff[0] != '\0') {
    //Generate and print time stamp
    sprintf(stamp, rx_time_stamp_format, return_buff_time());
    Serial.println("");
    Serial.print(stamp);
    log_file.println("");
    log_file.print(stamp);

    //Dump buffer, save number of bytes written and save file without closing it
    Serial.write(rx_out_buff);
    l = log_file.write(rx_out_buff); 
    log_file.flush();

    //If no bytes were written, communication with SD failed. Reboot.
    if (l == 0) {
      Serial.println("RESETTING");
      NVIC_SystemReset();
    }
  }

  //Clear buffer and deactivate flag
  memset(rx_out_buff, '\0', BUFFER_SIZE);
  rx_out_buff_flag = false;
  //rx_out_timer = 4294967295;
  //rx_out_timer = pow(2, 8 * sizeof(rx_out_timer)) - 1;
}

//Dump TX_IN buffer into text file
void dump_tx_in_buff() {
  uint16_t l; //Number of bytes written to file
  char stamp[TIME_STAMP_MAX_LENGTH];

  //Check if buffer is not empty
  if (tx_in_buff[0] != '\0') {
    //Generate and print time stamp
    sprintf(stamp, tx_time_stamp_format, return_buff_time());
    Serial.println("");
    Serial.print(stamp);
    log_file.println("");
    log_file.print(stamp);

    //Dump buffer, save number of bytes written and save file without closing it
    Serial.write(tx_in_buff);
    l = log_file.write(tx_in_buff);
    log_file.flush();

    //If no bytes were written, communication with SD failed. Reboot.
    if (l == 0) {
      Serial.println("RESETTING");
      NVIC_SystemReset();
    }
  }

  //Clear buffer and deactivate flag
  memset(tx_in_buff, '\0', BUFFER_SIZE);
  tx_in_buff_flag = false;
  //rx_out_timer = 4294967295;
  //tx_in_timer = pow(2, 8 * sizeof(tx_in_timer)) - 1;
  //tx_in_timer = 4294967295;
}

void setup() {
  init_IO();

  delay(1000);
  Serial.println("HI");

  init_SD();

  generate_log_name();

  open_log_file();

  init_buffers();
  
  Serial.println("HELLO");
  led_timer = millis();
  //rx_out_timer = pow(2, 8 * sizeof(rx_out_timer)) - 1;
  //tx_in_timer = pow(2, 8 * sizeof(tx_in_timer)) - 1;
  //rx_out_timer = 2000000000;
  //tx_in_timer = 2000000000;
}

void loop() {

  /*
    if ((long)millis() > BUFFER_TIMEOUT + (long)rx_out_timer) {
    rx_out_buff_flag = true;
    rx_out_timer = 2000000000;
    Serial.println("RX TIMEOUT");
    }

    if ((long)millis()  > BUFFER_TIMEOUT + (long)tx_in_timer) {
    tx_in_buff_flag = true;
    tx_in_timer = 2000000000;
    Serial.println("TX TIMEOUT");
    Serial.println((long)millis());
    Serial.println((long)tx_in_timer);
    Serial.println((long)millis() - (long)tx_in_timer);
    }
  */

  handle_rx_out();
  handle_tx_in();

  if (rx_out_buff_flag) {
    dump_rx_out_buff();
  }

  if (tx_in_buff_flag) {
    dump_tx_in_buff();
  }
}
