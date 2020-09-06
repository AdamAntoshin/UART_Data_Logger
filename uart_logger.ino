/*
   COM1 (RX) - 11
   COM2 (RX) - 20 (SDA)
*/

//Libraries
#include "wiring_private.h" // pinPeripheral() function
#include <SPI.h>
#include <SD.h>

//UART parameters
#define USB_BAUD_RATE 115200
#define COM1_BAUD_RATE_DEFAULT 9600
#define COM2_BAUD_RATE_DEFAULT 9600

//Buffer and logging parameters
#define BUFFER_SIZE 250
#define BUFFER_TIMEOUT 2000
#define CONFIG_SIZE 100
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
Uart COM1 (&sercom1, 11, 10, SERCOM_RX_PAD_0, UART_TX_PAD_2);
Uart COM2 (&sercom3, 20, 6, SERCOM_RX_PAD_0, UART_TX_PAD_2);

//SD File object declaration
File log_file, config_file;

//Global constants and variables declaration
const char com1_time_stamp_format[] = "[COM1-%07d]:";
const char com2_time_stamp_format[] = "[COM2-%07d]:";
uint32_t led_timer, file_create_time;
uint32_t com1_baud_rate, com2_baud_rate;
char log_name_tmp[5] = "LOG_";
char log_name[13];
char config_name[] = "CONFIG.TXT";
char com1_buff[BUFFER_SIZE], com2_buff[BUFFER_SIZE], config_buff[CONFIG_SIZE]; //RAM buffers
bool com1_buff_flag = false, com2_buff_flag = false; //Flags: dump buffers to file?
uint16_t indx = 0; //File index
//uint32_t com1_timer, com2_timer;

//Link SERCOMs to declared UART channels
void SERCOM1_Handler()
{
  COM1.IrqHandler();
}

void SERCOM3_Handler()
{
  COM2.IrqHandler();
}

//Function declarations
void init_IO();
void init_SD();
bool ascii_is_number();
uint32_t parse_sub_int_config();
void handle_config();
void generate_log_name();
void open_log_file();
void init_SERCOM();
void init_buffers();
void handle_status_led();
uint32_t return_buff_time();
void handle_com1();
void handle_com2();
void dump_com1_buff();
void dump_com2_buff();

//Initialize IO
void init_IO() {
  Serial.begin(USB_BAUD_RATE);
  //COM1.begin(COM1_BAUD_RATE_DEFAULT);
  //COM2.begin(COM2_BAUD_RATE_DEFAULT);

  pinMode(LED_BUILTIN, OUTPUT);
}

//Initialize COMs
void init_coms() {
  COM1.begin(com1_baud_rate);
  COM2.begin(com2_baud_rate);
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

//Check is ASCII character represents a number
bool ascii_is_number(char c) {
  return (c >= 48 && c <= 57);
  }

//Find and parse a numerical value after a substring in a string
uint32_t parse_sub_int_config(char str[], char sub[]) {
  char *p; //Pointer that points to a char
  char parsed_sub[20]; //Parsed number as string
  uint32_t parsed_sub_int = 0; //Parsed number
 
  p = strstr(str, sub); //Have pointer point to start of substring
  if (p == NULL) {
    return 0;
    }
  p += strlen(sub); //Point to character after substring

  int i = 0, j = 0;

  //Go through string until encountering a number
  while (!ascii_is_number(*(p + i))) {
    i++;
    }

  //Read numbers until reaching a non-number character
  while (ascii_is_number(*(p + i))) {
    parsed_sub[j] = *(p + i);
    Serial.println(*(p + i));
    Serial.println(parsed_sub[j]);
    j++;
    i++;
    }

  //Turn number string into an actual number
  for (i = j - 1; i >= 0; i--) {
    parsed_sub_int += (parsed_sub[i] - 48) * pow(10, j - i - 1);
    }
   
  return parsed_sub_int;
  }

//Parse/create config file and set baud rates accordingly
void handle_config() {
  //If config file doesn't exist, create one with default values
  if (!SD.exists(config_name)) {
    Serial.println("CONFIG NOT FOUND");
    config_file = SD.open(config_name, FILE_WRITE);

    //If opening fails, SD card comm failed. Reboot.
    if (!config_file) {
      Serial.println("CONFIG OPEN FAILED");
      NVIC_SystemReset();
      }

    //Fill config file with default values
    config_file.print("COM1: ");
    config_file.println(COM1_BAUD_RATE_DEFAULT);
    config_file.print("COM2: ");
    config_file.println(COM2_BAUD_RATE_DEFAULT);
    config_file.close();

    //Assign default baud rates
    com1_baud_rate = COM1_BAUD_RATE_DEFAULT;
    com2_baud_rate = COM2_BAUD_RATE_DEFAULT;

    Serial.print("COM1 = ");
    Serial.println(com1_baud_rate);
    Serial.print("COM2 = ");
    Serial.println(com2_baud_rate);
   
    return;
    }

  //If config file exists, open it
  Serial.println("OPENING CONFIG TO READ");
  config_file = SD.open(config_name);

  //If open fails, SD card comm failed. Reboot.
  if (!config_file) {
      Serial.println("CONFIG OPEN FAILED");
      NVIC_SystemReset();
      }
  Serial.println("CONFIG FOUND");

  //Read config file and put in buffer
  int i = 0;
  while (config_file.available() > 0) {
    config_buff[i++] = config_file.read();
    Serial.write(config_buff[i - 1]);
    }

  config_file.close();
  Serial.println("CONFIG CLOSED");

  //Parse baud rates from buffer
  com1_baud_rate = parse_sub_int_config(config_buff, "COM1:");
  Serial.println("COM1 PARSED");
  com2_baud_rate = parse_sub_int_config(config_buff, "COM2:");
  Serial.println("COM2 PARSED");

  //If baud rate is 0, parsing failed or invalid config file. Assign default values.
  if (com1_baud_rate == 0) {
    com1_baud_rate = COM1_BAUD_RATE_DEFAULT;
    }
  if (com2_baud_rate == 0) {
    com2_baud_rate = COM2_BAUD_RATE_DEFAULT;
    }

  Serial.print("COM1 = ");
  Serial.println(com1_baud_rate);
  Serial.print("COM2 = ");
  Serial.println(com2_baud_rate);
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
  char header[50];
  Serial.print("Opening ");
  Serial.println(log_name);

  log_file = SD.open(log_name, FILE_WRITE);

  //If file open fails, restart
  if (!log_file) {
    Serial.println("OPEN FAILED");
    NVIC_SystemReset();
  }

  sprintf(header, "COM1 BAUDRATE: %lu BPS", com1_baud_rate);
  log_file.println(header);
  sprintf(header, "COM2 BAUDRATE: %lu BPS", com2_baud_rate);
  log_file.println(header);
  log_file.println("");

  log_file.flush();
 
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
  memset(com1_buff, '\0', BUFFER_SIZE);
  memset(com2_buff, '\0', BUFFER_SIZE);
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

//Handle COM1 buffer
void handle_com1() {
  if (COM1.available() > 0) {
    char c = COM1.read();

    //c is a valid character
    if (c >= 32 && c <= 126) {
      char strChar[2];
      strChar[1] = '\0';
      strChar[0] = c;
      strcat(com1_buff, strChar); //Add to buffer string
    }
    //Replace tab with "[TAB]"
    else if (c == ASCII_TAB) {
      strcat(com1_buff, "[TAB]");
    }
    //Ignore line feeds
    else if (c == ASCII_LF) {
      //strcat(com1_buff, "[LF]");
      //com1_buff_flag = true;
      Serial.println("COM1 LF");
    }
    //Replace carriage return with "[CR]" and set flag, ending the buffer
    else if (c == ASCII_CR) {
      strcat(com1_buff, "[CR]");
      com1_buff_flag = true;
      Serial.println("COM1 CR");
    }
    //Replace invalid characters with ASCII hex code
    else {
      char hex_buff[10];
      sprintf(hex_buff, "[0x%02X]", c);
      strcat(com1_buff, hex_buff);
    }

    //Check if buffer is near full and set flag accordingly
    if (strlen(com1_buff) > BUFFER_SIZE - 7) {
      com1_buff_flag = true;
      Serial.println("COM1 BUFFER FULL");
    }

    //com1_timer = millis();
  }
}

//Handle COM2 buffer
void handle_com2() {
  if (COM2.available() > 0) {
    char c = COM2.read();

    //c is a valid character
    if (c >= 32 && c <= 126) {
      char strChar[2];
      strChar[1] = '\0';
      strChar[0] = c;
      strcat(com2_buff, strChar);
    }
    //Replace tab with "[TAB]"
    else if (c == ASCII_TAB) {
      strcat(com2_buff, "[TAB]");
    }
    //Ignore line feeds
    else if (c == ASCII_LF) {
      //strcat(com2_buff, "[LF]");
      //com2_buff_flag = true;
      Serial.println("COM2 LF");
    }
    //Replace carriage return with "[CR]" and set flag, ending the buffer
    else if (c == ASCII_CR) {
      strcat(com2_buff, "[CR]");
      com2_buff_flag = true;
      Serial.println("COM2 CR");
    }
    //Replace invalid characters with ASCII hex code
    else {
      char hex_buff[10];
      sprintf(hex_buff, "[0x%02X]", c);
      strcat(com2_buff, hex_buff);
    }

    //Check if buffer is near full and set flag accordingly
    if (strlen(com2_buff) > BUFFER_SIZE - 7) {
      com2_buff_flag = true;
      Serial.println("COM2 BUFFER FULL");
    }

    //com2_timer = millis();
  }
}

//Dump COM1 buffer into text file
void dump_com1_buff() {
  uint16_t l; //Number of bytes written to file
  char stamp[TIME_STAMP_MAX_LENGTH];

  //Check if buffer is not empty
  if (com1_buff[0] != '\0') {
    //Generate and print time stamp
    sprintf(stamp, com1_time_stamp_format, return_buff_time());
    Serial.println("");
    Serial.print(stamp);
    log_file.println("");
    log_file.print(stamp);

    //Dump buffer, save number of bytes written and save file without closing it
    Serial.write(com1_buff);
    l = log_file.write(com1_buff);
    log_file.flush();

    //If no bytes were written, communication with SD failed. Reboot.
    if (l == 0) {
      Serial.println("RESETTING");
      NVIC_SystemReset();
    }
  }

  //Clear buffer and deactivate flag
  memset(com1_buff, '\0', BUFFER_SIZE);
  com1_buff_flag = false;
  //com1_timer = 4294967295;
  //com1_timer = pow(2, 8 * sizeof(com1_timer)) - 1;
}

//Dump COM2 buffer into text file
void dump_com2_buff() {
  uint16_t l; //Number of bytes written to file
  char stamp[TIME_STAMP_MAX_LENGTH];

  //Check if buffer is not empty
  if (com2_buff[0] != '\0') {
    //Generate and print time stamp
    sprintf(stamp, com2_time_stamp_format, return_buff_time());
    Serial.println("");
    Serial.print(stamp);
    log_file.println("");
    log_file.print(stamp);

    //Dump buffer, save number of bytes written and save file without closing it
    Serial.write(com2_buff);
    l = log_file.write(com2_buff);
    log_file.flush();

    //If no bytes were written, communication with SD failed. Reboot.
    if (l == 0) {
      Serial.println("RESETTING");
      NVIC_SystemReset();
    }
  }

  //Clear buffer and deactivate flag
  memset(com2_buff, '\0', BUFFER_SIZE);
  com2_buff_flag = false;
  //com1_timer = 4294967295;
  //com2_timer = pow(2, 8 * sizeof(tx_in_timer)) - 1;
  //com2_timer = 4294967295;
}

void setup() {
  init_IO();

  delay(1000);
  Serial.println("HI");

  init_SD();

  handle_config();
 
  init_coms();
 
  generate_log_name();

  open_log_file();

  init_buffers();
 
  Serial.println("HELLO");
  led_timer = millis();
  //com1_timer = pow(2, 8 * sizeof(rx_out_timer)) - 1;
  //com2_timer = pow(2, 8 * sizeof(tx_in_timer)) - 1;
  //com1_timer = 2000000000;
  //com2_timer = 2000000000;
}

void loop() {

  /*
    if ((long)millis() > BUFFER_TIMEOUT + (long)com1_timer) {
    com1_buff_flag = true;
    com1_timer = 2000000000;
    Serial.println("COM1 TIMEOUT");
    }
    if ((long)millis()  > BUFFER_TIMEOUT + (long)com2_timer) {
    com2_buff_flag = true;
    com2_timer = 2000000000;
    Serial.println("COM2 TIMEOUT");
    Serial.println((long)millis());
    Serial.println((long)com2_timer);
    Serial.println((long)millis() - (long)com2_timer);
    }
  */
  handle_status_led();
 
  handle_com1();
  handle_com2();

  if (com1_buff_flag) {
    dump_com1_buff();
  }

  if (com2_buff_flag) {
    dump_com2_buff();
  }
}
