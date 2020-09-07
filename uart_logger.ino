/*
   P1 (RX) - 11
   P2 (RX) - 20 (SDA)
*/

//Libraries
#include "wiring_private.h" // pinPeripheral() function
#include <SPI.h>
#include <SD.h>

//UART parameters
#define USB_BAUD_RATE 115200
#define P1_BAUD_RATE_DEFAULT 9600
#define P2_BAUD_RATE_DEFAULT 9600

//Buffer and logging parameters
#define BUFFER_SIZE 250
#define BUFFER_TIMEOUT 2000
#define CONFIG_SIZE 100
#define TIME_STAMP_MAX_LENGTH 30
#define SD_CS 4

//General parameters
#define SD_ERROR_BLINK_DELAY 100
#define SD_ERROR_BLINK_NUM 10
#define STATUS_BLINK_ON 500
#define STATUS_BLINK_OFF 3000

#define SD_DETECT_PIN 7
#define BUTTON_PIN A1
#define P3_PIN A4
#define P4_PIN A5

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
Uart P1 (&sercom1, 11, 10, SERCOM_RX_PAD_0, UART_TX_PAD_2);
Uart P2 (&sercom3, 20, 6, SERCOM_RX_PAD_0, UART_TX_PAD_2);

//SD File object declaration
File log_file, config_file;

//Global constants and variables declaration
const char p1_time_stamp_format[] = "[P1-%07d]:";
const char p2_time_stamp_format[] = "[P2-%07d]:";
const char p3_time_stamp_format[] = "[P3-%07d]:";
const char p4_time_stamp_format[] = "[P4-%07d]:";
uint32_t led_timer, file_create_time;
uint32_t p1_baud_rate = P1_BAUD_RATE_DEFAULT, p2_baud_rate = P2_BAUD_RATE_DEFAULT;
char log_name_tmp[5] = "LOG_";
char log_name[13];
char config_name[] = "CONFIG.TXT";
char p1_buff[BUFFER_SIZE], p2_buff[BUFFER_SIZE], config_buff[CONFIG_SIZE]; //RAM buffers
bool p1_buff_flag = false, p2_buff_flag = false; //Flags: dump buffers to file?
bool halt_flag = false, is_sd_connected = false, sd_init_flag = false, p3_last_state, p4_last_state;
uint16_t indx = 0; //File index

//Link SERCOMs to declared UART channels
void SERCOM1_Handler()
{
  P1.IrqHandler();
}

void SERCOM3_Handler()
{
  P2.IrqHandler();
}

//Function declarations
void init_IO();
void isr_button();
void isr_sd();
void init_coms();
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
void handle_p1();
void handle_p2();
void dump_p1_buff();
void dump_p2_buff();

//Initialize IO
void init_IO() {
  Serial.begin(USB_BAUD_RATE);
  //P1.begin(P1_BAUD_RATE_DEFAULT);
  //P2.begin(P2_BAUD_RATE_DEFAULT);

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(P3_PIN, INPUT_PULLUP);
  pinMode(P4_PIN, INPUT_PULLUP);
  pinMode(SD_DETECT_PIN, INPUT_PULLUP);
 
  p3_last_state = digitalRead(P3_PIN);
  p4_last_state = digitalRead(P4_PIN);

  //If button pressed, set halt flag
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), isr_button, FALLING);

  //If SD card detected, set SD flag
  attachInterrupt(digitalPinToInterrupt(SD_DETECT_PIN), isr_sd, RISING);
}

//ISR function - set halt flag when button pressed/SD card removed
void isr_button() {
  halt_flag = true;
  }

//ISR function - set sd flag when card inserted
void isr_sd() {
  is_sd_connected = true;
  }

//Initialize COMs
void init_coms() {
  P1.begin(p1_baud_rate);
  P2.begin(p2_baud_rate);
  }

//Initialize SD card (legacy)
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

    //If opening fails, SD card comm failed. Halt.
    if (!config_file) {
      Serial.println("CONFIG OPEN FAILED");
      //NVIC_SystemReset();
      //halt_flag = true;
      halt_f();
      return;
      }

    //Fill config file with default values
    config_file.print("P1: ");
    config_file.println(P1_BAUD_RATE_DEFAULT);
    config_file.print("P2: ");
    config_file.println(P2_BAUD_RATE_DEFAULT);
    config_file.close();

    //Assign default baud rates
    p1_baud_rate = P1_BAUD_RATE_DEFAULT;
    p2_baud_rate = P2_BAUD_RATE_DEFAULT;

    Serial.print("P1 = ");
    Serial.println(p1_baud_rate);
    Serial.print("P2 = ");
    Serial.println(p2_baud_rate);
   
    return;
    }

  //If config file exists, open it
  Serial.println("OPENING CONFIG TO READ");
  config_file = SD.open(config_name);

  //If open fails, SD card comm failed. Halt.
  if (!config_file) {
      Serial.println("CONFIG OPEN FAILED");
      //NVIC_SystemReset();
      //halt_flag = true;
      halt_f();
      return;
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
  p1_baud_rate = parse_sub_int_config(config_buff, "P1:");
  Serial.println("P1 PARSED");
  p2_baud_rate = parse_sub_int_config(config_buff, "P2:");
  Serial.println("P2 PARSED");

  //If baud rate is 0, parsing failed or invalid config file. Assign default values.
  if (p1_baud_rate == 0) {
    p1_baud_rate = P1_BAUD_RATE_DEFAULT;
    }
  if (p2_baud_rate == 0) {
    p2_baud_rate = P2_BAUD_RATE_DEFAULT;
    }

  Serial.print("P1 = ");
  Serial.println(p1_baud_rate);
  Serial.print("P2 = ");
  Serial.println(p2_baud_rate);
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

  //If file open fails, halt
  if (!log_file) {
    Serial.println("OPEN FAILED");
    //halt_flag = true;
    halt_f();
    return;
    //NVIC_SystemReset();
  }

  sprintf(header, "P1 BAUDRATE: %lu BPS", p1_baud_rate);
  log_file.println(header);
  sprintf(header, "P2 BAUDRATE: %lu BPS", p2_baud_rate);
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
  memset(p1_buff, '\0', BUFFER_SIZE);
  memset(p2_buff, '\0', BUFFER_SIZE);
  }

//Update status led to blink slowly while sd connected and faster when not
void handle_status_led() {
  if (is_sd_connected && millis() - led_timer >= STATUS_BLINK_ON && digitalRead(LED_BUILTIN) == HIGH) {
    led_timer = millis();
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }
  if (is_sd_connected && millis() - led_timer >= STATUS_BLINK_OFF && digitalRead(LED_BUILTIN) == LOW) {
    led_timer = millis();
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }
  if (!is_sd_connected && millis() - led_timer >= SD_ERROR_BLINK_DELAY) {
    led_timer = millis();
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    }
}

//Return buffer dump time in seconds
uint32_t return_buff_time() {
  return (millis() - file_create_time) / 1000;
  }

//Handle P1 buffer
void handle_p1() {
  if (P1.available() > 0) {
    //Serial.println("P1 RECIEVED");
    char c = P1.read();

    //c is a valid character
    if (c >= 32 && c <= 126) {
      char strChar[2];
      strChar[1] = '\0';
      strChar[0] = c;
      strcat(p1_buff, strChar); //Add to buffer string
    }
    //Replace tab with "[TAB]"
    else if (c == ASCII_TAB) {
      strcat(p1_buff, "[TAB]");
    }
    //Ignore line feeds
    else if (c == ASCII_LF) {
      //strcat(p1_buff, "[LF]");
      //p1_buff_flag = true;
      Serial.println("P1 LF");
    }
    //Replace carriage return with "[CR]" and set flag, ending the buffer
    else if (c == ASCII_CR) {
      strcat(p1_buff, "[CR]");
      p1_buff_flag = true;
      Serial.println("P1 CR");
    }
    //Replace invalid characters with ASCII hex code
    else {
      char hex_buff[10];
      sprintf(hex_buff, "[0x%02X]", c);
      strcat(p1_buff, hex_buff);
    }

    //Check if buffer is near full and set flag accordingly
    if (strlen(p1_buff) > BUFFER_SIZE - 7) {
      p1_buff_flag = true;
      Serial.println("P1 BUFFER FULL");
    }

    //p1_timer = millis();
  }
}

//Handle P2 buffer
void handle_p2() {
  if (P2.available() > 0) {
    char c = P2.read();

    //c is a valid character
    if (c >= 32 && c <= 126) {
      char strChar[2];
      strChar[1] = '\0';
      strChar[0] = c;
      strcat(p2_buff, strChar);
    }
    //Replace tab with "[TAB]"
    else if (c == ASCII_TAB) {
      strcat(p2_buff, "[TAB]");
    }
    //Ignore line feeds
    else if (c == ASCII_LF) {
      //strcat(p2_buff, "[LF]");
      //p2_buff_flag = true;
      Serial.println("P2 LF");
    }
    //Replace carriage return with "[CR]" and set flag, ending the buffer
    else if (c == ASCII_CR) {
      strcat(p2_buff, "[CR]");
      p2_buff_flag = true;
      Serial.println("P2 CR");
    }
    //Replace invalid characters with ASCII hex code
    else {
      char hex_buff[10];
      sprintf(hex_buff, "[0x%02X]", c);
      strcat(p2_buff, hex_buff);
    }

    //Check if buffer is near full and set flag accordingly
    if (strlen(p2_buff) > BUFFER_SIZE - 7) {
      p2_buff_flag = true;
      Serial.println("P2 BUFFER FULL");
    }

    //p2_timer = millis();
  }
}

//Handle P3 digital input
void handle_p3() {
  uint16_t l; //Number of bytes written to file
  char stamp[TIME_STAMP_MAX_LENGTH];

  bool p3_current_state = digitalRead(P3_PIN);
 
  if (p3_current_state != p3_last_state) {
    p3_last_state = p3_current_state;

    sprintf(stamp, p3_time_stamp_format, return_buff_time());
    Serial.println("");
    Serial.print(stamp);
    Serial.println(p3_current_state);

    //Write to SD card if one is connected
    if (is_sd_connected) {
      log_file.println("");
      log_file.print(stamp);
      l = log_file.println(p3_current_state);
      log_file.flush();

      //If no bytes were written, communication with SD failed. Halt.
      if (l == 0) {
        Serial.println("ERROR, HALTING");
        halt_f();
        //halt_flag = true;
        //NVIC_SystemReset();
      }
    }
   
   

   
  }
}

void handle_p4() {
  uint16_t l; //Number of bytes written to file
  char stamp[TIME_STAMP_MAX_LENGTH];

  bool p4_current_state = digitalRead(P4_PIN);
 
  if (p4_current_state != p4_last_state) {
    p4_last_state = p4_current_state;

    sprintf(stamp, p4_time_stamp_format, return_buff_time());
    Serial.println("");
    Serial.print(stamp);
    Serial.println(p4_current_state);

    //Write to SD card if one is connected
    if (is_sd_connected) {
      log_file.println("");
      log_file.print(stamp);
      l = log_file.println(p4_current_state);
      log_file.flush();
 
      //If no bytes were written, communication with SD failed. Reboot.
      if (l == 0) {
        Serial.println("ERROR, HALTING");
        halt_f();
        //halt_flag = true;
        //NVIC_SystemReset();
      }
    }
  }
}

//Dump P1 buffer into serial monitor and text file (if card connected)
void dump_p1_buff() {
  uint16_t l; //Number of bytes written to file
  char stamp[TIME_STAMP_MAX_LENGTH];

  //Check if buffer is not empty
  if (p1_buff[0] != '\0') {
    //Generate and print time stamp
    sprintf(stamp, p1_time_stamp_format, return_buff_time());
    Serial.println("");
    Serial.print(stamp);
    Serial.write(p1_buff);

    if (is_sd_connected) {
      log_file.println("");
      log_file.print(stamp);    
      l = log_file.write(p1_buff);
      log_file.flush();
 
      //If no bytes were written, communication with SD failed. Halt.
      if (l == 0) {
        Serial.println("ERROR, HALTING");
        halt_f();
        //halt_flag = true;
        //NVIC_SystemReset();
      }
    }
  }

  //Clear buffer and deactivate flag
  memset(p1_buff, '\0', BUFFER_SIZE);
  p1_buff_flag = false;
}

//Dump P2 buffer into serial monitor and text file (if one is connected)
void dump_p2_buff() {
  uint16_t l; //Number of bytes written to file
  char stamp[TIME_STAMP_MAX_LENGTH];

  //Check if buffer is not empty
  if (p2_buff[0] != '\0') {
    //Generate and print time stamp
    sprintf(stamp, p2_time_stamp_format, return_buff_time());
    Serial.println("");
    Serial.print(stamp);
    Serial.write(p2_buff);

    if (is_sd_connected) {
      log_file.println("");
      log_file.print(stamp);    
      l = log_file.write(p2_buff);
      log_file.flush();
 
      //If no bytes were written, communication with SD failed. Reboot.
      if (l == 0) {
        Serial.println("ERROR, HALTING");
        halt_flag = true;
        //NVIC_SystemReset();
      }
    }
  }

  //Clear buffer and deactivate flag
  memset(p2_buff, '\0', BUFFER_SIZE);
  p2_buff_flag = false;
}

//Halt
void halt_f() {
  detachInterrupt(digitalPinToInterrupt(SD_DETECT_PIN));
  detachInterrupt(digitalPinToInterrupt(BUTTON_PIN));
  log_file.close();
  while (1) {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(200);
    }
  }

void setup() {
  init_IO();

  delay(1000);
  Serial.println("HI");
  init_coms();
  init_SERCOM();
  init_buffers();
 
  Serial.println("HELLO");
  led_timer = millis();
}

void loop() {
  handle_status_led();

  //If SD card was just connected, try to initialize file
  if (is_sd_connected && !sd_init_flag) {
    Serial.println("SD DETECTED");
    if (SD.begin(SD_CS)) {
      handle_config();
      P1.end();
      P2.end();
      init_coms();
      init_SERCOM();
      generate_log_name();
      open_log_file();

      //If SD card is disconnected, set halt flag
      detachInterrupt(digitalPinToInterrupt(SD_DETECT_PIN));
      attachInterrupt(digitalPinToInterrupt(SD_DETECT_PIN), isr_button, FALLING);

      //Make sure SD card isn't initialized again
      sd_init_flag = true;
      }
    else {
      Serial.println("SD INIT FAILED");
      is_sd_connected = false;
      }
    }
 
  handle_p1();
  handle_p2();
  handle_p3();
  handle_p4();
 
  if (p1_buff_flag) {
    dump_p1_buff();
  }

  if (p2_buff_flag) {
    dump_p2_buff();
  }

  if (halt_flag) {
    halt_f();
    }
}
