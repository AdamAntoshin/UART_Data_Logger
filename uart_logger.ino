/*
   P1 - 11
   P2 - 20 (SDA)
   P3 - A4
   P4 - A5
   P5 - 0 (RX)
   P6 - 5
*/

#define DEBUG //Comment out to disable debugging mode

#ifdef DEBUG
  #define DEBUG_PRINT(x) Serial.print(x)
  #define DEBUG_PRINTLN(x) Serial.println(x)
  #define DEBUG_WRITE(x) Serial.write(x)
#else
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x)
  #define DEBUG_WRITE(x)
#endif

//Libraries
#include "wiring_private.h" // pinPeripheral() function
#include <SPI.h>
#include <SD.h>

//UART parameters
#define USB_BAUD_RATE 115200
#define P1_BAUD_RATE_DEFAULT 9600
#define P2_BAUD_RATE_DEFAULT 9600
#define P5_BAUD_RATE_DEFAULT 9600
#define P6_BAUD_RATE_DEFAULT 9600

//Buffer and logging parameters
#define BUFFER_SIZE 250
#define BUFFER_TIMEOUT 2000
#define CONFIG_SIZE 100
#define TIME_STAMP_MAX_LENGTH 30
#define MAX_SIZE 1073741824
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
#define P5 Serial1
Uart P6 (&sercom2, 5, 2, SERCOM_RX_PAD_3, UART_TX_PAD_2);

//SD File object declaration
File log_file, config_file;

//Global constants and variables declaration
const char p1_time_stamp_format[] = "[P1-%07d]:";
const char p2_time_stamp_format[] = "[P2-%07d]:";
const char p3_time_stamp_format[] = "[P3-%07d]:";
const char p4_time_stamp_format[] = "[P4-%07d]:";
const char p5_time_stamp_format[] = "[P5-%07d]:";
const char p6_time_stamp_format[] = "[P6-%07d]:";
uint32_t led_timer, file_create_time = 0;
uint32_t p1_baud_rate = P1_BAUD_RATE_DEFAULT, p2_baud_rate = P2_BAUD_RATE_DEFAULT;
uint32_t p5_baud_rate = P5_BAUD_RATE_DEFAULT, p6_baud_rate = P6_BAUD_RATE_DEFAULT;
char log_name_tmp[5] = "LOG_";
char log_name[13];
char config_name[] = "CONFIG.TXT";
char p1_buff[BUFFER_SIZE], p2_buff[BUFFER_SIZE], config_buff[CONFIG_SIZE]; //RAM buffers
char p5_buff[BUFFER_SIZE], p6_buff[BUFFER_SIZE];
bool p1_buff_flag = false, p2_buff_flag = false; //Flags: dump buffers to file?
bool p5_buff_flag = false, p6_buff_flag = false;
bool halt_flag = false, is_sd_connected = false, sd_init_flag = false, p3_last_state, p4_last_state, di_override_flag = false;
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


void SERCOM2_Handler()
{
  P6.IrqHandler();
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
void handle_serial(Uart &P, char buff[], bool &flag, char p_name[]);
void handle_dinput(uint8_t dpin, bool &last_state, const char time_stamp_format[]);
void dump_serial_buff(char buff[], const char time_stamp_format[], bool &buff_flag, char p_name[]);
void handle_log_size();
void halt_f();

//Initialize IO
void init_IO() {
  Serial.begin(USB_BAUD_RATE);
  //P1.begin(P1_BAUD_RATE_DEFAULT);
  //P2.begin(P2_BAUD_RATE_DEFAULT);

  DEBUG_PRINTLN("Starting...");
 
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  //pinMode(P3_PIN, INPUT_PULLUP);
  //pinMode(P4_PIN, INPUT_PULLUP);
  pinMode(P3_PIN, INPUT);
  pinMode(P4_PIN, INPUT);
  pinMode(SD_DETECT_PIN, INPUT_PULLUP);

  DEBUG_PRINTLN("I/O PINS INITIALIZED");
 
  p3_last_state = digitalRead(P3_PIN);
  p4_last_state = digitalRead(P4_PIN);

  DEBUG_PRINTLN("P3+4 READ");

 
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
  P5.begin(p5_baud_rate);
  P6.begin(p6_baud_rate);
 
  DEBUG_PRINT("P1 Initialized at baud rate: ");
  DEBUG_PRINTLN(p1_baud_rate);
  DEBUG_PRINT("P2 Initialized at baud rate: ");
  DEBUG_PRINTLN(p2_baud_rate);
  DEBUG_PRINT("P5 Initialized at baud rate: ");
  DEBUG_PRINTLN(p5_baud_rate);
  DEBUG_PRINT("P6 Initialized at baud rate: ");
  DEBUG_PRINTLN(p6_baud_rate);
  }

void handle_SD_bootup() {
  if (digitalRead(SD_DETECT_PIN) == HIGH) {
    is_sd_connected = true;
    DEBUG_PRINTLN("SD DETECTED ON BOOTUP");
    }
  else {
    //If SD card detected, set SD flag
    attachInterrupt(digitalPinToInterrupt(SD_DETECT_PIN), isr_sd, RISING);
    DEBUG_PRINTLN("SD INTERRUPT ATTACHED");
    }
  }

//Initialize SD card (legacy)
void init_SD() {
  //If init fails, blink LED and try again
  while (!SD.begin(SD_CS)) {
    int i;
    DEBUG_PRINTLN("SD INIT FAILED, TRYING AGAIN");
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
    DEBUG_PRINT("Substring \"");
    DEBUG_PRINT(sub);
    DEBUG_PRINTLN("\" not found");
    return 0;
    }

  DEBUG_PRINT("Substring \"");
  DEBUG_PRINT(sub);
  DEBUG_PRINTLN("\" found");
  DEBUG_PRINT("Substring length: ");
  DEBUG_PRINTLN(strlen(sub));
 
  p += strlen(sub); //Point to character after substring

  int i = 0, j = 0;

  //Go through string until encountering a number
  while (!ascii_is_number(*(p + i))) {
    i++;
    }

  DEBUG_PRINT("Number found after ");
  DEBUG_PRINT(i);
  DEBUG_PRINTLN(" bytes");
 
  //Read numbers until reaching a non-number character
  while (ascii_is_number(*(p + i))) {
    parsed_sub[j] = *(p + i);
    Serial.println(*(p + i));
    Serial.println(parsed_sub[j]);
    j++;
    i++;
    }

  DEBUG_PRINT("Number length: ");
  DEBUG_PRINTLN(j);
 
  //Turn number string into an actual number
  for (i = j - 1; i >= 0; i--) {
    parsed_sub_int += (parsed_sub[i] - 48) * pow(10, j - i - 1);
    }
   
  DEBUG_PRINT("PARSED NUMBER: ");
  DEBUG_PRINT(parsed_sub_int);
 
  return parsed_sub_int;
  }

//Parse/create config file and set baud rates accordingly
void handle_config() {
  //If config file doesn't exist, create one with default values
  if (!SD.exists(config_name)) {
    DEBUG_PRINTLN("CONFIG NOT FOUND");
    config_file = SD.open(config_name, FILE_WRITE);

    //If opening fails, SD card comm failed. Halt.
    if (!config_file) {
      DEBUG_PRINTLN("CONFIG OPEN FAILED");
      //NVIC_SystemReset();
      //halt_flag = true;
      halt_f();
      return;
      }
     
    DEBUG_PRINTLN("CONFIG FILE CREATED");
   
    //Fill config file with default values
    config_file.print("P1: ");
    config_file.println(P1_BAUD_RATE_DEFAULT);
    config_file.print("P2: ");
    config_file.println(P2_BAUD_RATE_DEFAULT);
    config_file.print("P5: ");
    config_file.println(P5_BAUD_RATE_DEFAULT);
    config_file.print("P6: ");
    config_file.println(P6_BAUD_RATE_DEFAULT);
    config_file.close();

    DEBUG_PRINTLN("CONFIG FILE WRITTEN");
   
    //Assign default baud rates
    p1_baud_rate = P1_BAUD_RATE_DEFAULT;
    p2_baud_rate = P2_BAUD_RATE_DEFAULT;
    p5_baud_rate = P5_BAUD_RATE_DEFAULT;
    p6_baud_rate = P6_BAUD_RATE_DEFAULT;

    DEBUG_PRINT("P1 = ");
    DEBUG_PRINTLN(p1_baud_rate);
    DEBUG_PRINT("P2 = ");
    DEBUG_PRINTLN(p2_baud_rate);
    DEBUG_PRINT("P5 = ");
    DEBUG_PRINTLN(p5_baud_rate);
    DEBUG_PRINT("P6 = ");
    DEBUG_PRINTLN(p6_baud_rate);
   
    return;
    }

  //If config file exists, open it
  DEBUG_PRINTLN("OPENING CONFIG TO READ");
  config_file = SD.open(config_name);

  //If open fails, SD card comm failed. Halt.
  if (!config_file) {
      DEBUG_PRINTLN("CONFIG OPEN FAILED");
      //NVIC_SystemReset();
      //halt_flag = true;
      halt_f();
      return;
      }
  DEBUG_PRINTLN("CONFIG FOUND");

  //Read config file and put in buffer
  int i = 0;
  while (config_file.available() > 0) {
    config_buff[i++] = config_file.read();
    DEBUG_PRINTLN(config_buff[i - 1]);
    }

  config_file.close();
  DEBUG_PRINTLN("CONFIG CLOSED");

  //Parse baud rates from buffer
  p1_baud_rate = parse_sub_int_config(config_buff, "P1:");
  DEBUG_PRINTLN("P1 PARSED");
  p2_baud_rate = parse_sub_int_config(config_buff, "P2:");
  DEBUG_PRINTLN("P2 PARSED");
  p5_baud_rate = parse_sub_int_config(config_buff, "P5:");
  DEBUG_PRINTLN("P5 PARSED");
  p6_baud_rate = parse_sub_int_config(config_buff, "P6:");
  DEBUG_PRINTLN("P6 PARSED");

  //If baud rate is 0, parsing failed or invalid config file. Assign default values.
  if (p1_baud_rate == 0) {
    p1_baud_rate = P1_BAUD_RATE_DEFAULT;
    }
  if (p2_baud_rate == 0) {
    p2_baud_rate = P2_BAUD_RATE_DEFAULT;
    }
  if (p5_baud_rate == 0) {
    p5_baud_rate = P5_BAUD_RATE_DEFAULT;
    }
  if (p6_baud_rate == 0) {
    p6_baud_rate = P6_BAUD_RATE_DEFAULT;
    }

  DEBUG_PRINT("P1 = ");
  DEBUG_PRINTLN(p1_baud_rate);
  DEBUG_PRINT("P2 = ");
  DEBUG_PRINTLN(p2_baud_rate);
  DEBUG_PRINT("P5 = ");
  DEBUG_PRINTLN(p5_baud_rate);
  DEBUG_PRINT("P6 = ");
  DEBUG_PRINTLN(p6_baud_rate);
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
    DEBUG_PRINT(log_name);
    DEBUG_PRINTLN(" exists...");
    indx++;
    sprintf(log_name, "%s%04d.TXT", log_name_tmp, indx);
  }
}

//Open (create) new log file
void open_log_file() {
  char header[50];
  DEBUG_PRINT("Opening ");
  DEBUG_PRINTLN(log_name);

  log_file = SD.open(log_name, FILE_WRITE);

  //If file open fails, halt
  if (!log_file) {
    DEBUG_PRINTLN("OPEN FAILED");
    //halt_flag = true;
    halt_f();
    return;
    //NVIC_SystemReset();
  }

  sprintf(header, "P1 BAUDRATE: %lu BPS", p1_baud_rate);
  log_file.println(header);
  sprintf(header, "P2 BAUDRATE: %lu BPS", p2_baud_rate);
  log_file.println(header);
  sprintf(header, "P5 BAUDRATE: %lu BPS", p5_baud_rate);
  log_file.println(header);
  sprintf(header, "P6 BAUDRATE: %lu BPS", p6_baud_rate);
  log_file.println(header);
  log_file.println("");

  DEBUG_PRINTLN("LOG HEADER WRITTEN");
 
  log_file.flush();

  DEBUG_PRINTLN("LOG HEADER SAVED");
 
  //Mark file creation time for time stamps
  file_create_time = millis();
}

//Initialize SERCOM pins
void init_SERCOM() {
  // Assign pins 10 & 11 SERCOM functionality
  DEBUG_PRINTLN("Initializing SERCOM");
 
  pinPeripheral(10, PIO_SERCOM);
  pinPeripheral(11, PIO_SERCOM);
  pinPeripheral(20, PIO_SERCOM);
  pinPeripheral(5, PIO_SERCOM);

  DEBUG_PRINTLN("SERCOM Initialized");
}

//Fill both buffers with null terminator, "clearing" them
void init_buffers() {
  DEBUG_PRINTLN("Initializing BUFFERS");
  memset(p1_buff, '\0', BUFFER_SIZE);
  memset(p2_buff, '\0', BUFFER_SIZE);
  memset(p5_buff, '\0', BUFFER_SIZE);
  memset(p6_buff, '\0', BUFFER_SIZE);
  DEBUG_PRINTLN("BUFFERS Initialized");
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

void handle_test(char arr[], bool &b) {
  arr[0] = 'x';
  b = true;
  }

//Handle serial buffer
void handle_serial(Uart &P, char buff[], bool &flag, char p_name[]) {
  if (P.available() > 0) {
    //Serial.println("P1 RECIEVED");
    char c = P.read();

    //c is a valid character
    if (c >= 32 && c <= 126) {
      char strChar[2];
      strChar[1] = '\0';
      strChar[0] = c;
      strcat(buff, strChar); //Add to buffer string
    }
    //Replace tab with "[TAB]"
    else if (c == ASCII_TAB) {
      strcat(buff, "[TAB]");
    }
    //Ignore line feeds
    else if (c == ASCII_LF) {
      //strcat(p1_buff, "[LF]");
      //p1_buff_flag = true;
      //DEBUG_PRINTLN("\nP1 LF");
      DEBUG_PRINTLN("\n");
      DEBUG_WRITE(p_name);
      DEBUG_PRINTLN(" LF");
    }
    //Replace carriage return with "[CR]" and set flag, ending the buffer
    else if (c == ASCII_CR) {
      strcat(buff, "[CR]");
      flag = true;
      //DEBUG_PRINTLN("\nP1 CR");
      DEBUG_PRINTLN("");
      DEBUG_WRITE(p_name);
      DEBUG_PRINTLN(" CR");
    }
    //Replace invalid characters with ASCII hex code
    else {
      char hex_buff[10];
      sprintf(hex_buff, "[0x%02X]", c);
      strcat(buff, hex_buff);
    }

    //Check if buffer is near full and set flag accordingly
    if (strlen(buff) > BUFFER_SIZE - 7) {
      flag = true;
      //DEBUG_PRINTLN("\nP1 BUFFER FULL");
      DEBUG_PRINTLN("\n");
      DEBUG_WRITE(p_name);
      DEBUG_PRINTLN(" BUFFER FULL");
    }
  }
}

void handle_dinput(uint8_t dpin, bool &last_state, const char time_stamp_format[]) {
  uint16_t l; //Number of bytes written to file
  char stamp[TIME_STAMP_MAX_LENGTH];

  bool current_state = digitalRead(dpin);
 
  if (current_state != last_state || di_override_flag) {
    last_state = current_state;

    sprintf(stamp, time_stamp_format, return_buff_time());
    Serial.println("");
    Serial.print(stamp);
    Serial.println(current_state);

    //Write to SD card if one is connected
    if (is_sd_connected) {
      log_file.println("");
      log_file.print(stamp);
      l = log_file.println(current_state);
      log_file.flush();

      //If no bytes were written, communication with SD failed. Halt.
      if (l == 0) {
        DEBUG_PRINTLN("ERROR, HALTING");
        halt_f();
        //halt_flag = true;
        //NVIC_SystemReset();
      }
    }  
  }
}

//Dump P1 buffer into serial monitor and text file (if card connected)
void dump_serial_buff(char buff[], const char time_stamp_format[], bool &buff_flag, char p_name[]) {
  uint16_t l; //Number of bytes written to file
  char stamp[TIME_STAMP_MAX_LENGTH];

  //Check if buffer is not empty
  if (buff[0] != '\0') {
    //Generate and print time stamp
    sprintf(stamp, time_stamp_format, return_buff_time());
    Serial.println("");
    Serial.print(stamp);
    Serial.write(buff);

    if (is_sd_connected) {
      log_file.println("");
      log_file.print(stamp);    
      l = log_file.write(buff);
      log_file.flush();
 
      //If no bytes were written, communication with SD failed. Halt.
      if (l == 0) {
        DEBUG_PRINTLN("ERROR, HALTING");
        halt_f();
        //halt_flag = true;
        //NVIC_SystemReset();
      }
    }
  }

  //Clear buffer and deactivate flag
  memset(buff, '\0', BUFFER_SIZE);
  DEBUG_WRITE(p_name);
  DEBUG_PRINTLN(" BUFFER CLEARED");
 
  buff_flag = false;
}

void handle_log_size() {
  if (is_sd_connected) {
    if (log_file.size() > MAX_SIZE) {
      log_file.print("\n\nMAX SIZE REACHED");
      DEBUG_PRINTLN("MAX SIZE REACHED");
      DEBUG_PRINT(log_file.size());
      halt_f();
      }
    }
  }

//Halt
void halt_f() {
  detachInterrupt(digitalPinToInterrupt(SD_DETECT_PIN));
  detachInterrupt(digitalPinToInterrupt(BUTTON_PIN));
  DEBUG_PRINTLN("\nFILE CLOSED");
  log_file.println("\nFILE CLOSED");
  log_file.close();
  while (1) {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(200);
    }
  }

void setup() {
  init_IO();

  delay(1000);
 
  init_coms();
  init_SERCOM();
  init_buffers();
 
  di_override_flag = true;
  //handle_p3();
  //handle_p4();
  handle_dinput(P3_PIN, p3_last_state, p3_time_stamp_format);
  handle_dinput(P4_PIN, p4_last_state, p4_time_stamp_format);
  di_override_flag = false;

  handle_SD_bootup();
 
  led_timer = millis();
}

void loop() {
  /*
  bool x = false;
  char arrx[1];
  arrx[0] = 'y';
  DEBUG_PRINTLN("Before:");
  DEBUG_PRINT("X = "); DEBUG_PRINTLN(x);
  DEBUG_PRINT("arrx[0] = "); DEBUG_WRITE(arrx[0]); DEBUG_PRINTLN("");
  handle_test(arrx, x);
  DEBUG_PRINTLN("After:");
  DEBUG_PRINT("X = "); DEBUG_PRINTLN(x);
  DEBUG_PRINT("arrx[0] = "); DEBUG_WRITE(arrx[0]); DEBUG_PRINTLN("");
  */
 
  handle_status_led();

  //If SD card was *just* connected, try to initialize file
  if (is_sd_connected && !sd_init_flag) {
    DEBUG_PRINTLN("SD DETECTED");
    if (SD.begin(SD_CS)) {
      handle_config();
      P1.end();
      P2.end();
      P5.end();
      P6.end();
      init_coms();
      init_SERCOM();
      generate_log_name();
      open_log_file();

      di_override_flag = true;
      //handle_p3();
      //handle_p4();
      handle_dinput(P3_PIN, p3_last_state, p3_time_stamp_format);
      handle_dinput(P4_PIN, p4_last_state, p4_time_stamp_format);
      di_override_flag = false;
     
      //If button pressed, set halt flag
      attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), isr_button, FALLING);
      DEBUG_PRINTLN("HALT BUTTON INTERRUPT ATTACHED");
     
      //If SD card is disconnected, set halt flag
      detachInterrupt(digitalPinToInterrupt(SD_DETECT_PIN));
      DEBUG_PRINTLN("SD INTERRUPT DETACHED");
      attachInterrupt(digitalPinToInterrupt(SD_DETECT_PIN), isr_button, FALLING);
      DEBUG_PRINTLN("SD INTERRUPT ATTACHED (DISCONNECTION DETECTION)");
      //Make sure SD card isn't initialized again
      sd_init_flag = true;
      }
    else {
      DEBUG_PRINTLN("SD INIT FAILED");
      is_sd_connected = false;
      }
    }
 
  handle_serial(P1, p1_buff, p1_buff_flag, "P1");
  handle_serial(P2, p2_buff, p2_buff_flag, "P2");
  handle_dinput(P3_PIN, p3_last_state, p3_time_stamp_format);
  handle_dinput(P4_PIN, p4_last_state, p4_time_stamp_format);
  handle_serial(P5, p5_buff, p5_buff_flag, "P5");
  handle_serial(P6, p6_buff, p6_buff_flag, "P6");
 
  if (p1_buff_flag) {
    dump_serial_buff(p1_buff, p1_time_stamp_format, p1_buff_flag, "P1");
  }

  if (p2_buff_flag) {
    dump_serial_buff(p2_buff, p2_time_stamp_format, p2_buff_flag, "P2");
  }

  if (p5_buff_flag) {
    dump_serial_buff(p5_buff, p5_time_stamp_format, p5_buff_flag, "P5");
  }

  if (p6_buff_flag) {
    dump_serial_buff(p6_buff, p6_time_stamp_format, p6_buff_flag, "P6");
  }

  handle_log_size();

  if (halt_flag) {
    halt_f();
    }
}
