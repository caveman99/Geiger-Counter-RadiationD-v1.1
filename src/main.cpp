/*
   Geiger Counter

   This code interacts with the Alibaba RadiationD-v1.1 (CAJOE) Geiger counter
   board and creates an I2C interface compatible with the RadSense protocol.
   Connect the output of the Geiger counter to pin inputPin.

   Author: Andreas Spiess
   Based on initial work of Mark A. Heckler (@MkHeck, mark.heckler@gmail.com)
   Hacked by @crazyquark for Adafruit.io
   Added RadSens I2C protocol by @caveman99
   License: MIT License
   Please use freely with attribution. Thank you!
*/

#define PRINT_DEBUG_MESSAGES

#include <Arduino.h>
#include <Preferences.h>
#include <RingBuf.h>
#include <Wire.h>
#include <esp_task_wdt.h>

#define INPUT_PIN 12

#define PERIOD_LOG 1 // Logging period in seconds
#define WDT_TIMEOUT 10

#define RS_REG_COUNT 21
#define RS_DEFAULT_I2C_ADDRESS 0x66
#define RS_DEVICE_ID 0x7D
#define RS_FIRMWARE_VER 0x01
#define RS_DEFAULT_SENSITIVITY                                                 \
  18 // imp/uR (microroentgen, abbreviated “uR” is one-millionth of a roentgen)
#define RS_DEFAULT_HV_GENERATOR 1

// Device id, default value: 0x7D
// Size: 8 bit
#define RS_DEVICE_ID_RG 0x00

// Firmware version
// Size: 8 bit
#define RS_FIRMWARE_VER_RG 0x01

// Radiation intensity (dynamic period T < 123 sec), uR / h, 10 times larger
// Size: 24 bit
#define RS_RAD_INTENSY_DYNAMIC_RG 0x03

// Radiation intensity (static period T = 500 sec), uR / h, 10 times larger
// Size: 24 bit
#define RS_RAD_INTENSY_STATIC_RG 0x06

/*Contains the accumulated number of pulses registered by the module
since the last I2C data reading. The value is reset each
time it is read. Allows you to process directly the pulses
from the Geiger counter and implement other algorithms. The value is updated
when each pulse is registered.
Size: 16 bit */
#define RS_PULSE_COUNTER_RG 0x09

/*This register is used to change the device address when multiple
devices need to be connected to the same line at the same
time. By default, it contains the value 0x66. At the end of recording, the new
value is stored in the non-volatile memory of the microcontroller.
Size: 8 bit
Access: R/W*/
#define RS_DEVICE_ADDRESS_RG 0x10

/*Control register for a high-voltage voltage Converter. By
default, it is in the enabled state. To enable the HV generator,
write 1 to the register, and 0 to disable it. If you try to write other
values, the command is ignored.
Size: 8 bit
Access: R/W*/
#define RS_HV_GENERATOR_RG 0x11

/*Contains the value coefficient used for calculating
the radiation intensity. If necessary (for example, when installing a different
type of counter), the necessary sensitivity value in
imp/MKR is entered in the register. The default value is 105 imp/MKR. At the end
of recording, the new value is stored in the non-volatile memory of the
microcontroller.
Size: 16 bit
Access: R/W*/
#define RS_SENSITIVITY_RG 0x12

/*Control register for a indication diode. By
default, it is in the enabled state. To enable the indication,
write 1 to the register, and 0 to disable it. If you try to write other
values, the command is ignored.
Size: 8 bit
Access: R/W*/
#define RS_LED_CONTROL_RG 0x14

/*Control register for a low power mode. to enable send 1 to the register, and 0
to disable) Size: 8 bit Access: R/W*/
#define RS_LMP_MODE_RG 0x0C

unsigned long counts = 0; // Tube events
unsigned long lastCounts = 0;
uint16_t deltaCounts = 0; // Last counts
uint8_t i2cAddress, ledControl, hvGenerator, lmpMode = 0;
uint16_t sensitivity = 0;
unsigned long lastCountTime; // Time measurement
unsigned long lastSend = 0;
RingBuf<unsigned char, 32> txBuf;
byte OUT;
RingBuf<uint16_t, 500> slide_window;
float static_reading,
    dynamic_reading = 0; // Static reading for the last 5 minutes

Preferences preferences;

void IRAM_ATTR
ISR_impulse() { // Captures count of events from Geiger counter board
  counts++;
  deltaCounts++;
}

/****reset***/
void software_Reset() // Restarts program from beginning but does not reset the
                      // peripherals and registers
{
  Serial.print("resetting");
  esp_restart();
}

void requestEvent() {
  while (txBuf.pop(OUT))
    Wire.write(OUT);
}

void receiveEvent(int numBytes) {
  int command = Wire.read();
  switch (command) {
  case RS_DEVICE_ID_RG: // 0x00
    txBuf.push(RS_DEVICE_ID);
    break;
  case RS_FIRMWARE_VER_RG: // 0x01
    txBuf.push(RS_FIRMWARE_VER);
    break;
  case RS_RAD_INTENSY_DYNAMIC_RG: // 0x03
  {
    // multiply dynamic_reading by 10 and convert to 3 byte value
    uint32_t dynamicReading = dynamic_reading * 10;
    txBuf.push((dynamicReading >> 16) & 0xFF); // MSB
    txBuf.push((dynamicReading >> 8) & 0xFF);  // Middle byte
    txBuf.push(dynamicReading & 0xFF);         // LSB
    break;
  }
  case RS_RAD_INTENSY_STATIC_RG: // 0x06
  {
    // multiply static_reading by 10 and convert to 3 byte value
    uint32_t staticReading = static_reading * 10;
    txBuf.push((staticReading >> 16) & 0xFF); // MSB
    txBuf.push((staticReading >> 8) & 0xFF);  // Middle byte
    txBuf.push(staticReading & 0xFF);         // LSB
    break;
  }
  case RS_PULSE_COUNTER_RG:         // 0x09
    txBuf.push(deltaCounts >> 8);   // MSB
    txBuf.push(deltaCounts & 0xFF); // LSB
    deltaCounts = 0;                // Reset the delta counts after reading
    break;
  case RS_DEVICE_ADDRESS_RG: // 0x10
    if (numBytes > 1) {
      uint8_t newAddress = Wire.read();
      preferences.begin("I2C", false);
      preferences.putUChar("ADDRESS", newAddress);
      preferences.end();
      software_Reset(); // Restart the ESP32 to apply the new address
    } else {
      txBuf.push(preferences.getUChar("ADDRESS", RS_DEFAULT_I2C_ADDRESS));
    }
    break;
  case RS_HV_GENERATOR_RG: // 0x11
    if (numBytes > 1) {
      uint8_t newHV = Wire.read();
      if (newHV < 2) {
        preferences.begin("I2C", false);
        preferences.putUChar("HV_GENERATOR", newHV);
        preferences.end();
        hvGenerator = newHV;
      }
    } else {
      txBuf.push(preferences.getUChar("HV_GENERATOR", RS_DEFAULT_HV_GENERATOR));
    }
    break;
  case RS_SENSITIVITY_RG: // 0x12
    if (numBytes > 1) {
      uint16_t newSensitivity = Wire.read() << 8;
      newSensitivity |= Wire.read();
      preferences.begin("I2C", false);
      preferences.putUShort("SENSITIVITY", newSensitivity);
      preferences.end();
      sensitivity = newSensitivity;
    } else {
      txBuf.push(preferences.putUShort("SENSITIVITY", RS_DEFAULT_SENSITIVITY) >>
                 8); // MSB
      txBuf.push(preferences.putUShort("SENSITIVITY", RS_DEFAULT_SENSITIVITY) &
                 0xFF); // LSB
    }
    break;
  case RS_LED_CONTROL_RG: // 0x14
    if (numBytes > 1) {
      uint8_t newLED = Wire.read();
      if (newLED < 2) {
        preferences.begin("I2C", false);
        preferences.putUChar("LED_CONTROL", newLED);
        preferences.end();
        ledControl = newLED;
      }
    } else {
      txBuf.push(preferences.getUChar("LED_CONTROL", 1));
    }
    break;
  case RS_LMP_MODE_RG: // 0x0C
    if (numBytes > 1) {
      uint8_t newLMP = Wire.read();
      if (newLMP < 2) {
        preferences.begin("I2C", false);
        preferences.putUChar("LMP_MODE", newLMP);
        preferences.end();
        lmpMode = newLMP;
      }
    } else {
      txBuf.push(preferences.getUChar("LMP_MODE", 0));
    }
    break;

  default: // any other command sent
    break;
  }
  // empty buffer
  while (txBuf.pop(OUT))
    Wire.write(OUT);
}

void setup() {
  Serial.begin(115200);
  pinMode(INPUT_PIN, INPUT); // Set pin for capturing Tube events
  attachInterrupt(INPUT_PIN, ISR_impulse, FALLING);
  Serial.println("GC Initialized");

  esp_task_wdt_config_t wdt_config = {.timeout_ms = WDT_TIMEOUT * 1000,
                                      .idle_core_mask =
                                          (1 << portNUM_PROCESSORS) - 1,
                                      .trigger_panic = true};
  esp_task_wdt_init(&wdt_config);
  esp_task_wdt_add(NULL); // add current thread to WDT watch

  preferences.begin("I2C", false);
  i2cAddress = preferences.getUInt("ADDRESS", RS_DEFAULT_I2C_ADDRESS);
  ledControl = preferences.getUChar("LED_CONTROL", 1);
  hvGenerator = preferences.getUChar("HV_GENERATOR", RS_DEFAULT_HV_GENERATOR);
  sensitivity = preferences.getUShort("SENSITIVITY", RS_DEFAULT_SENSITIVITY);
  lmpMode = preferences.getUChar("LMP_MODE", 0);
  preferences.end();

  Wire.begin(i2cAddress);
  Wire.onRequest(requestEvent);
  Wire.onReceive(receiveEvent);
  Serial.println("I2C Initialized");
}

void loop() {
  esp_task_wdt_reset();
  unsigned long currentTime = millis();
  if (currentTime - lastCountTime > (PERIOD_LOG * 1000)) {
    static_reading = 0;
    slide_window.pushOverwrite(
        counts - lastCounts); // Push the new count into the ring buffer
    lastCounts = counts;
    lastCountTime = currentTime;
    static_reading = 0;
    for (uint8_t j = 0; j < slide_window.size(); j++)
      static_reading += slide_window[j];
    static_reading =
        (static_reading * 3600) / (slide_window.size() * sensitivity);
    // only sample the last 60 seconds for dynamic reading
    dynamic_reading = 0;
    for (uint8_t j = 0; j < min(slide_window.size(), (uint16_t)60); j++)
      dynamic_reading += slide_window[j];
    dynamic_reading = (dynamic_reading * 3600) /
                      (min(slide_window.size(), (uint16_t)60) * sensitivity);

    if (ledControl == 1) {
      digitalWrite(LED_BUILTIN, HIGH);
      sleep(100);
      digitalWrite(LED_BUILTIN, LOW);
    } else {
      digitalWrite(LED_BUILTIN, LOW);
    }
  }
  delay(50);
}
