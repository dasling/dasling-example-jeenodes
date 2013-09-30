/*
 * JeeNode Wireless Blinking LED - Receiver.
 *
 * A very basic example of how to set an LED status (turning it
 * on and off) based on the value received from another JeeNode.
 *
 * Copyright (c) 2013 Davide Alberani <da@erlug.linux.it>
 *
 * It's assumed that you've already set the nodes IDs following
 * the guide in http://jeelabs.net/projects/cafe/wiki/POF_03_-_Wireless_light_sensor
 * This example is absolutely NOT optimized, and will consume a lot
 * of power unnecessarily; in the same page, you can find many improvements.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <JeeLib.h>

# define LED_PIN 9

uint8_t led_status = 0;
int useHex = 0;

static void showNibble (byte nibble) {
  char c = '0' + (nibble & 0x0F);
  if (c > '9')
    c += 7;
  Serial.print(c);
}

static void showByte (byte value) {
  if (useHex) {
    showNibble(value >> 4);
    showNibble(value);
  } else
    Serial.print((int) value);
}

#include <util/parity.h>

//MilliTimer sendTimer;
//uint8_t pending = 0;

void sendBits(uint16_t data, uint8_t bits) {
    if (bits == 8) {
        ++bits;
        data = (data << 1) | parity_even_bit(data);
    }
    for (uint16_t mask = bit(bits-1); mask != 0; mask >>= 1) {
        // Timing values empirically obtained, and used to adjust for on/off
        // delay in the RF12. The actual on-the-air bit timing we're after is
        // 600/600us for 1 and 400/400us for 0 - but to achieve that the RFM12B
        // needs to be turned on a bit longer and off a bit less. In addition
        // there is about 25 uS overhead in sending the on/off command over SPI.
        // With thanks to JGJ Veken for his help in getting these values right.
        int width = data & mask ? 600 : 400;
        rf12_onOff(1);
        delayMicroseconds(width + 150);
        rf12_onOff(0);
        delayMicroseconds(width - 200);
    }
}

void fhtcmd(uint16_t house, uint8_t addr, uint8_t cmd, uint8_t ext) {
	// normal operation, use addr = 0 (broadcast)

        // add the flags (nibble) to the commands (nibble) = 1 byte
        uint8_t flag_battery = 0x00;    // bit 4 of high nibble
        uint8_t flag_extension = 0x20;  // bit 5 of high nibble (always on)
        uint8_t flag_bidir = 0x00;  // bit 6 of high nibble (bidirectional command between regulators and central)        
        uint8_t flag_repetition = 0x00;  // bit 7 of high nibble (1 if a repetition)
        uint8_t flags = flag_battery | flag_extension | flag_bidir | flag_repetition;
        
        // add the flags to the command
        uint8_t cmd_and_flags = cmd | flags; 

        uint8_t sum = 0x0c + (house >> 8) + house + addr + cmd_and_flags + ext;
        
	for (uint8_t i = 0; i < 3; ++i) { // data packets are always sent two times for safety
		sendBits(1, 13);
		sendBits(house >> 8, 8);
		sendBits(house, 8);
		sendBits(addr, 8);
		sendBits(cmd_and_flags, 8);
		sendBits(ext, 8);  // erweiterungsbyte
		sendBits(sum, 8);
		sendBits(0, 1);
		delay(10);
	}
        Serial.print("\nSend (DEC): "); Serial.print(house >> 8); Serial.print(','); Serial.print(house & 0x00ff); Serial.print(','); Serial.print(addr), 
        Serial.print(','); Serial.print(cmd_and_flags); Serial.print(','); Serial.print(ext); Serial.print(','); Serial.print(sum);
        Serial.println("");    
        Serial.print("Send (HEX): "); Serial.print(house >> 8, HEX); Serial.print(','); Serial.print(house & 0x00ff, HEX); Serial.print(','); Serial.print(addr, HEX), 
        Serial.print(','); Serial.print(cmd_and_flags, HEX); Serial.print(','); Serial.print(ext, HEX); Serial.print(','); Serial.print(sum, HEX);
        Serial.println("");    

}

// Compute the MODBUS RTU CRC
uint16_t modbus_crc (volatile uint8_t buf[], int len)
{
  uint16_t crc = 0xFFFF;
 
  for (int pos = -1; pos < len; pos++) {
    if (pos == -1) {
      crc ^= (uint16_t)0x2D;          // Added the sync byte
    }
    else {
      crc ^= (uint16_t)buf[pos];          // XOR byte into least sig. byte of crc
    }
    
    for (int i = 8; i != 0; i--) {    // Loop over each bit
      if ((crc & 0x0001) != 0) {      // If the LSB is set
        crc >>= 1;                    // Shift right and XOR 0xA001
        crc ^= 0xA001;
      }
      else                            // Else LSB is not set
        crc >>= 1;                    // Just shift right
    }
  }
  // Note, this number has low and high bytes swapped, so use it accordingly (or swap bytes)
  uint8_t temp_high_value = (uint8_t)(crc >> 8);      // Save high byte value
  crc <<= 8;                                          // Store low byte value in high byte
  crc = crc | temp_high_value;
  
  return crc;  
}
    
void setup () {
    // initialize the serial port and the RF12 driver
    Serial.begin(57600);
    rf12_config();
    // set up easy transmissions
    rf12_easyInit(5);
    // set LED port to output
    pinMode(LED_PIN, OUTPUT);
} 
  
  
void loop() {
  // check if something has been received and if it's correct.
  if (!rf12_recvDone()) {
    return;
  }
  
  // Check the CRC
  uint16_t received_modbus_crc16 = (rf12_buf[3 + rf12_len] << 8) | rf12_buf[4 + rf12_len];
  uint16_t calculated_modbus_crc16 = modbus_crc(rf12_buf, 3 + rf12_len);
  
  if (received_modbus_crc16 == calculated_modbus_crc16) {
    //Serial.print("Received Modbus CRC16: OK");
  } 
  else {
    Serial.print("Modbus CRC16: WRONG - ");
    Serial.print("Calculated: ");Serial.print(calculated_modbus_crc16); Serial.print(", Received: ");Serial.println(received_modbus_crc16);
    return; 
  }
  
  uint8_t n = rf12_len;
  Serial.print(' ');
    showByte(rf12_hdr);
    for (byte i = 0; i < n; ++i) {
      if (!useHex)
        Serial.print(' ');
      showByte(rf12_data[i]);
    }
  Serial.println();

  if (rf12_len != sizeof led_status) {
    Serial.print("WRONG DATA SIZE -");
    Serial.print("Data length received:"); Serial.print(rf12_len); Serial.print(", expected data length of:"); Serial.println(sizeof led_status); 
    return;
  }
  
  // copy the data in another memory location
  memcpy(&led_status, (int*) rf12_data, sizeof led_status);
    
  // turn on or off the LED, accordingly to the received value
  Serial.println("Turning on/off led");
  digitalWrite(LED_PIN, led_status);
  
  Serial.println("Setting up for FHT");
  // Init_for_868.35 OOK FHT8v control

    rf12_control(0x0000); // SPI transfer added to avoid power-up problems 
    rf12_control(0x8205); // DC (disable clk pin), enable lbd
    rf12_control(0x80C7 | (RF12_868MHZ << 4)); // EL (ena TX), EF (ena RX FIFO), 12.0pF
    
    rf12_control(0x8209); // PAVE: Added this according to http://jeelabs.net/projects/cafe/wiki/Receiving_OOKASK_with_a_modified_RFM12B
    //rf12_xfer(0xA640); // 868MHz
    //rf12_xfer(0xA67C); // PAVE changed this to 868,3MHz
    rf12_control(0xA68a); // PAVE changed this to 868,25MHz

    //rf12_control(0xC606); // approx 49.2 Kbps, i.e. 10000/29/(1+6) Kbps

    rf12_control(0x94A0); // PAVE: VDI,FAST,134kHz,0dBm,-103dBm
    //rf12_xfer(0x94A2); // VDI,FAST,134kHz,0dBm,-91dBm
    rf12_control(0xC2AC); // AL,!ml,DIG,DQD4
    rf12_control(0xCA83);
    rf12_control(0xCE00 | 212); // PAVE
    //if (group != 0) {
    //    rf12_xfer(0xCA83); // FIFO8,2-SYNC,!ff,DR
    //    rf12_xfer(0xCE00 | group); // SYNC=2DXX；
    //} else {
    //    rf12_xfer(0xCA8B); // FIFO8,1-SYNC,!ff,DR
    //    rf12_xfer(0xCE2D); // SYNC=2D；
    //}
    rf12_control(0xC483); // @PWR,NO RSTRIC,!st,!fi,OE,EN
    rf12_control(0x9850); // !mp,90kHz,MAX OUT
    rf12_control(0xCC77); // OB1，OB0, LPX,！ddy，DDIT，BW0
    rf12_control(0xE000); // NOT USE
    rf12_control(0xC800); // NOT USE
    rf12_control(0xC040); // PAVE: 1.66MHz,2.2V
    //rf12_xfer(0xC049); // 1.66MHz,3.1V
    detachInterrupt(0);

  Serial.println("Two minutes of FHT control signals");
    // Send out two minutes commands to steer value  
    for (uint8_t i2 = 0 ; i2 < 120 ; i2++) {
      delay(500);
      // test ten cycles of transmissions
      if ((led_status % 2) == 0) {
        Serial.println("-O");
        fhtcmd(0x325C, 0, 6, 255); // open completely
      } else {
        Serial.println("-C");
        fhtcmd(0x325C, 0, 6, 0); // broadcast to set valves to 50%, and they should all respond
      }
    } 
    
  Serial.println("Reinit for Jeenode communication");  
    // Set back for receiving jeenode communication
    rf12_initialize(12,RF12_868MHZ,212);

}

