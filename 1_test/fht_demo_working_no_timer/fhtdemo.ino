/// @dir fs20demo
/// This example sends commands to the Conrad/ELV 868 MHz FS20 units via OOK.
// 2009-02-21 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php

// Note thar RFM12B radios are not really designed for OOK (on-off keying),
// but this can be simulated anyway by simply turning the transmitter on and 
// off via the SPI interface. Doing so takes about 25 usecs, so the delays
// used for encoding simple bit patterns need to be adjusted accordingly.

// Adpated by Paul-Armand Verhaegen based on http://sourceforge.net/p/opentrv/wiki/FHT%20Protocol/

#include <JeeLib.h>
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

void fs20cmd(uint16_t house, uint8_t addr, uint8_t cmd) {
	uint8_t sum = 6 + (house >> 8) + house + addr + cmd;
	for (uint8_t i = 0; i < 3; ++i) {
		sendBits(1, 13);
		sendBits(house >> 8, 8);
		sendBits(house, 8);
		sendBits(addr, 8);
		sendBits(cmd, 8);
		sendBits(sum, 8);
		sendBits(0, 1);
		delay(9);
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

void setup() {
    Serial.begin(57600);
    Serial.println("\n[fs20demo]");

    rf12_initialize(0, RF12_868MHZ);
    
    //rf12_control(0xA68a); // A68A    868.2500 MHz
    
    // Send 0xA67C 67C is 1660 in this command it set the freq to 868,3 = 860 + 1660 * 0.005
    // rf12_control(0xA67C);
    // rf12_control(cmd);
}

void loop() {  
  // Syn with FHT
  Serial.println("sync");
  
  uint8_t i = 241;
  while (i > 2) { 
//  for (uint8_t i = 243; i > 2; i = i-2 ) { 
    
    //rf12_recvDone();
   
    delay(780); // Educated guess that sending a command (twice) will take about 40ms    
//    if (sendTimer.poll(100)) {
//      Serial.println("polled");
//      pending = 1;
      //sendTimer.set(100);
//      }
    
//    if (pending && rf12_canSend()) {
//       pending = 0;
      Serial.println(i);     

       fhtcmd(0x325C, 0, 0x0c, i); //countdown timer for synchronization

       i = i - 2;
//     }
  }
  
  Serial.println("Waiting 4 + 0.5 * (HC2 & 7) seconds (in total)");  
  int stand_off = 4000 + 500 * (0x5C & 7);
  delay(stand_off); // 3 seconds pause
  
  Serial.println("Set half-way -> ends the sync process");          
  fhtcmd(0x325C, 0, 0x00, 0x3A); // broadcast to set valves to 50%, and they should all respond
  
  Serial.println("Period 115 + 0.5 * (HC2 & 7) seconds (in total)");        
  uint32_t period = 115000 + 500 * (0x5C & 7);
  uint16_t waittime = 500; // half a second, just to be back soon enough, can make this better if needed (with maximum wait time)
  uint32_t total_waittime = 0; // total time already waited
  
  for (i = 0 ; i < 10 ; i++) {
    while (total_waittime < period) {
      delay(waittime);
      Serial.println(total_waittime);
      total_waittime += waittime;
    }
    total_waittime = 0;
    // test ten cycles of transmissions
    if ((i % 2) == 0) {
      Serial.println("Sent open");
      fhtcmd(0x325C, 0, 6, 255); // open completely
    } else {
      Serial.println("Sent closed");
      fhtcmd(0x325C, 0, 6, 0); // broadcast to set valves to 50%, and they should all respond
    }
  }
}


