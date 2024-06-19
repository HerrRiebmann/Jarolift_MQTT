void InitCC1101(){
  // initialize the transceiver chip
  WriteLog("[INFO] - initializing the CC1101 Transceiver. If you get stuck here, it is probably not connected.", true);
  cc1101.init();
  cc1101.setSyncWord(syncWord, false);
  cc1101.setCarrierFreq(CFREQ_433);
  cc1101.disableAddressCheck();   // if not specified, will only display "packet received"
}

void CC1101Loop(){
  if (iset) {
    cc1101.cmdStrobe(CC1101_SCAL);
    delay(50);
    enterrx();
    iset = false;
    delay(200);
    attachInterrupt(RX_PORT, radio_rx_measure, CHANGE); // Interrupt on change of RX_PORT
  }

  CheckRxBuffer();
}

void CheckRxBuffer(){
  // Check if RX buffer is full
  if ((lowbuf[0] > 3650) && (lowbuf[0] < 4300) && (pbwrite >= 65) && (pbwrite <= 75)) {     // Decode received data...
    if (debug_log_radio_receive_all)
      WriteLog("[INFO] - received data", true);
    iset = true;
    ReadRSSI();
    pbwrite = 0;


    for (int i = 0; i <= 31; i++) {                          // extracting Hopcode
      if (lowbuf[i + 1] < hibuf[i + 1]) {
        rx_hopcode = rx_hopcode & ~(1 << i) | (0 << i);
      } else {
        rx_hopcode = rx_hopcode & ~(1 << i) | (1 << i);
      }
    }
    for (int i = 0; i <= 27; i++) {                         // extracting Serialnumber
      if (lowbuf[i + 33] < hibuf[i + 33]) {
        rx_serial = rx_serial & ~(1 << i) | (0 << i);
      } else {
        rx_serial = rx_serial & ~(1 << i) | (1 << i);
      }
    }
    rx_serial_array[0] = (rx_serial >> 24) & 0xFF;
    rx_serial_array[1] = (rx_serial >> 16) & 0xFF;
    rx_serial_array[2] = (rx_serial >> 8) & 0xFF;
    rx_serial_array[3] = rx_serial & 0xFF;

    for (int i = 0; i <= 3; i++) {                        // extracting function code
      if (lowbuf[61 + i] < hibuf[61 + i]) {
        rx_function = rx_function & ~(1 << i) | (0 << i);
      } else {
        rx_function = rx_function & ~(1 << i) | (1 << i);
      }
    }

    for (int i = 0; i <= 7; i++) {                        // extracting high disc
      if (lowbuf[65 + i] < hibuf[65 + i]) {
        rx_disc_h = rx_disc_h & ~(1 << i) | (0 << i);
      } else {
        rx_disc_h = rx_disc_h & ~(1 << i) | (1 << i);
      }
    }

    rx_disc_high[0] = rx_disc_h & 0xFF;
    rx_keygen ();
    rx_decoder();
    if (rx_function == 0x4)steadycnt++;           // to detect a long press....
    else steadycnt--;
    if (steadycnt > 10 && steadycnt <= 40) {
      rx_function = 0x3;
      steadycnt = 0;
    }

    Serial.printf(" serialnumber: 0x%08x // function code: 0x%02x // disc: 0x%02x\n\n", rx_serial, rx_function, rx_disc_h);

    // send mqtt message with received Data:
    if (mqtt_client.connected() && mqtt_send_radio_receive_all) {
      String Topic = "stat/" + config.mqtt_devicetopic + "/received";
      const char * msg = Topic.c_str();
      char payload[220];
      snprintf(payload, sizeof(payload),
               "{\"serial\":\"0x%08x\", \"rx_function\":\"0x%x\", \"rx_disc_low\":%d, \"rx_disc_high\":%d, \"RSSI\":%d, \"counter\":%d, \"rx_device_key_lsb\":\"0x%08x\", \"rx_device_key_msb\":\"0x%08x\", \"decoded\":\"0x%08x\"}",
               rx_serial, rx_function, rx_disc_low[0], rx_disc_h, value, rx_disc_low[3], rx_device_key_lsb, rx_device_key_msb, decoded );
      mqtt_client.publish(msg, payload);
    }

    rx_disc_h = 0;
    rx_hopcode = 0;
    rx_function = 0;
  }
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// CC1101 radio functions group
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//####################################################################
// Receive Routine
//####################################################################
void ICACHE_RAM_ATTR radio_rx_measure()
{
  static long LineUp, LineDown, Timeout;
  long LowVal, HighVal;
  int pinstate = digitalRead(RX_PORT); // Read current pin state
  if (micros() - Timeout > 3500) {
    pbwrite = 0;
  }
  if (pinstate)                       // pin is now HIGH, was low
  {
    LineUp = micros();                // Get actual time in LineUp
    LowVal = LineUp - LineDown;       // calculate the LOW pulse time
    if (LowVal < debounce) return;
    if ((LowVal > 300) && (LowVal < 4300))
    {
      if ((LowVal > 3650) && (LowVal < 4300)) {
        Timeout = micros();
        pbwrite = 0;
        lowbuf[pbwrite] = LowVal;
        pbwrite++;
      }
      if ((LowVal > 300) && (LowVal < 1000)) {
        lowbuf[pbwrite] = LowVal;
        pbwrite++;
        Timeout = micros();
      }
    }
  }
  else
  {
    LineDown = micros();          // line went LOW after being HIGH
    HighVal = LineDown - LineUp;  // calculate the HIGH pulse time
    if (HighVal < debounce) return;
    if ((HighVal > 300) && (HighVal < 1000))
    {
      hibuf[pbwrite] = HighVal;
    }
  }
} // void ICACHE_RAM_ATTR radio_rx_measure

//####################################################################
// Generation of the encrypted message (Hopcode)
//####################################################################
void keeloq () {
  Keeloq k(device_key_msb, device_key_lsb);
  unsigned int result = (disc << 16) | devcnt;  // Append counter value to discrimination value
  dec = k.encrypt(result);
} // void keeloq

//####################################################################
// Keygen generates the device crypt key in relation to the masterkey and provided serial number.
// Here normal key-generation is used according to 00745a_c.PDF Appendix G.
// https://github.com/hnhkj/documents/blob/master/KEELOQ/docs/AN745/00745a_c.pdf
//####################################################################
void keygen () {
  Keeloq k(config.ulMasterMSB, config.ulMasterLSB);
  uint64_t keylow = new_serial | 0x20000000;
  unsigned long enc = k.decrypt(keylow);
  device_key_lsb  = enc;              // Stores LSB devicekey 16Bit
  keylow = new_serial | 0x60000000;
  enc    = k.decrypt(keylow);
  device_key_msb  = enc;              // Stores MSB devicekey 16Bit

  Serial.printf(" created devicekey low: 0x%08x // high: 0x%08x\n", device_key_lsb, device_key_msb);
} // void keygen

//####################################################################
// Simple TX routine. Repetitions for simulate continuous button press.
// For standard commands (up/down/stop etc.) send the message two times.
//####################################################################
void radio_tx(int repetitions) {
  pack = (button << 60) | (new_serial << 32) | dec;
  for (int a = 0; a < repetitions; a++)
  {
    digitalWrite(TX_PORT, LOW);      // preamble: 1560us "on" level
    delayMicroseconds(1560);
    radio_tx_frame(12);              // sync: 12 sequences 400us "off" / 400us "on"
    digitalWrite(TX_PORT, HIGH);     // start of frame: 3960us "off" level
    delayMicroseconds(3960);

    for (int i = 0; i < 64; i++) {

      int out = ((pack >> i) & 0x1); // Bitmask to get MSB and send it first
      if (out == 0x1)
      {
        digitalWrite(TX_PORT, LOW);  // Simple encoding of bit state 1: 400us "on" / 800us "off"
        delayMicroseconds(PULSE_SHORT);
        digitalWrite(TX_PORT, HIGH);
        delayMicroseconds(PULSE_LONG);
      }
      else
      {
        digitalWrite(TX_PORT, LOW);  // Simple encoding of bit state 0: 800us "on" / 400us "off"
        delayMicroseconds(PULSE_LONG);
        digitalWrite(TX_PORT, HIGH);
        delayMicroseconds(PULSE_SHORT);
      }
    }
    radio_tx_group_h();              // Last 8Bit. For motor 8-16.

    digitalWrite(TX_PORT, HIGH);     // end of frame: 16ms "off" level
    delay(16);                       // delay in loop context is save for wdt
  }
} // void radio_tx

//####################################################################
// Sending of high_group_bits 8-16
//####################################################################
void radio_tx_group_h() {
  for (int i = 0; i < 8; i++) {
    int out = ((disc_h >> i) & 0x1); // Bitmask to get MSB and send it first
    if (out == 0x1)
    {
      digitalWrite(TX_PORT, LOW);    // Simple encoding of bit state 1
      delayMicroseconds(PULSE_SHORT);
      digitalWrite(TX_PORT, HIGH);
      delayMicroseconds(PULSE_LONG);
    }
    else
    {
      digitalWrite(TX_PORT, LOW);    // Simple encoding of bit state 0
      delayMicroseconds(PULSE_LONG);
      digitalWrite(TX_PORT, HIGH);
      delayMicroseconds(PULSE_SHORT);
    }
  }
} // void radio_tx_group_h

//####################################################################
// Generates sync-pulses ("off"/"on" transitions)
//####################################################################
void radio_tx_frame(int l) {
  for (int i = 0; i < l; ++i) {
    digitalWrite(TX_PORT, HIGH);
    delayMicroseconds(PULSE_SHORT);
    digitalWrite(TX_PORT, LOW);
    delayMicroseconds(PULSE_SHORT);
  }
} // void radio_tx_frame

//####################################################################
// Calculate device code from received serial number
//####################################################################
void rx_keygen () {
  Keeloq k(config.ulMasterMSB, config.ulMasterLSB);
  uint32_t keylow = rx_serial | 0x20000000;
  unsigned long enc = k.decrypt(keylow);
  rx_device_key_lsb  = enc;        // Stores LSB devicekey 16Bit
  keylow = rx_serial | 0x60000000;
  enc    = k.decrypt(keylow);
  rx_device_key_msb  = enc;        // Stores MSB devicekey 16Bit

  Serial.printf(" received devicekey low: 0x%08x // high: 0x%08x", rx_device_key_lsb, rx_device_key_msb);
} // void rx_keygen

//####################################################################
// Decoding of the hopping code
//####################################################################
void rx_decoder () {
  Keeloq k(rx_device_key_msb, rx_device_key_lsb);
  unsigned int result = rx_hopcode;
  decoded = k.decrypt(result);
  rx_disc_low[0] = (decoded >> 24) & 0xFF;
  rx_disc_low[1] = (decoded >> 16) & 0xFF;
  rx_disc_low[2] = (decoded >> 8) & 0xFF;
  rx_disc_low[3] = decoded & 0xFF;

  Serial.printf(" // decoded: 0x%08x\n", decoded);
} // void rx_decoder

//####################################################################
// calculate RSSI value (Received Signal Strength Indicator)
//####################################################################
void ReadRSSI()
{
  byte rssi = 0;
  rssi = (cc1101.readReg(CC1101_RSSI, CC1101_STATUS_REGISTER));
  if (rssi >= 128)
  {
    value = 255 - rssi;
    value /= 2;
    value += 74;
  }
  else
  {
    value = rssi / 2;
    value += 74;
  }
  Serial.print(" CC1101_RSSI ");
  Serial.println(value);
} // void ReadRSSI

//####################################################################
// put CC1101 to receive mode
//####################################################################
void enterrx() {
  cc1101.setRxState();
  delay(2);
  rx_time = micros();
  while (((marcState = cc1101.readStatusReg(CC1101_MARCSTATE)) & 0x1F) != 0x0D )
  {
    if (micros() - rx_time > 50000) break; // Quit when marcState does not change...
  }
} // void enterrx

//####################################################################
// put CC1101 to send mode
//####################################################################
void entertx() {
  cc1101.setTxState();
  delay(2);
  rx_time = micros();
  while (((marcState = cc1101.readStatusReg(CC1101_MARCSTATE)) & 0x1F) != 0x13 && 0x14 && 0x15)
  {
    if (micros() - rx_time > 50000) break; // Quit when marcState does not change...
  }
} // void entertx
