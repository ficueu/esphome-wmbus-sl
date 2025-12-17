#include "rf_cc1101.h"

namespace esphome
{
  namespace wmbus
  {

    static const char *TAG = "rxLoop";

    static bool wait_marcstate_(const char *phase, uint8_t target, uint32_t timeout_ms)
    {
      const uint32_t t0 = millis();
      uint32_t last_log = t0;

      while (true)
      {
        const uint8_t marc = ELECHOUSE_cc1101.SpiReadStatus(CC1101_MARCSTATE);
        if (marc == target)
          return true;

        const uint32_t now = millis();

        // co ~250ms daj log "żyję i czekam"
        if (now - last_log >= 250)
        {
          ESP_LOGW(TAG, "%s: waiting for MARCSTATE=0x%02X, now=0x%02X (elapsed=%ums)",
                   phase, target, marc, (unsigned)(now - t0));
          last_log = now;
        }

        if (now - t0 >= timeout_ms)
        {
          ESP_LOGE(TAG, "%s: TIMEOUT waiting for MARCSTATE=0x%02X, last=0x%02X (elapsed=%ums)",
                   phase, target, marc, (unsigned)(now - t0));
          return false;
        }

        delay(1); // krytyczne: oddaje CPU -> watchdog nie strzeli
      }
    }

    bool RxLoop::init(uint8_t mosi, uint8_t miso, uint8_t clk, uint8_t cs,
                      uint8_t gdo0, uint8_t gdo2, float freq, bool syncMode)
    {
      bool retVal = false;
      this->syncMode = syncMode;
      this->gdo0 = gdo0;
      this->gdo2 = gdo2;
      pinMode(this->gdo0, INPUT);
      pinMode(this->gdo2, INPUT);
      ESP_LOGE("wmbus", "init stage1");
      ELECHOUSE_cc1101.setSpiPin(clk, miso, mosi, cs);
      ESP_LOGE("wmbus", "init stage2");
      ELECHOUSE_cc1101.Init();
      ESP_LOGE("wmbus", "init stage3");
      for (uint8_t i = 0; i < TMODE_RF_SETTINGS_LEN; i++)
      {
        ELECHOUSE_cc1101.SpiWriteReg(TMODE_RF_SETTINGS_BYTES[i << 1],
                                     TMODE_RF_SETTINGS_BYTES[(i << 1) + 1]);
      }
      ESP_LOGE("wmbus", "init stage4");
      uint32_t freq_reg = uint32_t(freq * 65536 / 26);
      uint8_t freq2 = (freq_reg >> 16) & 0xFF;
      uint8_t freq1 = (freq_reg >> 8) & 0xFF;
      uint8_t freq0 = freq_reg & 0xFF;

      ESP_LOGD(TAG, "Set CC1101 frequency to %3.3fMHz [%02X %02X %02X]",
               freq / 1.0, freq2, freq1, freq0);
      // don't use setMHZ() -- seems to be broken, or used in wrong place
      ELECHOUSE_cc1101.SpiWriteReg(CC1101_FREQ2, freq2);
      ELECHOUSE_cc1101.SpiWriteReg(CC1101_FREQ1, freq1);
      ELECHOUSE_cc1101.SpiWriteReg(CC1101_FREQ0, freq0);

      ELECHOUSE_cc1101.SpiStrobe(CC1101_SCAL);

      byte cc1101Version = ELECHOUSE_cc1101.SpiReadStatus(CC1101_VERSION);

      if ((cc1101Version != 0) && (cc1101Version != 255))
      {
        retVal = true;
        ESP_LOGD(TAG, "CC1101 version '%d'", cc1101Version);
        ELECHOUSE_cc1101.SetRx();
        ESP_LOGD(TAG, "CC1101 initialized");
        delay(4);
      }
      else
      {
        ESP_LOGE(TAG, "CC1101 initialization FAILED!");
      }

      return retVal;
    }

    bool RxLoop::task()
    {
      do
      {
        ESP_LOGV(TAG, "task(): state=%d gdo0=%d gdo2=%d marc=0x%02X",
                 rxLoop.state, digitalRead(this->gdo0), digitalRead(this->gdo2),
                 ELECHOUSE_cc1101.SpiReadStatus(CC1101_MARCSTATE));

        switch (rxLoop.state)
        {
        case INIT_RX:
          start();
          return false;

        // RX active, waiting for SYNC
        case WAIT_FOR_SYNC:
          if (digitalRead(this->gdo2))
          { // assert when SYNC detected
            ESP_LOGD(TAG, "SYNC detected (GDO2=1) -> WAIT_FOR_DATA");
            rxLoop.state = WAIT_FOR_DATA;
            sync_time_ = millis();
          }
          break;

        // waiting for enough data in Rx FIFO buffer
        case WAIT_FOR_DATA:
          if (digitalRead(this->gdo0))
          { // assert when Rx FIFO buffer threshold reached
            ESP_LOGD(TAG, "FIFO threshold (GDO0=1) -> parse preamble");
            uint8_t preamble[2];
            // Read the 3 first bytes,
            ELECHOUSE_cc1101.SpiReadBurstReg(CC1101_RXFIFO, rxLoop.pByteIndex, 3);
            rxLoop.bytesRx = 3;
            const uint8_t *currentByte = rxLoop.pByteIndex;
            // Mode C
            if (*currentByte == WMBUS_MODE_C_PREAMBLE)
            {
              currentByte++;
              data_in.mode = 'C';
              // Block A
              if (*currentByte == WMBUS_BLOCK_A_PREAMBLE)
              {
                currentByte++;
                rxLoop.lengthField = *currentByte;
                rxLoop.length = 2 + packetSize(rxLoop.lengthField);
                data_in.block = 'A';
              }
              // Block B
              else if (*currentByte == WMBUS_BLOCK_B_PREAMBLE)
              {
                currentByte++;
                rxLoop.lengthField = *currentByte;
                rxLoop.length = 2 + 1 + rxLoop.lengthField;
                data_in.block = 'B';
              }
              // Unknown type, reinit loop
              else
              {
                rxLoop.state = INIT_RX;
                return false;
              }
              // don't include C "preamble"
              *(rxLoop.pByteIndex) = rxLoop.lengthField;
              rxLoop.pByteIndex += 1;
            }
            // Mode T Block A
            else if (decode3OutOf6(rxLoop.pByteIndex, preamble))
            {
              rxLoop.lengthField = preamble[0];
              data_in.lengthField = rxLoop.lengthField;
              rxLoop.length = byteSize(packetSize(rxLoop.lengthField));
              data_in.mode = 'T';
              data_in.block = 'A';
              rxLoop.pByteIndex += 3;
            }
            // Unknown mode, reinit loop
            else
            {
              rxLoop.state = INIT_RX;
              return false;
            }

            rxLoop.bytesLeft = rxLoop.length - 3;

            if (rxLoop.length < MAX_FIXED_LENGTH)
            {
              // Set CC1101 into length mode
              ELECHOUSE_cc1101.SpiWriteReg(CC1101_PKTLEN, (uint8_t)rxLoop.length);
              ELECHOUSE_cc1101.SpiWriteReg(CC1101_PKTCTRL0, FIXED_PACKET_LENGTH);
              rxLoop.cc1101Mode = FIXED;
            }
            else
            {
              // Set CC1101 into infinite mode
              ELECHOUSE_cc1101.SpiWriteReg(CC1101_PKTLEN, (uint8_t)(rxLoop.length % MAX_FIXED_LENGTH));
            }

            rxLoop.state = READ_DATA;
            max_wait_time_ += extra_time_;

            ELECHOUSE_cc1101.SpiWriteReg(CC1101_FIFOTHR, RX_FIFO_THRESHOLD);
          }
          break;

        // waiting for more data in Rx FIFO buffer
        case READ_DATA:
          if (digitalRead(this->gdo0))
          { // assert when Rx FIFO buffer threshold reached
            if ((rxLoop.bytesLeft < MAX_FIXED_LENGTH) && (rxLoop.cc1101Mode == INFINITE))
            {
              ELECHOUSE_cc1101.SpiWriteReg(CC1101_PKTCTRL0, FIXED_PACKET_LENGTH);
              rxLoop.cc1101Mode = FIXED;
            }
            // Do not empty the Rx FIFO (See the CC1101 SWRZ020E errata note)
            uint8_t bytesInFIFO = ELECHOUSE_cc1101.SpiReadStatus(CC1101_RXBYTES) & 0x7F;
            ELECHOUSE_cc1101.SpiReadBurstReg(CC1101_RXFIFO, rxLoop.pByteIndex, bytesInFIFO - 1);

            rxLoop.bytesLeft -= (bytesInFIFO - 1);
            rxLoop.pByteIndex += (bytesInFIFO - 1);
            rxLoop.bytesRx += (bytesInFIFO - 1);
            max_wait_time_ += extra_time_;
          }
          break;
        }

        uint8_t overfl = ELECHOUSE_cc1101.SpiReadStatus(CC1101_RXBYTES) & 0x80;
        // end of packet in length mode
        if ((!overfl) && (!digitalRead(gdo2)) && (rxLoop.state > WAIT_FOR_DATA))
        {
          ESP_LOGD(TAG, "End of packet detected -> read remaining bytesLeft=%d", rxLoop.bytesLeft);
          ELECHOUSE_cc1101.SpiReadBurstReg(CC1101_RXFIFO, rxLoop.pByteIndex, (uint8_t)rxLoop.bytesLeft);
          rxLoop.bytesRx += rxLoop.bytesLeft;
          data_in.length = rxLoop.bytesRx;
          this->returnFrame.rssi = (int8_t)ELECHOUSE_cc1101.getRssi();
          this->returnFrame.lqi = (uint8_t)ELECHOUSE_cc1101.getLqi();
          ESP_LOGV(TAG, "Have %d bytes from CC1101 Rx, RSSI: %d dBm LQI: %d", rxLoop.bytesRx, this->returnFrame.rssi, this->returnFrame.lqi);
          if (rxLoop.length != data_in.length)
          {
            ESP_LOGE(TAG, "Length problem: req(%d) != rx(%d)", rxLoop.length, data_in.length);
          }
          if (this->syncMode)
          {
            ESP_LOGV(TAG, "Synchronus mode enabled.");
          }
          if (mBusDecode(data_in, this->returnFrame))
          {
            rxLoop.complete = true;
            this->returnFrame.mode = data_in.mode;
            this->returnFrame.block = data_in.block;
          }
          rxLoop.state = INIT_RX;
          return rxLoop.complete;
        }
        start(false);
      } while ((this->syncMode) && (rxLoop.state > WAIT_FOR_SYNC));
      return rxLoop.complete;
    }

    WMbusFrame RxLoop::get_frame()
    {
      return this->returnFrame;
    }

    static inline bool wait_marcstate_(uint8_t target, uint32_t timeout_ms)
    {
      uint32_t t0 = millis();
      while (ELECHOUSE_cc1101.SpiReadStatus(CC1101_MARCSTATE) != target)
      {
        if (millis() - t0 > timeout_ms)
          return false;
        delay(1); // najprostsze “karmienie” WDT w Arduino/ESPHome
      }
      return true;
    }

    bool RxLoop::start(bool force)
    {
      // waiting to long for next part of data?
      bool reinit_needed = ((millis() - sync_time_) > max_wait_time_) ? true : false;

      const uint8_t marc0 = ELECHOUSE_cc1101.SpiReadStatus(CC1101_MARCSTATE);
      ESP_LOGD(TAG, "start(force=%d) reinit_needed=%d marc0=0x%02X gdo0=%d gdo2=%d",
               force, reinit_needed, marc0,
               digitalRead(this->gdo0), digitalRead(this->gdo2));

      if (!force)
      {
        if (!reinit_needed)
        {
          // already in RX?
          if (marc0 == MARCSTATE_RX)
          {
            ESP_LOGV(TAG, "start(): already in RX, skipping reinit");
            return false;
          }
        }
      }

      // init RX here, each time we're idle
      rxLoop.state = INIT_RX;
      sync_time_ = millis();
      max_wait_time_ = extra_time_;

      ESP_LOGD(TAG, "start(): SIDLE");
      ELECHOUSE_cc1101.SpiStrobe(CC1101_SIDLE);
      if (!wait_marcstate_("SIDLE->IDLE", MARCSTATE_IDLE, 500))
      {
        // nie wchodzimy w busy-loop -> brak WDT, mamy log i wyjście
        return false;
      }

      ESP_LOGD(TAG, "start(): flush FIFOs");
      ELECHOUSE_cc1101.SpiStrobe(CC1101_SFTX); // flush Tx FIFO
      ELECHOUSE_cc1101.SpiStrobe(CC1101_SFRX); // flush Rx FIFO

      // Initialize RX info variable
      rxLoop.lengthField = 0;
      rxLoop.length = 0;
      rxLoop.bytesLeft = 0;
      rxLoop.bytesRx = 0;
      rxLoop.pByteIndex = data_in.data;
      rxLoop.complete = false;
      rxLoop.cc1101Mode = INFINITE;

      this->returnFrame.frame.clear();
      this->returnFrame.rssi = 0;
      this->returnFrame.lqi = 0;
      this->returnFrame.mode = 'X';
      this->returnFrame.block = 'X';

      std::fill(std::begin(data_in.data), std::end(data_in.data), 0);
      data_in.length = 0;
      data_in.lengthField = 0;
      data_in.mode = 'X';
      data_in.block = 'X';

      ESP_LOGD(TAG, "start(): set FIFO/PKT regs (FIFOTHR, PKTCTRL0)");
      // Set Rx FIFO threshold to 4 bytes
      ELECHOUSE_cc1101.SpiWriteReg(CC1101_FIFOTHR, RX_FIFO_START_THRESHOLD);
      // Set infinite length
      ELECHOUSE_cc1101.SpiWriteReg(CC1101_PKTCTRL0, INFINITE_PACKET_LENGTH);

      ESP_LOGD(TAG, "start(): SRX");
      ELECHOUSE_cc1101.SpiStrobe(CC1101_SRX);
      if (!wait_marcstate_("SRX->RX", MARCSTATE_RX, 500))
      {
        return false;
      }

      rxLoop.state = WAIT_FOR_SYNC;
      ESP_LOGD(TAG, "start(): RX armed, state=WAIT_FOR_SYNC");
      return true; // re-started Rx
    }

  }
}