/*
 * Muca_Raw_original.ino — UNMODIFIED reference firmware
 * =====================================================
 * Verbatim copy of the upstream Muca library example:
 *     https://github.com/muca-board/Muca/blob/master/examples/Muca_Raw/Muca_Raw.ino
 *
 * This is the sketch the star-nose muca board has been running so far, and
 * therefore the sketch that produced every dataset in
 * ../../logs/ and in mucaboard_data/ and mucaboard_data_raw/.
 *
 * It is kept here UNTOUCHED, as the control condition for the gain sweep:
 * flash this to reproduce the historical scale exactly.
 *
 * Note the commented-out `muca.setGain(100);` line below — because it is
 * commented out, the FT5316's analog gain (factory-mode register 0x07) is
 * left at its power-on default. That default is why the reported scale has
 * never changed between sessions.
 *
 * Everything below this comment block is upstream code, unmodified.
 * ---------------------------------------------------------------------------
 */

#include <Muca.h>

Muca muca;

void setup() {
  Serial.begin(115200);

  muca.init(false);
  muca.useRawData(true); // If you use the raw data, the interrupt is not working
 // muca.setGain(100);
}

void loop() {
  GetRaw();
}

void GetRaw() {

  if (muca.updated()) {
    for (int i = 0; i < NUM_TX * NUM_RX; i++) {
      Serial.print(muca.grid[i]);
      if (i != NUM_TX * NUM_RX - 1)
        Serial.print(",");
    }
    Serial.println();
  }
}
