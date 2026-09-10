/*
 * Muca_Raw_gain.ino — raw streaming + LIVE analog-gain control
 * ============================================================
 * Star-Nose Sensor | capacitance_combination_calibration/gain_calibration_arduino
 *
 * WHAT THIS CHANGES vs. ../Muca_Raw_original/Muca_Raw_original.ino
 * ----------------------------------------------------------------
 * The original sketch streams the 252 raw cells and nothing else, leaving the
 * FT5316's analog gain at its power-on default (the `muca.setGain(100);` line
 * is commented out upstream). That is why every session so far has come back
 * on the same scale, and why a 1-2.5 pF capacitor pins the loaded cell at
 * ~64,200-65,200 counts against the 16-bit ceiling of 65,535.
 *
 * This sketch keeps the data format BYTE-FOR-BYTE identical — still 252
 * comma-separated integers, one line per frame, 115200 baud — so every
 * existing reader (sensor_raw.py, sensor.py, the visualizers) works unchanged.
 * It only ADDS a small serial command channel so the host can change the gain
 * between measurements instead of re-flashing 31 times.
 *
 * THE REGISTER BEHIND IT
 * ----------------------
 * The FT5x16 preliminary datasheet (../../Technical_Datasheet/) documents the
 * analog envelope only — 12-bit ADC, 21 TX x 12 RX = 252 cells, max 60 pF per
 * channel, "optimal sensing mutual capacitor: 1 pF - 4 pF" — but contains NO
 * register map. The gain knob lives in the FT5x06/FT5x16 factory-mode register
 * set that the Muca library drives:
 *
 *     0x00 = 0x40 (MODE_TEST)  enter factory mode          [required first]
 *     0x07 = <gain>            AFE analog gain             <- muca.setGain()
 *     0x88 = <rate>            scan/report rate (3..14)    <- muca.setReportRate()
 *     0x00 = 0xC0              raw-data streaming          <- muca.useRawData()
 *
 * Muca::setGain() enters MODE_TEST, writes 0x07, and stays in MODE_TEST while
 * raw mode is on — which is exactly what we want. Register 0x07 is volatile,
 * so the gain must be re-applied after every reset; that is what STARTUP_GAIN
 * and the `G` command do.
 *
 * The library passes the value straight through as a byte with no clamping
 * (the upstream example's commented-out call uses 100). The USEFUL range is
 * not documented anywhere authoritative — that is the whole point of the
 * sweep. Do not assume; measure. GAIN_MIN/GAIN_MAX below are guard rails on
 * what this sketch will accept, not a claim about the silicon.
 *
 * SERIAL COMMAND PROTOCOL  (host -> board, newline terminated)
 * -----------------------------------------------------------
 *   G<n>   set analog gain to n            e.g.  G12
 *   G?     report the gain currently set
 *   R<n>   set report rate, 3..14          e.g.  R7
 *   P      pause streaming   (quiet line, for settling / register reads)
 *   S      start streaming   (resume)
 *   I      print firmware + configuration info
 *   A      run the chip's auto-calibration  (see WARNING below)
 *   H      print this help
 *
 * Every reply this sketch prints starts with '#', and the library's own
 * chatter starts with '['. Neither ever starts with a digit, so a parser that
 * accepts only all-digit-leading lines as data (exactly what sensor_raw.py
 * already does) is unaffected. muca_gain_link.py reads the '#' lines as
 * command acknowledgements.
 *
 * WARNING — AUTO-CALIBRATION
 * --------------------------
 * The datasheet advertises "auto-calibration: insensitive to capacitance and
 * environmental variations". That is a feature for a phone screen and a
 * hazard for absolute capacitance measurement: it silently re-baselines the
 * cells, so the same physical capacitor can read differently minutes apart.
 * The `A` command is provided for deliberate experiments only. Do NOT run it
 * in the middle of a sweep — it invalidates the comparison between gains.
 *
 * Author: added for the gain-calibration study, Sept 2026.
 * Original raw-streaming logic from muca-board/Muca (examples/Muca_Raw).
 */

#include <Muca.h>

Muca muca;

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

// Gain applied automatically at boot.
//   0  = do NOT touch register 0x07 -> identical to the original firmware.
//   >0 = write this gain at startup.
// Leave at 0 so that flashing this sketch alone changes nothing measurable;
// the sweep sets the gain explicitly over serial for every step.
#define STARTUP_GAIN   0

#define GAIN_REG       0x07     // FT5x16 factory-mode analog gain
#define GAIN_MIN       1        // guard rail for this sketch, not a silicon spec
#define GAIN_MAX       255      // register 0x07 is one byte
#define GAIN_SETTLE_MS 250      // let the AFE settle before frames are trusted

#define RATE_MIN       3
#define RATE_MAX       14

#define CMD_BUF_LEN    16

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------

int  currentGain = -1;          // -1 = never written -> power-on default
int  currentRate = -1;          // -1 = never written -> power-on default
bool streaming   = true;

char cmdBuf[CMD_BUF_LEN];
byte cmdLen = 0;

// ---------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);

  muca.init(false);
  muca.useRawData(true);        // with raw data the interrupt is not used

#if STARTUP_GAIN > 0
  applyGain(STARTUP_GAIN);
#endif

  printInfoLine();
  Serial.println(F("# READY"));
}

void loop() {
  handleSerial();
  if (streaming) {
    GetRaw();
  }
}

// ---------------------------------------------------------------------------
// Data streaming — format identical to the original sketch
// ---------------------------------------------------------------------------

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

// ---------------------------------------------------------------------------
// Gain / rate
// ---------------------------------------------------------------------------

void applyGain(int gain) {
  if (gain < GAIN_MIN || gain > GAIN_MAX) {
    Serial.print(F("# ERR GAIN out-of-range "));
    Serial.println(gain);
    return;
  }

  bool wasStreaming = streaming;
  streaming = false;            // keep the line quiet while the register moves

  muca.setGain(gain);           // enters MODE_TEST, writes 0x07, stays in TEST
  delay(GAIN_SETTLE_MS);
  currentGain = gain;

  // Read the register back so the host logs what the chip actually holds,
  // not merely what we asked for.
  int readback = muca.getRegister(GAIN_REG);

  Serial.print(F("# GAIN "));
  Serial.print(currentGain);
  Serial.print(F(" READBACK "));
  Serial.println(readback);

  streaming = wasStreaming;
}

void applyRate(int rate) {
  if (rate < RATE_MIN || rate > RATE_MAX) {
    Serial.print(F("# ERR RATE out-of-range "));
    Serial.println(rate);
    return;
  }

  bool wasStreaming = streaming;
  streaming = false;

  muca.setReportRate((unsigned short) rate);
  delay(GAIN_SETTLE_MS);
  currentRate = rate;

  Serial.print(F("# RATE "));
  Serial.println(currentRate);

  streaming = wasStreaming;
}

// ---------------------------------------------------------------------------
// Reporting
// ---------------------------------------------------------------------------

void printInfoLine() {
  Serial.print(F("# INFO sketch=Muca_Raw_gain cells="));
  Serial.print(NUM_TX * NUM_RX);
  Serial.print(F(" tx="));
  Serial.print(NUM_TX);
  Serial.print(F(" rx="));
  Serial.print(NUM_RX);
  Serial.print(F(" fw="));
  Serial.print(muca.getFWVersion());
  Serial.print(F(" gain="));
  if (currentGain < 0) Serial.print(F("default")); else Serial.print(currentGain);
  Serial.print(F(" rate="));
  if (currentRate < 0) Serial.print(F("default")); else Serial.print(currentRate);
  Serial.println();
}

void printGainLine() {
  Serial.print(F("# GAIN "));
  if (currentGain < 0) Serial.print(F("default")); else Serial.print(currentGain);
  Serial.print(F(" READBACK "));
  Serial.println(muca.getRegister(GAIN_REG));
}

void printHelp() {
  Serial.println(F("# HELP G<n> set gain | G? report gain | R<n> rate 3-14"));
  Serial.println(F("# HELP P pause | S stream | I info | A autocal | H help"));
}

// ---------------------------------------------------------------------------
// Serial command parsing
// ---------------------------------------------------------------------------

void handleSerial() {
  while (Serial.available() > 0) {
    char c = (char) Serial.read();

    if (c == '\n' || c == '\r') {
      if (cmdLen > 0) {
        cmdBuf[cmdLen] = '\0';
        executeCommand(cmdBuf);
        cmdLen = 0;
      }
    } else if (cmdLen < CMD_BUF_LEN - 1) {
      cmdBuf[cmdLen++] = c;
    } else {
      cmdLen = 0;               // overlong garbage -> drop it
      Serial.println(F("# ERR command too long"));
    }
  }
}

void executeCommand(char *cmd) {
  char op = cmd[0];
  if (op >= 'a' && op <= 'z') op -= 32;      // accept lower case

  switch (op) {

    case 'G':
      if (cmd[1] == '?' || cmd[1] == '\0') printGainLine();
      else                                 applyGain(atoi(cmd + 1));
      break;

    case 'R':
      if (cmd[1] == '?' || cmd[1] == '\0') {
        Serial.print(F("# RATE "));
        if (currentRate < 0) Serial.println(F("default")); else Serial.println(currentRate);
      } else {
        applyRate(atoi(cmd + 1));
      }
      break;

    case 'P':
      streaming = false;
      Serial.println(F("# PAUSED"));
      break;

    case 'S':
      streaming = true;
      Serial.println(F("# STREAMING"));
      break;

    case 'I':
      printInfoLine();
      break;

    case 'A':
      // Deliberate, destructive to a sweep in progress — see the warning above.
      streaming = false;
      Serial.println(F("# AUTOCAL start"));
      muca.autocal();
      Serial.println(F("# AUTOCAL done"));
      streaming = true;
      break;

    case 'H':
      printHelp();
      break;

    default:
      Serial.print(F("# ERR unknown command "));
      Serial.println(cmd);
      break;
  }
}
