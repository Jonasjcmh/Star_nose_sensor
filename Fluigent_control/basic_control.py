import sys
sys.path.insert(0, "/Users/jonathantirado/Documents/GitHub/fgt-SDK/Python")

import Fluigent.SDK as fgt
import time

P_IDX     = 0        # pressure channel index
PRESSURE  = 800      # mbar — the "on" level
ON_TIME   = 0.05      # seconds at PRESSURE
OFF_TIME  = 3.0      # seconds at 0
N_CYCLES  = None     # set an integer for a fixed count, or None for infinite

fgt.fgt_init()
try:
    fgt.fgt_get_pressureChannelsInfo()   # confirm channel range once

    cycle = 0
    while N_CYCLES is None or cycle < N_CYCLES:
        fgt.fgt_set_pressure(P_IDX, PRESSURE)
        print(f"[{cycle}] ON  -> {PRESSURE} mbar | measured {fgt.fgt_get_pressure(P_IDX):.1f}")
        time.sleep(ON_TIME)

        fgt.fgt_set_pressure(P_IDX, 0)
        print(f"[{cycle}] OFF -> 0 mbar | measured {fgt.fgt_get_pressure(P_IDX):.1f}")
        time.sleep(OFF_TIME)

        cycle += 1

except KeyboardInterrupt:
    print("\nStopped by user.")
finally:
    fgt.fgt_set_pressure(P_IDX, 0)   # always vent
    fgt.fgt_close()
    print("Vented and closed.")