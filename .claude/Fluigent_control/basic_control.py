import Fluigent.SDK as fgt
import time

def main():
    fgt.fgt_init()
    try:
        fgt.fgt_get_pressureChannelsInfo()   # verify range

        p_idx = 0
        max_p = 200                          # mbar — set to YOUR channel's max
        setpoint = min(100, max_p)

        fgt.fgt_set_pressure(p_idx, setpoint)
        time.sleep(2)
        print(f"Measured: {fgt.fgt_get_pressure(p_idx):.1f} mbar")

        fgt.fgt_set_pressure(p_idx, 0)       # vent before closing
    finally:
        fgt.fgt_close()

if __name__ == "__main__":
    main()