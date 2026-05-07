import usb.core
import usb.util
import time

class USB6009:
    VENDOR_ID  = 0x3923
    PRODUCT_ID = 0x717b

    def __init__(self):
        self.dev = usb.core.find(idVendor=self.VENDOR_ID, idProduct=self.PRODUCT_ID)
        if self.dev is None:
            raise ValueError("USB-6009 not found")

        # Detach kernel driver if active
        if self.dev.is_kernel_driver_active(0):
            self.dev.detach_kernel_driver(0)

        # Do NOT call set_configuration() - device is already configured
        # Just claim the interface
        usb.util.claim_interface(self.dev, 0)
        print("USB-6009 connected successfully")

    def read_analog(self, channel=0):
        """Read analog input, return raw response for debugging."""
        cmd = bytes([0x13, channel, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00] + [0x00] * 56)

        # Try both endpoint pairs
        for ep_out, ep_in in [(0x01, 0x81), (0x02, 0x82)]:
            try:
                self.dev.write(ep_out, cmd, timeout=2000)
                time.sleep(0.05)
                resp = bytes(self.dev.read(ep_in, 64, timeout=2000))
                print(f"EP{ep_out} response: {resp[:16].hex()}")
                return resp
            except usb.core.USBError as e:
                print(f"EP 0x{ep_out:02x} error: {e}")

    def close(self):
        usb.util.release_interface(self.dev, 0)
        usb.util.dispose_resources(self.dev)
        print("Closed")


if __name__ == "__main__":
    daq = USB6009()
    try:
        for i in range(3):
            daq.read_analog(channel=0)
            time.sleep(0.5)
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        daq.close()