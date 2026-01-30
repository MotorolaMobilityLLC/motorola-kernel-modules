# Zinitix: single multiplexed sysfs node `threshold` (0x0020..0x0024)

This patch adds one read/write sysfs attribute `threshold` to the Zinitix
BT54x touchscreen driver. It allows
userspace to read and write the threshold-related registers in the range
`0x0020`..`0x0024`.

## Interface

NOTE: Commands should be run as a root user.
- **Read** a register (two-step):
  1. Select the address to read:
     ```bash
     echo 0x0023 > /sys/class/tsp/tsp/threshold
     ```
  2. Read value (decimal):
     ```bash
     cat /sys/class/tsp/tsp/threshold
      ```
example:
adb root
adb shell
huron:/ # echo "0x0020" > /sys/class/tsp/tsp/threshold
huron:/ # cat /sys/class/tsp/tsp/threshold
90

- **Write** a register (one-step):
  ```bash
  # in hexadecimal. 0x0023 is addr, 0x64 is value (=100)
  echo "0x0023 64" > threshold
  ```
ex:
cd /sys/class/tsp/tsp
huron:/sys/class/tsp/tsp # echo "0x0023 64" > threshold
huron:/ # cat /sys/class/tsp/tsp/threshold
100


Addresses outside `0x0020..0x0024` yield `-EINVAL`. Values > `0xFFFF` yield
`-ERANGE`. If the device is powered off, I²C ops may fail (`-EIO`).
