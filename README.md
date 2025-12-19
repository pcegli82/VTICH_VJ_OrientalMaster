# VJ_OrientalMaster

Arduino/PlatformIO library to control up to **10 Oriental Motor AZD-C(D)** (AZ Series) controllers via **Modbus RTU over RS-485**.

## What it supports

- Direct Data Operation (write 0x0058..0x0067, trigger=1)
- Driver input command (reference) 0x007C/0x007D
- Driver input command (automatic OFF pulse) 0x0078/0x0079
- Driver output status 0x007E/0x007F (READY/ALARM/BUSY/MOVE/INPOS)
- Feedback position (32-bit) 0x0120/0x0121
- Command position (32-bit) 0x0122/0x0123

## Library concept

- You register up to 10 motors by their Modbus **Slave ID**.
- Per motor you can configure ratios via **MPA(...)**.
- You send motion parameters via **SMP(...)**.
- You can set inputs via **SIN(...)** or pulse inputs via **SIP(...)**.
- You can read outputs via **GOU(...)**.
- You can read scaled feedback/command position via **GFP(...) / GCP(...)**.
- The library can poll output status and notify your application via a callback whenever **READY/ALARM/MOVE/INPOS** changes.

## PlatformIO

Add the library as a dependency (when hosted on GitHub):

```ini
lib_deps =
  https://github.com/<YOUR_ORG_OR_USER>/VJ_OrientalMaster.git
  4-20ma/ModbusMaster@^2.0.1
```

See `examples/platformio_basic` for a full working PlatformIO project.

## Notes

- This library assumes a **single RS-485 bus** shared by multiple slave IDs.
- Call `oriental.update()` regularly in `loop()` to drive polling + callbacks.
