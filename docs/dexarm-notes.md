# DexArm technical notes

Notes collected while building this app against a DexArm (hardware V3.2, firmware V2.2.2 and later V2.3.5).
The firmware details come from reading [Marlin_For_DexArm](https://github.com/Rotrics-Dev/Marlin_For_DexArm)
and from testing on the arm.

## Hardware and connection

| | |
|---|---|
| Controller | STM32F407ZG |
| USB | CDC serial, VID:PID `0483:5740`, usually `/dev/ttyACM0`, 115200 baud |
| Permissions | the user needs to be in the `dialout` group |
| Firmware | DexArm releases are numbered separately from their Marlin base (2.0.5.4): `M2010` gives the DexArm version, `M115` only the Marlin one |

## Useful G-codes

| Code | Effect |
|---|---|
| `M1112` | home (goes to X0 Y300 Z0 and corrects from the encoders) |
| `M114` | commanded position plus joint thetas (V2.3.5+ also prints `Real position`, from the encoders) |
| `M895` | encoder position (XYZ) |
| `M890` | raw encoder counts |
| `M889 X Y Z` | set the encoder calibration (saved to EEPROM) |
| `M17` / `M84` | motors on / off. Both resync the firmware position from the encoders |
| `M410` | quick stop. Handled at once by the emergency parser, but still replies "ok" later |
| `M204 P T` / `M205 X Y Z` | acceleration / jerk |
| `M500` | save settings to EEPROM |
| `M2010` / `M2011` | firmware / hardware version |
| `M2007` | reboot the controller (USB drops for about 1 s) |
| `M503` | dump settings |

## Calibration

Each joint has a magnetic encoder; `M889` stores the encoder value that counts as "zero" for each joint
(X = base, Y = shoulder, Z = elbow). For the base, 1024 counts = 90°.

Example: on one arm the base zero was exactly 90° off after a repair, so the base drove into its end stop
when homing. Setting `M889 X<value − 1024> …` (keeping Y and Z) fixed it without opening the arm.
The physical base stops are at about ±113°; the firmware only allows ±90° (Y ≥ 0).

## Kinematics

- Upper arm and forearm: 150 mm each. Horizontal offset = module offset (`FRONT_MODULE_OFFSET`, from `M503`) + 12.5 mm.
- Joint angles used in this app:
  - base (yaw): 0 = straight forward, + = clockwise seen from above (toward +X)
  - shoulder: upper arm elevation above horizontal
  - elbow: forearm angle below horizontal
- Firmware thetas: `theta_a = yaw`, `theta_b = 105.24 − shoulder`, `theta_c = −9.60 − elbow`.
- Reach: the firmware checks every segment against a table (Z −127…167 mm, min/max radius per 2 mm of Z;
  copied to `dexarm_web/armlimit.json`) and silently cuts moves short ("XY beyond limit").
- URDF joint mapping (URDF-of-DexArm): `bases = −yaw`, `B_Rot->In_1 = 85.73° − shoulder`, `Join->In_2 = elbow − 22.80°`.

## Motion behavior

- Each `G1` is split into segments at 200 per second (minimum 0.5 mm).
- Planner buffer: 16 blocks (about 80 ms of motion). Command buffer: 4.
- The "ok" for a `G1` arrives once it is buffered, not when the move finishes.
- Shipped acceleration is 60 mm/s² with 10 mm/s jerk and S-curve acceleration on. The firmware allows up to 3000 mm/s² and 300 mm/s.
- The planner always plans to stop at the end of what it has been sent, so how far ahead moves are queued limits top speed.

## Known firmware bugs in V2.2.2 (fixed by V2.3.5)

Found by comparing versions of the firmware source ([Marlin_For_DexArm](https://github.com/Rotrics-Dev/Marlin_For_DexArm));
the fix on the arm was the official V2.3.5 image, which includes both upstream commits.


1. **`M895` corrupts moves in progress.** `get_current_position_from_position_sensor()` calls
   `planner.set_machine_position_mm()`, overwriting the planner position with the encoder reading. Polling `M895`
   during a move therefore resets the planner mid-motion: stutter and position jumps. Fixed upstream in f9e0bbf86 (2022-01-07).
2. **`M410` can wedge the serial port.** The emergency parser ran `quickstop_stepper()` (including `planner.synchronize()`)
   inside the serial receive path. Symptom: the USB port stays present but writes time out until a power cycle.
   Fixed in a46957302 (2022-01-08).

Also: the watchdog is disabled (`USE_WATCHDOG` is commented out), so a firmware hang never recovers on its own.
V2.3.5 refuses motion after `M410` until the arm is homed ("Send M1112 or click HOME…").

## Firmware update procedure

Official firmware images are on the [Rotrics downloads page](https://rotrics.com/pages/downloads). The
*DexArm Firmware V2.3.5* zip (Feb 2022) contains `F2.3.5_HW3.1.bin` and `F2.3.5_HW3.2..bin`; use the one matching
`M2011`. The procedure below comes from Rotrics Studio (`server/src/firmwareUpgradeManager.js`) and is implemented
in `tools/flash_dexarm.py`:

1. Send `M2002`, then `M2003`. The arm answers "Reset to enter update bootloader" and reboots into the bootloader
   (USB product string "STM32 Virtual ComPort").
2. Bootloader menu at 115200 baud: `1` download image, `2` upload (back up) image, `3` run the application,
   `4` enable write protection, `5` product info (`a5` also prints the hardware version).
3. Images are sent with YMODEM: 128-byte SOH frames with CRC16, then EOT and an empty header frame.
   The bootloader answers `C`, `C`, then ACK (0x06) per frame or NAK (0x15) to resend.
4. After "Programming Completed Successfully!", send `3` to start the new firmware.

Notes:
- The application is linked at 0x08010000, after the bootloader at 0x08000000. The bootloader is separate, so a failed
  transfer can be retried from bootloader mode (the board's buttons are the fallback way in).
- Upload (`2`) returns 983040 bytes (0x08010000–0x080FFFFF), which includes the settings sector: a full backup.
- Settings (EEPROM emulation, last flash sector, layout "V77") survived the V2.2.2 → V2.3.5 update.
- Rotrics' update API (`http://api.rotrics.com/version/firmware/version?version=<fw>&hardwareVersion=<hw>`) returns
  the official image URL for a given hardware version.
