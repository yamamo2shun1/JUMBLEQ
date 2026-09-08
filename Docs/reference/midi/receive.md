# MIDI Receive Specification

This document describes the MIDI receive specification based on the `JUMBLEQ/Appli/Core` implementation as of September 2026.

## 1. Overview

- Incoming MIDI messages are processed over USB MIDI.
- Incoming messages used to change UI settings are accepted only on **MIDI Ch. 15** (1-based channel numbering).

## 2. Program Change (Host -> Device)

### 2.1 Acceptance Conditions

- Program Change messages are accepted only on MIDI Ch. 15.
- Program Change messages on all other channels are ignored.

### 2.2 Program Change Map

| PC Number | Function |
|---:|---|
| 0/1 | Set the Ch. 1 input type (Line / Phono). |
| 2/3 | Set the Ch. 2 input type (Line / Phono). |
| 4/5/6/7 | Assign an input source (Ch. 1 / Ch. 2 / USB[1/2] / USB[3/4]) to Channel Fader A. |
| 8/9/10/11 | Assign an input source (Ch. 1 / Ch. 2 / USB[1/2] / USB[3/4]) to Channel Fader B. |
| 12/13/14/15 | Assign an input source (Ch. 1 / Ch. 2 / USB[1/2] / USB[3/4]) to Post Fader. |
| 16/17 | Disable or enable DVS mode for Ch. 1. |
| 18/19 | Disable or enable DVS mode for Ch. 2. |
| 20/21/22 | Select the Return source (USB[1/2] / USB[3/4] / None). |
| 23/24/25/26 | Select the headphone output source (FADER_A / FADER_B / THRU / MASTER). |
| 27/28 | Assign magnetic switch 2 to the auxiliary fade-down function for Channel Fader A or B. |
| 29/30 | Assign magnetic switch 3 to the auxiliary fade-down function for Channel Fader A or B. |
| 31/32 | Set the Channel Fader A direction (Normal / Reverse). |
| 33/34 | Set the Channel Fader B direction (Normal / Reverse). |
| 120/121 | Disable or enable Channel Fader curve edit mode. |
| 122/123 | Select the output mode (CC / Note) for magnetic controls and channel fader sensors. |
| 126 | Transmit a dump of the parameters currently configured on the device. |
| 127 | Save the current settings to EEPROM. |

Notes:

- `PC22` selects `None`, which disables the Return signal path.
- Auxiliary fade-down assignments for magnetic switches 2 and 3 take effect immediately when the Program Change message is received.
- The default assignments are Channel Fader A for magnetic switch 2 and Channel Fader B for magnetic switch 3.
- Direction can be configured independently for Channel Faders A and B and takes effect immediately. The default is Normal for both faders.
- Reverse inverts the final fader response for Fade Up and all Fade Down controls assigned to that fader, including auxiliary Fade Down controls. It does not change the live MIDI values transmitted by the individual magnetic sensors.
- `PC127` saves the complete device configuration listed in Section 5 to EEPROM, including the DVS enable states, Return and headphone source assignments, magnetic output mode, auxiliary fade-down assignments, Direction settings, and curves for Channel Faders A and B.
- `PC126` transmits the **current operating state (parameters eligible for saving)** rather than reading and transmitting the EEPROM contents.

## 3. Control Change (Host -> Device)

### 3.1 Acceptance Conditions

Control Change messages are accepted only when all of the following conditions are met:

- Channel Fader curve edit mode is enabled.
- The MIDI channel is Ch. 15.
- The CC number is supported (`20`, `21`, or `22`).

### 3.2 Control Change Map

| CC Number | Parameter | CC Value |
|---:|---|---|
| 20 | `XFADE_CURVE_A` | 0 to 127 |
| 21 | `XFADE_CURVE_B` | 0 to 127 |
| 22 | `DVS_FADER_DELAY_MS` | 0 to 120 ms; values 121 to 127 are clamped to 120 ms |

Curve response:

- `0` selects the strongest late-rise curve; the level changes mainly near the end of the key travel.
- `64` selects an exactly linear response across the full key travel.
- `127` selects the strongest early-rise curve; the level changes sharply near the start of the key travel.
- The late- and early-rise families use point-reflected exponential curves.
- The default value is `64` for both Channel Faders A and B.
- The DVS fader delay defaults to 50 ms and is shared by both input channels.

## 4. `curve_edit_mode` (Channel Fader Curve Edit Mode) Behavior

- Use Program Change messages to enable or disable the mode (`PC121` ON / `PC120` OFF).
- While enabled, the main OLED shows the current curve CC values for Channel Faders A and B together with a graphical preview of each curve. The first line shows `FADER DLY: nms` when either DVS channel is enabled and `FADER DLY: ---` when both are disabled.
- While disabled, the main OLED uses its normal display and shows the gain values for each input and output channel.
- There is no automatic timeout.

## 5. EEPROM Storage and Startup Restore

- Save trigger: `PC127` on Ch. 15
- Saved parameters:
  - CH1/CH2 input types
  - Channel Fader A/B and Post Fader assignments
  - CH1/CH2 DVS enable states
  - DVS magnetic-switch fader delay
  - Return source assignment
  - Headphone output source
  - Magnetic output mode (CC / Note)
  - Auxiliary fade-down assignments for magnetic switches 2 and 3
  - Direction settings (Normal / Reverse) for Channel Faders A/B
  - Curve settings for Channel Faders A/B
- EEPROM record version: `0x0008` (`0x0007` records are loaded with a 50 ms DVS fader delay)

## 6. Implementation Notes

- Channel values in the implementation are zero-based (for example, `14` = MIDI Ch. 15).
- The Ch. 15 restrictions for incoming Program Change and Control Change messages are checked in their respective dispatch functions.
- Internally, `CC64` maps to zero exponential curvature. Values below it produce negative (late-rise) curvature, and values above it produce positive (early-rise) curvature. The absolute bipolar curve amount is cubed before it scales the exponent, providing finer adjustment around the linear midpoint while preserving the endpoint curves. The maximum curvature is calibrated so the `CC127` response reaches 90% output at 2% key travel. EEPROM retains the legacy normalized-width representation for backward compatibility; that stored value is converted back to its CC value before the exponential curve is evaluated.
