# MIDI Transmit Specification

This document describes the MIDI transmit specification based on the `JUMBLEQ/Appli/Core` implementation as of September 2026.

## 1. Overview

- Outgoing MIDI messages are sent to the host over USB MIDI.
- Live control output is sent on MIDI Ch. 1.
- Configuration dump responses are sent on MIDI Ch. 15.

## 2. Live Control Output (Device -> Host)

The device transmits the following messages on MIDI Ch. 1. In the implementation, channel value `0` corresponds to MIDI Ch. 1.

| Control | CC Mode | Note Mode | Description |
|---|---|---|---|
| MIDI-assigned potentiometers | CC `0..4` | — | Sends the current knob value. |
| Magnetic controls 0..3 | CC `12..15` | Notes `68..71` | Output mode is selected using `PC122` or `PC123`. |
| Magnetic channel fader sensors 0..5 | CC `20..25` | Notes `60..65` | Output mode is selected using `PC122` or `PC123`. |

In CC mode, magnetic channel fader updates are sent only when the normalized raw value changes by more than `0.01`. This reduces MIDI traffic and jitter.

In Note mode, magnetic controls and channel fader sensors send Note On/Off messages. The peak sensor value measured during the initial 12 ms window is converted to Note On velocity.

CC20 through CC22 have different roles depending on the MIDI channel: on Ch. 1 they carry live values from magnetic channel fader sensors 0 through 2. In a Ch. 15 configuration dump, CC20 and CC21 carry curve settings for Channel Faders A and B, while CC22 carries the DVS fader delay.

## 3. Implementation Notes

- Channel values in the implementation are zero-based (for example, `0` = MIDI Ch. 1).
- `PC122` selects CC output for magnetic controls and channel fader sensors.
- `PC123` selects Note output for magnetic controls and channel fader sensors.

## 4. Configuration Dump Response (When PC126 Is Received)

When the device receives `PC126` on Ch. 15 from the host, it sends the current values of all parameters eligible for saving back to the host.

- Transmit channel: **MIDI Ch. 15** (implementation value `14`)
- Message types: Program Change and Control Change

### 4.1 Program Change (Device -> Host, Ch. 15)

| PC Value | Meaning |
|---:|---|
| 0/1 | Ch. 1 input type (Line / Phono) |
| 2/3 | Ch. 2 input type (Line / Phono) |
| 4/5/6/7 | Channel Fader A source (Ch. 1 / Ch. 2 / USB[1/2] / USB[3/4]) |
| 8/9/10/11 | Channel Fader B source (Ch. 1 / Ch. 2 / USB[1/2] / USB[3/4]) |
| 12/13/14/15 | Post Fader source (Ch. 1 / Ch. 2 / USB[1/2] / USB[3/4]) |
| 16/17/18 | Ch. 1 input mode (Off / DVS / SYNTH) |
| 19/20/21 | Ch. 2 input mode (Off / DVS / SYNTH) |
| 22/23/24 | Return source (USB[1/2] / USB[3/4] / None) |
| 25/26/27/28 | Headphone output source (FADER_A / FADER_B / THRU / MASTER) |
| 29/30 | Auxiliary fade-down assignment for magnetic switch 2 (Channel Fader A/B) |
| 31/32 | Auxiliary fade-down assignment for magnetic switch 3 (Channel Fader A/B) |
| 33/34 | Channel Fader A direction (Normal / Reverse) |
| 35/36 | Channel Fader B direction (Normal / Reverse) |
| 122/123 | Output mode for magnetic controls and channel fader sensors (CC / Note) |

The dump contains exactly one value from each Direction pair: `PC33` (Normal) or `PC34` (Reverse) for Channel Fader A, followed by `PC35` (Normal) or `PC36` (Reverse) for Channel Fader B.

### 4.2 Control Change (Device -> Host, Ch. 15)

| CC Number | Meaning |
|---:|---|
| 20 | `XFADE_CURVE_A` (`0` = strongest late rise, `64` = linear, `127` = strongest early rise) |
| 21 | `XFADE_CURVE_B` (`0` = strongest late rise, `64` = linear, `127` = strongest early rise) |
| 22 | `DVS_FADER_DELAY_MS` (`0..120` ms) |

The curve values use the same scale as incoming curve-edit Control Change messages. The DVS fader delay value is transmitted directly in milliseconds. This allows the host to restore the current settings without conversion.
