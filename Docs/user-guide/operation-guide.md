# Operation Guide

This guide explains JUMBLEQ's physical controls, audio routing, USB signal routing, input operating modes, and OLED indications.

## Signal Routing Overview

Audio sources are assigned to Channel Faders A and B using [JUMBLEQ Configurator](./configurator.md). In standard operation, the Channel Fader A signal is sent to USB 1/2 and the Channel Fader B signal is sent to USB 3/4. Either USB pair can be selected as a Return source and mixed with its corresponding channel-fader signal.

See the [Signal Flow reference](../reference/signal-flow.md) for the detailed audio-routing diagram.

## Input Operating Modes and USB Signal Routing

Input Ch. 1 and Input Ch. 2 can each be set independently to Off, DVS, or SYNTH using a compatible configurator. Off selects standard mixer routing, while DVS selects the existing USB DVS send/return path. SYNTH reserves the input for internal oscillator processing; the selector, saved state, and OLED indication are provided before the oscillator audio engine itself is added.

DVS can be enabled independently for Input Ch. 1 and Input Ch. 2 using [JUMBLEQ Configurator](./configurator.md). The DVS state changes the USB Send source as follows:

| DVS state | Send source | Destination |
|---|---|---|
| Disabled | Channel Fader A (post-fader) | USB 1/2 |
| Disabled | Channel Fader B (post-fader) | USB 3/4 |
| Input Ch. 1 enabled | Input Ch. 1 (pre-fader) | USB 1/2 |
| Input Ch. 2 enabled | Input Ch. 2 (pre-fader) | USB 3/4 |

### When DVS Is Disabled

When DVS is disabled for a USB pair, its corresponding channel-fader signal is sent to the connected USB host as shown in the table above. The Return source can be set to any USB pair whose corresponding DVS setting is disabled, or to None:

| Return setting | Wet signal | Dry signal |
|---|---|---|
| USB 1/2 | USB 1/2 | Channel Fader A (post-fader) |
| USB 3/4 | USB 3/4 | Channel Fader B (post-fader) |
| None | Muted | Channel Faders A and B remain fully dry |

The Dry/Wet knob crossfades the selected Wet signal against its corresponding Dry signal. At 0% the selected path is fully dry; at 100% it is fully wet. The Return knob adjusts the level of the selected Wet signal.

When Return is set to None, Dry is fixed at 100% and Wet at 0%. The Dry/Wet and Return knobs therefore have no effect.

### When DVS Is Enabled

The USB pair corresponding to the enabled DVS channel is used for the DVS send and return path instead of the standard channel-fader Send and selectable Return path. JUMBLEQ Configurator prevents that USB pair from being selected as the Return source.

The Dry/Wet and Return knobs do not control the DVS path. If the other USB pair is not reserved by DVS and is selected as the Return source, both knobs continue to control that Return path normally.

Magnetic-switch changes to a channel fader reach the DSP with a configurable control delay when the corresponding input's DVS setting is enabled. The delay is shared by both input channels, can be set from 0 to 120 ms, and defaults to 50 ms. Input Ch. 1 and USB 1/2 assignments follow Ch. 1 DVS; Input Ch. 2 and USB 3/4 assignments follow Ch. 2 DVS. This includes PC-return audio assigned directly from USB. The delay follows the input assignment of each fader, including when A and B are swapped. When the corresponding DVS setting is disabled, the fader responds without this additional delay. Magnetic-switch MIDI output retains its existing timing.

Successive fader changes are queued in order, so a brief cut is delayed together with its release. Actual timing also includes the UI task and DSP-write scheduling. Changing the assigned input, its DVS state, or the configured delay cancels pending changes and synchronizes the DSP to the current fader target. While curve edit mode is active, the main OLED shows `FADER DLY: nms` when either DVS channel is enabled and `FADER DLY: ---` when both are disabled.

## Knob Assignments

The following diagram shows the current knob assignments.

<img width="640" alt="image" src="https://github.com/user-attachments/assets/e3424fb8-cb9b-4c27-b2f8-0ff7365313c5" />

### Knob Functions

| Panel label | Range | Function |
|---|---:|---|
| CC.1 | 0–127 | Sends MIDI CC 0 on MIDI Ch. 1. |
| CC.2 | 0–127 | Sends MIDI CC 1 on MIDI Ch. 1. |
| CC.3 | 0–127 | Sends MIDI CC 2 on MIDI Ch. 1. |
| CC.4 | 0–127 | Sends MIDI CC 3 on MIDI Ch. 1. |
| CC.5 | 0–127 | Sends MIDI CC 4 on MIDI Ch. 1. |
| Input Ch. 1 | −80 to +10 dB | Adjusts the level of the analog Channel 1 input before it is routed through the mixer or to the DVS send path. |
| Input Ch. 2 | −80 to +10 dB | Adjusts the level of the analog Channel 2 input before it is routed through the mixer or to the DVS send path. |
| Output Ch. 1 | −80 to +10 dB | Adjusts the level of the analog Channel 1 output. |
| Output Ch. 2 | −80 to +10 dB | Adjusts the level of the analog Channel 2 output. |
| Dry/Wet | 0–100% | Crossfades the selected USB Return signal against its corresponding channel-fader signal. See [USB Signal Routing](#usb-signal-routing). |
| Return | −80 to +10 dB | Adjusts the level of the USB pair selected as the Return source. See [USB Signal Routing](#usb-signal-routing). |
| HP | −80 to +10 dB | Adjusts the headphone output level. |

## Magnetic-switch Assignments

<img width="640" alt="image" src="https://github.com/user-attachments/assets/0e5bbf12-1b3f-4bd9-9b69-4d325e650d13" />

The switches labeled `fade` provide behavior similar to the channel faders on a conventional DJ mixer. Instead of moving a physical fader, key travel changes a virtual fader position according to the current [fader curve setting](#fader-curve-settings). The A fade switches control Channel Fader A, and the B fade switches control Channel Fader B.

The following diagram shows the current magnetic-switch assignments.

### Magnetic-switch Functions

| Panel label | Function |
|---|---|
| A (B) fade down | Controls Channel Fader A (B) in the normal or reverse direction depending on whether the corresponding fade-up switch is held. See [Fade-down Direction](#fade-down-direction). |
| A (B) fade up | Moves Channel Fader A (B) from its current position toward full level. The response follows the corresponding channel-fader curve setting. |
| Aux. A/B (B/A) fade down | Provides an additional fade-down control assignable to Channel Fader A or B using JUMBLEQ Configurator. It follows the same direction behavior as the assigned fader's fade-down switch. See [Auxiliary Fade-down Switches](#auxiliary-fade-down-switches). |
| MIDI CC / Note (4 switches) | Sends MIDI CC 12–15 or Notes 68–71 on MIDI Ch. 1 with a linear response to key travel. The output mode is selected using JUMBLEQ Configurator. |

### Auxiliary Fade-down Switches

<img width="480" alt="image" src="https://github.com/user-attachments/assets/d2f6f36c-b0cb-4ead-a585-a3e3bf55507e" />

The two auxiliary fade-down switches can each be assigned to Channel Fader A or B using JUMBLEQ Configurator. By assigning both auxiliary switches to the same Channel Fader, that fader can be operated with up to three fade-down switches: its main fade-down switch plus the two auxiliary switches.

This arrangement makes rapid multi-click patterns easier to perform, including scratch techniques such as 3-click flares and crab scratches. By default, the A/B auxiliary switch is assigned to Channel Fader A, and the B/A auxiliary switch is assigned to Channel Fader B.

### Fader Curve Settings

<img width="640" alt="image" src="https://github.com/user-attachments/assets/15989d82-41b1-49b2-b1c8-d45238cbbf39" />

The fader curve determines how the key travel of a fade switch is converted to the virtual fader position. Channel Faders A and B can be configured independently using JUMBLEQ Configurator.

| Setting | Response | Style |
|---|---|---|
| 0% | Late-rise response: most of the level change occurs near the end of the key travel. | Long-transition effects |
| 50% | Linear response across the full key travel. | Mixing |
| 100% | Early-rise response: the level rises sharply near the start of the key travel. | Scratching |

Intermediate settings follow a continuous exponential curve. The 0% and 100% responses are exact point reflections of one another around the 50% linear response. The corresponding curve applies to the fade-up and fade-down switches, including any auxiliary fade-down switch assigned to that Channel Fader.

### Fade-down Direction

The A and B fade-down switches change direction depending on whether the matching fade-up switch is held:

| Matching fade-up switch | Pressing fade down | Releasing fade down | Typical use |
|---|---|---|---|
| Not held | Moves the fader toward full level after soft takeover. | Returns the fader to mute after takeover. | Momentary fade-in |
| Held | Moves the fader toward mute. | Restores the level set by the held fade-up switch. | Momentary cut |

When fade up is not held, the fade-down switch uses reversed key travel: its released position represents mute, and pressing it deeper represents a higher level. Soft takeover prevents a sudden level jump by waiting until this reversed value reaches the current fader level before the switch takes control.

This behavior applies independently to Channel Faders A and B. An auxiliary fade-down switch follows the same behavior as the fader to which it is assigned.

See the [MIDI Transmit Specification](../reference/midi/transmit.md) for the MIDI output from all magnetic sensors.

## OLED Indications

The main OLED shows the current sample rate, input and output gains, Return status, and headphone level. The Return indication changes with the selected source:

| Return setting | OLED source indication |
|---|---|
| USB 1/2 | `RTN\|U12` |
| USB 3/4 | `RTN\|U34` |
| None | `RTN\|OFF` |

When USB 1/2 or USB 3/4 is selected, the Return gain is shown after the source indication. When Return is set to None, the gain field shows `---`.

The sub OLED shows the sources assigned to Channel Faders A and B, their input types and operating modes, the post-fader source, and the selected headphone-monitor source. `[D]` indicates DVS, `[S]` indicates SYNTH, and `[ ]` indicates Off for the assigned input channel.
