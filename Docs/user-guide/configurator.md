# JUMBLEQ Configurator

JUMBLEQ Configurator changes device settings by sending MIDI messages to JUMBLEQ.

## Web App (Recommended)

The browser-based [JUMBLEQ Configurator](https://configurator.jumbleq.io/) is the primary version and receives maintenance priority. It runs without installing a dedicated application; use a browser with Web MIDI support and connect JUMBLEQ to the computer over USB.

[Open JUMBLEQ Configurator](https://configurator.jumbleq.io/)

### Getting Started

1. Connect JUMBLEQ to the computer over USB and power it on.
2. Open the Web app and allow MIDI access when prompted by the browser.
3. Confirm that the connection status shows **JUMBLEQ connected** and that the device settings are synchronized.

The Web app provides controls for input and channel-fader routing, channel-fader response curves, DVS, monitor and Return routing, magnetic-switch behavior, and device settings. It also supports reading the current settings from JUMBLEQ and importing or exporting presets.

### Using the Web App on iPad

Safari does not provide Web MIDI. To connect JUMBLEQ from an iPad:

1. Install [MIDIWeb Browser](https://apps.apple.com/jp/app/midiweb-browser/id6757226617?l=en-US) from the App Store.
2. Connect JUMBLEQ to the iPad over USB and power it on.
3. Open [configurator.jumbleq.io](https://configurator.jumbleq.io/) inside MIDIWeb Browser.
4. Select **Connect device**, allow MIDI access, and wait for the settings to synchronize.

Connection and MIDI communication have been verified with MIDIWeb Browser on iPadOS 26.5.

<img width="480" alt="JUMBLEQ Configurator Web app showing routing and channel-fader settings" src="https://github.com/user-attachments/assets/77791d7c-2981-479a-be9e-e17c0968e9c8" />
<img width="480"" alt="UMBLEQ Configurator Web app showing controls and device settings" src="https://github.com/user-attachments/assets/36ef499a-27e5-489c-96c2-58ebba73b6ec" />

## Alternative Max Versions

The Max Standalone and Max for Live versions remain in the repository for existing workflows, but they have a lower maintenance priority than the Web app. Their source files and packaged versions are available in the [`Max` directory](../../Max/). The source patches expose Off, DVS, and SYNTH input-mode choices for both input channels.

### Max Standalone

The Max source patch can be opened with [Max 9](https://cycling74.com/products/max-9). Packaged archives for Windows and macOS are also retained in the `Max` directory.

<img width="400" alt="JUMBLEQ Configurator for Max" src="https://github.com/user-attachments/assets/d568f29f-4c74-4a79-a0e7-26b0f5e07027" />

### Max for Live

The [Max for Live device](../../Max/JUMBLEQ_Configurator.amxd) allows JUMBLEQ settings to be changed directly from Ableton Live.

<img width="480" alt="JUMBLEQ Configurator for Max for Live" src="https://github.com/user-attachments/assets/4fddf9b7-5925-4448-b21a-bae903f854b2" />

For the MIDI messages used by the configurator, see the [MIDI reference](../reference/midi/README.md).
