/*
 * ui_midi_control_internal.h
 *
 * Private API of the TinyUSB MIDI I/O boundary. The module converts packets to
 * events / sends primitives; product-state dispatch stays in the facade.
 */

#ifndef UI_MIDI_CONTROL_INTERNAL_H_
#define UI_MIDI_CONTROL_INTERNAL_H_

#include <stdbool.h>
#include <stdint.h>

// Note edge scan constants shared by the pot-mag and ch_fader note outputs.
#define MIDI_NOTE_ON_THRESHOLD      4U
#define MIDI_NOTE_OFF_THRESHOLD     2U
#define MIDI_NOTE_VEL_WINDOW_MS     12U
#define MIDI_NOTE_VEL_GAMMA         0.65f

typedef struct
{
    uint8_t raw_cin;  // packet[0] for the RX log
    uint8_t status;
    uint8_t channel;
    uint8_t data1;
    uint8_t data2;
} UiMidiEvent_t;

// TinyUSBの受信packetを1件読み出す。packetが無ければfalse。
bool ui_midi_control_read(UiMidiEvent_t* event);

void ui_midi_control_send_cc(uint8_t number, uint8_t value, uint8_t channel);
void ui_midi_control_send_note(uint8_t note, uint8_t velocity, uint8_t channel);
void ui_midi_control_send_program(uint8_t program, uint8_t channel);

#endif /* UI_MIDI_CONTROL_INTERNAL_H_ */
