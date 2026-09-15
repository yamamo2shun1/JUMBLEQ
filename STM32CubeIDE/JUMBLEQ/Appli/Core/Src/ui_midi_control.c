/*
 * ui_midi_control.c
 *
 * TinyUSB MIDI I/O boundary: packet read conversion and CC/Note/Program send.
 * Product setting semantics are handled by the caller (ui_control.c).
 */

#include "main.h"
#include "ui_midi_control_internal.h"

bool ui_midi_control_read(UiMidiEvent_t* event)
{
    if (!tud_midi_available())
    {
        return false;
    }

    uint8_t packet[4];
    tud_midi_packet_read(packet);

    event->raw_cin = packet[0];
    event->status  = (uint8_t) (packet[1] & 0xF0U);
    event->channel = (uint8_t) (packet[1] & 0x0FU);
    event->data1   = packet[2];
    event->data2   = packet[3];

    return true;
}

void ui_midi_control_send_cc(uint8_t number, uint8_t value, uint8_t channel)
{
    uint8_t control_change[3] = {0xB0 | channel, number, value};
    tud_midi_stream_write(0, control_change, 3);
}

void ui_midi_control_send_note(uint8_t note, uint8_t velocity, uint8_t channel)
{
    uint8_t note_msg[3];

    if (velocity == 0U)
    {
        note_msg[0] = 0x80U | channel;
        note_msg[1] = note;
        note_msg[2] = 0U;
    }
    else
    {
        note_msg[0] = 0x90U | channel;
        note_msg[1] = note;
        note_msg[2] = velocity;
    }

    tud_midi_stream_write(0, note_msg, 3);
}

void ui_midi_control_send_program(uint8_t program, uint8_t channel)
{
    uint8_t program_change[2] = {0xC0 | channel, program};
    tud_midi_stream_write(0, program_change, 2);
}
