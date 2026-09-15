/*
 * audio_usb_control_internal.h
 *
 * Private API of the UAC2 control plane module. The LED blink intervals are
 * owned there; the transport reports the applied stream state so the blink
 * behavior stays identical to the previous audio_control.c implementation.
 */

#ifndef AUDIO_USB_CONTROL_INTERNAL_H_
#define AUDIO_USB_CONTROL_INTERNAL_H_

#include "main.h"

void audio_usb_control_set_tx_stream_blink(bool streaming);
void audio_usb_control_set_rx_stream_blink(bool streaming);

#endif /* AUDIO_USB_CONTROL_INTERNAL_H_ */
