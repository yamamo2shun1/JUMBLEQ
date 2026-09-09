# Configurator: Safe UF2 Bootloader Transition Implementation Guide

## Objective

Add a safe way to prepare JUMBLEQ for UF2 bootloader mode from the Web Configurator without allowing a single MIDI message or an accidental click to restart the device during a performance.

The Configurator must only arm a short confirmation window. The firmware must not restart until the user confirms the request locally by holding the physical SW3 button. The corresponding application-firmware support is available in JUMBLEQ firmware v0.14.4.

This guide is based on JUMBLEQ Configurator version `0.9.6` at commit `4effe739693c23542e1bcf9992d8802ee78d48fc`.

## Safety Model

Entering UF2 mode must require all three of the following actions:

1. JUMBLEQ is connected and synchronized in the Configurator.
2. The user opens a confirmation dialog and selects **Arm UF2 mode**.
3. Within 10 seconds, the user releases SW3 once and then holds SW3 on the physical JUMBLEQ unit for 2 continuous seconds.

The Configurator action alone must never reboot the device. It only starts the firmware's temporary physical-confirmation window.

The release-before-hold requirement is enforced by the firmware, but the Configurator must explain it. It prevents a stuck or already-held switch from confirming a newly received arm request.

## MIDI Protocol

Reserve the currently unused Program Change values `124` and `125` on MIDI Ch. 15.

| Direction | Message | Function |
|---|---|---|
| Host to device | PC124 Ch. 15 | Arm the physical UF2 confirmation window. |
| Host to device | PC125 Ch. 15 | Cancel the physical UF2 confirmation window. |

The raw message bytes are:

```text
Arm UF2 confirmation
Status:  0xCE   (Program Change, Ch. 15)
Program: 124

Cancel UF2 confirmation
Status:  0xCE   (Program Change, Ch. 15)
Program: 125
```

These are transient control commands:

- Do not add them to `JumbleqConfig`, `SYNC_FIELDS`, configuration dumps, EEPROM saves, preset import, or preset export.
- Do not pass them through `decodeConfigMessage()` as setting values.
- Do not reuse PC126 or PC127. They remain the current-configuration request and EEPROM-save commands.
- Older firmware ignores PC124 and PC125, so the Configurator can be deployed before the firmware implementation without altering the device state.

Recommended exports in `app/midi/jumbleq-midi.ts`:

```ts
export const ARM_UF2_BOOTLOADER = programChange(124);
export const CANCEL_UF2_BOOTLOADER = programChange(125);
```

## User Experience

### Entry Point

Add an **Enter UF2 mode** button to the existing **Device** section, near **Read from device** and the preset actions.

The button must:

- Be visible as a maintenance action rather than a normal audio setting.
- Use warning styling distinct from **Save to device**.
- Be disabled unless the MIDI status is `ready` and the JUMBLEQ input and output ports are open.
- Open a confirmation dialog without sending MIDI on the first click.

Do not place the action in the persistent save bar or among frequently used audio controls.

### Confirmation Dialog

Recommended copy:

```text
Enter UF2 bootloader mode?

Audio and USB MIDI will stop when JUMBLEQ restarts. The Configurator
does not save settings automatically before entering the bootloader.

After arming, release SW3 once, then hold it for 2 seconds on JUMBLEQ.
Keep holding it until the JUMBLEQ UF2 drive appears.
```

Actions:

- Secondary: **Cancel**
- Primary warning action: **Arm UF2 mode**

If `dirty` is true, display an additional prominent warning:

```text
There are unsaved changes. They may be lost when JUMBLEQ restarts.
Use Save to device first if you want to keep them.
```

Do not save to EEPROM automatically. Entering a bootloader must not have the hidden side effect of making temporary settings permanent.

### Waiting for Physical Confirmation

After PC124 is sent successfully, keep the dialog open and replace its contents with a waiting state:

```text
Request sent

On JUMBLEQ, release SW3 once, then hold it for 2 seconds.
Keep holding it until the JUMBLEQ UF2 drive appears.

Request expires in 10 seconds.
```

Requirements:

- Show a visible countdown from 10 seconds.
- Disable repeated arm submissions while the request is active.
- Provide a **Cancel request** button that sends PC125 when MIDI is still connected.
- Treat Escape, dialog close, manual MIDI disconnect, and component unmount as cancellation. Send PC125 on a best-effort basis before clearing local state.
- When the 10-second timer expires without a device disconnect, send PC125 on a best-effort basis and show that the request expired.
- Do not claim that the device accepted the command because the initial protocol has no acknowledgement message. Use **Request sent**, not **JUMBLEQ armed**.

Recommended expiry copy:

```text
The request expired. JUMBLEQ did not enter UF2 mode.
Try again, or use the manual SW3 + RESET procedure.
```

### Expected USB MIDI Disconnection

When the connected JUMBLEQ MIDI ports disappear during the active 10-second window, treat it as an expected bootloader transition rather than a generic connection failure.

Recommended copy:

```text
JUMBLEQ MIDI disconnected

Check that the JUMBLEQ UF2 drive appears, then copy the .uf2 firmware
file to it. The Configurator will reconnect after JUMBLEQ restarts.
```

The Web Configurator cannot confirm that the operating system mounted the UF2 drive, so it must not report **UF2 mode entered** as a verified fact. It may only report the observed MIDI disconnection and tell the user what to check.

Keep the existing reconnect target. When the application firmware returns after the UF2 copy, reuse the current automatic reconnect and configuration-sync path.

If the user unplugged the cable instead of entering the bootloader, provide a **Return to connection controls** or equivalent action so the special transition state does not trap the UI.

## MIDI Hook Changes

Update `app/midi/use-jumbleq-midi.ts` so the bootloader transition is coordinated with the existing connection and reconnect lifecycle.

Recommended public state:

```ts
export type Uf2TransitionState =
  | "idle"
  | "awaiting-switch"
  | "midi-disconnected"
  | "expired";
```

Recommended public API additions:

```ts
uf2TransitionState: Uf2TransitionState;
uf2ArmDeadline: number | null;
armUf2Bootloader: () => boolean;
cancelUf2Bootloader: () => void;
clearUf2Transition: () => void;
```

`armUf2Bootloader()` must:

1. Refuse the operation unless the status is `ready` and the selected MIDI output is connected.
2. End any active curve-edit session before arming. PC120 must be sent before PC124 when curve edit mode is active.
3. Send `ARM_UF2_BOOTLOADER` exactly once.
4. Only enter `awaiting-switch` if the send succeeds.
5. Record a deadline 10 seconds in the future and start the expiry timer.
6. Preserve the current reconnect target.

`cancelUf2Bootloader()` must:

1. Send `CANCEL_UF2_BOOTLOADER` if the output is still available.
2. Clear the deadline and expiry timer.
3. Return the transition state to `idle`.

The MIDI state-change handler must:

- Detect disappearance of the selected JUMBLEQ ports while `awaiting-switch`.
- Change the transition state to `midi-disconnected` and clear the arm timer.
- Avoid showing the normal unexpected-disconnection error for this case.
- Continue watching for the same device identity so existing automatic reconnection still works after the firmware update.
- Return the transition state to `idle` only after the MIDI ports reopen and the normal PC126 synchronization completes successfully.

Do not infer a successful bootloader transition from a PC124 send alone.

## Page and Dialog Changes

Update `app/page.tsx` to:

- Add the Device-section entry point.
- Render the confirmation, countdown, expiry, and expected-disconnection states.
- Pass `dirty` into the dialog so unsaved changes are clearly disclosed.
- Keep normal setting controls unchanged until the device actually disconnects.
- Prevent multiple dialogs from being open at the same time.

The existing `HelpDialog` already implements focus capture, Escape handling, body-scroll locking, and focus restoration. Reuse that behavior for the UF2 dialog, preferably by extracting a small shared accessible-dialog wrapper rather than copying divergent keyboard handling.

The UF2 dialog must have:

- `role="dialog"`
- `aria-modal="true"`
- A labelled title
- Keyboard focus containment
- Focus restoration to the **Enter UF2 mode** button on close
- A live region for the state change and countdown that does not announce every animation frame

Update `app/globals.css` with warning-action and UF2-dialog styles consistent with the existing dark UI. Do not rely on color alone to communicate that the action interrupts audio.

## Help and Documentation

Add a short **Firmware update** section to the existing Help dialog that explains:

- The Configurator only starts a temporary confirmation window.
- SW3 must be held on the physical device.
- Audio and MIDI disconnect during the update.
- Unsaved settings are not stored automatically.
- The manual SW3 + RESET procedure remains available.

Link to the JUMBLEQ firmware-update guide:

```text
https://github.com/yamamo2shun1/JUMBLEQ/blob/main/Docs/user-guide/firmware-update.md
```

## Compatibility Behavior

### Firmware Without PC124/PC125 Support

Older firmware silently ignores both commands. Therefore:

- The Configurator must remain usable after the request expires.
- The UI must not label a successful MIDI send as proof of firmware support.
- The expiry message must direct the user to the manual SW3 + RESET procedure.
- No configuration value, sync counter, or dirty state may change when PC124 or PC125 is sent.

A future firmware-version or capability response may be used to hide or disable the action on unsupported firmware, but adding a version protocol is not part of this change.

### iPad and MIDIWeb Browser

Use standard Web MIDI Program Change messages only. Do not request SysEx permission for this feature. The modal and countdown must work at narrow mobile widths and with touch input.

## File-Level Change List

| File | Required change |
|---|---|
| `app/midi/jumbleq-midi.ts` | Export PC124 arm and PC125 cancel messages; keep both outside configuration decoding and synchronization. |
| `app/midi/use-jumbleq-midi.ts` | Add transition state, arm/cancel methods, timeout cleanup, and expected-disconnection handling integrated with auto-reconnect. |
| `app/page.tsx` | Add the Device action and accessible multi-state confirmation dialog; disclose dirty settings. |
| `app/globals.css` | Add warning-action, dialog, countdown, mobile, and status styles. |
| `tests/jumbleq-midi.test.mjs` | Verify exact command bytes and confirm neither command decodes as a configuration field. |
| `e2e/web-midi-mock.ts` | Allow tests to simulate no response, expected MIDI disappearance after PC124, and application-mode reconnection. |
| `e2e/jumbleq-configurator.spec.ts` | Cover the complete UI, safety, timeout, disconnect, and reconnect flows. |
| `README.md` | Mention the UF2 transition in the E2E coverage list after the feature is enabled. |

## Test Requirements

### Unit Tests

- `ARM_UF2_BOOTLOADER` equals `[0xCE, 124]`.
- `CANCEL_UF2_BOOTLOADER` equals `[0xCE, 125]`.
- PC124 and PC125 do not decode as configuration settings.
- Existing PC120 through PC123, PC126, and PC127 behavior remains unchanged.

### End-to-End Tests

- The entry button is disabled when JUMBLEQ is not ready.
- The first button click opens the confirmation dialog and sends no MIDI.
- Confirming sends one PC124 message and displays the physical SW3 instructions.
- An active curve-edit session sends PC120 before PC124.
- Cancelling sends PC125 and clears the countdown.
- Escape and dialog close perform the same cancellation.
- Expiry sends PC125, reports expiry, and leaves the Configurator usable.
- Dirty settings produce the unsaved-settings warning and are not automatically saved with PC127.
- Expected MIDI disappearance shows the UF2 guidance rather than a generic error.
- Reappearance of the same MIDI device triggers the existing PC126 sync and returns to the normal connected state.
- Unexpected disconnection outside the arm window retains the existing reconnect behavior.
- The flow is keyboard accessible and usable at the existing mobile viewport.

## Acceptance Criteria

- No MIDI is sent merely by opening the confirmation dialog.
- No single UI click can both arm and restart JUMBLEQ.
- PC124 is sent only after explicit confirmation while the synchronized device is connected.
- The Configurator never sends PC127 as part of the bootloader flow.
- The physical SW3 requirement and 10-second limit are visible before the arm command is sent.
- The waiting state can be cancelled and cannot submit duplicate arm commands.
- A missing MIDI device during the arm window is treated as an expected transition without falsely claiming that the UF2 volume was mounted.
- The Configurator automatically reconnects and synchronizes when JUMBLEQ application firmware returns.
- Older firmware remains fully usable and falls back to the documented manual entry method.
- `npm test` and `npm run test:e2e` pass.

## Out of Scope

- Uploading or selecting a UF2 file from the Configurator.
- Detecting the mounted UF2 mass-storage volume from the browser.
- Changing the UF2 bootloader itself.
- Application-firmware implementation details; the required behavior is documented only as an integration contract below.
- Adding the feature to the legacy Max Standalone or Max for Live Configurators.

## Firmware Integration Contract

The application firmware must honor this Configurator contract:

- PC124 Ch. 15 starts a 10-second arm window but never resets immediately.
- The device requires SW3 to be observed released after arming, then continuously pressed for 2 seconds.
- PC125 Ch. 15, timeout, or USB removal cancels the arm window.
- The OLED clearly shows the waiting, countdown, and cancellation states.
- On physical confirmation, the application clears both OLED displays and then resets while SW3 is still held so the existing bootloader switch check selects UF2 mode.
- Normal audio and configuration behavior is unchanged while no arm window is active.

## References

- [Configurator MIDI implementation](https://github.com/yamamo2shun1/JUMBLEQ-Configurator/blob/4effe739693c23542e1bcf9992d8802ee78d48fc/app/midi/jumbleq-midi.ts)
- [Configurator MIDI hook](https://github.com/yamamo2shun1/JUMBLEQ-Configurator/blob/4effe739693c23542e1bcf9992d8802ee78d48fc/app/midi/use-jumbleq-midi.ts)
- [Configurator page](https://github.com/yamamo2shun1/JUMBLEQ-Configurator/blob/4effe739693c23542e1bcf9992d8802ee78d48fc/app/page.tsx)
- [MIDI receive specification](../reference/midi/receive.md)
- [Firmware update guide](../user-guide/firmware-update.md)
- [Existing boot switch check](../../STM32CubeIDE/MX25UW25645GXDI00_STM32H7S3Z8T/Boot/Core/Src/boot_mode.c)
