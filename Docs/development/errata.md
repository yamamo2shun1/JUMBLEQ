# STM32H7S3 Errata

## Erratum 2.2.15

ST documents the following STM32H7R/S device erratum:

- [I/O compensation could alter the duty cycle of a high-frequency output signal](https://www.st.com/resource/en/errata_sheet/es0596-stm32h7rxx7sxx-device-errata-stmicroelectronics.pdf)

The recommended workaround has already been implemented in the boot-side code:

- [`sbs.c`](../../STM32CubeIDE/MX25UW25645GXDI00_STM32H7S3Z8T/Boot/Core/Src/sbs.c)

## Erratum 2.2.17

[ES0596 Rev 10, section 2.2.17](https://www.st.com/resource/en/errata_sheet/es0596-stm32h7rxx7sxx-device-errata-stmicroelectronics.pdf)
describes a system hang on access to `0x25000000`–`0x25FFFFFF`, including
speculative accesses and devices without GFXMMU.

Both the application-side [`main.c`](../../STM32CubeIDE/JUMBLEQ/Appli/Core/Src/main.c)
and the Boot-side [`main.c`](../../STM32CubeIDE/MX25UW25645GXDI00_STM32H7S3Z8T/Boot/Core/Src/main.c)
configure this unused range as MPU Region 4 with Strongly-ordered, no-access
and execute-never attributes before enabling the CPU caches. DMA completion
diagnostics and the existing cache settings remain enabled.

The user confirmed at least six hours of continuous operation with the
application-only comparison UF2, without a freeze. Audio, LEDs, OLED/UI,
volume and mute operations were reported normal, meeting the agreed test
criterion.

The user then built and programmed Boot and App through STM32CubeIDE. The
generated Boot Release ELF was inspected and contained the same MPU protection
before cache enable. The Boot+App configuration ran overnight without a freeze;
audio/UI, volume/mute, power cycling, UF2 mode and transition to the application
were all reported normal on 2026-09-29.

These results meet the agreed hardware test criterion and support the
workaround, but do not directly prove which speculative access caused the
original freeze. App UF2 programming alone does not update Boot. The current
App Release (STLINK) Run configuration downloads Boot without automatically
building it, so Boot must be rebuilt separately before programming changes.
