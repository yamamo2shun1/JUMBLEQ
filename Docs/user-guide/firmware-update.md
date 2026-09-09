# Firmware Update

JUMBLEQ includes a built-in UF2 bootloader, so application firmware can be updated without a dedicated programming tool.

## Enter UF2 Bootloader Mode

### Starting from Power Off

1. Make sure JUMBLEQ is powered off.
2. Hold down SW3.
3. While continuing to hold SW3, connect the USB cable.
4. Release SW3 after JUMBLEQ appears as a USB flash drive.

### While JUMBLEQ Is Running

1. Hold down SW3.
2. While continuing to hold SW3, press and release the RESET button.
3. Release SW3 after JUMBLEQ appears as a USB flash drive.

### From JUMBLEQ Configurator

Firmware v0.14.4 and later can safely prepare UF2 mode from the Web Configurator and clear both OLED displays before restarting:

1. Connect and synchronize JUMBLEQ in the Configurator.
2. In the **Device** section, select **Enter UF2 mode** and then **Arm UF2 mode**.
3. Within 10 seconds, release SW3 once and then hold it continuously for 2 seconds.
4. Keep holding SW3 until JUMBLEQ appears as a USB flash drive.

The Configurator request alone does not restart JUMBLEQ. The request is cancelled if SW3 is not confirmed within 10 seconds, the Configurator cancels it, or USB is disconnected. Unsaved settings are not stored automatically before the restart.

## Install Firmware

Drag and drop the JUMBLEQ `.uf2` firmware file onto the USB drive. The firmware update completes when the file copy finishes.

For information about generating UF2 files from an STM32CubeIDE project, see [Build and UF2 generation](../development/build-and-uf2.md).
