# JUMBLEQ
“JUMBLEQ” is short for “jumbled equipment”

## Audio Spec.
- Analog: 4in/4out
- USB: 4in/4out(USB Audio Class 2.0) and MIDI in/out
- Format: 24bit, 48kHz/96kHz

## Electronic Spec.
- MCU: [STM32H7S3Z8](https://www.st.com/en/microcontrollers-microprocessors/stm32h7s3z8.html)
- DSP: [ADAU1466](https://www.analog.com/jp/products/adau1466.html)
- CODEC: [AK4619](https://www.akm.com/jp/ja/products/audio/audio-codec/ak4619vn/)
- IO: [Gateron Low Profile Magnet Jade Pro](https://www.gateron.com/products/gateron-full-pom-low-profile-magnetic-jade-pro-switch-set?srsltid=AfmBOopFkdCLZGMJNqQ-K2jzlEZM16yoIFbTWwkXe4nwWVk6TughK0Un&VariantsId=10870) x6 and Pot x8

## for Windows User
I’m developing JUMBLEQ as a solo project. I can’t realistically develop both an ASIO driver and the necessary firmware support on my own. If you’re using this device on Windows, please use [ASIO4ALL](https://asio4all.org/)—even if you’d prefer a dedicated ASIO driver.

## Firmware Update
JUMBLEQ uses a UF2 bootloader.
When you connect the USB cable while holding down SW3, it is recognized as a USB flash drive, so you can update the firmware simply by copying app.uf2—created using the procedure below—to the drive.
```
> arm-none-eabi-objcopy -O binary JUMBLEQ_Appli.elf app.bin
> python uf2conv.py -c -b 0x90010000 -f STM32H7RS -o app.uf2 app.bin
```

## Block Diagram
<img width="811" height="701" alt="JUMBLEQ_diagram" src="https://github.com/user-attachments/assets/09df1829-d854-44bf-9779-de79fafe3dac" />

## Pinout (main board rev.C)
![JUMBLEQ_main_pinout_revC](https://github.com/user-attachments/assets/fcb13605-1c4f-45cb-ab58-35bc1016bd3e)

## Modules
- Original  
[AK4619 Audio Module](https://github.com/YamamotoWorksDev/AK4619_Module)  
Magnetic Switch & POT Module

- Third party  
[0.96インチ 128×64ドット有機ELディスプレイ(OLED) 白色](https://akizukidenshi.com/catalog/g/g112031/)  
[SparkFun Qwiic OLED Display (0.91 in., 128x32)](https://www.sparkfun.com/sparkfun-qwiic-oled-display-0-91-in-128x32-lcd-24606.html)  
[SparkFun Qwiic EEPROM Breakout - 512Kbit](https://www.sparkfun.com/sparkfun-qwiic-eeprom-breakout-512kbit.html)

## Byte Options

<img width="1494" height="94" alt="XSPI1_HSLV" src="https://github.com/user-attachments/assets/ebc3223b-eeab-4298-8f9b-c5c720aa8761" />
<img width="1499" height="283" alt="DTCM_AXI_SHARE" src="https://github.com/user-attachments/assets/3e3834df-2523-460f-911c-b9adc2ca7339" />
