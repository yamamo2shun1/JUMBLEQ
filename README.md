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

## Block Diagram
<img width="1217" height="842" alt="JUMBLEQ drawio" src="https://github.com/user-attachments/assets/cbeb1f4e-1354-4cfa-bd1b-a0ead0e7d694" />

## Prototype 1
![DSCF6165-2](https://github.com/user-attachments/assets/813ef216-b056-449b-9184-85f06c5ffc14)
![DSCF6172-2](https://github.com/user-attachments/assets/47018ccd-0286-43c2-a8e6-ca9e42f9c408)
![DSCF6176-2](https://github.com/user-attachments/assets/3022de92-4aaf-4b04-854f-da39a3611bc7)

## Main board
![OTO_overall](https://github.com/user-attachments/assets/7e5d887a-9eaa-4ffe-a4b5-3ef646b3e20a)
![OTO_top](https://github.com/user-attachments/assets/bd03cb58-2311-4a0e-bf9f-21e4cad265d9)

## CODEC module
[AK4619 Audio Module](https://github.com/YamamotoWorksDev/AK4619_Module)

## Byte Options

<img width="1494" height="94" alt="XSPI1_HSLV" src="https://github.com/user-attachments/assets/ebc3223b-eeab-4298-8f9b-c5c720aa8761" />
<img width="1499" height="283" alt="DTCM_AXI_SHARE" src="https://github.com/user-attachments/assets/3e3834df-2523-460f-911c-b9adc2ca7339" />
