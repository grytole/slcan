# SLCAN

#### Required hardware
* BluePill board (or board with any other STM32 chip with CAN support);
* USB2UART adapter board;
* CAN bus driver board.

#### Hardware preparation
* Connect USB2UART adapter to USART2 pins of STM32:
  USB2UART | STM32
  ---------|-------------
  RX       | A2(USART2_TX)
  TX       | A3(USART2_RX)
* Connect CAN driver to CAN1 pins of STM32:
  CAN driver | STM32
  -------|-------------
  CANRX  | PB8(CAN1_RX)
  CANTX  | PB9(CAN1_TX)

#### Firmware preparation
```
sudo apt install git gcc-arm-none-eabi stm32flash

cd ~/dev
git clone https://github.com/grytole/slcan.git
cd ./slcan
git submodule add https://github.com/libopencm3/libopencm3.git
git commit -m "import libopencm3"
make -C ./libopencm3
make

sudo make flash
```

#### Usage
```
sudo modprobe can
sudo modprobe can-raw
sudo modprobe slcan
sudo slcand -s8 -S115200 -o -F ttyUSB0
sudo ip link set slcan0 up

candump -a slcan0

cansend slcan0 123#aabbccdd
```

#### Current state (21-jun-2021)
* Supported CAN speeds:
  Speed (kbaud) | Option
  ------|-------
  10|S0
  20|S1
  50|S2
  100|S3
  125|S4
  250|S5
  500|S6
  800|S7
  1000|S8
* UART speed change command (Un) not implemented: default settings are 115200 (8n1);
* Filters not implemented;
* Permanent settings storage not implemented.
  
