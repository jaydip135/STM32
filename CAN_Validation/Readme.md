# README: Raspberry Pi Dual CAN Communication with STM32 POC

## 1. Setup Raspberry Pi for CAN Communication

### 1.1 Install CAN Utilities

Install the required tools to interface with CAN on Raspberry Pi:

```bash
sudo apt update
sudo apt install can-utils python3-can
```

### 1.2 Configure CAN Interfaces

Edit the Raspberry Pi configuration file to enable CAN functionality:

```bash
sudo nano /boot/config.txt
```

Add the following lines at the end of the file:

```bash
dtoverlay=mcp2515-can0,oscillator=16000000,interrupt=25
dtoverlay=mcp2515-can1,oscillator=16000000,interrupt=24
```

Save and exit the file.

### 1.3 Configure CAN Network

Create a configuration file to set up the CAN interfaces:

```bash
sudo nano /etc/network/interfaces.d/can.conf
```

Add the following content:

```bash
auto can0
iface can0 inet manual
  pre-up /sbin/ip link set can0 type can bitrate 125000
  up /sbin/ifconfig can0 up
  down /sbin/ifconfig can0 down

auto can1
iface can1 inet manual
  pre-up /sbin/ip link set can1 type can bitrate 125000
  up /sbin/ifconfig can1 up
  down /sbin/ifconfig can1 down
```

Save and exit the file.

### 1.4 Bring Up CAN Interfaces

Reboot the Raspberry Pi to apply the changes:

```bash
sudo reboot
```

After reboot, activate the interfaces:

```bash
sudo ip link set can0 up type can bitrate 125000
sudo ip link set can1 up type can bitrate 125000
```

Verify the interfaces are up:

```bash
ifconfig
```

### 1.5 Run the Python Script

1. Clone the repository or download the Python file:
   ```bash
   git clone <repository-link>
   cd <repository-folder>
   ```
2. Run the Python script:
   ```bash
   sudo python3 can_blink_and_send.py
   ```
   The script will:
   - Listen for messages on `can1`.
   - Blink the Raspberry Pi ACT LED based on received CAN data.
   - Send a predefined CAN message back on `can1`.

### Testing with CAN-utils

To test message sending and receiving:

- Send a test message:
  ```bash
  cansend can1 446#FF0A
  ```
- Monitor incoming messages:
  ```bash
  candump can1
  ```

---

## 2. STM32 Setup (NextGen POC)

### 2.1 Download Code

1. Clone or download the STM32 firmware repository.
2. Open the project in **STM32CubeIDE**.

### 2.2 Upload Firmware to STM32

1. Connect the STM32 board to your computer.
2. Build and flash the project to the STM32.
3. Ensure the STM32 is configured for CAN communication at 125 kbps.

---

## 3. Hardware Setup

### Components:

1. **Raspberry Pi**: Dual CAN HAT attached.
2. **CAN Transceiver**: Used to connect the Raspberry Pi to the STM32 POC.
3. **STM32 Board**: Configured as the receiving/transmitting node.

### Wiring:

- **Raspberry Pi CAN HAT**:
  - `CAN_H` and `CAN_L` to the transceiver.
- **CAN Transceiver**:
  - Match `CAN_H` and `CAN_L` to the STM32 board.

### Overview:

```
Raspberry Pi Dual CAN HAT  -->  CAN Transceiver  -->  STM32 POC
```



