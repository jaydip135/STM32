import can
import time

LED_PATH = "/sys/class/leds/ACT/brightness"

def set_led(state):
    with open(LED_PATH, 'w') as led_file:
        led_file.write('1' if state else '0')

def main():
    bus = can.interface.Bus(channel='can1', bustype='socketcan')
    print("Listening for CAN messages...")

    while True:
        msg = bus.recv()
        if msg and msg.arbitration_id == 0x446 and len(msg.data) >= 2:
            delay_ms, loop_count = msg.data[0], msg.data[1]
            print(f"Blinking LED with delay: {delay_ms} ms, loops: {loop_count}")
            
            for _ in range(loop_count):
                set_led(1)
                time.sleep(delay_ms / 1000)
                set_led(0)
                time.sleep(delay_ms / 1000)

            bus.send(can.Message(arbitration_id=0x107, data=[100, 10], is_extended_id=False))
            print("Sent CAN message: ID=0x107, Data=[100, 10]")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        set_led(0)
        print("Exiting...")

