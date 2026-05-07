from machine import UART
from time import sleep_ms

BAUDRATE = 115200
UART_TX_PIN = 1
UART_RX_PIN = 3

STATE_FORWARD = "FORWARD"
STATE_LEFT = "LEFT"
STATE_RIGHT = "RIGHT"
STATE_SEARCH_LEFT = "SEARCH_LEFT"
STATE_SEARCH_RIGHT = "SEARCH_RIGHT"
STATE_STOP = "STOP"

PATH_CMD_PREFIX = "PATH "
NODE_CMD_PREFIX = "NODE "

last_turn = "LEFT"
current_state = STATE_STOP
rx_buffer = ""

uart = UART(1, BAUDRATE, tx=UART_TX_PIN, rx=UART_RX_PIN)


def choose_line_follow_state(frame):
    global last_turn
    global current_state

    if len(frame) != 3:
        return current_state

    if frame[0] not in "01" or frame[1] not in "01" or frame[2] not in "01":
        return current_state

    # Protocol:
    # 0 = line detected
    # 1 = no line
    left_on_line = frame[0] == "0"
    center_on_line = frame[1] == "0"
    right_on_line = frame[2] == "0"

    # Forward first.
    if center_on_line:
        return STATE_FORWARD

    if left_on_line and not right_on_line:
        last_turn = "LEFT"
        return STATE_LEFT

    if right_on_line and not left_on_line:
        last_turn = "RIGHT"
        return STATE_RIGHT

    if left_on_line and right_on_line:
        if last_turn == "RIGHT":
            return STATE_RIGHT
        return STATE_LEFT

    if last_turn == "RIGHT":
        return STATE_SEARCH_RIGHT

    return STATE_SEARCH_LEFT


while True:
    if uart.any():
        try:
            data = uart.read()
        except Exception:
            data = None

        if data:
            try:
                rx_buffer += data.decode("utf-8")
            except Exception:
                rx_buffer = ""

            if len(rx_buffer) > 128:
                rx_buffer = rx_buffer[-64:]

            while "\n" in rx_buffer:
                frame, rx_buffer = rx_buffer.split("\n", 1)
                frame = frame.strip()

                if frame.startswith(PATH_CMD_PREFIX):
                    current_state = STATE_FORWARD
                    continue

                if frame.startswith(NODE_CMD_PREFIX):
                    continue

                current_state = choose_line_follow_state(frame)

    try:
        uart.write(current_state + "\n")
    except Exception:
        pass

    sleep_ms(20)
