"""Webots controller for HIL shortest-path navigation using P1-P27 graph.

This version:
1. Computes shortest path before movement.
2. Prints the shortest path before movement.
3. Moves using ESP32 line-following states.
4. Uses shortest path to force turns at planned nodes.
5. Prints every crossed node.
6. Stops immediately when the crossed node is the goal.
7. Snaps the robot exactly to the goal node.
"""

from controller import Supervisor
import serial
from serial import SerialException
import time
import math


# -------------------- USER CONFIG --------------------
SERIAL_PORT = "COM3"
BAUDRATE = 115200
TIMEOUT_S = 0.02

# Choose any start and goal from P1 to P27
PATH_START_NODE = "P1"
PATH_GOAL_NODE = "P27"

LINE_SENSOR_NAMES = ("gs0", "gs1", "gs2")
LEFT_MOTOR_NAME = "left wheel motor"
RIGHT_MOTOR_NAME = "right wheel motor"

LINE_THRESHOLD = 500.0
MAX_SPEED = 6.28

ROBOT_DEF_NAME = "E_PUCK"

ARENA_WIDTH_M = 1.2
ARENA_HEIGHT_M = 0.8
TRACK_IMAGE_WIDTH_PX = 1024.0
TRACK_IMAGE_HEIGHT_PX = 674.0
TRACK_Z = -6.39438e-05

# Start slightly after the node so the robot does not start exactly on a junction.
START_EDGE_OFFSET_M = 0.025

# Node radius for printing/crossing nodes.
NODE_REACHED_RADIUS_M = 0.045

# Turn forcing only at path nodes.
FORCED_TURN_STEPS = 42
FORCED_STRAIGHT_STEPS = 10

NO_RESPONSE_LIMIT = 80
DEBUG_PRINT_PERIOD = 50

# If robot starts sideways, try 1.5708, -1.5708, or 3.1416.
ROBOT_HEADING_OFFSET_RAD = 0.0
# ----------------------------------------------------


STATE_FORWARD = "FORWARD"
STATE_LEFT = "LEFT"
STATE_RIGHT = "RIGHT"
STATE_SEARCH_LEFT = "SEARCH_LEFT"
STATE_SEARCH_RIGHT = "SEARCH_RIGHT"
STATE_STOP = "STOP"

VALID_STATES = (
    STATE_FORWARD,
    STATE_LEFT,
    STATE_RIGHT,
    STATE_SEARCH_LEFT,
    STATE_SEARCH_RIGHT,
    STATE_STOP,
)


NODE_PIXELS = {
    "P1":  (87, 0),
    "P2":  (174, 0),
    "P3":  (261, 0),
    "P4":  (348, 0),

    "P5":  (87, 128),
    "P6":  (174, 128),
    "P7":  (261, 128),
    "P8":  (348, 128),
    "P9":  (511, 128),
    "P10": (935, 128),

    "P11": (511, 252),
    "P12": (935, 252),

    "P13": (87, 339),
    "P14": (511, 339),
    "P15": (935, 339),

    "P16": (87, 426),
    "P17": (511, 426),

    "P18": (87, 550),
    "P19": (511, 550),
    "P20": (674, 550),
    "P21": (761, 550),
    "P22": (848, 550),
    "P23": (935, 550),

    "P24": (674, 673),
    "P25": (761, 673),
    "P26": (848, 673),
    "P27": (935, 673),
}


GRAPH = {
    "P1":  ("P5",),
    "P2":  ("P6",),
    "P3":  ("P7",),
    "P4":  ("P8",),

    "P5":  ("P1", "P6", "P13"),
    "P6":  ("P2", "P5", "P7"),
    "P7":  ("P3", "P6", "P8"),
    "P8":  ("P4", "P7", "P9"),
    "P9":  ("P8", "P10", "P11"),
    "P10": ("P9", "P12"),

    "P11": ("P9", "P12", "P14"),
    "P12": ("P10", "P11", "P15"),

    "P13": ("P5", "P14", "P16"),
    "P14": ("P11", "P13", "P15", "P17"),
    "P15": ("P12", "P14", "P23"),

    "P16": ("P13", "P17", "P18"),
    "P17": ("P14", "P16", "P19"),

    "P18": ("P16", "P19"),
    "P19": ("P17", "P18", "P20"),
    "P20": ("P19", "P21", "P24"),
    "P21": ("P20", "P22", "P25"),
    "P22": ("P21", "P23", "P26"),
    "P23": ("P15", "P22", "P27"),

    "P24": ("P20",),
    "P25": ("P21",),
    "P26": ("P22",),
    "P27": ("P23",),
}


def texture_px_to_world(px, py):
    world_x = ((px / TRACK_IMAGE_WIDTH_PX) - 0.5) * ARENA_WIDTH_M
    world_y = (0.5 - (py / TRACK_IMAGE_HEIGHT_PX)) * ARENA_HEIGHT_M
    return (world_x, world_y, TRACK_Z)


NODE_POSITIONS = {
    label: texture_px_to_world(px, py)
    for label, (px, py) in NODE_PIXELS.items()
}


def clamp_speed(value):
    return max(-MAX_SPEED, min(MAX_SPEED, value))


def state_to_speeds(state):
    if state == STATE_FORWARD:
        return 0.55 * MAX_SPEED, 0.55 * MAX_SPEED

    if state == STATE_LEFT:
        return 0.14 * MAX_SPEED, 0.55 * MAX_SPEED

    if state == STATE_RIGHT:
        return 0.55 * MAX_SPEED, 0.14 * MAX_SPEED

    if state == STATE_SEARCH_LEFT:
        return -0.32 * MAX_SPEED, 0.32 * MAX_SPEED

    if state == STATE_SEARCH_RIGHT:
        return 0.32 * MAX_SPEED, -0.32 * MAX_SPEED

    return 0.0, 0.0


def sensor_detects_line(sensor_value):
    return sensor_value < LINE_THRESHOLD


def build_sensor_message(left_on_line, center_on_line, right_on_line):
    msg = ""
    msg += "0" if left_on_line else "1"
    msg += "0" if center_on_line else "1"
    msg += "0" if right_on_line else "1"
    return (msg + "\n").encode("utf-8")


def edge_cost(node_a, node_b):
    ax, ay = NODE_PIXELS[node_a]
    bx, by = NODE_PIXELS[node_b]
    dx = ax - bx
    dy = ay - by
    return (dx * dx + dy * dy) ** 0.5


def dijkstra(start_node, goal_node):
    if start_node not in GRAPH or goal_node not in GRAPH:
        return []

    dist = {}
    prev = {}
    unvisited = []

    for node in GRAPH:
        dist[node] = 10**9
        prev[node] = None
        unvisited.append(node)

    dist[start_node] = 0

    while unvisited:
        current = unvisited[0]

        for node in unvisited:
            if dist[node] < dist[current]:
                current = node

        if current == goal_node:
            break

        if dist[current] >= 10**9:
            break

        unvisited.remove(current)

        for neighbor in GRAPH[current]:
            if neighbor not in unvisited:
                continue

            new_dist = dist[current] + edge_cost(current, neighbor)

            if new_dist < dist[neighbor]:
                dist[neighbor] = new_dist
                prev[neighbor] = current

    if dist[goal_node] >= 10**9:
        return []

    path = []
    node = goal_node

    while node is not None:
        path.insert(0, node)
        node = prev[node]

    return path


def edge_heading(node_a, node_b):
    ax, ay = NODE_PIXELS[node_a]
    bx, by = NODE_PIXELS[node_b]

    dx = bx - ax
    dy = by - ay

    if abs(dx) >= abs(dy):
        return "E" if dx >= 0 else "W"

    return "N" if dy < 0 else "S"


def heading_to_turn(in_heading, out_heading):
    if in_heading == out_heading:
        return "FORWARD"

    order = ("N", "E", "S", "W")

    in_idx = order.index(in_heading)
    out_idx = order.index(out_heading)

    delta = (out_idx - in_idx) % 4

    if delta == 1:
        return "RIGHT"

    if delta == 3:
        return "LEFT"

    return "LEFT"


def build_turn_plan(path):
    turns_by_node = {}

    if len(path) < 3:
        return turns_by_node

    incoming_heading = edge_heading(path[0], path[1])

    for i in range(1, len(path) - 1):
        node = path[i]
        outgoing_heading = edge_heading(path[i], path[i + 1])
        turns_by_node[node] = heading_to_turn(incoming_heading, outgoing_heading)
        incoming_heading = outgoing_heading

    return turns_by_node


def validate_path_nodes():
    known = ", ".join(sorted(GRAPH.keys()))

    if PATH_START_NODE not in GRAPH:
        raise ValueError(f"Unknown start node '{PATH_START_NODE}'. Known nodes: {known}")

    if PATH_GOAL_NODE not in GRAPH:
        raise ValueError(f"Unknown goal node '{PATH_GOAL_NODE}'. Known nodes: {known}")


def distance_3d(a, b):
    dx = a[0] - b[0]
    dy = a[1] - b[1]
    dz = a[2] - b[2]
    return (dx * dx + dy * dy + dz * dz) ** 0.5


def point_offset_toward(start_pos, next_pos, offset):
    dx = next_pos[0] - start_pos[0]
    dy = next_pos[1] - start_pos[1]
    dz = next_pos[2] - start_pos[2]

    length = (dx * dx + dy * dy + dz * dz) ** 0.5

    if length <= 0.0001:
        return start_pos

    scale = offset / length

    return (
        start_pos[0] + dx * scale,
        start_pos[1] + dy * scale,
        start_pos[2] + dz * scale,
    )


def compute_start_rotation(path):
    if len(path) < 2:
        return (0.0, 0.0, 1.0, 0.0)

    start_pos = NODE_POSITIONS[path[0]]
    next_pos = NODE_POSITIONS[path[1]]

    dx = next_pos[0] - start_pos[0]
    dy = next_pos[1] - start_pos[1]

    angle = math.atan2(dy, dx) + ROBOT_HEADING_OFFSET_RAD

    return (0.0, 0.0, 1.0, angle)


def get_robot_node(supervisor):
    return supervisor.getFromDef(ROBOT_DEF_NAME)


def get_robot_position(supervisor):
    robot_node = get_robot_node(supervisor)

    if robot_node is None:
        return None

    return robot_node.getPosition()


def snap_robot_to_node(supervisor, node_label):
    robot_node = get_robot_node(supervisor)

    if robot_node is None:
        return

    translation_field = robot_node.getField("translation")

    if translation_field is None:
        return

    exact_pos = NODE_POSITIONS[node_label]
    translation_field.setSFVec3f(list(exact_pos))
    robot_node.resetPhysics()


def reset_robot_to_start_pose(supervisor, path):
    robot_node = get_robot_node(supervisor)

    if robot_node is None:
        raise RuntimeError(f"Could not find DEF {ROBOT_DEF_NAME}")

    translation_field = robot_node.getField("translation")
    rotation_field = robot_node.getField("rotation")

    if translation_field is None or rotation_field is None:
        raise RuntimeError("Could not access robot translation/rotation fields")

    start_pos = NODE_POSITIONS[PATH_START_NODE]

    if len(path) >= 2:
        next_pos = NODE_POSITIONS[path[1]]
        start_pos = point_offset_toward(start_pos, next_pos, START_EDGE_OFFSET_M)

    start_rotation = compute_start_rotation(path)

    translation_field.setSFVec3f(list(start_pos))
    rotation_field.setSFRotation(list(start_rotation))
    robot_node.resetPhysics()

    print(f"[START] Robot placed at {PATH_START_NODE}")


def try_connect_serial():
    try:
        connection = serial.Serial()
        connection.port = SERIAL_PORT
        connection.baudrate = BAUDRATE
        connection.timeout = TIMEOUT_S
        connection.write_timeout = TIMEOUT_S

        connection.dtr = False
        connection.rts = False
        connection.open()
        connection.setDTR(False)
        connection.setRTS(False)

        time.sleep(3.0)

        connection.reset_input_buffer()
        connection.reset_output_buffer()

        print(f"[HIL] Serial connected at {SERIAL_PORT} @ {BAUDRATE} bps")
        return connection

    except SerialException as exc:
        print(f"[HIL] Serial open failed: {exc}")
        return None


def main():
    validate_path_nodes()

    path = dijkstra(PATH_START_NODE, PATH_GOAL_NODE)

    if len(path) == 0:
        raise RuntimeError(f"No valid path from {PATH_START_NODE} to {PATH_GOAL_NODE}")

    turns_by_node = build_turn_plan(path)

    print("")
    print("========== SHORTEST PATH ==========")
    print(f"[PATH] Start: {PATH_START_NODE}")
    print(f"[PATH] Goal : {PATH_GOAL_NODE}")
    print(f"[PATH] Shortest path: {' -> '.join(path)}")
    print(f"[PATH] Planned turns: {turns_by_node}")
    print("===================================")
    print("")

    robot = Supervisor()
    timestep = int(robot.getBasicTimeStep())

    reset_robot_to_start_pose(robot, path)

    print(f"[NODE] Crossed {PATH_START_NODE}")

    line_sensors = []

    for name in LINE_SENSOR_NAMES:
        sensor = robot.getDevice(name)

        if sensor is None:
            raise RuntimeError(f"Sensor not found: {name}")

        sensor.enable(timestep)
        line_sensors.append(sensor)

    left_motor = robot.getDevice(LEFT_MOTOR_NAME)
    right_motor = robot.getDevice(RIGHT_MOTOR_NAME)

    if left_motor is None:
        raise RuntimeError(f"Left motor not found: {LEFT_MOTOR_NAME}")

    if right_motor is None:
        raise RuntimeError(f"Right motor not found: {RIGHT_MOTOR_NAME}")

    left_motor.setPosition(float("inf"))
    right_motor.setPosition(float("inf"))
    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)

    ser = try_connect_serial()

    current_state = STATE_STOP
    no_response_steps = 0
    route_command_sent = False
    debug_counter = 0

    next_node_event_index = 1

    forced_override_state = None
    forced_override_steps = 0
    navigation_done = False

    while robot.step(timestep) != -1:
        sensor_values = [sensor.getValue() for sensor in line_sensors]

        left_line = sensor_detects_line(sensor_values[0])
        center_line = sensor_detects_line(sensor_values[1])
        right_line = sensor_detects_line(sensor_values[2])

        msg = build_sensor_message(left_line, center_line, right_line)

        robot_pos = get_robot_position(robot)

        if robot_pos is not None and next_node_event_index < len(path):
            target_node = path[next_node_event_index]
            target_pos = NODE_POSITIONS[target_node]
            distance_to_target = distance_3d(robot_pos, target_pos)

            if distance_to_target <= NODE_REACHED_RADIUS_M:
                print(f"[NODE] Crossed {target_node}")

                if ser is not None:
                    try:
                        ser.write(f"NODE {target_node}\n".encode("utf-8"))
                    except Exception:
                        pass

                # IMPORTANT FIX:
                # If the crossed node is the goal, stop immediately.
                if target_node == PATH_GOAL_NODE:
                    left_motor.setVelocity(0.0)
                    right_motor.setVelocity(0.0)

                    snap_robot_to_node(robot, PATH_GOAL_NODE)

                    print("")
                    print("========== NAVIGATION DONE ==========")
                    print(f"[GOAL] Reached {PATH_GOAL_NODE}")
                    print(f"[PATH] Completed path: {' -> '.join(path)}")
                    print("=====================================")
                    print("")

                    navigation_done = True
                    break

                if target_node in turns_by_node:
                    turn_action = turns_by_node[target_node]

                    if turn_action == "LEFT":
                        forced_override_state = STATE_SEARCH_LEFT
                        forced_override_steps = FORCED_TURN_STEPS
                        print(f"[TURN] Forced LEFT at {target_node}")

                    elif turn_action == "RIGHT":
                        forced_override_state = STATE_SEARCH_RIGHT
                        forced_override_steps = FORCED_TURN_STEPS
                        print(f"[TURN] Forced RIGHT at {target_node}")

                    else:
                        forced_override_state = STATE_FORWARD
                        forced_override_steps = FORCED_STRAIGHT_STEPS
                        print(f"[TURN] Forced FORWARD at {target_node}")

                next_node_event_index += 1

        if navigation_done:
            break

        if ser is not None:
            try:
                if not route_command_sent:
                    ser.write(f"PATH {PATH_START_NODE} {PATH_GOAL_NODE}\n".encode("utf-8"))
                    route_command_sent = True
                    print(f"[HIL] Sent command: PATH {PATH_START_NODE} {PATH_GOAL_NODE}")

                ser.write(msg)

                valid_reply_received = False

                while ser.in_waiting:
                    reply = ser.readline().decode("utf-8", errors="ignore").strip()

                    if reply in VALID_STATES:
                        current_state = reply
                        valid_reply_received = True
                        no_response_steps = 0

                if not valid_reply_received:
                    no_response_steps += 1

                    if no_response_steps >= NO_RESPONSE_LIMIT:
                        current_state = STATE_STOP

            except SerialException as exc:
                print(f"[HIL] Serial runtime error: {exc}")

                try:
                    ser.close()
                except Exception:
                    pass

                ser = None
                current_state = STATE_STOP
                no_response_steps = 0
                route_command_sent = False

        final_state = current_state

        if forced_override_steps > 0 and forced_override_state is not None:
            final_state = forced_override_state
            forced_override_steps -= 1

            if forced_override_steps == 0:
                forced_override_state = None

        left_speed, right_speed = state_to_speeds(final_state)

        left_motor.setVelocity(clamp_speed(left_speed))
        right_motor.setVelocity(clamp_speed(right_speed))

        debug_counter += 1

        if debug_counter >= DEBUG_PRINT_PERIOD:
            debug_counter = 0
            target_info = "DONE"

            if next_node_event_index < len(path):
                target_info = path[next_node_event_index]

            print(
                "[HIL]",
                "target=" + target_info,
                "send=" + msg.decode("utf-8").strip(),
                "state=" + final_state,
                "sensors=" + str([round(v, 1) for v in sensor_values]),
            )


if __name__ == "__main__":
    main()
