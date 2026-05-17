from ct_pi_uart import (
    ArduinoPathClient,
    format_coord,
    find_merge_goal,
    generate_micro_path
)


class FakeSerial:
    def __init__(self, lines):
        self.lines = lines
        self.written = []
        self.is_open = True

    def write(self, data):
        self.written.append(data)

    def readline(self):
        if self.lines:
            return self.lines.pop(0)
        return b""

    def reset_input_buffer(self):
        pass

    def close(self):
        self.is_open = False


def test_parse_coord():
    client = ArduinoPathClient.__new__(ArduinoPathClient)

    assert client.parse_coord("(1, 2, 3, 90)") == (1.0, 2.0, 3.0, 90.0)
    assert client.parse_coord("bad data") is None
    assert client.parse_coord("(1, 2, 3)") is None


def test_get_macro_path():
    client = ArduinoPathClient.__new__(ArduinoPathClient)
    client.TIME_OUT = 1
    client.ser = FakeSerial([
        b"(1, 2, 3, 90)\n",
        b"(4, 5, 6, 180)\n",
        b"END\n"
    ])

    path = client.get_macro_path()

    assert path == [
        (1.0, 2.0, 3.0, 90.0),
        (4.0, 5.0, 6.0, 180.0)
    ]

    assert client.ser.written[0] == b"Start\n"


def test_send_micro_path_success():
    client = ArduinoPathClient.__new__(ArduinoPathClient)
    client.TIME_OUT = 1
    client.ser = FakeSerial([
        b"Initiate\n",
        b"Done\n"
    ])

    micro_path = [
        "(1.00, 2.00, 3.00, 90.00)",
        "(4.00, 5.00, 6.00, 180.00)"
    ]

    result = client.send_micro_path(micro_path)

    assert result is True
    assert client.ser.written[0] == b"Ready\n"
    assert b"2\n" in client.ser.written[1]


def test_format_coord():
    assert format_coord((1, 2, 3, 90)) == "(1.00, 2.00, 3.00, 90.00)"


def test_find_merge_goal():
    current = (1, 1, 1, 0)
    macro_path = [
        (10, 10, 1, 0),
        (2, 1, 1, 0),
        (5, 5, 1, 0)
    ]

    assert find_merge_goal(current, macro_path) == (2, 1, 1, 0)


def test_generate_micro_path():
    current = (0, 0, 1, 0)
    macro_path = [(3, 0, 1, 0)]
    obs = {
        "detected": 1,
        "front": 350,
        "left": 900,
        "right": 500
    }

    result = generate_micro_path(current, macro_path, obs)

    assert result == [
        "(0.00, 1.00, 1.00, 0.00)",
        "(1.00, 1.00, 1.00, 0.00)",
        "(2.00, 0.00, 1.00, 0.00)",
        "(3.00, 0.00, 1.00, 0.00)"
    ]
