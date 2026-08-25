import serial
import time
import lewansoul_lx16a

SERIAL_PORT = "/dev/ttyUSB0"
BAUDRATE = 115200
TIMEOUT = 1


class Cassandra:
    def __init__(self, port=SERIAL_PORT):
        self.controller = lewansoul_lx16a.ServoController(
            serial.Serial(port, BAUDRATE, timeout=TIMEOUT)
        )
        self.servos = {}

    def init_servos(self, ids=range(1, 12)):
        """Initialize servos dynamically"""
        self.servos = {i: self.controller.servo(i) for i in ids}

    def move(self, positions, time_ms=500):
        """Move servos individually"""
        for servo_id, pos in positions.items():
            self.servos[servo_id].move(pos, time_ms)

    def move_sync(self, positions, time_ms=1000):
        """Prepare synchronous movement"""
        for servo_id, pos in positions.items():
            self.servos[servo_id].move_prepare(pos, time_ms)
        self.controller.move_start()

    # === High-level behaviors ===

    #    def rotate_body(self):
    # self.move({5: 800}, time_ms=1000) # 600
    #       pray_movement()
    #      self.move({5: 600}, time_ms=1500) # 200
    #     time.sleep(2)
    #    self.move({5: 200}, time_ms=1500) # 600
    #   time.sleep(2)
    #  self.move({5: 400}, time_ms=1500) # 600

    def pray_pose(self):
        self.move_sync({1: 450, 2: 700, 3: 500, 4: 450})

    def movement_whole_servos(self):
        self.move(
            {
                1: 250,
                2: 500,
                3: 300,
                4: 200,
                6: 800,
                7: 300,
                8: 685,
                9: 400,
            }
        )
        time.sleep(3)
        self.base_positions()

    def base_positions(self):
        self.move_sync(
            {
                1: 450,
                2: 730,
                3: 500,
                4: 450,
                5: 400,
                6: 660,
                7: 100,
                8: 470,
                9: 200,
                10: 460,
                11: 550,
            }
        )

    def pray_movement(self):
        self.move(
            {
                1: 275,
                2: 730,
                3: 560,
                4: 230,
                6: 850,
                7: 100,
                8: 400,
                9: 410,
            }
        )
        time.sleep(1)

    def rotate_body(self):
        # self.move({5: 800}, time_ms=1000) # 600
        self.pray_movement()
        time.sleep(1)
        self.move({5: 600}, time_ms=1500)  # 200
        time.sleep(2)
        self.move({5: 200}, time_ms=1500)  # 600
        time.sleep(2)
        self.move({5: 400}, time_ms=1500)  # 600

    def pray_movement_right(self):
        self.move(
            {
                6: 850,
                7: 100,
                8: 400,
                9: 410,
            }
        )
        time.sleep(2)
        self.base_positions()
        time.sleep(2)

    def pray_movement_left(self):
        self.move(
            {
                1: 160,
                2: 750,
                3: 560,
                4: 230,
            }
        )
        time.sleep(2)
        self.base_positions()
        time.sleep(2)

    def rotate_head_right_down(self):
        self.move(
            {
                10: 650,
                11: 650,
            },
            time_ms=1000,
        )
        time.sleep(1)

    def rotate_head_left_up(self):
        self.move(
            {
                10: 350,
                11: 350,
            },
            time_ms=1000,
        )
        time.sleep(1)

    def rotate_head_left(self):
        self.move({10: 300})
        time.sleep(1)

    def rotate_head_right(self):
        self.move({10: 700})
        time.sleep(1)

    def rotate_head_up(self):
        self.move({11: 300})
        time.sleep(1)

    def rotate_head_down(self):
        self.move({11: 700})
        time.sleep(1)

    def position_crest(self):
        self.move(
            {
                2: 350,
                7: 500,
            },
            time_ms=1000,
        )
        time.sleep(3)


if __name__ == "__main__":
    cassandra = Cassandra()
    cassandra.init_servos()

    cassandra.rotate_head_left()
    cassandra.base_positions()
    cassandra.rotate_body()
    cassandra.rotate_head_right()
    cassandra.base_positions()

    cassandra.pray_movement()
    cassandra.rotate_head_up()
    cassandra.pray_movement_left()
    cassandra.base_positions()
    cassandra.pray_movement_right()
    cassandra.rotate_head_down()
    cassandra.base_positions()

    cassandra.movement_whole_servos()
    cassandra.pray_movement_right()
    cassandra.base_positions()
    cassandra.pray_movement()
    cassandra.rotate_head_down()
    cassandra.rotate_head_right()
    cassandra.position_crest()
    cassandra.rotate_head_left()
    cassandra.base_positions()
