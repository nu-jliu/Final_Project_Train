import socket
import json
import logging
import colorama
import copy
import math

from adafruit_servokit import ServoKit

colorama.init(autoreset=True)


class ColorfulFormatter(logging.Formatter):
    COLORS = {
        logging.DEBUG: colorama.Fore.BLUE,
        logging.INFO: colorama.Fore.GREEN,
        logging.WARNING: colorama.Fore.YELLOW,
        logging.ERROR: colorama.Fore.RED,
        logging.CRITICAL: colorama.Fore.MAGENTA,
    }

    def format(self, record):
        color = self.COLORS.get(record.levelno, colorama.Fore.WHITE)
        level_name = f"[{record.levelname}]"
        time = f"[{self.formatTime(record)}]"
        process = f"[{record.process}]"
        message = f"{color}{level_name} {time} {process} {record.msg}{colorama.Style.RESET_ALL}"
        return message


BUFF_SIZE = 1024

if __name__ == "__main__":
    home_pos = [180.0, 100.0, 35.0, 80.0]

    current_pos: list[float] = [None] * 4
    desired_pos: list[float] = copy.deepcopy(home_pos)

    logger = logging.getLogger("logger")
    logger.setLevel(logging.INFO)

    # Create a console handler
    console_handler = logging.StreamHandler()
    console_handler.setLevel(logging.INFO)

    # Add the colorful formatter to the handler
    formatter = ColorfulFormatter(
        "%(asctime)s - %(name)s - %(levelname)s - %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
    )
    console_handler.setFormatter(formatter)

    # Add the handler to the logger
    logger.addHandler(console_handler)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind("0.0.0.0", 8000)

    kit = ServoKit(channels=16)

    while True:
        data, addr = sock.recvfrom(1024)
        try:
            body = json.loads(data)

            for i in range(4):
                try:
                    joint_angle = body[i]
                    logger.info(f"Joint {i} angle: {joint_angle}")

                    desired_pos[i] = home_pos[i] + joint_angle * 160 / math.pi
                except KeyError:
                    continue
        except json.JSONDecodeError as e:
            logger.error("JSON Format error")
