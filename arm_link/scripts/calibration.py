#!/usr/bin/env python3

from serial_comm import SerialComm


def main():
    serial = SerialComm()
    serial.open_connection()
    serial.send_message("000000" + "000000" + "000000" + "000000\n")

    while (1):
        serial.send_message("000000" + "000000" + "000000" + input("Position command: ") + "\n")


if __name__ == "__main__":
    main()
