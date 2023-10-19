import pyautogui
import serial
import argparse
import time
import logging

class MyControllerMap:
    def __init__(self):
        self.button = {'A': 'L'} # Fast forward (10 seg) pro Youtube

class SerialControllerInterface:
    # Protocolo
    # byte 1 -> Botão 1 (estado - Apertado 1 ou não 0)
    # byte 2 -> EOP - End of Packet -> valor reservado 'X'

    def __init__(self, port, baudrate):
        self.ser = serial.Serial(port, baudrate=baudrate)
        self.mapping = MyControllerMap()
        self.incoming = '0'
        pyautogui.PAUSE = 0  ## remove delay
    
    def update(self):

        # list of dicts with commands info
        commands = [['ctrl', 'left'], 'space', ['ctrl', 'right']]

        ## Sync protocol
        while self.incoming != b'X':
            self.incoming = self.ser.read()
            logging.debug("Received INCOMING: {}".format(self.incoming))

        # for i in range(len(commands)):
        #     data = self.ser.read()
        #     logging.debug("Received DATA: {}".format(data))

        #     if i == 1:
        #         if data == b'1':
        #             logging.info("KEYDOWN {}".format(commands[i]))
        #             pyautogui.press(commands[i])
        #         # elif data == b'0':
        #         #     logging.info("KEYUP {}".format(commands[i]))
        #         #     pyautogui.keyUp(commands[i])
        #     else:
        #         if data == b'1':
        #             logging.info("KEYDOWN {}".format(commands[i]))
        #             pyautogui.hotkey(commands[i][0], commands[i][1]) # Press the Ctrl-C hotkey combination.
                    
        data = self.ser.read()
        # convert data to binary with 8 bits
        #data = bin(int.from_bytes(data, byteorder='big'))
        # data = bin(int.from_bytes(data, byteorder='big'))
        logging.debug("Received DATA: {}".format(data))

        primeiro_byte = data[0]
        primeiro_bit = primeiro_byte & 1

        if primeiro_bit == 1:
            logging.debug("bt0")

        

        # if data == b'1':
        #     logging.info("KEYDOWN A")
        #     pyautogui.keyDown(self.mapping.button['A'])
        # elif data == b'0':
        #     logging.info("KEYUP A")
        #     pyautogui.keyUp(self.mapping.button['A'])
        
        self.incoming = self.ser.read()


class DummyControllerInterface:
    def __init__(self):
        self.mapping = MyControllerMap()

    def update(self):
        pyautogui.keyDown(self.mapping.button['A'])
        time.sleep(0.1)
        pyautogui.keyUp(self.mapping.button['A'])
        logging.info("[Dummy] Pressed A button")
        time.sleep(1)


if __name__ == '__main__':
    interfaces = ['dummy', 'serial']
    argparse = argparse.ArgumentParser()
    argparse.add_argument('serial_port', type=str)
    argparse.add_argument('-b', '--baudrate', type=int, default=9600)
    argparse.add_argument('-c', '--controller_interface', type=str, default='serial', choices=interfaces)
    argparse.add_argument('-d', '--debug', default=False, action='store_true')
    args = argparse.parse_args()
    if args.debug:
        logging.basicConfig(level=logging.DEBUG)

    print("Connection to {} using {} interface ({})".format(args.serial_port, args.controller_interface, args.baudrate))
    if args.controller_interface == 'dummy':
        controller = DummyControllerInterface()
    else:
        controller = SerialControllerInterface(port=args.serial_port, baudrate=args.baudrate)

    while True:
        controller.update()
