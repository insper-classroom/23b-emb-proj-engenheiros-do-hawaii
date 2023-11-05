import pyautogui
import serial
import argparse
import time
import logging
from pycaw.pycaw import AudioUtilities, IAudioEndpointVolume
from ctypes import cast, POINTER
from comtypes import CLSCTX_ALL

devices = AudioUtilities.GetSpeakers()
interface = devices.Activate(IAudioEndpointVolume._iid_, CLSCTX_ALL, None)
volume = cast(interface, POINTER(IAudioEndpointVolume))

volume2decibal = {
            0.00: -51,
            0.05: -40,
            0.10: -30.848,
            0.15: -26,
            0.20: -22.477,
            0.25: -20,
            0.30: -17.111,
            0.35: -15,
            0.40: -13.152,
            0.45: -11,
            0.50: -10.015,
            0.55: -8.5,
            0.60: -7.415,
            0.65: -6,
            0.70: -4.991,
            0.75: -4,
            0.80: -3.26,
            0.85: -2,
            0.90: -1.381,
            0.95: -0.6,
            1:    0
        }

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
        self.Handshake = False
        self.sleep = False
        pyautogui.PAUSE = 0  ## remove delay

    def update(self):
        if self.sleep == False:
            if self.Handshake == False:
                while self.incoming != b'A':
                    self.incoming = self.ser.read()
                    print("Dentro ",self.incoming)
                    logging.debug("Handskahe Not Done: {}".format(self.incoming))
                self.Handshake = True
                self.ser.write(b'A')
                self.incoming = b'X'

            if self.Handshake:
                ## Sync protocol 
                while self.incoming != b'X':
                    self.incoming = self.ser.read()
                    logging.debug("Handshake Done: {}".format(self.incoming))

                            
                datas = self.ser.read()
                data = datas[0]

                #voltar
                if data == 1:
                    print("botao 1")
                    pyautogui.hotkey('ctrl', 'left')
                #pausar
                if data == 2:
                    print("botao 2")
                    pyautogui.press('space')
                #pular
                if data == 3:
                    print("botao 3")
                    pyautogui.hotkey('ctrl', 'right')
                #sleep
                if data == 4:
                    print("botao 4")
                    self.sleep = True

                #volume
                if data == 5:
                    data = self.ser.read()
                    print("data0 ", data[0])
                    vol = data[0]/100
                    print("volume 0 ",vol)

                    if vol < 0:
                        vol = 0
                    if vol > 1:
                        vol = 1

                    print("Volume = ",vol)

                    for value in volume2decibal:
                        if vol - 0.025 < value:
                            vol = volume2decibal[value]
                            break
                    print("Decibels = ",vol)

                    volume.SetMasterVolumeLevel(vol, None)
                
                self.incoming = self.ser.read()
        else:
            while self.incoming != b'X':
                self.incoming = self.ser.read()
                logging.debug("Sleep: {}".format(self.incoming))
      
            datas = self.ser.read()
            data = datas[0]

            #sleep
            if data == 4:
                print("botao")
                self.sleep = False
                self.Handshake = False
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
