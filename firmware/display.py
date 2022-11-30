import busio
import struct

class Display(object):
    """Display"""
    def __init__(self, uart_tx_pin, uart_rx_pin):
        """Display
        :param ~microcontroller.Pin uart_tx_pin: UART TX pin that connects to display
        :param ~microcontroller.Pin uart_tx_pin: UART RX pin that connects to display
        """

        # configure UART for communications with display
        self.__uart = busio.UART(uart_tx_pin, uart_rx_pin, baudrate=19200, timeout=0.005)

        # init variables
        self.__read_and_pack__state = 0
        self.__read_and_pack__len = 0
        self.__read_and_pack__cnt = 0

    #every 50ms, read and process UART data
    def process_data(self):
        self.__read_and_pack()

        buf = bytearray(0) #just empty
        self.__pack_and_send(buf, FrameType.ALIVE)

    # code taken from:
    # https://github.com/LacobusVentura/MODBUS-CRC16
    def __crc16(self, data):
    #     table = [ 
    #         0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
    #         0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
    #         0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
    #         0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
    #         0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
    #         0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
    #         0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
    #         0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
    #         0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
    #         0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
    #         0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
    #         0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
    #         0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
    #         0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
    #         0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
    #         0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
    #         0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
    #         0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
    #         0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
    #         0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
    #         0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
    #         0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
    #         0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
    #         0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
    #         0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
    #         0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
    #         0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
    #         0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
    #         0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
    #         0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
    #         0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
    #         0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040
    #     ]

    #     xor = 0
    #     crc = 0xFFFF
    #     for byte in data:
    #         xor = (byte ^ crc) & 0xff
    #         crc >>= 8
    #         crc ^= table[xor]
    #         crc &= 0xFFFF # important, crc must stay 16bits all the way through

        crc = 0xffff
        for _data in data:

            crc = crc ^ (_data & 0xffff)

            for i in range(8):
                if crc & 1:
                    crc = (crc >> 1) ^ 0xA001
                else:
                    crc = crc >> 1
            
        crc = crc & 0xffff

        # print(" ")
        # print(",".join(["0x{:02X}".format(i) for i in data]))
        # print(crc)

        return crc

    def __pack_and_send(self, buf, frame_type):
        #start byte + len + frame type + data + CRC 16 bits
        len_buf = len(buf)
        _len = len_buf + 3 #1 type of frame + data bytes + 2 CRC bytes 
        len_package = _len + 2 # len + 1 start byte

        data_array = bytearray(len_package)
        data_array[0] = 0x43; # start byte
        data_array[1] = _len #frame type + data + CRC 16 bits
        data_array[2] = frame_type
        
        if len_buf > 0:
            data_array[3: 3 + len_buf] = buf # copy data
                
        crc = self.__crc16(data_array[0: len_package - 2]) #CRC of full package minus last 2 bytes of CRC16
        data_array[len_package - 2] = crc & 0x00ff
        data_array[len_package - 1] = (crc & 0xff00) >> 8

        # send packet to UART
        self.__uart.write(data_array)

    def __read_and_pack(self):
        rx_array = self.__uart.read()

        # print(",".join(["0x{:02X}".format(i) for i in rx_array]))
        
        data_array = bytearray(256)
        for data in rx_array:
            if self.__read_and_pack__state == 0:
                if (data == 0x59):
                    data_array[0] = data
                    self.__read_and_pack__state = 1
                else:
                    self.__read_and_pack__state = 0

            elif self.__read_and_pack__state == 1:
                data_array[1] = data
                self.__read_and_pack__len = data
                self.__read_and_pack__state = 2

            elif self.__read_and_pack__state == 2:
                data_array[self.__read_and_pack__cnt  + 2] = data
                self.__read_and_pack__cnt += 1

                if self.__read_and_pack__cnt >= self.__read_and_pack__len:
                    self.__read_and_pack__cnt = 0
                    self.__read_and_pack__state = 0

                    # calculate the CRC of the package
                    crc = self.__crc16(data_array[0: self.__read_and_pack__len])
                    crc_original = struct.unpack_from('<H', data_array, self.__read_and_pack__len)[0]
                    
                    if crc == crc_original:
                        print("CRC rx packet ok")

class FrameType():
    ALIVE = 0
    STATUS = 1
    PERIODIC = 2
    CONFIGURATIONS = 3
    FIRMWARE_VERSION = 4