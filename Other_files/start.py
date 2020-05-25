#!/usr/bin/python
# -*- coding: utf-8 -*-

import serial
import time
import struct

def serial_send(msg):
    ser = serial.Serial("/dev/ttyS1")
    ser.stopbits = 2
    tmpBuffer = bytearray.fromhex(msg)
    ser.write(tmpBuffer)
    ser.close()

#сюда необходимо ввести строку для отображения
text = "ПРИМЕР СТРОКИ"

#скорость бегущей строки, чем меньше значение, тем быстрее бежит текст
time_scroll = 0.01

rus5x7 = [0x00, 0x00, 0x00, 0x00, 0x00, 0x00,      # Code for char  
0x00, 0x00, 0x5F, 0x00, 0x00, 0x00,      # Code for char !
0x00, 0x03, 0x00, 0x03, 0x00, 0x00,      # Code for char "
0x14, 0x3E, 0x14, 0x3E, 0x14, 0x00,      # Code for char #
0x00, 0x2E, 0x7F, 0x3A, 0x00, 0x00,      # Code for char $
0x03, 0x73, 0x1C, 0x67, 0x60, 0x00,      # Code for char %
0x38, 0x4F, 0x5D, 0x37, 0x40, 0x00,      # Code for char &
0x00, 0x00, 0x0B, 0x07, 0x00, 0x00,      # Code for char '
0x00, 0x3E, 0x41, 0x00, 0x00, 0x00,      # Code for char (
0x00, 0x41, 0x3E, 0x00, 0x00, 0x00,      # Code for char )
0x00, 0x02, 0x07, 0x02, 0x00, 0x00,      # Code for char *
0x08, 0x08, 0x3E, 0x08, 0x08, 0x00,      # Code for char +
0x00, 0x00, 0x60, 0x00, 0x00, 0x00,      # Code for char ,
0x08, 0x08, 0x08, 0x08, 0x08, 0x00,      # Code for char -
0x00, 0x00, 0x20, 0x00, 0x00, 0x00,      # Code for char .
0x00, 0x70, 0x1C, 0x07, 0x00, 0x00,      # Code for char /
0x00, 0x7F, 0x41, 0x41, 0x7F, 0x00,      # Code for char 0
0x00, 0x00, 0x42, 0x7F, 0x40, 0x00,      # Code for char 1
0x00, 0x66, 0x51, 0x51, 0x4E, 0x00,      # Code for char 2
0x00, 0x22, 0x49, 0x49, 0x36, 0x00,      # Code for char 3
0x00, 0x18, 0x14, 0x12, 0x7F, 0x00,      # Code for char 4
0x00, 0x4F, 0x49, 0x49, 0x79, 0x00,      # Code for char 5
0x00, 0x3C, 0x4A, 0x49, 0x31, 0x00,      # Code for char 6
0x00, 0x01, 0x01, 0x71, 0x0F, 0x00,      # Code for char 7
0x00, 0x36, 0x49, 0x49, 0x36, 0x00,      # Code for char 8
0x00, 0x06, 0x49, 0x49, 0x3E, 0x00,      # Code for char 9
0x00, 0x00, 0x36, 0x36, 0x00, 0x00,      # Code for char :
0x00, 0x00, 0x76, 0x36, 0x00, 0x00,      # Code for char ;
0x08, 0x14, 0x22, 0x41, 0x00, 0x00,      # Code for char <
0x00, 0x14, 0x14, 0x14, 0x14, 0x00,      # Code for char =
0x41, 0x22, 0x14, 0x08, 0x00, 0x00,      # Code for char >
0x00, 0x06, 0x01, 0x59, 0x0E, 0x00,      # Code for char ?
0x7F, 0x5D, 0x55, 0x5D, 0x51, 0x1F,      # Code for char @
0x00, 0x7E, 0x11, 0x11, 0x7E, 0x00,      # Code for char A
0x00, 0x7F, 0x49, 0x49, 0x36, 0x00,      # Code for char B
0x00, 0x3E, 0x41, 0x41, 0x22, 0x00,      # Code for char C
0x00, 0x7F, 0x41, 0x41, 0x3E, 0x00,      # Code for char D
0x00, 0x7F, 0x49, 0x49, 0x41, 0x00,      # Code for char E
0x00, 0x7F, 0x09, 0x09, 0x01, 0x00,      # Code for char F
0x00, 0x3E, 0x41, 0x51, 0x32, 0x10,      # Code for char G
0x00, 0x7F, 0x08, 0x08, 0x7F, 0x00,      # Code for char H
0x00, 0x41, 0x7F, 0x41, 0x00, 0x00,      # Code for char I
0x00, 0x20, 0x41, 0x7F, 0x01, 0x00,      # Code for char J
0x00, 0x7F, 0x08, 0x14, 0x63, 0x00,      # Code for char K
0x00, 0x7F, 0x40, 0x40, 0x60, 0x00,      # Code for char L
0x7F, 0x0E, 0x3C, 0x0E, 0x7F, 0x00,      # Code for char M
0x7F, 0x06, 0x1C, 0x30, 0x7F, 0x00,      # Code for char N
0x00, 0x3E, 0x41, 0x41, 0x3E, 0x00,      # Code for char O
0x00, 0x7F, 0x11, 0x11, 0x0E, 0x00,      # Code for char P
0x00, 0x3E, 0x41, 0x51, 0x3E, 0x40,      # Code for char Q
0x00, 0x7F, 0x11, 0x11, 0x6E, 0x00,      # Code for char R
0x00, 0x46, 0x49, 0x49, 0x31, 0x00,      # Code for char S
0x01, 0x01, 0x7F, 0x01, 0x01, 0x00,      # Code for char T
0x00, 0x3F, 0x40, 0x40, 0x3F, 0x00,      # Code for char U
0x07, 0x1C, 0x70, 0x70, 0x1C, 0x07,      # Code for char V
0x3F, 0x60, 0x38, 0x60, 0x3F, 0x00,      # Code for char W
0x77, 0x14, 0x08, 0x14, 0x77, 0x00,      # Code for char X
0x07, 0x08, 0x70, 0x08, 0x07, 0x00,      # Code for char Y
0x00, 0x61, 0x59, 0x4D, 0x43, 0x00,      # Code for char Z
0x00, 0x7F, 0x41, 0x00, 0x00, 0x00,      # Code for char [
0x00, 0x03, 0x06, 0x1C, 0x70, 0x00,      # Code for char BackSlash
0x00, 0x41, 0x7F, 0x00, 0x00, 0x00,      # Code for char ]
0x00, 0x02, 0x01, 0x02, 0x00, 0x00,      # Code for char ^
0x00, 0x40, 0x40, 0x40, 0x40, 0x00,      # Code for char _
0x00, 0x07, 0x0B, 0x00, 0x00, 0x00,      # Code for char `
0x00, 0x7E, 0x11, 0x11, 0x7E, 0x00,      # Code for char A
0x00, 0x7F, 0x49, 0x49, 0x36, 0x00,      # Code for char B
0x00, 0x3E, 0x41, 0x41, 0x22, 0x00,      # Code for char C
0x00, 0x7F, 0x41, 0x41, 0x3E, 0x00,      # Code for char D
0x00, 0x7F, 0x49, 0x49, 0x41, 0x00,      # Code for char E
0x00, 0x7F, 0x09, 0x09, 0x01, 0x00,      # Code for char F
0x00, 0x3E, 0x41, 0x51, 0x32, 0x10,      # Code for char G
0x00, 0x7F, 0x08, 0x08, 0x7F, 0x00,      # Code for char H
0x00, 0x41, 0x7F, 0x41, 0x00, 0x00,      # Code for char I
0x00, 0x20, 0x41, 0x7F, 0x01, 0x00,      # Code for char J
0x00, 0x7F, 0x08, 0x14, 0x63, 0x00,      # Code for char K
0x00, 0x7F, 0x40, 0x40, 0x60, 0x00,      # Code for char L
0x7F, 0x0E, 0x3C, 0x0E, 0x7F, 0x00,      # Code for char M
0x7F, 0x06, 0x1C, 0x30, 0x7F, 0x00,      # Code for char N
0x00, 0x3E, 0x41, 0x41, 0x3E, 0x00,      # Code for char O
0x00, 0x7F, 0x11, 0x11, 0x0E, 0x00,      # Code for char P
0x00, 0x3E, 0x41, 0x51, 0x3E, 0x40,      # Code for char Q
0x00, 0x7F, 0x11, 0x11, 0x6E, 0x00,      # Code for char R
0x00, 0x46, 0x49, 0x49, 0x31, 0x00,      # Code for char S
0x01, 0x01, 0x7F, 0x01, 0x01, 0x00,      # Code for char T
0x00, 0x3F, 0x40, 0x40, 0x3F, 0x00,      # Code for char U
0x07, 0x1C, 0x70, 0x70, 0x1C, 0x07,      # Code for char V
0x3F, 0x60, 0x38, 0x60, 0x3F, 0x00,      # Code for char W
0x77, 0x14, 0x08, 0x14, 0x77, 0x00,      # Code for char X
0x07, 0x08, 0x70, 0x08, 0x07, 0x00,      # Code for char Y
0x00, 0x61, 0x59, 0x4D, 0x43, 0x00,      # Code for char Z
0x00, 0x08, 0x77, 0x41, 0x00, 0x00,      # Code for char {
0x00, 0x00, 0x7F, 0x00, 0x00, 0x00,      # Code for char |
0x00, 0x00, 0x41, 0x77, 0x08, 0x00,      # Code for char }
0x1C, 0x04, 0x1C, 0x10, 0x1C, 0x00,      # Code for char ~
0x60, 0x18, 0x06, 0x18, 0x60, 0x00,
0x1E, 0x00, 0x00, 0x00, 0x00, 0x00,      # Code for char 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00,      # Code for char €
0x00, 0x00, 0x00, 0x00, 0x00, 0x00,      # Code for char 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00,      # Code for char ‚
0x00, 0x00, 0x00, 0x00, 0x00, 0x00,      # Code for char ƒ
0x00, 0x00, 0x00, 0x00, 0x00, 0x00,      # Code for char „
0x00, 0x00, 0x00, 0x00, 0x00, 0x00,      # Code for char …
0x00, 0x00, 0x00, 0x00, 0x00, 0x00,      # Code for char †
0x00, 0x00, 0x00, 0x00, 0x00, 0x00,      # Code for char ‡
0x00, 0x00, 0x00, 0x00, 0x00, 0x00,      # Code for char ˆ
0x00, 0x00, 0x00, 0x00, 0x00, 0x00,      # Code for char ‰
0x00, 0x00, 0x00, 0x00, 0x00, 0x00,      # Code for char Š
0x00, 0x00, 0x00, 0x00, 0x00, 0x00,      # Code for char ‹
0x00, 0x00, 0x00, 0x00, 0x00, 0x00,      # Code for char Œ
0x00, 0x00, 0x00, 0x00, 0x00, 0x00,      # Code for char 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00,      # Code for char Ž
0x00, 0x00, 0x00, 0x00, 0x00, 0x00,      # Code for char 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00,      # Code for char 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00,      # Code for char ?
0x00, 0x00, 0x00, 0x00, 0x00, 0x00,      # Code for char ’
0x00, 0x00, 0x00, 0x00, 0x00, 0x00,      # Code for char “
0x00, 0x00, 0x00, 0x00, 0x00, 0x00,      # Code for char ”
0x00, 0x00, 0x00, 0x00, 0x00, 0x00,      # Code for char •
0x00, 0x08, 0x08, 0x08, 0x08, 0x00,      # Code for char –
0x08, 0x08, 0x08, 0x08, 0x08, 0x08,      # Code for char —
0x00, 0x00, 0x00, 0x00, 0x00, 0x00,      # Code for char ˜
0x00, 0x00, 0x00, 0x00, 0x00, 0x00,      # Code for char ™
0x00, 0x00, 0x00, 0x00, 0x00, 0x00,      # Code for char š
0x00, 0x00, 0x00, 0x00, 0x00, 0x00,      # Code for char ›
0x00, 0x00, 0x00, 0x00, 0x00, 0x00,      # Code for char œ
0x00, 0x00, 0x00, 0x00, 0x00, 0x00,      # Code for char 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00,      # Code for char ž
0x00, 0x00, 0x00, 0x00, 0x00, 0x00,      # Code for char Ÿ
0x00, 0x00, 0x00, 0x00, 0x00, 0x00,      # Code for char  
0x00, 0x1E, 0x00, 0x00, 0x00, 0x00,      # Code for char ¡
0x18, 0x1C, 0x14, 0x00, 0x00, 0x00,      # Code for char ¢
0x18, 0x1E, 0x02, 0x00, 0x00, 0x00,      # Code for char £
0x1C, 0x14, 0x1C, 0x00, 0x00, 0x00,      # Code for char ¤
0x12, 0x1C, 0x12, 0x00, 0x00, 0x00,      # Code for char ¥
0x00, 0x36, 0x00, 0x00, 0x00, 0x00,      # Code for char ¦
0x2C, 0x36, 0x2A, 0x00, 0x00, 0x00,      # Code for char §
0x7E, 0x53, 0x52, 0x43, 0x42, 0x00,      # Code for char ¨
0x3E, 0x5D, 0x55, 0x55, 0x3E, 0x00,      # Code for char ©
0x04, 0x0E, 0x00, 0x00, 0x00, 0x00,      # Code for char ª
0x08, 0x14, 0x2A, 0x55, 0x22, 0x41,      # Code for char «
0x00, 0x02, 0x02, 0x02, 0x06, 0x00,      # Code for char ¬
0x08, 0x08, 0x00, 0x00, 0x00, 0x00,      # Code for char ­
0x1C, 0x22, 0x2E, 0x12, 0x1C, 0x00,      # Code for char ®
0x01, 0x01, 0x01, 0x00, 0x00, 0x00,      # Code for char ¯
0x00, 0x07, 0x05, 0x07, 0x00, 0x00,      # Code for char °
0x00, 0x24, 0x2E, 0x24, 0x00, 0x00,      # Code for char ±
0x08, 0x0E, 0x00, 0x00, 0x00, 0x00,      # Code for char ²
0x08, 0x0E, 0x00, 0x00, 0x00, 0x00,      # Code for char ³
0x00, 0x02, 0x00, 0x00, 0x00, 0x00,      # Code for char ´
0x3C, 0x10, 0x1C, 0x00, 0x00, 0x00,      # Code for char µ
0x04, 0x3E, 0x3E, 0x00, 0x00, 0x00,      # Code for char ¶
0x00, 0x08, 0x00, 0x00, 0x00, 0x00,      # Code for char ·
0x00, 0x7D, 0x54, 0x45, 0x00, 0x00,      # Code for char ¸
0x7F, 0x06, 0x1C, 0x30, 0x7F, 0x00,      # Code for char ¹
0x04, 0x0E, 0x00, 0x00, 0x00, 0x00,      # Code for char º
0x41, 0x22, 0x55, 0x2A, 0x14, 0x08,      # Code for char »
0x00, 0x0E, 0x0C, 0x0C, 0x00, 0x00,      # Code for char ¼
0x00, 0x0E, 0x04, 0x00, 0x1C, 0x00,      # Code for char ½
0x00, 0x0E, 0x08, 0x0C, 0x00, 0x00,      # Code for char ¾
0x18, 0x16, 0x00, 0x00, 0x00, 0x00,      # Code for char ¿
0x7E, 0x11, 0x11, 0x11, 0x7E, 0x00,      # Code for char À
0x7F, 0x49, 0x49, 0x49, 0x31, 0x00,      # Code for char Á
0x7F, 0x49, 0x49, 0x49, 0x77, 0x00,      # Code for char Â
0x7F, 0x01, 0x01, 0x01, 0x01, 0x00,      # Code for char Ã
0x60, 0x3F, 0x31, 0x3F, 0x60, 0x00,      # Code for char Ä
0x7F, 0x49, 0x49, 0x41, 0x41, 0x00,      # Code for char Å
0x67, 0x18, 0x7F, 0x18, 0x67, 0x00,      # Code for char Æ
0x22, 0x41, 0x49, 0x49, 0x36, 0x00,      # Code for char Ç
0x7F, 0x30, 0x1C, 0x06, 0x7F, 0x00,      # Code for char È
0x7F, 0x30, 0x19, 0x0C, 0x7F, 0x00,      # Code for char É
0x7F, 0x08, 0x14, 0x22, 0x41, 0x00,      # Code for char Ê
0x70, 0x1C, 0x07, 0x1C, 0x70, 0x00,      # Code for char Ë
0x7F, 0x0C, 0x38, 0x0C, 0x7F, 0x00,      # Code for char Ì
0x7F, 0x08, 0x08, 0x08, 0x7F, 0x00,      # Code for char Í
0x3E, 0x41, 0x41, 0x41, 0x3E, 0x00,      # Code for char Î
0x7F, 0x01, 0x01, 0x01, 0x7F, 0x00,      # Code for char Ï
0x7E, 0x11, 0x11, 0x11, 0x0E, 0x00,      # Code for char Ð
0x3E, 0x41, 0x41, 0x41, 0x22, 0x00,      # Code for char Ñ
0x01, 0x01, 0x7F, 0x01, 0x01, 0x00,      # Code for char Ò
0x03, 0x04, 0x08, 0x10, 0x7F, 0x00,      # Code for char Ó
0x1F, 0x11, 0x7F, 0x11, 0x1F, 0x00,      # Code for char Ô
0x77, 0x14, 0x08, 0x14, 0x77, 0x00,      # Code for char Õ
0x3F, 0x20, 0x20, 0x3F, 0x60, 0x00,      # Code for char Ö
0x0F, 0x10, 0x10, 0x10, 0x7F, 0x00,      # Code for char ×
0x7F, 0x40, 0x7F, 0x40, 0x7F, 0x00,      # Code for char ?
0x3F, 0x20, 0x3F, 0x20, 0x7F, 0x60,      # Code for char Ù
0x01, 0x01, 0x7F, 0x48, 0x78, 0x00,      # Code for char Ú
0x7F, 0x48, 0x78, 0x00, 0x7F, 0x00,      # Code for char Û
0x7F, 0x48, 0x78, 0x00, 0x00, 0x00,      # Code for char Ü
0x22, 0x41, 0x49, 0x49, 0x3E, 0x00,      # Code for char Ý
0x7F, 0x08, 0x7F, 0x41, 0x7F, 0x00,      # Code for char Þ
0x46, 0x29, 0x19, 0x09, 0x7F, 0x00,      # Code for char ß
0x14, 0x1E, 0x18, 0x00, 0x00, 0x00,      # Code for char à
0x7E, 0x11, 0x11, 0x11, 0x7E, 0x00,      # Code for char À
0x7F, 0x49, 0x49, 0x49, 0x31, 0x00,      # Code for char Á
0x7F, 0x49, 0x49, 0x49, 0x77, 0x00,      # Code for char Â
0x7F, 0x01, 0x01, 0x01, 0x01, 0x00,      # Code for char Ã
0x60, 0x3F, 0x31, 0x3F, 0x60, 0x00,      # Code for char Ä
0x7F, 0x49, 0x49, 0x41, 0x41, 0x00,      # Code for char Å
0x67, 0x18, 0x7F, 0x18, 0x67, 0x00,      # Code for char Æ
0x22, 0x41, 0x49, 0x49, 0x36, 0x00,      # Code for char Ç
0x7F, 0x30, 0x1C, 0x06, 0x7F, 0x00,      # Code for char È
0x7F, 0x30, 0x19, 0x0C, 0x7F, 0x00,      # Code for char É
0x7F, 0x08, 0x14, 0x22, 0x41, 0x00,      # Code for char Ê
0x70, 0x1C, 0x07, 0x1C, 0x70, 0x00,      # Code for char Ë
0x7F, 0x0C, 0x38, 0x0C, 0x7F, 0x00,      # Code for char Ì
0x7F, 0x08, 0x08, 0x08, 0x7F, 0x00,      # Code for char Í
0x3E, 0x41, 0x41, 0x41, 0x3E, 0x00,      # Code for char Î
0x7F, 0x01, 0x01, 0x01, 0x7F, 0x00,      # Code for char Ï
0x7E, 0x11, 0x11, 0x11, 0x0E, 0x00,      # Code for char Ð
0x3E, 0x41, 0x41, 0x41, 0x22, 0x00,      # Code for char Ñ
0x01, 0x01, 0x7F, 0x01, 0x01, 0x00,      # Code for char Ò
0x03, 0x04, 0x08, 0x10, 0x7F, 0x00,      # Code for char Ó
0x1F, 0x11, 0x7F, 0x11, 0x1F, 0x00,      # Code for char Ô
0x77, 0x14, 0x08, 0x14, 0x77, 0x00,      # Code for char Õ
0x3F, 0x20, 0x20, 0x3F, 0x60, 0x00,      # Code for char Ö
0x0F, 0x10, 0x10, 0x10, 0x7F, 0x00,      # Code for char ×
0x7F, 0x40, 0x7F, 0x40, 0x7F, 0x00,      # Code for char ?
0x3F, 0x20, 0x3F, 0x20, 0x7F, 0x60,      # Code for char Ù
0x01, 0x01, 0x7F, 0x48, 0x78, 0x00,      # Code for char Ú
0x7F, 0x48, 0x78, 0x00, 0x7F, 0x00,      # Code for char Û
0x7F, 0x48, 0x78, 0x00, 0x00, 0x00,      # Code for char Ü
0x22, 0x41, 0x49, 0x49, 0x3E, 0x00,      # Code for char Ý
0x7F, 0x08, 0x7F, 0x41, 0x7F, 0x00,      # Code for char Þ
0x46, 0x29, 0x19, 0x09, 0x7F, 0x00,      # Code for char ß
0x14, 0x1E, 0x18, 0x00, 0x00, 0x00]      # Code for char ÿ

paket = ''
temp = text[::-1]
text = temp
while 1:
    for sym in text:
        x = ord(sym)
        #print(x)
        if (x==1105) or (x==1025):
            x = 169
        if x>= 1040:
            x = x-847
        x= x - 32
        for i in range(6):
            #print(rus5x7[x*6+(5-i)])
            paket = str(hex(rus5x7[x*6+(5-i)])[2:].zfill(2))
            serial_send(paket)
            time.sleep(time_scroll)