# Addressable_Room_Lights

Controls a variable length string of addressable LEDs with the ability to change various illumination settings via the UART Bluefruit bluetooth phone app. All commands with a '#' symbol accept a number between 0-255 in its place unlesss otherwise specified. 

Requires download of the Adafruit_Neopixel library.
Developed on the Adafruit Feather M0 Bluefruit LE.

UART COMMAND LIST
'#':    changes LED brightness (32 bit value)
'r':    resets program to default values
'b':    white illumination
'p':    color wave illumination
'bf#':  speed of fade in for white illumination
'pf#':  speed of fade in for color wave illumination
't#':   changes length of time before auto shut off of LEDs
