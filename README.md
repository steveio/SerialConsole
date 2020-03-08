# SerialConsole
Simple Linux Serial Console Parser written in C for Arduino CLI

Compilation:
gcc ./serial_console.c -o serial_console

Usage: ./serial_console <port> <baud>

./serial_console /dev/ttyUSB0 115200

Log to file:
./serial_console /dev/ttyUSB0 115200 > serial_log.txt
