# This configures the MPF server stuff.  GPIB support is commented out, but can
# added by simply deleting the "# !GPIB! #" comments.

###############################################################################
# Initialize IP carrier
# ipacAddCarrier(ipac_carrier_t *pcarrier, char *cardParams)
#   pcarrier   - pointer to carrier driver structure
#   cardParams - carrier-specific init parameters

# For MVME162 and MVME172 CPU boards
carrier = "ipac"
ipacAddCarrier(&ipmv162, "A:l=3,3 m=0xe0000000,64;B:l=3,3 m=0xe0010000,64;C:l=3,3 m=0xe0020000,64;D:l=3,3 m=0xe0030000,64")

# For SBS VIPC616-01 IP carrier board.  A32 and A24 examples follow:
#carrier = "VIPC616_01"

# - A32 addressing.
# - default I/O Base Address (0x6000).
# - IP modules occupy 32MB of memory from 0x9000 0000 to 0x91FF FFFF.
# - IP module A at 0x90000000,
#      module B at 0x90800000,
#      module C at 0x91000000,
#      module D at 0x91800000
#ipacAddCarrier(&vipc616_01,"6000,90000000")

# - A24 addressing.
# - default I/O Base Address (0x6000).
# - default IP module base address at 0x00D0 0000
#   occupies 512 bytes of memory from 0x00D0 0000 to 0x00D0 01FF.
# - IP module A at 0x0x00D0 007F,
#      module B at 0x0x00D0 0100,
#      module C at 0x0x00D0 017F,
#      module D at 0x0x00D0 01FF
#ipacAddCarrier(&vipc616_01, "0x6000,D00000,128")

initIpacCarrier(carrier, 0)
#ipacReport(2)

###############################################################################
# Initialize Octal UART module
#initOctalUART("moduleName","carrierName","carrierSite",nports,intVec)
initOctalUART("octalUart0",carrier,"IP_a",8,100)

# initOctalUARTPort(char* portName,char* moduleName,int port,int baud,
#                   char* parity,int stop_bits,int bits_char,char* flow_control)
# 'baud' is the baud rate. 1200, 2400, 4800, 9600, 19200, 38400
# 'parity' is "E" for even, "O" for odd, "N" for none.
# 'bits_per_character' = {5,6,7,8}
# 'stop_bits' = {1,2}
# 'flow_control' is "N" for none, "H" for hardware
# Port 0 is Generic Serial Record

initOctalUARTPort("UART[0]","octalUart0",0, 9600,"N",1,8,"N")
initSerialServer("a-Serial[0]","UART[0]",1000,20,"\r",1)

