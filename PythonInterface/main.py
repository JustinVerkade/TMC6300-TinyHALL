import serial

def main():
    comport = serial.Serial("COM6", 115200)
    package = b"\x00\x00\x00\x00\x00\x00\x00\x00\x00"
    comport.write(package)
    ret = comport.read(9)
    print(ret)

if __name__ == "__main__":
    main()