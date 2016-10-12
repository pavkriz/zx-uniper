@Grab(group='com.fazecast', module='jSerialComm', version='1.3.11')

import com.fazecast.jSerialComm.SerialPort;

class PcTool {

    static SerialPort comPort;

    static final int CMD_ALIVE = 1;
    static final int CMD_IDE = 3;
    static final int FRAME_OVERHEAD = 4;
    static final int MIN_FRAME_SIZE = 4;
    static final int FRAME_CMD_INDEX = 1;
    static final int FRAME_LEN_INDEX = 2;
    static final int CDC_TX_BLOCK = 256;

    static byte[] commandBuffer = new int[2000];
    static int commandBufferLen = 0;
    static boolean waitFirstAlive;
    static int missedAlives;


    //Offset(h) 00 01 02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F
//
//00000000  7A 42 FF 3F 37 C8 10 00 00 00 00 00 3F 00 00 00  zB.?7.......?...
//00000010  00 00 00 00 20 20 20 20 57 20 2D 44 4D 57 4E 41  ....    W -DMWNA
//00000020  33 4B 33 30 39 34 33 38 00 00 00 10 32 00 30 32  3K309438....2.02
//00000030  30 2E 4B 30 30 32 44 57 20 43 44 57 35 32 30 30  0.K002DW CDW5200
//00000040  42 42 35 2D 52 35 41 44 20 30 20 20 20 20 20 20  BB5-R5AD 0
//00000050  20 20 20 20 20 20 20 20 20 20 20 20 20 20 10 80                .€
//00000060  00 00 00 2F 01 40 00 00 00 00 07 00 DD 10 0F 00  .../.@......Ý...
//00000070  FF 00 0D F6 FB 00 10 01 FF FF FF 0F 00 00 07 04  ...ö............
//00000080  03 00 78 00 78 00 78 00 78 00 00 00 00 00 00 00  ..x.x.x.x.......
//00000090  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00  ................
//000000A0  FE 00 00 00 6B 74 01 7F 33 46 69 74 01 3E 23 46  ....kt..3Fit.>#F
//000000B0  3F 00 00 00 00 00 00 00 FE FF 0D 60 80 80 08 00  ?..........`€€..
//000000C0  00 00 00 00 A0 86 01 00 70 59 1C 1D 00 00 00 00  .... †..pY......
//000000D0  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00  ................
//000000E0  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00  ................
//000000F0  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00  ................
//00000100  01 00 00 00 00 00 00 00 00 00 76 12 00 00 00 00  ..........v.....
//00000110  00 00 00 00 00 00 00 00 00 00 00 00 04 00 00 00  ................
//00000120  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00  ................
//00000130  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00  ................
//00000140  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00  ................
//00000150  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00  ................
//00000160  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00  ................
//00000170  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00  ................
//00000180  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00  ................
//00000190  00 00 00 00 00 00 00 00 00 00 00 00 3F 00 00 00  ............?...
//000001A0  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00  ................
//000001B0  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00  ................
//000001C0  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00  ................
//000001D0  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00  ................
//000001E0  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00  ................
//000001F0  00 00 00 00 00 00 00 00 00 00 00 00 00 00 A5 B2  ................

    static byte[] ideIdentifyBuffer = [
        // WD2500BB ATA_CMD_IDENTIFY  0xEC command output buffer, lowbyte, highbyte
        0x7A,0x42,0xFF,0x3F,0x37,0xC8,0x10,0x00,0x00,0x00,0x00,0x00,0x3F,0x00,0x00,0x00,
        0x00,0x00,0x00,0x00,0x20,0x20,0x20,0x20,0x57,0x20,0x2D,0x44,0x4D,0x57,0x4E,0x41,
        0x33,0x4B,0x33,0x30,0x39,0x34,0x33,0x38,0x00,0x00,0x00,0x10,0x32,0x00,0x30,0x32,
        0x30,0x2E,0x4B,0x30,0x30,0x32,0x44,0x57,0x20,0x43,0x44,0x57,0x35,0x32,0x30,0x30,
        0x42,0x42,0x35,0x2D,0x52,0x35,0x41,0x44,0x20,0x30,0x20,0x20,0x20,0x20,0x20,0x20,
        0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x10,0x80,
        0x00,0x00,0x00,0x2F,0x01,0x40,0x00,0x00,0x00,0x00,0x07,0x00,0xDD,0x10,0x0F,0x00,
        0xFF,0x00,0x0D,0xF6,0xFB,0x00,0x10,0x01,0xFF,0xFF,0xFF,0x0F,0x00,0x00,0x07,0x04,
        0x03,0x00,0x78,0x00,0x78,0x00,0x78,0x00,0x78,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
        0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
        0xFE,0x00,0x00,0x00,0x6B,0x74,0x01,0x7F,0x33,0x46,0x69,0x74,0x01,0x3E,0x23,0x46,
        0x3F,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFE,0xFF,0x0D,0x60,0x80,0x80,0x08,0x00,
        0x00,0x00,0x00,0x00,0xA0,0x86,0x01,0x00,0x70,0x59,0x1C,0x1D,0x00,0x00,0x00,0x00,
        0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
        0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
        0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
        0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x76,0x12,0x00,0x00,0x00,0x00,
        0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x04,0x00,0x00,0x00,
        0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
        0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
        0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
        0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
        0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
        0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
        0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
        0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x3F,0x00,0x00,0x00,
        0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
        0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
        0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
        0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
        0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
        0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xA5,0xB2
    ];

    static byte[] hdfImage = new File('C:\\data\\Elektronika\\Speccy\\esxdos\\divideo.hdf').readBytes();

    final protected static char[] hexArray = "0123456789ABCDEF".toCharArray();
    public static String bytesToHex(byte[] bytes) {
        char[] hexChars = new char[bytes.length * 2];
        for ( int j = 0; j < bytes.length; j++ ) {
            int v = bytes[j] & 0xFF;
            hexChars[j * 2] = hexArray[v >>> 4];
            hexChars[j * 2 + 1] = hexArray[v & 0x0F];
        }
        return new String(hexChars);
    }

    public static void printBytesToHexBy16(byte[] bytes, int offset, int len) {
        for ( int j = 0; j < len; j++ ) {
            if (j % 16 == 0) {
                println();
                print Integer.toHexString(j) + ":   "
            };
            int v = bytes[j + offset] & 0xFF;
            print hexArray[v >>> 4];
            print hexArray[v & 0x0F];
            if (j%8 == 0 && j%16 != 0) print "|" else print " ";
        }
        println();
    }

    static def tryProcessCommands() {
        while (commandBufferLen > 0) {
            while (commandBuffer[0] != 0 && commandBufferLen > 0) discardBytes(1);
            if (commandBufferLen < MIN_FRAME_SIZE) return; // not a complete packet yet
            int cmdPayloadLen = commandBuffer[FRAME_LEN_INDEX];
            int cmdPacketLen = cmdPayloadLen + FRAME_OVERHEAD;
            if (commandBufferLen < cmdPacketLen) return; // not a complete packet yet
            if (commandBuffer[cmdPacketLen - 1] != 0) {
                // framing error, discard one byte and repeat
                discardBytes(1);
            } else {
                // framing OK, we have the whole command in the buffer
                processCommand();
            }
        }
    }

    static def processCommand() {
        int cmdPayloadLen = commandBuffer[2];
        byte[] buf = new byte[cmdPayloadLen];
        System.arraycopy(commandBuffer, 3, buf, 0, cmdPayloadLen);
        if (commandBuffer[FRAME_CMD_INDEX] != CMD_ALIVE || waitFirstAlive) {
            println "CMD:" + commandBuffer[1] + " LEN:" + cmdPayloadLen + " PAYLOAD: " + bytesToHex(buf);
        }
        if (commandBuffer[FRAME_CMD_INDEX] == CMD_ALIVE) {
            waitFirstAlive = false;
            missedAlives = 0;
        } else if (commandBuffer[FRAME_CMD_INDEX] == CMD_IDE) {
            processIdeCommand(buf);
        }
        // discard the command from the buffer
        discardBytes(commandBuffer[FRAME_LEN_INDEX] + FRAME_OVERHEAD);
    }

    static def processIdeCommand(byte[] payload) {
        if (Byte.toUnsignedInt(payload[0]) == 0xEC) {
            // ATA IDENTIFY CMD
            println "ATA IDENTIFY CMD";
            //sendBytes8Each(ideIdentifyBuffer);
            //sendBytesBlock(hdfImage, 0x16, 512); // send IDE/ATA identification data, as returned by ATA command 0xEC from a real drive
            sendBytesBlock(ideIdentifyBuffer, 0, 512);
            println "DONE";
        } else if (Byte.toUnsignedInt(payload[0]) == 0x20) {
            // ATA READ BLOCK
            int lbaBlock = Byte.toUnsignedInt(payload[1]) + Byte.toUnsignedInt(payload[2])*256 + Byte.toUnsignedInt(payload[3])*256*256 + (Byte.toUnsignedInt(payload[4]) & 0x0f)*256*256*256;
            println "ATA READ BLOCK CMD LBA=" + lbaBlock;
            sendBytesBlock(hdfImage, 0x216 + lbaBlock*512, 512);
        }
    }

    static def sendBytesBlock(byte[] buffer, int offset, int len) {
        println "Sending block:";
        printBytesToHexBy16(buffer, offset, len);
        int parts = len / CDC_TX_BLOCK;
        byte[] buf = new byte[CDC_TX_BLOCK];
        for (int i = 0; i < parts; i++) {
            print "Sending part " + i + " ...";
            System.arraycopy(buffer, i*CDC_TX_BLOCK + offset, buf, 0, CDC_TX_BLOCK);
            comPort.writeBytes(buf, CDC_TX_BLOCK);
            println " ...sent.";
        }
        int rest = len - CDC_TX_BLOCK*parts;
        if (rest > 0) {
            print "Sending rest part ...";
            System.arraycopy(buffer, parts*CDC_TX_BLOCK + offset, buf, 0, rest);
            comPort.writeBytes(buf, rest);
            println " ...sent.";
        }
        println "block sent.";
    }

    static def discardBytes(int numberOfBytes) {
        System.arraycopy(commandBuffer, numberOfBytes, commandBuffer, 0, commandBuffer.length - numberOfBytes);
        commandBufferLen = commandBufferLen - numberOfBytes;
    }

    static def portExists(String portName) {
        SerialPort p = SerialPort.getCommPort(portName);
        println p;
    }

    static main(arguments) {
        //printBytesToHexBy16(ideIdentifyBuffer, 0, 512);

        while (true) {
            println "Waiting for the COM port"
            while (SerialPort.getCommPorts().size() == 0) {
                Thread.sleep(1);
            }
            waitFirstAlive = true;
            comPort = SerialPort.getCommPorts()[0]; // get first serial ports available
            //comPort.setComPortParameters(115200, 8, SerialPort.ONE_STOP_BIT, SerialPort.NO_PARITY);
            println "COM port found, connecting: " + comPort.getDescriptivePortName(); // print port's description
            comPort.openPort();
            //comPort.setComPortParameters(115200, 8, SerialPort.ONE_STOP_BIT, SerialPort.NO_PARITY);
            missedAlives = 0;
            try {
                while (true) {
                    while (comPort.bytesAvailable() <= 0) {
                        Thread.sleep(1);
                        missedAlives++;
                        if (missedAlives > 2000) throw new MissingAlives();
                    }

                    if (comPort.bytesAvailable() > 0) {
                        byte[] readBuffer = new byte[comPort.bytesAvailable()];
                        int numRead = comPort.readBytes(readBuffer, readBuffer.length);
                        //System.out.println("Read " + numRead + " bytes.");
                        print "."
                        System.arraycopy(readBuffer, 0, commandBuffer, commandBufferLen, numRead);
                        commandBufferLen += numRead;
                        tryProcessCommands();
                        //sendBytesBlock(ideIdentifyBuffer, 0, 512);
                    } else {
                        println "Port closed";
                    }
                }
            } catch (MissingAlives e) {
                println "Missing alives, port has been probably lost, trying to reconnect";
                commandBufferLen = 0; // empty buffer
            } catch (Exception e) {
                e.printStackTrace();
            }
            comPort.closePort();
        }
        //
        //CommPortIdentifier cp = CommPortIdentifier.getPortIdentifier("COM12");
        //SerialPort port = cp.openPort("SendTape", 100);
        //if (port == null) {
        //    System.out.println("Unable to open the com port.");
        //    System.exit(1);
        //}
    }

    static class MissingAlives extends RuntimeException {}

}