// Nanode5 Modbus Slave (IO module)
// Implements basic Modbus based on info from http://www.simplymodbus.ca/
// Create a Modbus TCP Slave device for OpenPLC
// https://www.openplcproject.com/
//
// Code is targeted to a Nanode5 (Arduino clone with ENC28J60 ethernet)
// https://wiki.london.hackspace.org.uk/view/Project:Nanode
// Nanode pin8 = CS of ENC28J60. Change accordingly for other boards/shields.
// When programming with FTDI use board settings in Arduino IDE: Arduino Duemilanove/Diecimila
//
// Available I/O on Nanode5 (X=NOT available, Aout = PWM)
//
//      Din Dout  Ain Aout Remarks
// ---+----+----+----+----+------------------------------------
// A0 |    |    |    |  X |
// A1 |    |    |    |  X |
// A2 |    |    |    |  X |
// A3 |    |    |    |  X |
// A4 |    |    |    |  X |
// A5 |    |    |    |  X |
// ---+----+----+----+----+------------------------------------
// D0 |  X |  X |  X |  X | RX -> do not use
// D1 |  X |  X |  X |  X | TX -> do not use
// D2 |    |    |  X |  X |
// D3 |    |    |  X |    |
// D4 |    |    |  X |  X |
// D5 |    |    |  X |    |
// D6 |    |    |  X |    | Led on Nanode5 (LOW=LED on)
// D7 |    |    |  X |  X | CS for EEPROM on Nanode5
// D8 |    |    |  X |  X | CS for ENC28J60 -> do not use
// D9 |    |    |  X |    | CS for FLASH (if installed)
// D10|    |    |  X |    | SS
// D11|  X |  X |  X |  X | MOSI for ENC28J60
// D12|  X |  X |  X |  X | MISO for ENC28J60
// D13|  X |  X |  X |  X | SCK for ENC28J60
// ---+----+----+----+----+------------------------------------
//  # | 15 | 15 |  6 |  5 |

#define MACADDRESS 0x74, 0x69, 0x69, 0x2D, 0x30, 0x31
#define MYIPADDR 192, 168, 67, 50
#define MYIPMASK 255, 255, 255, 0
#define MYDNS 192, 168, 67, 1
#define MYGW 192, 168, 67, 1
#define LISTENPORT 502 // Modbus
#define UARTBAUD 57600

#include <UIPEthernet.h>
#include "utility/logging.h"


#define MB_MAX_INPUT_REGISTERS   (6)  // input registers    (read)       (Analog In)
#define MB_MAX_HOLDING_REGISTERS (6)  // holding registeres (read-write) (Analog Out)
#define MB_MAX_COILS             (5)  // coils              (read-write) (Digital Out)
#define MB_MAX_DISCRETE_INPUTS   (8)  // Discretes Inputs   (read) 

// MAXDATALEN = #registers x 2 bytes  + 1 byte len or #Coils/DI / 8 + 1 + 1 byte len
//              for Arduino/ATMega328: 6 Ain = 6 input registers = 6x2+1 = 13
//              255-8 = 247 should be max, to keep FrameLen at 255 max
//              247 supports 123 registers per message
//              ToDo->Calculate
#define MAXDATALEN (13)

//Modbus functions
#define MB_FC_PROTOCOL_ERROR           (0)
#define MB_FC_READ_COILS               (1)
#define MB_FC_READ_DISCRETE_INPUT      (2)
#define MB_FC_READ_REGISTERS           (3)
#define MB_FC_READ_INPUT_REGISTERS     (4)
#define MB_FC_WRITE_COIL               (5)
#define MB_FC_WRITE_REGISTER           (6)
#define MB_FC_WRITE_MULTIPLE_COILS     (15)
#define MB_FC_WRITE_MULTIPLE_REGISTERS (16)
#define MB_FC_DEVICE_INFORMATION       (43)
//TODO: append with states and then use the state for response/error
//      the logic in the cases could then also be rearranged to check 
//      for errors first (and break) instead of nested if's


//end of defines

//Modbus registers
word MbInputRegisters[MB_MAX_INPUT_REGISTERS];
word MbHoldingRegisters[MB_MAX_HOLDING_REGISTERS];
bool MbCoils[MB_MAX_COILS];
bool MbDiscreteInputs[MB_MAX_DISCRETE_INPUTS];

// MODBUS Word is High-byte, Low-byte (different from ATMega328)
struct MbWord_t {
    byte H;
    byte L;
};

// Struct to define a buffer for the MODBUS messages
// TODO: use a union instead of the typecasts
struct MbFrame_t {
    MbWord_t TransactionID;
    MbWord_t ProtocolID;
    MbWord_t MsgLen;
    byte UnitID;
    byte FunctionCode;
    byte Data[MAXDATALEN];
};

EthernetServer MbServer = EthernetServer(LISTENPORT);

void printframe(MbFrame_t* fr) {
    Serial.print("TransactionID: ");
    Serial.println(word(fr -> TransactionID.H, fr -> TransactionID.L));
    Serial.print("ProtocolID   : ");
    Serial.println(word(fr -> ProtocolID.H, fr -> ProtocolID.L));
    Serial.print("MessageMsgLen: ");
    Serial.println(word(fr -> MsgLen.H, fr -> MsgLen.L));
    Serial.print("UnitID       : ");
    Serial.println(fr -> UnitID);
    Serial.print("FunctionCode : ");
    Serial.println(fr -> FunctionCode);

    uint16_t datalen = word(fr -> MsgLen.H, fr -> MsgLen.L) - 2; // 2 = sizeof(UnitID, FunctionCode)
    if (datalen > sizeof(fr -> Data))
        datalen = sizeof(fr -> Data);

    Serial.print("Data         :");
    for (int i = 0; i < datalen; i++) {
        Serial.print(" 0x");
        if (fr -> Data[i] < 16) Serial.print("0");
        Serial.print(fr -> Data[i], HEX);
    }
    Serial.println();
}

void setup() {
    // Init logging  
#if ACTLOGLEVEL > LOG_NONE
    LogObject.begin(UARTBAUD);
#endif

    // Init Modbus Registers
    for (int i = 0; i < MB_MAX_INPUT_REGISTERS; i++) MbInputRegisters[i] = i; // Init with some data for test-purposes
    for (int i = 0; i < MB_MAX_HOLDING_REGISTERS; i++) MbHoldingRegisters[i] = i + MB_MAX_INPUT_REGISTERS;
    for (int i = 0; i < MB_MAX_COILS; i++) MbCoils[i] = false;
    for (int i = 0; i < MB_MAX_DISCRETE_INPUTS; i++) MbDiscreteInputs[i] = false;

    // Init used IO
    pinMode(6, OUTPUT);
    digitalWrite(6, HIGH); // LED, HIGH = LED off

    // Init Network
    uint8_t mac[6] = {MACADDRESS};
    uint8_t myIP[4] = {MYIPADDR};
    uint8_t myMASK[4] = {MYIPMASK};
    uint8_t myDNS[4] = {MYDNS};
    uint8_t myGW[4] = {MYGW};

    Ethernet.init(8); // Nanode pin8 = CS of ENC28J60
    Ethernet.begin(mac, myIP, myDNS, myGW, myMASK);
    MbServer.begin();
}
void loop() {
    MbFrame_t MbFrame; // To save memory the buffer is used for request AND responds(!)
    size_t FrameLen;

    EthernetClient client = MbServer.available();
    FrameLen = client.available();
    if (FrameLen) {
        int CopyBytes = (FrameLen < sizeof(MbFrame) ? FrameLen : sizeof(MbFrame));
        client.read(reinterpret_cast < uint8_t* > ( & MbFrame), CopyBytes);
        client.flush(); // Discard the rest of the frame

        // TODO if we have to truncate message -> error 2 - illegal data address

        Serial.println("Request received");
        printframe( & MbFrame);

        if (MbFrame.ProtocolID.L != 0 || MbFrame.ProtocolID.H != 0) {
            // Error (not ModBus protocol)
            Serial.println("error: ProtocolID != 0 - not a Modbus message - discard");
            MbFrame.FunctionCode = MB_FC_PROTOCOL_ERROR;
        }

        switch (MbFrame.FunctionCode) {
        case MB_FC_PROTOCOL_ERROR:
            // Not a modbus message received -> discard
            break;
        case MB_FC_READ_COILS:
        case MB_FC_READ_DISCRETE_INPUT: {
            // Procedure is the same for Coils and Discrete Inputs
            // Set the Register array and its size to use
            uint16_t maxreg;
            bool * regarray;

            if (MbFrame.FunctionCode == MB_FC_READ_COILS) {
                maxreg = MB_MAX_COILS;
                regarray = MbCoils;
            } else {
                maxreg = MB_MAX_DISCRETE_INPUTS;
                regarray = MbDiscreteInputs;
            }

            // Parse request
            struct MbReadDiscretesReq_t {
                MbWord_t Start;
                MbWord_t Quantity;
            };

            MbReadDiscretesReq_t* req = reinterpret_cast < MbReadDiscretesReq_t* > ( & MbFrame.Data[0]);
            word start = word(req -> Start.H, req -> Start.L);
            word quant = word(req -> Quantity.H, req -> Quantity.L);

            // Perform read coils & create response
            if ((start + quant <= maxreg) && quant > 0) {
                struct MbReadDiscretesResp_t {
                    byte DataLen; // in bytes
                    byte Data[];
                };

                MbReadDiscretesResp_t* resp = reinterpret_cast < MbReadDiscretesResp_t* > ( & MbFrame.Data[0]);

                resp -> DataLen = ((quant + 7) / 8);
                uint16_t DataBits = 0; // Assume never more than 16 Coils on ATMega328
                for (int i = quant - 1; i >= 0; i--) {
                    DataBits = (DataBits << 1);
                    DataBits |= ( * (regarray + start + i) & 0x01);
                }
                resp -> Data[0] = lowByte(DataBits);
                if (quant > 8)
                    resp -> Data[1] = highByte(DataBits);

                MbFrame.MsgLen.L = resp -> DataLen + 3; // UnitID, FunctionCode, DataLen
                MbFrame.MsgLen.H = 0;
            } else {
                // Error: there are not so many registers...
                MbFrame.FunctionCode |= 0x80;
                MbFrame.Data[0] = 2; // Exception code 2 - Illegal Data Aaddress
                MbFrame.MsgLen.L = 3;
                MbFrame.MsgLen.H = 0;
            }
        }
        break;
        case MB_FC_READ_REGISTERS:
        case MB_FC_READ_INPUT_REGISTERS: {
            // Procedure is the same for Registers and Input Registers
            // Set the Register array and its size to use
            uint16_t maxreg;
            word * regarray;

            if (MbFrame.FunctionCode == MB_FC_READ_REGISTERS) {
                maxreg = MB_MAX_HOLDING_REGISTERS;
                regarray = MbHoldingRegisters;
            } else {
                maxreg = MB_MAX_INPUT_REGISTERS;
                regarray = MbInputRegisters;
            }

            // Parse request
            struct MbReadRegsReq_t {
                MbWord_t Start;
                MbWord_t Quantity;
            };

            MbReadRegsReq_t* req = reinterpret_cast < MbReadRegsReq_t* > ( & MbFrame.Data[0]);
            word start = word(req -> Start.H, req -> Start.L);
            word quant = word(req -> Quantity.H, req -> Quantity.L);

            if ((start + quant <= maxreg) && quant > 0) {
                // Create the response
                struct MbReadRegistersResp_t {
                    byte DataLen; // in bytes
                    MbWord_t Data[];
                };

                MbReadRegistersResp_t* Resp = reinterpret_cast < MbReadRegistersResp_t* > ( & MbFrame.Data[0]);
                for (word i = 0; i < quant; i++) {
                    Resp -> Data[i].L = lowByte( * (regarray + start + i));
                    Resp -> Data[i].H = highByte( * (regarray + start + i));
                }
                Resp -> DataLen = quant * 2; // quant is in bytes, data in Words
                MbFrame.MsgLen.L = Resp -> DataLen + 3; // 3 = sizeof(UnitID, FunctionCode, DataLen)
                MbFrame.MsgLen.H = 0;
            } else {
                // Error: there are not so many registers...
                MbFrame.FunctionCode |= 0x80;
                MbFrame.Data[0] = 2; // Exception code 2 - Illegal Data Aaddress
                MbFrame.MsgLen.L = 3; // UnitID, FunctionCode, ExceptionCode
                MbFrame.MsgLen.H = 0;
            }
        }
        break;
        case MB_FC_WRITE_COIL: {
            //Structure of MbFrame.Data[]
            struct MbWriteCoilReq_t {
                MbWord_t Coil;
                MbWord_t CoilStatus;
            };

            MbWriteCoilReq_t* req = reinterpret_cast < MbWriteCoilReq_t* > ( & MbFrame.Data[0]);
            word coil = word(req -> Coil.H, req -> Coil.L);
            word stat = word(req -> CoilStatus.H, req -> CoilStatus.L);

            if ((coil > 0) && (coil <= MB_MAX_COILS) && ((stat == 0) || (stat == 0xFF))) {
                MbCoils[coil] = (stat > 0);
                // no need to create the response as response = request
            } else {
                // Error: there are not so many registers...
                MbFrame.FunctionCode |= 0x80;
                MbFrame.Data[0] = 2; // Exception code 2 - Illegal Data Aaddress
                MbFrame.MsgLen.L = 3; // UnitID, FunctionCode, ExceptionCode
                MbFrame.MsgLen.H = 0;
            }
        }
        break;
        case MB_FC_WRITE_REGISTER: {
            struct MbWriteRegReq_t {
                MbWord_t Reg;
                MbWord_t RegValue;
            };

            MbWriteRegReq_t* req = reinterpret_cast < MbWriteRegReq_t* > ( & MbFrame.Data[0]);
            word reg = word(req -> Reg.H, req -> Reg.L);
            word val = word(req -> RegValue.H, req -> RegValue.L);

            if (reg <= MB_MAX_HOLDING_REGISTERS) {
                MbHoldingRegisters[reg] = val;
                // no need to create the response as response = request
            } else {
                // Error: there are not so many registers...
                MbFrame.FunctionCode |= 0x80;
                MbFrame.Data[0] = 2; // Exception code 2 - Illegal Data Aaddress
                MbFrame.MsgLen.L = 3; // UnitID, FunctionCode, ExceptionCode
                MbFrame.MsgLen.H = 0;
            }
        }
        break;
        case MB_FC_WRITE_MULTIPLE_COILS: {
            struct MbWriteMultiCoilsReq_t {
                MbWord_t Start;
                MbWord_t Quantity;
                byte DataLen; // in bytes
                byte Data[];
            };

            MbWriteMultiCoilsReq_t* req = reinterpret_cast < MbWriteMultiCoilsReq_t* > ( & MbFrame.Data[0]);
            word start = word(req -> Start.H, req -> Start.L);
            word quant = word(req -> Quantity.H, req -> Quantity.L);

            if ((start + quant <= MB_MAX_COILS) && quant > 0) {
                uint16_t DataBits = 0; // Assume never more than 16 Coils on ATMega328
                if (quant > 8) // 
                    DataBits = word(req -> Data[0], req -> Data[1]);
                else
                    DataBits = req -> Data[0];

                for (int i = 0; i < quant; i++) {
                    MbCoils[start + i] = DataBits & 0x01;
                    DataBits = (DataBits >> 1);
                }
                // response = truncated request (data is stripped)
                MbFrame.MsgLen.L = 6; // UnitID, FunctionCode, Start, Quantity
                MbFrame.MsgLen.H = 0;
            } else {
                // Error: there are not so many registers...
                MbFrame.FunctionCode |= 0x80;
                MbFrame.Data[0] = 2; // Exception code 2 - Illegal Data Aaddress
                MbFrame.MsgLen.L = 3; // UnitID, FunctionCode, ExceptionCode
                MbFrame.MsgLen.H = 0;
            }
        }
        break;
        case MB_FC_WRITE_MULTIPLE_REGISTERS: {
            //Structure of MbFrame.Data[]
            struct MbWriteMultiRegReq_t {
                MbWord_t Start;
                MbWord_t Quantity;
                byte DataLen; // in bytes
                MbWord_t Data[];
            };

            MbWriteMultiRegReq_t* req = reinterpret_cast < MbWriteMultiRegReq_t* > ( & MbFrame.Data[0]);
            word start = word(req -> Start.H, req -> Start.L);
            word quant = word(req -> Quantity.H, req -> Quantity.L);

            if ((start + quant <= MB_MAX_HOLDING_REGISTERS) && quant > 0) {
                for (int i = start; i < (start + quant); i++) {
                    MbHoldingRegisters[i] = word(req -> Data[i].H, req -> Data[i].L);
                }
                //Generate response (just remove datalen and data)
                MbFrame.MsgLen.L = 6; // UnitID, FunctionCode, Start, Quantity
                MbFrame.MsgLen.H = 0;
            } else {
                // Error: there are not so many registers...
                MbFrame.FunctionCode |= 0x80;
                MbFrame.Data[0] = 2; // Exception code 2 - Illegal Data Aaddress
                MbFrame.MsgLen.L = 3; // UnitID, FunctionCode, ExceptionCode
                MbFrame.MsgLen.H = 0;
            }
        }
        break;
        default:
            // Exception 01 - Illegal Function      
            MbFrame.FunctionCode |= 0x80;
            MbFrame.Data[0] = 1; // Exception code 01 - Illegal Function
            MbFrame.MsgLen.L = 3; // UnitID, FunctionCode, ExceptionCode
            MbFrame.MsgLen.H = 0;
            break;
        }

        // Send reply only when Modbus messgave was received
        if (MbFrame.FunctionCode != MB_FC_PROTOCOL_ERROR) {
            FrameLen = MbFrame.MsgLen.L + 6; // TransactioID, ProtocolID, MsgLen
            Serial.println("Response sent");
            printframe( & MbFrame);
            client.write(reinterpret_cast < char * > ( & MbFrame), FrameLen);
        }
    }

    // Update IO
    digitalWrite(6, (MbCoils[1] ? LOW : HIGH)); // Map %QX100.1 to D6 (LED)

}
