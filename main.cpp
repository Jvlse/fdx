#include <iostream>
#include <string>
#include <b15f/b15f.h>
#include <chrono>
#include <thread>

#include <vector>
#include <bitset>

#include <unistd.h>

using namespace std::chrono;

int CLOCK_MS = 50;  // für die clock
bool isMaster = false;  // gibt an ob Sender (true) oder Empfänger (false)

struct ControlSignals {
    std::bitset<4> StartByte, EndByte, StartChecksum, EndChecksum, ACK, AQR, ReversedStartByte;

    // alle bitsets die mit 1 beginnen sind Kontrolsignale
    ControlSignals() : StartByte(0b1011), EndByte(0b1001), StartChecksum(0b1010), EndChecksum(0b1000), ACK(0b1100), AQR(0b1110),ReversedStartByte(0b1101) {}
};

// clocking
std::bitset<4> manchester(bool flank, std::bitset<4> data) {
    if(!flank) {
        return data; // falls flanke -> return data
    }

    return data ^ std::bitset<4>(0b1111); // falls keine flanke -> return !data
}


/**
 * Konvertiert jeden char vom string zu binär (ASCII)
 * Returned binary Representation von jedem char als vektor von bytes.
 * 
 * @param s string der zu binär konvertiert wird
 * @return std::vector<std::bitset<8>> vektor der jeden char in binär enthält
 */
std::vector<std::bitset<8>> strToBinary(const std::string& s) {
    int n = s.length();
    std::vector<std::bitset<8>> binaryVector;

    // Alle character durchlaufen
    for (int i = 0; i < n; i++) {
        int val = static_cast<int>(s[i]); // jeder char wird zu ASCII decimal
        std::bitset<8> bin(val); // Konvertiert ASCII zu binary
        binaryVector.push_back(bin); // Add the binary representation to the vector
    }

    return binaryVector;
}

// bytes werden in 3 teile geteilt
std::vector<std::bitset<4>> splitBytes(const std::vector<std::bitset<8>>& fullBytes) {
    std::vector<std::bitset<4>> triplets;

    for (const auto& byte : fullBytes) { // alle bitsets fangen mit 0 an um zu zeigen, dass es kein control signal ist
        std::bitset<4> firstThrird(0b0111 & (byte.to_ulong() >> 5)); // byte wird um 5 nach rechts geshiftet und dann mit 0111 AND genommen
        std::bitset<4> secondThird(0b0111 & (byte.to_ulong() >> 2)); // -> 2 nach rechts
        std::bitset<4> lastThird(0b0110 & (byte.to_ulong() << 1)); // letztes Bit immer 0 -> 1 nach links, evtl 0 als check mit modulo

        triplets.push_back(firstThrird);
        triplets.push_back(secondThird);
        triplets.push_back(lastThird);
    }
    return triplets;
}

// Sendet einen Byte, die ersten 4 bit ohne flanke und die restlichen mit flanke
void sendSingle(std::bitset<8> data, B15F &drv){
    drv.setRegister(&PORTA, manchester(false, data.to_ulong()).to_ulong());
    drv.delay_ms(CLOCK_MS);
    drv.setRegister(&PORTA, manchester(true, data.to_ulong()).to_ulong());
    drv.delay_ms(CLOCK_MS);
}

void sendBuffer(std::vector<std::bitset<4>> buffer, B15F &drv) {
    for (const auto& bin : buffer) {
        if(isMaster){
            std::cout << manchester(false, bin) << std::endl;
            sendSingle(static_cast<std::bitset<8>>(bin.to_ulong()), drv);
            std::cout << std::endl;
        }else{
            std::cout << manchester(false, bin) << std::endl;
            sendSingle(static_cast<std::bitset<8>>(bin.to_ulong()), drv);
        }
    }
}

// setzt byte wieder zusammen und returned es als char
char translateByte(const std::vector<std::bitset<4>>& byteBuffer) {
    // größe muss 3 sein um byte zusammensetzen zu können
    if (byteBuffer.size() != 3) {
        throw std::invalid_argument("Error: Word buffer must contain exactly 3 4-bit bitsets. But it contains " + std::to_string(byteBuffer.size()) + " bitsets.");
    }

    // Kombiniert 4-bit bitsets zu 8-bit bitset
    std::bitset<8> combinedBits;
    combinedBits |= (byteBuffer[0].to_ulong() << 5);
    combinedBits |= (byteBuffer[1].to_ulong() << 2);
    combinedBits |= (byteBuffer[2].to_ulong() >> 1);

    return static_cast<char>(combinedBits.to_ulong());
}

int createChecksum(std::bitset<8>& byte) {
    return byte.count();
}


void receiveBuffer(B15F& drv) {
    ControlSignals controlSignals;

    std::bitset<4> currentHalfByte = 0b0000;
    bool byteStarted = false;
    bool byteEnded = false;

    bool checksumStarted = false;
    bool checksumEnded = false;

    bool skip = false;

    std::vector<std::bitset<4>> byteBuffer;
    std::vector<std::bitset<4>> checksumBuffer;

    // timer initialisieren
    time_point<system_clock> t = system_clock::now();

    while (true) {
        std::bitset<4> tmp(drv.getRegister(&PINA));
        time_point<system_clock> now = system_clock::now();

        if(now >= t+ milliseconds(static_cast<int>(CLOCK_MS * 1.6))){
            currentHalfByte = tmp;
            t = now;
            skip = false;
            continue;
        }

        
        if(manchester(true, tmp) == currentHalfByte){
            t= now;
            if(skip){
                skip = false;
                currentHalfByte=tmp;
                continue;
            }
            skip = true;

            if (currentHalfByte == controlSignals.EndByte) {
                byteEnded = true;
            }

            if (byteStarted && !byteEnded) {
                byteBuffer.push_back(currentHalfByte);
            }

            if (currentHalfByte == controlSignals.StartByte) {
                byteStarted = true;
                byteEnded = false;
                byteBuffer.clear();
                checksumBuffer.clear();
            }

            if (currentHalfByte == controlSignals.EndChecksum) {
                checksumEnded = true;
                checksumStarted = false;
            }

            if (checksumStarted && !checksumEnded) {
                
                checksumBuffer.push_back(currentHalfByte);
            }

            if (currentHalfByte == controlSignals.StartChecksum) {
                checksumStarted = true;
                checksumEnded = false;
            } 

            if (currentHalfByte == controlSignals.EndChecksum && byteStarted && byteEnded) {
                // checksum ausrechnen

                char byte = translateByte(byteBuffer);
                std::cout << byte << std::flush;
                std::bitset<8> myBitset(byte);
                int calculatedChecksum = createChecksum(myBitset);

                if (!checksumBuffer.empty() && checksumBuffer.size() > 0) {
                    int checksum = checksumBuffer[0].to_ulong();

                    if (checksum == calculatedChecksum) {
                        // sendBuffer({controlSignals.ACK}, drv);
                    } else {
                        sendBuffer({controlSignals.AQR}, drv);
                    }
                }  else {
                    sendBuffer({controlSignals.AQR}, drv);
                }
                byteBuffer.clear();
                checksumBuffer.clear();
            } else if (currentHalfByte == controlSignals.EndChecksum) {
                // in mitte des bytes angefangen mitzulesen
                sendBuffer({controlSignals.AQR}, drv);
                byteBuffer.clear();
                checksumBuffer.clear();
            }
        }
        currentHalfByte=tmp;
    }

    // while (1) {
        // currentHalfByte = drv.getRegister (&PINA);
        // cout << ((int)drv.getRegister (&PINA)) << endl; drv.delay_ms(10);

        // if (currentHalfByte == controlSignals.EndChecksum) {
            // ACK oder AQR senden
        // }    
    // }
}


int main () {
    B15F& drv = B15F::getInstance();

    if (isatty(fileno(stdin))) {
        std::cout <<"Listening mode" << std::endl;
        drv.setRegister(&DDRA, 0b11110000); // Setzt erste 4 bits auf 1111 (letzte 4 zum Übertragen)
        receiveBuffer(drv);
        return 0;
    } else {
        isMaster = true;    // Gibt an das Signal von diesem Gerät gesendet wird
        std::cout <<"Writing mode" << std::endl;
        drv.setRegister(&DDRA, 0b00001111); // Setzt erste 4 bits auf 1111 (erste 4 zum Übertragen)
    }

    ControlSignals controlSignals;

    // Name des file wird über cin eingegeben
    std::string input;
    std::getline(std::cin, input);

    // Konvertiert string zu bytes und teilt sie in 3 4-bit bitsets
    std::vector<std::bitset<4>> stringBinaryRepesentation = splitBytes(strToBinary(input));
    
    // für jedes byte 3 bitsets
    for (size_t i = 0; i < stringBinaryRepesentation.size(); i += 3) {
        std::vector<std::bitset<4>> buffer;
        
        buffer.push_back(controlSignals.StartByte);

        // nächste 3 bitsets von stringBinaryRepesentation pushen
        for (size_t j = 0; j < 3 && (i + j) < stringBinaryRepesentation.size(); ++j) {
            buffer.push_back(stringBinaryRepesentation[i + j]);
        }

        buffer.push_back(controlSignals.EndByte);
        buffer.push_back(controlSignals.StartChecksum);
        // Checksum hier

        // nach 3 4-bit bitsets checksum
        std::bitset<8> combinedBits;
        combinedBits |= (stringBinaryRepesentation[i].to_ulong() << 5);
        combinedBits |= (stringBinaryRepesentation[i + 1].to_ulong() << 2);
        combinedBits |= (stringBinaryRepesentation[i + 2].to_ulong() >> 1);
        buffer.push_back(std::bitset<4>(createChecksum(combinedBits)));

        buffer.push_back(controlSignals.EndChecksum);
        
        for (const auto& bin : buffer) {
            sendSingle(bin.to_ulong(), drv);
            if(drv.getRegister(&PINA) == controlSignals.AQR.to_ulong() || drv.getRegister(&PINA) == manchester(true, controlSignals.AQR).to_ulong()){
                std::cout << "Fehler bei Übertragung, erneut senden!" << std::endl;
                i -=6;
                break;
            }
        }
        // Auf ACK oder AQR warten
        // bei AQR, buffer erneut senden; evtl loop zum resenden
    }
    
    // th1.join();
    // Multithreading um gleichzeitig zu senden und zu empfangen

    return 0;
}
