#include <iostream>
#include <string>
#include <b15f/b15f.h>

#include <vector>
#include <bitset>


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


int main () {
    B15F& drv = B15F::getInstance();

    if (isatty(fileno(stdin))) {
        std::cout <<"Listening mode" << std::endl;
        drv.setRegister(&DDRA, 0b11110000); // Setzt erste 4 bits auf 1111 (letzte 4 zum Übertragen)
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
}
