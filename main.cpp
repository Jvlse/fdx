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


int main () {
    
}
