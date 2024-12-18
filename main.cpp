#include <iostream>
#include <string>
#include <b15f/b15f.h>
#include <chrono>

#include <vector>
#include <bitset>

#include <unistd.h>

using namespace std::chrono;

int CLOCK_MS = 50;
bool isMaster = false;


// usage: echo ~/mnt/datadisk/*random.bin | ./main
// compile:  g++ -std=c++17 main.cpp -lb15fdrv -o main

 
struct ControlSignals {
    std::bitset<4> StartByte, EndByte, StartChecksum, EndChecksum, ACK, AQR, ReversedStartByte, Beacon, Beacon2;

    // all binaries are prefixed with a 1 to indicate that the 4-bit value is a command
    ControlSignals() : StartByte(0b1011), EndByte(0b1001), StartChecksum(0b1010), EndChecksum(0b1000), ACK(0b1101), AQR(0b1110), ReversedStartByte(0b1101), Beacon(0b1100), Beacon2(0b0011) {}
};


std::bitset<4> manchester(bool flank, std::bitset<4> data) {
    return flank? ~data : data;
}


/**
 * Converts each character of a given string to its binary representation. (ASCII / UTF-8)
 * Returns the binary representation of each character as a vector of bitsets.
 * 
 * @param s The input string to be converted to binary.
 * @return std::vector<std::bitset<8>> Vector of bitsets representing the binary of each character.
 */
std::vector<std::bitset<8>> strToBinary(const std::string& s) {
    int n = s.length();
    std::vector<std::bitset<8>> binaryVector;

    // Iterate through each character in the string
    for (int i = 0; i < n; i++) {
        int val = static_cast<int>(s[i]); // convert each char to ASCII value
        std::bitset<8> bin(val); // Convert ASCII value to binary
        binaryVector.push_back(bin); // Add the binary representation to the vector
    }

    return binaryVector;
}


std::vector<std::bitset<4>> splitBytes(const std::vector<std::bitset<8>>& fullBytes) {
    std::vector<std::bitset<4>> triplets;

    for (const auto& byte : fullBytes) {
        // all bitsets are prefixed with a 0 to indicate that the 4-bit value is a word

        std::bitset<4> firstThrird(0b0111 & byte.to_ulong() >> 5); 
        std::bitset<4> secondThird(0b0111 & (byte.to_ulong() >> 2));
        std::bitset<4> lastThird((0b0110 & byte.to_ulong() << 1)); // last bit is always 0 (don't care)

        triplets.push_back(firstThrird);
        triplets.push_back(secondThird);
        triplets.push_back(lastThird);
    }

    return triplets;
}


void sendSingle(std::bitset<4> data, B15F &drv){
    std::cout << "Sending nibble!" << std::endl;
    drv.setRegister(&PORTA, manchester(false, data).to_ulong());
    drv.delay_ms(CLOCK_MS);
    drv.setRegister(&PORTA, manchester(true, data).to_ulong());
    drv.delay_ms(CLOCK_MS);
}

void sendBuffer(std::vector<std::bitset<4>> buffer, B15F &drv) {
    for (const auto& bin : buffer) { 
        if(isMaster){
            std::cout << manchester(false, bin) << std::endl;
            sendSingle(bin, drv);
            std::cout << "a" << std::endl;
        }else{
            std::cout << manchester(false, bin) << std::endl;
            sendSingle(bin, drv);
        }
    }
    
}


char translateByte(const std::vector<std::bitset<4>>& byteBuffer) {
    // Check if the size of the vector is 3
    if (byteBuffer.size() != 3) {
        throw std::invalid_argument("Error: Word buffer must contain exactly 3 4-bit bitsets. But it contains " + std::to_string(byteBuffer.size()) + " bitsets.");
    }

    // Combine the 4-bit bitsets into a single 8-bit bitset
    std::bitset<8> combinedBits;
    combinedBits |= (byteBuffer[0].to_ulong() << 5);
    combinedBits |= (byteBuffer[1].to_ulong() << 2);
    combinedBits |= (byteBuffer[2].to_ulong() >> 1);

    return static_cast<char>(combinedBits.to_ulong());
}


template<std::size_t N>
void reverse(std::bitset<N> &b) {
    for(std::size_t i = 0; i < N/2; ++i) {
        bool t = b[i];
        b[i] = b[N-i-1];
        b[N-i-1] = t;
    }
}

int createChecksum(std::bitset<8>& byte) {
    return 8-byte.count();
}


void receiveBuffer(B15F& drv) {
    ControlSignals controlSignals;

    std::bitset<4> currentHalfByte = 0b0000; // arbitrary value
    bool byteStarted = false;
    bool byteEnded = false;

    bool checksumStarted = false;
    bool checksumEnded = false;

    bool skip = false;

    std::vector<std::bitset<4>> byteBuffer;
    std::vector<std::bitset<4>> checksumBuffer;

    // timer init
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

        
        if(manchester(true, tmp) == currentHalfByt e){
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
                // calculate checksum over byteBuffer

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
                // started listening in the middle of a word
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
            // send ACK or AQR
        // } 
    // }
}





void read(B15F drv)
{
    std::bitset<4> prev = 0b0111;
    while(1){
        std::bitset<4> temp(drv.getRegister(&PINA));
        std::cout << temp << " - "<< manchester(true, temp) << std::endl;
        if(manchester(true, temp) == prev){
            // wort!
            std::cout << "wORT " << std::endl;
        } 
        prev = temp;    
    }
}


int main () {
    B15F& drv = B15F::getInstance();
	
    ControlSignals controlSignals;

    if(isatty(fileno(stdin))) {
        std::cout <<"Listening mode" << std::endl;
        drv.setRegister(&DDRA, controlSignals.Beacon.to_ulong() << 4);	// BEACON
        while(drv.getRegister(&DDRA) != 0b11111111) {}
        receiveBuffer(drv);
        // read(drv);
        return 0;
    } else {
        isMaster = true;
        std::cout <<"Writing mode" << std::endl;
        drv.setRegister(&DDRA, controlSignals.Beacon.to_ulong());
    }
	
	
    // while(drv.getRegister(&DDRA) != 0b11111111) {}
	

    // first get string / file from pipe
    std::string input;
    std::getline(std::cin, input);

    // Convert the string to bytes and split each byte into 3 4-bit bitsets
    std::vector<std::bitset<4>> stringBinaryRepesentation = splitBytes(strToBinary(input));

    // start while loop here
    for (size_t i = 0; i < stringBinaryRepesentation.size(); i += 3) {
        std::vector<std::bitset<4>> buffer;
        
        buffer.push_back(controlSignals.StartByte);

        // add next 3 bitsets of stringBinaryRepesentation
        for (size_t j = 0; j < 3 && (i + j) < stringBinaryRepesentation.size(); ++j) {
            buffer.push_back(stringBinaryRepesentation[i + j]);
        }

        buffer.push_back(controlSignals.EndByte);
        buffer.push_back(controlSignals.StartChecksum);
        // Checksum here, todo later

        // after 3 4-bit words have been sent = 1 Byte, create checksum
        std::bitset<8> byte;
        byte |= (stringBinaryRepesentation[i].to_ulong() << 5);
        byte |= (stringBinaryRepesentation[i + 1].to_ulong() << 2);
        byte |= (stringBinaryRepesentation[i + 2].to_ulong() >> 1);
        buffer.push_back(std::bitset<4>(createChecksum(byte)));

        buffer.push_back(controlSignals.EndChecksum);
        
		bool error = false;
        for (const auto& nibble : buffer) {
            sendSingle(nibble, drv);
            if(drv.getRegister(&PINA) == controlSignals.AQR.to_ulong() || drv.getRegister(&PINA) == manchester(true, controlSignals.AQR).to_ulong()){
                error = true;
            }
        }
        if(error) {
        	std::cout << "Fehler bei Übertragung, erneut senden!" << std::endl;
            i-=3; // reduce i to get back to beginning of byte
            error = false;
        }

        // wait for ACK or AQR, todo later
        // if AQR, resend buffer; you may want to add a loop here to handle resending
    }

    return 0;
}
