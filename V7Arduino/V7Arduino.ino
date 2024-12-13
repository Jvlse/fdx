#include <Arduino.h>
#include <bitHelpers.h>
#include <StandardCplusplus.h>
#include <boost_1_51_0.h>

// echo ~/mnt/datadisk/*random.bin + "\n" > /dev/ttyUSB0

// Constants
const int CLOCK_MS = 50; // Clock duration in milliseconds

// Pin Definitions
// const int DATA_PIN = 2; // Pin used for data transmission and reception
const int DATA_PINS[4] = {2, 3, 4, 5};  // Pins for 4-bit output

// Control Signals
const uint8_t START_BYTE = 0b1011;
const uint8_t END_BYTE = 0b1001;
const uint8_t START_CHECKSUM = 0b1010;
const uint8_t END_CHECKSUM = 0b1000;
const uint8_t ACK = 0b1100;
const uint8_t AQR = 0b1110;

// Global Variables
const int MAX_BUFFER_SIZE = 32; // Maximum size for buffers
uint8_t byteBuffer[MAX_BUFFER_SIZE];
uint8_t checksumBuffer[MAX_BUFFER_SIZE];
int byteBufferIndex = 0;
int checksumBufferIndex = 0;
bool byteStarted = false;
bool byteEnded = false;
bool receivingChecksum = false;
bool isMaster = false; // Determines whether the Arduino is in sender mode

// Helper Functions
uint8_t manchester(bool flank, uint8_t data) {
    return flank ? (data ^ 0b1111) : data;
}

void strToBinary(const char* str, uint8_t* binaryArray, int& length) {
    length = 0;
    for (int i = 0; str[i] != '\0'; i++) {
        binaryArray[length++] = static_cast<uint8_t>(str[i]);
    }
}

void splitBytes(const uint8_t* fullBytes, int fullLength, uint8_t* triplets, int& tripletLength) {
    tripletLength = 0;
    for (int i = 0; i < fullLength; i++) {
        uint8_t byte = fullBytes[i];
        triplets[tripletLength++] = (byte >> 5) & 0b0111;
        triplets[tripletLength++] = (byte >> 2) & 0b0111;
        triplets[tripletLength++] = (byte << 1) & 0b0110;
    }
}

char translateByte(uint8_t* byteBuffer) {
    if (byteBufferIndex != 3) {
        Serial.println("Error: byteBuffer must contain exactly 3 chunks.");
        return '\0';
    }

    uint8_t combined = 0;
    combined |= (byteBuffer[0] << 5);
    combined |= (byteBuffer[1] << 2);
    combined |= (byteBuffer[2] >> 1);

    return static_cast<char>(combined);
}

int createChecksum(const uint8_t byte) {
    int count = 0;
    for (int i = 0; i < 8; ++i) {
        count += (byte >> i) & 1;
    }
    return count;
}



uint8_t[] recieveSingle(uint8_t sendData, uint8_t[] recievedData) {
    uint8_t data[4] = getDATA_PINS();
    for (int i = 0; i < 4; i++) {
      recievedData.push(manchester(false, digitalRead(i)));
      if(write) {
        digitalWrite(i+4, manchester(false, data); 
      }
    }
    delay(CLOCK_MS);
    
    for (int i = 0; i < 4; i++) {
      recievedData.push(manchester(false, digitalRead(i)));
      if(write) {
        digitalWrite(i+4, manchester(false, data);
        write = false;
      }
    }
    delay(CLOCK_MS);
}



uint8_t buffer[];
uint8_t newNumber = 0b100;

void push(uint8_t buffer[]) {
  for (int i = buffer.length; i > 0; i--) {
    if (i == 0) {
      buffer[i] = newNumber;
    } else {
      buffer[i] = buffer[i - 1];
    }
  }
}



// Sending Functions
void sendSingle(uint8_t data) {
    int data[4] = getDATA_PINS();
    for (int i = 0; i < 4; i++) {
      digitalWrite(DATA_PINS[i], manchester(false, data%pow(2, i))
    }
    delay(CLOCK_MS);
    digitalWrite(DATA_PINS, manchester(true, data));
    delay(CLOCK_MS);
}

void sendBuffer(const uint8_t* buffer, int length) {
    for (int i = 0; i < length; i++) {
        sendSingle(buffer[i]);
    }
}

int getDATA_PINS() {
    int data[4];
    for (int i = 0; i < 4; i++) {
        data[i] = digitalRead(DATA_PINS[i]) * pow(2, i);
    }
    return data;
}

// Receiving Functions
void processReceivedData() {
    char receivedChar = translateByte(byteBuffer);

    uint8_t receivedBits = static_cast<uint8_t>(receivedChar);
    int calculatedChecksum = createChecksum(receivedBits);

    if (checksumBufferIndex > 0) {
        int receivedChecksum = checksumBuffer[0];

        if (receivedChecksum == calculatedChecksum) {
            Serial.print("Received character: ");
            Serial.println(receivedChar);
        } else {
            Serial.println("Checksum mismatch! Sending AQR.");
        }
    } else {
        Serial.println("No checksum received. Sending AQR.");
    }

    byteBufferIndex = 0;
    checksumBufferIndex = 0;
}

uint8_t readNibble() {
  uint8_t nibble = 0;
  for (int i = 0; i < 4; i++) {
    nibble |= (digitalRead(DATA_PINS[i]) << i);  // Read bits and assemble into a nibble
  }
  return nibble;
}

void receiveData() {
    uint8_t currentHalfByte;

    if (currentHalfByte != 0b00000000) {
        currentHalfByte = getDATA_PINS();

        if (currentHalfByte & 0b1000) { // Control signal check
            if (currentHalfByte == START_BYTE) {
                byteStarted = true;
                byteEnded = false;
                byteBufferIndex = 0;
                checksumBufferIndex = 0;
            } else if (currentHalfByte == END_BYTE) {
                byteEnded = true;
            } else if (currentHalfByte == START_CHECKSUM) {
                receivingChecksum = true;
            } else if (currentHalfByte == END_CHECKSUM) {
                receivingChecksum = false;
                processReceivedData();
            }
        } else if (byteStarted && !byteEnded) {
            byteBuffer[byteBufferIndex++] = currentHalfByte;
        } else if (receivingChecksum) {
            checksumBuffer[checksumBufferIndex++] = currentHalfByte;
        }
    }
}

void setPinsAsOutput() {
  for (int i = 0; i < 4; i++) {
    pinMode(DATA_PINS[i], OUTPUT);
  }
}

void setPinsAsInput() {
  for (int i = 0; i < 4; i++) {
    pinMode(DATA_PINS[i], INPUT);
  }
  

// Main Setup and Loop
void setup() {
    Serial.begin(9600);

    setPinsAsInput();

    
    Serial.read();

    Serial.println(isMaster ? "Master (Sender) Mode" : "Slave (Receiver) Mode");
}

void loop() {
    if (isMaster) {
        // Sending Mode
        const char* input = "Hello, Arduino!";
        uint8_t stringBinaryRepesentation[MAX_BUFFER_SIZE];
        int binaryLength = 0;

        strToBinary(input, stringBinaryRepesentation, binaryLength);

        uint8_t triplets[MAX_BUFFER_SIZE];
        int tripletLength = 0;

        splitBytes(stringBinaryRepesentation, binaryLength, triplets, tripletLength);

        uint8_t buffer[MAX_BUFFER_SIZE];
        int bufferLength = 0;

        for (int i = 0; i < tripletLength; i += 3) {
            buffer[bufferLength++] = START_BYTE;

            for (int j = 0; j < 3 && (i + j) < tripletLength; ++j) {
                buffer[bufferLength++] = triplets[i + j];
            }

            buffer[bufferLength++] = END_BYTE;
            buffer[bufferLength++] = START_CHECKSUM;

            uint8_t combinedBits = 0;
            combinedBits |= (triplets[i] << 5);
            combinedBits |= (triplets[i + 1] << 2);
            combinedBits |= (triplets[i + 2] >> 1);

            buffer[bufferLength++] = createChecksum(combinedBits);
            buffer[bufferLength++] = END_CHECKSUM;

            sendBuffer(buffer, bufferLength);
        }

        delay(2000); // Delay before resending
    } else {
        // Receiving Mode
        receiveData();
    }
}
