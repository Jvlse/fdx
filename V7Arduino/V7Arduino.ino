#include <Arduino.h>
#include <bitHelpers.h>
#include <Vector.h>

// pc, linux pipe -> b15 -> arduino -> speichert auf anderem pc, von anderem pc -> adruino -> b15 -> pc über linux pipe

// echo ~/mnt/datadisk/*random.bin + "\n" > /dev/ttyUSB0

// Constants
const int CLOCK_MS = 50; // Clock duration in milliseconds

// Pin Definitions
const int SEND_PINS[4] = {2, 3, 4, 5};  // Pins for 4-bit output
const int RECIEVE_PINS[4] = {6, 7, 8, 9};  // Pins for 4-bit output

// Control Signals
const uint8_t START_PCKG = 0b1011;
const uint8_t END_PCKG = 0b1001;
const uint8_t START_CHECKSUM = 0b1010;
const uint8_t END_CHECKSUM = 0b1000;
const uint8_t AQR = 0b1110;
const uint8_t BEACON = 0b1100;
const uint8_t BEACON2 = 0b0011;

// Global Variables
const int MAX_BUFFER_SIZE = 32; // Maximum size for buffers
//uint8_t byteBuffer[MAX_BUFFER_SIZE];
Vector<uint8_t> byteBuffer;
Vector<uint8_t> recievedBuffer;
uint8_t checksumBuffer[MAX_BUFFER_SIZE];
int byteBufferIndex = 0;
int checksumBufferIndex = 0;
bool pckgStarted = false;
bool pckgEnded = false;
bool receivingChecksum = false;
bool write = false;
bool read = false;
uint8_t lastHalfByte = 0;

void setup() {
  Serial.begin(9600);
  setPinsAsInput();
  Serial.read();

  Serial.println(write ? "Master (Sender) Mode" : "Slave (Receiver) Mode");
}

void loop() {
  if (write) {
    uint8_t stringBinaryRepesentation[MAX_BUFFER_SIZE];
    int binaryLength = 0;

    uint8_t triplets[MAX_BUFFER_SIZE];
    int tripletLength = 0;

    uint8_t buffer[MAX_BUFFER_SIZE];
    int bufferLength = 0;

    for (int i = 0; i < tripletLength; i += 3) {
      buffer[bufferLength++] = START_PCKG;

      for (int j = 0; j < 3 && (i + j) < tripletLength; j++) {
        buffer[bufferLength++] = triplets[i + j];
      }

      buffer[bufferLength++] = END_PCKG;
      buffer[bufferLength++] = START_CHECKSUM;

      uint8_t combinedBits = 0;
      combinedBits |= (triplets[i] << 5);
      combinedBits |= (triplets[i + 1] << 2);
      combinedBits |= (triplets[i + 2] >> 1);

      buffer[bufferLength++] = createChecksum(combinedBits);
      buffer[bufferLength++] = END_CHECKSUM;

      sendBuffer(buffer, recievedBuffer);
    }

    delay(2000); // Delay before resending
  } else {
    receiveData();
  }
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

char translateByte(Vector<uint8_t> byteBuffer) {
  if (byteBufferIndex != 3) {
    Serial.println("Error: byteBuffer must contain exactly 3 chunks.");
    return '\0'; // AQR
  }

  uint8_t combined = 0;
  combined |= (byteBuffer[0] << 5);
  combined |= (byteBuffer[1] << 2);
  combined |= (byteBuffer[2] >> 1);

  return static_cast<char>(combined);
}

int createChecksum(const uint8_t byte) {
  int count = 0;
  for (int i = 0; i < 8; i++) {
    count += (byte >> i) & 1;
  }

  // Vector<Vector<uint8_t>> checksum;
  // int reihe = 0;
  // int zeile = 0;
  // for (int i = 0; i < 64; i++) {
  //   reihe += buffer[i];
  //   zeile += buffer[pow(8, i)-1]
  //   if(i % 8 == 0) {
  //     checksum.push_back((Vector<uint8_t>) reihe, zeile);
  //     reihe = 0;
  //     zeile = 0;
  //   }
  // }

  return count;
}


uint8_t getRECIEVE_PINS() {
  uint8_t data = 0;
  for (int i = 0; i < 4; i++) {
    if (digitalRead(RECIEVE_PINS[i])) {
      data |= (1 << i); // Set the corresponding bit if the pin is HIGH
    }
  }
  return data;
}


Vector<uint8_t> recieveSingle(uint8_t sendData, Vector<uint8_t> recievedData) {
  setPinsAsInput();
  uint8_t tmp = getRECIEVE_PINS();
  int i = 0;
  for (i; i < 4; i++) {
    setPinsAsOutput();
    digitalWrite(i + 4, sendData);
  }
  recievedData.push_back(tmp);
  delay(CLOCK_MS);
  return recievedData;
}


// Sending Functions
void sendSingle(uint8_t sendData) {
  for (int i = 0; i < 4; i++) {
    setPinsAsOutput();
    digitalWrite(i + 4, sendData);
  }
  delay(CLOCK_MS);
}

void sendBuffer(Vector<uint8_t> buffer, Vector<uint8_t> recievedBuffer) {
  for (size_t i = 0; i < buffer.size(); i++) {
    if (read) {
      recievedBuffer = recieveSingle(buffer[i], recievedBuffer); // TODO: replace i in recievedBuffer
    }
    else {
      sendSingle(buffer[i]);
    }
  }
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

// TODO: fix, halfByte ist momentan noch mit manchester verschlüsselt
void receiveData() {
  setPinsAsInput();
  uint8_t halfByte = getRECIEVE_PINS();
  if (halfByte == lastHalfByte) {
    halfByte = lastHalfByte;
    if (halfByte & 0b1000) { // Control signal check
      if (halfByte == START_PCKG) {
        pckgStarted = true;
        pckgEnded = false;
        byteBufferIndex = 0;
        checksumBufferIndex = 0;
      } else if (halfByte == END_PCKG) {
        pckgEnded = true;
      } else if (halfByte == START_CHECKSUM) {
        receivingChecksum = true;
      } else if (halfByte == END_CHECKSUM) {
        receivingChecksum = false;
        processReceivedData();
      } else if (halfByte == AQR) {
        byteBufferIndex -= (8 + 5);
      } else if (halfByte == BEACON2) {
        // TODO: für richtige seite
        digitalWrite(3, 1);
        digitalWrite(4, 1);
      }
    } else if (pckgStarted && !pckgEnded) {
      byteBuffer[byteBufferIndex++] = halfByte;
    } else if (receivingChecksum) {
      checksumBuffer[checksumBufferIndex++] = halfByte;
    }
  }
  lastHalfByte = halfByte; // TODO: Überprüfen
}

// TODO: für alle pins, auch send
void setPinsAsOutput() {
  for (int i = 0; i < 4; i++) {
    pinMode(RECIEVE_PINS[i], OUTPUT);
  }
}

void setPinsAsInput() {
  for (int i = 0; i < 4; i++) {
    pinMode(RECIEVE_PINS[i], INPUT);
  }
}
