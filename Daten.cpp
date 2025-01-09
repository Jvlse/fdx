#include <iostream>
#include <fstream>
#include <bitset>
#include "Daten.hpp"

const int blockgroesse = 255;

void debugBlockInhalt(Block* &bloecke, std::streampos &anzBytes) {
    for (int i=0; i<(anzBytes/blockgroesse)+1; ++i) {
        std::cerr << "    +++Block " << i << "+++" << std::endl;
        std::cerr << "      Parität: " << bloecke[i].getParitaet() << std::endl;
        for (unsigned int j=0; j<bloecke[i].getBlocklaenge(); ++j) {
            std::cerr << j << ": " << std::bitset<8>(bloecke[i].getDaten()[j]) << std::endl;
        }
    }
}

//Klasse Übertragung
bool Uebertragung::ladeBinFile() {
    std::cerr << "wird vorbereitet...";

    Block* bloeckeZwischen = new Block[8000000];

    char* byteListe = new char[255];
    unsigned long bytesGelesen = 0;
    unsigned long bloeckeGelesen = 0;

    do {
        std::cin.read(byteListe, 255);
        bytesGelesen = std::cin.gcount();
        bloeckeZwischen[bloeckeGelesen].setBlocknummer(bloeckeGelesen);
        bloeckeZwischen[bloeckeGelesen].setBlocklaenge(bytesGelesen);
        bloeckeZwischen[bloeckeGelesen].setDaten(byteListe);
        
        bloeckeGelesen +=1;
        //std::cout << "Block " <<  bloeckeZwischen[bloeckeGelesen-1].getBlocknummer() << ": " << bloeckeZwischen[bloeckeGelesen-1].getBlocklaenge() << " Bytes" << std::endl;
    } while (bytesGelesen == 255);
    
    
    setAnzBloecke(bloeckeGelesen);
    for (int unsigned long i=0; i<bloeckeGelesen; ++i) {
        bloecke[i] = bloeckeZwischen[i];
        bloecke[i].setDaten(bloeckeZwischen[i].getDaten());
        //std::cout << "BlockF " <<  bloecke[i].getBlocknummer() << ": " << bloecke[i].getBlocklaenge() << " Bytes" << std::endl;
    }

    std::cerr << "fertig" << std::endl;
    return true;
}

void Uebertragung::schreibeBinFile() {
   for (unsigned int i=0; i<anzBloecke; ++i) {
    for (unsigned int j=0; j<getBlock(i).getBlocklaenge(); ++j) {
        std::cout << getBlock(i).getByte(j);
    }
   }
   std::cout.flush();
}

Block Uebertragung::getBlock(int index) {
    return bloecke[index];
}

unsigned long Uebertragung::getAnzBloecke() {
    return anzBloecke;
}

int Uebertragung::getDEBUGNr() {
    return 4;
}

void Uebertragung::setAnzBloecke(unsigned long anz) {
    anzBloecke = anz;
    bloecke = new Block[anzBloecke];
    for (unsigned long i=0; i<anzBloecke; ++i) {
        bloecke[i].setBlocknummer(i);
    }
}

void Uebertragung::setBlocklaenge(int index, unsigned int laenge) {
    bloecke[index].setBlocklaenge(laenge);
}

void Uebertragung::setParitaet(int index, unsigned int paritaet) {
    bloecke[index].setParitaet(paritaet);
}

void Uebertragung::setDaten(int BlockIndex, char* daten, unsigned int paritaet) {
    bloecke[BlockIndex].setDaten(daten);
    //std::cout << "Erster Wert: " << std::bitset<8>(bloecke[BlockIndex].getByte(0)) << std::endl;
    if (bloecke[BlockIndex].getParitaet() == paritaet) {
        bloecke[BlockIndex].setFlagKorrektEmpfangen(true);
        std::cerr<<"Block " << BlockIndex << " - OK" << std::endl;
    } else {
        bloecke[BlockIndex].setFlagKorrektEmpfangen(false);
        std::cerr<<"Block " << BlockIndex << " - Parität (" << bloecke[BlockIndex].getParitaet() <<  ") stimmt nicht überein!" << std::endl;
    }
}

bool Uebertragung::checkParitaetBestanden() {
    for (long unsigned int i=0; i<anzBloecke; ++i) {
        if (!bloecke[i].getFlagKorrektEmpfangen()) {
            return false;
        }
    }
    return true;
}

std::queue<unsigned long> Uebertragung::getWiederholer() {
    std::queue<unsigned long> wdh;
    for (unsigned long int i=0; i<anzBloecke; ++i) {
        if (bloecke[i].getFlagKorrektEmpfangen() ==false) {
            wdh.push(bloecke[i].getBlocknummer());
        }
    }
    return wdh;
}

int Uebertragung::getAnzBytes() {
    int zwischen = 0;
    for (unsigned long i=0; i<anzBloecke; ++i) {
        zwischen += bloecke[i].getBlocklaenge();
    }
    return zwischen;
}


// Klasse Block
Block::Block(Block& copy) {
    blocknummer = copy.blocknummer;
    blocklaenge = copy.blocklaenge;
    paritaet = copy.paritaet;
    daten = new char[blocklaenge];
    std::copy(copy.daten, copy.daten + blocklaenge, daten);
}

Block::Block() {

}

void Block::setBlocknummer(unsigned long n) {
    blocknummer = n;
}

void Block::setBlocklaenge(unsigned int n) {
    blocklaenge = n;
}

unsigned long Block::getBlocknummer() {
    return blocknummer;
}

unsigned int Block::getBlocklaenge() {
    return blocklaenge;
}

void Block::setDaten(char* d) {
    daten = new char[blockgroesse];
    std::copy(d, d+blockgroesse, daten);
    berechneParitaet();
}

char* Block::getDaten() {
    char* zwischen = new char[blockgroesse]; 
    std::copy(daten, daten+blockgroesse, zwischen);
    return zwischen;
}

Block::~Block() {
    delete[] daten;
}

unsigned char Block::getByte(int index) {
    return daten[index];
}

unsigned int Block::getParitaet() {
   return paritaet;
}

void Block::setParitaet(int parit) {
    paritaet = parit;
}

void Block::berechneParitaet() {
    paritaet = 0;
    char zwischen;
    for (unsigned int i=0; i<blocklaenge; ++i) {
        for (int j=0; j<8; ++j) {
            zwischen = 0;
            zwischen = ((getByte(i) >> j) & 1) << 0;
            if ((int)zwischen > 0) paritaet++;
        }
    }
}

void Block::setFlagKorrektEmpfangen(bool flagValue) {
    flagKorrektEmpfangen = flagValue;
}

bool Block::getFlagKorrektEmpfangen() {
    return flagKorrektEmpfangen;
}
