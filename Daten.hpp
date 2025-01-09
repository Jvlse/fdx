#pragma once
#include <vector>
#include <string>
#include <queue>
#include <chrono>
#include <b15f/b15f.h>


class Block {
    public:
        Block();
        Block(Block& copy);
        ~Block();
        void setDaten(char* d);
        void setBlocknummer(unsigned long n);
        void setBlocklaenge(unsigned int n);
        unsigned long getBlocknummer();
        unsigned int getBlocklaenge();
        char* getDaten();
        unsigned char getByte(int index);
        unsigned int getParitaet();
        void berechneParitaet();

        //Empfang
        void setParitaet(int parit);
        void setFlagKorrektEmpfangen(bool flagValue);
        bool getFlagKorrektEmpfangen();
    private:
        unsigned long blocknummer;
        unsigned int blocklaenge;
        unsigned int paritaet;
        char* daten; //Verweist auf Char Array

        //Empfang
        bool flagKorrektEmpfangen = false;

        


}; 

class Uebertragung {
    public:
        bool ladeBinFile();
        void schreibeBinFile();
        Block getBlock(int index);
        unsigned long getAnzBloecke();
        int getDEBUGNr();

        //Empfang
        void setAnzBloecke(unsigned long anz);
        void setBlocklaenge(int index, unsigned int laenge);
        void setParitaet(int index, unsigned int paritaet);
        void setDaten(int BlockIndex, char* daten, unsigned int paritaet);
        bool checkParitaetBestanden();
        std::queue<unsigned long> getWiederholer();
        int getAnzBytes();
    private:
        std::string dateipfad;
        Block* bloecke;
        unsigned long anzBloecke;

};

class B15Schnittstelle {
    private:
        Uebertragung sendeObjekt;
        char steuer1 = 0b00001010;
        char steuer2 = 0b00000101; 
        int sendeDelay = 10;
        B15F& drv = B15F::getInstance();
        std::queue<char> qAusgang;
        std::queue<char> qPrioAusgang;
        char vorherigEingefuegt = 0b00000000;
        
        Uebertragung empfangsObjekt;

        bool verbindungEtabliert = false;
        bool uebertragungAktiv = false;
        char eingangDaten = 0;
        char eingangDatenVorgaenger = 0;
        bool steuerSigErkannt = 0;
        char steuerSig = 0;
        


        //Lesen aus dem Strom
        int aktuelleBlockNr = 0; 
        int verbleibendeHalbbytes = 0;
        bool erstesHalbbitGelesen = false;
        char halbbitPuffer = 0;
        char* datenPuffer;
        int aktuellerDatenpufferIndex = 0;
        unsigned int sollParitaet = 0;

        int verbleibendeHalbbitsSteuerZahl = 0;
        int steuerZahl = -1;

        int warteZweiZyklen = 0;  //Warte, bis SteuerBefehl nach SteuerSig wegrutscht um Daten/Zahl zu lesen
                                        //Lesen grundsätzich auf eingangDatenVorgaenger!

        bool flagBlockNr = false;
        bool flagBlockLaeng = false;
        bool leseDaten = false; //Darf Bits als Daten interpretieren
        

        //Erwartung von Steuerzahl
        bool bekommeAnzBloecke = false;
        bool bekommeBlockNr = false;
        bool bekommeBlockLaenge = false;
        bool bekommeParitaet = false;
        bool sendeBlockErneut = false;

        //Verbindung etablieren
        char vorherigGelesen = 0;
        char aktuellGelesen = 0;
        bool prioAktiv = true;

        //Daten senden untersuchen
        char pufferDaten1 = 0;
        char pufferDaten2 = 0;
        char pufferDaten3 = 0;

        
        
    public:
        B15Schnittstelle();
        void addQAusgangDaten(char vierBit);
        void addQPrioAusgangDaten(char vierBit);
        void addQAusgangSteuersignal(char steuercode);
        void addQPrioAusgangSteuersignal(char steuercode);
        void pushAnzBloecke(unsigned long x);
        void pushAusgangBlock(Block b); //inkl. Nummer+Länge+Parität
        void pushAusgangBlockErneut(unsigned long nr); //Nur Nummer zur Anfrage
        void schreibeNextAusgang();
        void sendeGeladeneDatei();
        void sendenBeendenAnfrage();
        bool getVebindungEtabliert();
        void DEBUGAusgangsQueue();
        void DEBUGKonsolenAusgabe(char ausgabe);
        void TESTEVerarbeitungEingang();
        void etabliereVerbindung();
        void schreibeGeladeneDatei();

        char getEingang();
        void eingang4Bit(char vierBit);
};