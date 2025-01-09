//#include <b15f/b15f.h>
#include "Daten.hpp"
#include <iostream>
#include <fstream>
#include <thread> 
#include <bitset>
#include <chrono>


B15Schnittstelle schnittstelle;
auto startClock = std::chrono::high_resolution_clock::now();
auto endClock = std::chrono::high_resolution_clock::now();


//Filtert die Ausgangsseite raus
unsigned char filterAusgang(unsigned char achtBit) {
    //std::cerr << "Vorher :" << std::bitset<8>(achtBit) << std::endl;
    for (int i=0; i<4; ++i) {
        achtBit &= ~(1<<i);
    }
    //std::cerr << "Nachher:" << std::bitset<8>(achtBit) << std::endl;
    return achtBit;
}

unsigned char filterAusgangInvert(unsigned char achtBit) {
    //std::cerr << "Vorher :" << std::bitset<8>(achtBit) << std::endl;
    char zwischen = 0;
    
    for (int i=0; i<4; ++i) {
        achtBit &= ~(1<<i);
    }
    //std::cerr << "Mitte:" << std::bitset<8>(achtBit) << std::endl;
    for (int i=4; i<8; ++i) {
        zwischen |= ((achtBit >> ((i)) & 1) << ((7-i)+4));
    }
    //std::cerr << "Nachher:" << std::bitset<8>(zwischen) << std::endl;
    return zwischen;
}

//Filtert die Eingangsseite raus
unsigned char filterEingang(unsigned char achtBit) {
    //std::cerr << "Vorher :" << std::bitset<8>(achtBit) << std::endl;
    for (int i=4; i<8; ++i) {
        achtBit &= ~(1<<i);
    }
    //std::cerr << "Nachher:" << std::bitset<8>(achtBit) << std::endl;
    return achtBit;
}

void hintergrundAufgabeIO() {
    unsigned char zwischen = filterAusgang(schnittstelle.getEingang());
    unsigned char letzter = zwischen;
    unsigned char aktuell = zwischen;
    
    while (1) {
        //Lesen
        aktuell = filterAusgang(schnittstelle.getEingang());
                
        
        if (letzter!=aktuell) {
            schnittstelle.eingang4Bit(aktuell);
            letzter = aktuell;
            schnittstelle.DEBUGKonsolenAusgabe(aktuell);
        }
        
        //Schreiben
        schnittstelle.schreibeNextAusgang(); 

        
        //Nochmal lesen
        aktuell = filterAusgang(schnittstelle.getEingang());
    
         if (letzter!=aktuell) {
            schnittstelle.eingang4Bit(aktuell);
            letzter = aktuell;
            schnittstelle.DEBUGKonsolenAusgabe(aktuell);
        }
        
    }
}



//int main(int argc, char *argv[])
int main(int argc, char *argv[]) {
    Uebertragung sendeObjekt; 

    //sendeObjekt.ladeBinFile();
    schnittstelle.sendeGeladeneDatei();


    schnittstelle.etabliereVerbindung();

    schnittstelle.schreibeGeladeneDatei();
    //schnittstelle.addQAusgangSteuersignal(0b00001100);


    //Verbindung etabliert, Daten senden und empfangen



   

    

    //schnittstelle.TESTEVerarbeitungEingang();
    //schnittstelle.DEBUGAusgangsQueue();
    
    

    /*
    //schnittstelle.DEBUGAusgangsQueue();
    char zwischen = filterAusgang(schnittstelle.getEingang());
    char letzter = zwischen;
    char aktuell = zwischen;

    while(1) {
        aktuell = filterAusgang(schnittstelle.getEingang());
                
        
        if (letzter!=aktuell) {
            schnittstelle.eingang4Bit(aktuell);
            letzter = aktuell;
            std::cerr << std::bitset<8>(aktuell) << std::endl;
        }
        
        //Schreiben
        schnittstelle.schreibeNextAusgang(); 
    } 
    */
}



//Klasse B15Schnittstelle
B15Schnittstelle::B15Schnittstelle() {
    sendeObjekt.ladeBinFile();
    drv.setRegister(&DDRA, 0b00001111);
    drv.setRegister(&PORTA, 0b00001111);
    drv.delay_ms(500);
    drv.setRegister(&PORTA, 0b00000000);
    drv.delay_ms(500);
    
}

void B15Schnittstelle::pushAnzBloecke(unsigned long x) {
    addQAusgangSteuersignal(0b00000010);
     
    for (int i=7; i>=0; --i) {              //Höchstwertigstes Halbbyte zuerst
        char zwischen = 0;
        for (int j=0;j<4; ++j) {
            zwischen |= ((x >> ((i*4)+j)) & 1) << j;
        }
        addQAusgangDaten(zwischen);
    }
}


void B15Schnittstelle::pushAusgangBlockErneut(unsigned long nr) {
    //Blocknummer erneut senden
    addQPrioAusgangSteuersignal(0b00000111);
   

    for (int i=7; i>=0; --i) {              //Höchstwertigstes Halbbyte zuerst
        char zwischen = 0;
        for (int j=0;j<4; ++j) {
            zwischen |= ((nr >> ((i*4)+j)) & 1) << j;
        }
        addQPrioAusgangDaten(zwischen);
        addQPrioAusgangSteuersignal(0b00000000);
    }
}


void B15Schnittstelle::pushAusgangBlock(Block b) {
    //Blocknummer
    addQAusgangSteuersignal(0b00000011);
   

    for (int i=7; i>=0; --i) {              //Höchstwertigstes Halbbyte zuerst
        char zwischen = 0;
        for (int j=0;j<4; ++j) {
            zwischen |= ((b.getBlocknummer() >> ((i*4)+j)) & 1) << j;
        }
        addQAusgangDaten(zwischen);
    }


    //Blocklänge
    addQAusgangSteuersignal(0b00000100);

    for (int i=1; i>=0; --i) {             //Höchstwertigstes Halbbyte zuerst
        char zwischen = 0;
        for (int j=0;j<4; ++j) {
            zwischen |= ((b.getBlocklaenge() >> ((i*4)+j)) & 1) << j;
        }
        addQAusgangDaten(zwischen);
    }


    //Parätsbits
    addQAusgangSteuersignal(0b00000110);

    for (int i=3; i>=0; --i) {          //Höchstwertigstes Halbbyte zuerst
        char zwischen = 0;
        for (int j=0;j<4; ++j) {
            zwischen |= ((b.getParitaet() >> ((i*4)+j)) & 1) << j;
        }
        addQAusgangDaten(zwischen);
    }


    //Alle Bytes zur Queue hinzufügen
    for (unsigned int i=0; i<b.getBlocklaenge(); ++i) {

        for (int j=1; j>=0; --j) {      //Höchstwertigstes Halbbyte zuerst
            char zwischen = 0;
            for (int k=0;k<4; ++k) {
                zwischen |= ((b.getByte(i) >> ((j*4)+k)) & 1) << k;
            }
            addQAusgangDaten(zwischen);
        }   
    }
    addQAusgangSteuersignal(0b00000000);

}

void B15Schnittstelle::schreibeNextAusgang() {
        
    if (prioAktiv) {
        if (!qPrioAusgang.empty()) {
            //std::cerr << std::bitset<8>(qPrioAusgang.front()) << std::endl;
            //drv.setRegister(&DDRA, 0b00001111);
            drv.setRegister(&PORTA, (int) qPrioAusgang.front());
            qPrioAusgang.pop();
        }
    } else {
        if (!qAusgang.empty()) {
            
            pufferDaten3 = pufferDaten2;
            pufferDaten2 = pufferDaten1;
            pufferDaten1 = qAusgang.front();

            if ((pufferDaten3 == 0b00001010) && (pufferDaten2 == 0b00000101) && (pufferDaten1 == 0b00000011)) {
                //std::cerr << "Nächster Befehl: Block senden" << std::endl;
                if (!qPrioAusgang.empty()) {
                    drv.setRegister(&PORTA, (int) qPrioAusgang.front());
                    qPrioAusgang.pop();
                } else {
                    drv.setRegister(&PORTA, (int) pufferDaten1);
                    qAusgang.pop(); 
                }
            } else {
                drv.setRegister(&PORTA, (int) pufferDaten1);
                qAusgang.pop(); 
            }


            
        } else {
            if (!qPrioAusgang.empty()) {
            //std::cerr << std::bitset<8>(qPrioAusgang.front()) << std::endl;
            //drv.setRegister(&DDRA, 0b00001111);
            drv.setRegister(&PORTA, (int) qPrioAusgang.front());
            qPrioAusgang.pop();
        }
        }
    }
    
    
    
        
}

void B15Schnittstelle::addQPrioAusgangDaten(char vierBit) {       
        //Überprüfe, ob gleiches Byte gesendet wird -> Steuerzeichen einfügen
        if (vorherigEingefuegt == vierBit) {
            qPrioAusgang.push(steuer1);
            qPrioAusgang.push(steuer2);
            if (vierBit == 0b00000001) {
                qPrioAusgang.push(0b00000000);
                vorherigEingefuegt = 0b00000000;
            } else {
                qPrioAusgang.push(0b00000001);
                vorherigEingefuegt = 0b00000001;
            }
        }

    //Überprüfe, ob Anfang des Steuerzeichens gesendet wird -> Steuerzeichen einfügen
        if (vierBit == steuer1) {
            qPrioAusgang.push(vierBit);
            qPrioAusgang.push(steuer2);
            qPrioAusgang.push(0b00001011);
            vorherigEingefuegt = 0b00001011;
        } else {
            qPrioAusgang.push(vierBit);
            vorherigEingefuegt = vierBit;
            
            /*Nicht benötigt
            if (vorherigEingefuegt == steuer1 && vierBit == steuer2) {
                qAusgang.push(0b00001100); //Daten entsprechen Steuerzeichen
            }
            */
        }
    

}

void B15Schnittstelle::addQPrioAusgangSteuersignal(char steuercode) {
    qPrioAusgang.push(steuer1);
    qPrioAusgang.push(steuer2);
    qPrioAusgang.push(steuercode);
    vorherigEingefuegt = steuercode;
}

void B15Schnittstelle::addQAusgangDaten(char vierBit) {
        
        
        //Überprüfe, ob gleiches Byte gesendet wird -> Steuerzeichen einfügen
        if (vorherigEingefuegt == vierBit) {
            qAusgang.push(steuer1);
            qAusgang.push(steuer2);
            if (vierBit == 0b00000001) {
                qAusgang.push(0b00000000);
                vorherigEingefuegt = 0b00000000;
            } else {
                qAusgang.push(0b00000001);
                vorherigEingefuegt = 0b00000001;
            }
        }

    //Überprüfe, ob Anfang des Steuerzeichens gesendet wird -> Steuerzeichen einfügen
        if (vierBit == steuer1) {
            qAusgang.push(vierBit);
            qAusgang.push(steuer2);
            qAusgang.push(0b00001011);
            vorherigEingefuegt = 0b00001011;
        } else {
            qAusgang.push(vierBit);
            vorherigEingefuegt = vierBit;
            
            /*Nicht benötigt
            if (vorherigEingefuegt == steuer1 && vierBit == steuer2) {
                qAusgang.push(0b00001100); //Daten entsprechen Steuerzeichen
            }
            */
        }
    

}

void B15Schnittstelle::addQAusgangSteuersignal(char steuercode) {
    qAusgang.push(steuer1);
    qAusgang.push(steuer2);
    qAusgang.push(steuercode);
    vorherigEingefuegt = steuercode;
}

void B15Schnittstelle::DEBUGAusgangsQueue() {   
    do {
        std::cerr << std::bitset<8>(qAusgang.front()) << std::endl;
        qAusgang.pop();
    } while(!qAusgang.empty());

    std::cerr << "DEBUG " << sendeObjekt.getDEBUGNr() << std::endl;
}

void B15Schnittstelle::TESTEVerarbeitungEingang() {   
    int falschesByteeinschieben = 0;
    char vorg = 0;
    
    do {
        falschesByteeinschieben++;
        //std::cerr << std::bitset<8>(qAusgang.front()) << std::endl;
        char zwischen = qAusgang.front();
        zwischen = zwischen << 4;
        eingang4Bit(zwischen);
        //std::cerr << std::bitset<8>(zwischen) << std::endl;
        if (zwischen == vorg) {std::cerr << "Achtung: gleiches Zeichen gefunden" <<std::endl;}
        vorg = zwischen;
        if (falschesByteeinschieben == 120) {
            eingang4Bit(0b00000000);
        }

        qAusgang.pop();
    } while(!qAusgang.empty());


    if (!qPrioAusgang.empty()) do {
        char zwischen = qPrioAusgang.front();
        zwischen = zwischen << 4;
        //std::cerr << std::bitset<8>(zwischen) << std::endl;
        eingang4Bit(zwischen);
        qPrioAusgang.pop();
    } while(!qPrioAusgang.empty());


    if (!qAusgang.empty()) do {
        char zwischen = qAusgang.front();
        zwischen = zwischen << 4;
        //std::cerr << std::bitset<8>(zwischen) << std::endl;
        eingang4Bit(zwischen);
        qAusgang.pop();
    } while(!qAusgang.empty()); 
}

char B15Schnittstelle::getEingang() {
    //drv.setRegister(&DDRA, 0b00000000);
    return drv.getRegister(&PINA);
    //return 'a';
}

void B15Schnittstelle::eingang4Bit(char vierBit){
    //std::cerr << std::bitset<8>(vierBit) << std::endl;
    eingangDatenVorgaenger = eingangDaten;
    eingangDaten = vierBit;
    //std::cerr << "Verarbeite Paar:" << std::bitset<8>(eingangDaten) << " " << std::bitset<8>(eingangDatenVorgaenger) << std::endl;

    if (warteZweiZyklen > 0)    //Warten, bis Befehlscode aus dem Weg
        warteZweiZyklen--;
    else {
        if (warteZweiZyklen<=0 && steuerSigErkannt) {
            warteZweiZyklen = 1;
            steuerSig = eingangDaten;

            //Fälle
            switch (steuerSig) {
                case (char) 0b00000000: {
                    //std::cerr << "Befehl: Ignorieren" << std::endl;
                    //Unsichtbares Verhalten
                    break;
                }
                case (char) 0b00010000: {
                    //std::cerr << "Befehl: Ignorieren" << std::endl;
                    //Unsichtbares Verhalten
                    break;
                }
                case (char)0b00100000: {
                    std::cerr << "Blockanzahl empfangen: ";
                    //Anz Blöcke empfangen
                    uebertragungAktiv = true;     
                    bekommeAnzBloecke = true;
                    verbleibendeHalbbitsSteuerZahl = 8;
                    startClock = std::chrono::high_resolution_clock::now();
                    break;
                }
                case (char) 0b00110000: {
                    std::cerr << "Blocknummer empfangen: ";
                    //Blocknummer empfangen
                    bekommeBlockNr = true;
                    verbleibendeHalbbitsSteuerZahl = 8;
                    break;
                }
                case (char) 0b01000000: {
                    std::cerr << "Blocklänge empfangen: ";
                    //Blocklänge empfangen
                    bekommeBlockLaenge = true;
                    verbleibendeHalbbitsSteuerZahl = 2;
                    break;
                }
                case (char) 0b01100000: {
                    std::cerr << "Paritätssumme empfangen: ";
                    //Parität bekommen
                    bekommeParitaet = true;
                    verbleibendeHalbbitsSteuerZahl = 4;
                    break;
                }
                case (char) 0b01110000: {
                    //Block erneut anfordern
                    std::cerr << "Zum erneut senden einreihen: Block ";
                    sendeBlockErneut = true;
                    verbleibendeHalbbitsSteuerZahl = 8;
                    break;
                }
                case (char) 0b10000000: {
                    //Andere Seite fragt Ende an
                    std::cerr << "Ende angefordert ";
                    if (empfangsObjekt.checkParitaetBestanden()) {
                        std::cerr << "- Übertragung erfolgreich";
                        auto endClock = std::chrono::high_resolution_clock::now();
                        int zeit = std::chrono::duration_cast<std::chrono::seconds>(endClock - startClock).count();
                        if (zeit<=0) {
                          zeit=1;  
                        } 
                        std::cerr << " ("<< ((int) empfangsObjekt.getAnzBytes()/zeit) << " Byte/s in " << zeit << " s)" << std::endl;
                        uebertragungAktiv = false;  
                        empfangsObjekt.schreibeBinFile();
                        
                    } else {
                        std::cerr << "- Übertragung gescheitert" << std::endl;
                        
                        //gescheiterte Blöcke erneut senden
                        std::queue<unsigned long> wdh = empfangsObjekt.getWiederholer();
                        for (unsigned long int i=0; i<wdh.size(); ++i) {
                            pushAusgangBlockErneut(wdh.front());
                            wdh.pop();
                        }
                        addQPrioAusgangSteuersignal(0b00001001);
                    }
                    break;
                }
                case (char) 0b10010000: {
                    //Empfänger möchte Senden beenden Anfrage
                    std::cerr << "Einreihen beendet" << std::endl;
                    addQPrioAusgangSteuersignal(0b00000000);
                    addQPrioAusgangSteuersignal(0b00001000);
                    
                    prioAktiv = true;
                    while(!qPrioAusgang.empty()) {
                        aktuellGelesen = filterAusgangInvert(getEingang());
                        if (vorherigGelesen != aktuellGelesen) {
                            //std::cerr << "Gelesen: " << std::bitset<8>(aktuellGelesen) << std::endl;
                            eingang4Bit(aktuellGelesen);
                            vorherigGelesen = aktuellGelesen;
                        }

                        schreibeNextAusgang();

                        drv.delay_ms(1);

                        aktuellGelesen = filterAusgangInvert(getEingang());
                        if (vorherigGelesen != aktuellGelesen) {
                            //std::cerr << "Gelesen: " << std::bitset<8>(aktuellGelesen) << std::endl;
                            eingang4Bit(aktuellGelesen);
                            vorherigGelesen = aktuellGelesen;
                        }
                    }
                    prioAktiv = false;
                    
                    
                    break;
                }
                case (char) 0b10110000: {
                    //Daten entsprechen Anfang des Befehlszeichen
                    
                    if (leseDaten) {
                        //std::cerr << verbleibendeHalbbytes;
                        verbleibendeHalbbytes--; 
                        if (erstesHalbbitGelesen == false) {
                            erstesHalbbitGelesen = true;
                            halbbitPuffer = 0b10100000;
                            //std::cerr << " Vorne : " << std::bitset<8>(halbbitPuffer) << std::endl;
                        } else {

                            halbbitPuffer |= (0b00001010);
                            erstesHalbbitGelesen = false;
                            datenPuffer[aktuellerDatenpufferIndex] = halbbitPuffer;
                            aktuellerDatenpufferIndex++;
                            //std::cerr /*<< " Hinten: "*/ << std::bitset<8>(halbbitPuffer) << std::endl;
                            halbbitPuffer = 0;
                            
                            if (verbleibendeHalbbytes <= 0) {
                                //std::cerr << "Speichere Datenobjekt" << std::endl;     
                                empfangsObjekt.setDaten(aktuelleBlockNr, datenPuffer, sollParitaet);
                                delete[] datenPuffer;
                                leseDaten = false;
                            }
                        }
                    } else {
                        if (verbleibendeHalbbitsSteuerZahl > 0) {       //Fall: Es werden noch Teile der Steuerzahl erwartet, Steuerzahl zusammensetzen
                            if (steuerZahl >= 0) {
                                steuerZahl = steuerZahl << 4;
                            } else {
                                steuerZahl = 0;
                            }
                            for (int k=4;k<8; ++k) {
                            steuerZahl |= 0b00001010;
                            }  
                            verbleibendeHalbbitsSteuerZahl--;
                            if (verbleibendeHalbbitsSteuerZahl <=0) {
                                //Steuerzahlerwartung schließen und Ereignis auslösen!
                                std::cerr << (int) steuerZahl << std::endl;
                                if (bekommeAnzBloecke) {
                                    bekommeAnzBloecke = false;
                                    empfangsObjekt.setAnzBloecke(steuerZahl);
                                    steuerZahl = -1;
                                } else {
                                if (bekommeBlockNr) {
                                        bekommeBlockNr = false;
                                        flagBlockNr = true;
                                        aktuelleBlockNr = steuerZahl;
                                        steuerZahl = -1;
                                    } else {
                                        if (bekommeBlockLaenge) {
                                            flagBlockLaeng = true;
                                            bekommeBlockLaenge = false; 
                                            empfangsObjekt.setBlocklaenge(aktuelleBlockNr,steuerZahl);
                                            verbleibendeHalbbytes = steuerZahl*2;  
                                            aktuellerDatenpufferIndex = 0;
                                            steuerZahl = -1;
                                        } else {
                                            if (bekommeParitaet) {
                                                sollParitaet = steuerZahl;  
                                                bekommeParitaet = false;
                                                if (flagBlockNr && flagBlockLaeng) {
                                                    //std::cerr << "Datenblock beginnt" << std::endl;
                                                    datenPuffer = new char[verbleibendeHalbbytes/2];
                                                    flagBlockNr = false;
                                                    flagBlockLaeng = false;
                                                    leseDaten = true;
                                                }
                                                steuerZahl = -1;
                                            } else {
                                                if (sendeBlockErneut)  {
                                                    pushAusgangBlock(sendeObjekt.getBlock(steuerZahl));
                                                    addQAusgangSteuersignal(0b00000001);
                                                    sendeBlockErneut = false; 
                                                    steuerZahl = -1;
                                                }
                                            }
                                        }
                                    } 
                                }
                            }
                        }
                    }

                    break;
                }
                case (char) 0b11000000: {
                    //Verbindung etablieren - Empfänger eröffnet Verbindung
                    verbindungEtabliert = true;
                    std::cerr << "Verbindung erfolgreich hergestellt" << std::endl;
                    break;
                }
                default: {
                    std::cerr << "Es ist ein Fehler aufgetreten: Unbekanntes Befehlszeichen" << std::endl;
                }
            }
            steuerSigErkannt = false;
        } else {
            if (eingangDatenVorgaenger == (char) 0b10100000 && eingangDaten == (char) 0b01010000) {
                steuerSigErkannt = true;
                //std::cerr << "Steuersignal!" << std::endl;
            } else {
                if (verbleibendeHalbbitsSteuerZahl > 0) {       //Fall: Es werden noch Teile der Steuerzahl erwartet, Steuerzahl zusammensetzen
                        if (steuerZahl >= 0) {
                            steuerZahl = steuerZahl << 4;
                        } else {
                            steuerZahl = 0;
                        }
                        for (int k=4;k<8; ++k) {
                        steuerZahl |= (eingangDatenVorgaenger >> (k) & 1) << (-4+k);
                        }  
                        verbleibendeHalbbitsSteuerZahl--;
                        if (verbleibendeHalbbitsSteuerZahl <=0) {
                            //Steuerzahlerwartung schließen und Ereignis auslösen!
                            std::cerr << (int) steuerZahl << std::endl;
                            if (bekommeAnzBloecke) {
                                bekommeAnzBloecke = false;
                                empfangsObjekt.setAnzBloecke(steuerZahl);
                                steuerZahl = -1;
                            } else {
                            if (bekommeBlockNr) {
                                    bekommeBlockNr = false;
                                    flagBlockNr = true;
                                    aktuelleBlockNr = steuerZahl;
                                    steuerZahl = -1;
                                } else {
                                    if (bekommeBlockLaenge) {
                                        flagBlockLaeng = true;
                                        bekommeBlockLaenge = false; 
                                        empfangsObjekt.setBlocklaenge(aktuelleBlockNr,steuerZahl);
                                        verbleibendeHalbbytes = steuerZahl*2;  
                                        aktuellerDatenpufferIndex = 0;
                                        steuerZahl = -1;
                                    } else {
                                        if (bekommeParitaet) {
                                            sollParitaet = steuerZahl;  
                                            bekommeParitaet = false;
                                            if (flagBlockNr && flagBlockLaeng) {
                                                //std::cerr << "Datenblock beginnt" << std::endl;
                                                datenPuffer = new char[verbleibendeHalbbytes/2];
                                                flagBlockNr = false;
                                                flagBlockLaeng = false;
                                                leseDaten = true;
                                            }
                                            steuerZahl = -1;
                                        } else {
                                            if (sendeBlockErneut)  {
                                                //zwischen.berechneParitaet();
                                                //std::cerr << "Parität richtiger Block: " << zwischen.getParitaet() << std::endl;
                                                pushAusgangBlock(sendeObjekt.getBlock(steuerZahl));
                                                addQAusgangSteuersignal(0b00000001);
                                                sendeBlockErneut = false; 
                                                steuerZahl = -1;
                                            }
                                        }
                                    }
                                } 
                            }
                        }
                } else {
                    //Fall: Lese Daten
                    if (leseDaten) {
                        //std::cerr << verbleibendeHalbbytes << std::endl;
                        verbleibendeHalbbytes--; 
                        if (erstesHalbbitGelesen == false) {
                            erstesHalbbitGelesen = true;
                            halbbitPuffer = eingangDatenVorgaenger;
                            //std::cerr << " Vorne : " << std::bitset<8>(halbbitPuffer) << std::endl;
                        } else {

                            halbbitPuffer |= ((unsigned char) eingangDatenVorgaenger >> 4);
                            erstesHalbbitGelesen = false;
                            datenPuffer[aktuellerDatenpufferIndex] = halbbitPuffer;
                            //std::cerr << " Vorne : " << std::bitset<8>(halbbitPuffer) << std::endl;
                            aktuellerDatenpufferIndex++;
                            halbbitPuffer = 0;
                            
                            if (verbleibendeHalbbytes <= 0) {
                                //std::cerr << "Speichere Datenobjekt" << std::endl;                              
                                empfangsObjekt.setDaten(aktuelleBlockNr, datenPuffer, sollParitaet);
                                //std::cerr << "Block 0, Byte 0: " << std::bitset<8>(empfangsObjekt->getBlock(0).getByte(0));
                                delete[] datenPuffer;
                                leseDaten = false;
                            }
                        }
                    }
                }
            }
        }
    }
}

void B15Schnittstelle::DEBUGKonsolenAusgabe(char ausgabe) {
    std::cerr << std::bitset<8>(ausgabe) << std::endl;
}

void B15Schnittstelle::sendenBeendenAnfrage() {
    addQAusgangSteuersignal(0b00001000);
}

void B15Schnittstelle::sendeGeladeneDatei() {
    pushAnzBloecke(sendeObjekt.getAnzBloecke());
    for (long unsigned int i=0; i<sendeObjekt.getAnzBloecke(); ++i) {
        pushAusgangBlock(sendeObjekt.getBlock(i));
    }
    sendenBeendenAnfrage();
    addQPrioAusgangSteuersignal(0b00000000);
}

bool B15Schnittstelle::getVebindungEtabliert() {
    return verbindungEtabliert;
}

void B15Schnittstelle::etabliereVerbindung() {
    //Warte auf Bereitsignal von anderer Seite.
    drv.setRegister(&DDRA, 0b00001111);
    do {
        addQPrioAusgangSteuersignal(0b00001100);
        for (int i=0; i<3; ++i) {
            aktuellGelesen = filterAusgangInvert(getEingang());
            if (vorherigGelesen != aktuellGelesen) {
                //std::cerr << "Gelesen: " << std::bitset<8>(aktuellGelesen) << std::endl;
                eingang4Bit(aktuellGelesen);
                vorherigGelesen = aktuellGelesen;
            }
    
            schreibeNextAusgang();

            aktuellGelesen = filterAusgangInvert(getEingang());
            if (vorherigGelesen != aktuellGelesen) {
                //std::cerr << "Gelesen: " << std::bitset<8>(aktuellGelesen) << std::endl;
                eingang4Bit(aktuellGelesen);
                vorherigGelesen = aktuellGelesen;
            }
            drv.delay_ms(1);
        }
    } while (schnittstelle.getVebindungEtabliert()==false);

     addQPrioAusgangSteuersignal(0b00001100);
     while(!qPrioAusgang.empty()) {
        aktuellGelesen = filterAusgangInvert(getEingang());
        if (vorherigGelesen != aktuellGelesen) {
            //std::cerr << "Gelesen: " << std::bitset<8>(aktuellGelesen) << std::endl;
            eingang4Bit(aktuellGelesen);
            vorherigGelesen = aktuellGelesen;
        }

        schreibeNextAusgang();
        drv.delay_ms(1);
     }

    
     prioAktiv = false;

}

void B15Schnittstelle::schreibeGeladeneDatei() {
    vorherigGelesen = 0;
    aktuellGelesen = 0;

    do {
        aktuellGelesen = filterAusgangInvert(getEingang());
        if (vorherigGelesen != aktuellGelesen) {
            //std::cerr << "Gelesen: " << std::bitset<8>(aktuellGelesen) << std::endl;
            eingang4Bit(aktuellGelesen);
            vorherigGelesen = aktuellGelesen;
        }
        schreibeNextAusgang();

        aktuellGelesen = filterAusgangInvert(getEingang());
        if (vorherigGelesen != aktuellGelesen) {
            //std::cerr << "Gelesen: " << std::bitset<8>(aktuellGelesen) << std::endl;
            eingang4Bit(aktuellGelesen);
            vorherigGelesen = aktuellGelesen;
        }
        drv.delay_ms(1);


    } while(!qAusgang.empty() || uebertragungAktiv);

}






