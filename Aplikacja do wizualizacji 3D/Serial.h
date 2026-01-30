#ifndef SERIAL_H
#define SERIAL_H

// --- POLECENIA DLA WINDOWSA, ŻEBY NIE WCHODZIŁ W DROGĘ RAYLIBOWI ---
#define WIN32_LEAN_AND_MEAN // Wyłącza rzadko używane części Windows API
#define NOGDI               // Wyłącza grafikę Windows (GDI) -> naprawia błąd Rectangle
#define NOUSER              // Wyłącza funkcje okienkowe -> naprawia błąd CloseWindow i DrawText
// -------------------------------------------------------------------

#include <windows.h>
#include <string>

class Serial {
private:
    HANDLE hSerial;
    bool connected;
    COMSTAT status;
    DWORD errors;

public:
    Serial(const char *portName);
    ~Serial();

    int readData(char *buffer, unsigned int nbChar);
    bool writeData(const char *buffer, unsigned int nbChar);
    bool isConnected();
    void closeSerial();
};

#endif