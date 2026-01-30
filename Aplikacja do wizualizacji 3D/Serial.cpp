#include "Serial.h"
#include <stdio.h>

Serial::Serial(const char *portName) {
    this->connected = false;

    // Próba otwarcia portu
    this->hSerial = CreateFileA(portName,
            GENERIC_READ | GENERIC_WRITE,
            0,
            NULL,
            OPEN_EXISTING,
            FILE_ATTRIBUTE_NORMAL,
            NULL);

    if (this->hSerial == INVALID_HANDLE_VALUE) {
        if (GetLastError() == ERROR_FILE_NOT_FOUND) {
            printf("ERROR: Port %s not found.\n", portName);
        }
        else {
            printf("ERROR: Unknown connection error.\n");
        }
    }
    else {
        DCB dcbSerialParams = {0};

        // Konfiguracja parametrów (musi pasować do Arduino!)
        if (!GetCommState(this->hSerial, &dcbSerialParams)) {
            printf("failed to get current serial parameters!");
        }
        else {
        dcbSerialParams.BaudRate = CBR_115200; // Prętkość komunikacji
            dcbSerialParams.ByteSize = 8;  // 8 bitów danych - stadardowa konfiguracja dla UART
            dcbSerialParams.StopBits = ONESTOPBIT; // 1 bit stopu - standardowa konfiguracja
            dcbSerialParams.Parity = NOPARITY; // Brak kontroli parzystośći - standardowa konfiguracja
            dcbSerialParams.fDtrControl = DTR_CONTROL_ENABLE; // Reset Arduino przy połączeniu

            if (!SetCommState(hSerial, &dcbSerialParams)) {
                printf("ALERT: Could not set Serial Port parameters");
            }
            else {
                this->connected = true;
                PurgeComm(this->hSerial, PURGE_RXCLEAR | PURGE_TXCLEAR); // Czyścimy bufor
                Sleep(2000); // Czekamy 2s na restart Arduino (autoreset)
            }
        }
    }
}

Serial::~Serial() {
    if (this->connected) {
        this->connected = false;
        CloseHandle(this->hSerial);
    }
}

int Serial::readData(char *buffer, unsigned int nbChar) {
    DWORD bytesRead;
    unsigned int toRead = 0;

    ClearCommError(this->hSerial, &this->errors, &this->status);

    if (this->status.cbInQue > 0) {
        if (this->status.cbInQue > nbChar) {
            toRead = nbChar;
        } else {
            toRead = this->status.cbInQue;
        }

        if (ReadFile(this->hSerial, buffer, toRead, &bytesRead, NULL)) {
            return bytesRead;
        }
    }
    return 0;
}

bool Serial::writeData(const char *buffer, unsigned int nbChar) {
    DWORD bytesSend;

    if (!WriteFile(this->hSerial, (void *)buffer, nbChar, &bytesSend, 0)) {
        ClearCommError(this->hSerial, &this->errors, &this->status);
        return false;
    }
    else {
        return true;
    }
}

bool Serial::isConnected() {
    return this->connected;
}

void Serial::closeSerial() {
    CloseHandle(this->hSerial);
    this->connected = false;
}
