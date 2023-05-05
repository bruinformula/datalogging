#include <fstream>
#include <iostream>
#include <stdint.h>
#include <vector>

#include "lib/MLX90640.h"

using namespace std;

// build with `g++ main.cpp lib/*.cpp -o frame-data-converter


//split shitass string by char
std::vector<std::string> splitString(const std::string& str, char c)
{
    std::vector<std::string> tokens;

    std::string::size_type pos = 0;
    std::string::size_type prev = 0;
    while ((pos = str.find(c, prev)) != std::string::npos) {
        tokens.push_back(str.substr(prev, pos - prev));
        prev = pos + 1;
    }
    tokens.push_back(str.substr(prev));

    return tokens;
}

//parse shitass data from raw dash log, shits it out to a file
void shitassData(char* fname) {
    fstream filestream;
    filestream.open(fname, ios::in);
    string rawIn = "";
    if (!filestream) {
        cout << "No such file";
    }
    else {
        char ch;

        while (1) {
            filestream >> ch;
            if (filestream.eof())
                break;

            rawIn.push_back(ch);
        }

    }
    filestream.close();

    vector<string> in = splitString(rawIn, '\n');
    vector<string> canMessages;

    vector<string> temp;
    for (std::string s : in) {
        temp = splitString(s, ',');
        if (temp[0] == "CAN") {
            canMessages.push_back(temp[4].substr(0, 3));
            canMessages.push_back(temp[4].substr(4, 7));
            canMessages.push_back(temp[4].substr(8, 11));
            canMessages.push_back(temp[4].substr(12, 15 ));
        }
    }

    fstream my_file;
    my_file.open("shitass can data from " + string(fname), ios::out);
    if (!my_file) {
        cout << "File not created!";
    }
    else {
        int i = 0;
        cout << "File created successfully!";
        for (std::string s : canMessages) {
            if (i % 24 == 0) my_file << '\n';
            my_file << s + " ";
            i++;
        }
        my_file.close();
    }
}

uint8_t makeByte(char* digits) {
    for (int i = 0; i < 2; i++) {
        if (digits[i] >= 'A') {
            digits[i] -= 'A';
            digits[i] += 10;
        }
        else {
            digits[i] -= '0';
        }
    }
    return 16 * digits[0] + digits[1];
}

bool ishexdig(char c) {
    return ((c >= '0') && (c <= '9')) || ((c >= 'A') && (c <= 'F'));
}

int loadFile(char* fname, uint8_t* buffer, int maxlen) {
    ifstream in;
    in.open(fname);
    char nextByte[2];
    int num_bytes = 0;
    for (num_bytes = 0; num_bytes < maxlen; num_bytes++) {
        int nybblePos = 0;
        for (;;) {
            in.read(nextByte + nybblePos, 1);
            if (in.fail() && (nybblePos == 1)) {
                fprintf(stderr, "Warning: An odd number of hex digits were provided in %s\r\n", fname);
                nextByte[1] = '0';
                buffer[num_bytes] = makeByte(nextByte);
                goto eof;
            }
            else if (in.fail() || in.eof()) {
                fprintf(stderr, "Warning: only %d bytes were parsesd from %s\r\n", num_bytes, fname);
                goto eof;
            }
            else {
                if (ishexdig(nextByte[nybblePos])) {
                    nybblePos++;
                    if (nybblePos == 2) {
                        buffer[num_bytes] = makeByte(nextByte);
                        goto eobyte;
                    }
                }
            }
        }
    eobyte:
        continue;
    }
eof:
    in.close();
    return num_bytes;
}

int main() {
    uint8_t eeData[1664] = { 0 };
    char ee_fname[] = "eeprom_data.hex";
    loadFile(ee_fname, eeData, 1664);
    MLX90640_DataProcessor proc;
    proc.loadCalibration(eeData);

    uint8_t frameData[1672] = { 0 };
    float pixels[768] = { 0 };
    char frame1_fname[] = "cpcb1.hex";
    loadFile(frame1_fname, frameData, 1672);
    proc.decodeFrame(frameData, pixels);
    char frame2_fname[] = "frame2_data.hex";
    loadFile(frame2_fname, frameData, 1672);
    proc.decodeFrame(frameData, pixels);

    for (int i = 0; i < 24; i++) {
        for (int j = 0; j < 32; j++) {
            fprintf(stdout, "%.5f%s", pixels[i * 32 + j], j == 31 ? "\n" : " ");
        }
    }
}