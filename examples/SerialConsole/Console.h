// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <Arduino.h>
#include <algorithm>
#include <ctype.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

namespace Console {
char command[48];
size_t commandLength = 0;
bool commandOverflow = false;
// Buffer writes so a full UART cannot block motion updates.
char output[512];
size_t outputHead = 0, outputSize = 0;
unsigned long droppedMessages = 0;

void report(const char* format, ...) __attribute__((format(printf, 1, 2)));
void report(const char* format, ...) {
    char message[256];
    va_list args;
    va_start(args, format);
    const int length = vsnprintf(message, sizeof(message), format, args);
    va_end(args);
    if (length < 0 || static_cast<size_t>(length) >= sizeof(message) ||
        static_cast<size_t>(length) > sizeof(output) - outputSize) {
        ++droppedMessages;
        return;
    }
    const size_t tail = (outputHead + outputSize) % sizeof(output);
    const size_t first = std::min<size_t>(length, sizeof(output) - tail);
    memcpy(output + tail, message, first);
    memcpy(output, message + first, length - first);
    outputSize += length;
}

void drainOutput() {
    const int space = Serial.availableForWrite();
    if (space <= 0 || !outputSize) return;
    const size_t contiguous = std::min(outputSize, sizeof(output) - outputHead);
    const size_t written = Serial.write(reinterpret_cast<const uint8_t*>(output + outputHead),
                                        std::min<size_t>(space, contiguous));
    outputHead = (outputHead + written) % sizeof(output);
    outputSize -= written;
    if (droppedMessages && sizeof(output) - outputSize >= 128) {
        report("Serial backlog dropped %lu messages. Use status for current state.\n", droppedMessages);
        droppedMessages = 0;
    }
}

// Return at most one complete command per loop iteration.
const char* readCommand() {
    for (size_t i = 0; i < sizeof(command) && Serial.available() > 0; ++i) {
        const int c = Serial.read();
        if (c == '\n' || c == '\r') {
            while (commandLength && isspace(static_cast<unsigned char>(command[commandLength - 1]))) {
                --commandLength;
            }
            command[commandLength] = '\0';
            commandLength = 0;
            if (commandOverflow) {
                report("Invalid or oversized command.\n");
                commandOverflow = false;
                return nullptr;
            }
            const char* start = command;
            while (isspace(static_cast<unsigned char>(*start))) ++start;
            return *start ? start : nullptr;
        }
        if (c == '\b' || c == 127) {
            if (commandLength) --commandLength;
        } else if (c <= 0 || c > 126 || commandLength == sizeof(command) - 1) {
            commandOverflow = true;
        } else {
            command[commandLength++] = static_cast<char>(c);
        }
    }
    return nullptr;
}
}  // namespace Console
