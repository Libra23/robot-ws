#ifndef UTILITY_H
#define UTILITY_H

#include <memory.h> 

void ConvertFromBuffer(const void* buf, void* data, size_t size) {
    memmove(data, buf, size);
}

#endif