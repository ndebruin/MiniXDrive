#include <Arduino.h>

// struct CoProcStructTX {
//   bool camFlash;    // 1 byte
//   // total: 1 byte
// };

struct CoProcStructRX {
    int64_t posX;   // 8 bytes
    int64_t posY;   // 8 bytes
    float yaw;      // 4 bytes
    // total: 20 bytes
};