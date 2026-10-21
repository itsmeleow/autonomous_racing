#ifndef LIDAR_H
#define LIDAR_H

#include <stdint.h>
#include <stddef.h>

class Stream;

/* LD19 LiDAR sends packet length of 47 bytes with 12 data points per packet */
#define POINT_PER_PACK 12
#define PACKET_LENGTH 47
#define HEADER 0x54
#define VER_LEN 0x2C


typedef struct __attribute__((packed)) {
    uint16_t distance;
    uint8_t intensity;
} LidarPoint;

typedef struct __attribute__((packed)) {
    uint8_t     header;
    uint8_t     ver_len;
    uint16_t    speed;
    uint16_t    start_angle;
    LidarPoint point[POINT_PER_PACK];
    uint16_t    end_angle;
    uint16_t    timestamp;
    uint8_t     crc8;
} LiDARFrame;

// The application configures and owns the UART before calling begin().
class LiDAR {
public:
    void begin(Stream &serial);
    void reset();

    // Call frequently from loop(); consumes bytes already available on the UART.
    void update();
    bool available() const;
    // Returns false when no new frame is ready; leaves output unchanged.
    bool readFrame(LiDARFrame &output);

private:
    void processByte(uint8_t byte);
    bool decodePacket(LiDARFrame &output) const;

    Stream *serial_ = nullptr;
    uint8_t buffer_[sizeof(LiDARFrame)] = {};
    size_t buffered_ = 0;
    LiDARFrame frame_ = {};
    bool frameReady_ = false;
};

#endif
