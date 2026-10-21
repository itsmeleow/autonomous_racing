#include "lidar.h"
#include "crc_check.h"
#include <Arduino.h>
#include <iostream>

void LiDAR::begin(Stream &serial) {
    serial_ = &serial;
    reset();
}

void LiDAR::reset() {
    buffered_ = 0;
    frameReady_ = false;
    frame_ = {};
}

void LiDAR::update() {
    if (serial_ == nullptr) {
        return;
    }

    while (serial_->available() > 0) {
        const int byte = serial_->read();
        if (byte < 0) {
            break;
        }
        processByte(static_cast<uint8_t>(byte));
    }
}

bool LiDAR::available() const {
    return frameReady_;
}

bool LiDAR::readFrame(LiDARFrame &output) {
    if (!frameReady_) {
        return false;
    }
    output = frame_;
    frameReady_ = false;
    return true;
}


/* Builds the data packet */
void LiDAR::processByte(uint8_t byte) {
    if (buffered_ == 0 && byte != HEADER) {
        return;
    }

    buffer_[buffered_++] = byte;
    if (buffered_ < sizeof(buffer_)) {
        return;
    }

    LiDARFrame decoded = {};
    if (decodePacket(decoded)) {
        frame_ = decoded;
        frameReady_ = true;
        buffered_ = 0;
        return;
    }

    // Slide past a rejected candidate, retaining any possible header inside it.
    size_t next = 1;
    while (next < buffered_ && buffer_[next] != HEADER) {
        ++next;
    }
    buffered_ -= next;
    for (size_t i = 0; i < buffered_; ++i) {
        buffer_[i] = buffer_[i + next];
    }
}

/* Decode packet according to the LD19 sensor protocol.
Packet is validated before decoded and stored in a LiDARFrame */
bool LiDAR::decodePacket(LiDARFrame &output) const {
    /* check valid packet length, header, and ver_len */
    if (buffered_ != PACKET_LENGTH) {
        std::cout << "invalid packet length: " << buffered_;
        return false;
    }
    if (buffer_[0] != HEADER || buffer_[1] != VER_LEN) {
        std::cout << "invalid header or ver_len";
        return false;
    }

    /* perform CRC check to validate packet before decoding */
    const uint8_t received_crc = buffer_[PACKET_LENGTH - 1];
    const uint8_t calculated_crc = calCRC8(buffer_, PACKET_LENGTH - 1);
    if (received_crc != calculated_crc) {
        return false;
    }

    /* now we can decode after validation */
    LiDARFrame temp = {};

    temp.header = buffer_[0];
    temp.ver_len = buffer_[1];
    temp.speed = static_cast<uint16_t>(buffer_[2]) | (static_cast<uint16_t>(buffer_[3]) << 8);
    temp.start_angle = static_cast<uint16_t>(buffer_[4]) | (static_cast<uint16_t>(buffer_[5]) << 8);

    for (size_t i = 0; i < POINT_PER_PACK; ++i) {
        const size_t offset = 6 + i * 3;
        temp.point[i].distance = static_cast<uint16_t>(buffer_[offset]) | (static_cast<uint16_t>(buffer_[offset + 1]) << 8);
        temp.point[i].intensity = buffer_[offset + 2];
    }

    temp.end_angle = static_cast<uint16_t>(buffer_[42]) | (static_cast<uint16_t>(buffer_[43]) << 8);
    temp.timestamp = static_cast<uint16_t>(buffer_[44]) | (static_cast<uint16_t>(buffer_[45]) << 8);
    temp.crc8 = received_crc;

    output = temp;
    return true;
}
