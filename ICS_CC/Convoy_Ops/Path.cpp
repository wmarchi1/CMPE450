#include <Arduino.h>

#pragma pack(push, 1)

struct Coord {
    float x;
    float y;
    float speed;
    float heading;
};

#pragma pack(pop)


class Path {
public:
    Path(int cap) {
        capacity = (cap < 1) ? 1 : cap;
        buff = new Coord[capacity];   // one-time allocation
        curr = 0;
        end = 0;
    }

    ~Path() {
        delete[] buff;
        buff = nullptr;
    }

    // Insert new coordinate
    bool insert(const Coord& coord) {
        if (size() >= capacity) {
            return false; // no overwrite
        }
        buff[end % capacity] = coord;
        end++;
        return true;
    }

    // Get oldest element
    bool getHead(Coord& out) {
        if (size() == 0) return false;
        out = buff[curr % capacity];
        return true;
    }

    // Get newest element
    bool getRear(Coord& out) {
        if (size() == 0) return false;
        out = buff[(end + capacity - 1) % capacity];
        return true;
    }

    // Pop next element
    bool process(Coord& out) {
        if (size() == 0) return false;

        out = buff[curr % capacity];
        curr++;

        // Normalize to prevent overflow
        if (curr >= capacity * 5) {
            end -= curr;
            curr = 0;
        }

        return true;
    }

    uint64_t size() const {
        return end - curr;
    }

    
    void transmit() {
       for (uint64_t i = curr; i < end; i++) {
            Coord& c = buff[i % capacity];
            Serial.write((uint8_t*)&c, sizeof(Coord));
            delay(10); // helps Pi not overflow buffer during test
        }
    }

private:
    int capacity;
    uint64_t curr;
    uint64_t end;
    Coord* buff;
};
