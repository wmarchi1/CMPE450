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
        buff = new Coord[capacity];
        curr = 0;
        end = 0;
    }

    ~Path() {
        delete[] buff;
    }

    bool insert(const Coord& coord) {
        if (size() >= capacity) return false;
        buff[end % capacity] = coord;
        end++;
        return true;
    }

    uint64_t size() const {
        return end - curr;
    }

    void transmit() {
        Serial.println("ACK_START");

        for (uint64_t i = curr; i < end; i++) {
            Coord& c = buff[i % capacity];
            Serial.write((uint8_t*)&c, sizeof(Coord));
            delay(10); // helps Pi not overflow buffer during test
        }
        Serial.println("DONE_STREAM");
        
    }

private:
    int capacity;
    uint64_t curr;
    uint64_t end;
    Coord* buff;
};
