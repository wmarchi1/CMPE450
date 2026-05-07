#include <string>
#include <Arduino.h>

struct Coord {
    float x;
    float y;
    float speed;
    float heading;
};

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

    // Debug print (no heap usage)
    void transmit() {
        char buffer[80];
        uint64_t start = curr;

        while (start < end) {
            Coord& c = buff[start % capacity];

            snprintf(buffer, sizeof(buffer),
                     "(%.2f, %.2f, %.2f, %.2f)",
                     c.x, c.y, c.speed, c.heading);

            Serial.println(buffer);
            start++;
        }
    }

private:
    int capacity;
    uint64_t curr;
    uint64_t end;
    Coord* buff;
};
// int PATH_LEN = 5;
// std::string path[5] = {"(0, 0, 40)", "(1, 0, 70)", "(1, 3, 20)", "(3, 3, 0)", "(5, 3, 0)"};

Path macro_path(5);
std::string* micro_path = nullptr;
int micro_path_size = 0;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Coord test = {.x=0, .y=0, .speed=12, .heading=15};
  Coord test2 = {.x=1.12, .y=3, .speed=22.4, .heading=10};
  Coord test3 = {.x=3, .y=3, .speed=0.992, .heading=90};
  Coord test4 = {.x=5.01, .y=3.14, .speed=18.246, .heading=17};


  macro_path.insert(test);
  macro_path.insert(test2);
  macro_path.insert(test3);
  macro_path.insert(test4);
}

void loop() {
    while (Serial.available()) { 
        char buffer[128];
        int n = Serial.readBytesUntil('\n', buffer, sizeof(buffer) - 1);
        buffer[n] = '\0';
        std::string line = buffer;

        // ---------- Handle "Start" ----------
        if (line == "Start") {
            macro_path.transmit();
            Serial.println("END");
            continue;
        }

        // ---------- Handle "Ready" ----------
        if (line == "Ready") {
            Serial.println("Initiate"); 

            unsigned long timeout = 600;  //200ms
            unsigned long start = millis();
            while (Serial.available() == 0 && (millis() - start) < timeout) {} //small wait for UART buffer to be filled
            if (Serial.available() == 0) { // timeout: skip
                Serial.println("Incomplete");
                continue;
            }
            n = Serial.readBytesUntil('\n', buffer, sizeof(buffer) - 1);
            buffer[n] = '\0';
            micro_path_size = atoi(buffer);

            if (micro_path_size <= 0 || micro_path_size > 1000) continue;

            micro_path = new std::string[micro_path_size];

            // ---------- Read micro path lines ----------
            for (int i = 0; i < micro_path_size; i++) {
                Serial.println("here");
                while (Serial.available() == 0 && (millis() - start) < timeout) {}
                if (Serial.available() == 0) {
                    Serial.println("Incomplete");
                    break;
                }
                n = Serial.readBytesUntil('\n', buffer, sizeof(buffer) - 1);
                buffer[n] = '\0';
                micro_path[i] = buffer;
            }

            Serial.println("Done"); 

            delete[] micro_path;  //free memory
            micro_path = nullptr;
        }
    }
}
