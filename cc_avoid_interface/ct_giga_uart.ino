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

Coord current_position = {0, 0, 12, 15};


// Read one UART line with timeout
bool readLine(char* buffer, int maxLen, unsigned long timeoutMs) {
    unsigned long start = millis();

    while (Serial.available() == 0 && millis() - start < timeoutMs) {
        // wait for data
    }

    if (Serial.available() == 0) {
        return false;
    }

    int n = Serial.readBytesUntil('\n', buffer, maxLen - 1);
    buffer[n] = '\0';

    return true;
}


// Convert received text into Coord
bool parseCoord(const char* text, Coord& out) {
    float x;
    float y;
    float speed;
    float heading;

    int matched = sscanf(text, "(%f, %f, %f, %f)", &x, &y, &speed, &heading);

    if (matched != 4) {
        return false;
    }

    out.x = x;
    out.y = y;
    out.speed = speed;
    out.heading = heading;

    return true;
}


// Send current position to Pi
void sendCurrentPosition() {
    char buffer[80];

    snprintf(buffer, sizeof(buffer),
             "POS,(%.2f, %.2f, %.2f, %.2f)",
             current_position.x,
             current_position.y,
             current_position.speed,
             current_position.heading);

    Serial.println(buffer);
}


// Send convoy path to Pi
void sendConvoyPath() {
    macro_path.transmit();
    Serial.println("END");
}


void setup() {
    Serial.begin(115200);

    Coord test  = {.x = 0,    .y = 0,    .speed = 12,    .heading = 15};
    Coord test2 = {.x = 1.12, .y = 3,    .speed = 22.4,  .heading = 10};
    Coord test3 = {.x = 3,    .y = 3,    .speed = 0.992, .heading = 90};
    Coord test4 = {.x = 5.01, .y = 3.14, .speed = 18.246,.heading = 17};

    macro_path.insert(test);
    macro_path.insert(test2);
    macro_path.insert(test3);
    macro_path.insert(test4);

    Serial.println("GIGA UART READY");
}


void loop() {
    while (Serial.available()) {
        char buffer[128];

        int n = Serial.readBytesUntil('\n', buffer, sizeof(buffer) - 1);
        buffer[n] = '\0';

        std::string line = buffer;

        // Pi requests current position
        if (line == "GET_POS") {
            sendCurrentPosition();
            continue;
        }

        // Pi requests convoy path
        if (line == "GET_PATH") {
            sendConvoyPath();
            continue;
        }

        // Keep old Start command for compatibility
        if (line == "Start") {
            sendConvoyPath();
            continue;
        }

        // Pi is ready to send a generated micro path
        if (line == "Ready") {
            Serial.println("Initiate");

            // Read number of waypoints
            if (!readLine(buffer, sizeof(buffer), 600)) {
                Serial.println("Incomplete");
                continue;
            }

            micro_path_size = atoi(buffer);

            // Validate path size
            if (micro_path_size <= 0 || micro_path_size > 1000) {
                Serial.println("Incomplete");
                continue;
            }

            // Clear old micro path
            delete[] micro_path;
            micro_path = new Coord[micro_path_size];

            bool complete = true;

            // Read each waypoint from Pi
            for (int i = 0; i < micro_path_size; i++) {
                if (!readLine(buffer, sizeof(buffer), 600)) {
                    complete = false;
                    break;
                }

                if (!parseCoord(buffer, micro_path[i])) {
                    complete = false;
                    break;
                }
            }

            // If something failed, clear the partial path
            if (!complete) {
                Serial.println("Incomplete");

                delete[] micro_path;
                micro_path = nullptr;
                micro_path_size = 0;

                continue;
            }

            Serial.println("Done");
        }
    }
}
