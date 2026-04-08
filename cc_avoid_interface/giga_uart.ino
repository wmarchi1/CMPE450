#include <string>
#include <Arduino.h>

class Path {
public:
    Path(int cap) {
        capacity = (cap < 1) ? 1 : cap;
        buff = new std::string[capacity]();
        curr = 0;
        end = 0;
    }

    ~Path() {
        delete[] buff;
        buff = nullptr;
    }

    // Returns the oldest element (head)
    std::string getHead() {
        if (size() == 0)
            return "Error! Empty buff";
        return buff[curr % capacity];
    }

    // Returns the newest element (rear)
    std::string getRear() {
        if (size() == 0)
            return "Error! Empty buff";
        return buff[(end - 1) % capacity];
    }

    // Inserts a new element if there is space
    bool insert(const std::string& coord) {
        if (size() >= capacity) {
            // Buffer is full; cannot overwrite
            return false;
        }
        buff[end % capacity] = coord;
        end++;
        return true;
    }

    // Processes (reads) the next element if available
    std::string process() {
        if (size() == 0)
            return "Error! Empty buff";
        std::string coord = buff[curr % capacity];
        curr++;

        // Normalize to prevent overflow
        if (curr >= capacity * 5) {
            end -= curr;
            curr = 0;
        }
        return coord;
    }

    // Returns the number of unread elements
    uint64_t size() const {
        return end - curr;
    }

    bool move_curr(const std::string& coord) { // moves the curr position to the position of the given node
        uint64_t location = search(coord);

        if (location >= end)
            // coordinate doesn't exist
            return false;
        
        curr = location;
        return true;
        
    }

    uint64_t search(const std::string& coord) {
        uint64_t start = curr;

        while (start < end) {
            if (buff[start % capacity] == coord) {
                return start;
            }

            start++;
        }

        return start;
    }

    void transmit() { //sends data through UART
        uint64_t start = curr;

        while (start < end) {
            Serial.println(buff[start % capacity].c_str()); // println wants arduino's String class not the std::string class
            start++;
        }

    }

private:
    int capacity;
    uint64_t curr;  // consumer position
    uint64_t end;   // producer position
    std::string* buff;
};

// int PATH_LEN = 5;
// std::string path[5] = {"(0, 0, 40)", "(1, 0, 70)", "(1, 3, 20)", "(3, 3, 0)", "(5, 3, 0)"};

Path macro_path(5);
std::string* micro_path = nullptr;
int micro_path_size = 0;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  macro_path.insert("(0, 0, 40)");
  macro_path.insert("(1, 0, 70)");
  macro_path.insert("(1, 3, 20)");
  macro_path.insert("(3, 3, 0)");
  macro_path.insert("(5, 3, 0)");
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
