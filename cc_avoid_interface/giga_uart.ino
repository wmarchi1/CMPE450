#include <Arduino.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Wire.h>


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

    //returns true if both coords are the same
    bool compare(Coord& c1, Coord& c2) {
      if (c1.x != c2.x)
        return false;

      if (c1.y != c2.y)
        return false;

      if (c1.heading != c2.heading)
        return false;

      if (c1.speed != c2.speed)
        return false;

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

    void transmit(Coord& currPos, Coord& targetPos) {
        Serial.println("ACK_START");

        
        Serial.write((uint8_t*)&currPos, sizeof(Coord));
        delay(10); // helps Pi not overflow buffer during test
        Serial.write((uint8_t*)&targetPos, sizeof(Coord));


        Serial.println("DONE_STREAM");        

    }

private:
    int capacity;
    uint64_t curr;
    uint64_t end;
    Coord* buff;
};

// RF — transceiver for sending micro_path checkpoint
RF24 radio(2, 3);
const byte address[6] = "00001";


// ---------------- TEST DATA ----------------
Path path(10);
Coord currPos = {5, 5, 0.1, 0};
Coord targetPos = {10, 15, 0.1, 90};

// ---------------- SETUP ----------------
void setup() {
    Serial.begin(115200);

    while (!Serial) {
        ; // wait for USB serial
    }

    delay(2000);

    SPI1.begin();
    radio.begin(&SPI1);
    radio.openWritingPipe(address);
    radio.setPALevel(RF24_PA_MIN);
    radio.stopListening();

    Serial.println("ARDUINO READY");

    // Populate test path
    for (int i = 0; i < 5; i++) {
        Coord c;
        c.x = i * 1.1;
        c.y = i * 2.2;
        c.speed = i * 3.3;
        c.heading = i * 10.0;

        path.insert(c);
    }
}

// ---------------- LOOP ----------------
void loop() {

    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();

        // ---------- TEST 1: STREAM FULL PATH ----------
        if (cmd == "Start") {
            // Serial.println("ACK_START");
            delay(100);

            path.transmit(currPos, targetPos);

        }

        // ---------- TEST 2: SINGLE PACKET HANDSHAKE ----------
        else if (cmd == "Ready") {
            Serial.println("ACK_READY");

            delay(50);

            Serial.println("Initiate");

            Coord received;
            Serial.readBytes((char*)&received, sizeof(Coord));
            
            radio.write(&received, sizeof(received));

            Serial.print("RX: ");
            Serial.print(received.x);
            Serial.print(", ");
            Serial.print(received.y);
            Serial.print(", ");
            Serial.print(received.speed);
            Serial.print(", ");
            Serial.println(received.heading);

            Serial.println("Done");
        }

        // ---------- UNKNOWN ----------
        else {
            Serial.println("UNKNOWN_CMD");
        }
    }
}
