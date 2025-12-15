#include <Adafruit_GPS.h>

Adafruit_GPS GPS(&Serial1);

// ------------------------ CONFIG ------------------------
static const uint32_t GPS_BAUD = 9600;

// Distance interval to place new checkpoints (meters)
float checkpointIntervalM = 50.0f;

// How close a follower must get to count as "reached" (meters)
float reachRadiusM = 10.0f;

// Max checkpoints stored (ring buffer)
static const int MAX_CKPTS = 20;

// Max number of followers tracked
static const int MAX_FOLLOWERS = 5;
// -------------------------------------------------------

struct Checkpoint {
  double lat;
  double lon;
  uint32_t createdMs;
  uint32_t seq;
};

struct FollowerState {
  bool active = false;
  int id = -1;
  uint32_t lastSeenMs = 0;
  uint32_t reachedSeq = 0;   // highest checkpoint sequence reached
};

Checkpoint ckpts[MAX_CKPTS];
int ckptHead = 0;     // points to oldest
int ckptCount = 0;
uint32_t ckptSeqCounter = 0;

FollowerState followers[MAX_FOLLOWERS];

// Manual start / delayed start state
bool checkpointingEnabled = false;  // true when we are generating checkpoints
bool pendingStart = false;          // true when START (possibly delayed) is armed
unsigned long startDelayMs = 0;
unsigned long startTriggerMs = 0;

// Last GPS fix used for interval tracking
bool haveLastFix = false;
double lastFixLat = 0.0;
double lastFixLon = 0.0;

// Accumulator distance since last checkpoint
float distSinceLastCheckpointM = 0.0;

String serialLine;

// ------------------------ MATH ------------------------
static double deg2rad(double d) { return d * 0.017453292519943295; }

// Haversine distance in meters
static float distanceMeters(double lat1, double lon1, double lat2, double lon2) {
  const double R = 6371000.0; // meters
  double dLat = deg2rad(lat2 - lat1);
  double dLon = deg2rad(lon2 - lon1);

  double a = sin(dLat / 2) * sin(dLat / 2) +
             cos(deg2rad(lat1)) * cos(deg2rad(lat2)) *
             sin(dLon / 2) * sin(dLon / 2);

  double c = 2 * atan2(sqrt(a), sqrt(1.0 - a));
  return (float)(R * c);
}

// ------------------------ CHECKPOINT QUEUE ------------------------
static bool hasActiveCheckpoint() { return ckptCount > 0; }

static Checkpoint& activeCheckpoint() {
  // Oldest checkpoint is the "active" one followers should reach next
  return ckpts[ckptHead];
}

static void pushCheckpoint(double lat, double lon) {
  int idx = (ckptHead + ckptCount) % MAX_CKPTS;

  if (ckptCount == MAX_CKPTS) {
    // overwrite oldest (advance head)
    ckptHead = (ckptHead + 1) % MAX_CKPTS;
    idx = (ckptHead + ckptCount - 1) % MAX_CKPTS;
  } else {
    ckptCount++;
  }

  ckptSeqCounter++;
  ckpts[idx].lat = lat;
  ckpts[idx].lon = lon;
  ckpts[idx].createdMs = millis();
  ckpts[idx].seq = ckptSeqCounter;

  Serial.print("[CKPT] Added #");
  Serial.print(ckpts[idx].seq);
  Serial.print("  lat=");
  Serial.print(lat, 6);
  Serial.print(" lon=");
  Serial.print(lon, 6);
  Serial.print(" (interval ");
  Serial.print(checkpointIntervalM);
  Serial.println(" m)");
}

static void popActiveCheckpoint() {
  if (ckptCount <= 0) return;
  Serial.print("[CKPT] Completed active checkpoint #");
  Serial.println(activeCheckpoint().seq);

  ckptHead = (ckptHead + 1) % MAX_CKPTS;
  ckptCount--;
}

static void printActiveCheckpoint() {
  if (!hasActiveCheckpoint()) {
    Serial.println("[CKPT] No active checkpoint yet.");
    return;
  }
  Checkpoint &c = activeCheckpoint();
  Serial.print("[CKPT] ACTIVE #");
  Serial.print(c.seq);
  Serial.print("  lat=");
  Serial.print(c.lat, 6);
  Serial.print(" lon=");
  Serial.print(c.lon, 6);
  Serial.print("  reachRadius=");
  Serial.print(reachRadiusM);
  Serial.println(" m");
}

// ------------------------ FOLLOWER TRACKING ------------------------
static int followerIndexById(int id) {
  for (int i = 0; i < MAX_FOLLOWERS; i++) {
    if (followers[i].active && followers[i].id == id) return i;
  }
  return -1;
}

static int allocateFollower(int id) {
  int idx = followerIndexById(id);
  if (idx >= 0) return idx;
  for (int i = 0; i < MAX_FOLLOWERS; i++) {
    if (!followers[i].active) {
      followers[i].active = true;
      followers[i].id = id;
      followers[i].lastSeenMs = millis();
      followers[i].reachedSeq = 0;
      return i;
    }
  }
  return -1;
}

static void printFollowersStatus() {
  Serial.println("[FOLLOWERS] Status:");
  for (int i = 0; i < MAX_FOLLOWERS; i++) {
    if (!followers[i].active) continue;
    Serial.print("  ID ");
    Serial.print(followers[i].id);
    Serial.print(" reachedSeq=");
    Serial.print(followers[i].reachedSeq);
    Serial.print(" lastSeenMsAgo=");
    Serial.println(millis() - followers[i].lastSeenMs);
  }
}

static bool allFollowersReachedActive() {
  if (!hasActiveCheckpoint()) return false;
  uint32_t activeSeq = activeCheckpoint().seq;

  bool anyFollower = false;
  for (int i = 0; i < MAX_FOLLOWERS; i++) {
    if (!followers[i].active) continue;
    anyFollower = true;
    if (followers[i].reachedSeq < activeSeq) return false;
  }
  // If no followers have ever reported, we do NOT auto-advance.
  return anyFollower;
}

static void onFollowerReport(int id, double lat, double lon) {
  int idx = allocateFollower(id);
  if (idx < 0) {
    Serial.println("[ERR] No free follower slots.");
    return;
  }
  followers[idx].lastSeenMs = millis();

  if (!hasActiveCheckpoint()) {
    Serial.println("[FOLLOWER] No active checkpoint to evaluate yet.");
    return;
  }

  Checkpoint &c = activeCheckpoint();
  float d = distanceMeters(lat, lon, c.lat, c.lon);

  Serial.print("[FOLLOWER ");
  Serial.print(id);
  Serial.print("] distToActive#");
  Serial.print(c.seq);
  Serial.print(" = ");
  Serial.print(d, 1);
  Serial.println(" m");

  if (d <= reachRadiusM) {
    if (followers[idx].reachedSeq < c.seq) {
      followers[idx].reachedSeq = c.seq;
      Serial.print("[FOLLOWER ");
      Serial.print(id);
      Serial.print("] REACHED checkpoint #");
      Serial.println(c.seq);
    }
  }

  if (allFollowersReachedActive()) {
    popActiveCheckpoint();
    printActiveCheckpoint();
    printFollowersStatus();
  }
}

// ------------------------ SERIAL COMMANDS ------------------------
static void handleCommandLine(const String& line) {
  String s = line;
  s.trim();
  if (s.length() == 0) return;

  // Interval command: I <meters>
  if (s.startsWith("I ")) {
    float m = s.substring(2).toFloat();
    if (m > 1.0f) {
      checkpointIntervalM = m;
      Serial.print("[CMD] checkpointIntervalM = ");
      Serial.println(checkpointIntervalM);
    } else {
      Serial.println("[CMD] Invalid interval (must be > 1).");
    }
    return;
  }

  // Reach radius: R <meters>
  if (s.startsWith("R ")) {
    float m = s.substring(2).toFloat();
    if (m > 1.0f) {
      reachRadiusM = m;
      Serial.print("[CMD] reachRadiusM = ");
      Serial.println(reachRadiusM);
    } else {
      Serial.println("[CMD] Invalid radius (must be > 1).");
    }
    return;
  }

  // START [delaySeconds]
  if (s.startsWith("START")) {
    int space = s.indexOf(' ');
    int delaySec = 0;

    if (space > 0) {
      delaySec = s.substring(space + 1).toInt();
      if (delaySec < 0) delaySec = 0;
    }

    startDelayMs = (unsigned long)delaySec * 1000UL;
    startTriggerMs = millis() + startDelayMs;

    pendingStart = true;
    checkpointingEnabled = false;

    Serial.print("[CMD] Checkpointing will START in ");
    Serial.print(delaySec);
    Serial.println(" seconds (waiting for GPS fix).");
    return;
  }

  // STOP
  if (s.equalsIgnoreCase("STOP")) {
    checkpointingEnabled = false;
    pendingStart = false;
    Serial.println("[CMD] Checkpointing STOPPED.");
    return;
  }

  // CLEAR
  if (s.equalsIgnoreCase("CLEAR")) {
    ckptHead = 0;
    ckptCount = 0;
    haveLastFix = false;
    distSinceLastCheckpointM = 0.0f;

    checkpointingEnabled = false;
    pendingStart = false;

    Serial.println("[CMD] Cleared checkpoints. Send START or START <sec> to begin.");
    return;
  }

  // PRINT
  if (s.equalsIgnoreCase("PRINT")) {
    Serial.print("[MODE] checkpointingEnabled=");
    Serial.print(checkpointingEnabled ? "true" : "false");
    Serial.print(" pendingStart=");
    Serial.println(pendingStart ? "true" : "false");
    printActiveCheckpoint();
    printFollowersStatus();
    return;
  }

  // Follower report: F <id> <lat> <lon>
  if (s.startsWith("F ")) {
    int p1 = s.indexOf(' ');
    int p2 = s.indexOf(' ', p1 + 1);
    int p3 = s.indexOf(' ', p2 + 1);
    if (p1 < 0 || p2 < 0 || p3 < 0) {
      Serial.println("[CMD] Bad follower format. Use: F <id> <lat> <lon>");
      return;
    }
    int id = s.substring(p1 + 1, p2).toInt();
    double lat = s.substring(p2 + 1, p3).toDouble();
    double lon = s.substring(p3 + 1).toDouble();
    onFollowerReport(id, lat, lon);
    return;
  }

  Serial.println("[CMD] Unknown command. Try: START [sec], STOP, I <m>, R <m>, F <id> <lat> <lon>, PRINT, CLEAR");
}

// ------------------------ GPS LOOP ------------------------
void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  Serial.println("Front Car: Checkpoint Generator (UNO R4 Serial1 + Adafruit_GPS)");
  Serial1.begin(GPS_BAUD);
  GPS.begin(GPS_BAUD);

  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);

  Serial.println("[READY] Commands:");
  Serial.println("  START [sec]   -> arm start (optional delayed) when GPS fix is available");
  Serial.println("  STOP          -> stop generating checkpoints");
  Serial.println("  I <meters>    -> checkpoint spacing");
  Serial.println("  R <meters>    -> reach radius for followers");
  Serial.println("  F <id> <lat> <lon> -> follower report");
  Serial.println("  PRINT         -> status");
  Serial.println("  CLEAR         -> clear checkpoints (then START again)");
}

void loop() {
  // Continuously read GPS characters
  GPS.read();

  // If a new NMEA sentence is ready, parse it
  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA())) {
      // parse failed
    } else {
      // Handle delayed START: once we have fix and delay time has passed, enable checkpointing
      if (pendingStart) {
        if (!GPS.fix) {
          // still waiting for fix
        } else if (millis() >= startTriggerMs) {
          checkpointingEnabled = true;
          pendingStart = false;

          haveLastFix = false;               // force checkpoint #1 at the next processing below
          distSinceLastCheckpointM = 0.0f;

          Serial.println("[AUTO] Checkpointing STARTED (delayed).");
        }
      }

      // Only generate checkpoints when enabled
      if (GPS.fix && checkpointingEnabled) {
        double lat = GPS.latitudeDegrees;
        double lon = GPS.longitudeDegrees;

        if (!haveLastFix) {
          haveLastFix = true;
          lastFixLat = lat;
          lastFixLon = lon;

          // First checkpoint at start moment
          pushCheckpoint(lat, lon);
          printActiveCheckpoint();
        } else {
          float d = distanceMeters(lastFixLat, lastFixLon, lat, lon);

          // Update last fix for incremental distance
          lastFixLat = lat;
          lastFixLon = lon;

          distSinceLastCheckpointM += d;

          if (distSinceLastCheckpointM >= checkpointIntervalM) {
            distSinceLastCheckpointM = 0.0f;
            pushCheckpoint(lat, lon);
            printActiveCheckpoint();
          }
        }
      }
    }
  }

  // Read Serial commands / follower updates
  while (Serial.available()) {
    char ch = (char)Serial.read();
    if (ch == '\n' || ch == '\r') {
      if (serialLine.length() > 0) {
        handleCommandLine(serialLine);
        serialLine = "";
      }
    } else {
      serialLine += ch;
      if (serialLine.length() > 120) serialLine.remove(0, 60);
    }
  }
}
