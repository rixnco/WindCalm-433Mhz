#include <Arduino.h>
#include <Preferences.h>
#include <RCSwitch.h>

#include <ctype.h>
#include <stdlib.h>
#include <string.h>

namespace {

constexpr uint8_t kTxPin = 27;
constexpr uint8_t kRxPin = 34;
constexpr unsigned long kSerialBaudRate = 115200;
constexpr unsigned int kFrameBitLength = 32;
constexpr unsigned long kPairDurationMs = 4000;
constexpr unsigned long kPairRepeatDelayMs = 110;
constexpr size_t kMaxLineLength = 96;
constexpr unsigned long kReceiverPairWindowMs = 10000;
constexpr unsigned long kReceiverPairHoldMs = 2000;
constexpr unsigned long kReceiverPairGapMs = 500;
constexpr size_t kMaxPairedRemotes = 24;
constexpr const char *kPrefsNamespace = "sw433";
constexpr const char *kPrefsPairCountKey = "pairCount";
constexpr const char *kPrefsPairDataKey = "pairData";

constexpr uint8_t kWallBeep = 0x0;
constexpr uint8_t kWallReverseA = 0x1;
constexpr uint8_t kWallReverseB = 0x2;
constexpr uint8_t kWallSpeed2 = 0x3;
constexpr uint8_t kWallSpeed1 = 0x4;
constexpr uint8_t kWallSpeed6 = 0x5;
constexpr uint8_t kWallLightOnOff = 0x6;
constexpr uint8_t kWallSpeed3 = 0x7;
constexpr uint8_t kWallSpeed5 = 0x8;
constexpr uint8_t kWallSpeed4 = 0x9;
constexpr uint8_t kWallTimerOff = 0xA;
constexpr uint8_t kWallTimer1h = 0xB;
constexpr uint8_t kWallTimer2h = 0xC;
constexpr uint8_t kWallTimer4h = 0xD;
constexpr uint8_t kWallTimer8h = 0xE;
constexpr uint8_t kWallFanOnOff = 0xF;

constexpr uint8_t kManualSpeed1 = 0x02;
constexpr uint8_t kManualSpeed4 = 0x03;
constexpr uint8_t kManualBeep = 0x04;
constexpr uint8_t kManualFanOnOff = 0x05;
constexpr uint8_t kManualSpeed3 = 0x06;
constexpr uint8_t kManualSpeed2 = 0x08;
constexpr uint8_t kManualLightOnOff = 0x09;
constexpr uint8_t kManualTemp = 0x0E;
constexpr uint8_t kManualTimer4h = 0x11;
constexpr uint8_t kManualSpeed5 = 0x14;
constexpr uint8_t kManualTimer2h = 0x17;
constexpr uint8_t kManualReverse = 0x18;
constexpr uint8_t kManualSpeed6 = 0x1A;
constexpr uint8_t kManualTimer1h = 0x1B;

constexpr uint8_t kWallTimerCycle[] = {kWallTimer1h, kWallTimer2h, kWallTimer4h, kWallTimer8h, kWallTimerOff};
constexpr uint8_t kManualTimerCycle[] = {kManualTimer1h, kManualTimer2h, kManualTimer4h};

enum class ProtocolMode : uint8_t {
  Wall = 0,
  Manual = 1,
  Receiver = 2,
};

struct WallState {
  uint8_t counter = 0;
  uint8_t timerIndex = 0;
  bool reverseUsesB = false;
};

struct ManualState {
  uint8_t pageCounter[2] = {0, 0};
  uint8_t timerIndex = 0;
};

struct DecodedFrame {
  uint32_t raw = 0;            // Raw 32-bit RF frame.
  uint32_t id = 0;             // Remote ID (20 bits, frame[31:12]).
  uint8_t command = 0;         // Command nibble (frame[11:8]).
  uint8_t x = 0;               // X nibble (frame[7:4]): counter/page+counter.
  uint8_t y = 0;               // Checksum nibble received (frame[3:0]).
  uint8_t key = 0;             // Derived key from ID (XOR based).
  uint8_t page = 0;            // Manual page bit (x >> 3).
  uint8_t manualFunction = 0;  // Manual function code: (page << 4) | command.
  bool checksumOk = false;     // True when x ^ command ^ key == y.
};

struct PairedRemote {
  uint32_t id = 0;
  ProtocolMode profile = ProtocolMode::Manual;
};

struct StoredPairedRemote {
  uint32_t id = 0;
  uint8_t profile = 0;
};

struct ReceiverPairingState {
  bool active = false;
  unsigned long startedAt = 0;
  uint32_t candidateId = 0;
  ProtocolMode candidateProfile = ProtocolMode::Manual;
  unsigned long candidateSince = 0;
  unsigned long candidateLastSeen = 0;
};

RCSwitch radio;
Preferences preferences;
bool preferencesReady = false;
ProtocolMode currentMode = ProtocolMode::Manual;
uint32_t remoteIds[2] = {0, 0};
WallState wallState;
ManualState manualState;
PairedRemote pairedRemotes[kMaxPairedRemotes];
size_t pairedRemoteCount = 0;
ReceiverPairingState receiverPairing;
char serialLine[kMaxLineLength] = {};
size_t serialLineLength = 0;
uint32_t lastRxFrame = 0;
uint32_t lastDisplayedRxFrame = 0;

void savePairedRemotes();

size_t modeIndex(ProtocolMode mode) {
  return mode == ProtocolMode::Wall ? 0U : 1U;
}

uint32_t &remoteIdForMode(ProtocolMode mode) {
  return remoteIds[modeIndex(mode)];
}

uint32_t &currentRemoteId() {
  return remoteIdForMode(currentMode);
}

bool equalsIgnoreCase(const char *left, const char *right) {
  if (left == nullptr || right == nullptr) {
    return false;
  }

  while (*left != '\0' && *right != '\0') {
    if (tolower(static_cast<unsigned char>(*left)) != tolower(static_cast<unsigned char>(*right))) {
      return false;
    }
    ++left;
    ++right;
  }

  return *left == '\0' && *right == '\0';
}

bool startsWithIgnoreCase(const char *text, const char *prefix) {
  if (text == nullptr || prefix == nullptr) {
    return false;
  }

  while (*prefix != '\0') {
    if (*text == '\0') {
      return false;
    }
    if (tolower(static_cast<unsigned char>(*text)) != tolower(static_cast<unsigned char>(*prefix))) {
      return false;
    }
    ++text;
    ++prefix;
  }

  return true;
}

char *trimWhitespace(char *text) {
  if (text == nullptr) {
    return nullptr;
  }

  while (*text != '\0' && isspace(static_cast<unsigned char>(*text))) {
    ++text;
  }

  char *end = text + strlen(text);
  while (end > text && isspace(static_cast<unsigned char>(*(end - 1)))) {
    --end;
  }
  *end = '\0';
  return text;
}

size_t splitTokens(char *text, char **tokens, size_t maxTokens) {
  size_t count = 0;
  char *cursor = text;

  while (*cursor != '\0' && count < maxTokens) {
    while (*cursor != '\0' && isspace(static_cast<unsigned char>(*cursor))) {
      ++cursor;
    }
    if (*cursor == '\0') {
      break;
    }

    tokens[count++] = cursor;

    while (*cursor != '\0' && !isspace(static_cast<unsigned char>(*cursor))) {
      ++cursor;
    }
    if (*cursor == '\0') {
      break;
    }

    *cursor = '\0';
    ++cursor;
  }

  return count;
}

const char *modeName(ProtocolMode mode) {
  switch (mode) {
    case ProtocolMode::Wall:
      return "wall";
    case ProtocolMode::Manual:
      return "manual";
    case ProtocolMode::Receiver:
      return "receiver";
  }

  return "manual";
}

bool parseMode(const char *text, ProtocolMode &mode) {
  if (equalsIgnoreCase(text, "wall")) {
    mode = ProtocolMode::Wall;
    return true;
  }
  if (equalsIgnoreCase(text, "manual")) {
    mode = ProtocolMode::Manual;
    return true;
  }
  if (equalsIgnoreCase(text, "receiver")) {
    mode = ProtocolMode::Receiver;
    return true;
  }
  return false;
}

bool parseRemoteId(const char *text, uint32_t &value) {
  if (text == nullptr || *text == '\0') {
    return false;
  }

  char *end = nullptr;
  const unsigned long parsed = strtoul(text, &end, 0);
  if (end == text || *end != '\0' || parsed > 0xFFFFFUL) {
    return false;
  }

  value = static_cast<uint32_t>(parsed);
  return true;
}

DecodedFrame decodeFrame(uint32_t frame) {
  DecodedFrame decoded;
  decoded.raw = frame;
  decoded.id = (frame >> 12) & 0xFFFFFUL;
  decoded.command = static_cast<uint8_t>((frame >> 8) & 0x0F);
  decoded.x = static_cast<uint8_t>((frame >> 4) & 0x0F);
  decoded.y = static_cast<uint8_t>(frame & 0x0F);
  decoded.key = 0x0A;
  for (uint8_t index = 0; index < 5; ++index) {
    decoded.key ^= static_cast<uint8_t>((decoded.id >> (index * 4)) & 0x0F);
  }
  decoded.key = static_cast<uint8_t>(decoded.key & 0x0F);
  decoded.page = static_cast<uint8_t>((decoded.x >> 3) & 0x01);
  decoded.manualFunction = static_cast<uint8_t>((decoded.page << 4) | decoded.command);
  decoded.checksumOk = static_cast<uint8_t>(decoded.x ^ decoded.command ^ decoded.key) == decoded.y;
  return decoded;
}

bool isPairingFrame(const DecodedFrame &decoded, ProtocolMode &profile) {
  if (!decoded.checksumOk) {
    return false;
  }

  if (decoded.command == kWallBeep) {
    profile = ProtocolMode::Wall;
    return true;
  }

  if (decoded.command == static_cast<uint8_t>(kManualFanOnOff & 0x0F) && decoded.page == 0) {
    profile = ProtocolMode::Manual;
    return true;
  }

  return false;
}

bool addPairedRemote(uint32_t id, ProtocolMode profile) {
  for (size_t index = 0; index < pairedRemoteCount; ++index) {
    if (pairedRemotes[index].id == id) {
      if (pairedRemotes[index].profile == profile) {
        Serial.printf("Receiver: %s id=0x%05lX already paired.\n", modeName(profile), static_cast<unsigned long>(id));
      } else {
        pairedRemotes[index].profile = profile;
        savePairedRemotes();
        Serial.printf("Receiver: id=0x%05lX profile updated to %s.\n", static_cast<unsigned long>(id), modeName(profile));
      }
      return false;
    }
  }

  if (pairedRemoteCount >= kMaxPairedRemotes) {
    Serial.println("Receiver: paired list is full.");
    return false;
  }

  pairedRemotes[pairedRemoteCount].id = id;
  pairedRemotes[pairedRemoteCount].profile = profile;
  ++pairedRemoteCount;
  savePairedRemotes();
  return true;
}

bool isStoredProfile(uint8_t profile) {
  return profile == static_cast<uint8_t>(ProtocolMode::Wall) || profile == static_cast<uint8_t>(ProtocolMode::Manual);
}

void savePairedRemotes() {
  if (!preferencesReady) {
    return;
  }

  preferences.putUChar(kPrefsPairCountKey, static_cast<uint8_t>(pairedRemoteCount));
  if (pairedRemoteCount == 0) {
    preferences.remove(kPrefsPairDataKey);
    return;
  }

  StoredPairedRemote stored[kMaxPairedRemotes] = {};
  for (size_t index = 0; index < pairedRemoteCount; ++index) {
    stored[index].id = pairedRemotes[index].id;
    stored[index].profile = static_cast<uint8_t>(pairedRemotes[index].profile);
  }

  preferences.putBytes(kPrefsPairDataKey, stored, pairedRemoteCount * sizeof(StoredPairedRemote));
}

void loadPairedRemotes() {
  pairedRemoteCount = 0;
  if (!preferencesReady) {
    return;
  }

  const uint8_t storedCountRaw = preferences.getUChar(kPrefsPairCountKey, 0);
  const size_t storedCount = storedCountRaw > kMaxPairedRemotes ? kMaxPairedRemotes : static_cast<size_t>(storedCountRaw);
  if (storedCount == 0) {
    return;
  }

  StoredPairedRemote stored[kMaxPairedRemotes] = {};
  const size_t requestedBytes = storedCount * sizeof(StoredPairedRemote);
  const size_t readBytes = preferences.getBytes(kPrefsPairDataKey, stored, requestedBytes);
  const size_t readCount = readBytes / sizeof(StoredPairedRemote);

  for (size_t index = 0; index < readCount; ++index) {
    if (!isStoredProfile(stored[index].profile) || stored[index].id == 0 || pairedRemoteCount >= kMaxPairedRemotes) {
      continue;
    }

    pairedRemotes[pairedRemoteCount].id = stored[index].id;
    pairedRemotes[pairedRemoteCount].profile = static_cast<ProtocolMode>(stored[index].profile);
    ++pairedRemoteCount;
  }
}

void resetPairedRemotes() {
  pairedRemoteCount = 0;
  if (preferencesReady) {
    preferences.remove(kPrefsPairCountKey);
    preferences.remove(kPrefsPairDataKey);
  }
  Serial.println("Receiver: paired list reset.");
}

void startReceiverPairing() {
  receiverPairing.active = true;
  receiverPairing.startedAt = millis();
  receiverPairing.candidateId = 0;
  receiverPairing.candidateSince = 0;
  receiverPairing.candidateLastSeen = 0;
  Serial.println("Receiver pairing started: waiting up to 10s for a pairing command held for at least 2s.");
}

void handleReceiverPairingFrame(const DecodedFrame &decoded) {
  if (!receiverPairing.active) {
    return;
  }

  ProtocolMode detectedProfile = ProtocolMode::Manual;
  if (!isPairingFrame(decoded, detectedProfile)) {
    return;
  }

  const unsigned long now = millis();
  const bool sameCandidate = receiverPairing.candidateId == decoded.id && receiverPairing.candidateProfile == detectedProfile;
  const bool candidateExpired = receiverPairing.candidateLastSeen != 0 && (now - receiverPairing.candidateLastSeen) > kReceiverPairGapMs;

  if (!sameCandidate || candidateExpired) {
    receiverPairing.candidateId = decoded.id;
    receiverPairing.candidateProfile = detectedProfile;
    receiverPairing.candidateSince = now;
    receiverPairing.candidateLastSeen = now;
    Serial.printf("Receiver pairing: candidate profile=%s id=0x%05lX detected.\n",
                  modeName(detectedProfile),
                  static_cast<unsigned long>(decoded.id));
    return;
  }

  receiverPairing.candidateLastSeen = now;
  if ((now - receiverPairing.candidateSince) < kReceiverPairHoldMs) {
    return;
  }

  if (addPairedRemote(decoded.id, detectedProfile)) {
    Serial.printf("Receiver pairing success: profile=%s id=0x%05lX.\n",
                  modeName(detectedProfile),
                  static_cast<unsigned long>(decoded.id));
  }
  receiverPairing.active = false;
}

void checkReceiverPairingTimeout() {
  if (!receiverPairing.active) {
    return;
  }

  if ((millis() - receiverPairing.startedAt) < kReceiverPairWindowMs) {
    return;
  }

  receiverPairing.active = false;
  Serial.println("Receiver pairing timeout: no valid pairing command held for 2s.");
}

bool lookupPairedProfile(uint32_t id, ProtocolMode &profile) {
  for (size_t index = 0; index < pairedRemoteCount; ++index) {
    if (pairedRemotes[index].id == id) {
      profile = pairedRemotes[index].profile;
      return true;
    }
  }
  return false;
}

const char *wallCommandName(uint8_t command) {
  switch (command & 0x0F) {
    case kWallBeep:
      return "Beep";
    case kWallReverseA:
      return "ReverseA";
    case kWallReverseB:
      return "ReverseB";
    case kWallSpeed1:
      return "Speed1";
    case kWallSpeed2:
      return "Speed2";
    case kWallSpeed3:
      return "Speed3";
    case kWallSpeed4:
      return "Speed4";
    case kWallSpeed5:
      return "Speed5";
    case kWallSpeed6:
      return "Speed6";
    case kWallLightOnOff:
      return "Light";
    case kWallFanOnOff:
      return "Fan";
    case kWallTimerOff:
      return "TimerOff";
    case kWallTimer1h:
      return "Timer1h";
    case kWallTimer2h:
      return "Timer2h";
    case kWallTimer4h:
      return "Timer4h";
    case kWallTimer8h:
      return "Timer8h";
    default:
      return "UnknownCmd";
  }
}

const char *manualFunctionName(uint8_t functionId) {
  switch (functionId) {
    case kManualSpeed1:
      return "Speed1";
    case kManualSpeed2:
      return "Speed2";
    case kManualSpeed3:
      return "Speed3";
    case kManualSpeed4:
      return "Speed4";
    case kManualSpeed5:
      return "Speed5";
    case kManualSpeed6:
      return "Speed6";
    case kManualLightOnOff:
      return "Light";
    case kManualFanOnOff:
      return "Fan";
    case kManualBeep:
      return "Beep";
    case kManualTemp:
      return "Temp";
    case kManualReverse:
      return "Reverse";
    case kManualTimer1h:
      return "Timer1h";
    case kManualTimer2h:
      return "Timer2h";
    case kManualTimer4h:
      return "Timer4h";
    default:
      return "UnknownFn";
  }
}

bool shouldDisplayReceiverFrame(uint32_t frame) {
  if (frame == lastDisplayedRxFrame) {
    return false;
  }

  lastDisplayedRxFrame = frame;
  return true;
}

uint8_t keyFromId(uint32_t id) {
  uint8_t key = 0x0A;
  for (uint8_t index = 0; index < 5; ++index) {
    key ^= static_cast<uint8_t>((id >> (index * 4)) & 0x0F);
  }
  return static_cast<uint8_t>(key & 0x0F);
}

uint32_t makeWallFrame(uint32_t id, uint8_t command, uint8_t counter) {
  const uint8_t x = static_cast<uint8_t>(counter & 0x0F);
  const uint8_t checksum = static_cast<uint8_t>(x ^ (command & 0x0F) ^ keyFromId(id));
  return ((id & 0xFFFFFUL) << 12) |
         (static_cast<uint32_t>(command & 0x0F) << 8) |
         (static_cast<uint32_t>(x) << 4) |
         static_cast<uint32_t>(checksum & 0x0F);
}

uint32_t makeManualFrame(uint32_t id, uint8_t functionId, uint8_t counter) {
  const uint8_t page = static_cast<uint8_t>((functionId >> 4) & 0x01);
  const uint8_t command = static_cast<uint8_t>(functionId & 0x0F);
  const uint8_t x = static_cast<uint8_t>((page << 3) | (counter & 0x07));
  const uint8_t checksum = static_cast<uint8_t>(x ^ command ^ keyFromId(id));
  return ((id & 0xFFFFFUL) << 12) |
         (static_cast<uint32_t>(command) << 8) |
         (static_cast<uint32_t>(x) << 4) |
         static_cast<uint32_t>(checksum & 0x0F);
}

void resetStates() {
  wallState.counter = 0;
  wallState.timerIndex = 0;
  wallState.reverseUsesB = false;
  manualState.pageCounter[0] = 0;
  manualState.pageCounter[1] = 0;
  manualState.timerIndex = 0;
}

bool ensureRemoteId() {
  if (currentRemoteId() != 0) {
    return true;
  }

  Serial.printf("Remote ID not set for %s mode. Use: id 0xAB121\n", modeName(currentMode));
  return false;
}

bool isReceiverMode() {
  return currentMode == ProtocolMode::Receiver;
}

bool sendWallCommand(uint8_t command, const char *label, bool incrementCounter) {
  if (isReceiverMode()) {
    Serial.println("Receiver mode is read-only.");
    return false;
  }
  if (!ensureRemoteId()) {
    return false;
  }

  const uint32_t remoteId = currentRemoteId();
  const uint8_t counter = wallState.counter;
  const uint32_t frame = makeWallFrame(remoteId, command, counter);
  radio.send(frame, kFrameBitLength);

  Serial.printf("TX %s mode=wall id=0x%05lX key=0x%X cmd=0x%X counter=0x%X frame=0x%08lX\n",
                label,
                static_cast<unsigned long>(remoteId),
                keyFromId(remoteId),
                command,
                counter,
                static_cast<unsigned long>(frame));

  if (incrementCounter) {
    wallState.counter = static_cast<uint8_t>((wallState.counter + 1) & 0x0F);
  }
  return true;
}

bool sendManualFunction(uint8_t functionId, const char *label, bool incrementCounter) {
  if (isReceiverMode()) {
    Serial.println("Receiver mode is read-only.");
    return false;
  }
  if (!ensureRemoteId()) {
    return false;
  }

  const uint32_t remoteId = currentRemoteId();
  const uint8_t page = static_cast<uint8_t>((functionId >> 4) & 0x01);
  const uint8_t counter = manualState.pageCounter[page];
  const uint32_t frame = makeManualFrame(remoteId, functionId, counter);
  radio.send(frame, kFrameBitLength);

  Serial.printf("TX %s mode=manual id=0x%05lX key=0x%X function=0x%02X page=%u counter=%u frame=0x%08lX\n",
                label,
                static_cast<unsigned long>(remoteId),
                keyFromId(remoteId),
                functionId,
                page,
                counter,
                static_cast<unsigned long>(frame));

  if (incrementCounter) {
    manualState.pageCounter[page] = static_cast<uint8_t>((manualState.pageCounter[page] + 1) & 0x07);
  }
  return true;
}

bool sendCurrentModeCommand(uint8_t wallCommand, uint8_t manualFunction, const char *label) {
  if (currentMode == ProtocolMode::Wall) {
    return sendWallCommand(wallCommand, label, true);
  }
  return sendManualFunction(manualFunction, label, true);
}

bool sendTimerHours(uint8_t hours) {
  if (currentMode == ProtocolMode::Manual) {
    switch (hours) {
      case 1:
        manualState.timerIndex = 1;
        return sendManualFunction(kManualTimer1h, "Timer1h", true);
      case 2:
        manualState.timerIndex = 2;
        return sendManualFunction(kManualTimer2h, "Timer2h", true);
      case 4:
        manualState.timerIndex = 0;
        return sendManualFunction(kManualTimer4h, "Timer4h", true);
      default:
        Serial.println("Manual mode supports Timer1h, Timer2h and Timer4h only.");
        return false;
    }
  }

  switch (hours) {
    case 1:
      wallState.timerIndex = 1;
      return sendWallCommand(kWallTimer1h, "Timer1h", true);
    case 2:
      wallState.timerIndex = 2;
      return sendWallCommand(kWallTimer2h, "Timer2h", true);
    case 4:
      wallState.timerIndex = 3;
      return sendWallCommand(kWallTimer4h, "Timer4h", true);
    case 8:
      wallState.timerIndex = 4;
      return sendWallCommand(kWallTimer8h, "Timer8h", true);
    default:
      Serial.println("Wall mode supports Timer1h, Timer2h, Timer4h and Timer8h only.");
      return false;
  }
}

bool sendTimerCycle() {
  if (currentMode == ProtocolMode::Manual) {
    const uint8_t functionId = kManualTimerCycle[manualState.timerIndex % 3];
    const bool ok = sendManualFunction(functionId, "Timer", true);
    if (ok) {
      manualState.timerIndex = static_cast<uint8_t>((manualState.timerIndex + 1) % 3);
    }
    return ok;
  }

  const uint8_t command = kWallTimerCycle[wallState.timerIndex % 5];
  const bool ok = sendWallCommand(command, command == kWallTimerOff ? "TimerOff" : "Timer", true);
  if (ok) {
    wallState.timerIndex = static_cast<uint8_t>((wallState.timerIndex + 1) % 5);
  }
  return ok;
}

bool sendTimerOff() {
  if (currentMode != ProtocolMode::Wall || isReceiverMode()) {
    Serial.println("TimerOff is only available in wall mode.");
    return false;
  }

  const bool ok = sendWallCommand(kWallTimerOff, "TimerOff", true);
  if (ok) {
    wallState.timerIndex = 0;
  }
  return ok;
}

bool sendReverse() {
  if (currentMode == ProtocolMode::Wall) {
    const uint8_t command = wallState.reverseUsesB ? kWallReverseB : kWallReverseA;
    const bool ok = sendWallCommand(command, "Reverse", true);
    if (ok) {
      wallState.reverseUsesB = !wallState.reverseUsesB;
    }
    return ok;
  }

  return sendManualFunction(kManualReverse, "Reverse", true);
}

bool sendPair() {
  if (isReceiverMode()) {
    startReceiverPairing();
    return true;
  }

  if (!ensureRemoteId()) {
    return false;
  }

  const unsigned long startedAt = millis();
  while (millis() - startedAt < kPairDurationMs) {
    if (currentMode == ProtocolMode::Wall) {
      sendWallCommand(kWallBeep, "pair", false);
    } else {
      sendManualFunction(kManualFanOnOff, "pair", false);
    }
    delay(kPairRepeatDelayMs);
  }

  Serial.println("pair finished.");
  return true;
}

void printStatus() {
  const uint32_t wallRemoteId = remoteIdForMode(ProtocolMode::Wall);
  const uint32_t manualRemoteId = remoteIdForMode(ProtocolMode::Manual);
  Serial.printf("mode=%s wallId=0x%05lX wallKey=0x%X manualId=0x%05lX manualKey=0x%X wall[counter=%u timer=%u reverseNext=%u] manual[counterP0=%u counterP1=%u timer=%u] receiver[pairing=%s paired=%u]\n",
                modeName(currentMode),
                static_cast<unsigned long>(wallRemoteId),
                keyFromId(wallRemoteId),
                static_cast<unsigned long>(manualRemoteId),
                keyFromId(manualRemoteId),
                wallState.counter,
                wallState.timerIndex,
                wallState.reverseUsesB ? 2 : 1,
                manualState.pageCounter[0],
                manualState.pageCounter[1],
                manualState.timerIndex,
                receiverPairing.active ? "on" : "off",
                static_cast<unsigned int>(pairedRemoteCount));

  for (size_t index = 0; index < pairedRemoteCount; ++index) {
    Serial.printf("  paired[%u] id=0x%05lX profile=%s\n",
                  static_cast<unsigned int>(index),
                  static_cast<unsigned long>(pairedRemotes[index].id),
                  modeName(pairedRemotes[index].profile));
  }
}

void printHelp() {
  Serial.println("UART commands:");
  Serial.println("  wall | manual | receiver");
  Serial.println("  id 0x146CD   (sets the ID for the current mode)");
  Serial.println("  Light");
  Serial.println("  Temp");
  Serial.println("  Speed1..Speed6 or Speed <1..6>");
  Serial.println("  Fan");
  Serial.println("  Timer1h | Timer2h | Timer4h | Timer8h");
  Serial.println("  Timer | TimerOff");
  Serial.println("  Reverse");
  Serial.println("  Beep");
  Serial.println("  pair   (TX pair in wall/manual, or start pairing capture in receiver)");
  Serial.println("  reset   (clear paired receiver list)");
  Serial.println("  status");
  Serial.println("  help");
}

void printPrompt() {
  Serial.print("> ");
}

void setMode(ProtocolMode mode) {
  currentMode = mode;
  Serial.printf("Mode changed to %s. This mode keeps its own ID, counters and cycle state.\n", modeName(currentMode));
}

bool handleSpeedTokens(char **tokens, size_t tokenCount) {
  int speed = -1;

  if (startsWithIgnoreCase(tokens[0], "Speed") && strlen(tokens[0]) > 5) {
    speed = atoi(tokens[0] + 5);
  } else if (equalsIgnoreCase(tokens[0], "Speed") && tokenCount >= 2) {
    speed = atoi(tokens[1]);
  }

  switch (speed) {
    case 1:
      return sendCurrentModeCommand(kWallSpeed1, kManualSpeed1, "Speed1");
    case 2:
      return sendCurrentModeCommand(kWallSpeed2, kManualSpeed2, "Speed2");
    case 3:
      return sendCurrentModeCommand(kWallSpeed3, kManualSpeed3, "Speed3");
    case 4:
      return sendCurrentModeCommand(kWallSpeed4, kManualSpeed4, "Speed4");
    case 5:
      return sendCurrentModeCommand(kWallSpeed5, kManualSpeed5, "Speed5");
    case 6:
      return sendCurrentModeCommand(kWallSpeed6, kManualSpeed6, "Speed6");
    default:
      return false;
  }
}

bool handleTimerTokens(char **tokens, size_t tokenCount) {
  if (equalsIgnoreCase(tokens[0], "Timer")) {
    if (tokenCount == 1) {
      return sendTimerCycle();
    }

    if (equalsIgnoreCase(tokens[1], "Off")) {
      return sendTimerOff();
    }

    char timerToken[12];
    strncpy(timerToken, tokens[1], sizeof(timerToken) - 1);
    timerToken[sizeof(timerToken) - 1] = '\0';
    const size_t length = strlen(timerToken);
    if (length > 0 && (timerToken[length - 1] == 'h' || timerToken[length - 1] == 'H')) {
      timerToken[length - 1] = '\0';
    }

    const int hours = atoi(timerToken);
    if (hours <= 0) {
      return false;
    }
    return sendTimerHours(static_cast<uint8_t>(hours));
  }

  if (equalsIgnoreCase(tokens[0], "TimerOff")) {
    return sendTimerOff();
  }
  if (equalsIgnoreCase(tokens[0], "Timer1h")) {
    return sendTimerHours(1);
  }
  if (equalsIgnoreCase(tokens[0], "Timer2h")) {
    return sendTimerHours(2);
  }
  if (equalsIgnoreCase(tokens[0], "Timer4h")) {
    return sendTimerHours(4);
  }
  if (equalsIgnoreCase(tokens[0], "Timer8h")) {
    return sendTimerHours(8);
  }
  return false;
}

void printReceivedFrame(uint32_t frame) {
  const DecodedFrame decoded = decodeFrame(frame);

  if (!decoded.checksumOk) {
    Serial.printf("RX 0x%08lX -> checksum invalid name=Invalid\n", static_cast<unsigned long>(decoded.raw));
    return;
  }

  ProtocolMode profile = ProtocolMode::Manual;
  if (!lookupPairedProfile(decoded.id, profile)) {
    Serial.printf("RX 0x%08lX Unknown name=Unknown\n", static_cast<unsigned long>(decoded.raw));
    return;
  }

  if (profile == ProtocolMode::Wall) {
    const char *commandName = wallCommandName(decoded.command);
    Serial.printf("RX 0x%08lX wall   cmd=0x%02X cnt=0x%X name=%s\n",
                  static_cast<unsigned long>(decoded.raw),
                  decoded.command,
                  decoded.x,
                  commandName);
    return;
  }

  const char *commandName = manualFunctionName(decoded.manualFunction);
  Serial.printf("RX 0x%08lX manual cmd=0x%02X cnt=0x%X name=%s\n",
                static_cast<unsigned long>(decoded.raw),
                decoded.manualFunction,
                static_cast<unsigned int>(decoded.x & 0x07),
                commandName);
}

void processLine(char *line) {
  char *trimmed = trimWhitespace(line);
  if (trimmed == nullptr || *trimmed == '\0') {
    return;
  }

  char *tokens[4] = {};
  const size_t tokenCount = splitTokens(trimmed, tokens, 4);
  if (tokenCount == 0) {
    return;
  }

  if (equalsIgnoreCase(tokens[0], "help")) {
    printHelp();
    return;
  }
  if (equalsIgnoreCase(tokens[0], "status")) {
    printStatus();
    return;
  }
  if (equalsIgnoreCase(tokens[0], "wall") || equalsIgnoreCase(tokens[0], "manual") || equalsIgnoreCase(tokens[0], "receiver")) {
    ProtocolMode mode;
    if (parseMode(tokens[0], mode)) {
      setMode(mode);
    }
    return;
  }
  if (equalsIgnoreCase(tokens[0], "id")) {
    if (isReceiverMode()) {
      Serial.println("Receiver mode does not use a remote ID.");
      return;
    }
    uint32_t parsedId = 0;
    if (tokenCount < 2 || !parseRemoteId(tokens[1], parsedId)) {
      Serial.println("Usage: id 0x146CD");
      return;
    }
    currentRemoteId() = parsedId;
    Serial.printf("Remote ID for %s set to 0x%05lX, key=0x%X\n",
                  modeName(currentMode),
                  static_cast<unsigned long>(currentRemoteId()),
                  keyFromId(currentRemoteId()));
    return;
  }
  if (handleSpeedTokens(tokens, tokenCount)) {
    return;
  }
  if (handleTimerTokens(tokens, tokenCount)) {
    return;
  }
  if (equalsIgnoreCase(tokens[0], "Light")) {
    sendCurrentModeCommand(kWallLightOnOff, kManualLightOnOff, "Light");
    return;
  }
  if (equalsIgnoreCase(tokens[0], "Temp")) {
    if (currentMode == ProtocolMode::Wall) {
      Serial.println("Temp is only available in manual mode.");
      return;
    }
    sendManualFunction(kManualTemp, "Temp", true);
    return;
  }
  if (equalsIgnoreCase(tokens[0], "Fan")) {
    sendCurrentModeCommand(kWallFanOnOff, kManualFanOnOff, "Fan");
    return;
  }
  if (equalsIgnoreCase(tokens[0], "Reverse")) {
    sendReverse();
    return;
  }
  if (equalsIgnoreCase(tokens[0], "Beep")) {
    sendCurrentModeCommand(kWallBeep, kManualBeep, "Beep");
    return;
  }
  if (equalsIgnoreCase(tokens[0], "pair")) {
    sendPair();
    return;
  }
  if (equalsIgnoreCase(tokens[0], "reset")) {
    resetPairedRemotes();
    return;
  }

  Serial.printf("Unknown command: %s\n", tokens[0]);
}

void handleSerial() {
  while (Serial.available() > 0) {
    const char incoming = static_cast<char>(Serial.read());

    if (incoming == '\r') {
      continue;
    }
    if (incoming == '\n') {
      serialLine[serialLineLength] = '\0';
      processLine(serialLine);
      serialLineLength = 0;
      printPrompt();
      continue;
    }
    if (serialLineLength >= (kMaxLineLength - 1)) {
      serialLineLength = 0;
      Serial.println("Input line too long.");
      printPrompt();
      continue;
    }

    serialLine[serialLineLength++] = incoming;
  }
}

void handleReceiver() {
  checkReceiverPairingTimeout();

  if (!radio.available()) {
    return;
  }

  const uint32_t frame = radio.getReceivedValue();
  const unsigned int bitLength = radio.getReceivedBitlength();
  radio.resetAvailable();

  if (bitLength != kFrameBitLength || frame == 0) {
    return;
  }

  const bool duplicate = frame == lastRxFrame;
  lastRxFrame = frame;

  if (!isReceiverMode() && duplicate) {
    return;
  }

  if (isReceiverMode()) {
    const DecodedFrame decoded = decodeFrame(frame);
    handleReceiverPairingFrame(decoded);
    if (shouldDisplayReceiverFrame(frame)) {
      printReceivedFrame(frame);
    }
  }
}

}  // namespace

void setup() {
  Serial.begin(kSerialBaudRate);
  radio.enableTransmit(kTxPin);
  radio.enableReceive(digitalPinToInterrupt(kRxPin));
  resetStates();

  preferencesReady = preferences.begin(kPrefsNamespace, false);
  if (preferencesReady) {
    loadPairedRemotes();
  }

  Serial.println();
  Serial.println("Switch-433 UART protocol bridge ready.");
  if (preferencesReady) {
    Serial.printf("Receiver: loaded %u paired remote(s).\n", static_cast<unsigned int>(pairedRemoteCount));
  } else {
    Serial.println("Receiver: preferences unavailable, paired list is volatile.");
  }
  printHelp();
  printStatus();
  printPrompt();
}

void loop() {
  handleReceiver();
  handleSerial();
}
