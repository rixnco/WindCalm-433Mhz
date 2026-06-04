#include <Arduino.h>
#include <Preferences.h>
#include <RCSwitch.h>

#include <ctype.h>
#include <stdlib.h>
#include <string.h>

namespace {

#if defined(ARDUINO_ESP32S3_DEV)
constexpr uint8_t kTxPin = 1;
constexpr uint8_t kRxPin = 5;
#elif defined(ARDUINO_ESP32_DEV)
constexpr uint8_t kTxPin = 27;
constexpr uint8_t kRxPin = 34;
#else
#error Unsupported board
#endif

constexpr unsigned long kSerialBaudRate = 115200;
constexpr unsigned int kFrameBitLength = 32;
constexpr uint8_t kDefaultTxProtocol = 1;
constexpr uint16_t kDefaultTxDelayUs = 350;
constexpr size_t kMaxLineLength = 96;
constexpr size_t kMaxControllerCount = 24;
constexpr unsigned long kDefaultPairTimeoutMs = 10000;
constexpr unsigned long kPairHoldMs = 2000;
constexpr unsigned long kPairRepeatMs = 110;
constexpr unsigned long kDefaultPairBurstMs = 4000;
constexpr const char *kPrefsNamespace = "sw433";
constexpr const char *kPrefsCountKey = "count";
constexpr const char *kPrefsDataKey = "data";

constexpr uint8_t kWallBeep = 0x0;
constexpr uint8_t kWallReverseA = 0x1;
constexpr uint8_t kWallReverseB = 0x2;
constexpr uint8_t kWallSpeed2 = 0x3;
constexpr uint8_t kWallSpeed1 = 0x4;
constexpr uint8_t kWallSpeed6 = 0x5;
constexpr uint8_t kWallLight = 0x6;
constexpr uint8_t kWallSpeed3 = 0x7;
constexpr uint8_t kWallSpeed5 = 0x8;
constexpr uint8_t kWallSpeed4 = 0x9;
constexpr uint8_t kWallTimerOff = 0xA;
constexpr uint8_t kWallTimer1h = 0xB;
constexpr uint8_t kWallTimer2h = 0xC;
constexpr uint8_t kWallTimer4h = 0xD;
constexpr uint8_t kWallTimer8h = 0xE;
constexpr uint8_t kWallFan = 0xF;

constexpr uint8_t kManualSpeed1 = 0x02;
constexpr uint8_t kManualSpeed4 = 0x03;
constexpr uint8_t kManualBeep = 0x04;
constexpr uint8_t kManualFan = 0x05;
constexpr uint8_t kManualSpeed3 = 0x06;
constexpr uint8_t kManualSpeed2 = 0x08;
constexpr uint8_t kManualLight = 0x09;
constexpr uint8_t kManualTemp = 0x0E;
constexpr uint8_t kManualTimer4h = 0x11;
constexpr uint8_t kManualSpeed5 = 0x14;
constexpr uint8_t kManualTimer2h = 0x17;
constexpr uint8_t kManualReverse = 0x18;
constexpr uint8_t kManualSpeed6 = 0x1A;
constexpr uint8_t kManualTimer1h = 0x1B;
constexpr uint8_t kQiachipPair = 0x0F;
constexpr uint8_t kQiachipLight = 0x08;
constexpr uint8_t kQiachipSpeed3 = 0x09;
constexpr uint8_t kQiachipSpeed2 = 0x03;
constexpr uint8_t kQiachipSpeed1 = 0x02;
constexpr uint8_t kQiachipStop = 0x06;
constexpr uint8_t kQiachipTimer1 = 0x01;
constexpr uint8_t kQiachipTimer2 = 0x07;
constexpr uint8_t kQiachipTimer4 = 0x05;
constexpr uint8_t kQiachipTimer8 = 0x0A;

constexpr uint8_t kWallTimerCycle[] = {kWallTimer1h, kWallTimer2h, kWallTimer4h, kWallTimer8h, kWallTimerOff};
constexpr uint8_t kManualTimerCycle[] = {kManualTimer1h, kManualTimer2h, kManualTimer4h};

enum class ControllerProfile : uint8_t {
  Wall = 0,
  Remote = 1,
  Qiachip = 2,
};

enum class AppMode : uint8_t {
  Receiver = 0,
  Controller = 1,
};

enum class CommandId : uint8_t {
  Unknown = 0,
  Ambiguous,
  Help,
  Status,
  Remove,
  List,
  Clear,
  Use,
  Exit,
  Light,
  Temp,
  Speed,
  Fan,
  Stop,
  Timer,
  Reverse,
  Beep,
  Radio,
  Pair,
};

struct WallData {
  uint8_t counter = 0;
  uint8_t timerIndex = 0;
  uint8_t reverseUsesB = 0;
};

struct ManualData {
  uint8_t counter = 0;
  uint8_t reserved = 0;
  uint8_t timerIndex = 0;
};

struct ControllerData {
  WallData wall;
  ManualData manual;
  uint8_t txProtocol = kDefaultTxProtocol;
  uint16_t txDelayUs = kDefaultTxDelayUs;
};

struct WallDataV1 {
  uint8_t counter = 0;
  uint8_t timerIndex = 0;
  uint8_t reverseUsesB = 0;
};

struct ManualDataV1 {
  uint8_t counter = 0;
  uint8_t reserved = 0;
  uint8_t timerIndex = 0;
};

struct ControllerDataV1 {
  WallDataV1 wall;
  ManualDataV1 manual;
};

struct ControllerEntry {
  uint32_t id = 0;
  ControllerProfile profile = ControllerProfile::Remote;
  ControllerData data;
};

struct StoredControllerEntry {
  uint32_t id;
  uint8_t profile;
  ControllerData data;
};

struct StoredControllerEntryV1 {
  uint32_t id;
  uint8_t profile;
  ControllerDataV1 data;
};

struct DecodedFrame {
  uint32_t raw = 0;
  uint32_t id = 0;
  uint8_t command = 0;
  uint8_t x = 0;
  uint8_t y = 0;
  uint8_t key = 0;
  uint8_t page = 0;
  uint8_t manualFunction = 0;
  bool checksumOk = false;
};

struct PairingState {
  bool active = false;
  unsigned long startedAt = 0;
  unsigned long timeoutMs = 0;
  uint32_t candidateId = 0;
  ControllerProfile candidateProfile = ControllerProfile::Remote;
  unsigned long candidateSince = 0;
  unsigned long candidateLastSeen = 0;
  uint32_t candidateDelaySum = 0;
  uint16_t candidateDelaySamples = 0;
  uint8_t candidateProtocol = 0;
};

struct ClearConfirmState {
  bool active = false;
};

struct CommandSpec {
  CommandId id;
  const char *keywords[3];
  size_t keywordCount;
  bool receiverAllowed;
  bool controllerAllowed;
};

RCSwitch radio;
Preferences preferences;
bool preferencesReady = false;
AppMode appMode = AppMode::Receiver;
int currentControllerIndex = -1;
ControllerEntry controllers[kMaxControllerCount];
size_t controllerCount = 0;
char serialLine[kMaxLineLength] = {};
size_t serialLineLength = 0;
uint32_t lastDisplayedFrame = 0;
bool lastDisplayedFrameValid = false;
PairingState pairingState;
ClearConfirmState clearConfirmState;

void saveControllers();
void printPrompt();

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

bool isDigitsOnly(const char *text) {
  if (text == nullptr || *text == '\0') {
    return false;
  }

  while (*text != '\0') {
    if (!isdigit(static_cast<unsigned char>(*text))) {
      return false;
    }
    ++text;
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

const char *profileName(ControllerProfile profile) {
  if (profile == ControllerProfile::Wall) {
    return "wall";
  }
  return profile == ControllerProfile::Qiachip ? "qiachip" : "remote";
}

bool parseProfile(const char *text, ControllerProfile &profile) {
  if (equalsIgnoreCase(text, "wall")) {
    profile = ControllerProfile::Wall;
    return true;
  }
  if (equalsIgnoreCase(text, "remote")) {
    profile = ControllerProfile::Remote;
    return true;
  }
  if (equalsIgnoreCase(text, "qiachip")) {
    profile = ControllerProfile::Qiachip;
    return true;
  }
  return false;
}

bool parseUnsigned(const char *text, unsigned long &value) {
  if (text == nullptr || *text == '\0') {
    return false;
  }

  char *end = nullptr;
  const unsigned long parsed = strtoul(text, &end, 0);
  if (end == text || *end != '\0') {
    return false;
  }

  value = parsed;
  return true;
}

bool parseControllerId(const char *text, uint32_t &value) {
  if (text == nullptr || *text == '\0') {
    return false;
  }

  const char *cursor = text;
  if (startsWithIgnoreCase(cursor, "0x")) {
    cursor += 2;
  }
  if (*cursor == '\0') {
    return false;
  }

  char *end = nullptr;
  const unsigned long parsed = strtoul(cursor, &end, 16);
  if (end == cursor || *end != '\0' || parsed > 0xFFFFFUL) {
    return false;
  }

  value = static_cast<uint32_t>(parsed);
  return true;
}

bool parseControllerIndex(const char *text, size_t &index) {
  if (text == nullptr || *text == '\0') {
    return false;
  }

  const size_t length = strlen(text);
  if (length == 0 || length > 2) {
    return false;
  }

  for (size_t i = 0; i < length; ++i) {
    if (!isdigit(static_cast<unsigned char>(text[i]))) {
      return false;
    }
  }

  unsigned long parsed = 0;
  if (!parseUnsigned(text, parsed)) {
    return false;
  }

  index = static_cast<size_t>(parsed);
  return true;
}

uint8_t keyFromId(uint32_t id) {
  uint8_t key = 0x0A;
  for (uint8_t index = 0; index < 5; ++index) {
    key ^= static_cast<uint8_t>((id >> (index * 4)) & 0x0F);
  }
  return static_cast<uint8_t>(key & 0x0F);
}

DecodedFrame decodeFrame(uint32_t frame) {
  DecodedFrame decoded;
  decoded.raw = frame;
  decoded.id = (frame >> 12) & 0xFFFFFUL;
  decoded.command = static_cast<uint8_t>((frame >> 8) & 0x0F);
  decoded.x = static_cast<uint8_t>((frame >> 4) & 0x0F);
  decoded.y = static_cast<uint8_t>(frame & 0x0F);
  decoded.key = keyFromId(decoded.id);
  decoded.page = static_cast<uint8_t>((decoded.x >> 3) & 0x01);
  decoded.manualFunction = static_cast<uint8_t>((decoded.page << 4) | decoded.command);
  decoded.checksumOk = static_cast<uint8_t>(decoded.x ^ decoded.command ^ decoded.key) == decoded.y;
  return decoded;
}

bool isPairingFrame(const DecodedFrame &decoded, ControllerProfile &profile) {
  if (!decoded.checksumOk) {
    return false;
  }

  if (decoded.command == kWallBeep) {
    profile = ControllerProfile::Wall;
    return true;
  }

  if (decoded.command == static_cast<uint8_t>(kManualFan & 0x0F) && decoded.page == 0) {
    profile = ControllerProfile::Remote;
    return true;
  }

  if (decoded.command == static_cast<uint8_t>(kQiachipPair & 0x0F) && decoded.page == 0) {
    profile = ControllerProfile::Qiachip;
    return true;
  }

  return false;
}

ControllerEntry *currentControllerMutable() {
  if (currentControllerIndex < 0 || currentControllerIndex >= static_cast<int>(controllerCount)) {
    return nullptr;
  }
  return &controllers[currentControllerIndex];
}

const ControllerEntry *currentController() {
  if (currentControllerIndex < 0 || currentControllerIndex >= static_cast<int>(controllerCount)) {
    return nullptr;
  }
  return &controllers[currentControllerIndex];
}

size_t findControllerIndex(uint32_t id) {
  for (size_t index = 0; index < controllerCount; ++index) {
    if (controllers[index].id == id) {
      return index;
    }
  }
  return static_cast<size_t>(-1);
}

bool hasController(uint32_t id) {
  return findControllerIndex(id) != static_cast<size_t>(-1);
}

bool addController(uint32_t id, ControllerProfile profile, unsigned int txProtocol, unsigned int txDelayUs) {
  if (id == 0) {
    Serial.println("Add failed: invalid ID.");
    return false;
  }
  if (hasController(id)) {
    Serial.printf("Add failed: ID 0x%05lX already exists.\n", static_cast<unsigned long>(id));
    return false;
  }
  if (controllerCount >= kMaxControllerCount) {
    Serial.println("Add failed: database is full.");
    return false;
  }

  controllers[controllerCount].id = id;
  controllers[controllerCount].profile = profile;
  controllers[controllerCount].data = {};
  controllers[controllerCount].data.txProtocol = txProtocol == 0 ? kDefaultTxProtocol : static_cast<uint8_t>(txProtocol);
  controllers[controllerCount].data.txDelayUs = txDelayUs == 0 ? kDefaultTxDelayUs : static_cast<uint16_t>(txDelayUs);
  ++controllerCount;
  saveControllers();
  Serial.printf("Added 0x%05lX profile=%s rf[proto=%u delay=%uus].\n",
                static_cast<unsigned long>(id),
                profileName(profile),
                controllers[controllerCount - 1].data.txProtocol,
                controllers[controllerCount - 1].data.txDelayUs);
  return true;
}

bool addController(uint32_t id, ControllerProfile profile) {
  return addController(id, profile, kDefaultTxProtocol, kDefaultTxDelayUs);
}

bool removeController(uint32_t id) {
  const size_t index = findControllerIndex(id);
  if (index == static_cast<size_t>(-1)) {
    Serial.printf("Remove failed: unknown ID 0x%05lX.\n", static_cast<unsigned long>(id));
    return false;
  }

  for (size_t i = index + 1; i < controllerCount; ++i) {
    controllers[i - 1] = controllers[i];
  }
  --controllerCount;

  if (currentControllerIndex == static_cast<int>(index)) {
    currentControllerIndex = -1;
    appMode = AppMode::Receiver;
  } else if (currentControllerIndex > static_cast<int>(index)) {
    --currentControllerIndex;
  }

  saveControllers();
  Serial.printf("Removed 0x%05lX.\n", static_cast<unsigned long>(id));
  return true;
}

void resetControllerList() {
  controllerCount = 0;
  currentControllerIndex = -1;
  appMode = AppMode::Receiver;
  if (preferencesReady) {
    preferences.remove(kPrefsCountKey);
    preferences.remove(kPrefsDataKey);
  }
  Serial.println("Database cleared.");
}

void saveControllers() {
  if (!preferencesReady) {
    return;
  }

  preferences.putUChar(kPrefsCountKey, static_cast<uint8_t>(controllerCount));
  if (controllerCount == 0) {
    preferences.remove(kPrefsDataKey);
    return;
  }

  StoredControllerEntry stored[kMaxControllerCount] = {};
  for (size_t index = 0; index < controllerCount; ++index) {
    stored[index].id = controllers[index].id;
    stored[index].profile = static_cast<uint8_t>(controllers[index].profile);
    stored[index].data = controllers[index].data;
  }
  preferences.putBytes(kPrefsDataKey, stored, controllerCount * sizeof(StoredControllerEntry));
}

void loadControllers() {
  controllerCount = 0;
  if (!preferencesReady) {
    return;
  }

  const uint8_t storedCountRaw = preferences.getUChar(kPrefsCountKey, 0);
  const size_t storedCount = storedCountRaw > kMaxControllerCount ? kMaxControllerCount : static_cast<size_t>(storedCountRaw);
  if (storedCount == 0) {
    return;
  }

  const size_t storedBytes = preferences.getBytesLength(kPrefsDataKey);
  if (storedBytes == 0) {
    return;
  }

  if ((storedBytes % sizeof(StoredControllerEntry)) == 0) {
    StoredControllerEntry stored[kMaxControllerCount] = {};
    const size_t maxBytes = sizeof(stored);
    const size_t requestedBytes = storedBytes < maxBytes ? storedBytes : maxBytes;
    const size_t readBytes = preferences.getBytes(kPrefsDataKey, stored, requestedBytes);
    size_t readCount = readBytes / sizeof(StoredControllerEntry);
    if (readCount > storedCount) {
      readCount = storedCount;
    }

    for (size_t index = 0; index < readCount && controllerCount < kMaxControllerCount; ++index) {
      if (stored[index].id == 0) {
        continue;
      }
      if (stored[index].profile != static_cast<uint8_t>(ControllerProfile::Wall) &&
          stored[index].profile != static_cast<uint8_t>(ControllerProfile::Remote) &&
          stored[index].profile != static_cast<uint8_t>(ControllerProfile::Qiachip)) {
        continue;
      }

      controllers[controllerCount].id = stored[index].id;
      controllers[controllerCount].profile = static_cast<ControllerProfile>(stored[index].profile);
      controllers[controllerCount].data = stored[index].data;
      if (controllers[controllerCount].data.txProtocol == 0) {
        controllers[controllerCount].data.txProtocol = kDefaultTxProtocol;
      }
      if (controllers[controllerCount].data.txDelayUs == 0) {
        controllers[controllerCount].data.txDelayUs = kDefaultTxDelayUs;
      }
      ++controllerCount;
    }
    return;
  }

  if ((storedBytes % sizeof(StoredControllerEntryV1)) == 0) {
    StoredControllerEntryV1 stored[kMaxControllerCount] = {};
    const size_t maxBytes = sizeof(stored);
    const size_t requestedBytes = storedBytes < maxBytes ? storedBytes : maxBytes;
    const size_t readBytes = preferences.getBytes(kPrefsDataKey, stored, requestedBytes);
    size_t readCount = readBytes / sizeof(StoredControllerEntryV1);
    if (readCount > storedCount) {
      readCount = storedCount;
    }

    for (size_t index = 0; index < readCount && controllerCount < kMaxControllerCount; ++index) {
      if (stored[index].id == 0) {
        continue;
      }
      if (stored[index].profile != static_cast<uint8_t>(ControllerProfile::Wall) &&
          stored[index].profile != static_cast<uint8_t>(ControllerProfile::Remote) &&
          stored[index].profile != static_cast<uint8_t>(ControllerProfile::Qiachip)) {
        continue;
      }

      controllers[controllerCount].id = stored[index].id;
      controllers[controllerCount].profile = static_cast<ControllerProfile>(stored[index].profile);
      controllers[controllerCount].data = {};
      controllers[controllerCount].data.wall.counter = stored[index].data.wall.counter;
      controllers[controllerCount].data.wall.timerIndex = stored[index].data.wall.timerIndex;
      controllers[controllerCount].data.wall.reverseUsesB = stored[index].data.wall.reverseUsesB;
      controllers[controllerCount].data.manual.counter = stored[index].data.manual.counter;
      controllers[controllerCount].data.manual.timerIndex = stored[index].data.manual.timerIndex;
      controllers[controllerCount].data.txProtocol = kDefaultTxProtocol;
      controllers[controllerCount].data.txDelayUs = kDefaultTxDelayUs;
      ++controllerCount;
    }
  }
}

bool lookupControllerProfile(uint32_t id, ControllerProfile &profile) {
  const size_t index = findControllerIndex(id);
  if (index == static_cast<size_t>(-1)) {
    return false;
  }
  profile = controllers[index].profile;
  return true;
}

bool shouldDisplayFrame(uint32_t frame) {
  if (lastDisplayedFrameValid && frame == lastDisplayedFrame) {
    return false;
  }
  lastDisplayedFrame = frame;
  lastDisplayedFrameValid = true;
  return true;
}

unsigned int txProtocolFor(const ControllerEntry &controller) {
  return controller.data.txProtocol == 0 ? kDefaultTxProtocol : controller.data.txProtocol;
}

unsigned int txDelayUsFor(const ControllerEntry &controller) {
  return controller.data.txDelayUs == 0 ? kDefaultTxDelayUs : controller.data.txDelayUs;
}

bool sendFrame(ControllerEntry &controller, uint32_t frame) {
  radio.setProtocol(txProtocolFor(controller));
  radio.setPulseLength(txDelayUsFor(controller));
  radio.send(frame, kFrameBitLength);
  return true;
}

bool sendWallAction(ControllerEntry &controller, uint8_t command, const char *label, bool incrementCounter) {
  const uint8_t counter = controller.data.wall.counter;
  const uint32_t frame = ((controller.id & 0xFFFFFUL) << 12) |
                         (static_cast<uint32_t>(command & 0x0F) << 8) |
                         (static_cast<uint32_t>(counter & 0x0F) << 4) |
                         static_cast<uint32_t>((counter ^ command ^ keyFromId(controller.id)) & 0x0F);
  sendFrame(controller, frame);
  Serial.printf("TX %s id=0x%05lX profile=wall proto=%u delay=%uus cmd=0x%X cnt=0x%X frame=0x%08lX\n",
                label,
                static_cast<unsigned long>(controller.id),
                txProtocolFor(controller),
                txDelayUsFor(controller),
                command,
                counter,
                static_cast<unsigned long>(frame));
  if (incrementCounter) {
    controller.data.wall.counter = static_cast<uint8_t>((controller.data.wall.counter + 1) & 0x0F);
    saveControllers();
  }
  return true;
}

bool sendManualAction(ControllerEntry &controller, uint8_t functionId, const char *label, bool incrementCounter) {
  const uint8_t page = static_cast<uint8_t>((functionId >> 4) & 0x01);
  const uint8_t counter = controller.data.manual.counter;
  const uint8_t command = static_cast<uint8_t>(functionId & 0x0F);
  const uint8_t x = static_cast<uint8_t>((page << 3) | (counter & 0x07));
  const uint32_t frame = ((controller.id & 0xFFFFFUL) << 12) |
                         (static_cast<uint32_t>(command) << 8) |
                         (static_cast<uint32_t>(x) << 4) |
                         static_cast<uint32_t>((x ^ command ^ keyFromId(controller.id)) & 0x0F);
  sendFrame(controller, frame);
  Serial.printf("TX %s id=0x%05lX profile=remote proto=%u delay=%uus fn=0x%02X cnt=0x%X frame=0x%08lX\n",
                label,
                static_cast<unsigned long>(controller.id),
                txProtocolFor(controller),
                txDelayUsFor(controller),
                functionId,
                counter,
                static_cast<unsigned long>(frame));
  if (incrementCounter) {
    controller.data.manual.counter = static_cast<uint8_t>((controller.data.manual.counter + 1) & 0x07);
    saveControllers();
  }
  return true;
}

bool sendPairBurst(ControllerEntry &controller, unsigned long durationMs) {
  const unsigned long startedAt = millis();
  while (millis() - startedAt < durationMs) {
    if (controller.profile == ControllerProfile::Wall) {
      sendWallAction(controller, kWallBeep, "Pair", false);
    } else if (controller.profile == ControllerProfile::Qiachip) {
      sendManualAction(controller, kQiachipPair, "Pair", false);
    } else {
      sendManualAction(controller, kManualFan, "Pair", false);
    }
    delay(kPairRepeatMs);
  }
  Serial.println("Pair finished.");
  return true;
}

bool sendQiachipLight(ControllerEntry &controller) {
  return sendManualAction(controller, kQiachipLight, "Light", true);
}

bool sendQiachipStop(ControllerEntry &controller) {
  return sendManualAction(controller, kQiachipStop, "Stop", true);
}

bool sendQiachipSpeed(ControllerEntry &controller, uint8_t speed) {
  switch (speed) {
    case 1:
      return sendManualAction(controller, kQiachipSpeed1, "Speed1", true);
    case 2:
      return sendManualAction(controller, kQiachipSpeed2, "Speed2", true);
    case 3:
      return sendManualAction(controller, kQiachipSpeed3, "Speed3", true);
    default:
      Serial.println("Qiachip supports Speed 1, 2 or 3 only.");
      return false;
  }
}

bool sendQiachipTimer(ControllerEntry &controller, bool hasValue, uint8_t value) {
  if (!hasValue) {
    Serial.println("Timer requires a value for profile qiachip.");
    return false;
  }

  switch (value) {
    case 1:
      return sendManualAction(controller, kQiachipTimer1, "Timer1", true);
    case 2:
      return sendManualAction(controller, kQiachipTimer2, "Timer2", true);
    case 4:
      return sendManualAction(controller, kQiachipTimer4, "Timer4", true);
    case 8:
      return sendManualAction(controller, kQiachipTimer8, "Timer8", true);
    default:
      Serial.println("Qiachip supports Timer 1, 2, 4 or 8 only.");
      return false;
  }
}

bool sendLight(ControllerEntry &controller) {
  return controller.profile == ControllerProfile::Wall
           ? sendWallAction(controller, kWallLight, "Light", true)
           : sendManualAction(controller, kManualLight, "Light", true);
}

bool sendFan(ControllerEntry &controller) {
  return controller.profile == ControllerProfile::Wall
           ? sendWallAction(controller, kWallFan, "Fan", true)
           : sendManualAction(controller, kManualFan, "Fan", true);
}

bool sendBeep(ControllerEntry &controller) {
  return controller.profile == ControllerProfile::Wall
           ? sendWallAction(controller, kWallBeep, "Beep", true)
           : sendManualAction(controller, kManualBeep, "Beep", true);
}

bool sendReverse(ControllerEntry &controller) {
  if (controller.profile == ControllerProfile::Wall) {
    const uint8_t command = controller.data.wall.reverseUsesB ? kWallReverseB : kWallReverseA;
    const bool ok = sendWallAction(controller, command, "Reverse", true);
    if (ok) {
      controller.data.wall.reverseUsesB = controller.data.wall.reverseUsesB ? 0 : 1;
      saveControllers();
    }
    return ok;
  }
  return sendManualAction(controller, kManualReverse, "Reverse", true);
}

bool sendTemp(ControllerEntry &controller) {
  if (controller.profile == ControllerProfile::Wall) {
    Serial.println("Temp is not valid for profile wall.");
    return false;
  }
  return sendManualAction(controller, kManualTemp, "Temp", true);
}

bool sendSpeed(ControllerEntry &controller, uint8_t speed) {
  switch (speed) {
    case 1:
      return controller.profile == ControllerProfile::Wall ? sendWallAction(controller, kWallSpeed1, "Speed1", true) : sendManualAction(controller, kManualSpeed1, "Speed1", true);
    case 2:
      return controller.profile == ControllerProfile::Wall ? sendWallAction(controller, kWallSpeed2, "Speed2", true) : sendManualAction(controller, kManualSpeed2, "Speed2", true);
    case 3:
      return controller.profile == ControllerProfile::Wall ? sendWallAction(controller, kWallSpeed3, "Speed3", true) : sendManualAction(controller, kManualSpeed3, "Speed3", true);
    case 4:
      return controller.profile == ControllerProfile::Wall ? sendWallAction(controller, kWallSpeed4, "Speed4", true) : sendManualAction(controller, kManualSpeed4, "Speed4", true);
    case 5:
      return controller.profile == ControllerProfile::Wall ? sendWallAction(controller, kWallSpeed5, "Speed5", true) : sendManualAction(controller, kManualSpeed5, "Speed5", true);
    case 6:
      return controller.profile == ControllerProfile::Wall ? sendWallAction(controller, kWallSpeed6, "Speed6", true) : sendManualAction(controller, kManualSpeed6, "Speed6", true);
    default:
      Serial.println("Speed must be between 1 and 6.");
      return false;
  }
}

bool sendTimer(ControllerEntry &controller, bool hasValue, uint8_t value) {
  if (controller.profile == ControllerProfile::Remote) {
    if (!hasValue) {
      Serial.println("Timer without parameter is invalid for profile remote.");
      return false;
    }
    switch (value) {
      case 1:
        controller.data.manual.timerIndex = 0;
        return sendManualAction(controller, kManualTimer1h, "Timer1h", true);
      case 2:
        controller.data.manual.timerIndex = 1;
        return sendManualAction(controller, kManualTimer2h, "Timer2h", true);
      case 4:
        controller.data.manual.timerIndex = 2;
        return sendManualAction(controller, kManualTimer4h, "Timer4h", true);
      default:
        Serial.println("Remote profile supports Timer 1, 2 or 4 only.");
        return false;
    }
  }

  if (!hasValue) {
    const uint8_t command = kWallTimerCycle[controller.data.wall.timerIndex % 5];
    const bool ok = sendWallAction(controller, command, command == kWallTimerOff ? "TimerOff" : "Timer", true);
    if (ok) {
      controller.data.wall.timerIndex = static_cast<uint8_t>((controller.data.wall.timerIndex + 1) % 5);
      saveControllers();
    }
    return ok;
  }

  switch (value) {
    case 0:
      controller.data.wall.timerIndex = 4;
      return sendWallAction(controller, kWallTimerOff, "TimerOff", true);
    case 1:
      controller.data.wall.timerIndex = 0;
      return sendWallAction(controller, kWallTimer1h, "Timer1h", true);
    case 2:
      controller.data.wall.timerIndex = 1;
      return sendWallAction(controller, kWallTimer2h, "Timer2h", true);
    case 4:
      controller.data.wall.timerIndex = 2;
      return sendWallAction(controller, kWallTimer4h, "Timer4h", true);
    case 8:
      controller.data.wall.timerIndex = 3;
      return sendWallAction(controller, kWallTimer8h, "Timer8h", true);
    default:
      Serial.println("Wall profile supports Timer 0, 1, 2, 4 or 8 only.");
      return false;
  }
}

bool isPairingCandidate(const DecodedFrame &decoded, ControllerProfile &profile) {
  return isPairingFrame(decoded, profile);
}

void startPairing(unsigned long timeoutMs) {
  pairingState.active = true;
  pairingState.startedAt = millis();
  pairingState.timeoutMs = timeoutMs;
  pairingState.candidateId = 0;
  pairingState.candidateSince = 0;
  pairingState.candidateLastSeen = 0;
  pairingState.candidateDelaySum = 0;
  pairingState.candidateDelaySamples = 0;
  pairingState.candidateProtocol = 0;
  Serial.printf("Pair: listening for pairing for %lus.\n", timeoutMs / 1000UL);
}

void finishPairingTimeout() {
  pairingState.active = false;
  Serial.println("Pair: pairing timeout.");
}

bool processPairingFrame(const DecodedFrame &decoded, unsigned int receivedDelay, unsigned int receivedProtocol) {
  if (!pairingState.active) {
    return false;
  }

  ControllerProfile detectedProfile = ControllerProfile::Remote;
  if (!isPairingCandidate(decoded, detectedProfile)) {
    return false;
  }

  const unsigned long now = millis();
  const bool sameCandidate = pairingState.candidateId == decoded.id && pairingState.candidateProfile == detectedProfile;
  const bool candidateExpired = pairingState.candidateLastSeen != 0 && (now - pairingState.candidateLastSeen) > 500;

  if (!sameCandidate || candidateExpired) {
    pairingState.candidateId = decoded.id;
    pairingState.candidateProfile = detectedProfile;
    pairingState.candidateSince = now;
    pairingState.candidateLastSeen = now;
    pairingState.candidateDelaySum = receivedDelay;
    pairingState.candidateDelaySamples = 1;
    pairingState.candidateProtocol = receivedProtocol == 0 ? kDefaultTxProtocol : static_cast<uint8_t>(receivedProtocol);
    Serial.printf("Pair: candidate profile=%s id=0x%05lX detected (proto=%u delay=%uus).\n",
                  profileName(detectedProfile),
                  static_cast<unsigned long>(decoded.id),
                  pairingState.candidateProtocol,
                  receivedDelay);
    return true;
  }

  pairingState.candidateLastSeen = now;
  if (pairingState.candidateDelaySamples < 0xFFFF) {
    pairingState.candidateDelaySum += receivedDelay;
    ++pairingState.candidateDelaySamples;
  }
  if (receivedProtocol != 0) {
    pairingState.candidateProtocol = static_cast<uint8_t>(receivedProtocol);
  }
  if ((now - pairingState.candidateSince) < kPairHoldMs) {
    return true;
  }

  const unsigned int protocol = pairingState.candidateProtocol == 0 ? kDefaultTxProtocol : pairingState.candidateProtocol;
  const unsigned int avgDelayUs = pairingState.candidateDelaySamples == 0
                                    ? kDefaultTxDelayUs
                                    : static_cast<unsigned int>(pairingState.candidateDelaySum / pairingState.candidateDelaySamples);
  if (addController(decoded.id, detectedProfile, protocol, avgDelayUs)) {
    Serial.printf("Pair: success proto=%u avgDelay=%uus samples=%u.\n",
                  protocol,
                  avgDelayUs,
                  pairingState.candidateDelaySamples);
  }
  pairingState.active = false;
  return true;
}

void checkPairingTimeout() {
  if (!pairingState.active) {
    return;
  }
  if ((millis() - pairingState.startedAt) >= pairingState.timeoutMs) {
    finishPairingTimeout();
  }
}

void printControllerData(const ControllerEntry &controller) {
  Serial.printf("id=0x%05lX profile=%s ", static_cast<unsigned long>(controller.id), profileName(controller.profile));
  const uint8_t key = keyFromId(controller.id);
  if (controller.profile == ControllerProfile::Wall) {
    Serial.printf("data[wall counter=%u timer=%u reverseNext=%u key=0x%X rfProto=%u rfDelay=%uus]\n",
                  controller.data.wall.counter,
                  controller.data.wall.timerIndex,
                  controller.data.wall.reverseUsesB ? 2 : 1,
                  key,
                  txProtocolFor(controller),
                  txDelayUsFor(controller));
    return;
  }

  Serial.printf("data[remote counter=%u timer=%u key=0x%X rfProto=%u rfDelay=%uus]\n",
                controller.data.manual.counter,
                controller.data.manual.timerIndex,
                key,
                txProtocolFor(controller),
                txDelayUsFor(controller));
}

void printHelp() {
  Serial.println("Commands:");
  if (appMode == AppMode::Receiver) {
    Serial.println("  Pair [<ID> <profile> | <time>]");
    Serial.println("  Remove <ID|index> | rm <ID|index>");
    Serial.println("  List | ls");
    Serial.println("  Clear [-f]");
    Serial.println("  Use <ID|index>");
  } else {
    const ControllerEntry *controller = currentController();
    const bool isControllerProfile = controller != nullptr && controller->profile == ControllerProfile::Remote;
    const bool isQiachipProfile = controller != nullptr && controller->profile == ControllerProfile::Qiachip;
    if (isQiachipProfile) {
      Serial.println("  Light");
      Serial.println("  Speed <1..3>");
      Serial.println("  Stop");
      Serial.println("  Timer 1|2|4|8");
    } else if (isControllerProfile) {
      Serial.println("  Light");
      Serial.println("  Temp");
      Serial.println("  Speed <1..6>");
      Serial.println("  Fan");
      Serial.println("  Timer 1|2|4");
    } else {
      Serial.println("  Light");
      Serial.println("  Temp");
      Serial.println("  Speed <1..6>");
      Serial.println("  Fan");
      Serial.println("  Timer [<0,1,2,4,8>]");
    }
    if (!isQiachipProfile) {
      Serial.println("  Reverse");
      Serial.println("  Beep");
    }
    Serial.println("  Rf [<protocol> <delay> | protocol <n> | delay <us>]");
    Serial.println("  Pair [<time>]");
    Serial.println("  Exit");
  }
  Serial.println("  Help | ?");
  Serial.println("  Status");
}

void printStatus() {
  if (appMode == AppMode::Receiver) {
    Serial.printf("mode=receiver controllers=%u pairing=%s\n",
                  static_cast<unsigned int>(controllerCount),
                  pairingState.active ? "on" : "off");
    for (size_t index = 0; index < controllerCount; ++index) {
      Serial.printf("  [%u] id=0x%05lX profile=%s\n",
                    static_cast<unsigned int>(index),
                    static_cast<unsigned long>(controllers[index].id),
                    profileName(controllers[index].profile));
    }
    return;
  }

  const ControllerEntry *controller = currentController();
  if (controller == nullptr) {
    Serial.println("mode=controller (no selected controller)");
    return;
  }

  printControllerData(*controller);
}

void printPrompt() {
  if (appMode == AppMode::Controller) {
    const ControllerEntry *controller = currentController();
    if (controller != nullptr) {
      Serial.printf("0x%05lX> ", static_cast<unsigned long>(controller->id));
      return;
    }
  }
  Serial.print("> ");
}

void addAmbiguousCommand(const char *commandName,
                         const char **commands,
                         size_t capacity,
                         size_t *count) {
  for (size_t index = 0; index < *count; ++index) {
    if (equalsIgnoreCase(commands[index], commandName)) {
      return;
    }
  }
  if (*count < capacity) {
    commands[*count] = commandName;
    ++(*count);
  }
}

void printAmbiguousCommand(const char *token, const char **commands, size_t count) {
  Serial.printf("Ambiguous command: %s [", token);
  for (size_t index = 0; index < count; ++index) {
    if (index > 0) {
      Serial.print(", ");
    }
    Serial.print(commands[index]);
  }
  Serial.println("]");
}

CommandId resolveCommand(const char *token, const char **ambiguousCommands, size_t ambiguousCapacity, size_t *ambiguousCount) {
  if (ambiguousCount != nullptr) {
    *ambiguousCount = 0;
  }

  static const CommandSpec specs[] = {
      {CommandId::Help, {"help", "?"}, 2, true, true},
      {CommandId::Status, {"status"}, 1, true, true},
      {CommandId::Remove, {"remove", "rm"}, 2, true, false},
      {CommandId::List, {"list", "ls"}, 2, true, false},
      {CommandId::Clear, {"clear"}, 1, true, false},
      {CommandId::Use, {"use"}, 1, true, false},
      {CommandId::Exit, {"exit"}, 1, false, true},
      {CommandId::Light, {"light"}, 1, false, true},
      {CommandId::Temp, {"temp", "tmp"}, 2, false, true},
      {CommandId::Speed, {"speed"}, 1, false, true},
      {CommandId::Fan, {"fan"}, 1, false, true},
      {CommandId::Stop, {"stop"}, 1, false, true},
      {CommandId::Timer, {"timer", "tmr"}, 2, false, true},
      {CommandId::Reverse, {"reverse"}, 1, false, true},
      {CommandId::Beep, {"beep"}, 1, false, true},
      {CommandId::Radio, {"radio", "rf"}, 2, false, true},
      {CommandId::Pair, {"pair"}, 1, true, true},
  };

  const ControllerEntry *controller = appMode == AppMode::Controller ? currentController() : nullptr;
  const bool isQiachipProfile = controller != nullptr && controller->profile == ControllerProfile::Qiachip;

  CommandId match = CommandId::Unknown;
  for (const CommandSpec &spec : specs) {
    const bool allowed = appMode == AppMode::Receiver ? spec.receiverAllowed : spec.controllerAllowed;
    if (!allowed) {
      continue;
    }
    if (appMode == AppMode::Controller) {
      if (isQiachipProfile) {
        if (spec.id == CommandId::Fan || spec.id == CommandId::Reverse || spec.id == CommandId::Beep) {
          continue;
        }
      } else if (spec.id == CommandId::Stop) {
        continue;
      }
    }
    bool specMatched = false;
    for (size_t index = 0; index < spec.keywordCount; ++index) {
      const char *keyword = spec.keywords[index];
      const bool exactMatch = equalsIgnoreCase(token, keyword);
      const bool prefixMatch = startsWithIgnoreCase(keyword, token);

      bool compactNumericMatch = false;
      if ((spec.id == CommandId::Speed || spec.id == CommandId::Timer) && startsWithIgnoreCase(token, keyword)) {
        const char *suffix = token + strlen(keyword);
        compactNumericMatch = isDigitsOnly(suffix);
      }

      if (exactMatch || prefixMatch || compactNumericMatch) {
        if (match != CommandId::Unknown) {
          if (ambiguousCommands != nullptr && ambiguousCount != nullptr) {
            addAmbiguousCommand(spec.keywords[0], ambiguousCommands, ambiguousCapacity, ambiguousCount);
          }
          return CommandId::Ambiguous;
        }
        match = spec.id;
        specMatched = true;
        break;
      }
    }
    if (specMatched && ambiguousCommands != nullptr && ambiguousCount != nullptr) {
      addAmbiguousCommand(spec.keywords[0], ambiguousCommands, ambiguousCapacity, ambiguousCount);
    }
  }

  return match;
}

void enterReceiverMode() {
  appMode = AppMode::Receiver;
  currentControllerIndex = -1;
}

void enterControllerMode(size_t index) {
  currentControllerIndex = static_cast<int>(index);
  appMode = AppMode::Controller;
}

bool handleReceiverPair(char **tokens, size_t tokenCount) {
  if (tokenCount == 1) {
    startPairing(kDefaultPairTimeoutMs);
    return true;
  }

  if (tokenCount == 2) {
    unsigned long timeout = 0;
    if (!parseUnsigned(tokens[1], timeout) || timeout == 0) {
      Serial.println("Pair: invalid timeout.");
      return false;
    }
    startPairing(timeout * 1000UL);
    return true;
  }

  if (tokenCount >= 3) {
    uint32_t id = 0;
    ControllerProfile profile = ControllerProfile::Remote;
    if (!parseControllerId(tokens[1], id) || !parseProfile(tokens[2], profile)) {
      Serial.println("Pair: usage Pair <ID> <profile> | Pair <time>");
      return false;
    }
    return addController(id, profile);
  }

  return false;
}

bool handleReceiverCommand(CommandId command, char **tokens, size_t tokenCount) {
  switch (command) {
    case CommandId::Help:
      printHelp();
      return true;
    case CommandId::Status:
      printStatus();
      return true;
    case CommandId::Pair:
      return handleReceiverPair(tokens, tokenCount);
    case CommandId::Remove: {
      if (tokenCount < 2) {
        Serial.println("Remove: usage Remove <ID|index> | rm <ID|index>");
        return false;
      }
      size_t index = static_cast<size_t>(-1);
      uint32_t id = 0;
      if (parseControllerIndex(tokens[1], index)) {
        if (index >= controllerCount) {
          Serial.printf("Remove: unknown index %u.\n", static_cast<unsigned int>(index));
          return false;
        }
        id = controllers[index].id;
      } else if (!parseControllerId(tokens[1], id)) {
        Serial.println("Remove: invalid selector (expected ID or index).");
        return false;
      }
      return removeController(id);
    }
    case CommandId::List:
      for (size_t index = 0; index < controllerCount; ++index) {
        Serial.printf("[%u] 0x%05lX %s\n",
                      static_cast<unsigned int>(index),
                      static_cast<unsigned long>(controllers[index].id),
                      profileName(controllers[index].profile));
      }
      return true;
    case CommandId::Clear:
      if (tokenCount >= 2 && equalsIgnoreCase(tokens[1], "-f")) {
        resetControllerList();
        return true;
      }
      clearConfirmState.active = true;
      Serial.print("Clear all controllers? [Y/n] ");
      return true;
    case CommandId::Use: {
      if (tokenCount < 2) {
        Serial.println("Use: usage Use <ID|index>");
        return false;
      }
      size_t index = static_cast<size_t>(-1);
      uint32_t id = 0;
      if (parseControllerIndex(tokens[1], index)) {
        if (index >= controllerCount) {
          Serial.printf("Use: unknown index %u.\n", static_cast<unsigned int>(index));
          return false;
        }
      } else {
        if (!parseControllerId(tokens[1], id)) {
          Serial.println("Use: invalid selector (expected ID or index).");
          return false;
        }
        index = findControllerIndex(id);
        if (index == static_cast<size_t>(-1)) {
          Serial.printf("Use: unknown ID 0x%05lX.\n", static_cast<unsigned long>(id));
          return false;
        }
      }
      enterControllerMode(index);
      printPrompt();
      return true;
    }
    case CommandId::Unknown:
      Serial.println("Unknown command.");
      return false;
    case CommandId::Ambiguous:
      Serial.println("Ambiguous command.");
      return false;
    default:
      Serial.println("Command not available in receiver mode.");
      return false;
  }
}

bool handleControllerCommand(CommandId command, char **tokens, size_t tokenCount) {
  ControllerEntry *controller = currentControllerMutable();
  if (controller == nullptr) {
    Serial.println("No controller selected.");
    return false;
  }

  switch (command) {
    case CommandId::Help:
      printHelp();
      return true;
    case CommandId::Status:
      printStatus();
      return true;
    case CommandId::Exit:
      enterReceiverMode();
      printPrompt();
      return true;
    case CommandId::Light:
      if (controller->profile == ControllerProfile::Qiachip) {
        return sendQiachipLight(*controller);
      }
      return sendLight(*controller);
    case CommandId::Temp:
      if (controller->profile == ControllerProfile::Qiachip) {
        Serial.println("Temp is not supported for profile qiachip.");
        return false;
      }
      return sendTemp(*controller);
    case CommandId::Fan:
      if (controller->profile == ControllerProfile::Qiachip) {
        return sendQiachipStop(*controller);
      }
      return sendFan(*controller);
    case CommandId::Stop:
      if (controller->profile == ControllerProfile::Qiachip) {
        return sendQiachipStop(*controller);
      }
      Serial.println("Stop is only supported for profile qiachip.");
      return false;
    case CommandId::Reverse:
      if (controller->profile == ControllerProfile::Qiachip) {
        Serial.println("Reverse is not supported for profile qiachip.");
        return false;
      }
      return sendReverse(*controller);
    case CommandId::Beep:
      if (controller->profile == ControllerProfile::Qiachip) {
        Serial.println("Beep is not supported for profile qiachip.");
        return false;
      }
      return sendBeep(*controller);
    case CommandId::Pair: {
      unsigned long duration = kDefaultPairBurstMs;
      if (tokenCount >= 2) {
        unsigned long seconds = 0;
        if (!parseUnsigned(tokens[1], seconds) || seconds == 0) {
          Serial.println("Pair: invalid time.");
          return false;
        }
        duration = seconds * 1000UL;
      }
      return sendPairBurst(*controller, duration);
    }
    case CommandId::Radio: {
      if (tokenCount == 1) {
        Serial.printf("RF: protocol=%u delay=%uus\n", txProtocolFor(*controller), txDelayUsFor(*controller));
        return true;
      }

      if (tokenCount == 3 && isDigitsOnly(tokens[1]) && isDigitsOnly(tokens[2])) {
        unsigned long protocol = 0;
        unsigned long delayUs = 0;
        if (!parseUnsigned(tokens[1], protocol) || protocol == 0 || protocol > 255) {
          Serial.println("Rf: protocol must be in range 1..255.");
          return false;
        }
        if (!parseUnsigned(tokens[2], delayUs) || delayUs == 0 || delayUs > 65535) {
          Serial.println("Rf: delay must be in range 1..65535 us.");
          return false;
        }
        controller->data.txProtocol = static_cast<uint8_t>(protocol);
        controller->data.txDelayUs = static_cast<uint16_t>(delayUs);
        saveControllers();
        Serial.printf("RF updated: protocol=%u delay=%uus\n", txProtocolFor(*controller), txDelayUsFor(*controller));
        return true;
      }

      if (tokenCount == 3 && (equalsIgnoreCase(tokens[1], "protocol") || equalsIgnoreCase(tokens[1], "proto"))) {
        unsigned long protocol = 0;
        if (!parseUnsigned(tokens[2], protocol) || protocol == 0 || protocol > 255) {
          Serial.println("Rf: protocol must be in range 1..255.");
          return false;
        }
        controller->data.txProtocol = static_cast<uint8_t>(protocol);
        saveControllers();
        Serial.printf("RF updated: protocol=%u delay=%uus\n", txProtocolFor(*controller), txDelayUsFor(*controller));
        return true;
      }

      if (tokenCount == 3 && (equalsIgnoreCase(tokens[1], "delay") || equalsIgnoreCase(tokens[1], "pulse"))) {
        unsigned long delayUs = 0;
        if (!parseUnsigned(tokens[2], delayUs) || delayUs == 0 || delayUs > 65535) {
          Serial.println("Rf: delay must be in range 1..65535 us.");
          return false;
        }
        controller->data.txDelayUs = static_cast<uint16_t>(delayUs);
        saveControllers();
        Serial.printf("RF updated: protocol=%u delay=%uus\n", txProtocolFor(*controller), txDelayUsFor(*controller));
        return true;
      }

      Serial.println("Rf: usage Rf [<protocol> <delay> | protocol <n> | delay <us>]");
      return false;
    }
    case CommandId::Speed: {
      int speed = -1;
      if (tokenCount >= 2) {
        speed = atoi(tokens[1]);
      }
      if (controller->profile == ControllerProfile::Qiachip) {
        return sendQiachipSpeed(*controller, static_cast<uint8_t>(speed));
      }
      return sendSpeed(*controller, static_cast<uint8_t>(speed));
    }
    case CommandId::Timer: {
      if (controller->profile == ControllerProfile::Qiachip) {
        if (tokenCount < 2) {
          return sendQiachipTimer(*controller, false, 0);
        }
        unsigned long value = 0;
        if (!parseUnsigned(tokens[1], value)) {
          Serial.println("Timer: invalid value.");
          return false;
        }
        return sendQiachipTimer(*controller, true, static_cast<uint8_t>(value));
      }
      if (controller->profile == ControllerProfile::Remote) {
        if (tokenCount < 2) {
          Serial.println("Timer without parameter is invalid for profile remote.");
          return false;
        }
        unsigned long value = 0;
        if (!parseUnsigned(tokens[1], value)) {
          Serial.println("Timer: invalid value.");
          return false;
        }
        return sendTimer(*controller, true, static_cast<uint8_t>(value));
      }

      if (tokenCount == 1) {
        return sendTimer(*controller, false, 0);
      }
      unsigned long value = 0;
      if (!parseUnsigned(tokens[1], value)) {
        Serial.println("Timer: invalid value.");
        return false;
      }
      return sendTimer(*controller, true, static_cast<uint8_t>(value));
    }
    case CommandId::Unknown:
      Serial.println("Unknown command.");
      return false;
    case CommandId::Ambiguous:
      Serial.println("Ambiguous command.");
      return false;
    default:
      Serial.println("Command not available in controller mode.");
      return false;
  }
}

void printReceivedFrame(const DecodedFrame &decoded,
                        unsigned int receivedBitLength,
                        unsigned int receivedDelay,
                        unsigned int receivedProtocol) {
  if (!decoded.checksumOk) {
    Serial.printf("RX 0x%08lX bits=%u delay=%u proto=%u -> checksum invalid name=Invalid\n",
                  static_cast<unsigned long>(decoded.raw),
                  receivedBitLength,
                  receivedDelay,
                  receivedProtocol);
    return;
  }

  ControllerProfile profile;
  if (!lookupControllerProfile(decoded.id, profile)) {
    Serial.printf("RX 0x%08lX bits=%u delay=%u proto=%u Unknown name=Unknown\n",
                  static_cast<unsigned long>(decoded.raw),
                  receivedBitLength,
                  receivedDelay,
                  receivedProtocol);
    return;
  }

  if (profile == ControllerProfile::Wall) {
    Serial.printf("RX 0x%08lX bits=%u delay=%u proto=%u wall cmd=0x%X cnt=0x%X name=%s\n",
                  static_cast<unsigned long>(decoded.raw),
                  receivedBitLength,
                  receivedDelay,
                  receivedProtocol,
                  decoded.command,
                  decoded.x,
                  kWallBeep == decoded.command      ? "Beep"
                  : kWallReverseA == decoded.command ? "ReverseA"
                  : kWallReverseB == decoded.command ? "ReverseB"
                  : kWallLight == decoded.command    ? "Light"
                  : kWallFan == decoded.command      ? "Fan"
                  : kWallTimerOff == decoded.command ? "TimerOff"
                  : kWallTimer1h == decoded.command  ? "Timer1h"
                  : kWallTimer2h == decoded.command  ? "Timer2h"
                  : kWallTimer4h == decoded.command  ? "Timer4h"
                  : kWallTimer8h == decoded.command  ? "Timer8h"
                                                     : "UnknownCmd");
    return;
  }

  if (profile == ControllerProfile::Qiachip) {
    Serial.printf("RX 0x%08lX bits=%u delay=%u proto=%u qiachip cmd=0x%X cnt=%u name=%s\n",
                  static_cast<unsigned long>(decoded.raw),
                  receivedBitLength,
                  receivedDelay,
                  receivedProtocol,
                  decoded.command,
                  static_cast<unsigned int>(decoded.x & 0x07),
                  decoded.command == kQiachipLight  ? "Light"
                  : decoded.command == kQiachipSpeed1 ? "Speed1"
                  : decoded.command == kQiachipSpeed2 ? "Speed2"
                  : decoded.command == kQiachipSpeed3 ? "Speed3"
                  : decoded.command == kQiachipStop   ? "Stop"
                  : decoded.command == kQiachipTimer1 ? "Timer1"
                  : decoded.command == kQiachipTimer2 ? "Timer2"
                  : decoded.command == kQiachipTimer4 ? "Timer4"
                  : decoded.command == kQiachipTimer8 ? "Timer8"
                  : decoded.command == kQiachipPair   ? "Pair"
                                                      : "UnknownCmd");
    return;
  }

  Serial.printf("RX 0x%08lX bits=%u delay=%u proto=%u remote fn=0x%02X cnt=%u name=%s\n",
                static_cast<unsigned long>(decoded.raw),
                receivedBitLength,
                receivedDelay,
                receivedProtocol,
                decoded.manualFunction,
                static_cast<unsigned int>(decoded.x & 0x07),
                kManualSpeed1 == decoded.manualFunction  ? "Speed1"
                : kManualSpeed2 == decoded.manualFunction ? "Speed2"
                : kManualSpeed3 == decoded.manualFunction ? "Speed3"
                : kManualSpeed4 == decoded.manualFunction ? "Speed4"
                : kManualSpeed5 == decoded.manualFunction ? "Speed5"
                : kManualSpeed6 == decoded.manualFunction ? "Speed6"
                : kManualLight == decoded.manualFunction  ? "Light"
                : kManualFan == decoded.manualFunction    ? "Fan"
                : kManualBeep == decoded.manualFunction   ? "Beep"
                : kManualTemp == decoded.manualFunction   ? "Temp"
                : kManualReverse == decoded.manualFunction ? "Reverse"
                : kManualTimer1h == decoded.manualFunction ? "Timer1h"
                : kManualTimer2h == decoded.manualFunction ? "Timer2h"
                : kManualTimer4h == decoded.manualFunction ? "Timer4h"
                                                          : "UnknownFn");
}

void handleControllerLine(char *line) {
  char *trimmed = trimWhitespace(line);
  if (trimmed == nullptr || *trimmed == '\0') {
    return;
  }

  if (clearConfirmState.active) {
    clearConfirmState.active = false;
    if (equalsIgnoreCase(trimmed, "y") || equalsIgnoreCase(trimmed, "yes")) {
      resetControllerList();
    } else {
      Serial.println("Clear canceled.");
    }
    printPrompt();
    return;
  }

  char *tokens[5] = {};
  const size_t tokenCount = splitTokens(trimmed, tokens, 5);
  if (tokenCount == 0) {
    return;
  }

  const char *ambiguousCommands[8] = {};
  size_t ambiguousCount = 0;
  CommandId command = resolveCommand(tokens[0], ambiguousCommands, 8, &ambiguousCount);
  if (command == CommandId::Unknown) {
    Serial.printf("Unknown command: %s\n", tokens[0]);
    printPrompt();
    return;
  }
  if (command == CommandId::Ambiguous) {
    printAmbiguousCommand(tokens[0], ambiguousCommands, ambiguousCount);
    printPrompt();
    return;
  }

  const bool ok = appMode == AppMode::Receiver ? handleReceiverCommand(command, tokens, tokenCount)
                                               : handleControllerCommand(command, tokens, tokenCount);
  if (ok && command != CommandId::Exit && !(appMode == AppMode::Controller && command == CommandId::Use)) {
    printPrompt();
  }
}

void handleReceiverTimeout() {
  if (!pairingState.active) {
    return;
  }
  checkPairingTimeout();
}

void handleReceiver() {
  handleReceiverTimeout();

  if (!radio.available()) {
    return;
  }

  const uint32_t frame = radio.getReceivedValue();
  const unsigned int bitLength = radio.getReceivedBitlength();
  const unsigned int receivedDelay = radio.getReceivedDelay();
  const unsigned int receivedProtocol = radio.getReceivedProtocol();
  radio.resetAvailable();

  if (bitLength != kFrameBitLength || frame == 0) {
    return;
  }

  const DecodedFrame decoded = decodeFrame(frame);
  if (appMode == AppMode::Receiver) {
    const bool consumedByPairing = processPairingFrame(decoded, receivedDelay, receivedProtocol);
    if (!consumedByPairing && shouldDisplayFrame(frame)) {
      printReceivedFrame(decoded, bitLength, receivedDelay, receivedProtocol);
    }
  }
}

}  // namespace

void setup() {
  Serial.begin(kSerialBaudRate);
  radio.enableTransmit(kTxPin);
  radio.enableReceive(digitalPinToInterrupt(kRxPin));

  preferencesReady = preferences.begin(kPrefsNamespace, false);
  if (preferencesReady) {
    loadControllers();
  }

  Serial.println();
  Serial.println("Switch-433 UART protocol bridge ready.");
  if (preferencesReady) {
    Serial.printf("Loaded %u controller(s).\n", static_cast<unsigned int>(controllerCount));
  } else {
    Serial.println("Preferences unavailable, database is volatile.");
  }
  printHelp();
  printStatus();
  printPrompt();
}

void loop() {
  handleReceiver();

  while (Serial.available() > 0) {
    const char incoming = static_cast<char>(Serial.read());
    if (incoming == '\r') {
      continue;
    }
    if (incoming == '\n') {
      serialLine[serialLineLength] = '\0';
      handleControllerLine(serialLine);
      serialLineLength = 0;
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

