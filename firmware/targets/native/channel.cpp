#include "open_evse.h"
#include "channel.h"

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

// ---------------------------------------------------------------------------
// Signal tables
//
// The firmware's whole observable surface. Digital pin state lives in
// EpoxyDuino's own pin store rather than a private array here, so that driving
// an input also feeds checkInterrupts() and a GFI edge dispatches gfi_isr()
// the way hardware would.
// ---------------------------------------------------------------------------

namespace {

struct Signal {
  const char *name;
  uint8_t pin;
};

const Signal kDigitalIn[] = {
  { "GFI",     GFI_REG     },
  { "ACLINE1", ACLINE1_REG },
  { "ACLINE2", ACLINE2_REG },
};

const Signal kDigitalOut[] = {
  { "GFITEST",    GFITEST_REG    },
  { "CHARGING",   CHARGING_REG   },
  { "CHARGING2",  CHARGING2_REG  },
  { "CHARGINGAC", CHARGINGAC_REG },
#ifdef MENNEKES_LOCK
  { "LOCKA",      MENNEKES_LOCK_PINA_REG },
  { "LOCKB",      MENNEKES_LOCK_PINB_REG },
#endif
#ifdef OEV6
  { "V6_CHARGING",  V6_CHARGING_PIN  },
  { "V6_CHARGING2", V6_CHARGING_PIN2 },
#endif
};

const Signal kAdc[] = {
  { "CURRENT",     CURRENT_PIN     },
  { "PILOT_SENSE", PILOT_SENSE_PIN },
  { "PP",          PP_PIN          },
};

constexpr size_t kDigitalInCount  = sizeof(kDigitalIn)  / sizeof(kDigitalIn[0]);
constexpr size_t kDigitalOutCount = sizeof(kDigitalOut) / sizeof(kDigitalOut[0]);
constexpr size_t kAdcCount        = sizeof(kAdc)        / sizeof(kAdc[0]);

// ---------------------------------------------------------------------------
// Connection state
// ---------------------------------------------------------------------------

int  g_listenFd = -1;
int  g_clientFd = -1;
bool g_begun    = false;

char   g_rxBuf[512];
size_t g_rxLen = 0;

// Last published values, so only changes go on the wire.
uint8_t g_lastOut[kDigitalOutCount];
bool    g_lastOutValid = false;

uint8_t  g_pilotState = PILOT_STATE_N12;
uint32_t g_pilotDuty  = 0;
int      g_pilotAmps  = -1;
bool     g_pilotDirty = true;

const char *pilotStateName(uint8_t s)
{
  switch (s) {
    case PILOT_STATE_P12: return "P12";
    case PILOT_STATE_PWM: return "PWM";
    default:              return "N12";
  }
}

void sendLine(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

void sendLine(const char *fmt, ...)
{
  if (g_clientFd < 0) return;

  char line[256];
  va_list ap;
  va_start(ap, fmt);
  int n = vsnprintf(line, sizeof(line) - 1, fmt, ap);
  va_end(ap);
  if (n < 0) return;
  if ((size_t)n > sizeof(line) - 2) n = sizeof(line) - 2;
  line[n++] = '\n';

  // Best effort. A driver that stops reading must not be able to wedge the
  // safety firmware, so a full pipe drops the update rather than blocking;
  // the next change re-publishes, and SNAP recovers the full picture.
  ssize_t w = write(g_clientFd, line, (size_t)n);
  if (w < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
    close(g_clientFd);
    g_clientFd = -1;
  }
}

void sendHello()
{
  sendLine("HELLO board=%s adc_bits=%d adc_max=%d mcu_id_len=%d version=%s",
#ifdef NATIVE_BOARD_NXT
           "nxt",
#else
           "oev6",
#endif
           ADC_RESOLUTION_BITS, ADC_MAX, MCU_ID_LEN, VERSION);
}

void sendSnapshot()
{
  sendHello();
  for (size_t i = 0; i < kDigitalOutCount; i++) {
    sendLine("OUT %s %d", kDigitalOut[i].name,
             digitalWriteValue(kDigitalOut[i].pin) ? 1 : 0);
  }
  sendLine("PILOT %s %u %d", pilotStateName(g_pilotState),
           (unsigned)g_pilotDuty, g_pilotAmps);
}

const Signal *findSignal(const Signal *table, size_t count, const char *name)
{
  for (size_t i = 0; i < count; i++) {
    if (strcmp(table[i].name, name) == 0) return &table[i];
  }
  return NULL;
}

void handleLine(char *line)
{
  char *verb = strtok(line, " \t");
  if (!verb) return;

  if (strcmp(verb, "IN") == 0) {
    const char *name = strtok(NULL, " \t");
    const char *val  = strtok(NULL, " \t");
    if (!name || !val) return;
    const Signal *s = findSignal(kDigitalIn, kDigitalInCount, name);
    if (!s) {
      sendLine("ERR unknown input %s", name);
      return;
    }
    // Into EpoxyDuino's input store, so checkInterrupts() sees the edge.
    digitalReadValue(s->pin, (atoi(val) != 0) ? HIGH : LOW);
  }
  else if (strcmp(verb, "ADC") == 0) {
    const char *name = strtok(NULL, " \t");
    const char *val  = strtok(NULL, " \t");
    if (!name || !val) return;
    const Signal *s = findSignal(kAdc, kAdcCount, name);
    if (!s) {
      sendLine("ERR unknown adc %s", name);
      return;
    }
    // "ADC <NAME> <high> [low]" -- one value is a steady level, two describe a
    // signal that swings between them. The firmware samples the pilot and the
    // ammeter peak-to-peak, so a level alone cannot represent either.
    long high = atol(val);
    const char *lowTok = strtok(NULL, " \t");
    long low = lowTok ? atol(lowTok) : high;
    if (high < 0) high = 0;
    if (low  < 0) low  = 0;
    if (high > ADC_MAX) high = ADC_MAX;
    if (low  > ADC_MAX) low  = ADC_MAX;
    nativeSetAdc(s->pin, (uint16_t)high, (uint16_t)low);
  }
  else if (strcmp(verb, "SNAP") == 0) {
    g_lastOutValid = false; // force a full republish
    sendSnapshot();
  }
  else if (strcmp(verb, "PING") == 0) {
    sendLine("PONG");
  }
  else {
    sendLine("ERR unknown command %s", verb);
  }
}

void pollClient()
{
  if (g_clientFd < 0) return;

  for (;;) {
    ssize_t n = read(g_clientFd, g_rxBuf + g_rxLen, sizeof(g_rxBuf) - g_rxLen - 1);
    if (n == 0) { // driver disconnected
      close(g_clientFd);
      g_clientFd = -1;
      g_rxLen = 0;
      return;
    }
    if (n < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK) break;
      close(g_clientFd);
      g_clientFd = -1;
      g_rxLen = 0;
      return;
    }

    g_rxLen += (size_t)n;
    g_rxBuf[g_rxLen] = '\0';

    char *start = g_rxBuf;
    char *nl;
    while ((nl = strchr(start, '\n')) != NULL) {
      *nl = '\0';
      char *end = nl;
      if (end > start && end[-1] == '\r') end[-1] = '\0';
      if (*start) handleLine(start);
      start = nl + 1;
    }

    // keep any partial line
    size_t remain = g_rxLen - (size_t)(start - g_rxBuf);
    memmove(g_rxBuf, start, remain);
    g_rxLen = remain;

    if (g_rxLen >= sizeof(g_rxBuf) - 1) { // overlong line, drop it
      g_rxLen = 0;
      sendLine("ERR line too long");
    }
  }
}

void publishChanges()
{
  if (g_clientFd < 0) return;

  for (size_t i = 0; i < kDigitalOutCount; i++) {
    const uint8_t v = digitalWriteValue(kDigitalOut[i].pin) ? 1 : 0;
    if (!g_lastOutValid || g_lastOut[i] != v) {
      sendLine("OUT %s %d", kDigitalOut[i].name, v);
      g_lastOut[i] = v;
    }
  }
  g_lastOutValid = true;

  if (g_pilotDirty) {
    sendLine("PILOT %s %u %d", pilotStateName(g_pilotState),
             (unsigned)g_pilotDuty, g_pilotAmps);
    g_pilotDirty = false;
  }
}

} // namespace

// ---------------------------------------------------------------------------

void channelBegin()
{
  if (g_begun) return;
  g_begun = true;

  const char *path = getenv("OPENEVSE_HW_SOCKET");
  if (!path || !*path) return; // channel disabled; run standalone

  struct sockaddr_un addr;
  memset(&addr, 0, sizeof(addr));
  addr.sun_family = AF_UNIX;
  if (strlen(path) >= sizeof(addr.sun_path)) {
    fprintf(stderr, "native: OPENEVSE_HW_SOCKET path too long\n");
    return;
  }
  strncpy(addr.sun_path, path, sizeof(addr.sun_path) - 1);

  g_listenFd = socket(AF_UNIX, SOCK_STREAM, 0);
  if (g_listenFd < 0) {
    fprintf(stderr, "native: socket() failed: %s\n", strerror(errno));
    return;
  }

  unlink(path); // a stale socket from a previous run would block bind()
  if (bind(g_listenFd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
    fprintf(stderr, "native: bind(%s) failed: %s\n", path, strerror(errno));
    close(g_listenFd);
    g_listenFd = -1;
    return;
  }
  if (listen(g_listenFd, 1) < 0) {
    fprintf(stderr, "native: listen() failed: %s\n", strerror(errno));
    close(g_listenFd);
    g_listenFd = -1;
    return;
  }
  fcntl(g_listenFd, F_SETFL, O_NONBLOCK);

  fprintf(stderr, "native: hardware channel on %s\n", path);

  // Also service from yield(). WDT_RESET() and the pin reads cover the wait
  // loops in the shared firmware, but not a tight sequence that only writes --
  // Gfi::SelfTest pulses GFITEST with nothing but delayMicroseconds() between
  // the edges. Registering here means delay() itself services the channel, so
  // the pulse is published while it is still high and the driver can answer it.
  epoxyRegisterYieldServiceCallback(
      [](void *) { channelService(); }, NULL);

  // Wait for the driver before going any further.
  //
  // With a channel configured, the hardware does not exist until something is
  // on the other end of it. Booting ahead of that means the power-on self
  // tests run against nothing: the GFI test pulses the coil, no trip comes
  // back, and the firmware latches EVSE_STATE_GFI_TEST_FAILED before the
  // driver ever attaches -- which then looks like a firmware fault rather than
  // an absent bench. Waiting here is the equivalent of powering the board up
  // with its hardware attached.
  //
  // Bounded so an unattended run still makes progress: OPENEVSE_HW_WAIT_MS,
  // default 5000, 0 to not wait at all.
  unsigned long waitMs = 5000;
  const char *waitEnv = getenv("OPENEVSE_HW_WAIT_MS");
  if (waitEnv && *waitEnv) waitMs = strtoul(waitEnv, NULL, 10);
  if (waitMs == 0) return;

  const unsigned long start = millis();
  while ((millis() - start) < waitMs) {
    channelService();
    if (g_clientFd >= 0) {
      fprintf(stderr, "native: driver attached\n");
      return;
    }
    delay(10);
  }
  fprintf(stderr, "native: no driver after %lums, continuing without one\n",
          waitMs);
}

void channelService()
{
  if (g_listenFd < 0) return;

  static bool inService = false;
  if (inService) return;
  inService = true;

  if (g_clientFd < 0) {
    int fd = accept(g_listenFd, NULL, NULL);
    if (fd >= 0) {
      fcntl(fd, F_SETFL, O_NONBLOCK);
      g_clientFd = fd;
      g_rxLen = 0;
      g_lastOutValid = false;
      sendSnapshot();
    }
  }

  pollClient();
  publishChanges();

  inService = false;
}

void channelPublishPilot(uint8_t state, uint32_t dutyTenthsPct, int amps)
{
  if (state == g_pilotState && dutyTenthsPct == g_pilotDuty && amps == g_pilotAmps) {
    return;
  }
  g_pilotState = state;
  g_pilotDuty  = dutyTenthsPct;
  g_pilotAmps  = amps;
  g_pilotDirty = true;
}
