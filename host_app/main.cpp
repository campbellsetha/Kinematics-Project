#include <ncurses.h>
#include <libserialport.h>

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <thread>

// ---------------------------------------------------------------------------
// Shared robot state — updated by serial reader thread, read by UI thread
// ---------------------------------------------------------------------------

struct SYSTEMSTATE {
    uint16_t leader_joints[6]   = {};
    uint16_t leader_speeds[6]   = {};
    uint16_t follower_joints[6] = {};
};

static SYSTEMSTATE       g_state;
static std::mutex        g_state_mutex;
static std::atomic<bool> g_running{true};

// ---------------------------------------------------------------------------
// Serial
// ---------------------------------------------------------------------------

static sp_port* g_port = nullptr;

static bool serialOpen(const char* port_name) {
    if (sp_get_port_by_name(port_name, &g_port) != SP_OK) return false;
    if (sp_open(g_port, SP_MODE_READ_WRITE) != SP_OK)      return false;
    sp_set_baudrate(g_port,     115200);
    sp_set_bits(g_port,         8);
    sp_set_parity(g_port,       SP_PARITY_NONE);
    sp_set_stopbits(g_port,     1);
    sp_set_flowcontrol(g_port,  SP_FLOWCONTROL_NONE);
    return true;
}

static void serialClose() {
    if (g_port) {
        sp_close(g_port);
        sp_free_port(g_port);
        g_port = nullptr;
    }
}

static void sendOperationMode(uint8_t mode) {
    if (!g_port) return;
    sp_blocking_write(g_port, &mode, 1, 100);
}

// ---------------------------------------------------------------------------
// Packet framing
//
// Wire format (28 bytes total):
//   [0]      0xAA          sync byte 0  — cannot appear in 12-bit servo data
//   [1]      0x55          sync byte 1  — cannot appear in 12-bit servo data
//   [2]      source        LEADER=2, FOLLOWER=3 (ROLE enum values)
//   [3]      0xFF          msgTyp        (ALL_POSITIONS)
//   [4..15]  positions     6 × uint16_t  servo positions (little-endian ticks)
//   [16..27] speeds        6 × uint16_t  servo speeds    (little-endian ticks)
//
// The reader accumulates bytes and syncs on the two-byte sequence 0xAA 0x55.
// Because servo values are 12-bit (0–4095 = 0x0000–0x0FFF), the high byte of
// every field is at most 0x0F (15), so 0xAA (170) can never appear inside the
// payload — false sync is structurally impossible.
// ---------------------------------------------------------------------------

static constexpr uint8_t SYNC0        = 0xAA;
static constexpr uint8_t SYNC1        = 0x55;
static constexpr uint8_t SRC_LEADER   = 2;
static constexpr uint8_t SRC_FOLLOWER = 3;
static constexpr int     PAYLOAD_SIZE = 25;    // sizeof(MSGALLPOSITIONS) with pack(1)
static constexpr int     PACKET_SIZE  = 28;    // 2 sync + 1 source + 25 payload

static void parsePacket(const uint8_t* buf) {
    // buf[0..1]=sync, buf[2]=source, buf[3]=msgTyp, buf[4..15]=positions, buf[16..27]=speeds
    // Only update the fields that belong to this source — don't zero unrelated fields.
    std::lock_guard<std::mutex> lock(g_state_mutex);
    if (buf[2] == SRC_LEADER) {
        std::memcpy(g_state.leader_joints, buf + 4,  12);
        std::memcpy(g_state.leader_speeds, buf + 16, 12);
    } else if (buf[2] == SRC_FOLLOWER) {
        // Only update joints where the read succeeded — preserve last known good on 0xFFFF.
        const uint16_t* src = reinterpret_cast<const uint16_t*>(buf + 4);
        for (int i = 0; i < 6; i++)
            if (src[i] != 0xFFFF) g_state.follower_joints[i] = src[i];
    }
}

static void serialReaderThread() {
    uint8_t accum[PACKET_SIZE * 2] = {};
    int     fill = 0;

    while (g_running) {
        if (!g_port) {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            continue;
        }

        int space = static_cast<int>(sizeof(accum)) - fill;
        int n = sp_nonblocking_read(g_port, accum + fill, space);
        if (n > 0) fill += n;

        while (fill >= PACKET_SIZE) {
            if (accum[0] != SYNC0 || accum[1] != SYNC1) {
                // Scan forward for the next 0xAA 0x55 pair
                int skip = 1;
                while (skip + 1 < fill &&
                       !(accum[skip] == SYNC0 && accum[skip + 1] == SYNC1))
                    ++skip;
                std::memmove(accum, accum + skip, fill - skip);
                fill -= skip;
                continue;
            }
            parsePacket(accum);
            std::memmove(accum, accum + PACKET_SIZE, fill - PACKET_SIZE);
            fill -= PACKET_SIZE;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

// ---------------------------------------------------------------------------
// Startup diagnostic
//
// Validates each hop of the signal path before entering the main menu:
//   Step 1 — Serial port open
//   Step 2 — Bytes arriving on the port
//   Step 3 — Valid framed packets decoding successfully
//   Step 4 — Per-servo health (0xFFFF = Leader UART read failure)
// ---------------------------------------------------------------------------

struct DiagResult {
    bool     serial_open       = false;
    int      bytes_rx          = 0;
    int      packets_rx        = 0;
    bool     servo_ok[6]       = {};   // at least one non-0xFFFF reading
    bool     servo_fail[6]     = {};   // at least one 0xFFFF reading
    uint16_t last_good_pos[6]  = {};   // last non-sentinel position seen
};

// Reads directly from g_port for scan_ms ms without touching the reader thread.
// Call only before the reader thread is started.
static DiagResult collectDiagData(int scan_ms) {
    DiagResult res;
    res.serial_open = (g_port != nullptr);
    if (!g_port) return res;

    uint8_t accum[PACKET_SIZE * 4] = {};
    int     fill = 0;

    auto deadline = std::chrono::steady_clock::now()
                  + std::chrono::milliseconds(scan_ms);

    while (std::chrono::steady_clock::now() < deadline) {
        int space = static_cast<int>(sizeof(accum)) - fill;
        int n = sp_nonblocking_read(g_port, accum + fill, space);
        if (n > 0) {
            res.bytes_rx += n;
            fill += n;
        }

        while (fill >= PACKET_SIZE) {
            if (accum[0] != SYNC0 || accum[1] != SYNC1) {
                int skip = 1;
                while (skip + 1 < fill &&
                       !(accum[skip] == SYNC0 && accum[skip + 1] == SYNC1))
                    ++skip;
                std::memmove(accum, accum + skip, fill - skip);
                fill -= skip;
                continue;
            }

            res.packets_rx++;
            // positions start at buf[4] (after 2 sync + source + msgTyp), 2 bytes each, little-endian
            for (int i = 0; i < 6; i++) {
                uint16_t pos;
                std::memcpy(&pos, accum + 4 + i * 2, sizeof(pos));
                if (pos == 0xFFFF) {
                    res.servo_fail[i] = true;
                } else {
                    res.servo_ok[i]      = true;
                    res.last_good_pos[i] = pos;
                }
            }
            std::memmove(accum, accum + PACKET_SIZE, fill - PACKET_SIZE);
            fill -= PACKET_SIZE;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return res;
}

static void drawDiagnosticScreen(const DiagResult& d,
                                  const char*       port_name,
                                  int               scan_ms,
                                  bool              scanning) {
    clear();
    mvprintw(1, 2, "=== DR1 Signal Path Diagnostic ===");

    int row = 3;

    // Step 1 — serial port
    if (d.serial_open)
        mvprintw(row, 2, "[1/4] Serial port ............ OPEN   %s", port_name);
    else
        mvprintw(row, 2, "[1/4] Serial port ............ FAILED %s", port_name);
    ++row;

    // Step 2 — byte stream
    if (scanning) {
        mvprintw(row, 2, "[2/4] Scanning ............... collecting %d s ...", scan_ms / 1000);
        mvprintw(row + 1, 2, "[3/4] Packet decode ..........");
        mvprintw(row + 2, 2, "[4/4] Servo health ...........");
        mvprintw(row + 9, 2, "Please wait...");
        refresh();
        return;
    }

    mvprintw(row, 2, "[2/4] Data received .......... %d bytes", d.bytes_rx);
    ++row;

    // Step 3 — packet decode
    if (d.packets_rx == 0) {
        mvprintw(row, 2, "[3/4] Packet decode .......... NO VALID PACKETS  (expected ~50/s)");
    } else {
        double rate = d.packets_rx * 1000.0 / scan_ms;
        mvprintw(row, 2, "[3/4] Packet decode .......... %d pkts  (%.1f /s, expected ~50/s)",
                 d.packets_rx, rate);
    }
    ++row;

    // Step 4 — per-servo health
    mvprintw(row++, 2, "[4/4] Servo health:");
    const char* names[] = {"J1", "J2", "J3", "J4", "J5", "J6"};
    for (int i = 0; i < 6; i++) {
        if (!d.servo_ok[i] && !d.servo_fail[i]) {
            mvprintw(row, 6, "%s  [  ---  ]  no data received", names[i]);
        } else if (d.servo_fail[i] && !d.servo_ok[i]) {
            mvprintw(row, 6, "%s  [ FAIL  ]  0xFFFF -- servo not responding on Leader UART", names[i]);
        } else if (d.servo_fail[i]) {
            mvprintw(row, 6, "%s  [ INTERM]  intermittent -- last good pos = %-5u",
                     names[i], d.last_good_pos[i]);
        } else {
            mvprintw(row, 6, "%s  [  OK   ]  pos = %-5u", names[i], d.last_good_pos[i]);
        }
        ++row;
    }

    // Summary line
    ++row;
    bool any_hard_fail = false;
    for (int i = 0; i < 6; i++)
        if (d.servo_fail[i] && !d.servo_ok[i]) any_hard_fail = true;

    if (!d.serial_open)
        mvprintw(row, 2, "[ ERROR ] Serial port not open — check USB cable and port name");
    else if (d.bytes_rx == 0)
        mvprintw(row, 2, "[ ERROR ] No bytes received — check Primary ESP power and USB cable");
    else if (d.packets_rx == 0)
        mvprintw(row, 2, "[ ERROR ] Bytes received but no valid packets — reflash firmware");
    else if (any_hard_fail)
        mvprintw(row, 2, "[ WARN  ] Some servos not responding -- check 12V supply and Leader UART wiring");
    else
        mvprintw(row, 2, "[ PASS  ] Signal path healthy");

    ++row; ++row;
    mvprintw(row, 2, "ENTER / SPACE = continue    R = rescan    Q = quit");
    refresh();
}

// Returns true to continue into the main menu, false to quit.
static bool runStartupDiagnostic(const char* port_name) {
    DiagResult empty{};
    empty.serial_open = (g_port != nullptr);

    while (true) {
        // Show "scanning" placeholder
        drawDiagnosticScreen(empty, port_name, 3000, true);

        // Collect 3 seconds of data synchronously
        DiagResult result = collectDiagData(3000);

        // Show results
        drawDiagnosticScreen(result, port_name, 3000, false);

        timeout(-1);
        while (true) {
            int ch = getch();
            if (ch == '\n' || ch == '\r' || ch == ' ')
                return true;
            if (ch == 'r' || ch == 'R')
                break;           // re-run scan
            if (ch == 'q' || ch == 'Q')
                return false;
        }
    }
}

// ---------------------------------------------------------------------------
// ncurses UI
// ---------------------------------------------------------------------------

static constexpr int L_LABEL_COL  = 2;
static constexpr int L_VALUE_COL  = 9;
static constexpr int SEP_COL      = 18;
static constexpr int R_LABEL_COL  = 21;
static constexpr int R_VALUE_COL  = 28;
static constexpr int JOINT_ROW[6] = {5, 6, 7, 8, 9, 10};

static int drawMainMenu() {
    clear();
    mvprintw(1, 2, "=== DR1 Kinematics Controller ===");
    mvprintw(3, 2, "1]  Teleoperation");
    mvprintw(4, 2, "2]  Waypoint");
    mvprintw(6, 2, "Q]  Quit");
    mvprintw(8, 2, "Select: ");
    refresh();

    while (true) {
        int ch = getch();
        if (ch == '1' || ch == '2' || ch == 'q' || ch == 'Q')
            return ch;
    }
}

static void drawStatusTable(const char* title) {
    clear();
    mvprintw(1, 2, "=== %s ===", title);

    mvprintw(3, L_LABEL_COL, "Joint");
    mvprintw(3, L_VALUE_COL, "Leader");
    mvprintw(4, L_LABEL_COL, "-----");
    mvprintw(4, L_VALUE_COL, "------");

    for (int r = 3; r <= 11; r++) mvprintw(r, SEP_COL, "|");

    mvprintw(3, R_LABEL_COL, "Joint");
    mvprintw(3, R_VALUE_COL, "Follower");
    mvprintw(4, R_LABEL_COL, "-----");
    mvprintw(4, R_VALUE_COL, "--------");

    const char* labels[] = {"J1", "J2", "J3", "J4", "J5", "J6"};
    for (int i = 0; i < 6; i++) {
        mvprintw(JOINT_ROW[i], L_LABEL_COL, labels[i]);
        mvprintw(JOINT_ROW[i], R_LABEL_COL, labels[i]);
    }

    mvprintw(13, 2, "B]  Back");
    refresh();
}

static void updateValues() {
    SYSTEMSTATE s;
    {
        std::lock_guard<std::mutex> lock(g_state_mutex);
        s = g_state;
    }

    for (int i = 0; i < 6; i++) {
        if (s.leader_joints[i] == 0xFFFF)
            mvprintw(JOINT_ROW[i], L_VALUE_COL, "%-7s", "---");
        else
            mvprintw(JOINT_ROW[i], L_VALUE_COL, "%-7d", (int)(s.leader_joints[i] * 0.08789));

        if (s.follower_joints[i] == 0xFFFF)
            mvprintw(JOINT_ROW[i], R_VALUE_COL, "%-7s", "---");
        else
            mvprintw(JOINT_ROW[i], R_VALUE_COL, "%-7d", (int)(s.follower_joints[i] * 0.08789));
    }

    refresh();
}

static void runOperationView(const char* title, uint8_t mode_byte) {
    sendOperationMode(mode_byte);
    drawStatusTable(title);
    timeout(500);

    while (true) {
        updateValues();
        int ch = getch();
        if (ch == 'b' || ch == 'B') break;
    }

    timeout(-1);
}

// ---------------------------------------------------------------------------
// Entry point
// ---------------------------------------------------------------------------

int main(int argc, char* argv[]) {
    signal(SIGPIPE, SIG_IGN);

    if (argc < 2) {
        fprintf(stderr, "Usage: %s <serial_port>\n  e.g. %s /dev/tty.usbmodem-0001\n",
                argv[0], argv[0]);
        return 1;
    }

    const char* port_name = argv[1];
    if (!serialOpen(port_name))
        fprintf(stderr, "Warning: could not open %s — running without serial\n", port_name);

    initscr();
    cbreak();
    noecho();
    keypad(stdscr, FALSE);
    curs_set(0);

    // Validate the full signal path before entering the main menu.
    // collectDiagData reads directly from g_port — the reader thread is not
    // started yet, so there is no concurrent access.
    if (!runStartupDiagnostic(port_name)) {
        endwin();
        serialClose();
        return 0;
    }

    // Hand the serial port over to the reader thread
    std::thread reader(serialReaderThread);

    bool running = true;
    while (running) {
        int choice = drawMainMenu();
        switch (choice) {
            case '1': runOperationView("Teleoperation", 0x00); break;
            case '2': runOperationView("Waypoint",      0x01); break;
            case 'q':
            case 'Q': running = false; break;
        }
    }

    endwin();
    g_running = false;
    reader.join();
    serialClose();

    return 0;
}
