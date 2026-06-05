// Author: Seth Campbell
// main.cpp
// Host-side terminal UI for monitoring and controlling the robot from a PC.
// Opens a serial connection to the Primary board, parses telemetry packets,
// and presents a live ncurses display with three modes: Teleoperation, Waypoint, and Axis Test.

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

// ── Shared robot state ────────────────────────────────────────────────────────
// Updated by the serial reader thread; read by the UI thread under g_state_mutex.

struct SystemState {
    uint16_t leader_joints[6]   = {};
    uint16_t follower_joints[6] = {};
    float    ee_x               = 0.0f;
    float    ee_y               = 0.0f;
    float    ee_z               = 0.0f;
};

static SystemState       g_state;
static std::mutex        g_state_mutex;
static std::atomic<bool> g_running{true};

// ── Serial port helpers ───────────────────────────────────────────────────────

static sp_port* g_port = nullptr;

// Opens the given serial port at 115200 8N1 and returns true on success.
static bool serial_open(const char* port_name) {
    if (sp_get_port_by_name(port_name, &g_port) != SP_OK) return false;
    if (sp_open(g_port, SP_MODE_READ_WRITE) != SP_OK)      return false;
    sp_set_baudrate(g_port,    115200);
    sp_set_bits(g_port,        8);
    sp_set_parity(g_port,      SP_PARITY_NONE);
    sp_set_stopbits(g_port,    1);
    sp_set_flowcontrol(g_port, SP_FLOWCONTROL_NONE);
    return true;
}

static void serial_close() {
    if (g_port) {
        sp_close(g_port);
        sp_free_port(g_port);
        g_port = nullptr;
    }
}

// Sends a single mode byte to the Primary board to switch the robot's operating mode.
static void send_operation_mode(uint8_t mode) {
    if (!g_port) return;
    sp_blocking_write(g_port, &mode, 1, 100);
}

// ── Packet framing ────────────────────────────────────────────────────────────
// Every packet from the Primary is 28 bytes:
//   [0–1]   0xAA 0x55      sync header
//   [2]     source         LEADER=2, FOLLOWER=3
//   [3]     msg_type       0xFF = ALL_POSITIONS
//   [4–15]  positions      6 x uint16_t servo ticks (little-endian)
//   [16–27] ee_x/y/z       3 x float end-effector metres (little-endian)
//
// 0xAA 0x55 cannot appear in the 12-bit servo payload, so false sync frames are impossible.

static constexpr uint8_t SYNC0        = 0xAA;
static constexpr uint8_t SYNC1        = 0x55;
static constexpr uint8_t SRC_LEADER   = 2;
static constexpr uint8_t SRC_FOLLOWER = 3;
static constexpr int     PAYLOAD_SIZE = 25;  // sizeof(MSGALLPOSITIONS) with pack(1)
static constexpr int     PACKET_SIZE  = 28;  // 2 sync + 1 source + 25 payload

// Copies the packet's servo ticks and XYZ fields into the shared state under lock.
static void parse_packet(const uint8_t* buf) {
    std::lock_guard<std::mutex> lock(g_state_mutex);
    if (buf[2] == SRC_LEADER) {
        std::memcpy(g_state.leader_joints, buf + 4,  12);
        std::memcpy(&g_state.ee_x,         buf + 16,  4);
        std::memcpy(&g_state.ee_y,         buf + 20,  4);
        std::memcpy(&g_state.ee_z,         buf + 24,  4);
    } else if (buf[2] == SRC_FOLLOWER) {
        const uint16_t* src = reinterpret_cast<const uint16_t*>(buf + 4);
        for (int i = 0; i < 6; i++)
            if (src[i] != 0xFFFF) g_state.follower_joints[i] = src[i];
    }
}

// Background thread: non-blocking reads from serial into a sliding buffer,
// finds sync headers, and hands complete packets to parse_packet().
static void serial_reader_thread() {
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
                // Scan forward to the next sync header and discard bytes before it
                int skip = 1;
                while (skip + 1 < fill &&
                       !(accum[skip] == SYNC0 && accum[skip + 1] == SYNC1))
                    ++skip;
                std::memmove(accum, accum + skip, fill - skip);
                fill -= skip;
                continue;
            }
            parse_packet(accum);
            std::memmove(accum, accum + PACKET_SIZE, fill - PACKET_SIZE);
            fill -= PACKET_SIZE;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

// ── ncurses layout constants ──────────────────────────────────────────────────

static constexpr int L_LABEL_COL  = 2;
static constexpr int L_VALUE_COL  = 9;
static constexpr int SEP_COL      = 18;
static constexpr int R_LABEL_COL  = 21;
static constexpr int R_VALUE_COL  = 28;
static constexpr int JOINT_ROW[6] = {5, 6, 7, 8, 9, 10};

// ── Teleoperation view ────────────────────────────────────────────────────────
// Displays live joint angles (in degrees) for both arms side by side,
// plus the leader's end-effector position in mm.

static void draw_teleop_layout() {
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

    mvprintw(12, L_LABEL_COL, "End Effector (mm):");
    mvprintw(13, L_LABEL_COL, "  X:");
    mvprintw(14, L_LABEL_COL, "  Y:");
    mvprintw(15, L_LABEL_COL, "  Z:");
    mvprintw(17, L_LABEL_COL, "B]  Back");
}

// Refreshes all numeric values on the teleop screen from the latest shared state.
// Tick values are multiplied by 0.08789 to convert to degrees (360 / 4096).
static void update_teleop_values() {
    SystemState s;
    {
        std::lock_guard<std::mutex> lock(g_state_mutex);
        s = g_state;
    }

    for (int i = 0; i < 6; i++) {
        if (s.leader_joints[i] == 0xFFFF)
            mvprintw(JOINT_ROW[i], L_VALUE_COL, "%-7s", "---");
        else
            mvprintw(JOINT_ROW[i], L_VALUE_COL, "%-7d",
                     (int)(s.leader_joints[i] * 0.08789f));

        if (s.follower_joints[i] == 0xFFFF)
            mvprintw(JOINT_ROW[i], R_VALUE_COL, "%-7s", "---");
        else
            mvprintw(JOINT_ROW[i], R_VALUE_COL, "%-7d",
                     (int)(s.follower_joints[i] * 0.08789f));
    }

    mvprintw(13, 7, "%8.2f", s.ee_x * 1000.0f);
    mvprintw(14, 7, "%8.2f", s.ee_y * 1000.0f);
    mvprintw(15, 7, "%8.2f", s.ee_z * 1000.0f);

    refresh();
}

// Sends TELEOP mode to the robot, draws the layout, and loops refreshing values until 'B' is pressed.
static void run_teleop_view() {
    send_operation_mode(0x00);
    clear();
    mvprintw(1, L_LABEL_COL, "=== Teleoperation ===");
    draw_teleop_layout();
    refresh();

    timeout(500);
    while (true) {
        update_teleop_values();
        int ch = getch();
        if (ch == 'b' || ch == 'B') break;
    }
    timeout(-1);
}

// ── Waypoint view ─────────────────────────────────────────────────────────────
// Shows leader joint angles and end-effector XYZ for FK verification.
// The follower is not active in this mode.

static void draw_waypoint_layout() {
    mvprintw(3, L_LABEL_COL, "Joint");
    mvprintw(3, L_VALUE_COL, "Leader (deg)");
    mvprintw(4, L_LABEL_COL, "-----");
    mvprintw(4, L_VALUE_COL, "------------");

    const char* labels[] = {"J1", "J2", "J3", "J4", "J5", "J6"};
    for (int i = 0; i < 6; i++)
        mvprintw(JOINT_ROW[i], L_LABEL_COL, labels[i]);

    mvprintw(12, L_LABEL_COL, "End Effector:");
    mvprintw(15, L_LABEL_COL, "B]  Back");
}

// Refreshes leader joint angles and end-effector XYZ on the waypoint screen.
static void update_waypoint_values() {
    SystemState s;
    {
        std::lock_guard<std::mutex> lock(g_state_mutex);
        s = g_state;
    }

    for (int i = 0; i < 6; i++) {
        if (s.leader_joints[i] == 0xFFFF)
            mvprintw(JOINT_ROW[i], L_VALUE_COL, "%-12s", "---");
        else
            mvprintw(JOINT_ROW[i], L_VALUE_COL, "%-12d",
                     (int)(s.leader_joints[i] * 0.08789f));
    }

    mvprintw(13, L_LABEL_COL,
             "X: %7.1f mm   Y: %7.1f mm   Z: %7.1f mm",
             s.ee_x * 1000.0f,
             s.ee_y * 1000.0f,
             s.ee_z * 1000.0f);

    refresh();
}

// Sends WAYPOINT mode to the robot and loops refreshing the display until 'B' is pressed.
static void run_waypoint_view() {
    send_operation_mode(0x01);
    clear();
    mvprintw(1, L_LABEL_COL, "=== Waypoint ===");
    draw_waypoint_layout();
    refresh();

    timeout(500);
    while (true) {
        update_waypoint_values();
        int ch = getch();
        if (ch == 'b' || ch == 'B') break;
    }
    timeout(-1);
}

// ── Axis test view ────────────────────────────────────────────────────────────
// Stub — sends AXIS_TEST mode to the robot and shows a placeholder screen.

static void run_axis_test_view() {
    send_operation_mode(0x02);
    clear();
    mvprintw(1, L_LABEL_COL, "=== Axis Test ===");
    mvprintw(3, L_LABEL_COL, "Not yet implemented.");
    mvprintw(5, L_LABEL_COL, "B]  Back");
    refresh();

    timeout(-1);
    while (true) {
        int ch = getch();
        if (ch == 'b' || ch == 'B') break;
    }
}

// ── Main menu ─────────────────────────────────────────────────────────────────

// Draws the top-level menu and blocks until the user presses a valid key.
static int draw_main_menu() {
    clear();
    mvprintw(1, 2, "=== DR1 Kinematics Controller ===");
    mvprintw(3, 2, "1]  Teleoperation");
    mvprintw(4, 2, "2]  Waypoint");
    mvprintw(5, 2, "3]  Axis Test");
    mvprintw(7, 2, "Q]  Quit");
    mvprintw(9, 2, "Select: ");
    refresh();

    while (true) {
        int ch = getch();
        if (ch == '1' || ch == '2' || ch == '3' || ch == 'q' || ch == 'Q')
            return ch;
    }
}

// ── Entry point ───────────────────────────────────────────────────────────────

// Expects a serial port path as the first argument (e.g. /dev/tty.usbmodem-0001).
// Starts the serial reader thread, then runs the ncurses menu loop until the user quits.
int main(int argc, char* argv[]) {
    signal(SIGPIPE, SIG_IGN);

    if (argc < 2) {
        fprintf(stderr, "Usage: %s <serial_port>\n  e.g. %s /dev/tty.usbmodem-0001\n",
                argv[0], argv[0]);
        return 1;
    }

    const char* port_name = argv[1];
    if (!serial_open(port_name))
        fprintf(stderr, "Warning: could not open %s — running without serial\n", port_name);

    initscr();
    cbreak();
    noecho();
    keypad(stdscr, FALSE);
    curs_set(0);

    std::thread reader(serial_reader_thread);

    bool running = true;
    while (running) {
        int choice = draw_main_menu();
        switch (choice) {
            case '1': run_teleop_view();    break;
            case '2': run_waypoint_view();  break;
            case '3': run_axis_test_view(); break;
            case 'q':
            case 'Q': running = false;      break;
        }
    }

    endwin();
    g_running = false;
    reader.join();
    serial_close();

    return 0;
}