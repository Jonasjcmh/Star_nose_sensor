/*
 * pressure_viewer.c
 *
 * Live SDL2 graph of the ESP32-C6 + Honeywell 100MDAA5 pressure sensor.
 * Reads "millis,voltage_V,pressure_mbar" lines directly from a serial
 * port and plots the chosen channel. Press SPACE, or click the green/
 * blue strip in the top-right corner, to switch between Pressure and
 * Voltage.
 *
 * Build (Linux/macOS, needs SDL2 dev package):
 *   gcc pressure_viewer.c -o pressure_viewer $(sdl2-config --cflags --libs)
 *
 *   Debian/Ubuntu: sudo apt install libsdl2-dev
 *   macOS (brew):  brew install sdl2
 *
 * Run:
 *   ./pressure_viewer /dev/ttyACM0 115200
 *
 * Note: this uses POSIX termios for the serial port, so it targets
 * Linux/macOS. On Windows, build under WSL, or replace open_serial()
 * and the read loop with the Win32 CreateFile/ReadFile API.
 */

#include <SDL2/SDL.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define WIN_W 900
#define WIN_H 500
#define GRAPH_MARGIN 40
#define MAX_POINTS 600

typedef struct {
    float voltage[MAX_POINTS];
    float pressure[MAX_POINTS];
    int   count;
    int   head; /* ring buffer start */
} DataBuf;

static DataBuf g_buf = {0};
static volatile int g_mode_pressure = 1; /* 1 = pressure, 0 = voltage */

static int open_serial(const char *path, int baud) {
    int fd = open(path, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) { perror("open"); return -1; }

    struct termios tty;
    memset(&tty, 0, sizeof tty);
    if (tcgetattr(fd, &tty) != 0) { perror("tcgetattr"); return -1; }

    speed_t speed;
    switch (baud) {
        case 9600:   speed = B9600;   break;
        case 57600:  speed = B57600;  break;
        default:     speed = B115200; break;
    }
    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_iflag &= ~IGNBRK;
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 1; /* 100ms read timeout */
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) { perror("tcsetattr"); return -1; }
    return fd;
}

static void push_sample(float v, float p) {
    int idx;
    if (g_buf.count < MAX_POINTS) {
        idx = (g_buf.head + g_buf.count) % MAX_POINTS;
        g_buf.count++;
    } else {
        idx = g_buf.head;
        g_buf.head = (g_buf.head + 1) % MAX_POINTS;
    }
    g_buf.voltage[idx]  = v;
    g_buf.pressure[idx] = p;
}

static int serial_thread(void *data) {
    int fd = *(int *)data;
    char line[256];
    int len = 0;
    char c;
    while (1) {
        int n = read(fd, &c, 1);
        if (n <= 0) { SDL_Delay(5); continue; }
        if (c == '\n') {
            line[len] = '\0';
            len = 0;
            float ms, v, p;
            if (sscanf(line, "%f,%f,%f", &ms, &v, &p) == 3) {
                push_sample(v, p);
            }
        } else if (c != '\r' && len < (int)sizeof(line) - 1) {
            line[len++] = c;
        }
    }
    return 0;
}

static void draw_graph(SDL_Renderer *ren) {
    SDL_SetRenderDrawColor(ren, 18, 18, 24, 255);
    SDL_RenderClear(ren);

    /* axes */
    SDL_SetRenderDrawColor(ren, 90, 90, 100, 255);
    SDL_RenderDrawLine(ren, GRAPH_MARGIN, WIN_H - GRAPH_MARGIN, WIN_W - 10, WIN_H - GRAPH_MARGIN);
    SDL_RenderDrawLine(ren, GRAPH_MARGIN, 10, GRAPH_MARGIN, WIN_H - GRAPH_MARGIN);

    /* mode toggle strip, top-right: left=pressure, right=voltage */
    SDL_Rect pRect = { WIN_W - 190, 10, 80, 24 };
    SDL_Rect vRect = { WIN_W - 100, 10, 80, 24 };
    SDL_SetRenderDrawColor(ren, g_mode_pressure ? 60 : 40, g_mode_pressure ? 200 : 40, 90, 255);
    SDL_RenderFillRect(ren, &pRect);
    SDL_SetRenderDrawColor(ren, g_mode_pressure ? 40 : 60, g_mode_pressure ? 40 : 160, g_mode_pressure ? 60 : 220, 255);
    SDL_RenderFillRect(ren, &vRect);

    char title[128];
    snprintf(title, sizeof title, "Pressure Viewer — mode: %s (SPACE to toggle)",
              g_mode_pressure ? "PRESSURE (mbar)" : "VOLTAGE (V)");
    SDL_SetWindowTitle(SDL_RenderGetWindow(ren), title);

    int n = g_buf.count;
    if (n < 2) { SDL_RenderPresent(ren); return; }

    float minV = 1e9f, maxV = -1e9f;
    static float vals[MAX_POINTS];
    for (int i = 0; i < n; i++) {
        int idx = (g_buf.head + i) % MAX_POINTS;
        float y = g_mode_pressure ? g_buf.pressure[idx] : g_buf.voltage[idx];
        vals[i] = y;
        if (y < minV) minV = y;
        if (y > maxV) maxV = y;
    }
    if (maxV - minV < 0.01f) { maxV += 0.5f; minV -= 0.5f; }

    int plotW = WIN_W - GRAPH_MARGIN - 10;
    int plotH = WIN_H - GRAPH_MARGIN - 10;

    SDL_SetRenderDrawColor(ren, 80, 220, 255, 255);
    for (int i = 1; i < n; i++) {
        int x1 = GRAPH_MARGIN + (int)((float)(i - 1) / (n - 1) * plotW);
        int x2 = GRAPH_MARGIN + (int)((float)i / (n - 1) * plotW);
        int y1 = 10 + plotH - (int)((vals[i - 1] - minV) / (maxV - minV) * plotH);
        int y2 = 10 + plotH - (int)((vals[i] - minV) / (maxV - minV) * plotH);
        SDL_RenderDrawLine(ren, x1, y1, x2, y2);
    }

    SDL_RenderPresent(ren);
}

int main(int argc, char **argv) {
    if (argc < 2) {
        fprintf(stderr, "Usage: %s <serial_port> [baud]\n", argv[0]);
        return 1;
    }
    int baud = argc > 2 ? atoi(argv[2]) : 115200;
    int fd = open_serial(argv[1], baud);
    if (fd < 0) return 1;

    if (SDL_Init(SDL_INIT_VIDEO) != 0) {
        fprintf(stderr, "SDL_Init failed: %s\n", SDL_GetError());
        return 1;
    }
    SDL_Window *win = SDL_CreateWindow("Pressure Viewer (SPACE to toggle V/P)",
        SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, WIN_W, WIN_H, SDL_WINDOW_SHOWN);
    SDL_Renderer *ren = SDL_CreateRenderer(win, -1, SDL_RENDERER_ACCELERATED);

    SDL_Thread *thread = SDL_CreateThread(serial_thread, "serial", &fd);
    (void)thread;

    int running = 1;
    SDL_Event ev;
    while (running) {
        while (SDL_PollEvent(&ev)) {
            if (ev.type == SDL_QUIT) running = 0;
            if (ev.type == SDL_KEYDOWN && ev.key.keysym.sym == SDLK_SPACE)
                g_mode_pressure = !g_mode_pressure;
            if (ev.type == SDL_MOUSEBUTTONDOWN) {
                int mx = ev.button.x, my = ev.button.y;
                if (my >= 10 && my <= 34) {
                    if (mx >= WIN_W - 190 && mx <= WIN_W - 110) g_mode_pressure = 1;
                    if (mx >= WIN_W - 100 && mx <= WIN_W - 20)  g_mode_pressure = 0;
                }
            }
        }
        draw_graph(ren);
        SDL_Delay(16);
    }

    close(fd);
    SDL_DestroyRenderer(ren);
    SDL_DestroyWindow(win);
    SDL_Quit();
    return 0;
}