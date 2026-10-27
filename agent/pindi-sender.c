/*
 * pindi-sender - PiNDI outgoing NDI (or H.264 RTP, SRT) streams from USB (UVC) HDMI capture devices and
 * CSI-2 HDMI bridges (TC358743 on a camera connector).
 *
 * Nothing is sent unless configured (web UI "Configure Outgoing Streams"):
 *   PINDI_IN<n>_ENABLED=1
 *   PINDI_IN<n>_DEVICE=/dev/v4l/by-id/...      stable device path (a CSI-2 bridge: its receiver's video node,
 *                                              e.g. /dev/v4l/by-path/platform-1f00110000.csi-video-index0)
 *   PINDI_IN<n>_NAME=IN_<serial>               NDI stream name (default IN_<serial>[_n])
 *   PINDI_IN<n>_MODE="MJPG 1920x1080@30"       optional, else the best mode up to 1080p30
 *   PINDI_IN<n>_AUDIO=auto                     the device's own USB audio (default), or off
 *   PINDI_IN<n>_AUDIO_SYNC=off                 send the audio as the device's clock delivers it (default:
 *                                              kept in step with this Pi's clock)
 *   PINDI_IN<n>_CODEC=speedhq                  Full NDI (default), hx (NDI|HX), rtp (H.264 over RTP, no NDI)
 *                                              or srt (H.264 in MPEG-TS over SRT, no NDI)
 *   PINDI_IN<n>_RTP_DEST=239.255.0.1:5004      rtp: where to send it (unicast or multicast address and port)
 *   PINDI_IN<n>_SRT_URL=srt://:9000            srt: listen on that port, or srt://host:port to call a receiver;
 *                                              ?passphrase=...&latency=<ms>&streamid=... (as for ffmpeg)
 *   PINDI_IN<n>_KBPS=                          rtp, srt: video bit rate in kbit/s (default: about 6 Mbit/s at
 *                                              1080p30)
 *
 * One thread per input: V4L2 capture -> decode workers (MJPEG decode straight to YUV, two threads
 * at HD) -> NDI send, and one for its audio: ALSA capture (48 kHz stereo) -> NDI send.
 * Status for the web UI in /run/pindi/in<n>.status.
 *
 * The decoded pictures and the audio are also published in shared memory (/run/pindi/in<n>.local,
 * struct local_feed). pindi-player shows an input of this device from there instead of receiving it
 * over NDI: no NDI encoding and decoding on the same Pi, which a Pi 3 cannot do at 1080p60.
 */
#define _GNU_SOURCE
#include <arpa/inet.h>
#include <ctype.h>
#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <math.h>
#include <netdb.h>
#include <netinet/in.h>
#include <poll.h>
#include <pthread.h>
#include <signal.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/random.h>
#include <sys/resource.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/sysmacros.h>
#include <sys/uio.h>
#include <time.h>
#include <unistd.h>

#include <linux/media-bus-format.h>
#include <linux/media.h>
#include <linux/v4l2-subdev.h>
#include <linux/videodev2.h>
#include <turbojpeg.h>
#include <alsa/asoundlib.h>

#include <dlfcn.h>
#include <Processing.NDI.Lib.h>
#include <Processing.NDI.DynamicLoad.h>

#if defined(__ARM_NEON) || defined(__ARM_NEON__)
#include <arm_neon.h>
#endif

#define LOG(...) do { printf(__VA_ARGS__); putchar('\n'); } while (0)
#define MAX_IN 8
#define NCAP 4
#define RUN_DIR "/run/pindi"
#define AUDIO_RATE 48000

static const NDIlib_v6 *ndi;
static volatile sig_atomic_t running = 1;
/* The NDI Advanced SDK's evaluation build said its 30 minutes are up: NDI|HX video stops (audio and
 * Full NDI go on). Only the library's own message tells, so its output is watched (watch_output). */
static volatile sig_atomic_t ndi_eval_over;
static void on_signal(int sig) { (void)sig; running = 0; }
static char serial[64];

static uint64_t now_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000 + ts.tv_nsec / 1000000;
}

static uint64_t now_us(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000 + ts.tv_nsec / 1000;
}

static const char *icfg(int n, const char *key, const char *def)
{
    char k[64];
    snprintf(k, sizeof k, "PINDI_IN%d_%s", n, key);
    const char *v = getenv(k);
    return (v && *v) ? v : def;
}

static void read_serial(void)
{
    FILE *f = fopen("/sys/firmware/devicetree/base/serial-number", "rb");
    size_t len = f ? fread(serial, 1, sizeof serial - 1, f) : 0, j = 0;
    if (f)
        fclose(f);
    for (size_t i = 0; i < len; i++)
        if (isalnum((unsigned char)serial[i]))
            serial[j++] = serial[i];
    serial[j] = 0;
    if (!serial[0])
        snprintf(serial, sizeof serial, "unknown");
}

static void write_file_atomic(const char *path, const char *data)
{
    char tmp[128];
    snprintf(tmp, sizeof tmp, "%s.tmp", path);
    FILE *f = fopen(tmp, "w");
    if (!f)
        return;
    fputs(data, f);
    fclose(f);
    rename(tmp, path);
}

static int xioctl(int fd, unsigned long req, void *arg)
{
    int r;
    do
        r = ioctl(fd, req, arg);
    while (r < 0 && errno == EINTR);
    return r;
}

/* ------------------------------------------------------------------ pixel conversion */

/* YUYV -> UYVY: swap each byte pair. */
static void yuyv_to_uyvy(const uint8_t *s, uint8_t *d, size_t bytes)
{
    size_t i = 0;
#if defined(__ARM_NEON) || defined(__ARM_NEON__)
    for (; i + 16 <= bytes; i += 16)
        vst1q_u8(d + i, vrev16q_u8(vld1q_u8(s + i)));
#endif
    for (; i + 2 <= bytes; i += 2) {
        d[i] = s[i + 1];
        d[i + 1] = s[i];
    }
}

/* Planar 4:2:2 (Y, U, V rows) -> UYVY row. */
static void yuv422p_to_uyvy(const uint8_t *y, const uint8_t *u, const uint8_t *v, uint8_t *d, int w)
{
    int x = 0;
#if defined(__ARM_NEON) || defined(__ARM_NEON__)
    for (; x + 32 <= w; x += 32) {
        uint8x16x2_t yy = vld2q_u8(y + x); /* even, odd luma */
        uint8x16x4_t o = { { vld1q_u8(u + x / 2), yy.val[0], vld1q_u8(v + x / 2), yy.val[1] } };
        vst4q_u8(d + x * 2, o);
    }
#endif
    for (; x + 2 <= w; x += 2) {
        d[x * 2] = u[x / 2];
        d[x * 2 + 1] = y[x];
        d[x * 2 + 2] = v[x / 2];
        d[x * 2 + 3] = y[x + 1];
    }
}

/* YUYV / UYVY -> planar 4:2:2. */
static void packed_to_422p(const uint8_t *s, uint8_t *y, uint8_t *u, uint8_t *v, int w, bool uyvy)
{
    int x = 0;
#if defined(__ARM_NEON) || defined(__ARM_NEON__)
    for (; x + 32 <= w; x += 32) {
        uint8x16x4_t p = vld4q_u8(s + x * 2); /* YUYV: Y0 U Y1 V; UYVY: U Y0 V Y1 */
        uint8x16x2_t yy = uyvy ? (uint8x16x2_t){ { p.val[1], p.val[3] } } : (uint8x16x2_t){ { p.val[0], p.val[2] } };
        vst2q_u8(y + x, yy);
        vst1q_u8(u + x / 2, uyvy ? p.val[0] : p.val[1]);
        vst1q_u8(v + x / 2, uyvy ? p.val[2] : p.val[3]);
    }
#endif
    for (; x + 2 <= w; x += 2) {
        const uint8_t *p = s + x * 2;
        y[x] = p[uyvy ? 1 : 0];
        y[x + 1] = p[uyvy ? 3 : 2];
        u[x / 2] = p[uyvy ? 0 : 1];
        v[x / 2] = p[uyvy ? 2 : 3];
    }
}

/* JPEG (capture cards, webcams) is full range, 0-255; NDI, the H.264 encoder and the display expect
 * video range: black 16, white 235, colour 16-240. Without this, everything below 16 is shown black
 * (in a dark scene from a capture card that was over half the picture). y' = 16 + y * 219/255,
 * c' = 128 + (c - 128) * 224/255, in place, in 8.8 fixed point. */
static void full_to_video(uint8_t *p, size_t n, bool chroma)
{
    const int mul = chroma ? 225 : 220, add = chroma ? 128 * 256 - 128 * 225 + 128 : 16 * 256 + 128;
    size_t i = 0;
#if defined(__ARM_NEON) || defined(__ARM_NEON__)
    const uint8x8_t vm = vdup_n_u8((uint8_t)mul);
    const uint16x8_t va = vdupq_n_u16((uint16_t)add);
    for (; i + 16 <= n; i += 16) {
        uint8x16_t v = vld1q_u8(p + i);
        uint16x8_t lo = vmlal_u8(va, vget_low_u8(v), vm), hi = vmlal_u8(va, vget_high_u8(v), vm);
        vst1q_u8(p + i, vcombine_u8(vshrn_n_u16(lo, 8), vshrn_n_u16(hi, 8)));
    }
#endif
    for (; i < n; i++)
        p[i] = (uint8_t)((p[i] * mul + add) >> 8);
}

/* ------------------------------------------------------------------ shared memory for pindi-player */

/* Keep in sync with pindi-player.c. Frames: LOCAL_SLOTS slots of slot_size bytes from LOCAL_DATA,
 * planar Y, U, V. A slot's slot_seq is 0 while it is being written. */
#define LOCAL_MAGIC 0x4c444e50 /* "PNDL" */
#define LOCAL_SLOTS 4
#define LOCAL_AUDIO 32768      /* audio ring: frames of 48 kHz stereo */
#define LOCAL_DATA 262144
enum { LOCAL_422P = 1, LOCAL_420P = 2 };

struct local_feed {
    uint32_t magic, version;
    char name[128];
    int32_t w, h, fps_n, fps_d;
    uint32_t fmt, slot_size;
    uint64_t seq; /* newest frame, 0 = none yet */
    uint64_t slot_seq[LOCAL_SLOTS];
    int64_t slot_time[LOCAL_SLOTS];  /* capture time, 100 ns since 1970 (as NDI timestamps) */
    uint64_t writer_ms, reader_ms;   /* CLOCK_MONOTONIC heartbeats */
    uint64_t audio_wr;               /* audio frames written so far */
    int16_t audio[LOCAL_AUDIO * 2];
};

#define LOAD(p) __atomic_load_n(&(p), __ATOMIC_ACQUIRE)
#define STORE(p, v) __atomic_store_n(&(p), (v), __ATOMIC_RELEASE)

/* Somebody (pindi-player) has read the feed within the last 1.5 s. */
static bool local_wanted(struct local_feed *lf)
{
    if (!lf)
        return false;
    uint64_t r = LOAD(lf->reader_ms);
    return r && now_ms() - r < 1500;
}

/* ------------------------------------------------------------------ capture modes */

struct mode { uint32_t fourcc; int w, h; double fps; uint32_t num, den; int bpl; /* bytes per raw line */ };

static bool fourcc_ok(uint32_t f)
{
    return f == V4L2_PIX_FMT_MJPEG || f == V4L2_PIX_FMT_YUYV || f == V4L2_PIX_FMT_UYVY;
}

/* Enumerate the device's modes and pick the configured one, else the best up to 1080p30. */
static bool choose_mode(int fd, const char *want, struct mode *out)
{
    char wf[8] = "";
    int ww = 0, wh = 0;
    double wfps = 0;
    if (want && *want && sscanf(want, "%4s %dx%d@%lf", wf, &ww, &wh, &wfps) < 3)
        wf[0] = 0;

    bool found = false;
    struct mode best = { 0 };
    double best_score = -1;
    struct v4l2_fmtdesc fd_desc = { .type = V4L2_BUF_TYPE_VIDEO_CAPTURE };
    for (fd_desc.index = 0; !xioctl(fd, VIDIOC_ENUM_FMT, &fd_desc); fd_desc.index++) {
        uint32_t f = fd_desc.pixelformat;
        if (!fourcc_ok(f))
            continue;
        struct v4l2_frmsizeenum fs = { .pixel_format = f };
        for (fs.index = 0; !xioctl(fd, VIDIOC_ENUM_FRAMESIZES, &fs); fs.index++) {
            if (fs.type != V4L2_FRMSIZE_TYPE_DISCRETE)
                break;
            int w = fs.discrete.width, h = fs.discrete.height;
            struct v4l2_frmivalenum fi = { .pixel_format = f, .width = w, .height = h };
            for (fi.index = 0; !xioctl(fd, VIDIOC_ENUM_FRAMEINTERVALS, &fi); fi.index++) {
                if (fi.type != V4L2_FRMIVAL_TYPE_DISCRETE || !fi.discrete.numerator)
                    break;
                double fps = (double)fi.discrete.denominator / fi.discrete.numerator;
                struct mode m = { f, w, h, fps, fi.discrete.numerator, fi.discrete.denominator, 0 };
                if (wf[0]) {
                    char fc[5];
                    memcpy(fc, &f, 4);
                    fc[4] = 0;
                    if (!strcasecmp(fc, wf) && w == ww && h == wh && (wfps <= 0 || fabs(fps - wfps) < 0.5)) {
                        *out = m;
                        return true;
                    }
                    continue;
                }
                if (w > 1920 || h > 1080)
                    continue;
                /* Prefer >= 24 fps, then resolution, then MJPEG (USB 2 can't carry raw 1080p),
                 * then frame rate up to 30. */
                double score = (fps >= 23.9 ? 1e9 : 0) + (double)w * h * 10 + (f == V4L2_PIX_FMT_MJPEG ? 5 : 0) +
                               (fps > 30.5 ? 0 : fps / 100);
                if (score > best_score) {
                    best_score = score;
                    best = m;
                    found = true;
                }
            }
        }
    }
    if (wf[0])
        LOG("capture: configured mode \"%s\" not offered by the device, using the default", want);
    if (wf[0])
        return choose_mode(fd, NULL, out);
    *out = best;
    return found;
}

/* ------------------------------------------------------------------ CSI-2 HDMI bridges (TC358743) */

/* An HDMI-to-CSI-2 bridge on a camera connector has no capture modes to choose from: it delivers
 * whatever the HDMI source sends (raw UYVY here). So it is set up from the source: an EDID that
 * offers only modes its CSI-2 lanes can carry, the source's timings, then the same format on every
 * link from the bridge to the receiver's video node (rp1-cfe on the Pi 5, unicam on the Pi 3/4).
 * Without a media controller (unicam's default) the video node itself takes the bridge's ioctls. */

/* PiNDI's EDIDs (display "PiNDI", checked with edid-decode), with stereo LPCM audio. 4 lanes:
 * 1080p60 (preferred), 1080p50/30/25/24, 720p60/50, SD. 2 lanes: 1080p30 (preferred), 1080p25/24,
 * 720p60/50, SD - nothing over 74.25 MHz, which needs a third lane. */
static const uint8_t edid_4lane[256] = {
    0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x41, 0xc4, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x24, 0x01, 0x03, 0x80, 0x30, 0x1b, 0x78, 0x0e, 0xee, 0x91, 0xa3, 0x54, 0x4c, 0x99, 0x26,
    0x0f, 0x50, 0x54, 0x21, 0x08, 0x00, 0x81, 0xc0, 0xd1, 0xc0, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
    0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x02, 0x3a, 0x80, 0x18, 0x71, 0x38, 0x2d, 0x40, 0x58, 0x2c,
    0x45, 0x00, 0xe0, 0x0e, 0x11, 0x00, 0x00, 0x1e, 0x00, 0x00, 0x00, 0xfd, 0x00, 0x18, 0x3d, 0x0f,
    0x44, 0x0f, 0x00, 0x0a, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x00, 0x00, 0x00, 0xfc, 0x00, 0x50,
    0x69, 0x4e, 0x44, 0x49, 0x0a, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x00, 0x00, 0x00, 0x10,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x73,
    0x02, 0x03, 0x20, 0xc1, 0x4a, 0x10, 0x1f, 0x22, 0x21, 0x20, 0x04, 0x13, 0x01, 0x02, 0x11, 0x23,
    0x09, 0x07, 0x07, 0x83, 0x01, 0x00, 0x00, 0x65, 0x03, 0x0c, 0x00, 0x10, 0x00, 0xe2, 0x00, 0x4a,
    [255] = 0xa5,
};
static const uint8_t edid_2lane[256] = {
    0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x41, 0xc4, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x24, 0x01, 0x03, 0x80, 0x30, 0x1b, 0x78, 0x0e, 0xee, 0x91, 0xa3, 0x54, 0x4c, 0x99, 0x26,
    0x0f, 0x50, 0x54, 0x21, 0x08, 0x00, 0x81, 0xc0, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
    0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x1d, 0x80, 0x18, 0x71, 0x38, 0x2d, 0x40, 0x58, 0x2c,
    0x45, 0x00, 0xe0, 0x0e, 0x11, 0x00, 0x00, 0x1e, 0x00, 0x00, 0x00, 0xfd, 0x00, 0x18, 0x3d, 0x0f,
    0x44, 0x08, 0x00, 0x0a, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x00, 0x00, 0x00, 0xfc, 0x00, 0x50,
    0x69, 0x4e, 0x44, 0x49, 0x0a, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x00, 0x00, 0x00, 0x10,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x26,
    0x02, 0x03, 0x1e, 0xc1, 0x48, 0x22, 0x21, 0x20, 0x04, 0x13, 0x01, 0x02, 0x11, 0x23, 0x09, 0x07,
    0x07, 0x83, 0x01, 0x00, 0x00, 0x65, 0x03, 0x0c, 0x00, 0x10, 0x00, 0xe2, 0x00, 0x4a,
    [255] = 0xd8,
};

#define CSI_ENTS 32
#define CSI_LINKS 128

struct csi {
    int sd;           /* the bridge's subdev node, or the video node (no media controller); -1 = none */
    bool own_sd;      /* sd was opened here */
    int lanes;        /* CSI-2 data lanes the board is wired for (device tree) */
    uint64_t link_hz; /* CSI-2 link frequency (device tree) */
    char why[192];    /* why there is no picture */
};

struct csi_graph {
    int nent, nlink;
    struct media_entity_desc ent[CSI_ENTS];
    struct media_link_desc link[CSI_LINKS];
};

/* A camera connector's receiver (rp1-cfe, unicam): its bus info ("platform:1f00110000.csi") in bus. */
static bool csi_receiver(int fd, char *bus, size_t n)
{
    struct v4l2_capability cap = { 0 };
    if (xioctl(fd, VIDIOC_QUERYCAP, &cap))
        return false;
    snprintf(bus, n, "%s", (const char *)cap.bus_info);
    return !strcmp((const char *)cap.driver, "rp1-cfe") || !strcmp((const char *)cap.driver, "unicam");
}

/* A device-tree property of big-endian cells: how many, the first n in v. */
static int dt_cells(const char *path, uint32_t *v, int n)
{
    uint8_t b[64];
    FILE *f = fopen(path, "rb");
    if (!f)
        return 0;
    size_t len = fread(b, 1, sizeof b, f);
    fclose(f);
    for (int i = 0; i < n && (size_t)i * 4 + 4 <= len; i++)
        v[i] = (uint32_t)b[i * 4] << 24 | (uint32_t)b[i * 4 + 1] << 16 | (uint32_t)b[i * 4 + 2] << 8 | b[i * 4 + 3];
    return (int)(len / 4);
}

/* Lanes and link frequency from the bridge's device-tree endpoint (dir: its of_node). */
static void csi_dt(struct csi *c, const char *dir)
{
    char p[PATH_MAX + 64];
    uint32_t v[2] = { 0 };
    snprintf(p, sizeof p, "%s/port/endpoint/data-lanes", dir);
    int lanes = dt_cells(p, v, 1);
    snprintf(p, sizeof p, "%s/port/endpoint/link-frequencies", dir);
    if (dt_cells(p, v, 2) >= 2)
        c->link_hz = (uint64_t)v[0] << 32 | v[1];
    if (lanes > 0)
        c->lanes = lanes;
}

/* /dev node of a character device number, opened. */
static int open_devnum(uint32_t major, uint32_t minor)
{
    char p[96], line[128], node[160] = "";
    snprintf(p, sizeof p, "/sys/dev/char/%u:%u/uevent", major, minor);
    FILE *f = fopen(p, "r");
    if (!f)
        return -1;
    while (fgets(line, sizeof line, f))
        if (!strncmp(line, "DEVNAME=", 8)) {
            line[strcspn(line, "\n")] = 0;
            snprintf(node, sizeof node, "/dev/%s", line + 8);
        }
    fclose(f);
    return *node ? open(node, O_RDWR | O_NONBLOCK | O_CLOEXEC) : -1;
}

/* The media device of the receiver with this bus info, with all its entities and links (each link
 * once: the media API lists them from their source side). */
static int csi_graph_load(const char *bus, struct csi_graph *g)
{
    for (int i = 0; i < 16; i++) {
        char p[32];
        snprintf(p, sizeof p, "/dev/media%d", i);
        int mfd = open(p, O_RDWR | O_CLOEXEC);
        if (mfd < 0)
            continue;
        struct media_device_info info = { 0 };
        if (xioctl(mfd, MEDIA_IOC_DEVICE_INFO, &info) || strcmp(info.bus_info, bus)) {
            close(mfd);
            continue;
        }
        g->nent = g->nlink = 0;
        uint32_t id = 0;
        for (;;) {
            struct media_entity_desc e = { .id = id | MEDIA_ENT_ID_FLAG_NEXT };
            if (g->nent >= CSI_ENTS || xioctl(mfd, MEDIA_IOC_ENUM_ENTITIES, &e))
                break;
            id = e.id;
            g->ent[g->nent++] = e;
            struct media_pad_desc pads[16];
            struct media_link_desc links[16];
            struct media_links_enum le = { .entity = e.id, .pads = pads, .links = links };
            if (e.pads > 16 || e.links > 16 || xioctl(mfd, MEDIA_IOC_ENUM_LINKS, &le))
                continue;
            for (int k = 0; k < e.links && g->nlink < CSI_LINKS; k++)
                g->link[g->nlink++] = links[k];
        }
        return mfd;
    }
    return -1;
}

static const struct media_entity_desc *csi_ent(const struct csi_graph *g, uint32_t id)
{
    for (int i = 0; i < g->nent; i++)
        if (g->ent[i].id == id)
            return &g->ent[i];
    return NULL;
}

/* Entity from feeds entity to (through links, enabled or not). */
static bool csi_feeds(const struct csi_graph *g, uint32_t from, uint32_t to, int depth)
{
    if (from == to)
        return true;
    for (int k = 0; depth < 8 && k < g->nlink; k++)
        if (g->link[k].sink.entity == to && csi_feeds(g, from, g->link[k].source.entity, depth + 1))
            return true;
    return false;
}

static int subdev_fmt(int fd, uint32_t pad, int w, int h)
{
    struct v4l2_subdev_format f = { .which = V4L2_SUBDEV_FORMAT_ACTIVE, .pad = pad };
    f.format.width = (uint32_t)w;
    f.format.height = (uint32_t)h;
    f.format.code = MEDIA_BUS_FMT_UYVY8_1X16;
    f.format.field = V4L2_FIELD_NONE;
    f.format.colorspace = V4L2_COLORSPACE_SRGB;
    return xioctl(fd, VIDIOC_SUBDEV_S_FMT, &f);
}

/* From the video node back to the bridge: enable each link on the way (the Pi 5's CSI-2 subdev also
 * feeds its ISP front end, which PiNDI doesn't use) and give both pads of every hop the format. */
static const char *csi_route(int mfd, const struct csi_graph *g, uint32_t video, uint32_t bridge, int w, int h)
{
    uint32_t ent = video;
    for (int hop = 0; hop < 8 && ent != bridge; hop++) {
        const struct media_link_desc *l = NULL;
        for (int k = 0; k < g->nlink && !l; k++)
            if (g->link[k].sink.entity == ent && csi_feeds(g, bridge, g->link[k].source.entity, 0))
                l = &g->link[k];
        if (!l)
            return "the camera connector's video node has no path to the HDMI bridge";
        if (!(l->flags & MEDIA_LNK_FL_ENABLED)) {
            struct media_link_desc s = *l;
            s.flags |= MEDIA_LNK_FL_ENABLED;
            if (xioctl(mfd, MEDIA_IOC_SETUP_LINK, &s))
                return "cannot connect the HDMI bridge to the camera connector's video node";
        }
        const struct media_entity_desc *src = csi_ent(g, l->source.entity), *snk = csi_ent(g, l->sink.entity);
        int sfd = src && src->dev.major ? open_devnum(src->dev.major, src->dev.minor) : -1;
        if (sfd >= 0) {
            subdev_fmt(sfd, l->source.index, w, h);
            close(sfd);
        }
        if (ent != video && snk && snk->dev.major && (sfd = open_devnum(snk->dev.major, snk->dev.minor)) >= 0) {
            subdev_fmt(sfd, l->sink.index, w, h);
            close(sfd);
        }
        ent = l->source.entity;
    }
    return ent == bridge ? NULL : "the camera connector's video node has no path to the HDMI bridge";
}

/* The EDID for the lanes, unless the bridge has it already (a new one makes the source look again). */
static void csi_edid(int n, const struct csi *c)
{
    const uint8_t *want = c->lanes >= 3 ? edid_4lane : edid_2lane;
    uint8_t have[256];
    struct v4l2_edid e = { .blocks = 2, .edid = have };
    if (!xioctl(c->sd, VIDIOC_G_EDID, &e) && e.blocks == 2 && !memcmp(have, want, sizeof have))
        return;
    memcpy(have, want, sizeof have); /* S_EDID writes the array back: it can't be the read-only one */
    struct v4l2_edid s = { .blocks = 2, .edid = have };
    if (xioctl(c->sd, VIDIOC_S_EDID, &s))
        LOG("input %d: cannot give the HDMI bridge its EDID: %s", n, strerror(errno));
    else
        LOG("input %d: HDMI bridge: %d CSI-2 lanes, the source is offered up to %s", n, c->lanes,
            c->lanes >= 3 ? "1080p60" : "1080p30");
}

/* Set up the bridge behind the receiver's video node fd from the HDMI source, and the video node
 * for its picture (UYVY) in m. NULL when it is ready, else why there is no picture (c->why). */
static const char *csi_setup(int n, int fd, const char *bus, struct csi *c, struct mode *m)
{
    struct csi_graph *g = calloc(1, sizeof *g);
    int mfd = g ? csi_graph_load(bus, g) : -1;
    const struct media_entity_desc *bridge = NULL, *video = NULL;
    struct stat st;
    bool have_st = !fstat(fd, &st);
    for (int i = 0; mfd >= 0 && i < g->nent; i++) {
        if (!strncmp(g->ent[i].name, "tc358743", 8))
            bridge = &g->ent[i];
        if (have_st && g->ent[i].dev.major == major(st.st_rdev) && g->ent[i].dev.minor == minor(st.st_rdev))
            video = &g->ent[i];
    }
    const char *why = NULL;
    c->lanes = 2;
    c->link_hz = 486000000; /* the overlays' default: 972 Mbit/s a lane */
    if (!bridge) {
        why = "no HDMI bridge on this camera connector (is it set up under Camera connectors?)";
        goto done;
    }
    char dir[PATH_MAX];
    if (bridge->dev.major) { /* media controller: the bridge has its own node */
        c->sd = open_devnum(bridge->dev.major, bridge->dev.minor);
        c->own_sd = c->sd >= 0;
        snprintf(dir, sizeof dir, "/sys/dev/char/%u:%u/device/of_node", bridge->dev.major, bridge->dev.minor);
    } else { /* the receiver passes everything on to it */
        c->sd = fd;
        snprintf(dir, sizeof dir, "/sys/bus/i2c/drivers/tc358743/%s/of_node", strchr(bridge->name, ' ') ?
                 strchr(bridge->name, ' ') + 1 : "");
    }
    if (c->sd < 0) {
        why = "cannot open the HDMI bridge";
        goto done;
    }
    csi_dt(c, dir);
    struct v4l2_event_subscription sub = { .type = V4L2_EVENT_SOURCE_CHANGE };
    xioctl(c->sd, VIDIOC_SUBSCRIBE_EVENT, &sub); /* before the timings are read: no change is missed */
    csi_edid(n, c);

    struct v4l2_dv_timings t = { 0 };
    if (xioctl(c->sd, VIDIOC_QUERY_DV_TIMINGS, &t)) {
        why = errno == ENOLINK ? "no HDMI signal" : errno == ENOLCK ? "HDMI signal not stable yet" :
              errno == ERANGE ? "the HDMI source's mode is not supported" : "cannot read the HDMI signal";
        goto done;
    }
    const struct v4l2_bt_timings *bt = &t.bt;
    uint64_t htot = V4L2_DV_BT_FRAME_WIDTH(bt), vtot = V4L2_DV_BT_FRAME_HEIGHT(bt);
    double fps = htot && vtot ? (double)bt->pixelclock / ((double)htot * vtot) : 0;
    if (bt->interlaced) {
        snprintf(c->why, sizeof c->why, "the HDMI source sends %ux%ui, interlaced: set it to a progressive mode",
                 bt->width, bt->height);
        why = c->why;
        goto done;
    }
    /* As the TC358743 driver counts: active pixels, 16 bits each (UYVY), per lane twice the link frequency. */
    uint64_t lane_bps = c->link_hz * 2;
    int need = (int)(((uint64_t)((double)bt->width * bt->height * fps * 16) + lane_bps - 1) / lane_bps);
    if (need > c->lanes) {
        snprintf(c->why, sizeof c->why, "%ux%up%.4g from the HDMI source needs %d CSI-2 lanes, this connector has "
                 "%d: set the source to 1080p30 or lower", bt->width, bt->height, fps, need, c->lanes);
        why = c->why;
        goto done;
    }
    if (xioctl(c->sd, VIDIOC_S_DV_TIMINGS, &t)) {
        why = "the HDMI bridge refused the source's timings";
        goto done;
    }
    int w = (int)bt->width, h = (int)bt->height;
    if (c->own_sd) {
        if (subdev_fmt(c->sd, 0, w, h)) {
            why = "the HDMI bridge cannot send UYVY";
            goto done;
        }
        if (video && (why = csi_route(mfd, g, video->id, bridge->id, w, h)))
            goto done;
    }
    struct v4l2_format f = { .type = V4L2_BUF_TYPE_VIDEO_CAPTURE };
    f.fmt.pix.width = (uint32_t)w;
    f.fmt.pix.height = (uint32_t)h;
    f.fmt.pix.pixelformat = V4L2_PIX_FMT_UYVY;
    f.fmt.pix.field = V4L2_FIELD_NONE;
    if (xioctl(fd, VIDIOC_S_FMT, &f) || f.fmt.pix.pixelformat != V4L2_PIX_FMT_UYVY || (int)f.fmt.pix.width != w ||
        (int)f.fmt.pix.height != h) {
        why = "the camera connector cannot capture the HDMI picture";
        goto done;
    }
    /* 59.94 and the like as 1001ths, the rest in thousandths. */
    double k = fps * 1.001;
    bool ntsc = fabs(k - lround(k)) < 0.005 && fabs(fps - lround(fps)) > 0.01;
    *m = (struct mode){ .fourcc = V4L2_PIX_FMT_UYVY, .w = w, .h = h, .fps = fps, .num = ntsc ? 1001 : 1000,
                        .den = (uint32_t)(ntsc ? lround(k) * 1000 : lround(fps * 1000)),
                        .bpl = (int)f.fmt.pix.bytesperline };
done:
    if (mfd >= 0)
        close(mfd);
    free(g);
    return why;
}

/* A source change (other mode, unplugged) was signalled: the bridge must be set up again. */
static bool csi_changed(struct csi *c)
{
    bool changed = false;
    struct v4l2_event ev;
    while (!xioctl(c->sd, VIDIOC_DQEVENT, &ev))
        changed |= ev.type == V4L2_EVENT_SOURCE_CHANGE;
    return changed;
}

static void csi_close(struct csi *c, int fd)
{
    if (c->sd >= 0 && c->sd != fd) {
        struct v4l2_event_subscription sub = { .type = V4L2_EVENT_ALL };
        xioctl(c->sd, VIDIOC_UNSUBSCRIBE_EVENT, &sub);
        if (c->own_sd)
            close(c->sd);
    }
    c->sd = -1;
}

/* ------------------------------------------------------------------ H.264 over RTP */

/* With PINDI_IN<n>_CODEC=rtp the hardware encoder's H.264 is sent as RTP (RFC 6184) to
 * PINDI_IN<n>_RTP_DEST (ip:port, unicast or multicast) instead of over NDI: no NDI Advanced SDK
 * licence, and any RTP receiver shows it (another PiNDI, or VLC, ffmpeg, OBS... with the SDP file
 * /run/pindi/in<n>.sdp). The audio goes to port + 2 as 48 kHz stereo L16; RTCP sender reports on
 * port + 1 and + 3 tie both to the wall clock, for lip sync and the receiver's latency figure.
 * The video is sent from the encoder thread, the audio from the audio thread: each stream has
 * one writer. */
#define RTP_PAYLOAD 1400       /* bytes per packet: under the Ethernet MTU, with room for VLAN/VPN headers */
#define RTP_PT_VIDEO 96
#define RTP_PT_AUDIO 97
#define RTP_AUDIO_FRAMES 240   /* 5 ms of stereo L16 per packet (960 bytes) */
#define RTP_TTL 16             /* multicast hops (ffmpeg's default) */

struct rtp_stream {
    uint16_t seq;
    uint32_t ssrc, ts_base;
    uint32_t packets, octets; /* for the sender reports */
    uint64_t sr_ms;           /* last sender report (monotonic) */
    uint32_t last_ts;         /* audio: the timestamp after the last packet, and when it was sent */
    int64_t last_100ns;
};

struct rtp_out {
    int fd;
    struct sockaddr_in dst;
    char dest[64];            /* "ip:port", as configured */
    bool multicast;
    struct rtp_stream v, a;
    uint64_t samples;         /* audio frames sent */
    uint8_t sps[128], pps[64];
    size_t sps_len, pps_len;
    char sdp_path[64], name[128];
    bool audio;               /* the SDP announces audio */
    double fps;
};

/* Wall clock, 100 ns since 1970 (as NDI timestamps and capture_time()). */
static int64_t now_100ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    return (int64_t)ts.tv_sec * 10000000 + ts.tv_nsec / 100;
}

static bool rtp_parse_dest(const char *s, struct sockaddr_in *a)
{
    char ip[32];
    const char *colon = strrchr(s, ':');
    char *end;
    if (!colon || colon - s >= (long)sizeof ip)
        return false;
    memcpy(ip, s, (size_t)(colon - s));
    ip[colon - s] = 0;
    long port = strtol(colon + 1, &end, 10);
    memset(a, 0, sizeof *a);
    a->sin_family = AF_INET;
    a->sin_port = htons((uint16_t)port);
    return !*end && port >= 1024 && port <= 65532 && inet_pton(AF_INET, ip, &a->sin_addr) == 1;
}

static bool rtp_open(struct rtp_out *r, int n, const char *dest, const char *name, bool audio, double fps)
{
    memset(r, 0, sizeof *r);
    r->fd = -1;
    snprintf(r->dest, sizeof r->dest, "%s", dest);
    snprintf(r->name, sizeof r->name, "%s", name);
    snprintf(r->sdp_path, sizeof r->sdp_path, RUN_DIR "/in%d.sdp", n);
    r->audio = audio;
    r->fps = fps;
    if (!rtp_parse_dest(dest, &r->dst)) {
        LOG("input %d: H.264 RTP: \"%s\" is not an IPv4 address and port (1024-65532)", n, dest);
        return false;
    }
    r->multicast = (ntohl(r->dst.sin_addr.s_addr) >> 28) == 14; /* 224.0.0.0/4 */
    r->fd = socket(AF_INET, SOCK_DGRAM | SOCK_CLOEXEC, 0);
    if (r->fd < 0)
        return false;
    int ttl = RTP_TTL, tos = 0x88, sndbuf = 1 << 20; /* DSCP AF41: interactive video */
    if (r->multicast)
        setsockopt(r->fd, IPPROTO_IP, IP_MULTICAST_TTL, &ttl, sizeof ttl);
    setsockopt(r->fd, IPPROTO_IP, IP_TOS, &tos, sizeof tos);
    setsockopt(r->fd, SOL_SOCKET, SO_SNDBUF, &sndbuf, sizeof sndbuf);
    uint32_t rnd[6];
    if (getrandom(rnd, sizeof rnd, 0) != (ssize_t)sizeof rnd)
        for (int i = 0; i < 6; i++)
            rnd[i] = (uint32_t)now_us() * 2654435761u + (uint32_t)i * 40503u;
    r->v.ssrc = rnd[0];
    r->a.ssrc = rnd[1] == rnd[0] ? rnd[1] + 1 : rnd[1];
    r->v.ts_base = rnd[2];
    r->a.ts_base = rnd[3];
    r->v.seq = (uint16_t)rnd[4];
    r->a.seq = (uint16_t)rnd[5];
    unlink(r->sdp_path); /* written with the first keyframe's parameter sets */
    LOG("input %d: H.264 RTP to %s%s, audio to port %d", n, dest, r->multicast ? " (multicast)" : "",
        ntohs(r->dst.sin_port) + 2);
    return true;
}

static void rtp_close(struct rtp_out *r)
{
    if (r->fd >= 0)
        close(r->fd);
    r->fd = -1;
}

/* Datagram to the destination's port + port_add. */
static void rtp_sendv(struct rtp_out *r, int port_add, struct iovec *iov, int niov)
{
    struct sockaddr_in to = r->dst;
    to.sin_port = htons((uint16_t)(ntohs(r->dst.sin_port) + port_add));
    struct msghdr m = { .msg_name = &to, .msg_namelen = sizeof to, .msg_iov = iov, .msg_iovlen = (size_t)niov };
    sendmsg(r->fd, &m, MSG_NOSIGNAL); /* UDP: a lost packet is the receiver's to cope with */
}

static void put32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v >> 24);
    p[1] = (uint8_t)(v >> 16);
    p[2] = (uint8_t)(v >> 8);
    p[3] = (uint8_t)v;
}

/* One RTP packet: 12-byte header, then up to two pieces of payload. */
static void rtp_packet(struct rtp_out *r, struct rtp_stream *s, int port_add, int pt, bool marker, uint32_t ts,
                       const void *p1, size_t n1, const void *p2, size_t n2)
{
    uint8_t h[12];
    h[0] = 0x80; /* version 2 */
    h[1] = (uint8_t)(pt | (marker ? 0x80 : 0));
    h[2] = (uint8_t)(s->seq >> 8);
    h[3] = (uint8_t)s->seq;
    put32(h + 4, ts);
    put32(h + 8, s->ssrc);
    struct iovec iov[3] = { { h, sizeof h }, { (void *)p1, n1 }, { (void *)p2, n2 } };
    rtp_sendv(r, port_add, iov, n2 ? 3 : 2);
    s->seq++;
    s->packets++;
    s->octets += (uint32_t)(n1 + n2);
}

/* RTCP sender report (RFC 3550): this stream's timestamp at wall-clock time t. */
static void rtcp_sr(struct rtp_out *r, struct rtp_stream *s, int port_add, int64_t t, uint32_t ts)
{
    uint8_t p[28];
    uint64_t secs = (uint64_t)(t / 10000000) + 2208988800u; /* NTP counts from 1900 */
    uint32_t frac = (uint32_t)(((uint64_t)(t % 10000000) << 32) / 10000000);
    p[0] = 0x80;
    p[1] = 200; /* SR */
    p[2] = 0;
    p[3] = 6; /* length in 32-bit words, minus one */
    put32(p + 4, s->ssrc);
    put32(p + 8, (uint32_t)secs);
    put32(p + 12, frac);
    put32(p + 16, ts);
    put32(p + 20, s->packets);
    put32(p + 24, s->octets);
    struct iovec iov = { p, sizeof p };
    rtp_sendv(r, port_add, &iov, 1);
    s->sr_ms = now_ms();
}

static size_t b64(const uint8_t *s, size_t n, char *d)
{
    static const char t[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    size_t o = 0;
    for (size_t i = 0; i < n; i += 3) {
        uint32_t v = (uint32_t)s[i] << 16 | (i + 1 < n ? (uint32_t)s[i + 1] << 8 : 0) | (i + 2 < n ? s[i + 2] : 0);
        d[o++] = t[v >> 18 & 63];
        d[o++] = t[v >> 12 & 63];
        d[o++] = i + 1 < n ? t[v >> 6 & 63] : '=';
        d[o++] = i + 2 < n ? t[v & 63] : '=';
    }
    d[o] = 0;
    return o;
}

/* The session description for RTP receivers (VLC, ffmpeg, OBS...): written when the parameter sets
 * are first seen or change (another capture mode). */
static void rtp_write_sdp(struct rtp_out *r)
{
    char src[INET_ADDRSTRLEN] = "0.0.0.0", dst[INET_ADDRSTRLEN], sps[180], pps[100], buf[1536];
    int t = socket(AF_INET, SOCK_DGRAM | SOCK_CLOEXEC, 0); /* this device's address towards the receiver */
    struct sockaddr_in me;
    socklen_t ml = sizeof me;
    if (t >= 0 && !connect(t, (struct sockaddr *)&r->dst, sizeof r->dst) &&
        !getsockname(t, (struct sockaddr *)&me, &ml))
        inet_ntop(AF_INET, &me.sin_addr, src, sizeof src);
    if (t >= 0)
        close(t);
    inet_ntop(AF_INET, &r->dst.sin_addr, dst, sizeof dst);
    b64(r->sps, r->sps_len, sps);
    b64(r->pps, r->pps_len, pps);
    int port = ntohs(r->dst.sin_port), len;
    len = snprintf(buf, sizeof buf,
                   "v=0\r\no=- %u 1 IN IP4 %s\r\ns=%s\r\nc=IN IP4 %s%s\r\nt=0 0\r\na=tool:PiNDI\r\n"
                   "m=video %d RTP/AVP %d\r\na=rtpmap:%d H264/90000\r\n"
                   "a=fmtp:%d packetization-mode=1;profile-level-id=%02X%02X%02X;sprop-parameter-sets=%s,%s\r\n"
                   "a=framerate:%.4g\r\n",
                   r->v.ssrc, src, r->name, dst, r->multicast ? "/16" : "", port, RTP_PT_VIDEO, RTP_PT_VIDEO,
                   RTP_PT_VIDEO, r->sps[1], r->sps[2], r->sps[3], sps, pps, r->fps);
    if (r->audio && len > 0 && len < (int)sizeof buf)
        snprintf(buf + len, sizeof buf - (size_t)len,
                 "m=audio %d RTP/AVP %d\r\na=rtpmap:%d L16/48000/2\r\na=ptime:5\r\n", port + 2, RTP_PT_AUDIO,
                 RTP_PT_AUDIO);
    write_file_atomic(r->sdp_path, buf);
}

/* One NAL unit (no start code): as it is if it fits a packet, else in FU-A fragments. */
static void rtp_send_nal(struct rtp_out *r, const uint8_t *nal, size_t len, uint32_t ts, bool last)
{
    if (len <= RTP_PAYLOAD) {
        rtp_packet(r, &r->v, 0, RTP_PT_VIDEO, last, ts, nal, len, NULL, 0);
        return;
    }
    uint8_t fu[2] = { (uint8_t)((nal[0] & 0xe0) | 28), (uint8_t)(0x80 | (nal[0] & 0x1f)) }; /* FU indicator, start */
    size_t at = 1; /* the NAL header travels in the FU header */
    while (at < len) {
        size_t n = len - at < RTP_PAYLOAD - 2 ? len - at : RTP_PAYLOAD - 2;
        bool end = at + n == len;
        if (end)
            fu[1] |= 0x40;
        rtp_packet(r, &r->v, 0, RTP_PT_VIDEO, last && end, ts, fu, 2, nal + at, n);
        fu[1] &= 0x1f; /* neither start nor end */
        at += n;
    }
}

/* Next NAL unit in Annex B data from *at: its start (after the start code) and length; false at the end. */
static bool annexb_next(const uint8_t *d, size_t len, size_t *at, const uint8_t **nal, size_t *nlen)
{
    size_t i = *at;
    while (i + 3 <= len && !(d[i] == 0 && d[i + 1] == 0 && d[i + 2] == 1))
        i++;
    if (i + 3 > len)
        return false;
    size_t s = i + 3, e = s;
    while (e + 3 <= len && !(d[e] == 0 && d[e + 1] == 0 && d[e + 2] == 1))
        e++;
    if (e + 3 > len)
        e = len;
    *at = e;
    while (e > s && d[e - 1] == 0) /* a 4-byte start code's first zero, or trailing zeros */
        e--;
    *nal = d + s;
    *nlen = e - s;
    return true;
}

/* One encoded picture (Annex B), captured at t (100 ns UTC). */
static void rtp_send_video(struct rtp_out *r, const uint8_t *data, size_t len, int64_t t)
{
    uint32_t ts = r->v.ts_base + (uint32_t)(t * 9 / 1000); /* 90 kHz */
    const uint8_t *nal = NULL, *next = NULL;
    size_t at = 0, nlen = 0, nnext = 0;
    bool has = annexb_next(data, len, &at, &nal, &nlen);
    while (has) {
        bool more;
        do /* the next non-empty unit: is this one the picture's last? */
            more = annexb_next(data, len, &at, &next, &nnext);
        while (more && !nnext);
        if (nlen) {
            int type = nal[0] & 0x1f;
            bool changed = false;
            if (type == 7 && nlen <= sizeof r->sps && (nlen != r->sps_len || memcmp(r->sps, nal, nlen))) {
                memcpy(r->sps, nal, nlen);
                r->sps_len = nlen;
                changed = true;
            } else if (type == 8 && nlen <= sizeof r->pps && (nlen != r->pps_len || memcmp(r->pps, nal, nlen))) {
                memcpy(r->pps, nal, nlen);
                r->pps_len = nlen;
                changed = true;
            }
            if (changed && r->sps_len >= 4 && r->pps_len)
                rtp_write_sdp(r);
            if (type != 9) /* access unit delimiters are optional; the marker bit ends a picture */
                rtp_send_nal(r, nal, nlen, ts, !more);
        }
        nal = next;
        nlen = nnext;
        has = more;
    }
    if (now_ms() - r->v.sr_ms >= 1000) {
        int64_t now = now_100ns();
        rtcp_sr(r, &r->v, 1, now, r->v.ts_base + (uint32_t)(now * 9 / 1000));
    }
}

/* n frames of 48 kHz stereo audio, sent as they are captured. */
static void rtp_send_audio(struct rtp_out *r, const int16_t *s, int n)
{
    uint8_t be[RTP_AUDIO_FRAMES * 4];
    for (int at = 0; at < n; at += RTP_AUDIO_FRAMES) {
        int k = n - at < RTP_AUDIO_FRAMES ? n - at : RTP_AUDIO_FRAMES;
        for (int i = 0; i < k * 2; i++) { /* network byte order */
            uint16_t v = (uint16_t)s[at * 2 + i];
            be[i * 2] = (uint8_t)(v >> 8);
            be[i * 2 + 1] = (uint8_t)v;
        }
        uint32_t ts = r->a.ts_base + (uint32_t)r->samples;
        rtp_packet(r, &r->a, 2, RTP_PT_AUDIO, false, ts, be, (size_t)k * 4, NULL, 0);
        r->samples += (uint64_t)k;
    }
    r->a.last_ts = r->a.ts_base + (uint32_t)r->samples;
    r->a.last_100ns = now_100ns();
    if (now_ms() - r->a.sr_ms >= 1000) /* the samples just sent end about now */
        rtcp_sr(r, &r->a, 3, r->a.last_100ns, r->a.last_ts);
}

/* ------------------------------------------------------------------ H.264 over SRT */

/* With PINDI_IN<n>_CODEC=srt the hardware encoder's H.264 and the audio go out as an MPEG transport
 * stream over SRT (Secure Reliable Transport): lost packets are sent again within the latency, so it
 * also crosses Wi-Fi, routed networks and the internet, optionally encrypted (AES, with a passphrase).
 * Any SRT receiver plays it: another PiNDI, VLC, ffmpeg, OBS, hardware decoders. PINDI_IN<n>_SRT_URL:
 *   srt://:9000              listener: receivers call this device on port 9000 (up to SRT_MAX_PEERS)
 *   srt://192.168.1.60:9000  caller: this device calls that receiver, and calls again when it drops
 * with ?passphrase=...&latency=<ms>&streamid=... as ffmpeg and VLC write them. The audio is 48 kHz
 * stereo PCM as SMPTE 302M (uncompressed, as over NDI and RTP). Timestamps are the capture times (UTC,
 * 90 kHz), and the service provider "PiNDI" (SDT) tells a PiNDI receiver so: its latency figure. The
 * encoder runs only while someone receives, and a new receiver gets a keyframe at once. libsrt is
 * loaded at run time. The video is muxed by the encoder thread, the audio by the audio thread: each
 * has its own message buffer and continuity counters; sending to the receivers is shared (mu). */
#define SRT_MAX_PEERS 8
#define TS_MSG 7                /* TS packets per SRT message: 1316 bytes, the live mode's payload */
#define TS_PID_PMT 0x1000
#define TS_PID_SDT 0x11
#define TS_PID_VIDEO 0x100
#define TS_PID_AUDIO 0x101
#define TS_PCR_DELAY 13500      /* the PCR runs 150 ms behind the capture clock: room for capture and encoding */
#define TS_AUDIO_MAX 960        /* frames per audio PES (20 ms) */
#define SRT_EAGAIN_MAJOR 6      /* SRT_EASYNCSND, SRT_ECONGEST...: the send buffer is full, nothing broke */
enum { SRTS_CONNECTED = 5, SRTO_SNDSYN = 1, SRTO_LATENCY = 23, SRTO_PASSPHRASE = 26, SRTO_IPTOS = 30,
       SRTO_CONNTIMEO = 36, SRTO_STREAMID = 46, SRT_EPOLL_IN = 1, SRT_EPOLL_ERR = 8 };

struct srt_url {
    bool listen;
    char host[128];
    int port;
    char passphrase[80];
    int latency; /* ms, -1: libsrt's default (120) */
    char streamid[128];
};

static int hexval(int c)
{
    return c >= '0' && c <= '9' ? c - '0' : c >= 'a' && c <= 'f' ? c - 'a' + 10 : c >= 'A' && c <= 'F' ? c - 'A' + 10 : -1;
}

/* "srt://[host]:port[?key=value&...]": no host (or "@", "0.0.0.0") listens, a host is called; mode=listener
 * or caller says so explicitly. Values may be %-encoded. Other keys (ffmpeg has many) are ignored. */
static bool srt_parse(const char *s, struct srt_url *u, const char **err)
{
    memset(u, 0, sizeof *u);
    u->latency = -1;
    *err = "not an srt:// address";
    if (strncasecmp(s, "srt://", 6))
        return false;
    s += 6;
    if (*s == '@')
        s++;
    const char *q = strchr(s, '?');
    size_t hl = q ? (size_t)(q - s) : strlen(s);
    const char *colon = memrchr(s, ':', hl);
    *err = "no port (srt://host:port, or srt://:port to listen)";
    if (!colon || (size_t)(colon - s) >= sizeof u->host)
        return false;
    memcpy(u->host, s, (size_t)(colon - s));
    u->host[colon - s] = 0;
    char *end;
    long port = strtol(colon + 1, &end, 10);
    if (end != s + hl || port < 1 || port > 65535)
        return false;
    u->port = (int)port;
    u->listen = !*u->host || !strcmp(u->host, "0.0.0.0");
    for (const char *p = q ? q + 1 : NULL; p && *p;) {
        const char *amp = strchr(p, '&'), *eq = strchr(p, '=');
        size_t kl = (size_t)((amp ? amp : p + strlen(p)) - p);
        char key[24], val[160];
        size_t vn = 0;
        if (eq && eq < p + kl && (size_t)(eq - p) < sizeof key) {
            memcpy(key, p, (size_t)(eq - p));
            key[eq - p] = 0;
            for (const char *v = eq + 1; v < p + kl && vn + 1 < sizeof val; v++) {
                if (*v == '%' && v + 2 < p + kl + 1 && hexval(v[1]) >= 0 && hexval(v[2]) >= 0) {
                    val[vn++] = (char)(hexval(v[1]) * 16 + hexval(v[2]));
                    v += 2;
                } else {
                    val[vn++] = *v;
                }
            }
            val[vn] = 0;
            if (!strcasecmp(key, "passphrase")) {
                *err = "the passphrase must be 10 to 79 characters";
                if (vn < 10 || vn > 79)
                    return false;
                memcpy(u->passphrase, val, vn + 1);
            } else if (!strcasecmp(key, "latency")) {
                long l = strtol(val, &end, 10);
                *err = "the latency must be 20 to 8000 ms";
                if (*end || l < 20 || l > 8000)
                    return false;
                u->latency = (int)l;
            } else if (!strcasecmp(key, "streamid")) {
                snprintf(u->streamid, sizeof u->streamid, "%.127s", val);
            } else if (!strcasecmp(key, "mode")) {
                *err = "mode must be listener or caller";
                if (strcasecmp(val, "listener") && strcasecmp(val, "caller"))
                    return false;
                u->listen = !strcasecmp(val, "listener");
            }
        }
        p = amp ? amp + 1 : NULL;
    }
    *err = "a caller needs the receiver's address (srt://host:port)";
    if (!u->listen && !*u->host)
        return false;
    return true;
}

/* libsrt, loaded at run time: SRT is optional (install.sh installs it when it can). */
static struct {
    bool tried, ok;
    char name[32];
    int (*startup)(void);
    int (*create_socket)(void);
    int (*setsockflag)(int s, int opt, const void *val, int len);
    int (*bind)(int s, const struct sockaddr *a, int len);
    int (*listen)(int s, int backlog);
    int (*accept)(int s, struct sockaddr *a, int *len);
    int (*connect)(int s, const struct sockaddr *a, int len);
    int (*sendmsg2)(int s, const char *buf, int len, void *mctrl);
    int (*close)(int s);
    int (*getsockstate)(int s);
    const char *(*getlasterror_str)(void);
    int (*getlasterror)(int *errno_loc);
    int (*epoll_create)(void);
    int (*epoll_add_usock)(int eid, int s, const int *events);
    int (*epoll_wait)(int eid, int *rd, int *rn, int *wr, int *wn, int64_t ms, int *lrd, int *lrn, int *lwr,
                      int *lwn);
    int (*epoll_release)(int eid);
    void (*setloglevel)(int level);
} srt;
static pthread_mutex_t srt_mu = PTHREAD_MUTEX_INITIALIZER;

static bool srt_load(void)
{
    pthread_mutex_lock(&srt_mu);
    if (!srt.tried) {
        srt.tried = true;
        static const char *const libs[] = { "libsrt-gnutls.so.1.5", "libsrt.so.1.5", "libsrt-openssl.so.1.5",
                                            "libsrt-gnutls.so.1.4", "libsrt.so.1.4" };
        void *lib = NULL;
        for (size_t i = 0; i < sizeof libs / sizeof *libs && !lib; i++)
            if ((lib = dlopen(libs[i], RTLD_NOW | RTLD_LOCAL)))
                snprintf(srt.name, sizeof srt.name, "%s", libs[i]);
#define SRT_SYM(f) (*(void **)&srt.f = lib ? dlsym(lib, "srt_" #f) : NULL)
        srt.ok = SRT_SYM(startup) && SRT_SYM(create_socket) && SRT_SYM(setsockflag) && SRT_SYM(bind) &&
                 SRT_SYM(listen) && SRT_SYM(accept) && SRT_SYM(connect) && SRT_SYM(sendmsg2) && SRT_SYM(close) &&
                 SRT_SYM(getsockstate) && SRT_SYM(getlasterror_str) && SRT_SYM(getlasterror) &&
                 SRT_SYM(epoll_create) && SRT_SYM(epoll_add_usock) && SRT_SYM(epoll_wait) &&
                 SRT_SYM(epoll_release) && SRT_SYM(setloglevel);
#undef SRT_SYM
        if (srt.ok) {
            srt.setloglevel(3); /* LOG_ERR */
            srt.ok = srt.startup() >= 0;
        }
        LOG("SRT %s%s", srt.ok ? "with " : "not available (no libsrt)", srt.ok ? srt.name : "");
    }
    bool ok = srt.ok;
    pthread_mutex_unlock(&srt_mu);
    return ok;
}

struct ts_msg {
    uint8_t b[TS_MSG * 188];
    int n; /* packets in b */
};

struct srt_out {
    int n;                      /* the input */
    struct srt_url u;
    char shown[200];            /* the address for logs and status: without the passphrase */
    char name[128];
    bool audio;
    pthread_t thr;
    bool thr_on;
    volatile bool quit;
    pthread_mutex_t mu;         /* the fields below up to state */
    int lsock;                  /* listener: its socket, -1 */
    int peer[SRT_MAX_PEERS], npeers;
    char peer_s[SRT_MAX_PEERS][48];
    bool joined;                /* a receiver came: force a keyframe */
    uint64_t dropped;           /* messages a receiver's full send buffer didn't take */
    char state[128];            /* why nothing is sent (caller: not connected) */
    struct ts_msg vm, am;       /* the video's, the audio's */
    uint8_t cc_pat, cc_pmt, cc_sdt, cc_v, cc_a;
    uint64_t psi_ms;
    uint8_t *pes;               /* the video PES being sent */
    size_t pes_cap;
    int64_t a_base;             /* audio: capture time (100 ns UTC) of a_frames' frame 0 */
    uint64_t a_frames;
};

static uint32_t crc32_mpeg(const uint8_t *p, size_t n)
{
    uint32_t c = 0xffffffff;
    while (n--) {
        c ^= (uint32_t)*p++ << 24;
        for (int k = 0; k < 8; k++)
            c = c & 0x80000000 ? c << 1 ^ 0x04c11db7 : c << 1;
    }
    return c;
}

static void srt_drop_peer(struct srt_out *s, int i, const char *why)
{
    LOG("input %d: SRT: receiver %s left (%s)", s->n, s->peer_s[i], why);
    srt.close(s->peer[i]);
    s->npeers--;
    s->peer[i] = s->peer[s->npeers];
    memcpy(s->peer_s[i], s->peer_s[s->npeers], sizeof s->peer_s[i]);
}

/* The message's packets to every receiver. Sockets don't block: a receiver that can't keep up loses
 * what doesn't fit its send buffer (live mode), the others carry on. */
static void srt_flush(struct srt_out *s, struct ts_msg *m)
{
    if (!m->n)
        return;
    pthread_mutex_lock(&s->mu);
    for (int i = 0; i < s->npeers;) {
        if (srt.sendmsg2(s->peer[i], (const char *)m->b, m->n * 188, NULL) < 0) {
            if (srt.getlasterror(NULL) / 1000 == SRT_EAGAIN_MAJOR) {
                s->dropped++;
            } else {
                srt_drop_peer(s, i, srt.getlasterror_str());
                continue;
            }
        }
        i++;
    }
    pthread_mutex_unlock(&s->mu);
    m->n = 0;
}

/* One TS packet: up to n bytes of p (returns how many), with an adaptation field for af_flags (0x40 random
 * access, 0x10 with the PCR pcr) or to fill the packet. */
static size_t ts_packet(struct srt_out *s, struct ts_msg *m, uint16_t pid, uint8_t *cc, bool start, int af_flags,
                        int64_t pcr, const uint8_t *p, size_t n)
{
    uint8_t *t = m->b + m->n * 188;
    size_t hdr = af_flags ? (af_flags & 0x10 ? 7 : 1) : 0; /* flags byte, PCR */
    size_t room = 184 - (hdr ? 1 + hdr : 0);
    size_t take = n < room ? n : room, left = room - take;
    size_t af = hdr ? 1 + hdr + left : left; /* adaptation field bytes, its length byte included */
    t[0] = 0x47;
    t[1] = (uint8_t)((start ? 0x40 : 0) | pid >> 8);
    t[2] = (uint8_t)pid;
    t[3] = (uint8_t)((af ? 0x30 : 0x10) | (*cc & 15));
    (*cc)++;
    if (af) {
        t[4] = (uint8_t)(af - 1);
        if (af >= 2) {
            t[5] = (uint8_t)af_flags;
            size_t at = 6;
            if (af_flags & 0x10) {
                uint64_t b = (uint64_t)pcr & 0x1ffffffffULL;
                t[6] = (uint8_t)(b >> 25);
                t[7] = (uint8_t)(b >> 17);
                t[8] = (uint8_t)(b >> 9);
                t[9] = (uint8_t)(b >> 1);
                t[10] = (uint8_t)((b & 1) << 7 | 0x7e);
                t[11] = 0;
                at = 12;
            }
            memset(t + at, 0xff, 4 + af - at);
        }
    }
    memcpy(t + 4 + af, p, take);
    if (++m->n == TS_MSG)
        srt_flush(s, m);
    return take;
}

/* A PSI section in one packet (pointer field first, 0xFF after). */
static void ts_section(struct srt_out *s, uint16_t pid, uint8_t *cc, uint8_t *sec, size_t len)
{
    uint32_t crc = crc32_mpeg(sec, len);
    put32(sec + len, crc);
    uint8_t p[184];
    memset(p, 0xff, sizeof p);
    p[0] = 0;
    memcpy(p + 1, sec, len + 4);
    ts_packet(s, &s->vm, pid, cc, true, 0, 0, p, sizeof p);
}

/* PAT, PMT and SDT: before each keyframe, and every half second. */
static void ts_psi(struct srt_out *s)
{
    uint8_t sec[180];
    static const uint8_t pat[] = { 0x00, 0xb0, 13, 0, 1, 0xc1, 0, 0, 0, 1, 0xe0 | TS_PID_PMT >> 8, TS_PID_PMT & 0xff };
    memcpy(sec, pat, sizeof pat);
    ts_section(s, 0, &s->cc_pat, sec, sizeof pat);

    size_t n = 0;
    uint8_t *p = sec;
    p[n++] = 0x02;
    n += 2; /* section length */
    p[n++] = 0;
    p[n++] = 1; /* program 1 */
    p[n++] = 0xc1;
    p[n++] = 0;
    p[n++] = 0;
    p[n++] = 0xe0 | TS_PID_VIDEO >> 8; /* PCR PID */
    p[n++] = TS_PID_VIDEO & 0xff;
    p[n++] = 0xf0;
    p[n++] = 0; /* no program descriptors */
    p[n++] = 0x1b; /* H.264 */
    p[n++] = 0xe0 | TS_PID_VIDEO >> 8;
    p[n++] = TS_PID_VIDEO & 0xff;
    p[n++] = 0xf0;
    p[n++] = 0;
    if (s->audio) {
        static const uint8_t a[] = { 0x06, 0xe0 | TS_PID_AUDIO >> 8, TS_PID_AUDIO & 0xff, 0xf0, 6,
                                     0x05, 4, 'B', 'S', 'S', 'D' }; /* registration: SMPTE 302M */
        memcpy(p + n, a, sizeof a);
        n += sizeof a;
    }
    p[1] = (uint8_t)(0xb0 | (n + 4 - 3) >> 8);
    p[2] = (uint8_t)(n + 4 - 3);
    ts_section(s, TS_PID_PMT, &s->cc_pmt, sec, n);

    size_t nl = strlen(s->name) > 64 ? 64 : strlen(s->name);
    n = 0;
    p[n++] = 0x42; /* SDT, this stream */
    n += 2;
    p[n++] = 0;
    p[n++] = 1;
    p[n++] = 0xc1;
    p[n++] = 0;
    p[n++] = 0;
    p[n++] = 0xff;
    p[n++] = 0x01; /* original network */
    p[n++] = 0xff;
    p[n++] = 0;
    p[n++] = 1; /* service 1 */
    p[n++] = 0xfc;
    size_t dl = 2 + 3 + 5 + nl;
    p[n++] = (uint8_t)(0x80 | dl >> 8); /* running */
    p[n++] = (uint8_t)dl;
    p[n++] = 0x48; /* service descriptor */
    p[n++] = (uint8_t)(dl - 2);
    p[n++] = 0x01; /* digital television */
    p[n++] = 5;
    memcpy(p + n, "PiNDI", 5);
    n += 5;
    p[n++] = (uint8_t)nl;
    memcpy(p + n, s->name, nl);
    n += nl;
    p[1] = (uint8_t)(0xf0 | (n + 4 - 3) >> 8);
    p[2] = (uint8_t)(n + 4 - 3);
    ts_section(s, TS_PID_SDT, &s->cc_sdt, sec, n);
    s->psi_ms = now_ms();
}

static void pes_pts(uint8_t *p, int64_t pts)
{
    uint64_t v = (uint64_t)pts & 0x1ffffffffULL;
    p[0] = (uint8_t)(0x21 | (v >> 29 & 0x0e));
    p[1] = (uint8_t)(v >> 22);
    p[2] = (uint8_t)(v >> 14 | 1);
    p[3] = (uint8_t)(v >> 7);
    p[4] = (uint8_t)(v << 1 | 1);
}

/* A PES packet (header, then d) as TS packets. The video's PES length is given whenever it fits 16 bits,
 * so a receiver knows the picture is complete without waiting for the next one. */
static void ts_pes(struct srt_out *s, struct ts_msg *m, uint16_t pid, uint8_t *cc, uint8_t sid, int64_t pts90,
                   int af_flags, int64_t pcr, const uint8_t *d, size_t n)
{
    uint8_t h[14] = { 0, 0, 1, sid, 0, 0, 0x84, 0x80, 5 }; /* data aligned, a PTS */
    size_t len = n + 8;
    if (len <= 0xffff) {
        h[4] = (uint8_t)(len >> 8);
        h[5] = (uint8_t)len;
    }
    pes_pts(h + 9, pts90);
    /* The first packet: the header and what fits after it. */
    uint8_t first[184];
    size_t room = 184 - (af_flags ? (af_flags & 0x10 ? 8 : 2) : 0) - sizeof h;
    size_t k = n < room ? n : room;
    memcpy(first, h, sizeof h);
    memcpy(first + sizeof h, d, k);
    ts_packet(s, m, pid, cc, true, af_flags, pcr, first, sizeof h + k);
    for (size_t at = k; at < n;)
        at += ts_packet(s, m, pid, cc, false, 0, 0, d + at, n - at);
    srt_flush(s, m); /* the whole picture or sound out now: no waiting for the next one */
}

static int64_t ts90(int64_t t100ns)
{
    return t100ns * 9 / 1000;
}

static int srt_peers(struct srt_out *s)
{
    pthread_mutex_lock(&s->mu);
    int n = s->npeers;
    pthread_mutex_unlock(&s->mu);
    return n;
}

/* Did a receiver come since the last call? (Then a keyframe is wanted.) */
static bool srt_take_joined(struct srt_out *s)
{
    pthread_mutex_lock(&s->mu);
    bool j = s->joined;
    s->joined = false;
    pthread_mutex_unlock(&s->mu);
    return j;
}

/* One encoded picture (Annex B), captured at t (100 ns UTC). */
static void srt_send_video(struct srt_out *s, const uint8_t *data, size_t len, int64_t t, bool key)
{
    if (!srt_peers(s))
        return;
    if (key || now_ms() - s->psi_ms >= 500)
        ts_psi(s);
    /* An access unit delimiter first (H.222.0 asks for one in each access unit), unless it has one. */
    static const uint8_t aud[] = { 0, 0, 0, 1, 0x09, 0xf0 };
    size_t at = 0;
    const uint8_t *nal = NULL;
    size_t nlen = 0;
    bool has_aud = annexb_next(data, len, &at, &nal, &nlen) && nlen && (nal[0] & 0x1f) == 9;
    size_t need = len + (has_aud ? 0 : sizeof aud);
    if (s->pes_cap < need) {
        free(s->pes);
        s->pes_cap = need + 65536;
        if (!(s->pes = malloc(s->pes_cap))) {
            s->pes_cap = 0;
            return;
        }
    }
    size_t n = 0;
    if (!has_aud) {
        memcpy(s->pes, aud, sizeof aud);
        n = sizeof aud;
    }
    memcpy(s->pes + n, data, len);
    n += len;
    ts_pes(s, &s->vm, TS_PID_VIDEO, &s->cc_v, 0xe0, ts90(t), 0x10 | (key ? 0x40 : 0), ts90(now_100ns()) - TS_PCR_DELAY,
           s->pes, n);
}

static uint8_t rev8(uint8_t b)
{
    b = (uint8_t)((b & 0xf0) >> 4 | (b & 0x0f) << 4);
    b = (uint8_t)((b & 0xcc) >> 2 | (b & 0x33) << 2);
    return (uint8_t)((b & 0xaa) >> 1 | (b & 0x55) << 1);
}

/* n frames of 48 kHz stereo audio as SMPTE 302M: 4-byte header, then each frame as two 20-bit AES3
 * subframes (16-bit sample, then the V, U, C and F bits; F starts each 192-frame block), bit-reversed. */
static void srt_send_audio(struct srt_out *s, const int16_t *smp, int n)
{
    if (!srt_peers(s))
        return;
    /* The first frame's capture time: they end about now. Kept on a frame count, so the PTS run evenly,
     * and taken again if that drifts from the clock by more than 40 ms. */
    int64_t now = now_100ns(), start = now - (int64_t)n * 10000000 / AUDIO_RATE;
    int64_t expect = s->a_base + (int64_t)(s->a_frames * 10000000 / AUDIO_RATE);
    if (!s->a_base || llabs(expect - start) > 400000) {
        s->a_base = start;
        s->a_frames = 0;
    }
    uint8_t buf[4 + TS_AUDIO_MAX * 5];
    for (int at = 0; at < n; at += TS_AUDIO_MAX) {
        int k = n - at < TS_AUDIO_MAX ? n - at : TS_AUDIO_MAX;
        put32(buf, (uint32_t)(k * 5) << 16); /* payload size; 2 channels, 16 bits */
        uint8_t *o = buf + 4;
        for (int i = 0; i < k; i++, o += 5) {
            uint16_t l = (uint16_t)smp[(at + i) * 2], r = (uint16_t)smp[(at + i) * 2 + 1];
            uint8_t f = (s->a_frames + (uint64_t)i) % 192 ? 0 : 0x10;
            o[0] = rev8((uint8_t)l);
            o[1] = rev8((uint8_t)(l >> 8));
            o[2] = (uint8_t)(rev8((uint8_t)((r & 0x0f) << 4)) | f);
            o[3] = rev8((uint8_t)(r >> 4));
            o[4] = rev8((uint8_t)(r >> 12));
        }
        int64_t t = s->a_base + (int64_t)(s->a_frames * 10000000 / AUDIO_RATE);
        ts_pes(s, &s->am, TS_PID_AUDIO, &s->cc_a, 0xbd, ts90(t), 0, 0, buf, 4 + (size_t)k * 5);
        s->a_frames += (uint64_t)k;
    }
}

/* SRT options both ends must agree on; set before connecting (inherited by a listener's receivers). */
static bool srt_options(struct srt_out *s, int sock)
{
    int tos = 0x88; /* DSCP AF41, as for RTP */
    srt.setsockflag(sock, SRTO_IPTOS, &tos, sizeof tos);
    if (s->u.latency >= 0 && srt.setsockflag(sock, SRTO_LATENCY, &s->u.latency, sizeof s->u.latency) < 0)
        return false;
    if (*s->u.passphrase &&
        srt.setsockflag(sock, SRTO_PASSPHRASE, s->u.passphrase, (int)strlen(s->u.passphrase)) < 0)
        return false;
    if (*s->u.streamid && !s->u.listen &&
        srt.setsockflag(sock, SRTO_STREAMID, s->u.streamid, (int)strlen(s->u.streamid)) < 0)
        return false;
    return true;
}

static void srt_set_state(struct srt_out *s, const char *fmt, ...) __attribute__((format(printf, 2, 3)));
static void srt_set_state(struct srt_out *s, const char *fmt, ...)
{
    char buf[sizeof s->state];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof buf, fmt, ap);
    va_end(ap);
    pthread_mutex_lock(&s->mu);
    if (strcmp(buf, s->state)) {
        if (*buf)
            LOG("input %d: SRT: %s", s->n, buf);
        snprintf(s->state, sizeof s->state, "%s", buf);
    }
    pthread_mutex_unlock(&s->mu);
}

static void srt_add_peer(struct srt_out *s, int sock, const struct sockaddr_in *a)
{
    bool no = false;
    srt.setsockflag(sock, SRTO_SNDSYN, &no, sizeof no);
    pthread_mutex_lock(&s->mu);
    if (s->npeers == SRT_MAX_PEERS) {
        pthread_mutex_unlock(&s->mu);
        LOG("input %d: SRT: refused a receiver: already %d", s->n, SRT_MAX_PEERS);
        srt.close(sock);
        return;
    }
    char ip[INET_ADDRSTRLEN] = "?";
    inet_ntop(AF_INET, &a->sin_addr, ip, sizeof ip);
    snprintf(s->peer_s[s->npeers], sizeof s->peer_s[0], "%s:%d", ip, ntohs(a->sin_port));
    s->peer[s->npeers++] = sock;
    s->joined = true;
    LOG("input %d: SRT: receiver %s connected (%d now)", s->n, s->peer_s[s->npeers - 1], s->npeers);
    pthread_mutex_unlock(&s->mu);
}

/* Connections: a listener accepts receivers; a caller calls its receiver, again when it drops. Receivers
 * whose connection broke are let go here too (a send to them may not fail at once). */
static void *srt_thread(void *arg)
{
    struct srt_out *s = arg;
    int eid = -1;
    uint64_t next_try = 0;
    while (!s->quit) {
        uint64_t now = now_ms();
        if (s->u.listen && s->lsock < 0 && now >= next_try) {
            int l = srt.create_socket();
            struct sockaddr_in a = { .sin_family = AF_INET, .sin_port = htons((uint16_t)s->u.port) };
            if (l >= 0 && srt_options(s, l) && !srt.bind(l, (struct sockaddr *)&a, sizeof a) &&
                !srt.listen(l, SRT_MAX_PEERS)) {
                s->lsock = l;
                eid = srt.epoll_create();
                int ev = SRT_EPOLL_IN | SRT_EPOLL_ERR;
                srt.epoll_add_usock(eid, l, &ev);
                srt_set_state(s, "waiting for receivers on port %d", s->u.port);
            } else {
                srt_set_state(s, "cannot listen on port %d: %s (trying again)", s->u.port, srt.getlasterror_str());
                if (l >= 0)
                    srt.close(l);
                next_try = now + 5000;
            }
        }
        if (s->lsock >= 0) {
            int rd[2], rn = 2;
            if (srt.epoll_wait(eid, rd, &rn, NULL, NULL, 300, NULL, NULL, NULL, NULL) > 0) {
                struct sockaddr_in a;
                int al = sizeof a;
                int c = srt.accept(s->lsock, (struct sockaddr *)&a, &al);
                if (c >= 0)
                    srt_add_peer(s, c, &a);
            }
        } else if (!s->u.listen && !srt_peers(s) && now >= next_try) {
            /* Called again every 2 s: the address is looked up each time (it may be a name, or change). */
            struct addrinfo hints = { .ai_family = AF_INET, .ai_socktype = SOCK_DGRAM }, *ai = NULL;
            char port[8];
            snprintf(port, sizeof port, "%d", s->u.port);
            int c = -1;
            next_try = now + 2000;
            if (getaddrinfo(s->u.host, port, &hints, &ai) || !ai) {
                srt_set_state(s, "calling %s: unknown address (trying again)", s->shown);
            } else if ((c = srt.create_socket()) >= 0 && srt_options(s, c) &&
                       !srt.connect(c, ai->ai_addr, (int)ai->ai_addrlen)) {
                srt_add_peer(s, c, (struct sockaddr_in *)ai->ai_addr);
                srt_set_state(s, "%s", "");
                c = -1;
            } else {
                srt_set_state(s, "calling %s: %s (trying again)", s->shown,
                              c < 0 ? "cannot make a socket" : srt.getlasterror_str());
            }
            if (c >= 0)
                srt.close(c);
            if (ai)
                freeaddrinfo(ai);
        } else {
            usleep(200000);
        }
        pthread_mutex_lock(&s->mu);
        for (int i = 0; i < s->npeers;) {
            if (srt.getsockstate(s->peer[i]) > SRTS_CONNECTED)
                srt_drop_peer(s, i, "connection closed");
            else
                i++;
        }
        pthread_mutex_unlock(&s->mu);
    }
    if (eid >= 0)
        srt.epoll_release(eid);
    return NULL;
}

/* PINDI_IN<n>_SRT_URL for input n. False (state: why) if it can't be sent. */
static bool srt_open(struct srt_out *s, int n, const char *url, const char *name, bool audio)
{
    memset(s, 0, sizeof *s);
    pthread_mutex_init(&s->mu, NULL);
    s->n = n;
    s->lsock = -1;
    s->audio = audio;
    snprintf(s->name, sizeof s->name, "%s", name);
    const char *err;
    if (!srt_parse(url, &s->u, &err)) {
        LOG("input %d: SRT: \"%s\": %s", n, url, err);
        snprintf(s->state, sizeof s->state, "not sent: the SRT address is not valid (%s)", err);
        return false;
    }
    int k = snprintf(s->shown, sizeof s->shown, "srt://%s:%d", s->u.listen ? "" : s->u.host, s->u.port);
    if (s->u.latency >= 0 && k > 0 && (size_t)k < sizeof s->shown)
        k += snprintf(s->shown + k, sizeof s->shown - (size_t)k, "?latency=%d", s->u.latency);
    if (*s->u.passphrase && k > 0 && (size_t)k < sizeof s->shown)
        snprintf(s->shown + k, sizeof s->shown - (size_t)k, " (encrypted)");
    if (!srt_load()) {
        snprintf(s->state, sizeof s->state, "not sent: SRT needs libsrt (run install.sh with internet access)");
        return false;
    }
    LOG("input %d: H.264 SRT %s %s", n, s->u.listen ? "listening at" : "calling", s->shown);
    s->thr_on = !pthread_create(&s->thr, NULL, srt_thread, s);
    return s->thr_on;
}

static void srt_close(struct srt_out *s)
{
    if (s->thr_on) {
        s->quit = true;
        pthread_join(s->thr, NULL);
        s->thr_on = false;
    }
    pthread_mutex_lock(&s->mu);
    while (s->npeers)
        srt_drop_peer(s, 0, "stopped");
    if (s->lsock >= 0)
        srt.close(s->lsock);
    s->lsock = -1;
    pthread_mutex_unlock(&s->mu);
    free(s->pes);
    s->pes = NULL;
    s->pes_cap = 0;
}

/* The status line while sending. */
static void srt_status(struct srt_out *s, char *buf, size_t n, double fps)
{
    pthread_mutex_lock(&s->mu);
    if (s->npeers == 1 && !s->u.listen)
        snprintf(buf, n, "sending H.264 SRT to %.80s (%.0f fps)", s->shown, fps);
    else if (s->npeers)
        snprintf(buf, n, "sending H.264 SRT to %d receiver%s on port %d (%.0f fps)", s->npeers,
                 s->npeers == 1 ? "" : "s", s->u.port, fps);
    else if (s->u.listen)
        snprintf(buf, n, "ready: SRT receivers can connect on port %d%s", s->u.port,
                 *s->u.passphrase ? " (encrypted)" : "");
    else
        snprintf(buf, n, "%.120s", *s->state ? s->state : "calling");
    pthread_mutex_unlock(&s->mu);
}

/* ------------------------------------------------------------------ input thread */

struct input {
    int n;
    char device[256], name[128], mode_s[64];
    bool audio_on;
    bool audio_sync; /* keep the audio in step with this Pi's clock (PINDI_IN<n>_AUDIO_SYNC) */
    bool hx; /* PINDI_IN<n>_CODEC=hx: NDI|HX through the hardware H.264 encoder */
    bool rtp; /* PINDI_IN<n>_CODEC=rtp: H.264 over RTP through the hardware encoder; no NDI sender */
    char rtp_dest[64];
    struct rtp_out r; /* open while sending (r.fd >= 0) */
    bool srt; /* PINDI_IN<n>_CODEC=srt: H.264 over SRT through the hardware encoder; no NDI sender */
    char srt_url[256];
    struct srt_out s; /* s.thr_on while it can send */
    int kbps; /* rtp, srt: PINDI_IN<n>_KBPS, 0 = automatic */
    bool video_range; /* PINDI_IN<n>_RANGE=video: the device's JPEG is already 16-235 (else converted) */
    pthread_t thr, athr;
    NDIlib_send_instance_t send;
    int w, h, fps_n, fps_d; /* last capture mode, for the black "no signal" frames */
    uint8_t *black;
    pthread_mutex_t lock;
    char audio[256]; /* audio status line (lock) */
    struct local_feed *lf; /* header of in<n>.local (fixed mapping, the audio thread uses it too) */
    int lf_fd;
    uint8_t *lf_frames;    /* its frame slots (remapped when the capture mode changes) */
    size_t lf_frames_len;
    char csi_why[192];     /* CSI-2 bridge: why there was no picture at the last try (logged on change) */
};

/* Create in<n>.local, or resize its frame slots for a new capture mode. */
static bool local_setup(struct input *in, int w, int h, uint32_t fmt, int fps_n, int fps_d)
{
    if (!in->lf) {
        char path[64];
        snprintf(path, sizeof path, RUN_DIR "/in%d.local", in->n);
        in->lf_fd = open(path, O_RDWR | O_CREAT | O_TRUNC | O_CLOEXEC, 0644);
        if (in->lf_fd < 0 || ftruncate(in->lf_fd, LOCAL_DATA))
            return false;
        in->lf = mmap(NULL, LOCAL_DATA, PROT_READ | PROT_WRITE, MAP_SHARED, in->lf_fd, 0);
        if (in->lf == MAP_FAILED) {
            in->lf = NULL;
            return false;
        }
        snprintf(in->lf->name, sizeof in->lf->name, "%s", in->name);
        in->lf->version = 1;
        STORE(in->lf->magic, LOCAL_MAGIC);
    }
    struct local_feed *lf = in->lf;
    size_t slot = (size_t)w * h * 2, len = slot * LOCAL_SLOTS;
    if (in->lf_frames && in->lf_frames_len != len) {
        munmap(in->lf_frames, in->lf_frames_len);
        in->lf_frames = NULL;
    }
    STORE(lf->seq, 0); /* readers wait for the first frame of the new layout */
    for (int i = 0; i < LOCAL_SLOTS; i++)
        STORE(lf->slot_seq[i], 0);
    if (!in->lf_frames) {
        if (ftruncate(in->lf_fd, LOCAL_DATA + len))
            return false;
        in->lf_frames = mmap(NULL, len, PROT_READ | PROT_WRITE, MAP_SHARED, in->lf_fd, LOCAL_DATA);
        if (in->lf_frames == MAP_FAILED) {
            in->lf_frames = NULL;
            return false;
        }
        in->lf_frames_len = len;
    }
    lf->w = w;
    lf->h = h;
    lf->fps_n = fps_n;
    lf->fps_d = fps_d;
    lf->fmt = fmt;
    STORE(lf->slot_size, (uint32_t)slot);
    return true;
}

/* NDI timestamp (100 ns since 1970) of a V4L2 buffer's capture time (monotonic). */
static int64_t capture_time(const struct timeval *tv)
{
    struct timespec rt, mt;
    clock_gettime(CLOCK_REALTIME, &rt);
    clock_gettime(CLOCK_MONOTONIC, &mt);
    int64_t age_us = ((int64_t)mt.tv_sec * 1000000 + mt.tv_nsec / 1000) - ((int64_t)tv->tv_sec * 1000000 + tv->tv_usec);
    if (!tv->tv_sec || age_us < 0 || age_us > 1000000)
        age_us = 0;
    return ((int64_t)rt.tv_sec * 1000000 + rt.tv_nsec / 1000 - age_us) * 10;
}

static void set_audio_status(struct input *in, const char *s)
{
    pthread_mutex_lock(&in->lock);
    snprintf(in->audio, sizeof in->audio, "%s", s);
    pthread_mutex_unlock(&in->lock);
}

/* fps = frames sent, cam_fps = frames from the device, damaged = frames skipped (last 2 s). */
static void status(struct input *in, const char *state, const char *mode, double fps, double cam_fps, int damaged)
{
    char buf[1024], path[64], audio[sizeof in->audio];
    pthread_mutex_lock(&in->lock);
    snprintf(audio, sizeof audio, "%s", in->audio);
    pthread_mutex_unlock(&in->lock);
    snprintf(buf, sizeof buf, "state=%s\ndevice=%s\nname=%s\nmode=%s\nfps=%.1f\ncam_fps=%.1f\ndamaged=%d\naudio=%s\n",
             state, in->device, in->name, mode ? mode : "", fps, cam_fps, damaged, audio);
    snprintf(path, sizeof path, RUN_DIR "/in%d.status", in->n);
    write_file_atomic(path, buf);
}

struct capbuf { void *start; size_t len; };

/* No picture from the device (HDMI source unplugged, device lost): send black once a second, so
 * receivers don't sit on a frozen last frame. Only while someone is connected. */
static void send_black(struct input *in, NDIlib_send_instance_t send)
{
    if (!send || in->w <= 0 || in->h <= 0 || !ndi->send_get_no_connections(send, 0))
        return;
    if (!in->black) {
        size_t n = (size_t)in->w * in->h * 2;
        in->black = malloc(n);
        if (!in->black)
            return;
        for (size_t i = 0; i < n; i += 2) {
            in->black[i] = 0x80; /* U/V */
            in->black[i + 1] = 0x10; /* Y: black */
        }
    }
    NDIlib_video_frame_v2_t f = {
        .xres = in->w, .yres = in->h, .FourCC = NDIlib_FourCC_video_type_UYVY,
        .frame_rate_N = in->fps_n, .frame_rate_D = in->fps_d,
        .frame_format_type = NDIlib_frame_format_type_progressive,
        .timecode = NDIlib_send_timecode_synthesize, .p_data = in->black, .line_stride_in_bytes = in->w * 2,
    };
    ndi->send_send_video_v2(send, &f); /* synchronous: never mixed with the async capture frames */
}

/* Decoding runs in worker threads: one MJPEG decoder thread manages about 30 fps at 1080p on a
 * Pi 3, two take turns for 60 (three with NDI|HX, whose encoding needs no CPU: decoding plus the
 * conversion for the encoder take ~50 ms a frame). Frame s goes to worker s % nw; results are published (shared memory,
 * NDI) strictly in order. NDI encoding has its own thread: SpeedHQ at 1080p takes ~50 ms a frame on
 * a Pi 3, so it sends the newest finished frame whenever it is free and the others skip NDI. That
 * way the encoder never holds up this device's own display. NDI frames: NOUT buffers, each free,
 * being filled by a worker, pending (the newest, waiting for the encoder) or being sent. */
#define MAX_WORKERS 3
#define NOUT 4
enum { O_FREE, O_FILLING, O_PENDING, O_SENDING };

/* ------------------------------------------------------------------ NDI|HX: hardware H.264 encoder */

/* With PINDI_IN<n>_CODEC=hx the stream is sent as NDI|HX: the Pi 3/4 hardware encoder
 * (bcm2835-codec, a V4L2 memory-to-memory device) makes H.264 from the UYVY pictures and the NDI
 * Advanced SDK sends it as is. No SpeedHQ encoding on the CPU, a tenth of the bandwidth. The
 * pipeline's NDI frames are then the encoder's input buffers: the workers convert into them
 * directly; O_SENDING = queued in the encoder. */
#define ENC_OUT_BUFS 4

struct hwenc {
    int fd;
    uint8_t *in_map[NOUT];
    size_t in_len[NOUT];
    uint8_t *out_map[ENC_OUT_BUFS];
    size_t out_len[ENC_OUT_BUFS];
    int bpl;            /* bytes per UYVY input line */
    uint8_t *pkt;       /* NDIlib_compressed_packet_t + frame + SPS/PPS */
    size_t pkt_cap;
    int64_t frame_100ns;
    uint64_t sent;
    int keys, forced; /* keyframes sent / forced for new receivers, since the last log */
    int conns;        /* receivers at the last frame: a new one gets a keyframe right away */
};

static char enc_dev[32]; /* "" = no hardware H.264 encoder */
/* Advanced SDK only (NULL with the standard SDK, which cannot send compressed video). */
static int (*ndi_target_bit_rate)(NDIlib_send_instance_t, const NDIlib_video_frame_v2_t *);
static bool (*ndi_keyframe_required)(NDIlib_send_instance_t, const NDIlib_video_frame_v2_t *);

static void hwenc_find(void)
{
    for (int i = 0; i < 64 && !*enc_dev; i++) {
        char path[64], name[64] = "";
        snprintf(path, sizeof path, "/sys/class/video4linux/video%d/name", i);
        FILE *f = fopen(path, "r");
        if (!f)
            continue;
        if (fgets(name, sizeof name, f) && !strncmp(name, "bcm2835-codec-encode", 20) && !strstr(name, "image"))
            snprintf(enc_dev, sizeof enc_dev, "/dev/video%d", i);
        fclose(f);
    }
}

static void hwenc_close(struct hwenc *e)
{
    if (e->fd < 0)
        return;
    int t = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
    xioctl(e->fd, VIDIOC_STREAMOFF, &t);
    t = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    xioctl(e->fd, VIDIOC_STREAMOFF, &t);
    for (int i = 0; i < NOUT; i++)
        if (e->in_map[i])
            munmap(e->in_map[i], e->in_len[i]);
    for (int i = 0; i < ENC_OUT_BUFS; i++)
        if (e->out_map[i])
            munmap(e->out_map[i], e->out_len[i]);
    close(e->fd);
    free(e->pkt);
    memset(e, 0, sizeof *e);
    e->fd = -1;
}

static void enc_ctrl(int fd, uint32_t id, int value)
{
    struct v4l2_control c = { .id = id, .value = value };
    xioctl(fd, VIDIOC_S_CTRL, &c);
}

/* send: the NDI sender (NDI|HX), NULL for RTP and SRT (what: their name). kbps: the bit rate asked for,
 * 0 = automatic. */
static bool hwenc_open(struct hwenc *e, const struct mode *m, NDIlib_send_instance_t send, const char *what, int n,
                       int kbps)
{
    memset(e, 0, sizeof *e);
    e->fd = open(enc_dev, O_RDWR | O_NONBLOCK | O_CLOEXEC);
    if (e->fd < 0)
        goto fail;
    struct v4l2_format in = { .type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE };
    in.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_UYVY;
    in.fmt.pix_mp.width = m->w;
    in.fmt.pix_mp.height = m->h;
    in.fmt.pix_mp.num_planes = 1;
    struct v4l2_format out = { .type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE };
    out.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_H264;
    out.fmt.pix_mp.width = m->w;
    out.fmt.pix_mp.height = m->h;
    out.fmt.pix_mp.num_planes = 1;
    out.fmt.pix_mp.plane_fmt[0].sizeimage = 2 << 20;
    if (xioctl(e->fd, VIDIOC_S_FMT, &in) || xioctl(e->fd, VIDIOC_S_FMT, &out))
        goto fail;
    e->bpl = (int)in.fmt.pix_mp.plane_fmt[0].bytesperline;
    struct v4l2_streamparm sp = { .type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE };
    sp.parm.output.timeperframe.numerator = m->num;
    sp.parm.output.timeperframe.denominator = m->den;
    xioctl(e->fd, VIDIOC_S_PARM, &sp);
    /* The bit rate NDI wants for HX at this size and rate; a keyframe every second; parameter sets
     * with every keyframe, so receivers can join any time. */
    NDIlib_video_frame_v2_t f = { .xres = m->w, .yres = m->h, .FourCC = (NDIlib_FourCC_video_type_e)NDI_LIB_FOURCC('H', '2', '6', '4'),
                                  .frame_rate_N = (int)m->den, .frame_rate_D = (int)m->num,
                                  .frame_format_type = NDIlib_frame_format_type_progressive };
    int bps = kbps > 0 ? kbps * 1000 : ndi_target_bit_rate && send ? ndi_target_bit_rate(send, &f) : 0;
    if (bps <= 0)
        bps = (int)((double)m->w * m->h * m->den / m->num * 0.1); /* ~12 Mbit/s at 1080p60 */
    if (bps > 25000000)
        bps = 25000000;
    enc_ctrl(e->fd, V4L2_CID_MPEG_VIDEO_BITRATE, bps);
    /* A keyframe at least every 30 frames: the encoder manages about 1080p30, so that's about a second
     * even from a 60 fps card (a period of 60 gave one every 2 s). */
    int period = (int)(m->den / m->num + 0.5);
    enc_ctrl(e->fd, V4L2_CID_MPEG_VIDEO_H264_I_PERIOD, period > 30 ? 30 : period);
    enc_ctrl(e->fd, V4L2_CID_MPEG_VIDEO_REPEAT_SEQ_HEADER, 1);
    enc_ctrl(e->fd, V4L2_CID_MPEG_VIDEO_H264_LEVEL, V4L2_MPEG_VIDEO_H264_LEVEL_4_2);

    struct v4l2_requestbuffers rb = { .count = NOUT, .type = in.type, .memory = V4L2_MEMORY_MMAP };
    if (xioctl(e->fd, VIDIOC_REQBUFS, &rb) || rb.count < NOUT)
        goto fail;
    for (int i = 0; i < NOUT; i++) {
        struct v4l2_plane pl = { 0 };
        struct v4l2_buffer b = { .index = i, .type = in.type, .memory = V4L2_MEMORY_MMAP, .m.planes = &pl, .length = 1 };
        if (xioctl(e->fd, VIDIOC_QUERYBUF, &b))
            goto fail;
        e->in_len[i] = pl.length;
        e->in_map[i] = mmap(NULL, pl.length, PROT_READ | PROT_WRITE, MAP_SHARED, e->fd, pl.m.mem_offset);
        if (e->in_map[i] == MAP_FAILED) {
            e->in_map[i] = NULL;
            goto fail;
        }
    }
    struct v4l2_requestbuffers ro = { .count = ENC_OUT_BUFS, .type = out.type, .memory = V4L2_MEMORY_MMAP };
    if (xioctl(e->fd, VIDIOC_REQBUFS, &ro) || ro.count < 1)
        goto fail;
    for (int i = 0; i < (int)ro.count && i < ENC_OUT_BUFS; i++) {
        struct v4l2_plane pl = { 0 };
        struct v4l2_buffer b = { .index = i, .type = out.type, .memory = V4L2_MEMORY_MMAP, .m.planes = &pl, .length = 1 };
        if (xioctl(e->fd, VIDIOC_QUERYBUF, &b))
            goto fail;
        e->out_len[i] = pl.length;
        e->out_map[i] = mmap(NULL, pl.length, PROT_READ, MAP_SHARED, e->fd, pl.m.mem_offset);
        if (e->out_map[i] == MAP_FAILED) {
            e->out_map[i] = NULL;
            goto fail;
        }
        if (xioctl(e->fd, VIDIOC_QBUF, &b))
            goto fail;
    }
    int t = in.type;
    if (xioctl(e->fd, VIDIOC_STREAMON, &t))
        goto fail;
    t = out.type;
    if (xioctl(e->fd, VIDIOC_STREAMON, &t))
        goto fail;
    e->frame_100ns = (int64_t)(10000000.0 * m->num / m->den);
    LOG("input %d: %s: hardware H.264 encoder %s, %.1f Mbit/s", n, what, enc_dev, bps / 1e6);
    return true;
fail:
    LOG("input %d: hardware H.264 encoder: %s - %s", n, strerror(errno), send ? "sending Full NDI" : "not sent");
    hwenc_close(e);
    return false;
}

/* NDIlib_compressed_packet_t (Processing.NDI.Advanced.h): 44 bytes before the frame data. */
#define PKT_HDR 44

/* One encoded frame -> NDI: H.264 Annex B, with SPS/PPS moved to the packet's extra data. */
static void hwenc_send(struct hwenc *e, NDIlib_send_instance_t send, const struct mode *m, const uint8_t *data,
                       size_t len, bool key, int64_t timecode)
{
    if (e->pkt_cap < PKT_HDR + len * 2) {
        free(e->pkt);
        e->pkt_cap = PKT_HDR + len * 2 + 65536;
        e->pkt = malloc(e->pkt_cap);
        if (!e->pkt) {
            e->pkt_cap = 0;
            return;
        }
    }
    /* Split into NAL units (start codes 00 00 01 / 00 00 00 01). */
    uint8_t *frame = e->pkt + PKT_HDR, *extra = e->pkt + PKT_HDR + len;
    size_t flen = 0, xlen = 0, i = 0;
    while (i + 3 < len) {
        size_t sc = i;
        while (sc + 3 < len && !(data[sc] == 0 && data[sc + 1] == 0 && data[sc + 2] == 1))
            sc++;
        if (sc + 3 >= len)
            break;
        size_t start = sc > 0 && data[sc - 1] == 0 ? sc - 1 : sc; /* include a 4-byte start code's zero */
        size_t nxt = sc + 3;
        while (nxt + 3 < len && !(data[nxt] == 0 && data[nxt + 1] == 0 && data[nxt + 2] == 1))
            nxt++;
        size_t end = nxt + 3 < len ? (data[nxt - 1] == 0 ? nxt - 1 : nxt) : len;
        int type = data[sc + 3] & 0x1f;
        if (type == 7 || type == 8) {
            memcpy(extra + xlen, data + start, end - start);
            xlen += end - start;
        } else {
            memcpy(frame + flen, data + start, end - start);
            flen += end - start;
        }
        i = end;
    }
    memmove(frame + flen, extra, xlen); /* extra data right after the frame data */
    int64_t pts = (int64_t)e->sent * e->frame_100ns;
    uint8_t *h = e->pkt;
    int32_t ver = PKT_HDR;
    uint32_t cc = NDI_LIB_FOURCC('H', '2', '6', '4'), flags = key ? 1 : 0, ds = (uint32_t)flen,
             es = key ? (uint32_t)xlen : 0;
    uint64_t zero = 0;
    memcpy(h, &ver, 4);
    memcpy(h + 4, &cc, 4);
    memcpy(h + 8, &pts, 8);
    memcpy(h + 16, &pts, 8); /* dts: no B-frames */
    memcpy(h + 24, &zero, 8);
    memcpy(h + 32, &flags, 4);
    memcpy(h + 36, &ds, 4);
    memcpy(h + 40, &es, 4);
    NDIlib_video_frame_v2_t f = {
        .xres = m->w, .yres = m->h,
        .FourCC = (NDIlib_FourCC_video_type_e)NDI_LIB_FOURCC('H', '2', '6', '4'),
        .frame_rate_N = (int)m->den, .frame_rate_D = (int)m->num,
        .frame_format_type = NDIlib_frame_format_type_progressive,
        .timecode = timecode,
        .p_data = e->pkt,
        .data_size_in_bytes = (int)(PKT_HDR + flen + es),
    };
    ndi->send_send_video_v2(send, &f);
    e->sent++;
    e->keys += key;
}

struct pipeline;
struct worker {
    struct pipeline *pl;
    pthread_t thr;
    bool busy, ndi, local;
    uint64_t seq;
    uint8_t *src;
    size_t len, cap;
    int64_t time;
    bool dark;  /* the decoded picture is black */
    tjhandle tj;
    double warn_rate; /* share of recent frames the JPEG decoder warned about */
    uint8_t *planes;  /* planar picture when there is no shared memory */
};

struct pipeline {
    struct input *in;
    NDIlib_send_instance_t send;
    struct mode m;
    pthread_mutex_t mu;
    pthread_cond_t cv;
    bool quit;
    uint64_t next; /* next frame number to publish */
    int nw;
    struct worker w[MAX_WORKERS];
    uint8_t *out[NOUT];
    int ostate[NOUT];
    int pending; /* out[] index waiting for the encoder, -1 = none */
    int64_t out_time[NOUT]; /* capture time of the picture in out[] (100 ns UTC: NDI timecode) */
    int64_t black_since;    /* capture time the picture went black (mu), 0 = it isn't */
    bool flush;  /* ndi_wait_idle: make NDI let go of the last frame */
    NDIlib_video_frame_v2_t pframe;
    pthread_t ndi_thr;
    int frames, damaged, ndi_sent; /* published / skipped / sent over NDI since the last status (mu) */
    bool hx;          /* H.264 through enc (NDI|HX, RTP or SRT); out[] are its input buffers */
    bool rtp;         /* the H.264 goes out as RTP (in->r), not to NDI */
    bool srt;         /* the H.264 goes out over SRT (in->s), not to NDI */
    struct hwenc enc;
    int out_stride;   /* bytes per UYVY line in out[] */
};

/* NDI|HX: queue frame i (if any) to the encoder, then send what it has finished. Up to NOUT
 * frames are in the encoder at once; with nothing new to queue, wait up to wait_ms for output. */
static void hx_step(struct pipeline *pl, int i, int wait_ms)
{
    struct hwenc *e = &pl->enc;
    if (i >= 0 && pl->send) {
        /* A new receiver can't start before a keyframe: send one now (NDI|HX: within 100 ms). The
         * SDK's own request is checked too, but it never fired with these compressed frames. (RTP
         * receivers join at the next regular keyframe, at most a second away.) */
        int conns = ndi->send_get_no_connections(pl->send, 0);
        if (conns > e->conns || (ndi_keyframe_required && ndi_keyframe_required(pl->send, &pl->pframe))) {
            enc_ctrl(e->fd, V4L2_CID_MPEG_VIDEO_FORCE_KEY_FRAME, 1);
            e->forced++;
        }
        e->conns = conns;
    } else if (i >= 0 && pl->srt && srt_take_joined(&pl->in->s)) { /* the same for a new SRT receiver */
        enc_ctrl(e->fd, V4L2_CID_MPEG_VIDEO_FORCE_KEY_FRAME, 1);
        e->forced++;
    }
    if (i >= 0) {
        struct v4l2_plane p = { .bytesused = (uint32_t)e->in_len[i], .length = (uint32_t)e->in_len[i] };
        struct v4l2_buffer b = { .index = i, .type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE, .memory = V4L2_MEMORY_MMAP,
                                 .m.planes = &p, .length = 1 };
        pthread_mutex_lock(&pl->mu);
        int64_t t = pl->out_time[i];
        pthread_mutex_unlock(&pl->mu);
        b.timestamp.tv_sec = t / 10000000;
        b.timestamp.tv_usec = t % 10000000 / 10;
        if (xioctl(e->fd, VIDIOC_QBUF, &b)) {
            pthread_mutex_lock(&pl->mu);
            pl->ostate[i] = O_FREE;
            pthread_cond_broadcast(&pl->cv);
            pthread_mutex_unlock(&pl->mu);
        }
    }
    uint64_t until = now_ms() + (uint64_t)wait_ms;
    bool got = false;
    for (;;) {
        for (;;) { /* pictures the encoder is done with */
            struct v4l2_plane p = { 0 };
            struct v4l2_buffer b = { .type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE, .memory = V4L2_MEMORY_MMAP,
                                     .m.planes = &p, .length = 1 };
            if (xioctl(e->fd, VIDIOC_DQBUF, &b))
                break;
            pthread_mutex_lock(&pl->mu);
            pl->ostate[b.index] = O_FREE;
            pthread_cond_broadcast(&pl->cv);
            pthread_mutex_unlock(&pl->mu);
        }
        for (;;) {
            struct v4l2_plane p = { 0 };
            struct v4l2_buffer b = { .type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE, .memory = V4L2_MEMORY_MMAP,
                                     .m.planes = &p, .length = 1 };
            if (xioctl(e->fd, VIDIOC_DQBUF, &b))
                break;
            if (p.bytesused && b.index < ENC_OUT_BUFS) {
                int64_t tc = ((int64_t)b.timestamp.tv_sec * 1000000 + b.timestamp.tv_usec) * 10;
                const uint8_t *data = e->out_map[b.index] + p.data_offset;
                size_t len = p.bytesused - p.data_offset;
                bool key = b.flags & V4L2_BUF_FLAG_KEYFRAME;
                if (pl->rtp) {
                    rtp_send_video(&pl->in->r, data, len, tc ? tc : now_100ns());
                    e->keys += key;
                } else if (pl->srt) {
                    srt_send_video(&pl->in->s, data, len, tc ? tc : now_100ns(), key);
                    e->keys += key;
                } else {
                    hwenc_send(e, pl->send, &pl->m, data, len, key, tc ? tc : NDIlib_send_timecode_synthesize);
                }
                pthread_mutex_lock(&pl->mu);
                pl->ndi_sent++;
                pthread_mutex_unlock(&pl->mu);
                got = true;
            }
            xioctl(e->fd, VIDIOC_QBUF, &b);
        }
        int left = (int)(until - now_ms());
        if (got || left <= 0)
            return;
        struct pollfd pf = { .fd = e->fd, .events = POLLIN };
        poll(&pf, 1, left);
    }
}

static void *ndi_thread(void *arg)
{
    struct pipeline *pl = arg;
    /* Below the decoders: this device's own display comes first. The SDK's encoder threads are
     * started by the first send and inherit this. */
    setpriority(PRIO_PROCESS, (id_t)gettid(), 10);
    int prev = -1; /* NDI keeps using an async frame until the next send (or a NULL flush) returns */
    pthread_mutex_lock(&pl->mu);
    for (;;) {
        if (pl->hx) {
            bool queued = false;
            for (int k = 0; k < NOUT; k++)
                queued |= pl->ostate[k] == O_SENDING;
            while (!pl->quit && pl->pending < 0 && !queued)
                pthread_cond_wait(&pl->cv, &pl->mu);
            if (pl->quit)
                break;
            int i = pl->pending;
            pl->pending = -1;
            if (i >= 0)
                pl->ostate[i] = O_SENDING;
            pthread_mutex_unlock(&pl->mu);
            hx_step(pl, i, i >= 0 ? 0 : 3);
            pthread_mutex_lock(&pl->mu);
            pthread_cond_broadcast(&pl->cv);
            continue;
        }
        while (!pl->quit && pl->pending < 0 && !(pl->flush && prev >= 0))
            pthread_cond_wait(&pl->cv, &pl->mu);
        if (pl->quit)
            break;
        int i = pl->pending;
        NDIlib_video_frame_v2_t f = pl->pframe;
        pl->pending = -1;
        if (i >= 0)
            pl->ostate[i] = O_SENDING;
        pthread_mutex_unlock(&pl->mu);
        uint64_t t = now_ms();
        ndi->send_send_video_async_v2(pl->send, i >= 0 ? &f : NULL); /* waits for the previous frame */
        if (now_ms() - t > 200)
            LOG("NDI took %llu ms to take a video frame", (unsigned long long)(now_ms() - t));
        pthread_mutex_lock(&pl->mu);
        if (prev >= 0)
            pl->ostate[prev] = O_FREE;
        prev = i;
        if (i >= 0)
            pl->ndi_sent++;
        pthread_cond_broadcast(&pl->cv);
    }
    pthread_mutex_unlock(&pl->mu);
    return NULL;
}

/* Wait until NDI has let go of every frame (before sending to it directly, or stopping). */
static void ndi_wait_idle(struct pipeline *pl)
{
    pthread_mutex_lock(&pl->mu);
    pl->flush = true;
    pthread_cond_broadcast(&pl->cv);
    for (;;) {
        bool busy = pl->pending >= 0;
        for (int i = 0; i < NOUT; i++)
            busy |= pl->ostate[i] == O_SENDING;
        if (!busy)
            break;
        pthread_cond_wait(&pl->cv, &pl->mu);
    }
    pl->flush = false;
    pthread_mutex_unlock(&pl->mu);
}

static void worker_frame(struct worker *wk)
{
    struct pipeline *pl = wk->pl;
    struct input *in = pl->in;
    const struct mode *m = &pl->m;
    struct local_feed *lf = in->lf;
    uint64_t s = wk->seq;
    int slot = (int)(s % LOCAL_SLOTS);
    size_t ysz = (size_t)m->w * m->h;
    bool local = wk->local && lf && in->lf_frames;
    uint8_t *base = local ? in->lf_frames + (size_t)slot * lf->slot_size : wk->planes;
    wk->dark = false;
    if (local)
        STORE(lf->slot_seq[slot], 0);
    int ob = -1; /* NDI buffer for this frame: only if one is free (else the encoder is busy: skip NDI) */
    if (wk->ndi) {
        pthread_mutex_lock(&pl->mu);
        for (int i = 0; i < NOUT && ob < 0; i++)
            if (pl->ostate[i] == O_FREE) {
                ob = i;
                pl->ostate[i] = O_FILLING;
            }
        pthread_mutex_unlock(&pl->mu);
    }
    bool to_ndi = ob >= 0;
    uint8_t *out = to_ndi ? pl->out[ob] : NULL;
    NDIlib_video_frame_v2_t f = {
        .xres = m->w, .yres = m->h,
        .frame_rate_N = (int)m->den, .frame_rate_D = (int)m->num,
        .picture_aspect_ratio = 0,
        .frame_format_type = NDIlib_frame_format_type_progressive,
        /* When the picture was captured: receivers pace frames by it (PINDI_OUT<n>_PACING). */
        .timecode = wk->time ? wk->time : NDIlib_send_timecode_synthesize,
        .p_data = out,
    };
    bool ok = true, planar_ok = true;
    uint32_t fmt = LOCAL_422P;
    bool skip = !to_ndi && !local;
    if (skip) {
        /* Nobody takes this frame: the encoder is still busy (the Pi 3/4 H.264 encoder manages about
         * 1080p30, so half of a 60 fps card's frames) and there is no local display. Don't decode it. */
    } else if (m->fourcc == V4L2_PIX_FMT_MJPEG) {
        int jw, jh, sub, cs, rc = -1;
        if (!wk->len || tjDecompressHeader3(wk->tj, wk->src, wk->len, &jw, &jh, &sub, &cs) || jw != m->w ||
            jh != m->h) {
            ok = false;
        } else if (sub == TJSAMP_420 || sub == TJSAMP_422) {
            int ch = sub == TJSAMP_420 ? m->h / 2 : m->h;
            unsigned char *dst[3] = { base, base + ysz, base + ysz + (size_t)m->w / 2 * ch };
            int strides[3] = { m->w, m->w / 2, m->w / 2 };
            rc = tjDecompressToYUVPlanes(wk->tj, wk->src, wk->len, dst, m->w, strides, m->h, TJFLAG_FASTDCT);
            fmt = sub == TJSAMP_420 ? LOCAL_420P : LOCAL_422P;
            if (!in->video_range) {
                full_to_video(dst[0], ysz, false);
                full_to_video(dst[1], (size_t)m->w / 2 * ch * 2, true); /* U and V are adjacent */
            }
        } else if (to_ndi && !pl->hx) {
            /* Other subsampling: RGB for NDI only (not for the H.264 encoder). */
            planar_ok = false;
            rc = tjDecompress2(wk->tj, wk->src, wk->len, out, m->w, m->w * 4, m->h, TJPF_BGRX, TJFLAG_FASTDCT);
            f.FourCC = NDIlib_FourCC_video_type_BGRX;
            f.line_stride_in_bytes = m->w * 4;
        } else {
            planar_ok = false;
            rc = 0; /* nothing to do with this one */
        }
        if (ok) {
            /* Warnings usually mean a truncated frame (grey/garbage band = visible flicker), so those
             * are dropped - unless the device warns on nearly every frame: then it's a harmless quirk. */
            bool warn = rc && tjGetErrorCode(wk->tj) == TJERR_WARNING;
            ok = !rc || (warn && wk->warn_rate > 0.8);
            wk->warn_rate = wk->warn_rate * 0.95 + (warn ? 0.05 : 0);
        }
        if (ok && planar_ok) {
            /* How dark the picture is (a sparse sample of the luma): a black picture together with silence
             * is a break in the programme (e.g. before an ad), not an audio fault - logged at publishing. */
            unsigned sum = 0, mx = 0, n = 0;
            for (int row = m->h / 32; row < m->h; row += m->h / 16)
                for (int x = 16; x < m->w; x += 64, n++) {
                    unsigned l = base[(size_t)row * m->w + x];
                    sum += l;
                    mx = l > mx ? l : mx;
                }
            wk->dark = n && sum / n < 24 && mx < 48;
        }
        if (ok && planar_ok && to_ndi) {
            if (fmt == LOCAL_420P && !pl->hx) { /* NDI's I420 is this very layout */
                memcpy(out, base, ysz * 3 / 2);
                f.FourCC = NDIlib_FourCC_video_type_I420;
                f.line_stride_in_bytes = m->w;
            } else { /* UYVY (for SpeedHQ, or the H.264 encoder's input) */
                bool c420 = fmt == LOCAL_420P;
                const uint8_t *y = base, *u = base + ysz, *v = base + ysz + ysz / (c420 ? 4 : 2);
                for (int row = 0; row < m->h; row++) {
                    size_t cr = (size_t)(c420 ? row / 2 : row) * m->w / 2;
                    yuv422p_to_uyvy(y + (size_t)row * m->w, u + cr, v + cr, out + (size_t)row * pl->out_stride, m->w);
                }
                f.FourCC = NDIlib_FourCC_video_type_UYVY;
                f.line_stride_in_bytes = pl->out_stride;
            }
        }
    } else if (wk->len < (size_t)m->bpl * (m->h - 1) + (size_t)m->w * 2) {
        ok = false; /* a short (damaged) raw frame: its rows would be read past the end of the data */
    } else {
        bool uyvy = m->fourcc == V4L2_PIX_FMT_UYVY;
        if (local)
            for (int row = 0; row < m->h; row++)
                packed_to_422p(wk->src + (size_t)row * m->bpl, base + (size_t)row * m->w,
                               base + ysz + (size_t)row * m->w / 2, base + ysz * 3 / 2 + (size_t)row * m->w / 2,
                               m->w, uyvy);
        if (to_ndi)
            for (int row = 0; row < m->h; row++) {
                const uint8_t *src = wk->src + (size_t)row * m->bpl;
                uint8_t *dst = out + (size_t)row * pl->out_stride;
                if (uyvy)
                    memcpy(dst, src, (size_t)m->w * 2);
                else
                    yuyv_to_uyvy(src, dst, (size_t)m->w * 2);
            }
        f.FourCC = NDIlib_FourCC_video_type_UYVY;
        f.line_stride_in_bytes = pl->out_stride;
    }

    /* Publish in order. */
    pthread_mutex_lock(&pl->mu);
    while (pl->next != s && !pl->quit)
        pthread_cond_wait(&pl->cv, &pl->mu);
    if (ok && planar_ok) {
        if (wk->dark && !pl->black_since) {
            pl->black_since = wk->time;
        } else if (!wk->dark && pl->black_since) {
            double secs = (wk->time - pl->black_since) / 1e7;
            if (secs >= 0.5)
                LOG("input %d: video: picture black for %.2f s", in->n, secs);
            pl->black_since = 0;
        }
    }
    if (ok) {
        if (local && planar_ok) {
            lf->slot_time[slot] = wk->time;
            lf->fmt = fmt;
            STORE(lf->slot_seq[slot], s);
            STORE(lf->seq, s);
        }
        if (to_ndi) { /* newest for the encoder; an older one still waiting is dropped */
            if (pl->pending >= 0)
                pl->ostate[pl->pending] = O_FREE;
            pl->pending = ob;
            pl->ostate[ob] = O_PENDING;
            pl->out_time[ob] = wk->time;
            pl->pframe = f;
            ob = -1;
        }
        pl->frames += !skip;
    } else {
        pl->damaged++;
    }
    if (ob >= 0)
        pl->ostate[ob] = O_FREE;
    pl->next = s + 1;
    wk->busy = false;
    pthread_cond_broadcast(&pl->cv);
    pthread_mutex_unlock(&pl->mu);
}

static void *worker_thread(void *arg)
{
    struct worker *wk = arg;
    struct pipeline *pl = wk->pl;
    for (;;) {
        pthread_mutex_lock(&pl->mu);
        while (!pl->quit && !wk->busy)
            pthread_cond_wait(&pl->cv, &pl->mu);
        bool quit = pl->quit;
        pthread_mutex_unlock(&pl->mu);
        if (quit)
            break;
        worker_frame(wk);
    }
    return NULL;
}

/* Wait until every frame handed out so far is published (NDI may then be flushed or sent to directly). */
static void pipeline_drain(struct pipeline *pl, uint64_t next_seq)
{
    pthread_mutex_lock(&pl->mu);
    while (pl->next < next_seq)
        pthread_cond_wait(&pl->cv, &pl->mu);
    pthread_mutex_unlock(&pl->mu);
}

/* No picture: black on this device's display too (planar 4:2:2), published as frame seq. */
static void local_black(struct input *in, struct pipeline *pl, uint64_t seq)
{
    struct local_feed *lf = in->lf;
    int slot = (int)(seq % LOCAL_SLOTS);
    size_t ysz = (size_t)in->w * in->h;
    uint8_t *base = in->lf_frames + (size_t)slot * lf->slot_size;
    STORE(lf->slot_seq[slot], 0);
    memset(base, 16, ysz);
    memset(base + ysz, 128, ysz);
    lf->fmt = LOCAL_422P;
    lf->slot_time[slot] = capture_time(&(struct timeval){ 0 });
    STORE(lf->slot_seq[slot], seq);
    STORE(lf->seq, seq);
    pthread_mutex_lock(&pl->mu);
    pl->next = seq + 1;
    pthread_mutex_unlock(&pl->mu);
}

static void run_device(struct input *in, NDIlib_send_instance_t send)
{
    int fd = open(in->device, O_RDWR | O_NONBLOCK | O_CLOEXEC);
    if (fd < 0) {
        status(in, "waiting for device", NULL, 0, 0, 0);
        return;
    }
    struct capbuf cap[NCAP] = { 0 };
    int ncap = 0, started = 0;
    struct pipeline pl = { .in = in, .send = send, .next = 1, .pending = -1, .enc = { .fd = -1 } };
    bool ndi_started = false;
    pthread_mutex_init(&pl.mu, NULL);
    pthread_cond_init(&pl.cv, NULL);
    struct mode m;
    char mode_desc[64] = "";
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    bool streaming = false;
    uint64_t seq = 1; /* next frame number to hand out */
    struct csi csi = { .sd = -1 };
    char bus[64];

    if (csi_receiver(fd, bus, sizeof bus)) {
        const char *why = csi_setup(in->n, fd, bus, &csi, &m);
        if (strcmp(why ? why : "", in->csi_why)) { /* each change once, not every retry */
            if (why)
                LOG("input %d: %s", in->n, why);
            snprintf(in->csi_why, sizeof in->csi_why, "%s", why ? why : "");
        }
        if (why) {
            status(in, why, NULL, 0, 0, 0);
            goto out;
        }
        snprintf(mode_desc, sizeof mode_desc, "HDMI %dx%dp%.4g", m.w, m.h, (double)m.den / m.num);
    } else {
        if (!choose_mode(fd, in->mode_s, &m)) {
            status(in, "no usable video format", NULL, 0, 0, 0);
            goto out;
        }
        struct v4l2_format fmt = { .type = V4L2_BUF_TYPE_VIDEO_CAPTURE };
        fmt.fmt.pix.width = m.w;
        fmt.fmt.pix.height = m.h;
        fmt.fmt.pix.pixelformat = m.fourcc;
        fmt.fmt.pix.field = V4L2_FIELD_NONE;
        if (xioctl(fd, VIDIOC_S_FMT, &fmt)) {
            status(in, "cannot set format", NULL, 0, 0, 0);
            goto out;
        }
        m.w = fmt.fmt.pix.width;
        m.h = fmt.fmt.pix.height;
        m.bpl = fmt.fmt.pix.bytesperline;
        struct v4l2_streamparm parm = { .type = V4L2_BUF_TYPE_VIDEO_CAPTURE };
        parm.parm.capture.timeperframe.numerator = m.num;
        parm.parm.capture.timeperframe.denominator = m.den;
        if (!xioctl(fd, VIDIOC_S_PARM, &parm) && parm.parm.capture.timeperframe.numerator) {
            m.num = parm.parm.capture.timeperframe.numerator;
            m.den = parm.parm.capture.timeperframe.denominator;
        }
        snprintf(mode_desc, sizeof mode_desc, "%.4s %dx%d@%.4g", (const char *)&m.fourcc, m.w, m.h,
                 (double)m.den / m.num);
    }
    if (m.bpl < m.w * 2)
        m.bpl = m.w * 2;
    if (in->w != m.w || in->h != m.h) {
        free(in->black); /* the black frame is made at the new size when needed */
        in->black = NULL;
    }
    in->w = m.w;
    in->h = m.h;
    in->fps_n = (int)m.den;
    in->fps_d = (int)m.num;
    if (!local_setup(in, m.w, m.h, LOCAL_422P, (int)m.den, (int)m.num))
        LOG("input %d: no shared memory for this device's own display: %s", in->n, strerror(errno));

    struct v4l2_requestbuffers req = { .count = NCAP, .type = V4L2_BUF_TYPE_VIDEO_CAPTURE, .memory = V4L2_MEMORY_MMAP };
    if (xioctl(fd, VIDIOC_REQBUFS, &req) || req.count < 2) {
        status(in, "cannot allocate capture buffers", mode_desc, 0, 0, 0);
        goto out;
    }
    for (ncap = 0; ncap < (int)req.count && ncap < NCAP; ncap++) {
        struct v4l2_buffer b = { .type = V4L2_BUF_TYPE_VIDEO_CAPTURE, .memory = V4L2_MEMORY_MMAP, .index = ncap };
        if (xioctl(fd, VIDIOC_QUERYBUF, &b))
            break;
        cap[ncap].len = b.length;
        cap[ncap].start = mmap(NULL, b.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, b.m.offset);
        if (cap[ncap].start == MAP_FAILED) {
            cap[ncap].start = NULL;
            break;
        }
        xioctl(fd, VIDIOC_QBUF, &b);
    }
    if (ncap < 2 || xioctl(fd, VIDIOC_STREAMON, &type)) {
        status(in, "cannot start capture", mode_desc, 0, 0, 0);
        goto out;
    }
    streaming = true;

    pl.m = m;
    const char *why = NULL; /* why an RTP or SRT input sends nothing */
    if (in->rtp || in->srt) {
        const char *what = in->rtp ? "H.264 RTP" : "H.264 SRT";
        if (in->rtp && in->r.fps != (double)m.den / m.num) { /* the SDP gives the frame rate: write it again */
            in->r.fps = (double)m.den / m.num;
            in->r.sps_len = 0;
        }
        if (!*enc_dev)
            why = in->rtp ? "not sent: H.264 RTP needs the hardware H.264 encoder of a Pi 3 or 4"
                          : "not sent: H.264 SRT needs the hardware H.264 encoder of a Pi 3 or 4";
        else if (in->rtp && in->r.fd < 0)
            why = "not sent: the RTP destination is not a valid address and port";
        else if (in->srt && !in->s.thr_on)
            why = in->s.state;
        else if (!(pl.hx = hwenc_open(&pl.enc, &m, NULL, what, in->n, in->kbps)))
            why = "not sent: the hardware H.264 encoder failed (see the log)";
        pl.rtp = pl.hx && in->rtp;
        pl.srt = pl.hx && in->srt;
        if (why)
            LOG("input %d: %s", in->n, why);
    } else if (in->hx && !*enc_dev) {
        LOG("input %d: NDI|HX needs the Pi's hardware H.264 encoder (none here) - sending Full NDI", in->n);
    } else if (in->hx && !ndi_target_bit_rate) {
        LOG("input %d: NDI|HX needs the NDI Advanced SDK - sending Full NDI", in->n);
    } else if (in->hx) {
        pl.hx = hwenc_open(&pl.enc, &m, send, "NDI|HX", in->n, 0);
    }
    pl.out_stride = pl.hx ? pl.enc.bpl : m.w * 2;
    pl.nw = m.fourcc == V4L2_PIX_FMT_MJPEG && m.w * m.h >= 1280 * 720 ? (pl.hx ? 3 : 2) : 1;
    for (int i = 0; i < NOUT; i++)
        if (!(pl.out[i] = pl.hx ? pl.enc.in_map[i] : malloc((size_t)m.w * m.h * 4)))
            goto out;
    if (pthread_create(&pl.ndi_thr, NULL, ndi_thread, &pl))
        goto out;
    ndi_started = true;
    for (int i = 0; i < pl.nw; i++) {
        struct worker *wk = &pl.w[i];
        wk->pl = &pl;
        wk->planes = malloc((size_t)m.w * m.h * 2);
        if (m.fourcc == V4L2_PIX_FMT_MJPEG)
            wk->tj = tjInitDecompress();
        if (!wk->planes || (m.fourcc == V4L2_PIX_FMT_MJPEG && !wk->tj) ||
            pthread_create(&wk->thr, NULL, worker_thread, wk))
            goto out;
        started++;
    }
    if (in->rtp)
        LOG("input %d: %s from %s as H.264 RTP to %s (%d decoding thread%s)", in->n, mode_desc, in->device,
            in->rtp_dest, pl.nw, pl.nw == 1 ? "" : "s");
    else if (in->srt)
        LOG("input %d: %s from %s as H.264 SRT, %s (%d decoding thread%s)", in->n, mode_desc, in->device,
            in->s.shown, pl.nw, pl.nw == 1 ? "" : "s");
    else
        LOG("input %d: %s from %s as NDI%s \"%s\" (%d decoding thread%s)", in->n, mode_desc, in->device,
            pl.hx ? "|HX" : "", in->name, pl.nw, pl.nw == 1 ? "" : "s");
    status(in, "starting", mode_desc, 0, 0, 0);

    int timeouts = 0, cam = 0, log_cam = 0, log_sent = 0, log_ndi = 0, log_damaged = 0, conns = 0;
    bool idle = false, local = false;
    uint64_t t0 = now_ms(), t_log = t0, conns_checked = 0;
    while (running) {
        if (in->lf)
            STORE(in->lf->writer_ms, now_ms());
        /* A CSI-2 bridge also signals source changes (POLLPRI), on its own node or on the video node. */
        struct pollfd p[2] = { { .fd = fd, .events = POLLIN | (csi.sd == fd ? POLLPRI : 0) },
                               { .fd = csi.sd, .events = POLLPRI } };
        int np = csi.sd >= 0 && csi.sd != fd ? 2 : 1;
        int r = poll(p, np, 1000);
        if (r > 0 && csi.sd >= 0 && (p[np - 1].revents & POLLPRI) && csi_changed(&csi)) {
            LOG("input %d: the HDMI source changed", in->n);
            break; /* set up again for the new mode */
        }
        if (r == 0) {
            status(in, "no signal", mode_desc, 0, 0, 0);
            pipeline_drain(&pl, seq);
            ndi_wait_idle(&pl); /* send_black sends directly */
            send_black(in, send);
            if (local_wanted(in->lf) && in->lf_frames)
                local_black(in, &pl, seq++);
            if (++timeouts >= 5)
                break; /* re-open the device */
            continue;
        }
        if (r < 0 || (p[0].revents & (POLLERR | POLLHUP)))
            break;
        if (!(p[0].revents & POLLIN))
            continue;
        struct v4l2_buffer b = { .type = V4L2_BUF_TYPE_VIDEO_CAPTURE, .memory = V4L2_MEMORY_MMAP };
        if (xioctl(fd, VIDIOC_DQBUF, &b)) {
            if (errno == EAGAIN)
                continue;
            break;
        }
        timeouts = 0;
        cam++;
        /* The SDK encodes (full and preview stream) even with nobody connected: with no receivers,
         * skip decoding and sending altogether. That's most of the CPU (and heat) on a Pi 3. */
        uint64_t tnow = now_ms();
        if (send && tnow - conns_checked >= 250) {
            conns = ndi->send_get_no_connections(send, 0);
            conns_checked = tnow;
        }
        local = local_wanted(in->lf);
        /* RTP is sent whether or not anyone receives it; SRT knows its receivers. */
        bool sending = conns > 0 || pl.rtp || (pl.srt && srt_peers(&in->s));
        if (!sending && !local) {
            xioctl(fd, VIDIOC_QBUF, &b);
            idle = true;
        } else {
            if (idle)
                LOG("input %d: %s, sending", in->n, sending ? "receiver connected" : "shown on this device");
            idle = false;
            struct worker *wk = &pl.w[seq % pl.nw];
            pthread_mutex_lock(&pl.mu);
            while (wk->busy)
                pthread_cond_wait(&pl.cv, &pl.mu);
            pthread_mutex_unlock(&pl.mu);
            if (wk->cap < b.bytesused) {
                free(wk->src);
                wk->cap = b.bytesused;
                wk->src = malloc(wk->cap);
                if (!wk->src)
                    wk->cap = 0;
            }
            wk->len = wk->src ? b.bytesused : 0;
            if (wk->len)
                memcpy(wk->src, cap[b.index].start, wk->len);
            xioctl(fd, VIDIOC_QBUF, &b);
            wk->time = capture_time(&b.timestamp);
            wk->ndi = sending;
            wk->local = local;
            pthread_mutex_lock(&pl.mu);
            wk->seq = seq++;
            wk->busy = true;
            pthread_cond_broadcast(&pl.cv);
            pthread_mutex_unlock(&pl.mu);
        }

        uint64_t now = now_ms();
        if (now - t0 >= 2000) {
            char st[160];
            NDIlib_tally_t tally = { 0 };
            pthread_mutex_lock(&pl.mu);
            int frames = pl.frames, damaged = pl.damaged, sent = pl.ndi_sent;
            pl.frames = pl.damaged = pl.ndi_sent = 0;
            pthread_mutex_unlock(&pl.mu);
            double secs = (now - t0) / 1000.0;
            if (conns)
                ndi->send_get_tally(send, &tally, 0); /* a switcher (vMix, TriCaster...) has it on program/preview */
            if (why) {
                snprintf(st, sizeof st, "%s%s", why, local ? " (shown on this device)" : "");
            } else if (pl.rtp) {
                snprintf(st, sizeof st, "sending H.264 RTP to %s (%.0f fps)%s", in->rtp_dest, sent / secs,
                         local ? ", also shown on this device" : "");
            } else if (pl.srt) {
                char ss[128];
                srt_status(&in->s, ss, sizeof ss, sent / secs);
                snprintf(st, sizeof st, "%s%s", ss, local ? ", also shown on this device" : "");
            }
            else if (in->hx && ndi_eval_over)
                snprintf(st, sizeof st, "NDI|HX video stopped: 30-minute limit of the NDI Advanced SDK evaluation "
                                        "(sound goes on)");
            else if (conns)
                snprintf(st, sizeof st, "sending (%d receiver%s, %.0f fps over NDI)%s%s", conns, conns == 1 ? "" : "s",
                         sent / secs, local ? ", also shown on this device" : "",
                         tally.on_program ? " - ON AIR" : tally.on_preview ? " - on preview" : "");
            else if (local)
                snprintf(st, sizeof st, "shown on this device (no NDI receivers)");
            else
                snprintf(st, sizeof st, "ready (no receivers)");
            status(in, st, mode_desc, frames / secs, cam / secs, damaged);
            log_cam += cam;
            log_sent += frames;
            log_ndi += sent;
            log_damaged += damaged;
            cam = 0;
            t0 = now;
        }
        if (now - t_log >= 60000) {
            double secs = (now - t_log) / 1000.0;
            LOG("input %d: last %.0f s: camera %.1f fps, decoded %.1f fps, %s %.1f fps, %d damaged frame%s skipped",
                in->n, secs, log_cam / secs, log_sent / secs, pl.rtp ? "RTP" : pl.srt ? "SRT" : "NDI", log_ndi / secs,
                log_damaged, log_damaged == 1 ? "" : "s");
            if (pl.rtp) {
                LOG("input %d: H.264 RTP: %d keyframes", in->n, pl.enc.keys);
                pl.enc.keys = 0;
            } else if (pl.srt) {
                pthread_mutex_lock(&in->s.mu);
                uint64_t dropped = in->s.dropped;
                in->s.dropped = 0;
                pthread_mutex_unlock(&in->s.mu);
                LOG("input %d: H.264 SRT: %d keyframes, %d forced for new receivers, %llu messages dropped "
                    "(a receiver's send buffer was full)", in->n, pl.enc.keys, pl.enc.forced,
                    (unsigned long long)dropped);
                pl.enc.keys = pl.enc.forced = 0;
            } else if (pl.hx) {
                LOG("input %d: NDI|HX: %d keyframes, %d forced for new receivers", in->n, pl.enc.keys, pl.enc.forced);
                pl.enc.keys = pl.enc.forced = 0;
            }
            log_cam = log_sent = log_ndi = log_damaged = 0;
            t_log = now;
        }
    }
out:
    if (started)
        pipeline_drain(&pl, seq);
    if (ndi_started)
        ndi_wait_idle(&pl); /* NDI is done with our buffers after this */
    pthread_mutex_lock(&pl.mu);
    pl.quit = true;
    pthread_cond_broadcast(&pl.cv);
    pthread_mutex_unlock(&pl.mu);
    for (int i = 0; i < started; i++)
        pthread_join(pl.w[i].thr, NULL);
    if (ndi_started)
        pthread_join(pl.ndi_thr, NULL);
    if (pl.hx)
        memset(pl.out, 0, sizeof pl.out); /* the encoder's buffers */
    hwenc_close(&pl.enc);
    for (int i = 0; i < MAX_WORKERS; i++) {
        if (pl.w[i].tj)
            tjDestroy(pl.w[i].tj);
        free(pl.w[i].src);
        free(pl.w[i].planes);
    }
    for (int i = 0; i < NOUT; i++)
        free(pl.out[i]);
    if (streaming)
        xioctl(fd, VIDIOC_STREAMOFF, &type);
    for (int i = 0; i < ncap; i++)
        if (cap[i].start)
            munmap(cap[i].start, cap[i].len);
    csi_close(&csi, fd);
    close(fd);
    if (running && streaming) /* else the reason it didn't start stays in the status */
        status(in, "device lost, retrying", mode_desc, 0, 0, 0);
}

/* ------------------------------------------------------------------ audio thread */

/* The ALSA card on the same USB device as the video device, -1 if it has none, -2 if the device is gone. */
static int audio_card_for(const char *video_dev)
{
    char real[PATH_MAX], sys[PATH_MAX + 64], usb[PATH_MAX];
    if (!realpath(video_dev, real) || strncmp(real, "/dev/", 5))
        return -2;
    snprintf(sys, sizeof sys, "/sys/class/video4linux/%s/device", real + 5);
    if (!realpath(sys, usb) || !strstr(usb, "/usb"))
        return -1; /* not on USB (a CSI-2 bridge): a sound card next to it would be some other device */
    char *slash = strrchr(usb, '/'); /* .../1-1.4/1-1.4:1.0 -> .../1-1.4 (the USB device) */
    if (!slash)
        return -1;
    *slash = 0;
    for (int card = 0; card < 32; card++) {
        char p[64], dev[PATH_MAX];
        snprintf(p, sizeof p, "/sys/class/sound/card%d/device", card);
        if (!realpath(p, dev))
            continue;
        size_t n = strlen(usb);
        if (!strncmp(dev, usb, n) && dev[n] == '/')
            return card;
    }
    return -1;
}

#define AUDIO_PERIOD 960 /* 20 ms */

/* 48 kHz stereo, 20 ms periods, 500 ms buffer. Set explicitly: with snd_pcm_set_params()'s choice
 * (25 ms periods) the MS2109 capture cards fail every read with an I/O error. The long buffer rides
 * out the thread being held off by video decoding instead of overrunning (lost audio). */
static int audio_params(snd_pcm_t *pcm)
{
    snd_pcm_hw_params_t *hw;
    snd_pcm_hw_params_alloca(&hw);
    unsigned rate = AUDIO_RATE;
    snd_pcm_uframes_t period = AUDIO_PERIOD, buffer = AUDIO_PERIOD * 25;
    int err = snd_pcm_hw_params_any(pcm, hw);
    if (!err)
        err = snd_pcm_hw_params_set_access(pcm, hw, SND_PCM_ACCESS_RW_INTERLEAVED);
    if (!err)
        err = snd_pcm_hw_params_set_format(pcm, hw, SND_PCM_FORMAT_S16_LE);
    if (!err)
        err = snd_pcm_hw_params_set_channels(pcm, hw, 2);
    if (!err)
        err = snd_pcm_hw_params_set_rate_near(pcm, hw, &rate, NULL);
    if (!err)
        err = snd_pcm_hw_params_set_period_size_near(pcm, hw, &period, NULL);
    if (!err)
        err = snd_pcm_hw_params_set_buffer_size_near(pcm, hw, &buffer);
    if (!err)
        err = snd_pcm_hw_params(pcm, hw);
    return err;
}

/* One line per second of audio: wall-clock time, peak and RMS in dBFS. Kept to about 10000 lines. */
static void levels_append(struct input *in, int peak, double rms)
{
    char path[64];
    snprintf(path, sizeof path, RUN_DIR "/in%d.levels", in->n);
    struct stat sb;
    if (!stat(path, &sb) && sb.st_size > 512 * 1024) {
        char old[72];
        snprintf(old, sizeof old, "%s.old", path);
        rename(path, old);
    }
    FILE *f = fopen(path, "a");
    if (!f)
        return;
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    struct tm tm;
    localtime_r(&ts.tv_sec, &tm);
    fprintf(f, "%02d:%02d:%02d.%03ld peak %6.1f rms %6.1f\n", tm.tm_hour, tm.tm_min, tm.tm_sec,
            ts.tv_nsec / 1000000, peak ? 20 * log10(peak / 32768.0) : -99.0,
            rms > 0 ? 20 * log10(rms / 32768.0) : -99.0);
    fclose(f);
}

/* Hand n frames to this device's own display and to NDI (or RTP, SRT). */
static void audio_out(struct input *in, const int16_t *s, int n)
{
    struct local_feed *lf = in->lf;
    if (local_wanted(lf)) { /* this device's own display */
        uint64_t wr = lf->audio_wr;
        for (int i = 0; i < n; i++) {
            size_t at = (size_t)((wr + (uint64_t)i) % LOCAL_AUDIO) * 2;
            lf->audio[at] = s[i * 2];
            lf->audio[at + 1] = s[i * 2 + 1];
        }
        STORE(lf->audio_wr, wr + (uint64_t)n);
    }
    if (in->rtp) {
        if (in->r.fd >= 0)
            rtp_send_audio(&in->r, s, n);
        return;
    }
    if (in->srt) {
        if (in->s.thr_on && *enc_dev)
            srt_send_audio(&in->s, s, n);
        return;
    }
    /* Nobody listening: drop, as for the video. */
    if (!ndi->send_get_no_connections(in->send, 0))
        return;
    NDIlib_audio_frame_interleaved_16s_t a = {
        .sample_rate = AUDIO_RATE, .no_channels = 2, .no_samples = n,
        .timecode = NDIlib_send_timecode_synthesize, .reference_level = 0, .p_data = (int16_t *)s,
    };
    uint64_t t = now_us();
    ndi->util_send_send_audio_interleaved_16s(in->send, &a);
    t = now_us() - t;
    if (t > 50000)
        LOG("input %d: audio: NDI took %llu ms to take 20 ms of audio", in->n, (unsigned long long)t / 1000);
}

/* Stereo frames s[0..r) resampled to d[0..n) (linear; n is r +- 1 for drift). */
static void audio_stretch(const int16_t *s, int r, int16_t *d, int n)
{
    for (int i = 0; i < n; i++) {
        double pos = (double)i * (r - 1) / (n - 1);
        int j = (int)pos, k = j + 1 < r ? j + 1 : j;
        double f = pos - j;
        for (int c = 0; c < 2; c++)
            d[i * 2 + c] = (int16_t)lrint(s[j * 2 + c] * (1 - f) + s[k * 2 + c] * f);
    }
}

static void *audio_thread(void *arg)
{
    struct input *in = arg;
    int16_t *buf = malloc(AUDIO_PERIOD * 2 * sizeof(int16_t));
    int16_t stretched[(AUDIO_PERIOD + 1) * 2];
    static const int16_t silence[AUDIO_PERIOD * 2];
    /* Ahead of the video decoding: a busy CPU then delays a picture rather than losing audio. */
    struct sched_param sp = { .sched_priority = 10 };
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp))
        LOG("input %d: audio runs without real-time priority", in->n);
    bool logged_none = false;
    while (running && buf) {
        int card = audio_card_for(in->device);
        if (card == -2) {
            set_audio_status(in, "waiting for the device");
            for (int i = 0; i < 20 && running; i++)
                usleep(100000);
            continue;
        }
        if (card < 0) {
            if (!logged_none)
                LOG("input %d: no USB audio on this device, sending video only", in->n);
            logged_none = true;
            set_audio_status(in, "none (the device has no audio input)");
            for (int i = 0; i < 50 && running; i++)
                usleep(100000);
            continue;
        }
        logged_none = false;
        char dev[32], *name = NULL;
        snprintf(dev, sizeof dev, "plughw:%d,0", card);
        snd_card_get_name(card, &name);
        snd_pcm_t *pcm = NULL;
        int err = snd_pcm_open(&pcm, dev, SND_PCM_STREAM_CAPTURE, 0);
        if (!err)
            err = audio_params(pcm);
        if (err) {
            LOG("input %d: cannot capture audio from %s: %s", in->n, dev, snd_strerror(err));
            char st[128];
            snprintf(st, sizeof st, "cannot capture: %s", snd_strerror(err));
            set_audio_status(in, st);
            if (pcm)
                snd_pcm_close(pcm);
            free(name);
            for (int i = 0; i < 50 && running; i++)
                usleep(100000);
            continue;
        }
        LOG("input %d: audio from %s (%s), 48 kHz stereo", in->n, name ? name : "?", dev);
        char src[64];
        snprintf(src, sizeof src, "%s", name ? name : dev);
        char st[256];
        snprintf(st, sizeof st, "48 kHz stereo from %s", src);
        set_audio_status(in, st);
        free(name);

        /* NDI's synthesized timecodes count samples, but the card's audio clock is not the Pi's: a
         * card running a little fast or slow, or audio lost to an overrun, leaves the audio drifting
         * from real time (and the video) until receivers drop it for a few seconds to resync. So the
         * samples sent are kept in step with the Pi's clock: a read stretched by one sample either
         * way for drift, silence for audio that was lost. */
        int warm = 0, xruns = 0, log_xruns = 0;
        uint64_t t_start = 0, t_stat = now_ms(), t_log = t_stat, sent = 0;
        int64_t adjusted = 0, log_lost = 0; /* samples added (+) or removed (-) for drift; filled with silence */
        double base = 0, drift = 0;
        uint64_t quiet = 0; /* frames of silence from the card in a row (logged from 500 ms) */
        int lv_peak = 0, lv_n = 0;
        double lv_sum = 0;
        while (running) {
            snd_pcm_sframes_t r = snd_pcm_readi(pcm, buf, AUDIO_PERIOD);
            if (r == -EPIPE || r == -ESTRPIPE || r == -EINTR) {
                if (r == -EPIPE) {
                    xruns++;
                    log_xruns++;
                }
                if (snd_pcm_recover(pcm, (int)r, 1) < 0)
                    break;
                continue;
            }
            if (r < 0) {
                LOG("input %d: audio read: %s", in->n, snd_strerror((int)r));
                break; /* device gone: look for it again */
            }
            /* The card itself going silent (its HDMI audio dropping) looks like a network dropout at the
             * receivers, so say when it happens (under -60 dBFS for half a second), and keep each second's
             * level in in<n>.levels to look back at. */
            for (snd_pcm_sframes_t i = 0; i < r; i++) {
                int a = abs(buf[i * 2]), b = abs(buf[i * 2 + 1]), m = a > b ? a : b;
                if (m > lv_peak)
                    lv_peak = m;
                lv_sum += (double)buf[i * 2] * buf[i * 2] + (double)buf[i * 2 + 1] * buf[i * 2 + 1];
                if (++lv_n == AUDIO_RATE) {
                    levels_append(in, lv_peak, sqrt(lv_sum / (2.0 * lv_n)));
                    lv_peak = lv_n = 0;
                    lv_sum = 0;
                }
                if (m <= 32) {
                    if (++quiet == AUDIO_RATE / 2)
                        LOG("input %d: audio: the card sends silence", in->n);
                } else {
                    if (quiet >= AUDIO_RATE / 2)
                        LOG("input %d: audio: sound again after %.2f s of silence from the card", in->n,
                            quiet / (double)AUDIO_RATE);
                    quiet = 0;
                }
            }
            uint64_t now = now_us();
            snd_pcm_sframes_t avail = snd_pcm_avail_update(pcm);
            if (avail < 0)
                avail = 0;
            /* Samples captured (this read and those still waiting) ahead of the Pi's clock. */
            double ahead = (double)(sent + (uint64_t)r + (uint64_t)avail) - base -
                           (double)(now - t_start) * AUDIO_RATE / 1e6;
            if (warm < 50) { /* the first second: let the capture settle, then start the clock */
                warm++;
                t_start = now;
                base = (double)(sent + (uint64_t)r + (uint64_t)avail);
                ahead = 0;
            }
            drift += (ahead - drift) * 0.01;
            const int16_t *out = buf;
            int n = (int)r;
            if (!in->audio_sync) {
                /* sent as the card delivers it */
            } else if (ahead < -AUDIO_RATE / 10) { /* over 100 ms lost (an overrun): silence in its place */
                int gap = (int)-ahead;
                log_lost += gap;
                while (gap > 0) {
                    int k = gap < AUDIO_PERIOD ? gap : AUDIO_PERIOD;
                    audio_out(in, silence, k);
                    sent += (uint64_t)k;
                    gap -= k;
                }
                drift = 0;
            } else if (ahead > AUDIO_RATE / 10) { /* card far faster than one sample per read can fix */
                base += ahead;
                drift = 0;
            } else if (n > 2 && (drift > AUDIO_RATE / 200 || drift < -AUDIO_RATE / 200)) { /* 5 ms */
                int m = drift > 0 ? n - 1 : n + 1;
                audio_stretch(buf, n, stretched, m);
                adjusted += m - n;
                out = stretched;
                n = m;
            }
            audio_out(in, out, n);
            sent += (uint64_t)n;

            uint64_t ms = now_ms();
            double secs = warm < 50 ? 0 : (now - t_start) / 1e6;
            double ppm = secs > 10 ? -(double)adjusted / (secs * AUDIO_RATE) * 1e6 : 0;
            if (ms - t_stat >= 2000) {
                if (in->audio_sync)
                    snprintf(st, sizeof st, "48 kHz stereo from %s (clock %+d ppm, %d overrun%s)", src,
                             (int)lrint(ppm), xruns, xruns == 1 ? "" : "s");
                else
                    snprintf(st, sizeof st, "48 kHz stereo from %s (device timing, %d overrun%s)", src, xruns,
                             xruns == 1 ? "" : "s");
                set_audio_status(in, st);
                t_stat = ms;
            }
            if (ms - t_log >= 60000) {
                if (in->audio_sync)
                    LOG("input %d: audio: card clock %+.0f ppm, %d overrun%s, %.2f s of lost audio filled", in->n,
                        ppm, log_xruns, log_xruns == 1 ? "" : "s", log_lost / (double)AUDIO_RATE);
                else
                    LOG("input %d: audio: device timing, %d overrun%s", in->n, log_xruns, log_xruns == 1 ? "" : "s");
                log_xruns = 0;
                log_lost = 0;
                t_log = ms;
            }
        }
        snd_pcm_close(pcm);
        if (running) {
            LOG("input %d: audio device lost", in->n);
            set_audio_status(in, "device lost, retrying");
            usleep(1000000);
        }
    }
    free(buf);
    return NULL;
}

static void *input_thread(void *arg)
{
    struct input *in = arg;
    NDIlib_send_instance_t send = in->send; /* made by main, in input order (none for RTP and SRT) */
    if (!send && !in->rtp && !in->srt) {
        LOG("input %d: cannot create NDI sender \"%s\"", in->n, in->name);
        status(in, "cannot create NDI sender", NULL, 0, 0, 0);
        return NULL;
    }
    in->r.fd = -1;
    if (in->rtp) /* before the audio thread, which sends on it too */
        rtp_open(&in->r, in->n, in->rtp_dest, in->name, in->audio_on, 0);
    if (in->srt)
        srt_open(&in->s, in->n, in->srt_url, in->name, in->audio_on);
    bool athr = in->audio_on && !pthread_create(&in->athr, NULL, audio_thread, in);
    if (!in->audio_on)
        set_audio_status(in, "off");
    while (running) {
        run_device(in, send);
        for (int i = 0; i < 20 && running; i++) {
            if (i % 10 == 0)
                send_black(in, send);
            usleep(100000);
        }
    }
    free(in->black);
    if (athr)
        pthread_join(in->athr, NULL);
    rtp_close(&in->r);
    if (in->srt)
        srt_close(&in->s);
    if (send)
        ndi->send_destroy(send);
    return NULL;
}

#define NDI_EVAL_MSG "designed for development use and will run on a stream for 30 minutes"

static int output_fd = -1; /* the original stderr (the journal) */

static void *output_thread(void *arg)
{
    int fd = (int)(intptr_t)arg, out = output_fd;
    char line[4096];
    size_t n = 0;
    for (;;) {
        ssize_t r = read(fd, line + n, sizeof line - 1 - n);
        if (r <= 0)
            break;
        n += (size_t)r;
        line[n] = 0;
        if (!ndi_eval_over && strstr(line, NDI_EVAL_MSG))
            ndi_eval_over = 1;
        char *nl = strrchr(line, '\n');
        size_t keep = nl ? (size_t)(line + n - (nl + 1)) : (n == sizeof line - 1 ? 0 : n);
        if (write(out, line, n - keep) < 0) { }
        memmove(line, line + n - keep, keep);
        n = keep;
    }
    return NULL;
}

/* Our stdout and stderr (the journal) through a pipe, to notice the NDI library's evaluation message. */
static void watch_output(void)
{
    int p[2];
    pthread_t t;
    int saved = dup(STDERR_FILENO);
    if (saved < 0 || pipe(p))
        return;
    fflush(stdout);
    output_fd = saved; /* output_thread writes the lines on to the original stderr */
    if (dup2(p[1], STDOUT_FILENO) < 0 || dup2(p[1], STDERR_FILENO) < 0 ||
        pthread_create(&t, NULL, output_thread, (void *)(intptr_t)p[0])) {
        dup2(saved, STDOUT_FILENO);
        dup2(saved, STDERR_FILENO);
        return;
    }
    pthread_detach(t);
    close(p[1]);
}

int main(void)
{
    setvbuf(stdout, NULL, _IOLBF, 0);
    watch_output();
    signal(SIGTERM, on_signal);
    signal(SIGINT, on_signal);
    read_serial();
    mkdir(RUN_DIR, 0755);

    struct input ins[MAX_IN];
    int nin = 0;
    for (int n = 1; n <= MAX_IN; n++) {
        char path[64];
        snprintf(path, sizeof path, RUN_DIR "/in%d.status", n);
        unlink(path);
        snprintf(path, sizeof path, RUN_DIR "/in%d.sdp", n);
        unlink(path);
        if (strcmp(icfg(n, "ENABLED", "0"), "1") || !*icfg(n, "DEVICE", ""))
            continue;
        struct input *in = &ins[nin++];
        memset(in, 0, sizeof *in);
        in->n = n;
        snprintf(in->device, sizeof in->device, "%s", icfg(n, "DEVICE", ""));
        char def[96];
        if (n == 1)
            snprintf(def, sizeof def, "IN_%s", serial);
        else
            snprintf(def, sizeof def, "IN_%s_%d", serial, n);
        snprintf(in->name, sizeof in->name, "%s", icfg(n, "NAME", def));
        snprintf(in->mode_s, sizeof in->mode_s, "%s", icfg(n, "MODE", ""));
        in->audio_on = strcasecmp(icfg(n, "AUDIO", "auto"), "off") != 0;
        in->audio_sync = strcasecmp(icfg(n, "AUDIO_SYNC", "on"), "off") != 0;
        const char *codec = icfg(n, "CODEC", "speedhq");
        in->hx = !strcasecmp(codec, "hx");
        in->rtp = !strcasecmp(codec, "rtp");
        in->srt = !strcasecmp(codec, "srt");
        snprintf(in->rtp_dest, sizeof in->rtp_dest, "%s", icfg(n, "RTP_DEST", ""));
        snprintf(in->srt_url, sizeof in->srt_url, "%s", icfg(n, "SRT_URL", ""));
        in->kbps = atoi(icfg(n, "KBPS", "0"));
        if (in->kbps < 0)
            in->kbps = 0;
        if (in->kbps > 25000) /* the encoder's limit */
            in->kbps = 25000;
        in->video_range = !strcasecmp(icfg(n, "RANGE", "full"), "video");
        pthread_mutex_init(&in->lock, NULL);
    }
    if (!nin) {
        LOG("no outgoing streams configured");
        return 0;
    }
    bool need_ndi = false;
    for (int i = 0; i < nin; i++)
        need_ndi |= !ins[i].rtp && !ins[i].srt;

    /* The NDI Advanced SDK's library is libndi_advanced.so.6 (its soname). */
    void *lib = dlopen(NDILIB_LIBRARY_NAME, RTLD_NOW | RTLD_LOCAL);
    if (!lib)
        lib = dlopen("libndi_advanced.so.6", RTLD_NOW | RTLD_LOCAL);
    const NDIlib_v6 *(*load)(void) = lib ? (const NDIlib_v6 *(*)(void))dlsym(lib, "NDIlib_v6_load") : NULL;
    ndi = load ? load() : NULL;
    if (ndi && !ndi->initialize())
        ndi = NULL;
    if (!ndi && need_ndi) {
        LOG("cannot load/initialise NDI");
        return 1;
    }
    if (!ndi)
        LOG("NDI not available: sending H.264 RTP and SRT only");
    ndi_target_bit_rate = !lib ? NULL : (int (*)(NDIlib_send_instance_t, const NDIlib_video_frame_v2_t *))dlsym(
        lib, "NDIlib_send_get_target_bit_rate");
    ndi_keyframe_required = !lib ? NULL : (bool (*)(NDIlib_send_instance_t, const NDIlib_video_frame_v2_t *))dlsym(
        lib, "NDIlib_send_is_keyframe_required");
    hwenc_find();
    /* The senders are made one after another in input order, not by the input threads as they start:
     * each takes the next free port, so input 1 gets the lower port every time (receivers that remember
     * a stream's address are less likely to find another input there after a restart). RTP and SRT
     * inputs have none. */
    for (int i = 0; i < nin; i++) {
        NDIlib_send_create_t sc = { .p_ndi_name = ins[i].name, .p_groups = NULL, .clock_video = false,
                                    .clock_audio = false };
        ins[i].send = ins[i].rtp || ins[i].srt ? NULL : ndi->send_create(&sc);
    }
    for (int i = 0; i < nin; i++)
        pthread_create(&ins[i].thr, NULL, input_thread, &ins[i]);
    for (int i = 0; i < nin; i++)
        pthread_join(ins[i].thr, NULL);
    if (ndi)
        ndi->destroy();
    return 0;
}
