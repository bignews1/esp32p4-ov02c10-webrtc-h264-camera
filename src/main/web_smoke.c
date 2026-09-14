/*
 * web_smoke.c — WebRTC 핸드셰이크 smoke test (esp_peer 1.5.4)
 *
 * 목적: 기존 SDP → ICE → DTLS smoke 경로를 유지하면서, 연결 성공 후
 *       OV02C10 1080p(1920x1080@30 2-lane) 또는 native720-HS(1288x728@60 2-lane)
 *       → CSI/ISP YU12 → HW H.264 GOP1/QP30..40 auto-RC 프레임을 PSRAM 3-slot queue에
 *       짧게 복사하고, 기존 smoke_pump가
 *       esp_peer_send_video()로 비동기 전송한다.
 *       카메라/encoder 루프가 SRTP/UDP 송신시간을 기다리지 않도록 분리한다.
 *       /webrtc-smoke는 호환 별칭이고
 *       제품 시험 페이지는 /camera-webrtc 이다.
 *
 * 확정 identity (dependencies.lock):
 *   esp_peer      = 1.5.4  (Espressif Component Registry)
 *   component_hash= 36c98e3420dad4675bbe13431bf4deefd0ba2b82f71dda458cb5471825d053b4
 *   esp_libsrtp   = 1.0.0 (transitive)   ESP-IDF = 5.5.4
 *
 * changelog 확인(누적, 1.5.4 ⊃):
 *   v1.4.2 "Treat DTLS role as client when peer send actpass" 포함
 *          → Browser offer(actpass) 시 esp_peer 가 스스로 DTLS role 을 정한다. 강제하지 않고 관찰만 한다.
 *   v1.5.2 "-0x7080 (mbedTLS < v3.6.6) handshake fail 수정" 포함.
 *
 * API 는 esp_peer 공개 헤더(esp_peer.h / esp_peer_types.h / esp_peer_default.h)에서 확인한 것만 사용.
 * 존재하지 않는 함수/필드는 사용하지 않았다. (예: DTLS role 강제 API 는 존재하지 않으므로 사용 안 함.)
 *
 * 보호: Core1 stepper/ISR 불가침 → pump 태스크는 Core0 에 고정. M32/RTA/SSE/max_open_sockets 미변경.
 * [WEBRTC-2CLIENT-FANOUT-AB-20260830] 카메라/ISP/H.264 encoder는 1개만 유지하고
 *       최대 2개의 독립 WebRTC peer에 동일 H.264 access unit을 fan-out 한다.
 *       모든 esp_peer_* 호출은 s->peer_mtx 로 직렬화(main_loop 와 동시호출 방지).
 *       Browser SDP의 a=candidate 라인은 "candidate:..." 별도 메시지로 ICE agent에 실제 주입.
 *       pump 종료 미확인 상태에서는 close/free 금지(안전한 deferred teardown).
 *       malloc/open 실패는 assert 하지 않고 503/graceful 로 처리.
 * [ANDROID-ICE-ESP-OFFER-AB-20260830] production page는 ESP-offerer signaling을 사용한다.
 *       ESP local host candidate가 offer SDP에 포함된 뒤 Android/PC browser가 answer를 생성한다.
 */

#include <string.h>
#include <strings.h>
#include <stdlib.h>
#include <stdio.h>
#include <ctype.h>
#include <stdint.h>
#include <limits.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_http_server.h"
#include "cJSON.h"

#include "esp_peer.h"
#include "esp_peer_default.h"

#include "web_smoke.h"
#include "camera_csi_diag.h"
#include "esp_h264_enc_single_hw.h"
#include "settings.h"
#include "camera_color_stage2.h"   /* [CAM-COLOR-STAGE2-20260901] */
#include "camera_webrtc_audio.h"  /* [CAM-AUDIO-OPUS24K-V4-20260902] */
#include "bsp_board_extra.h"      /* [CAM-AUDIO-GAIN-V2-INCLUDE-FIX-20260902] mic gain API */
#include "linux/videodev2.h"

/* [WEBRTC-FINAL-CANDIDATE-FC1-20260908]
 * Temporary TX-start, UDP-probe, browser telemetry traffic, and periodic pump-stack logs
 * are not part of production operation.  Core session/ICE/DTLS/camera behavior is unchanged. */

#include <errno.h>
#include "lwip/sockets.h"
#include "lwip/inet.h"

/* Production log quiet switch. Set to 1 temporarily when deep diagnostics are needed. */
#ifndef HA_VERBOSE_DIAG_LOGS
#define HA_VERBOSE_DIAG_LOGS 0
#endif


/* [ISP-LIVE-TUNE-20260829] local hooks implemented in the customized
 * esp_video_isp_pipeline.c. RAM-only; no sensor/AE/AWB/H264 changes. */
extern esp_err_t esp_video_isp_pipeline_live_tune_set(bool enabled,
                                                       int saturation,
                                                       int contrast,
                                                       int gamma_milli,
                                                       int sharpen_pct);
extern void esp_video_isp_pipeline_live_tune_get(bool *enabled,
                                                  int *saturation,
                                                  int *contrast,
                                                  int *gamma_milli,
                                                  int *sharpen_pct,
                                                  bool *controller_ready);

/* [DTLS-HANDSHAKE-WINDOW-C2-OWNER-HARDEN-20260908]
 * The timeout override has one transient DTLS window.  Ownership is bound to
 * the smoke session_id so tearing down viewer #1 cannot disarm viewer #2's
 * in-progress DTLS window. */
extern bool turn_gather_rx_ab_begin_session(uint32_t sid);
extern void turn_gather_rx_ab_restore_steady(uint32_t sid);
extern void turn_gather_rx_ab_disarm(uint32_t sid);

/* Embedded same-origin pages.
 * /webrtc-smoke   : detailed engineering diagnostics (existing page)
 * /camera-webrtc  : production camera UI (CAM-WEB-V1) */
extern const uint8_t s_webrtc_smoke_html_start[] asm("_binary_webrtc_smoke_v3_html_start");
extern const uint8_t s_webrtc_smoke_html_end[]   asm("_binary_webrtc_smoke_v3_html_end");
extern const uint8_t s_camera_live_html_start[]  asm("_binary_camera_live_html_start");
extern const uint8_t s_camera_live_html_end[]    asm("_binary_camera_live_html_end");

static const char *TAG = "web_smoke";
#define SMK(fmt, ...) ESP_LOGI(TAG, "[SMOKE] " fmt, ##__VA_ARGS__)
#define SMKE(fmt, ...) ESP_LOGE(TAG, "[SMOKE] " fmt, ##__VA_ARGS__)
#define SMKW(fmt, ...) ESP_LOGW(TAG, "[SMOKE] " fmt, ##__VA_ARGS__)

#define SMOKE_ESP_PEER_VER   "1.5.4"
#define SMOKE_ESP_PEER_HASH8 "36c98e34"

#define SMOKE_BODY_MAX     16384     /* Chrome SDP 여유 포함. 요청마다 임시 할당 */
#define SMOKE_MAX_CAND     12        /* 합칠 local candidate 최대 개수 */
#define SMOKE_CAND_LEN     512
#define SMOKE_SDP_WAIT_MS  3000      /* local SDP(on_msg) 대기 */
#define SMOKE_GATHER_MS    0         /* A/B: answerer HTTP answer를 즉시 반환; ICE/DTLS가 browser setRemoteDescription보다 먼저 달리는 race 제거 */
#define SMOKE_TURN_RELAY_WAIT_MAX_MS   1200 /* Claude review A/B: remote relay offer에서 local relay publish 여부만 안전하게 대기 */
#define SMOKE_TURN_RELAY_WAIT_POLL_MS  25
/* Live camera sessions have no fixed lifetime. They are reaped on explicit /stop
 * or on a real peer disconnect/failure. This avoids the old 120 s test-only cutoff. */
#define SMOKE_PUMP_CORE    0         /* Core0 고정 (Core1 = stepper) */
#define SMOKE_PUMP_PRIO    5
#define SMOKE_PUMP_STACK   16384  /* TURN relay + DTLS nested path can exceed 10KB transiently; A/B safety margin */
#define SMOKE_CLEANUP_PRIO  4
#define SMOKE_CLEANUP_STACK 3072
#define SMOKE_AGENT_RECV_TIMEOUT_MS 10 /* [DTLS-HANDSHAKE-WINDOW-C1-EXACT-20260907] steady media */   /* production: keep peer main_loop responsive; 100ms caps one-AU-per-loop TX near 10fps */
#define SMOKE_MAX_SESSIONS 2U            /* A/B: PC + smartphone */
#define SMOKE_DTLS_CONNECT_TIMEOUT_MS 3000U /* paired->DTLS/SRTP ready watchdog; LAN success is normally <<1s */
#define SMOKE_CLIENT_TOKEN_HEX_LEN    32U   /* [CAM-INITIAL-START-RECOVERY-V2-TOKEN-20260903] 128-bit per-page token */

/* [TURN-RELAY-AB-20260831]
 * Keep the proven public STUN path for direct P2P, and add our coturn server
 * over UDP on the ESP side so esp_peer can gather a local relay candidate.
 * The browser uses TURN/TLS over TCP/443; both sides then have type:4 relay
 * candidates, which the esp_peer ICE agent can pair when direct host/srflx
 * checks fail.  Keep storage static because esp_peer may consult the server
 * list after smoke_start_locked() returns. */
#define SMOKE_STUN_URL "stun:stun.l.google.com:19302"
#define SMOKE_TURN_URL "turn:turn.example.com:3478?transport=udp"
#define SMOKE_TURN_USER "YOUR_TURN_USER"
#define SMOKE_TURN_PSW "YOUR_TURN_PASSWORD"
static esp_peer_ice_server_cfg_t s_smoke_ice_servers[] = {
    {
        .stun_url = (char *)SMOKE_STUN_URL,
        .user = NULL,
        .psw = NULL,
    },
    {
        .stun_url = (char *)SMOKE_TURN_URL,
        .user = (char *)SMOKE_TURN_USER,
        .psw = (char *)SMOKE_TURN_PSW,
    },
};

/* [ASYNC-H264-TX] Do not create another FreeRTOS task: the existing Core0
 * smoke_pump drains a tiny PSRAM queue after esp_peer_main_loop(). This keeps
 * Core1 untouched and avoids consuming another internal-SRAM task stack. */
#define SMOKE_TX_QUEUE_DEPTH        3U
#define SMOKE_TX_SLOT_INITIAL_BYTES (256U * 1024U)
#define SMOKE_TX_SLOT_GROW_BYTES    (64U * 1024U)
#define SMOKE_TX_SLOT_MAX_BYTES     (2U * 1024U * 1024U)

/* [CAM-AUDIO-OPUS24K-V4-20260902] Small Opus queue per peer. Drop-oldest keeps live audio low-latency. */
#define SMOKE_AUDIO_QUEUE_DEPTH      8U /* [CAM-AUDIO-PUMP-JITTER-FIX-V1-20260902] 160ms Opus jitter cushion */
#define SMOKE_AUDIO_SLOT_BYTES       1536U

typedef struct {
    uint8_t *data;
    size_t   len;
    uint32_t pts_ms;
} smoke_audio_slot_t;

typedef struct {
    uint8_t *data;
    size_t   capacity;
    size_t   len;
    uint32_t pts_ms;
} smoke_tx_slot_t;

typedef struct {
    esp_peer_handle_t peer;
    TaskHandle_t      pump;
    SemaphoreHandle_t peer_mtx;   /* esp_peer_* 직렬화 */
    SemaphoreHandle_t sdp_ready;  /* on_msg 로 local SDP 확보 시 give */
    SemaphoreHandle_t done;       /* pump 종료 통지 */

    /* H.264 producer(camera) -> consumer(smoke_pump) SPSC queue.
     * Slot payloads live only in PSRAM; tx_lock protects ring metadata only. */
    SemaphoreHandle_t tx_lock;
    smoke_tx_slot_t   tx_slots[SMOKE_TX_QUEUE_DEPTH];
    uint32_t          tx_head;
    uint32_t          tx_tail;
    uint32_t          tx_count;
    uint32_t          tx_high_water;
    uint32_t          tx_enqueued;
    uint32_t          tx_queue_drops;
    uint32_t          tx_oversize_drops;
    uint32_t          tx_copy_us_last;
    uint32_t          tx_copy_us_max;
    uint32_t          tx_send_us_last;
    uint32_t          tx_send_us_max;

    /* [CAM-1080-ADAPT-GOP-R2-20260912] Per-viewer adaptive network telemetry. Session structs live in
     * PSRAM, so these counters consume no scarce INTERNAL DMA blocks. */
    uint32_t          adapt_tx_hwm_window;
    uint32_t          adapt_tx_drop_prev;
    uint32_t          adapt_tx_drop_last;
    uint32_t          adapt_browser_fps_x100;
    uint32_t          adapt_loss_bp;
    uint32_t          adapt_loss_ema_bp;
    uint32_t          adapt_jitter_ms;
    uint32_t          adapt_jitter_ema_ms;
    uint32_t          adapt_avail_kbps;
    uint32_t          adapt_rx_kbps;
    uint32_t          adapt_rx_ema_kbps;
    uint32_t          adapt_send_ema_us;
    uint8_t           adapt_bad_hist;
    uint8_t           adapt_good_streak;
    bool              adapt_stats_seen;
    int64_t           adapt_stats_ms;
    int64_t           adapt_last_log_ms;

    SemaphoreHandle_t audio_lock;
    smoke_audio_slot_t audio_slots[SMOKE_AUDIO_QUEUE_DEPTH];
    uint32_t          audio_head;
    uint32_t          audio_tail;
    uint32_t          audio_count;
    uint32_t          audio_high_water;
    uint32_t          audio_enqueued;
    uint32_t          audio_queue_drops;
    uint32_t          audio_sent;
    uint32_t          audio_send_drop;

    volatile bool     running;
    volatile bool     peer_connected; /* DTLS/SRTP connected; camera sender may transmit */
    volatile bool     dtls_connecting; /* CONNECTING seen, waiting for CONNECTED */
    volatile int64_t  dtls_start_ms;   /* first CONNECTING timestamp; 0 when disarmed */
    volatile uint32_t fanout_refs;    /* camera callback in-flight refs */
    bool              fanout_attached;
    uint32_t          session_id;
    char              client_token[SMOKE_CLIENT_TOKEN_HEX_LEN + 1U]; /* exact browser-page owner token; "" for legacy clients */
    volatile bool     auto_cleanup_requested; /* real peer loss: pump exits then safe cleanup task reaps session */
    bool              pump_done_seen; /* done semaphore를 teardown이 이미 소비했는지 (sticky) */
    uint32_t          media_sent;
    uint32_t          media_drop;
    int64_t           last_media_log_ms;
    bool              is_offerer; /* TEST2 = true */
    bool              remote_has_relay; /* browser offer contains typ relay */
    volatile bool     local_has_relay;  /* callback-owned readiness flag; HTTP waiter never dereferences local_sdp while polling */
    volatile uint32_t local_sdp_emit_count;
    volatile uint32_t local_candidate_msg_count;
    char              mode;       /* 'A'/'B'/'C' (로그용, 역할 강제 아님) */
    camera_h264_profile_t profile;  /* selected sensor-native WebRTC profile */
    /* [CAM-1080-ADAPT-GOP-R2-20260912] Keep video PTS monotonic across camera-only GOP restarts while
     * the PeerConnection/ICE/DTLS/audio session remains alive. */
    uint32_t          video_pts_last_raw_ms;
    uint64_t          video_pts_base_ms;
    uint64_t          video_pts_last_out_ms;
    bool              video_pts_have;
    int64_t           start_ms;

    char             *local_sdp;                       /* on_msg SDP */
    char              cand[SMOKE_MAX_CAND][SMOKE_CAND_LEN]; /* "a=candidate:..." */
    int               cand_n;
} smoke_sess_t;

static SemaphoreHandle_t s_gate = NULL;        /* primary/secondary lifecycle */
static SemaphoreHandle_t s_fanout_lock = NULL; /* camera callback target snapshot */
static smoke_sess_t     *s_active = NULL;      /* primary, s_gate protects */
static smoke_sess_t     *s_secondary = NULL;   /* second viewer, s_gate protects */
static smoke_sess_t     *s_fanout[SMOKE_MAX_SESSIONS] = {0}; /* s_fanout_lock protects */
static uint32_t          s_next_session_id = 1; /* s_gate protects */
static bool              s_cert_prepared = false;

/* [CAM-1080-ADAPT-GOP-R2-20260912] ESP-authoritative adaptive GOP state. Browser supplies only WebRTC
 * receive observations; the ESP fuses them with its own TX queue/send latency. */
typedef enum {
    ADAPT_GOP5_STABLE = 0,
    ADAPT_GOP1_STABLE,
    ADAPT_PROBING_GOP1,
    ADAPT_FREEZE,
} adapt_gop_state_t;

#define ADAPT_STATS_FRESH_MS       4000LL
#define ADAPT_FREEZE_MS            5000LL
#define ADAPT_COOLDOWN_MS         15000LL
#define ADAPT_PROBE_MS             3000LL
#define ADAPT_GOP1_HEADROOM_KBPS   5200U
/* [CAM-1080-ADAPT-GOP-R2D-20260913]
 * availableIncomingBitrate is application-limited on some Chrome/Android paths:
 * while GOP5 sends ~1Mbps, the estimate can sit around 1-2Mbps even on a fast LAN.
 * Therefore it may accelerate a probe when >=5.2Mbps, but it must never veto one.
 * With no strong positive estimate require 8 clean samples (~9.6s at 1.2s poll). */
#define ADAPT_CLEAN_PROBE_GOOD_STREAK 8U
#define ADAPT_FAST_PROBE_GOOD_STREAK  3U
#define ADAPT_GOOD_JITTER_MS           30U
#define ADAPT_BAD_LOSS_BP             200U  /* 2.00% */
#define ADAPT_GOOD_LOSS_BP           30U  /* 0.30% */
#define ADAPT_PROBE_LOSS_BP         100U  /* 1.00% */
#define ADAPT_BAD_JITTER_MS          50U
#define ADAPT_BAD_FPS_RATIO_X1000   850U
#define ADAPT_GOOD_FPS_RATIO_X1000  950U
#define ADAPT_PROBE_FPS_RATIO_X1000 900U

static adapt_gop_state_t s_adapt_state = ADAPT_GOP5_STABLE;
static adapt_gop_state_t s_adapt_resume_state = ADAPT_GOP5_STABLE;
static int64_t s_adapt_freeze_until_ms;
static int64_t s_adapt_cooldown_until_ms;
static int64_t s_adapt_probe_until_ms;
static uint32_t s_adapt_last_viewer_count;
static uint32_t s_adapt_probe_fail_count;
static uint32_t s_adapt_switch_count;

static void smoke_adapt_reset_primary_locked(void)
{
    const int64_t now_ms = esp_timer_get_time() / 1000LL;
    s_adapt_state = ADAPT_GOP5_STABLE;
    s_adapt_resume_state = ADAPT_GOP5_STABLE;
    s_adapt_freeze_until_ms = 0;
    s_adapt_cooldown_until_ms = now_ms + ADAPT_COOLDOWN_MS;
    s_adapt_probe_until_ms = 0;
    s_adapt_last_viewer_count = 0;
    s_adapt_probe_fail_count = 0;
    s_adapt_switch_count = 0;
}

/* [CAM-INITIAL-START-RECOVERY-V2-TOKEN-20260903]
 * The token is a random 128-bit value generated by one browser page. It is
 * deliberately NOT a user/device identity. It exists only to reclaim the
 * exact orphan session created by that same page when the HTTP answer carrying
 * session_id is lost. */
static bool smoke_client_token_valid(const char *token)
{
    if (!token || strlen(token) != SMOKE_CLIENT_TOKEN_HEX_LEN) return false;
    for (size_t i = 0; i < SMOKE_CLIENT_TOKEN_HEX_LEN; ++i) {
        if (!isxdigit((unsigned char)token[i])) return false;
    }
    return true;
}

static smoke_sess_t *smoke_find_client_token_locked(const char *token)
{
    if (!smoke_client_token_valid(token)) return NULL;
    if (s_active && s_active->client_token[0] &&
        strcmp(s_active->client_token, token) == 0) return s_active;
    if (s_secondary && s_secondary->client_token[0] &&
        strcmp(s_secondary->client_token, token) == 0) return s_secondary;
    return NULL;
}

/* ───────────── SDP 유틸 ───────────── */

static void parse_setup(const char *sdp, char *out, size_t outsz)
{
    out[0] = '\0';
    if (!sdp) return;
    const char *p = strstr(sdp, "a=setup:");
    if (!p) return;
    p += 8;
    size_t i = 0;
    while (*p && *p != '\r' && *p != '\n' && i + 1 < outsz) out[i++] = *p++;
    out[i] = '\0';
}

static const char *setup_to_dtls_role(const char *setup)
{
    if (!strcmp(setup, "active"))  return "client";
    if (!strcmp(setup, "passive")) return "server";
    if (!strcmp(setup, "actpass")) return "actpass(negotiating)";
    return "unknown";
}

static camera_h264_profile_t smoke_profile_from_text(const char *txt)
{
    if (txt && (!strcasecmp(txt, "720") || !strcasecmp(txt, "720p") ||
                !strcasecmp(txt, "720p-native") || !strcasecmp(txt, "720p60-native") ||
                !strcasecmp(txt, "720p-highspeed") || !strcasecmp(txt, "1288x728"))) {
        return CAMERA_H264_PROFILE_720P_NATIVE;
    }
    return CAMERA_H264_PROFILE_1080P;
}

/* [CAM-PROFILE-SEPARATE-NVS-20260910]
 * 1080p and 720p60 use completely independent saved presets.
 * There is no BASE/BIAS calculation. */
static void isp_tune_saved_for_profile(camera_h264_profile_t profile,
                                       bool *enabled,
                                       int *sat, int *con,
                                       int *gamma_milli, int *sharp)
{
    if (profile == CAMERA_H264_PROFILE_720P_NATIVE) {
        if (enabled) *enabled = (g_camera_isp_720_manual_enable != 0);
        if (sat) *sat = g_camera_isp_720_saturation;
        if (con) *con = g_camera_isp_720_contrast;
        if (gamma_milli) *gamma_milli = g_camera_isp_720_gamma_milli;
        if (sharp) *sharp = g_camera_isp_720_sharpen_pct;
        return;
    }

    if (enabled) *enabled = (g_camera_isp_manual_enable != 0);
    if (sat) *sat = g_camera_isp_saturation;
    if (con) *con = g_camera_isp_contrast;
    if (gamma_milli) *gamma_milli = g_camera_isp_gamma_milli;
    if (sharp) *sharp = g_camera_isp_sharpen_pct;
}

static esp_err_t isp_tune_apply_profile(camera_h264_profile_t profile)
{
    bool enabled = false;
    int sat = 136, con = 126, gamma_milli = 518, sharp = 100;
    isp_tune_saved_for_profile(profile, &enabled, &sat, &con, &gamma_milli, &sharp);

    esp_err_t err = esp_video_isp_pipeline_live_tune_set(enabled, sat, con,
                                                          gamma_milli, sharp);
    if (err == ESP_OK) {
        ESP_LOGI(TAG,
                 "[CAM-PROFILE-TUNE] apply profile=%s saved-mode=%s sat=%d con=%d gamma=%d sharp=%d",
                 camera_h264_stream_profile_name(profile),
                 enabled ? "MANUAL" : "AUTO",
                 sat, con, gamma_milli, sharp);
    } else {
        ESP_LOGW(TAG,
                 "[CAM-PROFILE-TUNE] apply failed profile=%s err=%s",
                 camera_h264_stream_profile_name(profile), esp_err_to_name(err));
    }
    return err;
}

static void smoke_profile_video_info(camera_h264_profile_t profile,
                                     uint32_t *width, uint32_t *height, uint32_t *fps)
{
    if (profile == CAMERA_H264_PROFILE_720P_NATIVE) {
        if (width) *width = 1288;
        if (height) *height = 728;
        if (fps) *fps = 60;
    } else {
        if (width) *width = 1920;
        if (height) *height = 1080;
        if (fps) *fps = 30;
    }
}

/* remote SDP 안의 a=candidate 를 로그로 (IP vs .local 판별) */
static void log_remote_candidates(const char *sdp)
{
    if (!sdp) return;
    const char *p = sdp;
    while ((p = strstr(p, "a=candidate:")) != NULL) {
        /* 형식: a=candidate:<foundation> <comp> <transport> <prio> <ADDRESS> <port> typ ... */
        const char *q = p + strlen("a=candidate:");
        int field = 0;
        const char *addr_start = NULL; int addr_len = 0;
        const char *r = q;
        while (*r && *r != '\r' && *r != '\n') {
            const char *tok = r;
            while (*r && *r != ' ' && *r != '\r' && *r != '\n') r++;
            int len = (int)(r - tok);
            if (field == 4) { addr_start = tok; addr_len = len; break; }
            field++;
            while (*r == ' ') r++;
        }
        if (addr_start && addr_len > 0 && addr_len < 128) {
            char a[128]; memcpy(a, addr_start, addr_len); a[addr_len] = '\0';
            bool mdns = (addr_len >= 6) && (strcasecmp(a + addr_len - 6, ".local") == 0);
            SMK("remote candidate=%s%s", a, mdns ? " (.local/mDNS)" : " (IP)");
        }
        p += 12;
    }
}

/* Count host/srflx/relay candidate lines already embedded in an SDP.
 * esp_peer may place host candidates directly in the local SDP rather than
 * emitting ESP_PEER_MSG_TYPE_CANDIDATE callbacks, especially in offerer mode. */
static int count_sdp_candidate_lines(const char *sdp)
{
    if (!sdp) return 0;
    int n = 0;
    const char *p = sdp;
    while ((p = strstr(p, "a=candidate:")) != NULL) {
        n++;
        p += strlen("a=candidate:");
    }
    return n;
}

/* Desktop Chrome often hides its LAN host candidate behind an RFC6762-style
 * random <name>.local token. esp_peer 1.5.4 does not resolve that token and
 * reports `Add remote ... unknown`, so the peer fails before ICE can pair.
 *
 * For this same-origin LAN UI the HTTP POST and WebRTC offer come from the
 * same browser host. Therefore the TCP peer address is the numeric LAN IPv4
 * that the .local host candidate represents on the route to this ESP.
 * Replace only candidate ADDRESS tokens ending in ".local"; ports, priority,
 * foundation, type and every numeric candidate remain untouched. */
static bool smoke_http_peer_ipv4(httpd_req_t *req, char out[16])
{
    if (!req || !out) return false;
    out[0] = '\0';

    int fd = httpd_req_to_sockfd(req);
    if (fd < 0) {
        SMKW("[MDNS-CAND-FIX] HTTP peer lookup failed: invalid request fd");
        return false;
    }

    struct sockaddr_storage ss;
    memset(&ss, 0, sizeof(ss));
    socklen_t sl = sizeof(ss);
    if (getpeername(fd, (struct sockaddr *)&ss, &sl) != 0) {
        int e = errno;
        SMKW("[MDNS-CAND-FIX] HTTP peer lookup failed fd=%d getpeername errno=%d (%s)",
             fd, e, strerror(e));
        return false;
    }

    if (ss.ss_family == AF_INET) {
        const struct sockaddr_in *sa = (const struct sockaddr_in *)&ss;
        if (inet_ntop(AF_INET, &sa->sin_addr, out, 16) != NULL) {
            SMK("[MDNS-CAND-FIX] HTTP peer fd=%d family=AF_INET ip=%s", fd, out);
            return true;
        }
        SMKW("[MDNS-CAND-FIX] HTTP peer AF_INET inet_ntop failed fd=%d errno=%d",
             fd, errno);
        return false;
    }

#if LWIP_IPV6
    if (ss.ss_family == AF_INET6) {
        const struct sockaddr_in6 *sa6 = (const struct sockaddr_in6 *)&ss;
        const uint8_t *a = (const uint8_t *)&sa6->sin6_addr;

        /* Dual-stack sockets can expose an IPv4 client as ::ffff:a.b.c.d.
         * Convert that mapped address back to the numeric IPv4 candidate. */
        const bool v4mapped =
            a[0] == 0 && a[1] == 0 && a[2] == 0 && a[3] == 0 &&
            a[4] == 0 && a[5] == 0 && a[6] == 0 && a[7] == 0 &&
            a[8] == 0 && a[9] == 0 && a[10] == 0xff && a[11] == 0xff;

        if (v4mapped) {
            struct in_addr v4;
            memcpy(&v4.s_addr, &a[12], sizeof(v4.s_addr));
            if (inet_ntop(AF_INET, &v4, out, 16) != NULL) {
                SMK("[MDNS-CAND-FIX] HTTP peer fd=%d family=AF_INET6(v4mapped) ip=%s",
                    fd, out);
                return true;
            }
        }

        char ip6[INET6_ADDRSTRLEN] = {0};
        (void)inet_ntop(AF_INET6, &sa6->sin6_addr, ip6, sizeof(ip6));
        SMKW("[MDNS-CAND-FIX] HTTP peer is native IPv6 (%s); no IPv4 substitution",
             ip6[0] ? ip6 : "unprintable");
        return false;
    }
#endif

    SMKW("[MDNS-CAND-FIX] HTTP peer unsupported family=%d fd=%d",
         (int)ss.ss_family, fd);
    return false;
}

static bool smoke_ipv4_is_private_lan(const char *ipv4)
{
    struct in_addr a;
    if (!ipv4 || inet_pton(AF_INET, ipv4, &a) != 1) return false;
    uint32_t h = ntohl(a.s_addr);
    return ((h & 0xff000000U) == 0x0a000000U) ||          /* 10.0.0.0/8 */
           ((h & 0xfff00000U) == 0xac100000U) ||          /* 172.16.0.0/12 */
           ((h & 0xffff0000U) == 0xc0a80000U);            /* 192.168.0.0/16 */
}

/* esp_peer 1.5.4 does not create a peer-reflexive remote candidate when the
 * browser advertises one address but the STUN Binding request arrives from a
 * different LAN address.  On this same-origin LAN page, the HTTP TCP peer is
 * the browser's actually-routable address toward the ESP.
 *
 * If the offer contains exactly one ICE candidate (our current non-trickle
 * browser path), pin that candidate ADDRESS to the private HTTP peer IPv4.
 * With multiple candidates, keep numeric candidates untouched and only
 * resolve mDNS .local addresses.  The UDP candidate port and all other fields
 * are preserved. */
static char *smoke_rewrite_lan_candidates_ipv4(const char *sdp,
                                                const char *ipv4,
                                                int *rewritten)
{
    if (rewritten) *rewritten = 0;
    if (!sdp || !ipv4 || !ipv4[0] || !smoke_ipv4_is_private_lan(ipv4)) return NULL;

    const int candidate_count = count_sdp_candidate_lines(sdp);
    const bool force_single = (candidate_count == 1);
    char *copy = strdup(sdp);
    if (!copy) return NULL;

    char *p = copy;
    int changed = 0;
    while ((p = strstr(p, "a=candidate:")) != NULL) {
        char *r = p + strlen("a=candidate:");
        int field = 0;
        char *addr_start = NULL;
        char *addr_end = NULL;

        while (*r && *r != '\r' && *r != '\n') {
            while (*r == ' ') r++;
            char *tok = r;
            while (*r && *r != ' ' && *r != '\r' && *r != '\n') r++;
            if (field == 4) {
                addr_start = tok;
                addr_end = r;
                break;
            }
            field++;
        }

        if (!addr_start || !addr_end || addr_end <= addr_start) {
            p += strlen("a=candidate:");
            continue;
        }

        size_t old_len = (size_t)(addr_end - addr_start);
        size_t new_len = strlen(ipv4);
        const bool is_mdns = old_len >= 6 &&
                             strncasecmp(addr_end - 6, ".local", 6) == 0;
        const bool already_peer =
            old_len == new_len && strncmp(addr_start, ipv4, new_len) == 0;

        if (already_peer || (!force_single && !is_mdns)) {
            p = addr_end;
            continue;
        }

        /* Candidate address can grow (e.g. public 39.x.x.x -> 192.168.x.x),
         * so realloc and then rebuild pointers from offsets. */
        size_t addr_off = (size_t)(addr_start - copy);
        size_t end_off = (size_t)(addr_end - copy);
        size_t cur_len = strlen(copy);
        if (new_len > old_len) {
            size_t grow = new_len - old_len;
            char *grown = realloc(copy, cur_len + grow + 1U);
            if (!grown) {
                free(copy);
                return NULL;
            }
            copy = grown;
            addr_start = copy + addr_off;
            addr_end = copy + end_off;
            memmove(addr_end + grow, addr_end, cur_len - end_off + 1U);
            addr_end += grow;
        } else if (new_len < old_len) {
            size_t shrink = old_len - new_len;
            addr_start = copy + addr_off;
            addr_end = copy + end_off;
            memmove(addr_end - shrink, addr_end, cur_len - end_off + 1U);
            addr_end -= shrink;
        } else {
            addr_start = copy + addr_off;
            addr_end = copy + end_off;
        }

        memcpy(addr_start, ipv4, new_len);
        changed++;
        p = addr_start + new_len;
    }

    if (rewritten) *rewritten = changed;
    if (changed == 0) {
        free(copy);
        return NULL;
    }
    return copy;
}

/* local_sdp + 수집한 candidate 를 합쳐 non-trickle SDP 문자열을 만든다(호출자 free) */
static char *build_aggregated_sdp(smoke_sess_t *s)
{
    if (!s->local_sdp) return NULL;
    size_t need = strlen(s->local_sdp) + 64;
    for (int i = 0; i < s->cand_n; i++) need += strlen(s->cand[i]) + 3;
    need += strlen("a=end-of-candidates\r\n") + 4;
    char *out = (char *)malloc(need);
    if (!out) return NULL;
    size_t n = 0;
    n += snprintf(out + n, need - n, "%s", s->local_sdp);
    /* SDP 끝이 개행으로 끝나도록 보정 */
    if (n >= 2 && !(out[n-1] == '\n')) { out[n++] = '\r'; out[n++] = '\n'; out[n] = '\0'; }
    for (int i = 0; i < s->cand_n; i++)
        n += snprintf(out + n, need - n, "%s\r\n", s->cand[i]);
    n += snprintf(out + n, need - n, "a=end-of-candidates\r\n");
    return out;
}

/* ───────────── esp_peer 콜백 (pump 태스크 컨텍스트) ───────────── */

/* [WEBRTC-2CLIENT-FANOUT-AB] Session table is intentionally tiny.
 * Control/lifecycle uses s_gate; the camera callback never takes s_gate. */
static uint32_t smoke_session_count_locked(void)
{
    return (s_active ? 1U : 0U) + (s_secondary ? 1U : 0U);
}

static smoke_sess_t *smoke_find_session_locked(uint32_t sid)
{
    if (sid == 0U) return NULL;
    if (s_active && s_active->session_id == sid) return s_active;
    if (s_secondary && s_secondary->session_id == sid) return s_secondary;
    return NULL;
}

static uint32_t smoke_fanout_count(void)
{
    if (!s_fanout_lock) return 0;
    uint32_t n = 0;
    if (xSemaphoreTake(s_fanout_lock, pdMS_TO_TICKS(10)) == pdTRUE) {
        for (uint32_t i = 0; i < SMOKE_MAX_SESSIONS; ++i) if (s_fanout[i]) n++;
        xSemaphoreGive(s_fanout_lock);
    }
    return n;
}

static bool smoke_fanout_attach(smoke_sess_t *sess)
{
    if (!sess || !s_fanout_lock) return false;
    if (xSemaphoreTake(s_fanout_lock, pdMS_TO_TICKS(50)) != pdTRUE) return false;
    if (!sess->fanout_attached) {
        for (uint32_t i = 0; i < SMOKE_MAX_SESSIONS; ++i) {
            if (!s_fanout[i]) {
                s_fanout[i] = sess;
                sess->fanout_attached = true;
                break;
            }
        }
    }
    bool ok = sess->fanout_attached;
    xSemaphoreGive(s_fanout_lock);
    return ok;
}

static void smoke_fanout_detach(smoke_sess_t *sess)
{
    if (!sess || !s_fanout_lock) return;
    if (xSemaphoreTake(s_fanout_lock, pdMS_TO_TICKS(50)) != pdTRUE) return;
    for (uint32_t i = 0; i < SMOKE_MAX_SESSIONS; ++i) {
        if (s_fanout[i] == sess) s_fanout[i] = NULL;
    }
    sess->fanout_attached = false;
    xSemaphoreGive(s_fanout_lock);
}

static bool smoke_fanout_wait_idle(smoke_sess_t *sess, uint32_t timeout_ms)
{
    if (!sess) return true;
    int64_t until = esp_timer_get_time() + (int64_t)timeout_ms * 1000;
    while (__atomic_load_n(&sess->fanout_refs, __ATOMIC_ACQUIRE) != 0U) {
        if (esp_timer_get_time() >= until) return false;
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    return true;
}

/* ───────────── [ASYNC-H264-TX] PSRAM SPSC queue ───────────── */

static uint32_t smoke_tx_count(smoke_sess_t *s)
{
    if (!s || !s->tx_lock) return 0;
    uint32_t n = 0;
    if (xSemaphoreTake(s->tx_lock, pdMS_TO_TICKS(5)) == pdTRUE) {
        n = s->tx_count;
        xSemaphoreGive(s->tx_lock);
    }
    return n;
}

static void smoke_tx_pool_free(smoke_sess_t *s)
{
    if (!s) return;
    for (uint32_t i = 0; i < SMOKE_TX_QUEUE_DEPTH; ++i) {
        if (s->tx_slots[i].data) {
            heap_caps_free(s->tx_slots[i].data);
            s->tx_slots[i].data = NULL;
        }
        s->tx_slots[i].capacity = 0;
        s->tx_slots[i].len = 0;
        s->tx_slots[i].pts_ms = 0;
    }
    for (uint32_t i = 0; i < SMOKE_AUDIO_QUEUE_DEPTH; ++i) {
        if (s->audio_slots[i].data) {
            heap_caps_free(s->audio_slots[i].data);
            s->audio_slots[i].data = NULL;
        }
        s->audio_slots[i].len = 0;
        s->audio_slots[i].pts_ms = 0;
    }
    if (s->audio_lock) {
        vSemaphoreDelete(s->audio_lock);
        s->audio_lock = NULL;
    }
    if (s->tx_lock) {
        vSemaphoreDelete(s->tx_lock);
        s->tx_lock = NULL;
    }
    s->tx_head = s->tx_tail = s->tx_count = 0;
    s->audio_head = s->audio_tail = s->audio_count = 0;
}

static bool smoke_tx_pool_init(smoke_sess_t *s)
{
    if (!s) return false;
    s->tx_lock = xSemaphoreCreateMutex();
    if (!s->tx_lock) return false;

    for (uint32_t i = 0; i < SMOKE_TX_QUEUE_DEPTH; ++i) {
        s->tx_slots[i].data = (uint8_t *)heap_caps_malloc(
            SMOKE_TX_SLOT_INITIAL_BYTES, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!s->tx_slots[i].data) {
            SMKE("H264 TX PSRAM slot alloc failed idx=%u bytes=%u",
                 (unsigned)i, (unsigned)SMOKE_TX_SLOT_INITIAL_BYTES);
            smoke_tx_pool_free(s);
            return false;
        }
        s->tx_slots[i].capacity = SMOKE_TX_SLOT_INITIAL_BYTES;
    }
    s->audio_lock = xSemaphoreCreateMutex();
    if (!s->audio_lock) {
        smoke_tx_pool_free(s);
        return false;
    }
    for (uint32_t i = 0; i < SMOKE_AUDIO_QUEUE_DEPTH; ++i) {
        s->audio_slots[i].data = (uint8_t *)heap_caps_malloc(
            SMOKE_AUDIO_SLOT_BYTES, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!s->audio_slots[i].data) {
            SMKE("Opus TX PSRAM slot alloc failed idx=%u bytes=%u",
                 (unsigned)i, (unsigned)SMOKE_AUDIO_SLOT_BYTES);
            smoke_tx_pool_free(s);
            return false;
        }
    }
    SMK("H264 queue depth=%u slot=%uKB; Opus queue depth=%u slot=%uB psram_free=%u",
        (unsigned)SMOKE_TX_QUEUE_DEPTH,
        (unsigned)(SMOKE_TX_SLOT_INITIAL_BYTES / 1024U),
        (unsigned)SMOKE_AUDIO_QUEUE_DEPTH, (unsigned)SMOKE_AUDIO_SLOT_BYTES,
        (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
    return true;
}

/* Called only on the single camera producer task. The ring entry is published
 * only after memcpy finishes, so smoke_pump can never observe a half-copied
 * H.264 access unit. A counted tail slot stays owned by the consumer until
 * esp_peer_send_video() returns, therefore the producer cannot overwrite it. */
static int smoke_send_h264(const uint8_t *data, size_t len, uint32_t pts_ms, void *ctx)
{
    smoke_sess_t *s = (smoke_sess_t *)ctx;
    if (!s || !data || len == 0 || !s->running || !s->peer_connected || !s->tx_lock) {
        return ESP_PEER_ERR_FAIL;
    }

    /* [CAM-1080-ADAPT-GOP-R2-20260912] camera_h264_live_task() restarts local PTS at zero after an
     * adaptive GOP change. Translate it to one monotonic timeline per peer. */
    {
        uint64_t out_pts;
        if (!s->video_pts_have) {
            s->video_pts_base_ms = 0;
            out_pts = (uint64_t)pts_ms;
            s->video_pts_have = true;
        } else {
            if (pts_ms < s->video_pts_last_raw_ms) {
                s->video_pts_base_ms = s->video_pts_last_out_ms + 1ULL - (uint64_t)pts_ms;
            }
            out_pts = s->video_pts_base_ms + (uint64_t)pts_ms;
            if (out_pts <= s->video_pts_last_out_ms) {
                out_pts = s->video_pts_last_out_ms + 1ULL;
                s->video_pts_base_ms = out_pts - (uint64_t)pts_ms;
            }
        }
        s->video_pts_last_raw_ms = pts_ms;
        s->video_pts_last_out_ms = out_pts;
        pts_ms = (uint32_t)out_pts;
    }

    if (len > SMOKE_TX_SLOT_MAX_BYTES) {
        s->tx_oversize_drops++;
        s->tx_queue_drops++;
        s->media_drop++;
        SMKE("sid=%u H264 TX oversize drop len=%u max=%u", (unsigned)s->session_id,
             (unsigned)len, (unsigned)SMOKE_TX_SLOT_MAX_BYTES);
        return ESP_PEER_ERR_FAIL;
    }

    uint32_t idx = 0;
    if (xSemaphoreTake(s->tx_lock, pdMS_TO_TICKS(5)) != pdTRUE) {
        s->tx_queue_drops++;
        s->media_drop++;
        return ESP_PEER_ERR_FAIL;
    }
    if (s->tx_count >= SMOKE_TX_QUEUE_DEPTH) {
        s->tx_queue_drops++;
        s->media_drop++;
        uint32_t qdrop = s->tx_queue_drops;
        xSemaphoreGive(s->tx_lock);
        if ((qdrop & 0x0fU) == 1U) {
            SMKE("sid=%u H264 TX queue full drop=%u depth=%u", (unsigned)s->session_id,
                 (unsigned)qdrop, (unsigned)SMOKE_TX_QUEUE_DEPTH);
        }
        return ESP_PEER_ERR_FAIL;
    }
    idx = s->tx_head;
    xSemaphoreGive(s->tx_lock);

    smoke_tx_slot_t *slot = &s->tx_slots[idx];
    if (len > slot->capacity) {
        size_t need = (len + SMOKE_TX_SLOT_GROW_BYTES - 1U) /
                      SMOKE_TX_SLOT_GROW_BYTES * SMOKE_TX_SLOT_GROW_BYTES;
        if (need > SMOKE_TX_SLOT_MAX_BYTES) need = SMOKE_TX_SLOT_MAX_BYTES;
        uint8_t *nb = (uint8_t *)heap_caps_malloc(need,
                            MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!nb) {
            s->tx_queue_drops++;
            s->media_drop++;
            SMKE("H264 TX PSRAM grow failed idx=%u need=%u psram_free=%u",
                 (unsigned)idx, (unsigned)need,
                 (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
            return ESP_PEER_ERR_FAIL;
        }
        heap_caps_free(slot->data);
        slot->data = nb;
        slot->capacity = need;
        SMK("sid=%u H264 TX slot[%u] grown to %uKB", (unsigned)s->session_id,
            (unsigned)idx, (unsigned)(need / 1024U));
    }

    int64_t copy_begin_us = esp_timer_get_time();
    memcpy(slot->data, data, len);
    int64_t copy_done_us = esp_timer_get_time();
    uint32_t copy_us = copy_done_us > copy_begin_us ?
        (uint32_t)(copy_done_us - copy_begin_us) : 0;
    slot->len = len;
    slot->pts_ms = pts_ms;

    /* Publish only after the complete bitstream copy is visible. */
    if (xSemaphoreTake(s->tx_lock, pdMS_TO_TICKS(5)) != pdTRUE) {
        s->tx_queue_drops++;
        s->media_drop++;
        return ESP_PEER_ERR_FAIL;
    }
    s->tx_head = (s->tx_head + 1U) % SMOKE_TX_QUEUE_DEPTH;
    s->tx_count++;
    s->tx_enqueued++;
    if (s->tx_count > s->tx_high_water) s->tx_high_water = s->tx_count;
    /* [CAM-1080-ADAPT-GOP-R2-20260912] Windowed high-water for the 1.2s adaptive control interval. */
    if (s->tx_count > s->adapt_tx_hwm_window) s->adapt_tx_hwm_window = s->tx_count;
    s->tx_copy_us_last = copy_us;
    if (copy_us > s->tx_copy_us_max) s->tx_copy_us_max = copy_us;
    xSemaphoreGive(s->tx_lock);
    return ESP_PEER_ERR_NONE;
}


/* One camera/H.264 producer, two independent peer queues.
 * fanout_refs makes session free safe while the callback is copying. */
static int smoke_fanout_h264(const uint8_t *data, size_t len, uint32_t pts_ms, void *ctx)
{
    (void)ctx;
    smoke_sess_t *targets[SMOKE_MAX_SESSIONS] = {0};
    uint32_t n = 0;

    if (!data || len == 0 || !s_fanout_lock) return ESP_PEER_ERR_FAIL;
    if (xSemaphoreTake(s_fanout_lock, pdMS_TO_TICKS(5)) != pdTRUE) {
        return ESP_PEER_ERR_FAIL;
    }
    for (uint32_t i = 0; i < SMOKE_MAX_SESSIONS; ++i) {
        smoke_sess_t *sess = s_fanout[i];
        if (sess && sess->running && sess->peer_connected) {
            __atomic_add_fetch(&sess->fanout_refs, 1U, __ATOMIC_ACQ_REL);
            targets[n++] = sess;
        }
    }
    xSemaphoreGive(s_fanout_lock);

    uint32_t accepted = 0;
    for (uint32_t i = 0; i < n; ++i) {
        smoke_sess_t *sess = targets[i];
        if (smoke_send_h264(data, len, pts_ms, sess) == ESP_PEER_ERR_NONE) accepted++;
        __atomic_sub_fetch(&sess->fanout_refs, 1U, __ATOMIC_ACQ_REL);
    }

    /* A slow second viewer is a per-peer queue drop, not a camera-level drop,
     * as long as at least one peer accepted the AU. */
    return accepted ? ESP_PEER_ERR_NONE : ESP_PEER_ERR_FAIL;
}

/* [CAM-AUDIO-OPUS24K-V4-20260902] Enqueue one encoded Opus packet. On overflow discard oldest
 * so a slow network cannot turn live audio into seconds of stale backlog. */
static int smoke_send_opus(const uint8_t *data, size_t len, uint32_t pts_ms, void *ctx)
{
    smoke_sess_t *s = (smoke_sess_t *)ctx;
    if (!s || !data || len == 0 || len > SMOKE_AUDIO_SLOT_BYTES ||
        !s->running || !s->peer_connected || !s->audio_lock) {
        return ESP_PEER_ERR_FAIL;
    }

    if (xSemaphoreTake(s->audio_lock, pdMS_TO_TICKS(2)) != pdTRUE) {
        s->audio_queue_drops++;
        return ESP_PEER_ERR_FAIL;
    }

    if (s->audio_count >= SMOKE_AUDIO_QUEUE_DEPTH) {
        s->audio_tail = (s->audio_tail + 1U) % SMOKE_AUDIO_QUEUE_DEPTH;
        s->audio_count--;
        s->audio_queue_drops++;
    }

    smoke_audio_slot_t *slot = &s->audio_slots[s->audio_head];
    memcpy(slot->data, data, len);
    slot->len = len;
    slot->pts_ms = pts_ms;
    s->audio_head = (s->audio_head + 1U) % SMOKE_AUDIO_QUEUE_DEPTH;
    s->audio_count++;
    s->audio_enqueued++;
    if (s->audio_count > s->audio_high_water) s->audio_high_water = s->audio_count;

    xSemaphoreGive(s->audio_lock);
    return ESP_PEER_ERR_NONE;
}

static int smoke_fanout_opus(const uint8_t *data, size_t len, uint32_t pts_ms, void *ctx)
{
    (void)ctx;
    smoke_sess_t *targets[SMOKE_MAX_SESSIONS] = {0};
    uint32_t n = 0, accepted = 0;

    if (!data || len == 0 || !s_fanout_lock) return ESP_PEER_ERR_FAIL;
    if (xSemaphoreTake(s_fanout_lock, pdMS_TO_TICKS(5)) != pdTRUE) {
        return ESP_PEER_ERR_FAIL;
    }

    for (uint32_t i = 0; i < SMOKE_MAX_SESSIONS; ++i) {
        smoke_sess_t *sess = s_fanout[i];
        if (sess && sess->running && sess->peer_connected) {
            __atomic_add_fetch(&sess->fanout_refs, 1U, __ATOMIC_ACQ_REL);
            targets[n++] = sess;
        }
    }
    xSemaphoreGive(s_fanout_lock);

    for (uint32_t i = 0; i < n; ++i) {
        smoke_sess_t *sess = targets[i];
        if (smoke_send_opus(data, len, pts_ms, sess) == ESP_PEER_ERR_NONE) accepted++;
        __atomic_sub_fetch(&sess->fanout_refs, 1U, __ATOMIC_ACQ_REL);
    }
    return accepted ? ESP_PEER_ERR_NONE : ESP_PEER_ERR_FAIL;
}

static uint32_t smoke_audio_count(smoke_sess_t *s)
{
    if (!s || !s->audio_lock) return 0;
    uint32_t n = 0;
    if (xSemaphoreTake(s->audio_lock, pdMS_TO_TICKS(2)) == pdTRUE) {
        n = s->audio_count;
        xSemaphoreGive(s->audio_lock);
    }
    return n;
}

/* peer_mtx is already held. Keep audio_lock across this small SRTP send so
 * the producer cannot overwrite the tail packet while esp_peer uses it. */
static bool smoke_audio_send_one_locked(smoke_sess_t *s)
{
    if (!s || !s->audio_lock || !s->peer || !s->running || !s->peer_connected) {
        return false;
    }
    if (xSemaphoreTake(s->audio_lock, pdMS_TO_TICKS(2)) != pdTRUE) return false;
    if (s->audio_count == 0U) {
        xSemaphoreGive(s->audio_lock);
        return false;
    }

    smoke_audio_slot_t *slot = &s->audio_slots[s->audio_tail];
    esp_peer_audio_frame_t frame = {
        .pts = slot->pts_ms,
        .data = slot->data,
        .size = (int)slot->len,
    };
    int r = esp_peer_send_audio(s->peer, &frame);

    slot->len = 0;
    s->audio_tail = (s->audio_tail + 1U) % SMOKE_AUDIO_QUEUE_DEPTH;
    s->audio_count--;
    if (r == ESP_PEER_ERR_NONE) s->audio_sent++;
    else s->audio_send_drop++;

    uint32_t sent = s->audio_sent;
    uint32_t drop = s->audio_send_drop;
    uint32_t qdrop = s->audio_queue_drops;
    xSemaphoreGive(s->audio_lock);

    if (r != ESP_PEER_ERR_NONE && ((drop & 0x0fU) == 1U)) {
        SMKW("sid=%u Opus send fail r=%d sent=%u send_drop=%u qdrop=%u",
             (unsigned)s->session_id, r, (unsigned)sent,
             (unsigned)drop, (unsigned)qdrop);
    }
    return true;
}
/* peer_mtx MUST already be held by smoke_pump. Returns true if one queued
 * access unit was consumed (success or send failure). */
static bool smoke_tx_send_one_locked(smoke_sess_t *s)
{
    if (!s || !s->tx_lock || !s->peer || !s->running || !s->peer_connected) return false;

    uint32_t idx = 0;
    uint32_t q_before = 0;
    size_t len = 0;
    uint32_t pts_ms = 0;
    uint8_t *data = NULL;
    if (xSemaphoreTake(s->tx_lock, pdMS_TO_TICKS(5)) != pdTRUE) return false;
    if (s->tx_count == 0) {
        xSemaphoreGive(s->tx_lock);
        return false;
    }
    idx = s->tx_tail;
    q_before = s->tx_count;
    data = s->tx_slots[idx].data;
    len = s->tx_slots[idx].len;
    pts_ms = s->tx_slots[idx].pts_ms;
    xSemaphoreGive(s->tx_lock);

    int r = ESP_PEER_ERR_FAIL;
    int64_t send_begin_us = esp_timer_get_time();
    if (data && len > 0 && len <= INT32_MAX) {
        esp_peer_video_frame_t frame = {
            .pts = pts_ms,
            .data = data,
            .size = (int)len,
        };
        r = esp_peer_send_video(s->peer, &frame);
    }
    int64_t send_done_us = esp_timer_get_time();
    uint32_t send_us = send_done_us > send_begin_us ?
        (uint32_t)(send_done_us - send_begin_us) : 0;

    if (xSemaphoreTake(s->tx_lock, pdMS_TO_TICKS(5)) == pdTRUE) {
        /* This tail entry stayed counted during send, so producer could not
         * reuse it. Release it only after esp_peer_send_video() returns. */
        if (s->tx_count > 0 && s->tx_tail == idx) {
            s->tx_slots[idx].len = 0;
            s->tx_tail = (s->tx_tail + 1U) % SMOKE_TX_QUEUE_DEPTH;
            s->tx_count--;
        }
        s->tx_send_us_last = send_us;
        if (send_us > s->tx_send_us_max) s->tx_send_us_max = send_us;
        xSemaphoreGive(s->tx_lock);
    }

    if (r == ESP_PEER_ERR_NONE) s->media_sent++;
    else s->media_drop++;

    int64_t now_ms = send_done_us / 1000;
    if (r != ESP_PEER_ERR_NONE ||
        (HA_VERBOSE_DIAG_LOGS && now_ms - s->last_media_log_ms >= 2000)) {
        SMK("sid=%u media H264 sent=%u drop=%u last=%u bytes pts=%ums r=%d q=%u/%u hwm=%u copy=%uus send=%uus",
            (unsigned)s->session_id, (unsigned)s->media_sent, (unsigned)s->media_drop,
            (unsigned)len, (unsigned)pts_ms, r,
            (unsigned)q_before, (unsigned)SMOKE_TX_QUEUE_DEPTH,
            (unsigned)s->tx_high_water,
            (unsigned)s->tx_copy_us_last, (unsigned)send_us);
        s->last_media_log_ms = now_ms;
    }
    return true;
}

static int smoke_on_state(esp_peer_state_t st, void *ctx)
{
    smoke_sess_t *sess = (smoke_sess_t *)ctx;
    switch (st) {
    case ESP_PEER_STATE_CANDIDATE_GATHERING:
        SMK("sid=%u ICE state=gathering", (unsigned)sess->session_id);
        break;
    case ESP_PEER_STATE_PAIRING:
        SMK("sid=%u ICE state=pairing", (unsigned)sess->session_id);
        break;
    case ESP_PEER_STATE_PAIRED: {
        SMK("sid=%u ICE state=paired", (unsigned)sess->session_id);
        esp_peer_addr_t addr;
        if (sess->peer && esp_peer_get_paired_addr(sess->peer, &addr) == ESP_PEER_ERR_NONE) {
            SMK("sid=%u remote=%u.%u.%u.%u:%u",
                (unsigned)sess->session_id,
                addr.ipv4[0], addr.ipv4[1], addr.ipv4[2], addr.ipv4[3], addr.port);
        }
        break;
    }
    case ESP_PEER_STATE_CONNECTING: {
        int64_t now_ms = esp_timer_get_time() / 1000;
        if (!sess->dtls_connecting) {
            sess->dtls_connecting = true;
            sess->dtls_start_ms = now_ms;
            SMK("sid=%u ICE state=connecting (DTLS) watchdog=%ums",
                (unsigned)sess->session_id, (unsigned)SMOKE_DTLS_CONNECT_TIMEOUT_MS);
        } else {
            SMK("sid=%u ICE state=connecting (DTLS)", (unsigned)sess->session_id);
        }
        break;
    }
    case ESP_PEER_STATE_CONNECTED: {
        int64_t now_ms = esp_timer_get_time() / 1000;
        int64_t dtls_ms = (sess->dtls_start_ms > 0 && now_ms >= sess->dtls_start_ms)
            ? (now_ms - sess->dtls_start_ms) : -1;
        sess->dtls_connecting = false;
        sess->dtls_start_ms = 0;
        turn_gather_rx_ab_restore_steady(sess->session_id);
        sess->peer_connected = true;
        SMK("sid=%u [DTLS-HANDSHAKE-WINDOW-C1-EXACT] CONNECTED -> steady recv=10ms before media",
            (unsigned)sess->session_id);
        if (!smoke_fanout_attach(sess)) {
            sess->peer_connected = false;
            SMKE("sid=%u fanout attach failed", (unsigned)sess->session_id);
            if (sess->running) {
                sess->auto_cleanup_requested = true;
                sess->running = false;
            }
            break;
        }

        uint32_t viewers = smoke_fanout_count();
        SMK("sid=%u DTLS/SRTP ready viewers=%u/%u handshake=%lldms",
            (unsigned)sess->session_id, (unsigned)viewers,
            (unsigned)SMOKE_MAX_SESSIONS, (long long)dtls_ms);
        SMKW("[DMA-DIAG D0 after_dtls_srtp_ready] free=%u largest=%u",
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA),
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA));

        if (viewers == 1U && !camera_h264_stream_is_active()) {
            esp_err_t cr = camera_h264_stream_start(smoke_fanout_h264, NULL);
            if (cr == ESP_OK) {
                SMK("camera H264 start requested: encoder_count=1");
            } else {
                SMKE("camera H264 start failed: %s", esp_err_to_name(cr));
                sess->peer_connected = false;
                smoke_fanout_detach(sess);
                if (sess->running) {
                    sess->auto_cleanup_requested = true;
                    sess->running = false;
                }
            }
        } else if (viewers > 1U) {
            SMK("sid=%u joined existing H264 stream encoder_count=1 viewers=%u",
                (unsigned)sess->session_id, (unsigned)viewers);
        }

        if (sess->peer_connected && !camera_webrtc_audio_is_active()) {
            SMKW("[DMA-DIAG D1 before_audio_start] free=%u largest=%u",
                 (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA),
                 (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA));
            esp_err_t ar = camera_webrtc_audio_start(smoke_fanout_opus, NULL);
            if (ar == ESP_OK) {
                SMK("camera audio start requested: ES8311 -> Opus %uHz mono",
                    (unsigned)CAM_WEBRTC_AUDIO_SAMPLE_RATE);
            } else {
                SMKW("camera audio start failed: %s; video remains active", esp_err_to_name(ar));
            }
        }
        break;
    }
    case ESP_PEER_STATE_CONNECT_FAILED:
        sess->dtls_connecting = false;
        sess->dtls_start_ms = 0;
        sess->peer_connected = false;
        smoke_fanout_detach(sess);
        if (smoke_fanout_count() == 0U) { camera_h264_stream_request_stop(); camera_webrtc_audio_request_stop(); }
        SMKE("sid=%u peer connect failed", (unsigned)sess->session_id);
        if (sess->running) {
            sess->auto_cleanup_requested = true;
            sess->running = false;
        }
        break;
    case ESP_PEER_STATE_DISCONNECTED:
        sess->dtls_connecting = false;
        sess->dtls_start_ms = 0;
        sess->peer_connected = false;
        smoke_fanout_detach(sess);
        if (smoke_fanout_count() == 0U) { camera_h264_stream_request_stop(); camera_webrtc_audio_request_stop(); }
        SMK("sid=%u disconnected", (unsigned)sess->session_id);
        if (sess->running) {
            sess->auto_cleanup_requested = true;
            sess->running = false;
        }
        break;
    case ESP_PEER_STATE_CLOSED:
        sess->dtls_connecting = false;
        sess->dtls_start_ms = 0;
        sess->peer_connected = false;
        smoke_fanout_detach(sess);
        if (smoke_fanout_count() == 0U) { camera_h264_stream_request_stop(); camera_webrtc_audio_request_stop(); }
        SMK("sid=%u closed", (unsigned)sess->session_id);
        if (sess->running) {
            sess->auto_cleanup_requested = true;
            sess->running = false;
        }
        break;
    default:
        break;
    }
    return 0;
}

static int smoke_on_msg(esp_peer_msg_t *info, void *ctx)
{
    smoke_sess_t *s = (smoke_sess_t *)ctx;
    if (!info || !info->data || info->size <= 0) return 0;

    if (info->type == ESP_PEER_MSG_TYPE_SDP) {
        char *sdp = (char *)heap_caps_malloc(info->size + 1, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!sdp) sdp = (char *)malloc(info->size + 1);
        if (!sdp) { SMKE("oom capturing local SDP"); return 0; }
        memcpy(sdp, info->data, info->size);
        sdp[info->size] = '\0';

        /* [TURN-RELAY-EMIT-SAFE-AB-20260831]
         * This callback runs from esp_peer_main_loop() while smoke_pump owns
         * peer_mtx.  Publish only scalar diagnostic state here.  The HTTP task
         * polls local_has_relay and never dereferences local_sdp outside
         * peer_mtx, avoiding the UAF in the earlier count-based wait patch. */
        bool sdp_has_relay = (strstr(sdp, " typ relay") != NULL);
        uint32_t emit_no = __atomic_add_fetch(&s->local_sdp_emit_count, 1U, __ATOMIC_RELAXED);
        if (sdp_has_relay) {
            __atomic_store_n(&s->local_has_relay, true, __ATOMIC_RELEASE);
        }

        free(s->local_sdp);
        s->local_sdp = sdp;

        char setup[24]; parse_setup(sdp, setup, sizeof(setup));
        SMK("local %s setup=%s", s->is_offerer ? "offer" : "answer", setup[0] ? setup : "(none)");
        SMK("DTLS role=%s", setup_to_dtls_role(setup));
        SMK("sid=%u [TURN-RELAY-EMIT-AB] local SDP emit=%u embedded=%d relay=%d",
            (unsigned)s->session_id, (unsigned)emit_no,
            count_sdp_candidate_lines(sdp), sdp_has_relay ? 1 : 0);
        xSemaphoreGive(s->sdp_ready);
    } else if (info->type == ESP_PEER_MSG_TYPE_CANDIDATE) {
        /* info->data 는 "candidate:..." → SDP 라인은 "a=candidate:..." */
        uint32_t msg_no = __atomic_add_fetch(&s->local_candidate_msg_count, 1U, __ATOMIC_RELAXED);
        if (s->cand_n < SMOKE_MAX_CAND) {
            int len = info->size;
            if (len > SMOKE_CAND_LEN - 4) len = SMOKE_CAND_LEN - 4;
            snprintf(s->cand[s->cand_n], SMOKE_CAND_LEN, "a=%.*s", len, (const char *)info->data);
            bool cand_has_relay = (strstr(s->cand[s->cand_n], " typ relay") != NULL);
            if (cand_has_relay) {
                __atomic_store_n(&s->local_has_relay, true, __ATOMIC_RELEASE);
            }
            SMK("sid=%u [TURN-RELAY-EMIT-AB] candidate msg=%u relay=%d cand_n=%d",
                (unsigned)s->session_id, (unsigned)msg_no,
                cand_has_relay ? 1 : 0, s->cand_n + 1);
            s->cand_n++;
        }
    }
    return 0;
}

/* ───────────── pump 태스크 (Core0) ───────────── */

static bool smoke_teardown_locked(smoke_sess_t *sess); /* forward: s_gate held */

/* lifetime 만료 시 pump가 직접 자기 세션을 free하지 않는다.
 * 별도 cleanup task가 글로벌 s_active를 s_gate 아래서 안전하게 회수한다. */
static void smoke_auto_cleanup(void *arg)
{
    uint32_t sid = (uint32_t)(uintptr_t)arg;
    bool cleaned_or_empty = false;
    for (int attempt = 0; attempt < 8; ++attempt) {
        if (xSemaphoreTake(s_gate, pdMS_TO_TICKS(3000)) == pdTRUE) {
            smoke_sess_t *sess = smoke_find_session_locked(sid);
            if (!sess) {
                cleaned_or_empty = true;
                xSemaphoreGive(s_gate);
                break;
            }
            cleaned_or_empty = smoke_teardown_locked(sess);
            xSemaphoreGive(s_gate);
            if (cleaned_or_empty) break;
        }
        vTaskDelay(pdMS_TO_TICKS(250));
    }
    if (!cleaned_or_empty) {
        SMKE("sid=%u auto cleanup deferred", (unsigned)sid);
    }
    vTaskDelete(NULL);
}

static void smoke_pump(void *arg)
{
    smoke_sess_t *s = (smoke_sess_t *)arg;
    SMK("sid=%u pump task started core=%d stack=%d", (unsigned)s->session_id, xPortGetCoreID(), SMOKE_PUMP_STACK);
    while (s->running) {
        bool did_tx = false;
        if (xSemaphoreTake(s->peer_mtx, pdMS_TO_TICKS(100)) == pdTRUE) {
            if (s->peer) esp_peer_main_loop(s->peer);
            /* Send one previously-copied H.264 AU after main_loop. The camera
             * is free to queue the next encoded frame while this SRTP/UDP send
             * is in progress. */
            if (s->peer && s->running && s->peer_connected) {
                /* [CAM-AUDIO-PUMP-JITTER-FIX-V1-20260902]
                 * Audio is 20ms/50fps while H264 send can occupy peer_mtx for
                 * tens of ms.  Preserve audio continuity by draining a bounded
                 * backlog first, then send exactly one video AU.
                 *
                 * Bounded at 4: audio cannot starve the established H264 path.
                 * With the 8-slot queue this tolerates ~160ms transient stalls. */
                bool did_audio = false;
                for (uint32_t k = 0; k < 4U; ++k) {
                    if (!smoke_audio_send_one_locked(s)) break;
                    did_audio = true;
                }
                bool did_video = smoke_tx_send_one_locked(s);
                did_tx = did_audio || did_video;
            }
            xSemaphoreGive(s->peer_mtx);
        }
        int64_t now_ms = esp_timer_get_time() / 1000;

        /* [DTLS-FAST-RECOVERY-CD78]
         * If ICE is already paired but DTLS never reaches CONNECTED, the peer can
         * sit in repeated agent_recv timeouts until the user presses Stop. Do not
         * tear down while inside esp_peer_main_loop / peer_mtx. Instead stop this
         * pump and reuse the existing safe auto-cleanup path after unwinding. */
        if (s->running && !s->peer_connected && s->dtls_connecting &&
            s->dtls_start_ms > 0 &&
            now_ms - s->dtls_start_ms >= (int64_t)SMOKE_DTLS_CONNECT_TIMEOUT_MS) {
            int64_t elapsed = now_ms - s->dtls_start_ms;
            SMKE("sid=%u [DTLS-WATCHDOG] timeout after %lldms -> safe auto cleanup",
                 (unsigned)s->session_id, (long long)elapsed);
            s->dtls_connecting = false;
            s->dtls_start_ms = 0;
            s->auto_cleanup_requested = true;
            s->running = false;
        }

        /* If backlog exists, loop again almost immediately; otherwise retain
         * the old pacing so ICE/DTLS maintenance behavior stays conservative. */
        vTaskDelay(pdMS_TO_TICKS((did_tx || smoke_tx_count(s) > 0 || smoke_audio_count(s) > 0) ? 1 : 10));
    }

    /* A real peer loss requests cleanup from the state callback, but the callback
     * runs inside esp_peer_main_loop while peer_mtx is held. Therefore cleanup is
     * spawned only after this pump has returned from esp_peer and released peer_mtx. */
    bool need_auto_cleanup = s->auto_cleanup_requested;
    if (need_auto_cleanup) {
        SMK("sid=%u peer ended -> safe auto cleanup", (unsigned)s->session_id);
        if (xTaskCreatePinnedToCore(smoke_auto_cleanup, "smoke_cleanup", SMOKE_CLEANUP_STACK,
                                    (void *)(uintptr_t)s->session_id,
                                    SMOKE_CLEANUP_PRIO, NULL, SMOKE_PUMP_CORE) != pdPASS) {
            SMKE("auto cleanup task create failed; stopped session will be reaped on next /stop or /offer");
        }
    }

    /* 이 give 이후에는 s를 절대로 다시 참조하지 않는다. teardown이 free해도 안전하다. */
    xSemaphoreGive(s->done);
    vTaskDelete(NULL);
}

/* ───────────── 세션 수명주기 (s_gate 안에서 호출) ───────────── */

/*
 * true  = 완전히 정리됨(또는 활성 세션 없음)
 * false = pump/mutex가 아직 안전한 종료점에 도달하지 않아 free를 보류함
 *
 * 중요: timeout 시 peer/semaphore/s를 절대 free하지 않는다.
 */
static bool smoke_teardown_locked(smoke_sess_t *sess)
{
    if (!sess) return true;

    uint32_t sid = sess->session_id;
    sess->peer_connected = false;
    sess->dtls_connecting = false;
    sess->dtls_start_ms = 0;
    smoke_fanout_detach(sess);

    uint32_t viewers_left = smoke_fanout_count();
    if (viewers_left == 0U && camera_h264_stream_is_active()) {
        camera_h264_stream_request_stop();
        esp_err_t cam_stop = camera_h264_stream_stop(4500);
        if (cam_stop != ESP_OK) {
            SMK("sid=%u teardown deferred: camera stopping (%s)",
                (unsigned)sid, esp_err_to_name(cam_stop));
            return false;
        }
    }
    if (viewers_left == 0U && esp_h264_enc_hw_ref_pool_ready()) {
        const size_t ref_df0 = heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
        const size_t ref_dl0 = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
        const bool ref_released = esp_h264_enc_hw_ref_pool_release();
        const size_t ref_df1 = heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
        const size_t ref_dl1 = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
        SMKW("[HANDOVER-DMA AB1] sid=%u ref_pool_release=%d dma=%u/%u -> %u/%u",
             (unsigned)sid, ref_released ? 1 : 0,
             (unsigned)ref_df0, (unsigned)ref_dl0,
             (unsigned)ref_df1, (unsigned)ref_dl1);
    }
    if (viewers_left == 0U) {
        camera_webrtc_audio_request_stop();
        esp_err_t aud_stop = camera_webrtc_audio_stop(1500);
        SMKW("[DMA-DIAG D8 after_audio_stop] sid=%u result=%s free=%u largest=%u",
             (unsigned)sid, esp_err_to_name(aud_stop),
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA),
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA));
        if (aud_stop != ESP_OK) {
            SMKW("sid=%u audio stop wait=%s; task retains ownership until exit",
                 (unsigned)sid, esp_err_to_name(aud_stop));
        }
    }

    sess->running = false;

    if (!sess->pump_done_seen) {
        if (!sess->done || xSemaphoreTake(sess->done, pdMS_TO_TICKS(2000)) != pdTRUE) {
            SMK("sid=%u teardown deferred: pump alive", (unsigned)sid);
            return false;
        }
        sess->pump_done_seen = true;
    }

    /* [DTLS-HANDSHAKE-WINDOW-C2-OWNER-HARDEN-20260908]
     * The pump is now guaranteed stopped.  Disarm only this session's window
     * before esp_peer_close() can invalidate the embedded udp_socket_t.
     * Owner matching prevents primary teardown from clearing a secondary
     * viewer that is currently handshaking. */
    turn_gather_rx_ab_disarm(sid);

    if (sess->peer_mtx) {
        if (xSemaphoreTake(sess->peer_mtx, pdMS_TO_TICKS(2000)) != pdTRUE) {
            SMK("sid=%u teardown deferred: peer mutex busy", (unsigned)sid);
            return false;
        }
        if (sess->peer) {
            int r = esp_peer_close(sess->peer);
            SMK("sid=%u esp_peer_close r=%d", (unsigned)sid, r);
            SMKW("[DMA-DIAG D9 after_esp_peer_close] sid=%u result=%d free=%u largest=%u",
                 (unsigned)sid, r,
                 (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA),
                 (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA));
            sess->peer = NULL;
        }
        xSemaphoreGive(sess->peer_mtx);
    } else if (sess->peer) {
        SMKE("sid=%u teardown deferred: peer exists without mutex", (unsigned)sid);
        return false;
    }

    if (!smoke_fanout_wait_idle(sess, 1000)) {
        SMK("sid=%u teardown deferred: fanout ref still active", (unsigned)sid);
        return false;
    }

    if (s_active == sess) {
        s_active = NULL;
        /* If viewer #1 leaves first, promote viewer #2 so a new second viewer
         * may later join the still-running single camera stream. */
        if (s_secondary) {
            s_active = s_secondary;
            s_secondary = NULL;
            SMK("sid=%u promoted secondary -> primary", (unsigned)s_active->session_id);
        }
    } else if (s_secondary == sess) {
        s_secondary = NULL;
    }

    smoke_tx_pool_free(sess);
    if (sess->sdp_ready) vSemaphoreDelete(sess->sdp_ready);
    if (sess->done)      vSemaphoreDelete(sess->done);
    if (sess->peer_mtx)  vSemaphoreDelete(sess->peer_mtx);
    free(sess->local_sdp);
    free(sess);

    SMK("sid=%u session torn down viewers_left=%u",
        (unsigned)sid, (unsigned)viewers_left);
    return true;
}

static void smoke_reap_stale_locked(void)
{
    /* [STALE-CAM-REAP-20260831]
     * A session that has already delivered media cannot remain a valid
     * "running primary" after the single camera/H264 producer has stopped.
     * Without this guard, a dead peer can keep s_active occupied forever and
     * every later POST /api/smoke/offer is rejected as primary_connecting
     * before "=== offer received ===" is logged.
     *
     * media_sent > 0 avoids touching the normal first-connect window where
     * DTLS is ready but camera_h264_stream_start() has not become active yet.
     * Reap at most SMOKE_MAX_SESSIONS entries so primary promotion is handled
     * in the same request without an unbounded loop.
     */
    for (uint32_t pass = 0; pass < SMOKE_MAX_SESSIONS; ++pass) {
        smoke_sess_t *victim = NULL;
        const bool cam_active = camera_h264_stream_is_active();

        if (s_active) {
            const bool camera_orphan =
                s_active->running &&
                s_active->media_sent > 0U &&
                !cam_active;
            if (!s_active->running || camera_orphan) {
                if (camera_orphan) {
                    SMK("sid=%u [STALE-CAM-REAP] primary running but camera inactive "
                        "media_sent=%u -> force reap",
                        (unsigned)s_active->session_id,
                        (unsigned)s_active->media_sent);
                    s_active->auto_cleanup_requested = false;
                    s_active->running = false;
                } else {
                    SMK("sid=%u stale primary -> reap",
                        (unsigned)s_active->session_id);
                }
                victim = s_active;
            }
        }

        if (!victim && s_secondary) {
            const bool camera_orphan =
                s_secondary->running &&
                s_secondary->media_sent > 0U &&
                !cam_active;
            if (!s_secondary->running || camera_orphan) {
                if (camera_orphan) {
                    SMK("sid=%u [STALE-CAM-REAP] secondary running but camera inactive "
                        "media_sent=%u -> force reap",
                        (unsigned)s_secondary->session_id,
                        (unsigned)s_secondary->media_sent);
                    s_secondary->auto_cleanup_requested = false;
                    s_secondary->running = false;
                } else {
                    SMK("sid=%u stale secondary -> reap",
                        (unsigned)s_secondary->session_id);
                }
                victim = s_secondary;
            }
        }

        if (!victim) break;
        if (!smoke_teardown_locked(victim)) {
            SMK("sid=%u [STALE-CAM-REAP] teardown deferred",
                (unsigned)victim->session_id);
            break;
        }
    }
}

/* First peer selects the camera profile. The second peer may join only after
 * the primary peer is connected and the single camera/H264 stream is active. */
/* [HANDOVER-DUAL-RELAY-AB2 2026-09-07] TEST ONLY: h_offer_post holds s_gate while this one-shot flag is consumed. */
static bool s_handover_dual_relay_ab2_next = false;
static esp_err_t smoke_start_locked(bool offerer, char mode,
                                    camera_h264_profile_t profile,
                                    smoke_sess_t **out_sess)
{
    if (out_sess) *out_sess = NULL;
    bool secondary = (s_active != NULL);

    if (secondary) {
        if (s_secondary) return ESP_ERR_NO_MEM;
        if (s_active->profile != profile) return ESP_ERR_INVALID_ARG;
        /* [DTLS-HANDSHAKE-WINDOW-C2-OWNER-HARDEN-20260908]
         * Load-bearing invariant: a second viewer may start only after the
         * primary is fully CONNECTED.  The owner guard in the timeout override
         * additionally fails closed if this invariant is ever regressed. */
        if (!s_active->peer_connected || !camera_h264_stream_is_active()) {
            return ESP_ERR_INVALID_STATE;
        }
    } else {
        esp_err_t pr = camera_h264_stream_set_profile(profile);
        if (pr != ESP_OK) {
            SMKE("camera profile select failed profile=%s err=%s",
                 camera_h264_stream_profile_name(profile), esp_err_to_name(pr));
            return pr;
        }

        esp_err_t tr = isp_tune_apply_profile(profile);
        if (tr != ESP_OK) {
            SMKW("camera profile tune apply failed profile=%s err=%s; continuing",
                 camera_h264_stream_profile_name(profile), esp_err_to_name(tr));
        }

        /* [CAM-COLOR2-PER-PROFILE-20260910]
         * Apply Hue/WB from the same selected profile in backend, so correct
         * color does not depend on browser request timing. */
        esp_err_t c2r = camera_color_stage2_apply_profile(
                            profile == CAMERA_H264_PROFILE_720P_NATIVE);
        if (c2r != ESP_OK) {
            SMKW("camera Color2 profile apply failed profile=%s err=%s; continuing",
                 camera_h264_stream_profile_name(profile), esp_err_to_name(c2r));
        }
    }

    if (!s_cert_prepared) {
        size_t free0 = esp_get_free_heap_size();
        size_t dma0 = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
        SMK("DTLS cert pregen begin free_heap=%u internal_dma_largest=%u",
            (unsigned)free0, (unsigned)dma0);
        int cr = esp_peer_pre_generate_cert();
        size_t free1 = esp_get_free_heap_size();
        size_t dma1 = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
        SMK("DTLS cert pregen end r=%d free_heap=%u internal_dma_largest=%u",
            cr, (unsigned)free1, (unsigned)dma1);
        if (cr != ESP_PEER_ERR_NONE) return ESP_FAIL;
        s_cert_prepared = true;
    }

    size_t freeheap = esp_get_free_heap_size();
    size_t dmablk = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
    SMK("peer preflight index=%u free_heap=%u internal_dma_largest=%u camera_active=%u",
        secondary ? 2U : 1U, (unsigned)freeheap, (unsigned)dmablk,
        (unsigned)camera_h264_stream_is_active());

    smoke_sess_t *sess = (smoke_sess_t *)heap_caps_calloc(
        1, sizeof(smoke_sess_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!sess) {
        sess = (smoke_sess_t *)calloc(1, sizeof(smoke_sess_t));
        SMK("session alloc fallback=internal size=%u", (unsigned)sizeof(smoke_sess_t));
    } else {
        SMK("session alloc=PSRAM size=%u", (unsigned)sizeof(smoke_sess_t));
    }
    if (!sess) return ESP_ERR_NO_MEM;

    uint32_t sid = s_next_session_id++;
    if (sid == 0U) sid = s_next_session_id++;
    sess->session_id = sid;
    sess->is_offerer = offerer;
    sess->mode = mode;
    sess->profile = profile;
    if (!secondary) {
        /* [CAM-1080-ADAPT-GOP-R2-20260912] Reset controller for every fresh primary. The camera profile
         * setter has already restored the 1080 GOP5 baseline. */
        smoke_adapt_reset_primary_locked();
    }
    sess->start_ms = esp_timer_get_time() / 1000;
    sess->peer_mtx  = xSemaphoreCreateMutex();
    sess->sdp_ready = xSemaphoreCreateBinary();
    sess->done      = xSemaphoreCreateBinary();
    if (!sess->peer_mtx || !sess->sdp_ready || !sess->done) goto fail;
    if (!smoke_tx_pool_init(sess)) goto fail;

    esp_peer_default_cfg_t peer_default_cfg = {
        .agent_recv_timeout = SMOKE_AGENT_RECV_TIMEOUT_MS,
        /* [ANDROID-ICE-LITE-AB-20260830]
         * Production uses Browser-offer -> ESP-answer.
         * ESP is an always-on LAN endpoint, so answerer mode uses ICE Lite.
         * This makes the browser/full ICE agent drive connectivity checks
         * toward the ESP host candidate instead of relying on ESP-initiated
         * checks toward Android's ephemeral UDP socket. */
        .ice_use_lite_mode = false, /* A/B: production answerer also FULL ICE; isolate ICE-Lite pairing asymmetry */
    };

    esp_peer_cfg_t cfg = {0};
    cfg.role                = offerer ? ESP_PEER_ROLE_CONTROLLING : ESP_PEER_ROLE_CONTROLLED;
    cfg.ice_trans_policy    = s_handover_dual_relay_ab2_next ? ESP_PEER_ICE_TRANS_POLICY_RELAY : ESP_PEER_ICE_TRANS_POLICY_ALL;
    SMK("sid=%u [HANDOVER-DUAL-RELAY-AB2] ESP ICE policy=%s",
        (unsigned)sess->session_id, s_handover_dual_relay_ab2_next ? "RELAY" : "ALL");
    cfg.server_lists        = s_smoke_ice_servers;
    cfg.server_num          = (uint8_t)(sizeof(s_smoke_ice_servers) / sizeof(s_smoke_ice_servers[0]));
    cfg.video_info.codec    = ESP_PEER_VIDEO_CODEC_H264;
    uint32_t video_w = 0, video_h = 0, video_fps = 0;
    smoke_profile_video_info(profile, &video_w, &video_h, &video_fps);
    cfg.video_info.width    = video_w;
    cfg.video_info.height   = video_h;
    cfg.video_info.fps      = video_fps;
    cfg.video_dir           = ESP_PEER_MEDIA_DIR_SEND_ONLY;
    cfg.audio_dir           = ESP_PEER_MEDIA_DIR_SEND_ONLY;
    cfg.audio_info.codec    = ESP_PEER_AUDIO_CODEC_OPUS;
    cfg.audio_info.sample_rate = CAM_WEBRTC_AUDIO_SAMPLE_RATE;
    cfg.audio_info.channel  = CAM_WEBRTC_AUDIO_CHANNELS;
    cfg.no_auto_reconnect   = true;
    cfg.enable_data_channel = false;
    cfg.on_state            = smoke_on_state;
    cfg.on_msg              = smoke_on_msg;
    cfg.ctx                 = sess;
    cfg.extra_cfg           = &peer_default_cfg;
    cfg.extra_size          = sizeof(peer_default_cfg);

    SMK("sid=%u WebRTC %s %ux%u@%u H264 + Opus %uHz mono; encoder_count=1",
        (unsigned)sid, camera_h264_stream_profile_name(profile),
        (unsigned)video_w, (unsigned)video_h, (unsigned)video_fps,
        (unsigned)CAM_WEBRTC_AUDIO_SAMPLE_RATE);
    SMK("sid=%u ICE mode=%s role=%s STUN=%s TURN=%s servers=%u",
        (unsigned)sid,
        peer_default_cfg.ice_use_lite_mode ? "LITE" : "FULL",
        offerer ? "offerer/controlling" : "answerer/controlled",
        SMOKE_STUN_URL, SMOKE_TURN_URL, (unsigned)cfg.server_num);

    int r = esp_peer_open(&cfg, esp_peer_get_default_impl(), &sess->peer);
    if (r != ESP_PEER_ERR_NONE || !sess->peer) {
        SMKE("sid=%u esp_peer_open failed r=%d", (unsigned)sid, r);
        goto fail;
    }

    sess->running = true;
    if (secondary) s_secondary = sess;
    else s_active = sess;

    if (xTaskCreatePinnedToCore(smoke_pump, "smoke_pump", SMOKE_PUMP_STACK, sess,
                                SMOKE_PUMP_PRIO, &sess->pump, SMOKE_PUMP_CORE) != pdPASS) {
        SMKE("sid=%u pump task create failed", (unsigned)sid);
        if (s_active == sess) s_active = NULL;
        if (s_secondary == sess) s_secondary = NULL;
        sess->running = false;
        esp_peer_close(sess->peer);
        sess->peer = NULL;
        goto fail;
    }

    SMK("sid=%u peer opened viewers_allocated=%u/%u role=%s",
        (unsigned)sid, (unsigned)smoke_session_count_locked(),
        (unsigned)SMOKE_MAX_SESSIONS,
        offerer ? "offerer" : "answerer");
    if (out_sess) *out_sess = sess;
    return ESP_OK;

fail:
    smoke_tx_pool_free(sess);
    if (sess->sdp_ready) vSemaphoreDelete(sess->sdp_ready);
    if (sess->done)      vSemaphoreDelete(sess->done);
    if (sess->peer_mtx)  vSemaphoreDelete(sess->peer_mtx);
    free(sess);
    return ESP_FAIL;
}

/* peer 호출 helper (mutex) */
static int smoke_peer_new_connection(smoke_sess_t *s)
{
    int r = ESP_PEER_ERR_FAIL;
    if (xSemaphoreTake(s->peer_mtx, pdMS_TO_TICKS(2000)) == pdTRUE) {
        if (s->peer) {
            /* [DTLS-HANDSHAKE-WINDOW-C2-OWNER-HARDEN-20260908]
             * Bind the single transient timeout window to this exact session
             * before esp_peer starts gather.  If another session still owns a
             * window, fail closed instead of silently clobbering it. */
            if (turn_gather_rx_ab_begin_session(s->session_id)) {
                r = esp_peer_new_connection(s->peer);
                if (r != ESP_PEER_ERR_NONE) {
                    turn_gather_rx_ab_disarm(s->session_id);
                }
            } else {
                SMKE("sid=%u [DTLS-C2-HARDEN] timeout-window owner busy",
                     (unsigned)s->session_id);
            }
        }
        xSemaphoreGive(s->peer_mtx);
    }
    return r;
}
static int smoke_peer_send_sdp(smoke_sess_t *s, const char *sdp)
{
    int r = ESP_PEER_ERR_FAIL;
    esp_peer_msg_t msg = { .type = ESP_PEER_MSG_TYPE_SDP,
                           .data = (uint8_t *)sdp, .size = (int)strlen(sdp) };
    if (xSemaphoreTake(s->peer_mtx, pdMS_TO_TICKS(2000)) == pdTRUE) {
        if (s->peer) r = esp_peer_send_msg(s->peer, &msg);
        xSemaphoreGive(s->peer_mtx);
    }
    return r;
}


/* [ANDROID-ICE-CANDIDATE-DEDUP-AB-20260830]
 * Browser localDescription already contains a=candidate lines. esp_peer's SDP
 * parser consumes those candidates while processing ESP_PEER_MSG_TYPE_SDP.
 *
 * Keep this A/B to one variable: embedded-SDP candidates only.
 * Do NOT inject the same candidate again as ESP_PEER_MSG_TYPE_CANDIDATE.
 * No camera/H264/ISP/ICE-Lite/peer-role/DTLS-watchdog/UDP-probe settings change. */
static int smoke_peer_send_remote_description(smoke_sess_t *s, const char *sdp)
{
    const int embedded = count_sdp_candidate_lines(sdp);
    int r = smoke_peer_send_sdp(s, sdp);
    SMK("remote SDP inject r=%d embedded_candidates=%d candidate_mode=SDP_ONLY",
        r, embedded);
    if (r != ESP_PEER_ERR_NONE) return r;
    if (embedded == 0) {
        SMK("warning: remote SDP contained no ICE candidate lines");
    }
    return ESP_PEER_ERR_NONE;
}

/* ───────────── HTTP 유틸 ───────────── */

static char *read_body(httpd_req_t *req)
{
    int total = req->content_len;
    if (HA_VERBOSE_DIAG_LOGS) SMK("HTTP body len=%d (max=%d)", total, SMOKE_BODY_MAX);
    if (total <= 0 || total > SMOKE_BODY_MAX) return NULL;
    char *buf = (char *)malloc(total + 1);
    if (!buf) return NULL;
    int off = 0;
    while (off < total) {
        int r = httpd_req_recv(req, buf + off, total - off);
        if (r <= 0) { free(buf); return NULL; }
        off += r;
    }
    buf[off] = '\0';
    return buf;
}

static esp_err_t send_json(httpd_req_t *req, const char *status, cJSON *obj)
{
    char *txt = cJSON_PrintUnformatted(obj);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_status(req, status);
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    esp_err_t e = httpd_resp_sendstr(req, txt ? txt : "{}");
    if (txt) cJSON_free(txt);
    return e;
}

static esp_err_t send_err(httpd_req_t *req, const char *status, const char *err)
{
    cJSON *o = cJSON_CreateObject();
    cJSON_AddStringToObject(o, "error", err);
    esp_err_t e = send_json(req, status, o);
    cJSON_Delete(o);
    return e;
}

static void add_diag(cJSON *root, smoke_sess_t *s)
{
    cJSON *d = cJSON_CreateObject();
    cJSON_AddStringToObject(d, "esp_peer_version", SMOKE_ESP_PEER_VER);
    cJSON_AddStringToObject(d, "esp_peer_hash8", SMOKE_ESP_PEER_HASH8);
    cJSON_AddStringToObject(d, "role", s->is_offerer ? "offerer" : "answerer");
    char setup[24] = {0};
    if (s->local_sdp) parse_setup(s->local_sdp, setup, sizeof(setup));
    cJSON_AddStringToObject(d, "local_setup", setup[0] ? setup : "unknown");
    cJSON_AddStringToObject(d, "dtls_role", setup_to_dtls_role(setup));
    uint32_t vw = 0, vh = 0, vf = 0;
    smoke_profile_video_info(s->profile, &vw, &vh, &vf);
    char video_desc[112];
    snprintf(video_desc, sizeof(video_desc), "OV02C10 %ux%u@%u H264 GOP1 All-I QP30..40 auto-RC (%s)",
             (unsigned)vw, (unsigned)vh, (unsigned)vf,
             camera_h264_stream_profile_name(s->profile));
    cJSON_AddStringToObject(d, "video", video_desc);
    cJSON_AddStringToObject(d, "profile", camera_h264_stream_profile_name(s->profile));
    cJSON_AddNumberToObject(d, "media_sent", s->media_sent);
    cJSON_AddNumberToObject(d, "media_drop", s->media_drop);
    cJSON_AddStringToObject(d, "audio", "ES8311 Opus 24kHz mono 48kbps");
    cJSON_AddNumberToObject(d, "audio_sent", s->audio_sent);
    cJSON_AddNumberToObject(d, "audio_drop", s->audio_send_drop + s->audio_queue_drops);
    cJSON_AddItemToObject(root, "diag", d);
}

/* SDP 생성 대기 + candidate 수집창 → 합친 SDP 반환(호출자 free). 실패 시 NULL */
static char *await_local_sdp_aggregated(smoke_sess_t *s)
{
    SMK("sid=%u synchronous wait: local SDP + gather window up to ~%d ms",
        (unsigned)s->session_id, SMOKE_SDP_WAIT_MS + SMOKE_GATHER_MS);
    if (xSemaphoreTake(s->sdp_ready, pdMS_TO_TICKS(SMOKE_SDP_WAIT_MS)) != pdTRUE) {
        SMKE("sid=%u timeout waiting local SDP", (unsigned)s->session_id);
        return NULL;
    }
    vTaskDelay(pdMS_TO_TICKS(SMOKE_GATHER_MS)); /* direct/LAN path stays answer-early */

    /* [TURN-RELAY-EMIT-SAFE-AB-20260831]
     * One-variable A/B after Claude review: when the browser offer already has
     * a relay candidate, wait only for a callback-published scalar flag.
     * Never inspect s->local_sdp here: smoke_on_msg may replace/free it from
     * the pump callback.  Direct/LAN/P2P offers retain the proven 0-ms path. */
    if (!s->is_offerer && s->remote_has_relay) {
        int waited_ms = 0;
        bool relay_ready = __atomic_load_n(&s->local_has_relay, __ATOMIC_ACQUIRE);
        SMK("sid=%u [TURN-RELAY-EMIT-AB] wait begin max=%dms relay=%d sdp_emit=%u cand_msg=%u",
            (unsigned)s->session_id, SMOKE_TURN_RELAY_WAIT_MAX_MS, relay_ready ? 1 : 0,
            (unsigned)__atomic_load_n(&s->local_sdp_emit_count, __ATOMIC_RELAXED),
            (unsigned)__atomic_load_n(&s->local_candidate_msg_count, __ATOMIC_RELAXED));
        while (!relay_ready && waited_ms < SMOKE_TURN_RELAY_WAIT_MAX_MS) {
            vTaskDelay(pdMS_TO_TICKS(SMOKE_TURN_RELAY_WAIT_POLL_MS));
            waited_ms += SMOKE_TURN_RELAY_WAIT_POLL_MS;
            relay_ready = __atomic_load_n(&s->local_has_relay, __ATOMIC_ACQUIRE);
            if (relay_ready || (waited_ms % 100) == 0) {
                SMK("sid=%u [TURN-RELAY-EMIT-AB] wait=%dms relay=%d sdp_emit=%u cand_msg=%u",
                    (unsigned)s->session_id, waited_ms, relay_ready ? 1 : 0,
                    (unsigned)__atomic_load_n(&s->local_sdp_emit_count, __ATOMIC_RELAXED),
                    (unsigned)__atomic_load_n(&s->local_candidate_msg_count, __ATOMIC_RELAXED));
            }
        }
        SMK("sid=%u [TURN-RELAY-EMIT-AB] wait end=%dms relay=%d",
            (unsigned)s->session_id, waited_ms, relay_ready ? 1 : 0);
    }
    char *agg = NULL;
    if (xSemaphoreTake(s->peer_mtx, pdMS_TO_TICKS(2000)) == pdTRUE) {
        int embedded = count_sdp_candidate_lines(s->local_sdp);
        agg = build_aggregated_sdp(s);
        int total = agg ? count_sdp_candidate_lines(agg) : 0;
        int agg_has_relay = (agg && strstr(agg, " typ relay") != NULL) ? 1 : 0;
        SMK("sid=%u local SDP candidates embedded=%d trickle=%d total=%d relay=%d role=%s ice=%s",
            (unsigned)s->session_id, embedded, s->cand_n, total, agg_has_relay,
            s->is_offerer ? "offerer" : "answerer",
            "FULL");
        if (total == 0) {
            SMKE("sid=%u local SDP has NO ICE candidate; browser cannot initiate LAN ICE",
                 (unsigned)s->session_id);
        }
        xSemaphoreGive(s->peer_mtx);
    }
    return agg;
}

/* [CAM-ISP-NVS-STACK-FIX-20260830]
 * Do not seed ISP/NVS state from GOT_IP/sys_evt.
 * Restore lazily from the HTTP server task instead. */
static void isp_tune_restore_saved_settings_once(void);

/* ───────────── HTTP 핸들러 ───────────── */

/* TEST1: POST /api/smoke/offer  {mode:"A"|"B", type:"offer", sdp}
 *   ESP = answerer. 역할 강제 없음. esp_peer 가 고른 answer setup 을 관찰/반환. */
/* [ANDROID-ICE-LITE-AB] Production signaling path:
 * Browser creates offer (full ICE) -> ESP answers as ICE Lite.
 * This also avoids the known esp_peer browser-answer mid=255 parser path. */
static esp_err_t h_offer_post(httpd_req_t *req)
{
    isp_tune_restore_saved_settings_once();
    char *body = read_body(req);
    if (!body) return send_err(req, "400 Bad Request", "bad_body");

    cJSON *in = cJSON_Parse(body);
    free(body);
    if (!in) return send_err(req, "400 Bad Request", "bad_json");

    const cJSON *jsdp = cJSON_GetObjectItem(in, "sdp");
    const cJSON *jmode = cJSON_GetObjectItem(in, "mode");
    const cJSON *jprofile = cJSON_GetObjectItem(in, "profile");
    const cJSON *jclient_token = cJSON_GetObjectItem(in, "client_token");
    const cJSON *jesp_relay_only = cJSON_GetObjectItem(in, "esp_relay_only");
    const bool esp_relay_only = cJSON_IsTrue(jesp_relay_only);
    char mode = (cJSON_IsString(jmode) && jmode->valuestring[0]) ? jmode->valuestring[0] : '?';
    camera_h264_profile_t profile = smoke_profile_from_text(
        cJSON_IsString(jprofile) ? jprofile->valuestring : NULL);
    const char *client_token = NULL;
    if (jclient_token) {
        if (!cJSON_IsString(jclient_token) ||
            !smoke_client_token_valid(jclient_token->valuestring)) {
            cJSON_Delete(in);
            return send_err(req, "400 Bad Request", "bad_client_token");
        }
        client_token = jclient_token->valuestring;
    }
    if (!cJSON_IsString(jsdp) || !jsdp->valuestring[0]) {
        cJSON_Delete(in);
        return send_err(req, "400 Bad Request", "no_sdp");
    }

    char remote_setup[24];
    parse_setup(jsdp->valuestring, remote_setup, sizeof(remote_setup));

    if (xSemaphoreTake(s_gate, pdMS_TO_TICKS(500)) != pdTRUE) {
        cJSON_Delete(in);
        return send_err(req, "409 Conflict", "session_gate_busy");
    }
    smoke_reap_stale_locked();

    /* A retry from the SAME browser page may arrive after its previous /offer
     * created an ESP session but the HTTP answer (and therefore session_id) was
     * lost. Reclaim only that exact token while holding s_gate, BEFORE normal
     * viewer-count/primary_connecting checks. Never touch another viewer. */
    if (client_token) {
        smoke_sess_t *owned = smoke_find_client_token_locked(client_token);
        if (owned) {
            if (owned->peer_connected) {
                xSemaphoreGive(s_gate);
                cJSON_Delete(in);
                return send_err(req, "409 Conflict", "client_session_active");
            }
            SMK("sid=%u [CLIENT-ORPHAN-RECLAIM] exact token %.8s -> teardown",
                (unsigned)owned->session_id, client_token);
            owned->auto_cleanup_requested = false;
            if (!smoke_teardown_locked(owned)) {
                xSemaphoreGive(s_gate);
                cJSON_Delete(in);
                return send_err(req, "409 Conflict", "client_orphan_cleanup_deferred");
            }
        }
    }

    uint32_t count = smoke_session_count_locked();
    if (count >= SMOKE_MAX_SESSIONS) {
        xSemaphoreGive(s_gate);
        cJSON_Delete(in);
        return send_err(req, "409 Conflict", "max_2_viewers");
    }
    if (s_active) {
        if (s_active->profile != profile) {
            xSemaphoreGive(s_gate);
            cJSON_Delete(in);
            return send_err(req, "409 Conflict", "profile_in_use");
        }
        /* [DTLS-HANDSHAKE-WINDOW-C2-OWNER-HARDEN-20260908]
         * Load-bearing invariant: a second viewer may start only after the
         * primary is fully CONNECTED.  The owner guard in the timeout override
         * additionally fails closed if this invariant is ever regressed. */
        if (!s_active->peer_connected || !camera_h264_stream_is_active()) {
            xSemaphoreGive(s_gate);
            cJSON_Delete(in);
            return send_err(req, "409 Conflict", "primary_connecting");
        }
    }

    SMK("=== offer received profile=%s viewer=%u/%u ===",
        camera_h264_stream_profile_name(profile),
        (unsigned)(count + 1U), (unsigned)SMOKE_MAX_SESSIONS);
    SMK("remote offer setup=%s", remote_setup[0] ? remote_setup : "(none)");
    log_remote_candidates(jsdp->valuestring);

    const char *remote_sdp = jsdp->valuestring;
    char *remote_sdp_owned = NULL;
    char http_peer_ip[16] = {0};
    int lan_rewritten = 0;
    const int remote_candidate_count = count_sdp_candidate_lines(jsdp->valuestring);
    if (remote_candidate_count > 0 && smoke_http_peer_ipv4(req, http_peer_ip)) {
        remote_sdp_owned = smoke_rewrite_lan_candidates_ipv4(
            jsdp->valuestring, http_peer_ip, &lan_rewritten);
        if (remote_sdp_owned && lan_rewritten > 0) {
            remote_sdp = remote_sdp_owned;
            SMK("[LAN-CAND-FIX] HTTP peer=%s candidates=%d rewrote=%d",
                http_peer_ip, remote_candidate_count, lan_rewritten);
            log_remote_candidates(remote_sdp);
        } else if (remote_candidate_count == 1 && smoke_ipv4_is_private_lan(http_peer_ip)) {
            SMK("[LAN-CAND-FIX] single candidate already matches HTTP peer=%s", http_peer_ip);
        }
    }

    smoke_sess_t *sess = NULL;
    s_handover_dual_relay_ab2_next = esp_relay_only;
    esp_err_t sr0 = smoke_start_locked(false, mode, profile, &sess);
    s_handover_dual_relay_ab2_next = false;
    if (sr0 != ESP_OK || !sess) {
        xSemaphoreGive(s_gate);
        free(remote_sdp_owned);
        cJSON_Delete(in);
        if (sr0 == ESP_ERR_INVALID_ARG)
            return send_err(req, "409 Conflict", "profile_in_use");
        if (sr0 == ESP_ERR_INVALID_STATE)
            return send_err(req, "409 Conflict", "primary_connecting");
        if (sr0 == ESP_ERR_NO_MEM)
            return send_err(req, "503 Service Unavailable", "peer_resource_exhausted");
        return send_err(req, "503 Service Unavailable", "peer_open_failed");
    }

    if (client_token) {
        snprintf(sess->client_token, sizeof(sess->client_token), "%s", client_token);
        SMK("sid=%u [CLIENT-TOKEN] bound %.8s",
            (unsigned)sess->session_id, client_token);
    }

    sess->remote_has_relay = (strstr(remote_sdp, " typ relay") != NULL);
    if (sess->remote_has_relay) {
        SMK("sid=%u [TURN-RELAY-EMIT-AB] remote offer has relay -> safe wait enabled",
            (unsigned)sess->session_id);
    }

    int nr = smoke_peer_new_connection(sess);
    SMK("sid=%u esp_peer_new_connection r=%d", (unsigned)sess->session_id, nr);
    if (nr != ESP_PEER_ERR_NONE) {
        free(remote_sdp_owned);
        cJSON_Delete(in);
        (void)smoke_teardown_locked(sess);
        xSemaphoreGive(s_gate);
        return send_err(req, "503 Service Unavailable", "new_connection_failed");
    }

    int sr = smoke_peer_send_remote_description(sess, remote_sdp);
    free(remote_sdp_owned);
    remote_sdp_owned = NULL;
    cJSON_Delete(in);
    if (sr != ESP_PEER_ERR_NONE) {
        (void)smoke_teardown_locked(sess);
        xSemaphoreGive(s_gate);
        return send_err(req, "503 Service Unavailable", "remote_description_failed");
    }

    char *answer = await_local_sdp_aggregated(sess);
    if (!answer) {
        (void)smoke_teardown_locked(sess);
        xSemaphoreGive(s_gate);
        return send_err(req, "504 Gateway Timeout", "no_answer");
    }

    cJSON *out = cJSON_CreateObject();
    cJSON_AddStringToObject(out, "type", "answer");
    cJSON_AddStringToObject(out, "sdp", answer);
    cJSON_AddNumberToObject(out, "session_id", sess->session_id);
    cJSON_AddNumberToObject(out, "peer_count", smoke_session_count_locked());
    cJSON_AddNumberToObject(out, "max_peers", SMOKE_MAX_SESSIONS);
    add_diag(out, sess);
    esp_err_t e = send_json(req, "200 OK", out);
    cJSON_Delete(out);
    free(answer);
    xSemaphoreGive(s_gate);
    return e;
}

/* TEST2: GET /api/smoke/offer?mode=C   → ESP = offerer. offer 생성/반환. */
static esp_err_t h_offer_get(httpd_req_t *req)
{
    isp_tune_restore_saved_settings_once();

    char mode = 'C';
    char profile_txt[24] = {0};
    char qbuf[96] = {0};
    camera_h264_profile_t profile = CAMERA_H264_PROFILE_1080P;
    if (httpd_req_get_url_query_str(req, qbuf, sizeof(qbuf)) == ESP_OK &&
        httpd_query_key_value(qbuf, "profile", profile_txt, sizeof(profile_txt)) == ESP_OK) {
        profile = smoke_profile_from_text(profile_txt);
    }

    if (xSemaphoreTake(s_gate, pdMS_TO_TICKS(500)) != pdTRUE)
        return send_err(req, "409 Conflict", "session_gate_busy");

    smoke_reap_stale_locked();

    uint32_t count = smoke_session_count_locked();
    if (count >= SMOKE_MAX_SESSIONS) {
        xSemaphoreGive(s_gate);
        return send_err(req, "409 Conflict", "max_2_viewers");
    }

    /* The second ESP-offerer peer joins the existing single encoder. It must
     * use the same camera profile, and the first stream must already be LIVE. */
    if (s_active) {
        if (s_active->profile != profile) {
            xSemaphoreGive(s_gate);
            return send_err(req, "409 Conflict", "profile_in_use");
        }
        /* [DTLS-HANDSHAKE-WINDOW-C2-OWNER-HARDEN-20260908]
         * Load-bearing invariant: a second viewer may start only after the
         * primary is fully CONNECTED.  The owner guard in the timeout override
         * additionally fails closed if this invariant is ever regressed. */
        if (!s_active->peer_connected || !camera_h264_stream_is_active()) {
            xSemaphoreGive(s_gate);
            return send_err(req, "409 Conflict", "primary_connecting");
        }
    }

    SMK("=== ESP-offer requested profile=%s viewer=%u/%u ===",
        camera_h264_stream_profile_name(profile),
        (unsigned)(count + 1U), (unsigned)SMOKE_MAX_SESSIONS);

    smoke_sess_t *sess = NULL;
    esp_err_t sr0 = smoke_start_locked(true, mode, profile, &sess);
    if (sr0 != ESP_OK || !sess) {
        xSemaphoreGive(s_gate);
        if (sr0 == ESP_ERR_INVALID_ARG)
            return send_err(req, "409 Conflict", "profile_in_use");
        if (sr0 == ESP_ERR_INVALID_STATE)
            return send_err(req, "409 Conflict", "primary_connecting");
        if (sr0 == ESP_ERR_NO_MEM)
            return send_err(req, "503 Service Unavailable", "peer_resource_exhausted");
        return send_err(req, "503 Service Unavailable", "peer_open_failed");
    }

    int nr = smoke_peer_new_connection(sess);
    SMK("sid=%u esp_peer_new_connection(offerer) r=%d",
        (unsigned)sess->session_id, nr);
    if (nr != ESP_PEER_ERR_NONE) {
        (void)smoke_teardown_locked(sess);
        xSemaphoreGive(s_gate);
        return send_err(req, "503 Service Unavailable", "new_connection_failed");
    }

    char *offer = await_local_sdp_aggregated(sess);
    if (!offer) {
        (void)smoke_teardown_locked(sess);
        xSemaphoreGive(s_gate);
        return send_err(req, "504 Gateway Timeout", "no_offer");
    }

    int local_candidates = count_sdp_candidate_lines(offer);
    if (local_candidates == 0) {
        /* Do not hand a known-unusable offer to Android. Keep failure explicit
         * so the next log tells us whether esp_peer 1.5.4 generated a host
         * candidate in offerer mode on this firmware. */
        free(offer);
        (void)smoke_teardown_locked(sess);
        xSemaphoreGive(s_gate);
        return send_err(req, "503 Service Unavailable", "no_local_ice_candidate");
    }

    cJSON *out = cJSON_CreateObject();
    cJSON_AddStringToObject(out, "type", "offer");
    cJSON_AddStringToObject(out, "sdp", offer);
    cJSON_AddNumberToObject(out, "session_id", sess->session_id);
    cJSON_AddNumberToObject(out, "peer_count", smoke_session_count_locked());
    cJSON_AddNumberToObject(out, "max_peers", SMOKE_MAX_SESSIONS);
    cJSON_AddNumberToObject(out, "local_candidate_count", local_candidates);
    add_diag(out, sess);

    esp_err_t e = send_json(req, "200 OK", out);
    cJSON_Delete(out);
    free(offer);
    xSemaphoreGive(s_gate);
    return e;
}

/* /api/smoke/offer 디스패치: GET(mode=C) vs POST */
static esp_err_t h_offer(httpd_req_t *req)
{
    if (req->method == HTTP_GET)  return h_offer_get(req);
    if (req->method == HTTP_POST) return h_offer_post(req);
    return send_err(req, "405 Method Not Allowed", "method");
}

/* TEST2: POST /api/smoke/answer  {mode:"C", type:"answer", sdp}  → browser answer 주입 */
static esp_err_t h_answer_post(httpd_req_t *req)
{
    char *body = read_body(req);
    if (!body) return send_err(req, "400 Bad Request", "bad_body");

    cJSON *in = cJSON_Parse(body);
    free(body);
    if (!in) return send_err(req, "400 Bad Request", "bad_json");

    const cJSON *jsdp = cJSON_GetObjectItem(in, "sdp");
    const cJSON *jsid = cJSON_GetObjectItem(in, "session_id");
    if (!cJSON_IsString(jsdp) || !jsdp->valuestring[0] || !cJSON_IsNumber(jsid)) {
        cJSON_Delete(in);
        return send_err(req, "400 Bad Request", "missing_answer_or_session_id");
    }
    uint32_t sid = (uint32_t)jsid->valuedouble;
    if (sid == 0U) {
        cJSON_Delete(in);
        return send_err(req, "400 Bad Request", "bad_session_id");
    }

    char remote_setup[24];
    parse_setup(jsdp->valuestring, remote_setup, sizeof(remote_setup));
    int remote_candidates = count_sdp_candidate_lines(jsdp->valuestring);

    if (xSemaphoreTake(s_gate, pdMS_TO_TICKS(500)) != pdTRUE) {
        cJSON_Delete(in);
        return send_err(req, "409 Conflict", "session_gate_busy");
    }

    smoke_sess_t *sess = smoke_find_session_locked(sid);
    if (!sess || !sess->is_offerer) {
        xSemaphoreGive(s_gate);
        cJSON_Delete(in);
        return send_err(req, "404 Not Found", "offer_session_not_found");
    }

    SMK("=== browser answer received sid=%u candidates=%d setup=%s ===",
        (unsigned)sid, remote_candidates,
        remote_setup[0] ? remote_setup : "(none)");
    log_remote_candidates(jsdp->valuestring);

    int sr = smoke_peer_send_remote_description(sess, jsdp->valuestring);
    cJSON_Delete(in);
    if (sr != ESP_PEER_ERR_NONE) {
        SMKE("sid=%u browser answer inject failed r=%d", (unsigned)sid, sr);
        (void)smoke_teardown_locked(sess);
        xSemaphoreGive(s_gate);
        return send_err(req, "503 Service Unavailable", "remote_description_failed");
    }

    cJSON *out = cJSON_CreateObject();
    cJSON_AddBoolToObject(out, "ok", true);
    cJSON_AddNumberToObject(out, "session_id", sid);
    cJSON_AddNumberToObject(out, "remote_candidate_count", remote_candidates);
    cJSON_AddNumberToObject(out, "peer_count", smoke_session_count_locked());
    esp_err_t e = send_json(req, "200 OK", out);
    cJSON_Delete(out);

    xSemaphoreGive(s_gate);
    return e;
}


static void add_isp_live_tune(cJSON *root,
                              camera_h264_profile_t profile,
                              bool force_live)
{
    bool live_enabled = false, ready = false;
    int live_sat = 136, live_con = 126, live_gamma = 518, live_sharp = 100;
    esp_video_isp_pipeline_live_tune_get(&live_enabled, &live_sat, &live_con,
                                         &live_gamma, &live_sharp, &ready);

    bool saved_enabled = false;
    int saved_sat = 136, saved_con = 126, saved_gamma = 518, saved_sharp = 100;
    isp_tune_saved_for_profile(profile, &saved_enabled,
                               &saved_sat, &saved_con, &saved_gamma, &saved_sharp);

    const bool stream_same_profile =
        camera_h264_stream_is_active() &&
        camera_h264_stream_get_profile() == profile;
    const bool use_live = force_live || stream_same_profile;

    const bool shown_enabled = use_live ? live_enabled : saved_enabled;
    const int shown_sat = use_live ? live_sat : saved_sat;
    const int shown_con = use_live ? live_con : saved_con;
    const int shown_gamma = use_live ? live_gamma : saved_gamma;
    const int shown_sharp = use_live ? live_sharp : saved_sharp;

    const bool dirty = use_live &&
        ((shown_enabled != saved_enabled) ||
         (shown_enabled && saved_enabled &&
          (shown_sat != saved_sat ||
           shown_con != saved_con ||
           shown_gamma != saved_gamma ||
           shown_sharp != saved_sharp)));

    cJSON *t = cJSON_CreateObject();
    cJSON_AddStringToObject(t, "profile",
                            profile == CAMERA_H264_PROFILE_720P_NATIVE ? "720p60" : "1080p");
    cJSON_AddBoolToObject(t, "profile_is_720",
                          profile == CAMERA_H264_PROFILE_720P_NATIVE);
    cJSON_AddBoolToObject(t, "enabled", shown_enabled);
    cJSON_AddBoolToObject(t, "controller_ready", ready);
    cJSON_AddNumberToObject(t, "saturation", shown_sat);
    cJSON_AddNumberToObject(t, "contrast", shown_con);
    cJSON_AddNumberToObject(t, "gamma_milli", shown_gamma);
    cJSON_AddNumberToObject(t, "sharpen_pct", shown_sharp);
    cJSON_AddBoolToObject(t, "saved_enabled", saved_enabled);
    cJSON_AddNumberToObject(t, "saved_saturation", saved_sat);
    cJSON_AddNumberToObject(t, "saved_contrast", saved_con);
    cJSON_AddNumberToObject(t, "saved_gamma_milli", saved_gamma);
    cJSON_AddNumberToObject(t, "saved_sharpen_pct", saved_sharp);

    cJSON_AddStringToObject(t, "storage_model", "per-profile");
    cJSON_AddBoolToObject(t, "dirty", dirty);
    cJSON_AddItemToObject(root, "isp_tune", t);
}


/* POST /api/smoke/isp
 * Live preview:
 *   {enabled:true,saturation:136,contrast:126,gamma_milli:518,sharpen_pct:100}
 * Save to NVS:
 *   same body + {save:true}
 * Restore IPA AUTO and persist that mode:
 *   {enabled:false,save:true}
 *
 * Drag updates are deliberately RAM-only. NVS is written only by the explicit
 * save/default buttons, avoiding flash commits on every slider movement. */
static esp_err_t h_isp_post(httpd_req_t *req)
{
    char *body = read_body(req);
    if (!body) return send_err(req, "400 Bad Request", "bad_body");
    cJSON *in = cJSON_Parse(body);
    free(body);
    if (!in) return send_err(req, "400 Bad Request", "bad_json");

    /* [CAM-AUDIO-GAIN-UI-QOS-V2-20260902]
     * Audio-only body. Return before normal ISP validation. */
    const cJSON *jmic = cJSON_GetObjectItem(in, "mic_gain_db");
    if (cJSON_IsNumber(jmic)) {
        float gain = (float)jmic->valuedouble;
        cJSON_Delete(in);
        if (gain < 0.0f || gain > 42.0f) { /* [CAM-AUDIO-NEUTRAL-GAIN-V2-20260902] */
            return send_err(req, "400 Bad Request", "mic_gain_out_of_range");
        }
        esp_err_t gr = bsp_extra_codec_input_gain_set(gain);
        if (gr != ESP_OK) {
            return send_err(req, "503 Service Unavailable", "mic_gain_apply_failed");
        }
        cJSON *out = cJSON_CreateObject();
        if (!out) return send_err(req, "503 Service Unavailable", "oom");
        cJSON_AddBoolToObject(out, "ok", true);
        cJSON_AddNumberToObject(out, "mic_gain_db", bsp_extra_codec_input_gain_get());
        esp_err_t e = send_json(req, "200 OK", out);
        cJSON_Delete(out);
        return e;
    }

    const cJSON *jen = cJSON_GetObjectItem(in, "enabled");
    const cJSON *jsave = cJSON_GetObjectItem(in, "save");
    const cJSON *jprofile = cJSON_GetObjectItem(in, "profile");
    camera_h264_profile_t profile = smoke_profile_from_text(
        cJSON_IsString(jprofile) ? jprofile->valuestring : NULL);

    bool requested_enabled = cJSON_IsBool(jen) ? cJSON_IsTrue(jen) : true;
    bool save_to_nvs = cJSON_IsBool(jsave) && cJSON_IsTrue(jsave);

    bool cur_enabled = false, ready = false;
    int sat = 136, con = 126, gamma_milli = 518, sharp = 100;
    esp_video_isp_pipeline_live_tune_get(&cur_enabled, &sat, &con,
                                         &gamma_milli, &sharp, &ready);

    const bool apply_enabled = requested_enabled;
    if (requested_enabled) {
        const cJSON *js = cJSON_GetObjectItem(in, "saturation");
        const cJSON *jc = cJSON_GetObjectItem(in, "contrast");
        const cJSON *jg = cJSON_GetObjectItem(in, "gamma_milli");
        const cJSON *jh = cJSON_GetObjectItem(in, "sharpen_pct");
        if (!cJSON_IsNumber(js) || !cJSON_IsNumber(jc) ||
                !cJSON_IsNumber(jg) || !cJSON_IsNumber(jh)) {
            cJSON_Delete(in);
            return send_err(req, "400 Bad Request", "missing_isp_values");
        }
        sat = js->valueint;
        con = jc->valueint;
        gamma_milli = jg->valueint;
        sharp = jh->valueint;

        /* [CAM-720-CONTRAST-255-R5_3-20260911]
         * Keep 1080 at 80..220. 720 alone may use 80..255, matching the
         * independent 720 NVS preset and the real queried ISP range. */
        const int profile_con_max =
            (profile == CAMERA_H264_PROFILE_720P_NATIVE) ?
            CAMERA_ISP_720_CON_MAX : CAMERA_ISP_CON_MAX;
        if (con < CAMERA_ISP_CON_MIN || con > profile_con_max) {
            cJSON_Delete(in);
            return send_err(req, "400 Bad Request", "isp_contrast_out_of_range");
        }
    }
    cJSON_Delete(in);

    esp_err_t ret = esp_video_isp_pipeline_live_tune_set(apply_enabled, sat, con,
                                                          gamma_milli, sharp);
    if (ret == ESP_ERR_INVALID_ARG) {
        return send_err(req, "400 Bad Request", "isp_value_out_of_range");
    }
    if (ret != ESP_OK) {
        return send_err(req, "503 Service Unavailable", "isp_apply_failed");
    }

    if (save_to_nvs) {
        /* [CAM-SAVE-REAL-RESULT-R1-20260911]
         * Explicit browser save is synchronous. HTTP 200 is returned only after
         * the real NVS commit succeeds. On failure, restore the saved-RAM copy
         * so the current live preview remains correctly marked dirty. */
        const int old_man = (profile == CAMERA_H264_PROFILE_720P_NATIVE) ?
                            g_camera_isp_720_manual_enable : g_camera_isp_manual_enable;
        const int old_sat = (profile == CAMERA_H264_PROFILE_720P_NATIVE) ?
                            g_camera_isp_720_saturation : g_camera_isp_saturation;
        const int old_con = (profile == CAMERA_H264_PROFILE_720P_NATIVE) ?
                            g_camera_isp_720_contrast : g_camera_isp_contrast;
        const int old_gam = (profile == CAMERA_H264_PROFILE_720P_NATIVE) ?
                            g_camera_isp_720_gamma_milli : g_camera_isp_gamma_milli;
        const int old_shp = (profile == CAMERA_H264_PROFILE_720P_NATIVE) ?
                            g_camera_isp_720_sharpen_pct : g_camera_isp_sharpen_pct;

        if (profile == CAMERA_H264_PROFILE_720P_NATIVE) {
            g_camera_isp_720_manual_enable = apply_enabled ? 1 : 0;
            if (apply_enabled) {
                g_camera_isp_720_saturation = sat;
                g_camera_isp_720_contrast = con;
                g_camera_isp_720_gamma_milli = gamma_milli;
                g_camera_isp_720_sharpen_pct = sharp;
            }
        } else {
            g_camera_isp_manual_enable = apply_enabled ? 1 : 0;
            if (apply_enabled) {
                g_camera_isp_saturation = sat;
                g_camera_isp_contrast = con;
                g_camera_isp_gamma_milli = gamma_milli;
                g_camera_isp_sharpen_pct = sharp;
            }
        }

        esp_err_t save_err = settings_save_to_nvs();
        if (save_err != ESP_OK) {
            if (profile == CAMERA_H264_PROFILE_720P_NATIVE) {
                g_camera_isp_720_manual_enable = old_man;
                g_camera_isp_720_saturation = old_sat;
                g_camera_isp_720_contrast = old_con;
                g_camera_isp_720_gamma_milli = old_gam;
                g_camera_isp_720_sharpen_pct = old_shp;
            } else {
                g_camera_isp_manual_enable = old_man;
                g_camera_isp_saturation = old_sat;
                g_camera_isp_contrast = old_con;
                g_camera_isp_gamma_milli = old_gam;
                g_camera_isp_sharpen_pct = old_shp;
            }
            ESP_LOGE(TAG, "[CAM-ISP-NVS] explicit save FAILED profile=%s err=%s",
                     profile == CAMERA_H264_PROFILE_720P_NATIVE ? "720p60" : "1080p",
                     esp_err_to_name(save_err));
            return send_err(req, "503 Service Unavailable", "nvs_save_failed");
        }

        ESP_LOGI(TAG,
                 "[CAM-ISP-NVS] explicit save OK profile=%s mode=%s sat=%d con=%d gamma=%d sharp=%d",
                 profile == CAMERA_H264_PROFILE_720P_NATIVE ? "720p60" : "1080p",
                 apply_enabled ? "MANUAL" : "AUTO",
                 profile == CAMERA_H264_PROFILE_720P_NATIVE ? g_camera_isp_720_saturation : g_camera_isp_saturation,
                 profile == CAMERA_H264_PROFILE_720P_NATIVE ? g_camera_isp_720_contrast : g_camera_isp_contrast,
                 profile == CAMERA_H264_PROFILE_720P_NATIVE ? g_camera_isp_720_gamma_milli : g_camera_isp_gamma_milli,
                 profile == CAMERA_H264_PROFILE_720P_NATIVE ? g_camera_isp_720_sharpen_pct : g_camera_isp_sharpen_pct);
    }

    cJSON *out = cJSON_CreateObject();
    cJSON_AddBoolToObject(out, "ok", true);
    cJSON_AddBoolToObject(out, "save_requested", save_to_nvs);
    add_isp_live_tune(out, profile, true);
    esp_err_t e = send_json(req, "200 OK", out);
    cJSON_Delete(out);
    return e;
}

/* POST /api/smoke/stop → 활성 세션 정리 */
static esp_err_t h_stop_post(httpd_req_t *req)
{
    uint32_t sid = 0;
    char qbuf[128] = {0};
    char sid_txt[16] = {0};
    char token_txt[SMOKE_CLIENT_TOKEN_HEX_LEN + 1U] = {0};
    if (httpd_req_get_url_query_str(req, qbuf, sizeof(qbuf)) == ESP_OK) {
        if (httpd_query_key_value(qbuf, "sid", sid_txt, sizeof(sid_txt)) == ESP_OK) {
            sid = (uint32_t)strtoul(sid_txt, NULL, 10);
        }
        const bool token_param_seen =
            (strncmp(qbuf, "token=", 6) == 0) || (strstr(qbuf, "&token=") != NULL);
        esp_err_t tq = httpd_query_key_value(qbuf, "token", token_txt, sizeof(token_txt));
        if (tq == ESP_OK) {
            if (!smoke_client_token_valid(token_txt)) {
                return send_err(req, "400 Bad Request", "bad_client_token");
            }
        } else if (token_param_seen) {
            /* Never let a malformed/oversize token silently fall through to
             * the legacy anonymous-stop branch. */
            return send_err(req, "400 Bad Request", "bad_client_token");
        }
    }

    if (xSemaphoreTake(s_gate, pdMS_TO_TICKS(3000)) != pdTRUE)
        return send_err(req, "409 Conflict", "session_gate_busy");

    smoke_reap_stale_locked();

    smoke_sess_t *target = NULL;
    if (sid != 0U) {
        target = smoke_find_session_locked(sid);
        if (!target) {
            xSemaphoreGive(s_gate);
            return send_err(req, "404 Not Found", "session_not_found");
        }
    } else if (token_txt[0]) {
        target = smoke_find_client_token_locked(token_txt);
        if (!target) {
            uint32_t remain = smoke_session_count_locked();
            xSemaphoreGive(s_gate);
            cJSON *out = cJSON_CreateObject();
            cJSON_AddBoolToObject(out, "ok", true);
            cJSON_AddBoolToObject(out, "already_gone", true);
            cJSON_AddNumberToObject(out, "peer_count", remain);
            esp_err_t e = send_json(req, "200 OK", out);
            cJSON_Delete(out);
            return e;
        }
        SMK("=== stop token %.8s -> sid=%u ===",
            token_txt, (unsigned)target->session_id);
    } else {
        uint32_t count = smoke_session_count_locked();
        if (count > 1U) {
            xSemaphoreGive(s_gate);
            return send_err(req, "409 Conflict", "session_id_required");
        }
        target = s_active ? s_active : s_secondary;
        if (!target) {
            xSemaphoreGive(s_gate);
            cJSON *out = cJSON_CreateObject();
            cJSON_AddBoolToObject(out, "ok", true);
            cJSON_AddNumberToObject(out, "peer_count", 0);
            esp_err_t e = send_json(req, "200 OK", out);
            cJSON_Delete(out);
            return e;
        }
    }

    SMK("=== stop sid=%u ===", (unsigned)target->session_id);
    bool cleaned = smoke_teardown_locked(target);
    uint32_t remain = smoke_session_count_locked();
    xSemaphoreGive(s_gate);

    cJSON *out = cJSON_CreateObject();
    cJSON_AddBoolToObject(out, "ok", cleaned);
    cJSON_AddNumberToObject(out, "peer_count", remain);
    cJSON_AddNumberToObject(out, "max_peers", SMOKE_MAX_SESSIONS);
    if (!cleaned) cJSON_AddBoolToObject(out, "deferred", true);
    esp_err_t e = send_json(req, cleaned ? "200 OK" : "202 Accepted", out);
    cJSON_Delete(out);
    return e;
}

/* GET /api/smoke/status -> peer + continuous camera/H264 status */
/* [CAM-1080-QP30-31-GOP8-10-12-FINAL-MATRIX-R3-20260913]
 * Diagnostic build only: freeze the normal 1080 adaptive GOP controller while
 * the encoder performs the six-point final matrix. */
#define CAM_1080_FINAL_MATRIX_R3 1

/* [CAM-1080-ADAPT-GOP-R2-20260912] ---------- ESP-authoritative 1080 Adaptive GOP ---------- */
static const char *smoke_adapt_state_name(adapt_gop_state_t st)
{
    switch (st) {
    case ADAPT_GOP5_STABLE: return "GOP5_STABLE";
    case ADAPT_GOP1_STABLE: return "GOP1_STABLE";
    case ADAPT_PROBING_GOP1: return "PROBING_GOP1";
    case ADAPT_FREEZE: return "FREEZE";
    default: return "?";
    }
}

static uint32_t smoke_adapt_popcount4(uint8_t v)
{
    v &= 0x0fU;
    uint32_t n = 0;
    for (uint32_t i = 0; i < 4U; ++i) n += (v >> i) & 1U;
    return n;
}

static uint32_t smoke_adapt_ema_u32(uint32_t oldv, uint32_t newv, bool first)
{
    if (first) return newv;
    return (oldv * 65U + newv * 35U + 50U) / 100U;
}

static uint32_t smoke_adapt_fps_ratio_x1000(const smoke_sess_t *s,
                                             const camera_h264_stream_stats_t *cs)
{
    if (!s || !cs || s->adapt_browser_fps_x100 == 0U || cs->encode_fps_x100 == 0U) return 0U;
    uint64_t r = (uint64_t)s->adapt_browser_fps_x100 * 1000ULL / (uint64_t)cs->encode_fps_x100;
    return r > 2000ULL ? 2000U : (uint32_t)r;
}

static void smoke_adapt_clear_windows_locked(void)
{
    smoke_sess_t *list[SMOKE_MAX_SESSIONS] = {s_active, s_secondary};
    for (uint32_t i = 0; i < SMOKE_MAX_SESSIONS; ++i) {
        smoke_sess_t *ps = list[i];
        if (!ps) continue;
        ps->adapt_bad_hist = 0;
        ps->adapt_good_streak = 0;
        ps->adapt_tx_drop_last = 0;
        ps->adapt_stats_seen = false;
        ps->adapt_stats_ms = 0;
        if (ps->tx_lock && xSemaphoreTake(ps->tx_lock, pdMS_TO_TICKS(5)) == pdTRUE) {
            ps->adapt_tx_hwm_window = ps->tx_count;
            ps->adapt_tx_drop_prev = ps->tx_queue_drops;
            xSemaphoreGive(ps->tx_lock);
        }
    }
}

static esp_err_t smoke_adapt_switch_locked(uint32_t target_gop, const char *reason)
{
    if (target_gop != 1U && target_gop != 5U) return ESP_ERR_INVALID_ARG;
    if (camera_h264_stream_get_profile() != CAMERA_H264_PROFILE_1080P) return ESP_ERR_NOT_SUPPORTED;
    const uint32_t old_gop = camera_h264_stream_get_gop();
    if (old_gop == target_gop) return ESP_OK;
    if (!camera_h264_stream_is_active()) return ESP_ERR_INVALID_STATE;

    SMKW("[CAM-1080-LIVE-GOP-R1-20260913] switch reason=%s GOP %u -> %u viewers=%u",
         reason ? reason : "?", (unsigned)old_gop, (unsigned)target_gop,
         (unsigned)smoke_fanout_count());

    /* Do not stop/restart camera or H264 here. The camera producer applies the
     * new GOP between frames to the already-open HW encoder. This preserves the
     * task stack, V4L2 queues, persistent ref, WEB20 pool, PTS timeline,
     * PeerConnection, ICE/DTLS/SRTP, and audio. */
    esp_err_t gr = camera_h264_stream_set_gop(target_gop);
    if (gr != ESP_OK) {
        SMKE("[CAM-1080-LIVE-GOP-R1-20260913] live switch GOP %u -> %u failed=%s; actual=%u",
             (unsigned)old_gop, (unsigned)target_gop, esp_err_to_name(gr),
             (unsigned)camera_h264_stream_get_gop());
        return gr;
    }

    if (camera_h264_stream_get_gop() != target_gop) {
        SMKE("[CAM-1080-LIVE-GOP-R1-20260913] live switch ACK mismatch target=%u actual=%u",
             (unsigned)target_gop, (unsigned)camera_h264_stream_get_gop());
        return ESP_FAIL;
    }

    s_adapt_switch_count++;
    smoke_adapt_clear_windows_locked();
    SMK("[CAM-1080-LIVE-GOP-R1-20260913] switch PASS GOP=%u no camera/H264 restart; "
        "peer/ICE/DTLS/audio/PTS kept",
        (unsigned)target_gop);
    return ESP_OK;
}

static bool smoke_adapt_session_fresh(const smoke_sess_t *ps, int64_t now_ms)
{
    return ps && ps->running && ps->peer_connected &&
           ps->profile == CAMERA_H264_PROFILE_1080P &&
           ps->adapt_stats_seen &&
           now_ms - ps->adapt_stats_ms >= 0 &&
           now_ms - ps->adapt_stats_ms <= ADAPT_STATS_FRESH_MS;
}

static uint32_t smoke_adapt_connected_1080_locked(void)
{
    uint32_t n = 0;
    smoke_sess_t *list[SMOKE_MAX_SESSIONS] = {s_active, s_secondary};
    for (uint32_t i = 0; i < SMOKE_MAX_SESSIONS; ++i) {
        smoke_sess_t *ps = list[i];
        if (ps && ps->running && ps->peer_connected &&
            ps->profile == CAMERA_H264_PROFILE_1080P) n++;
    }
    return n;
}

static bool smoke_adapt_any_congested_locked(const camera_h264_stream_stats_t *cs,
                                              int64_t now_ms, bool probe_mode)
{
    smoke_sess_t *list[SMOKE_MAX_SESSIONS] = {s_active, s_secondary};
    for (uint32_t i = 0; i < SMOKE_MAX_SESSIONS; ++i) {
        smoke_sess_t *ps = list[i];
        if (!ps || !ps->running || !ps->peer_connected ||
            ps->profile != CAMERA_H264_PROFILE_1080P) continue;
        if (ps->adapt_stats_seen && now_ms - ps->adapt_stats_ms > ADAPT_STATS_FRESH_MS) return true;
        if (!smoke_adapt_session_fresh(ps, now_ms)) continue;
        uint32_t ratio = smoke_adapt_fps_ratio_x1000(ps, cs);
        if (ps->adapt_tx_drop_last > 0U) return true; /* ring drop is already a late/strong signal */
        bool severe = ps->adapt_loss_ema_bp > ADAPT_BAD_LOSS_BP ||
                      ps->adapt_tx_hwm_window * 100U >= SMOKE_TX_QUEUE_DEPTH * 80U;
        if (probe_mode && severe) return true;
        if (smoke_adapt_popcount4(ps->adapt_bad_hist) >= 3U) return true;
        if (ratio > 0U && ratio < ADAPT_BAD_FPS_RATIO_X1000 && probe_mode) return true;
    }
    return false;
}

static bool smoke_adapt_all_headroom_locked(const camera_h264_stream_stats_t *cs, int64_t now_ms)
{
    uint32_t eligible = 0;
    smoke_sess_t *list[SMOKE_MAX_SESSIONS] = {s_active, s_secondary};
    for (uint32_t i = 0; i < SMOKE_MAX_SESSIONS; ++i) {
        smoke_sess_t *ps = list[i];
        if (!ps || !ps->running || !ps->peer_connected ||
            ps->profile != CAMERA_H264_PROFILE_1080P) continue;
        eligible++;
        if (!smoke_adapt_session_fresh(ps, now_ms)) return false;
        uint32_t ratio = smoke_adapt_fps_ratio_x1000(ps, cs);
        const bool strong_avail_hint = ps->adapt_avail_kbps >= ADAPT_GOP1_HEADROOM_KBPS;
        const uint32_t need_streak = strong_avail_hint ?
                                     ADAPT_FAST_PROBE_GOOD_STREAK : ADAPT_CLEAN_PROBE_GOOD_STREAK;
        const uint32_t frame_interval_us = cs->encode_fps_x100 ? (100000000U / cs->encode_fps_x100) : 0U;
        const bool tx_clean = frame_interval_us > 0U &&
                              ps->adapt_send_ema_us * 2U < frame_interval_us;
        if (!(ps->adapt_loss_ema_bp < ADAPT_GOOD_LOSS_BP &&
              ps->adapt_jitter_ema_ms < ADAPT_GOOD_JITTER_MS &&
              ps->adapt_tx_hwm_window * 100U < SMOKE_TX_QUEUE_DEPTH * 40U &&
              ps->adapt_tx_drop_last == 0U &&
              tx_clean &&
              ratio > ADAPT_GOOD_FPS_RATIO_X1000 &&
              ps->adapt_good_streak >= need_streak)) return false;
    }
    return eligible > 0U;
}

static bool smoke_adapt_probe_success_locked(const camera_h264_stream_stats_t *cs, int64_t now_ms)
{
    uint32_t eligible = 0;
    smoke_sess_t *list[SMOKE_MAX_SESSIONS] = {s_active, s_secondary};
    for (uint32_t i = 0; i < SMOKE_MAX_SESSIONS; ++i) {
        smoke_sess_t *ps = list[i];
        if (!ps || !ps->running || !ps->peer_connected ||
            ps->profile != CAMERA_H264_PROFILE_1080P) continue;
        eligible++;
        if (!smoke_adapt_session_fresh(ps, now_ms)) return false;
        uint32_t ratio = smoke_adapt_fps_ratio_x1000(ps, cs);
        if (!(ps->adapt_loss_ema_bp < ADAPT_PROBE_LOSS_BP &&
              ps->adapt_tx_drop_last == 0U && ratio > ADAPT_PROBE_FPS_RATIO_X1000)) return false;
    }
    return eligible > 0U;
}

static void smoke_adapt_fail_probe_locked(const char *reason, int64_t now_ms)
{
    esp_err_t rr = smoke_adapt_switch_locked(5U, reason);
    if (rr != ESP_OK) {
        /* Fail closed without lying about current encoder state. */
        s_adapt_state = ADAPT_GOP1_STABLE;
        s_adapt_probe_until_ms = 0;
        s_adapt_cooldown_until_ms = now_ms + 30000LL;
        SMKE("[CAM-1080-ADAPT-GOP-R2-20260912] probe fallback to GOP5 failed=%s; holding actual GOP1", esp_err_to_name(rr));
        return;
    }
    s_adapt_state = ADAPT_GOP5_STABLE;
    s_adapt_probe_fail_count++;
    uint32_t shift = s_adapt_probe_fail_count > 3U ? 3U : (s_adapt_probe_fail_count - 1U);
    int64_t backoff = 30000LL << shift;
    if (backoff > 180000LL) backoff = 180000LL;
    s_adapt_cooldown_until_ms = now_ms + backoff;
    s_adapt_probe_until_ms = 0;
}

static void smoke_adapt_evaluate_locked(uint32_t browser_freeze_ms)
{
    if (camera_h264_stream_get_profile() != CAMERA_H264_PROFILE_1080P) return;
#if CAM_1080_FINAL_MATRIX_R3
    (void)browser_freeze_ms;
    return;
#endif
    const int64_t now_ms = esp_timer_get_time() / 1000LL;
    const uint32_t viewers = smoke_adapt_connected_1080_locked();
    if (viewers == 0U) return;

    if (browser_freeze_ms > 0U) {
        uint32_t freeze_ms = browser_freeze_ms > (uint32_t)ADAPT_FREEZE_MS ? (uint32_t)ADAPT_FREEZE_MS : browser_freeze_ms;
        int64_t until = now_ms + (int64_t)freeze_ms;
        if (s_adapt_state != ADAPT_FREEZE) s_adapt_resume_state = s_adapt_state;
        if (until > s_adapt_freeze_until_ms) s_adapt_freeze_until_ms = until;
        /* [CAM-1080-ADAPT-GOP-R2D-20260913] A new ICE/candidate path invalidates
         * probe failures/backoff learned on the old path (e.g. LTE -> Wi-Fi).
         * Freeze first, then require fresh clean samples on the new path. */
        s_adapt_probe_fail_count = 0U;
        s_adapt_cooldown_until_ms = s_adapt_freeze_until_ms;
        s_adapt_state = ADAPT_FREEZE;
    }

    if (viewers != s_adapt_last_viewer_count) {
        const bool joined = viewers > s_adapt_last_viewer_count;
        s_adapt_last_viewer_count = viewers;
        s_adapt_cooldown_until_ms = now_ms + ADAPT_COOLDOWN_MS;
        smoke_adapt_clear_windows_locked();
        if (joined && camera_h264_stream_get_gop() == 1U) {
            if (smoke_adapt_switch_locked(5U, "viewer_join") == ESP_OK) {
                s_adapt_state = ADAPT_GOP5_STABLE;
                return;
            }
            s_adapt_state = ADAPT_GOP1_STABLE;
        }
    }

    if (now_ms < s_adapt_freeze_until_ms) {
        s_adapt_state = ADAPT_FREEZE;
        return;
    }
    if (s_adapt_state == ADAPT_FREEZE) {
        if (s_adapt_resume_state == ADAPT_PROBING_GOP1 && camera_h264_stream_get_gop() == 1U) {
            s_adapt_state = ADAPT_PROBING_GOP1;
            s_adapt_probe_until_ms = now_ms + ADAPT_PROBE_MS;
            smoke_adapt_clear_windows_locked();
            SMK("[CAM-1080-ADAPT-GOP-R2-20260912] network freeze ended -> resume GOP1 probe");
        } else {
            s_adapt_state = (camera_h264_stream_get_gop() == 1U) ?
                            ADAPT_GOP1_STABLE : ADAPT_GOP5_STABLE;
        }
    }

    camera_h264_stream_stats_t cs = {0};
    camera_h264_stream_get_stats(&cs);
    const uint32_t gop = camera_h264_stream_get_gop();

    if (s_adapt_state == ADAPT_PROBING_GOP1) {
        if (smoke_adapt_any_congested_locked(&cs, now_ms, true)) {
            smoke_adapt_fail_probe_locked("probe_congested", now_ms);
            return;
        }
        if (now_ms >= s_adapt_probe_until_ms) {
            if (smoke_adapt_probe_success_locked(&cs, now_ms)) {
                s_adapt_state = ADAPT_GOP1_STABLE;
                s_adapt_probe_fail_count = 0;
                s_adapt_probe_until_ms = 0;
                SMK("[CAM-1080-ADAPT-GOP-R2-20260912] GOP1 probe PASS -> GOP1_STABLE");
            } else {
                smoke_adapt_fail_probe_locked("probe_failed", now_ms);
            }
        }
        return;
    }

    if (gop == 1U) {
        s_adapt_state = ADAPT_GOP1_STABLE;
        if (smoke_adapt_any_congested_locked(&cs, now_ms, false)) {
            if (smoke_adapt_switch_locked(5U, "congested_3of4") == ESP_OK) {
                s_adapt_state = ADAPT_GOP5_STABLE;
                s_adapt_cooldown_until_ms = now_ms + ADAPT_COOLDOWN_MS;
            }
        }
        return;
    }

    s_adapt_state = ADAPT_GOP5_STABLE;
    if (now_ms >= s_adapt_cooldown_until_ms && smoke_adapt_all_headroom_locked(&cs, now_ms)) {
        if (smoke_adapt_switch_locked(1U, "headroom_probe") == ESP_OK) {
            s_adapt_state = ADAPT_PROBING_GOP1;
            s_adapt_probe_until_ms = now_ms + ADAPT_PROBE_MS;
            SMK("[CAM-1080-ADAPT-GOP-R2-20260912] GOP1 probe start duration=%lldms", (long long)ADAPT_PROBE_MS);
        }
    }
}

static void smoke_adapt_report_locked(uint32_t sid, bool valid, uint32_t browser_freeze_ms,
                                      uint32_t browser_fps_x100, uint32_t loss_bp,
                                      uint32_t jitter_ms, uint32_t avail_kbps,
                                      uint32_t rx_kbps)
{
    smoke_sess_t *ps = smoke_find_session_locked(sid);
    if (!ps || ps->profile != CAMERA_H264_PROFILE_1080P) return;
    const int64_t now_ms = esp_timer_get_time() / 1000LL;

    if (browser_freeze_ms > 0U) {
        /* [CAM-1080-ADAPT-GOP-R2C-20260913] A path/ICE transition invalidates
         * pre-transition headroom evidence. Require fresh clean samples after
         * the freeze before an unknown-bandwidth GOP1 probe is allowed. */
        ps->adapt_good_streak = 0U;
        ps->adapt_bad_hist = 0U;
        ps->adapt_stats_ms = now_ms;
        smoke_adapt_evaluate_locked(browser_freeze_ms);
        return;
    }
    if (!valid) {
        smoke_adapt_evaluate_locked(0U);
        return;
    }

    uint32_t hwm_window = 0, drop_total = 0, send_us = 0, q_now = 0;
    if (ps->tx_lock && xSemaphoreTake(ps->tx_lock, pdMS_TO_TICKS(5)) == pdTRUE) {
        q_now = ps->tx_count;
        hwm_window = ps->adapt_tx_hwm_window;
        if (hwm_window < q_now) hwm_window = q_now;
        drop_total = ps->tx_queue_drops;
        send_us = ps->tx_send_us_last;
        ps->adapt_tx_hwm_window = q_now;
        xSemaphoreGive(ps->tx_lock);
    }

    const bool first = !ps->adapt_stats_seen;
    uint32_t drop_delta = first ? 0U : (drop_total - ps->adapt_tx_drop_prev);
    ps->adapt_tx_drop_prev = drop_total;
    ps->adapt_tx_drop_last = drop_delta;
    ps->adapt_browser_fps_x100 = browser_fps_x100;
    ps->adapt_loss_bp = loss_bp;
    ps->adapt_jitter_ms = jitter_ms;
    ps->adapt_avail_kbps = avail_kbps;
    ps->adapt_rx_kbps = rx_kbps;
    ps->adapt_loss_ema_bp = smoke_adapt_ema_u32(ps->adapt_loss_ema_bp, loss_bp, first);
    ps->adapt_jitter_ema_ms = smoke_adapt_ema_u32(ps->adapt_jitter_ema_ms, jitter_ms, first);
    ps->adapt_rx_ema_kbps = smoke_adapt_ema_u32(ps->adapt_rx_ema_kbps, rx_kbps, first);
    ps->adapt_send_ema_us = smoke_adapt_ema_u32(ps->adapt_send_ema_us, send_us, first);
    ps->adapt_tx_hwm_window = hwm_window;
    ps->adapt_stats_seen = true;
    ps->adapt_stats_ms = now_ms;

    camera_h264_stream_stats_t cs = {0};
    camera_h264_stream_get_stats(&cs);
    uint32_t ratio = smoke_adapt_fps_ratio_x1000(ps, &cs);
    uint32_t frame_interval_us = cs.encode_fps_x100 ? (100000000U / cs.encode_fps_x100) : 0U;
    bool bad = ps->adapt_loss_ema_bp > ADAPT_BAD_LOSS_BP ||
               drop_delta > 0U ||
               hwm_window * 100U >= SMOKE_TX_QUEUE_DEPTH * 80U ||
               (frame_interval_us > 0U && ps->adapt_send_ema_us > frame_interval_us) ||
               (ratio > 0U && ratio < ADAPT_BAD_FPS_RATIO_X1000) ||
               ps->adapt_jitter_ema_ms > ADAPT_BAD_JITTER_MS;
    ps->adapt_bad_hist = (uint8_t)(((ps->adapt_bad_hist << 1) | (bad ? 1U : 0U)) & 0x0fU);

    /* [CAM-1080-ADAPT-GOP-R2D-20260913]
     * availableIncomingBitrate is only a positive hint.  A low/known value must
     * not veto an up-probe because Chrome BWE can remain application-limited by
     * the current low-rate GOP5 stream.  Clean delivery + low ESP send latency
     * earns a conservative probe; >=5.2Mbps merely shortens the clean streak.
     * The 3s GOP1 probe is the actual capacity test and backs off on failure. */
    const bool tx_clean = frame_interval_us > 0U &&
                          ps->adapt_send_ema_us * 2U < frame_interval_us;
    bool good = ps->adapt_loss_ema_bp < ADAPT_GOOD_LOSS_BP &&
                ps->adapt_jitter_ema_ms < ADAPT_GOOD_JITTER_MS &&
                hwm_window * 100U < SMOKE_TX_QUEUE_DEPTH * 40U &&
                drop_delta == 0U &&
                tx_clean &&
                ratio > ADAPT_GOOD_FPS_RATIO_X1000;
    ps->adapt_good_streak = good ? (uint8_t)(ps->adapt_good_streak < 255U ? ps->adapt_good_streak + 1U : 255U) : 0U;

    if (now_ms - ps->adapt_last_log_ms >= 3000LL) {
        SMK("[CAM-1080-ADAPT-GOP-R2-20260912] sid=%u metrics state=%s gop=%u browser=%u.%02u enc=%u.%02u ratio=%u "
            "loss=%u.%02u%% jitter=%ums avail=%uk(%s) rx=%uk send_ema=%uus hwm=%u/%u drop=%u hist=0x%x good=%u",
            (unsigned)sid, smoke_adapt_state_name(s_adapt_state), (unsigned)camera_h264_stream_get_gop(),
            (unsigned)(browser_fps_x100 / 100U), (unsigned)(browser_fps_x100 % 100U),
            (unsigned)(cs.encode_fps_x100 / 100U), (unsigned)(cs.encode_fps_x100 % 100U),
            (unsigned)ratio, (unsigned)(ps->adapt_loss_ema_bp / 100U),
            (unsigned)(ps->adapt_loss_ema_bp % 100U), (unsigned)ps->adapt_jitter_ema_ms,
            (unsigned)ps->adapt_avail_kbps, ps->adapt_avail_kbps ? "known" : "unknown",
            (unsigned)ps->adapt_rx_kbps, (unsigned)ps->adapt_send_ema_us,
            (unsigned)hwm_window, (unsigned)SMOKE_TX_QUEUE_DEPTH,
            (unsigned)drop_delta, (unsigned)ps->adapt_bad_hist, (unsigned)ps->adapt_good_streak);
        ps->adapt_last_log_ms = now_ms;
    }

    smoke_adapt_evaluate_locked(0U);
}

static esp_err_t h_status_get(httpd_req_t *req)
{
    isp_tune_restore_saved_settings_once();
    /* Reuse this already-registered URI for destructive one-shot camera
     * benchmarks so no additional httpd URI slots are consumed.
     *   ?cambench=yu12   capture-only CSI/ISP YUV420 timing
     *   ?cambench=rgb565 capture-only CSI/ISP RGB565 timing
     *   ?h264direct=1    direct esp_h264 HW API timing (bypass /dev/video11)
     *   ?raw10=1         legacy direct-RAW attempt (may be unsupported) */
    camera_h264_profile_t tune_profile = camera_h264_stream_get_profile();
    uint32_t color_measure_request_id = 0;
    bool want_color_measure = false;
    char qbuf[320] = {0};
    /* [CAM-1080-ADAPT-GOP-R2-20260912] Compact browser report carried on the existing status GET. */
    bool adapt_report_present = false;
    bool adapt_report_valid = false;
    bool adapt_report_freeze = false;
    uint32_t adapt_freeze_ms = 0;
    uint32_t adapt_sid = 0, adapt_bfps_x100 = 0, adapt_loss_bp = 0;
    uint32_t adapt_jitter_ms = 0, adapt_avail_kbps = 0, adapt_rx_kbps = 0;
    if (httpd_req_get_url_query_str(req, qbuf, sizeof(qbuf)) == ESP_OK) {
        char bench[16] = {0};
        char raw[8] = {0};
        char direct[8] = {0};
        char color_measure[8] = {0};
        char tune_profile_txt[24] = {0};
        char asid[16] = {0}, av[8] = {0}, afz[8] = {0}, afm[16] = {0}, af[16] = {0};
        char al[16] = {0}, aj[16] = {0}, ai[16] = {0}, ar[16] = {0};
        if (httpd_query_key_value(qbuf, "profile", tune_profile_txt, sizeof(tune_profile_txt)) == ESP_OK) {
            tune_profile = smoke_profile_from_text(tune_profile_txt);
        }
        if (httpd_query_key_value(qbuf, "asid", asid, sizeof(asid)) == ESP_OK) {
            adapt_report_present = true;
            adapt_sid = (uint32_t)strtoul(asid, NULL, 10);
            if (httpd_query_key_value(qbuf, "av", av, sizeof(av)) == ESP_OK)
                adapt_report_valid = (strcmp(av, "1") == 0);
            if (httpd_query_key_value(qbuf, "afz", afz, sizeof(afz)) == ESP_OK)
                adapt_report_freeze = (strcmp(afz, "1") == 0);
            if (httpd_query_key_value(qbuf, "afm", afm, sizeof(afm)) == ESP_OK)
                adapt_freeze_ms = (uint32_t)strtoul(afm, NULL, 10);
            if (adapt_report_freeze && adapt_freeze_ms == 0U) adapt_freeze_ms = (uint32_t)ADAPT_FREEZE_MS;
            if (adapt_freeze_ms > (uint32_t)ADAPT_FREEZE_MS) adapt_freeze_ms = (uint32_t)ADAPT_FREEZE_MS;
            if (httpd_query_key_value(qbuf, "af", af, sizeof(af)) == ESP_OK) adapt_bfps_x100 = (uint32_t)strtoul(af, NULL, 10);
            if (httpd_query_key_value(qbuf, "al", al, sizeof(al)) == ESP_OK) adapt_loss_bp = (uint32_t)strtoul(al, NULL, 10);
            if (httpd_query_key_value(qbuf, "aj", aj, sizeof(aj)) == ESP_OK) adapt_jitter_ms = (uint32_t)strtoul(aj, NULL, 10);
            if (httpd_query_key_value(qbuf, "ai", ai, sizeof(ai)) == ESP_OK) adapt_avail_kbps = (uint32_t)strtoul(ai, NULL, 10);
            if (httpd_query_key_value(qbuf, "ar", ar, sizeof(ar)) == ESP_OK) adapt_rx_kbps = (uint32_t)strtoul(ar, NULL, 10);
            if (adapt_sid == 0U || adapt_bfps_x100 > 24000U || adapt_loss_bp > 10000U ||
                adapt_jitter_ms > 10000U || adapt_avail_kbps > 1000000U || adapt_rx_kbps > 1000000U) {
                adapt_report_present = false;
            }
        }
        bool want_yu12 = false, want_rgb565 = false, want_raw10 = false, want_h264direct = false;
        if (httpd_query_key_value(qbuf, "cambench", bench, sizeof(bench)) == ESP_OK) {
            want_yu12 = (strcasecmp(bench, "yu12") == 0 || strcasecmp(bench, "yuv420") == 0);
            want_rgb565 = (strcasecmp(bench, "rgb565") == 0 || strcasecmp(bench, "rgbp") == 0);
        }
        if (httpd_query_key_value(qbuf, "raw10", raw, sizeof(raw)) == ESP_OK) {
            want_raw10 = (strcmp(raw, "1") == 0 || strcasecmp(raw, "true") == 0);
        }
        if (httpd_query_key_value(qbuf, "h264direct", direct, sizeof(direct)) == ESP_OK) {
            want_h264direct = (strcmp(direct, "1") == 0 || strcasecmp(direct, "true") == 0);
        }
        if (httpd_query_key_value(qbuf, "color_measure", color_measure, sizeof(color_measure)) == ESP_OK) {
            want_color_measure = (strcmp(color_measure, "1") == 0 ||
                                  strcasecmp(color_measure, "true") == 0);
        }
        if (want_yu12 || want_rgb565 || want_raw10 || want_h264direct) {
            bool peer_busy = true;
            if (xSemaphoreTake(s_gate, pdMS_TO_TICKS(200)) == pdTRUE) {
                peer_busy = (smoke_session_count_locked() != 0U);
                xSemaphoreGive(s_gate);
            }
            if (peer_busy || camera_h264_stream_is_active()) {
                return send_err(req, "409 Conflict", "stop_webrtc_before_camera_benchmark");
            }
            if (want_yu12) {
                return camera_csi_capture_bench_http(req, V4L2_PIX_FMT_YUV420, "YU12_capture_only");
            }
            if (want_rgb565) {
                return camera_csi_capture_bench_http(req, V4L2_PIX_FMT_RGB565, "RGB565_capture_only");
            }
            if (want_h264direct) {
                return camera_h264_direct_bench_http(req);
            }
            return camera_csi_raw10_diag_http(req);
        }
    }

    /* [CAM-1080-ADAPT-GOP-R2-20260912] Browser provides receive observations; ESP decides GOP.
     * The gate serializes two viewers and camera-only restart transitions. */
    if (adapt_report_present && camera_h264_stream_get_profile() == CAMERA_H264_PROFILE_1080P) {
        if (xSemaphoreTake(s_gate, pdMS_TO_TICKS(5000)) == pdTRUE) {
            smoke_adapt_report_locked(adapt_sid, adapt_report_valid, adapt_freeze_ms,
                                      adapt_bfps_x100, adapt_loss_bp, adapt_jitter_ms,
                                      adapt_avail_kbps, adapt_rx_kbps);
            xSemaphoreGive(s_gate);
        }
    }

    /* [CAM-AUTO-COLOR-CAL-R1-20260910]
     * Reuse status URI instead of consuming another httpd URI slot.
     * ?color_measure=1 arms exactly one next-frame O_UYY measurement, only
     * when the requested profile is the currently running camera profile. */
    if (want_color_measure &&
        camera_h264_stream_is_active() &&
        camera_h264_stream_get_profile() == tune_profile) {
        color_measure_request_id = camera_color_measure_request(tune_profile);
    }

    cJSON *out = cJSON_CreateObject();
    if (!out) return send_err(req, "503 Service Unavailable", "oom");

    bool peer_active = false, peer_connected = false;
    uint32_t peer_count = 0, peer_connected_count = 0;
    uint32_t media_sent = 0, media_drop = 0;
    uint32_t tx_q = 0, tx_hwm = 0, tx_enqueued = 0, tx_qdrop = 0, tx_oversize = 0;
    uint32_t tx_copy_last = 0, tx_copy_max = 0, tx_send_last = 0, tx_send_max = 0;
    if (xSemaphoreTake(s_gate, pdMS_TO_TICKS(200)) == pdTRUE) {
        smoke_sess_t *list[SMOKE_MAX_SESSIONS] = {s_active, s_secondary};
        peer_count = smoke_session_count_locked();
        peer_active = (peer_count != 0U);
        for (uint32_t i = 0; i < SMOKE_MAX_SESSIONS; ++i) {
            smoke_sess_t *ps = list[i];
            if (!ps) continue;
            if (ps->peer_connected) peer_connected_count++;
            media_sent += ps->media_sent;
            media_drop += ps->media_drop;
            if (ps->tx_lock && xSemaphoreTake(ps->tx_lock, pdMS_TO_TICKS(5)) == pdTRUE) {
                tx_q += ps->tx_count;
                if (ps->tx_high_water > tx_hwm) tx_hwm = ps->tx_high_water;
                tx_enqueued += ps->tx_enqueued;
                tx_qdrop += ps->tx_queue_drops;
                tx_oversize += ps->tx_oversize_drops;
                tx_copy_last = ps->tx_copy_us_last;
                if (ps->tx_copy_us_max > tx_copy_max) tx_copy_max = ps->tx_copy_us_max;
                tx_send_last = ps->tx_send_us_last;
                if (ps->tx_send_us_max > tx_send_max) tx_send_max = ps->tx_send_us_max;
                xSemaphoreGive(ps->tx_lock);
            }
        }
        peer_connected = (peer_connected_count != 0U);
        xSemaphoreGive(s_gate);
    }

    camera_h264_stream_stats_t cs = {0};
    camera_h264_stream_get_stats(&cs);
    cJSON_AddBoolToObject(out, "peer_active", peer_active);
    cJSON_AddBoolToObject(out, "peer_connected", peer_connected);
    cJSON_AddNumberToObject(out, "peer_count", peer_count);
    cJSON_AddNumberToObject(out, "peer_connected_count", peer_connected_count);
    cJSON_AddNumberToObject(out, "max_peers", SMOKE_MAX_SESSIONS);
    cJSON_AddNumberToObject(out, "mic_gain_db", bsp_extra_codec_input_gain_get()); /* [CAM-AUDIO-GAIN-UI-QOS-V2-20260902] */
    cJSON_AddNumberToObject(out, "media_sent", media_sent);
    cJSON_AddNumberToObject(out, "media_drop", media_drop);
    cJSON *tx = cJSON_CreateObject();
    cJSON_AddNumberToObject(tx, "depth_per_peer", SMOKE_TX_QUEUE_DEPTH);
    cJSON_AddNumberToObject(tx, "total_capacity", SMOKE_TX_QUEUE_DEPTH * peer_count);
    cJSON_AddNumberToObject(tx, "queued", tx_q);
    cJSON_AddNumberToObject(tx, "high_water", tx_hwm);
    cJSON_AddNumberToObject(tx, "enqueued", tx_enqueued);
    cJSON_AddNumberToObject(tx, "queue_drops", tx_qdrop);
    cJSON_AddNumberToObject(tx, "oversize_drops", tx_oversize);
    cJSON_AddNumberToObject(tx, "copy_us_last", tx_copy_last);
    cJSON_AddNumberToObject(tx, "copy_us_max", tx_copy_max);
    cJSON_AddNumberToObject(tx, "send_us_last", tx_send_last);
    cJSON_AddNumberToObject(tx, "send_us_max", tx_send_max);
    cJSON_AddItemToObject(out, "tx_queue", tx);
    cJSON *cam = cJSON_CreateObject();
    cJSON_AddBoolToObject(cam, "running", cs.running);
    cJSON_AddNumberToObject(cam, "width", cs.width);
    cJSON_AddNumberToObject(cam, "height", cs.height);
    cJSON_AddNumberToObject(cam, "sensor_fps", cs.sensor_fps);
    cJSON_AddNumberToObject(cam, "h264_gop", cs.h264_gop);
    cJSON_AddNumberToObject(cam, "h264_qp_min", cs.h264_qp_min);
    cJSON_AddNumberToObject(cam, "h264_qp_max", cs.h264_qp_max);
    cJSON_AddNumberToObject(cam, "h264_bitrate", cs.h264_bitrate);
    cJSON_AddNumberToObject(cam, "fps_x100", cs.encode_fps_x100);
    cJSON_AddNumberToObject(cam, "capture_fps_x100", cs.camera_fps_x100);
    cJSON_AddNumberToObject(cam, "encode_fps_x100", cs.encode_fps_x100);
    cJSON_AddNumberToObject(cam, "encoded", cs.encoded_frames);
    cJSON_AddNumberToObject(cam, "delivered", cs.delivered_frames);
    cJSON_AddNumberToObject(cam, "consumer_drops", cs.consumer_drops);
    cJSON_AddNumberToObject(cam, "encode_us_last", cs.encode_us_last);
    cJSON_AddNumberToObject(cam, "encode_us_max", cs.encode_us_max);
    cJSON_AddNumberToObject(cam, "camera_dq_us_last", cs.camera_dq_us_last);
    cJSON_AddNumberToObject(cam, "camera_dq_us_avg", cs.camera_dq_us_avg);
    cJSON_AddNumberToObject(cam, "camera_dq_us_max", cs.camera_dq_us_max);
    cJSON_AddNumberToObject(cam, "consumer_us_last", cs.consumer_us_last);
    cJSON_AddNumberToObject(cam, "consumer_us_max", cs.consumer_us_max);
    cJSON_AddNumberToObject(cam, "loop_us_last", cs.loop_us_last);
    cJSON_AddNumberToObject(cam, "loop_us_max", cs.loop_us_max);
    cJSON_AddNumberToObject(cam, "dma_free", cs.dma_free);
    cJSON_AddNumberToObject(cam, "dma_largest", cs.dma_largest);
    cJSON_AddNumberToObject(cam, "psram_free", cs.psram_free);
    cJSON_AddNumberToObject(cam, "last_errno", cs.last_errno);
    cJSON_AddStringToObject(cam, "stage", cs.last_stage);
    cJSON_AddStringToObject(cam, "profile",
        camera_h264_stream_profile_name(camera_h264_stream_get_profile()));
    cJSON_AddItemToObject(out, "camera", cam);

    /* [CAM-1080-ADAPT-GOP-R2-20260912] Read-only controller diagnostics for the production UI/logging. */
    cJSON *ag = cJSON_CreateObject();
    if (ag) {
        const int64_t now_ms = esp_timer_get_time() / 1000LL;
        cJSON_AddBoolToObject(ag, "enabled", camera_h264_stream_get_profile() == CAMERA_H264_PROFILE_1080P);
        cJSON_AddStringToObject(ag, "state", smoke_adapt_state_name(s_adapt_state));
        cJSON_AddNumberToObject(ag, "gop", camera_h264_stream_get_gop());
        cJSON_AddNumberToObject(ag, "switches", s_adapt_switch_count);
        cJSON_AddNumberToObject(ag, "cooldown_ms", s_adapt_cooldown_until_ms > now_ms ? (double)(s_adapt_cooldown_until_ms - now_ms) : 0.0);
        cJSON_AddNumberToObject(ag, "probe_ms", s_adapt_probe_until_ms > now_ms ? (double)(s_adapt_probe_until_ms - now_ms) : 0.0);
        if (xSemaphoreTake(s_gate, pdMS_TO_TICKS(50)) == pdTRUE) {
            smoke_sess_t *aps = adapt_sid ? smoke_find_session_locked(adapt_sid) : s_active;
            if (aps) {
                cJSON_AddNumberToObject(ag, "rx_kbps", aps->adapt_rx_kbps);
                cJSON_AddNumberToObject(ag, "rx_ema_kbps", aps->adapt_rx_ema_kbps);
                cJSON_AddNumberToObject(ag, "avail_kbps", aps->adapt_avail_kbps);
                cJSON_AddNumberToObject(ag, "loss_bp", aps->adapt_loss_ema_bp);
                cJSON_AddNumberToObject(ag, "jitter_ms", aps->adapt_jitter_ema_ms);
                cJSON_AddNumberToObject(ag, "browser_fps_x100", aps->adapt_browser_fps_x100);
                cJSON_AddNumberToObject(ag, "send_ema_us", aps->adapt_send_ema_us);
            }
            xSemaphoreGive(s_gate);
        }
        cJSON_AddItemToObject(out, "adaptive_gop", ag);
    }

    camera_color_measurement_t cm = {0};
    camera_color_measure_get(&cm);
    cJSON *measure = cJSON_CreateObject();
    if (measure) {
        cJSON_AddBoolToObject(measure, "valid", cm.valid);
        cJSON_AddNumberToObject(measure, "request_id", cm.request_id);
        cJSON_AddStringToObject(measure, "profile",
            cm.valid ? camera_h264_stream_profile_name(cm.profile) : "");
        cJSON_AddNumberToObject(measure, "elapsed_ms", cm.elapsed_ms);
        if (cm.valid) {
            cJSON_AddNumberToObject(measure, "y_mean", cm.y_mean);
            cJSON_AddNumberToObject(measure, "y_p10", cm.y_p10);
            cJSON_AddNumberToObject(measure, "y_p50", cm.y_p50);
            cJSON_AddNumberToObject(measure, "y_p90", cm.y_p90);
            cJSON_AddNumberToObject(measure, "u_mean", cm.u_mean);
            cJSON_AddNumberToObject(measure, "v_mean", cm.v_mean);
            cJSON_AddNumberToObject(measure, "u_abs", cm.u_abs);
            cJSON_AddNumberToObject(measure, "v_abs", cm.v_abs);
        }
        cJSON_AddItemToObject(out, "color_measure", measure);
    }
    cJSON_AddNumberToObject(out, "color_measure_request", color_measure_request_id);
    add_isp_live_tune(out, tune_profile, false);

    esp_err_t e = send_json(req, "200 OK", out);
    cJSON_Delete(out);
    return e;
}

/* GET /webrtc-smoke -> existing engineering diagnostic page. */
static esp_err_t h_smoke_page_get(httpd_req_t *req)
{
    isp_tune_restore_saved_settings_once();
    const size_t len = (size_t)(s_webrtc_smoke_html_end - s_webrtc_smoke_html_start);
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store, max-age=0");
    httpd_resp_set_hdr(req, "Pragma", "no-cache");
    return httpd_resp_send(req, (const char *)s_webrtc_smoke_html_start, len);
}

/* GET /camera-webrtc -> production camera UI.
 * Signaling/media backend is intentionally unchanged and still uses the proven
 * /api/smoke endpoints. This handler changes page presentation only. */
static esp_err_t h_camera_page_get(httpd_req_t *req)
{
    isp_tune_restore_saved_settings_once();
    const size_t len = (size_t)(s_camera_live_html_end - s_camera_live_html_start);
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store, max-age=0");
    httpd_resp_set_hdr(req, "Pragma", "no-cache");
    return httpd_resp_send(req, (const char *)s_camera_live_html_start, len);
}

/* [CAM-ISP-NVS-20260830]
 * settings_load_from_nvs() runs before the web server is started.
 * Seed the customized ISP pipeline once from those saved values. If the ISP
 * controller does not exist yet, live_tune_set() stores them and the normal IPA
 * config path applies them when the controller becomes ready. */
static void isp_tune_restore_saved_settings_once(void)
{
    static bool restored = false;
    if (restored) return;

    esp_err_t err = isp_tune_apply_profile(CAMERA_H264_PROFILE_1080P);

    if (err == ESP_OK) {
        restored = true;
        ESP_LOGI(TAG,
                 "[CAM-ISP-NVS] lazy restore OK 1080=%s %d/%d/%d/%d | 720=%s %d/%d/%d/%d",
                 g_camera_isp_manual_enable ? "MANUAL" : "AUTO",
                 g_camera_isp_saturation, g_camera_isp_contrast,
                 g_camera_isp_gamma_milli, g_camera_isp_sharpen_pct,
                 g_camera_isp_720_manual_enable ? "MANUAL" : "AUTO",
                 g_camera_isp_720_saturation, g_camera_isp_720_contrast,
                 g_camera_isp_720_gamma_milli, g_camera_isp_720_sharpen_pct);
    } else {
        ESP_LOGW(TAG, "[CAM-ISP-NVS] lazy restore failed: %s", esp_err_to_name(err));
    }
}

/* [CAM-AUDIO-REMOVE-PCM-WAV-DIAG-V1-20260902] WAV diagnostic URI removed; CAM_PROBE slot restored. */
/* ───────────── 등록 ───────────── */

esp_err_t web_smoke_register(httpd_handle_t server)
{
    if (!server) return ESP_ERR_INVALID_ARG;
    /* [CAM-AUDIO-CRACKLE-PEERLOG-DIAG-V2-20260902]
     * A/B diagnostic only.
     * Suppress PEER_DEF INFO/DEBUG bursts while preserving WARN/ERROR.
     * No media, queue, RTP, H264, ICE, TURN, gain or codec behavior changes. */
    esp_log_level_set("PEER_DEF", ESP_LOG_WARN);
    ESP_LOGW(TAG, "[AUDIO-CRACKLE-DIAG] PEER_DEF -> WARN (A/B test)");

    /* Do not call ISP restore here: registration runs inside GOT_IP/sys_evt. */
    if (!s_gate) {
        s_gate = xSemaphoreCreateMutex();
        if (!s_gate) return ESP_ERR_NO_MEM;
    }
    if (!s_fanout_lock) {
        s_fanout_lock = xSemaphoreCreateMutex();
        if (!s_fanout_lock) return ESP_ERR_NO_MEM;
    }
    { esp_err_t ae = camera_webrtc_audio_init(); if (ae != ESP_OK) return ae; }
    /* DTLS 인증서는 첫 smoke 세션에서 lazy 생성한다. 부팅 경로에는 비용을 주지 않는다. */

    static const httpd_uri_t u_page       = { .uri = "/webrtc-smoke",     .method = HTTP_GET,  .handler = h_smoke_page_get };
    static const httpd_uri_t u_cam_page   = { .uri = "/camera-webrtc",    .method = HTTP_GET,  .handler = h_camera_page_get };
    static const httpd_uri_t u_offer_get  = { .uri = "/api/smoke/offer",  .method = HTTP_GET,  .handler = h_offer };
    static const httpd_uri_t u_offer_post = { .uri = "/api/smoke/offer",  .method = HTTP_POST, .handler = h_offer };
    static const httpd_uri_t u_answer     = { .uri = "/api/smoke/answer", .method = HTTP_POST, .handler = h_answer_post };
    static const httpd_uri_t u_stop       = { .uri = "/api/smoke/stop",   .method = HTTP_POST, .handler = h_stop_post };
    static const httpd_uri_t u_status     = { .uri = "/api/smoke/status", .method = HTTP_GET,  .handler = h_status_get };
    static const httpd_uri_t u_isp        = { .uri = "/api/smoke/isp",    .method = HTTP_POST, .handler = h_isp_post };

    esp_err_t e = ESP_OK;
    e |= httpd_register_uri_handler(server, &u_page);
    e |= httpd_register_uri_handler(server, &u_cam_page);
    e |= httpd_register_uri_handler(server, &u_offer_get);
    e |= httpd_register_uri_handler(server, &u_offer_post);
    e |= httpd_register_uri_handler(server, &u_answer);
    e |= httpd_register_uri_handler(server, &u_stop);
    e |= httpd_register_uri_handler(server, &u_status);
    e |= httpd_register_uri_handler(server, &u_isp);
    /* [CAM-COLOR-STAGE2-20260901] one extra POST URI */
    e |= camera_color_stage2_register(server);
    if (e != ESP_OK) {
        ESP_LOGW(TAG, "smoke uri register incomplete (max_uri_handlers 여유 확인)");
    } else {
        ESP_LOGI(TAG, "camera WebRTC registered: max_viewers=%u one-camera/one-H264 fanout (esp_peer %s)", (unsigned)SMOKE_MAX_SESSIONS, SMOKE_ESP_PEER_VER);
    }
    return e;
}
