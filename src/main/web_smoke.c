/*
 * web_smoke.c ??WebRTC ?드?이??smoke test (esp_peer 1.5.4)
 *
 * 목적: 기존 SDP ??ICE ??DTLS smoke 경로????면?? ?결 ?공 ?? *       OV02C10 1080p(1920x1080@30 2-lane) ?는 native720-HS(1288x728@60 2-lane)
 *       ??CSI/ISP YU12 ??HW H.264 GOP1/QP30..40 auto-RC ?레?을 PSRAM 3-slot queue?? *       짧게 복사?고, 기존 smoke_pump가
 *       esp_peer_send_video()?비동??송?다.
 *       카메??encoder 루프가 SRTP/UDP ?신?간??기다리? ?도?분리?다.
 *       /webrtc-smoke???환 별칭?고
 *       ?품 ?험 ?이지??/camera-webrtc ?다.
 *
 * ?정 identity (dependencies.lock):
 *   esp_peer      = 1.5.4  (Espressif Component Registry)
 *   component_hash= 36c98e3420dad4675bbe13431bf4deefd0ba2b82f71dda458cb5471825d053b4
 *   esp_libsrtp   = 1.0.0 (transitive)   ESP-IDF = 5.5.4
 *
 * changelog ?인(?적, 1.5.4 ??:
 *   v1.4.2 "Treat DTLS role as client when peer send actpass" ?함
 *          ??Browser offer(actpass) ??esp_peer 가 ?스?DTLS role ???한?? 강제?? ?고 관찰만 ?다.
 *   v1.5.2 "-0x7080 (mbedTLS < v3.6.6) handshake fail ?정" ?함.
 *
 * API ??esp_peer 공개 ?더(esp_peer.h / esp_peer_types.h / esp_peer_default.h)?서 ?인??것만 ?용.
 * 존재?? ?는 ?수/?드???용?? ?았?? (?? DTLS role 강제 API ??존재?? ?으므??용 ????)
 *
 * 보호: Core1 stepper/ISR 불????pump ?스?는 Core0 ??고정. M32/RTA/SSE/max_open_sockets 미??
 * [WEBRTC-2CLIENT-FANOUT-AB-20260830] 카메??ISP/H.264 encoder??1개만 ???고
 *       최? 2개의 ?립 WebRTC peer???일 H.264 access unit??fan-out ?다.
 *       모든 esp_peer_* ?출? s->peer_mtx ?직렬??main_loop ? ?시?출 방?).
 *       Browser SDP??a=candidate ?인? "candidate:..." 별도 메시지?ICE agent???제 주입.
 *       pump 종료 미확???태?서??close/free 금?(?전??deferred teardown).
 *       malloc/open ?패??assert ?? ?고 503/graceful ?처리.
 * [ANDROID-ICE-ESP-OFFER-AB-20260830] production page??ESP-offerer signaling???용?다.
 *       ESP local host candidate가 offer SDP???함????Android/PC browser가 answer??성?다.
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
#include "freertos/timers.h"  /* [DMA-BLACKBOX-R2-20260919] static idle recovery timer */
#include "freertos/idf_additions.h"  /* [DMA-BB-EXPECTED-R3-20261010] pxTaskGetStackStart(): read-only owner probe */
#include "esp_log.h"
#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_http_server.h"
#include "nvs.h"  /* [CAM-USER-GAIN-R23] per-profile gain preference */
#include "cJSON.h"

#include "esp_peer.h"
#include "esp_peer_default.h"

#include "web_smoke.h"
#include "flight_recorder.h"  /* [FLIGHT-RECORDER-STAGE2-R1-20260916] */
#include "sface_db.h"       /* [SFACE-DB-R1] separate opt-in persistence */
#include "camera_csi_diag.h"
#include "face_ai_detect.h"  /* [FACE-AI-GREEN-BOX-OVERLAY-R4-20260929] read-only face box */
#include "face_ai_tap.h"     /* [FACE-AI-SCAN-GEOM-DUMP-R21-20261001] diagnostic A/B + frame snapshot */
#include "log_diag_policy.h" /* [LOG-COMPACT-R1-20261003] output policy only */
/* R16F8 current project header may not yet expose the resident-boundary helper. */
extern esp_err_t camera_h264_dma_boundary_bootstrap(void);
#include "esp_h264_enc_single_hw.h"
#include "settings.h"
#include "camera_color_stage2.h"   /* [CAM-COLOR-STAGE2-20260901] */
#include "camera_webrtc_audio.h"  /* [CAM-AUDIO-OPUS24K-V4-20260902] */
#include "sound.h"                /* [REV-AUDIO-SPEAKER-PHASE2-R1-20260917] */
#include "bsp_board_extra.h"      /* [CAM-AUDIO-GAIN-V2-INCLUDE-FIX-20260902] mic gain API */
#include "linux/videodev2.h"

/* [WEBRTC-FINAL-CANDIDATE-FC1-20260908]
 * Temporary TX-start, UDP-probe, browser telemetry traffic, and periodic pump-stack logs
 * are not part of production operation.  Core session/ICE/DTLS/camera behavior is unchanged. */

#include <errno.h>
#include <sys/time.h>
#include <unistd.h>
#include "lwip/sockets.h"
#include "lwip/inet.h"

/* Production log quiet switch. Set to 1 temporarily when deep diagnostics are needed. */
#ifndef HA_VERBOSE_DIAG_LOGS
#define HA_VERBOSE_DIAG_LOGS 0
#endif

/* [LOG-COMPACT-R1-20261003] Diagnostic-only output gates. */
#define SMK_DMA_DIAG(...) \
    do { if (HA_LOG_DIAG_DMA_VERBOSE) { SMKW(__VA_ARGS__); } } while (0)
#define SMK_WEBRTC_DIAG(...) \
    do { if (HA_LOG_DIAG_WEBRTC_VERBOSE) { SMKW(__VA_ARGS__); } } while (0)
#define SMK_WEBRTC_INFO(...) \
    do { if (HA_LOG_DIAG_WEBRTC_VERBOSE) { SMK(__VA_ARGS__); } } while (0)



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

/* [OV02C10-AE-ALLPATH-DIAG-R2-20260922]
 * Diagnostics for grouped AND split sensor exposure/gain command paths. */
extern esp_err_t ov02c10_ae_diag_watch_ms(uint32_t ms);
extern esp_err_t ov02c10_ae_diag_hold_ms(uint32_t ms);
extern void ov02c10_ae_diag_release(void);
/* [CAM-AUTOCAL-GAIN-SWEEP-R15-20260922] RAM-only 720p60 gain experiment. */
extern esp_err_t ov02c10_ae_diag_gain_force_ms(uint32_t ms, uint32_t gain_index);
extern void ov02c10_ae_diag_gain_force_release(void);
extern void ov02c10_ae_diag_gain_force_get(uint32_t *gain_value,
                                           uint32_t *left_ms,
                                           uint32_t *suppressed_count,
                                           bool *active);
extern void ov02c10_ae_diag_get(uint32_t *req_exp, uint32_t *req_gain,
                                uint32_t *applied_exp, uint32_t *applied_gain,
                                uint32_t *hold_left_ms, uint32_t *write_count,
                                uint32_t *group_count, uint32_t *exp_count,
                                uint32_t *gain_count, uint32_t *last_path,
                                uint32_t *suppressed_count, bool *hold_active);
/* [CAM-AE-GOVERNOR-R24-20260923]
 * Production AUTO/MANUAL gain is owned by the ISP/IPA boundary, not hidden
 * inside the sensor driver. This keeps physical gain and IPA sensor state in
 * sync and makes MANUAL->AUTO a seamless fresh handoff. */
extern esp_err_t esp_video_isp_pipeline_user_gain_set(bool manual, uint32_t gain_index);
extern void esp_video_isp_pipeline_user_gain_get(bool *manual, uint32_t *gain_index);
/* [CAM-AE-PROFILE-WARM-RESTORE-R48-20260924]
 * Snapshot real Exposure/Gain immediately before owner-only camera stop. */
extern esp_err_t esp_video_isp_pipeline_ae_warm_save_profile(bool profile_720);
/* [CAM-AE-WARM-R48F1-20260924]
 * Warm restore gate retained for normal profile-switch safety. */
extern void esp_video_isp_pipeline_ae_warm_set_restore_enabled(bool enabled);
/* [CAM-AE-RESUME-FASTLOCK-R50F1-20260926]
 * One-shot marker for the next physical AE seed. Used by normal fresh starts,
 * same-profile reconnects, profile switches and auto-cal. Warm restore runs
 * first; bounded FAST-LOCK only closes the remaining brightness error. */
extern void esp_video_isp_pipeline_ae_fastlock_prepare_next_seed(bool enabled);

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
extern const uint8_t s_camera_live_html_gz_start[] asm("_binary_camera_live_html_gz_start");
extern const uint8_t s_camera_live_html_gz_end[]   asm("_binary_camera_live_html_gz_end");

static const char *TAG = "web_smoke";
#define SMK(fmt, ...) ESP_LOGI(TAG, "[SMOKE] " fmt, ##__VA_ARGS__)
#define SMKE(fmt, ...) ESP_LOGE(TAG, "[SMOKE] " fmt, ##__VA_ARGS__)
#define SMKW(fmt, ...) ESP_LOGW(TAG, "[SMOKE] " fmt, ##__VA_ARGS__)

#define SMOKE_ESP_PEER_VER   "1.5.4"
#define SMOKE_ESP_PEER_HASH8 "36c98e34"

#define SMOKE_BODY_MAX     16384     /* Chrome SDP ?유 ?함. ?청마다 ?시 ?당 */
#define SMOKE_MAX_CAND     12        /* ?칠 local candidate 최? 개수 */
#define SMOKE_CAND_LEN     512

/* [LAN-DIRECT-ICE-ASYNC-R4-20260926]
 * Production LAN-direct discovery for public-domain / NAT-loopback access.
 * Never block the HTTP offer path waiting for mDNS.  ICE starts immediately;
 * one persistent PSRAM-stack worker resolves Chrome's .local host name in
 * parallel.  If a private IPv4 arrives while ICE is still PAIRING, the
 * original host candidate is trickled once with ADDRESS only rewritten.
 * A short RAM cache makes repeated profile/session offers for the same Chrome
 * .local name immediate.  Miss/late result is a no-op: existing srflx/TURN
 * fallback continues unchanged. */
#define SMOKE_LAN_MDNS_ASYNC_TIMEOUT_MS 450U
#define SMOKE_LAN_MDNS_CACHE_TTL_MS     30000U
#define SMOKE_LAN_MDNS_WORKER_STACK     4096
#define SMOKE_LAN_MDNS_WORKER_PRIO      4
#define SMOKE_LAN_MDNS_JOB_NONE         UINT8_MAX
#define SMOKE_SDP_WAIT_MS  3000      /* local SDP(on_msg) ??*/
#define SMOKE_GATHER_MS    0         /* A/B: answerer HTTP answer?즉시 반환; ICE/DTLS가 browser setRemoteDescription보다 먼? ?리??race ?거 */
#define SMOKE_TURN_RELAY_WAIT_MAX_MS   1200 /* Claude review A/B: remote relay offer?서 local relay publish ????전?게 ??*/
#define SMOKE_TURN_RELAY_WAIT_POLL_MS  25
/* Live camera sessions have no fixed lifetime. They are reaped on explicit /stop
 * or on a real peer disconnect/failure. This avoids the old 120 s test-only cutoff. */
#define SMOKE_PUMP_CORE    0         /* Core0 고정 (Core1 = stepper) */
#define SMOKE_PUMP_PRIO    5
#define SMOKE_PUMP_STACK   12288  /* TURN/mobile path measured up to ~11.2KB; keep 12KB */
#define SMOKE_PUMP_SLOT_NONE UINT8_MAX
#define SMOKE_AUDIO_GATE_PRIO 4
#define SMOKE_AUDIO_GATE_STACK 3072
#define SMOKE_AUDIO_GATE_TIMEOUT_MS 3500U
#define SMOKE_AUDIO_GATE_POLL_MS 50U
#define SMOKE_AGENT_RECV_TIMEOUT_MS 10 /* [DTLS-HANDSHAKE-WINDOW-C1-EXACT-20260907] steady media */   /* production: keep peer main_loop responsive; 100ms caps one-AU-per-loop TX near 10fps */
#define SMOKE_MAX_SESSIONS 2U            /* A/B: PC + smartphone */
#define SMOKE_DTLS_CONNECT_TIMEOUT_MS 3000U /* paired->DTLS/SRTP ready watchdog; LAN success is normally <<1s */
/* [STALE-CONNECT-REAP-20260914] A slot that never reached CONNECTED and whose
 * browser vanished before DTLS watchdog was armed is reclaimed after this
 * grace. Keep this above normal full-ICE + DTLS connect time. */
#define SMOKE_SESSION_CONNECT_GRACE_MS 15000U
#define SMOKE_CLIENT_TOKEN_HEX_LEN    32U   /* [CAM-INITIAL-START-RECOVERY-V2-TOKEN-20260903] 128-bit per-page token */
#define SMOKE_DIAG_ID_HEX_LEN        16U   /* [DUPVIEW-DMA-DIAG-R1-20260918] diagnostic-only random id */
/* [CAM-VIEWER-HEARTBEAT-R1-20260918]
 * A page is a viewer only while visible and its decoded-frame counter advances.
 * Hidden/dead pages get a short grace, then the persistent pump owns cleanup. */
#define SMOKE_VIEWER_HEARTBEAT_TIMEOUT_MS 15000U

/* [DMA-BLACKBOX-R3-HEAPMAP-20260919]
 * R2 edge/persist/recover + full-idle sampler, plus two full INTERNAL|DMA heap
 * structure snapshots using ESP-IDF's official heap_caps_dump(): first >=2KB
 * edge and final +5s full idle. No new heap buffer/task/stack is allocated. */
#define SMOKE_DMA_DROP_MIN_BYTES       1024U
#define SMOKE_DMA_EDGE_MIN_BYTES       2048U
#define SMOKE_DMA_DROP_CONFIRM_SAMPLES 3U
#define SMOKE_DMA_DROP_CONFIRM_MS      2000U
#define SMOKE_DMA_IDLE_RETRY_MS        20U
#define SMOKE_DMA_IDLE_MAX_RETRIES     5U

/* [REV-AUDIO-RX-AB-PHASE1-20260917]
 * Phase 1 reverse-audio RECEIVE A/B diagnostic. This ONLY enables the WebRTC
 * inbound audio (browser mic -> ESP) receive path so we can measure its
 * INTERNAL|DMA cost and confirm packet arrival. It does NOT decode Opus and
 * does NOT touch the speaker/I2S. Set to 0 to revert to byte-behaviour-identical
 * SEND_ONLY audio without git (A/B baseline). */
#ifndef SMOKE_REVERSE_AUDIO_RX_PHASE1
#define SMOKE_REVERSE_AUDIO_RX_PHASE1 1
#endif
/* Bound the NEW receive audio jitter buffer. peer_default defaults audio jitter
 * cache_size to 100kB when left 0 (see esp_peer_default_jitter_cfg_t) which is
 * far larger than a 20ms Opus voice stream needs and dangerous for the ~22KB
 * INTERNAL|DMA plateau IF peer_default places it internally. Start minimal and
 * measure (RXA0..RXA7). 8192B still holds ~20+ Opus voice packets, well above
 * the 100ms (5-packet) default cache_timeout window. */
#define SMOKE_RX_AUDIO_JITTER_BYTES   8192U
/* Phase 1 logs inbound packet counts/sizes only.
 * Opus RTP payload size is NOT used to infer speech vs silence. */

static void smoke_dma_probe_log(uint32_t sid, const char *phase,
                                size_t free_before, size_t largest_before)
{
    const size_t free_after = heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
    const size_t largest_after = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
    SMK_DMA_DIAG("[DMA-DIAG PUMP-PROBE] sid=%u %s free=%u largest=%u -> free=%u largest=%u delta_free=%d delta_largest=%d",
         (unsigned)sid, phase ? phase : "?",
         (unsigned)free_before, (unsigned)largest_before,
         (unsigned)free_after, (unsigned)largest_after,
         (int)free_after - (int)free_before,
         (int)largest_after - (int)largest_before);
}

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
    TaskHandle_t      pump;        /* persistent worker handle while assigned */
    uint8_t           pump_slot;   /* s_pump_* slot; SMOKE_PUMP_SLOT_NONE when idle */
    SemaphoreHandle_t peer_mtx;   /* esp_peer_* 직렬??*/
    SemaphoreHandle_t sdp_ready;  /* on_msg ?local SDP ?보 ??give */
    SemaphoreHandle_t done;       /* pump 종료 ?? */

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
    /* [LATE-CONNECTED-UAF-GUARD-R1-20260915]
     * Monotonic lifecycle fence. Once teardown starts, peer callbacks must not
     * re-attach this session to media fanout or restart shared camera/audio. */
    volatile bool     teardown_started;
    volatile bool     peer_connected; /* DTLS/SRTP connected; camera sender may transmit */
    volatile bool     dtls_connecting; /* CONNECTING seen, waiting for CONNECTED */
    volatile int64_t  dtls_start_ms;   /* first CONNECTING timestamp; 0 when disarmed */
    volatile uint32_t fanout_refs;    /* camera callback in-flight refs */
    bool              fanout_attached;
    uint32_t          session_id;
    char              client_token[SMOKE_CLIENT_TOKEN_HEX_LEN + 1U]; /* exact browser-page owner token; "" for legacy clients */
    /* [DUPVIEW-DMA-DIAG-R1-20260918]
     * Diagnostic metadata only. browser_id persists in browser localStorage;
     * page_id is regenerated per page life; offer_seq increments per /offer.
     * Session objects are PSRAM-preferred, so these fields do not consume the
     * scarce INTERNAL|DMA heap on the normal path. */
    char              diag_browser_id[SMOKE_DIAG_ID_HEX_LEN + 1U];
    char              diag_page_id[SMOKE_DIAG_ID_HEX_LEN + 1U];
    uint32_t          diag_offer_seq;
    bool              viewer_hb_required;
    volatile bool     viewer_hb_seen;
    volatile bool     viewer_hb_visible;
    volatile uint32_t viewer_hb_last_ms;
    volatile uint32_t viewer_hb_last_frames;
    volatile bool     auto_cleanup_requested; /* real peer loss: pump exits then safe cleanup task reaps session */
    bool              pump_done_seen; /* done semaphore?teardown???? ?비?는지 (sticky) */
    uint32_t          media_sent;
    uint32_t          media_drop;
    int64_t           last_media_log_ms;
    bool              is_offerer; /* TEST2 = true */
    bool              remote_has_relay; /* browser offer contains typ relay */

    /* [LAN-DIRECT-ICE-ASYNC-R4-20260926]
     * Session object lives in PSRAM.  The mDNS worker never keeps this pointer;
     * it returns only through a fixed global job slot keyed by session_id.
     * State callback opens/closes the proven PAIRING injection window, while
     * the existing persistent pump performs esp_peer_send_msg() under peer_mtx. */
    char              lan_mdns_name[128];
    char              lan_mdns_candidate_template[SMOKE_CAND_LEN]; /* original "candidate:..." with .local */
    char              lan_mdns_resolved_ip[16];
    uint16_t          lan_mdns_candidate_port;
    uint8_t           lan_mdns_job_slot;
    volatile bool     lan_mdns_pairing_open;
    volatile bool     lan_mdns_candidate_ready;
    volatile bool     lan_mdns_inject_pending;
    volatile bool     lan_mdns_inject_attempted;
    volatile bool     lan_stun_source_confirmed; /* R5F2: repeated current-session R5 evidence, not trusted until PAIRED */
    char              lan_stun_confirmed_ip[16]; /* R5F2: exact learned endpoint pending ICE verification */
    uint16_t          lan_stun_confirmed_port;
    bool              lan_relay_wait_bypass;     /* R5F2: only prior ICE-paired STUN cache may bypass relay wait */
    int               lan_mdns_inject_result;

    volatile bool     local_has_relay;  /* callback-owned readiness flag; HTTP waiter never dereferences local_sdp while polling */
    volatile uint32_t local_sdp_emit_count;
    volatile uint32_t local_candidate_msg_count;
    char              mode;       /* 'A'/'B'/'C' (로그?? ?? 강제 ?님) */
    camera_h264_profile_t profile;  /* selected sensor-native WebRTC profile */
    /* [CAM-1080-ADAPT-GOP-R2-20260912] Keep video PTS monotonic across camera-only GOP restarts while
     * the PeerConnection/ICE/DTLS/audio session remains alive. */
    uint32_t          video_pts_last_raw_ms;
    uint64_t          video_pts_base_ms;
    uint64_t          video_pts_last_out_ms;
    bool              video_pts_have;

    /* [CAM-D2-D3-RECOVERY-PROBE-R1-20260915]
     * Diagnostic-only state. Session objects live in PSRAM, so these fields do
     * not consume INTERNAL/DMA heap. They bracket camera_h264_stream_start()
     * and sample the first five seconds after the asynchronous camera start request. */
    bool              cam_start_probe_armed;
    uint8_t           cam_start_probe_next;
    int64_t           cam_start_probe_t0_ms;
    size_t            cam_start_probe_base_free;
    size_t            cam_start_probe_base_largest;
    size_t            cam_start_probe_prev_largest;

    /* [POST-IDLE-OFFER-PROBE-R1-20260915]
     * Diagnostic-only state for the surviving viewer after another viewer is
     * fully torn down.  smoke_sess_t is PSRAM-preferred; on the observed normal
     * PSRAM path these counters do not consume the INTERNAL|DMA heap. */
    volatile bool     idle_dma_probe_armed;
    uint8_t           idle_dma_probe_next;
    uint32_t          idle_dma_probe_t0_ms;
    uint32_t          idle_dma_probe_departed_sid;
    size_t            idle_dma_probe_base_free;
    size_t            idle_dma_probe_base_largest;
    size_t            idle_dma_probe_prev_largest;

    int64_t           start_ms;

    char             *local_sdp;                       /* on_msg SDP */
    char              cand[SMOKE_MAX_CAND][SMOKE_CAND_LEN]; /* "a=candidate:..." */
    int               cand_n;

#if SMOKE_REVERSE_AUDIO_RX_PHASE1
    /* [REV-AUDIO-RX-AB-PHASE1-20260917] Diagnostic-only receive counters.
     * The session object is PSRAM-preferred (see heap_caps_calloc at open), so
     * on the normal path these fields consume no INTERNAL|DMA heap. Updated only
     * from this session's single persistent pump (on_audio_* run inside
     * esp_peer_main_loop); atomics guard the teardown reader. */
    volatile uint32_t rx_audio_info_count;
    volatile uint32_t rx_audio_pkt_count;
    volatile uint32_t rx_audio_bytes;
    uint32_t          rx_audio_max_size;
    uint32_t          rx_audio_min_size;
    uint32_t          rx_audio_first_size;
    uint32_t          rx_audio_first_sr;
    uint8_t           rx_audio_first_ch;
    bool              rx_info_logged;
    bool              rx_first_data_logged;
    bool              rx_track_probe_logged;
    bool              rx_steady_logged;
#endif
} smoke_sess_t;

/* [LAN-DIRECT-ICE-ASYNC-R4-20260926]
 * Fixed two-slot resolver mailbox: no per-offer heap allocation.  Only the
 * worker transitions PENDING->RUNNING->HIT/MISS; offer/pump own FREE/PENDING
 * and terminal consumption. */
typedef enum {
    SMOKE_LAN_MDNS_JOB_FREE = 0,
    SMOKE_LAN_MDNS_JOB_PENDING,
    SMOKE_LAN_MDNS_JOB_RUNNING,
    SMOKE_LAN_MDNS_JOB_HIT,
    SMOKE_LAN_MDNS_JOB_MISS,
} smoke_lan_mdns_job_state_t;

typedef struct {
    volatile uint32_t state;
    uint32_t          session_id;
    char              name[128];
    char              ip[16];
    uint32_t          done_ms;
} smoke_lan_mdns_job_t;

#define SMOKE_LAN_CACHE_SRC_NONE 0U
#define SMOKE_LAN_CACHE_SRC_MDNS 1U
#define SMOKE_LAN_CACHE_SRC_STUN 2U

typedef struct {
    volatile uint32_t seq;   /* even=stable, odd=writer in progress */
    char              name[128];
    char              ip[16];
    uint32_t          saved_ms;
    uint8_t           source; /* mDNS discovery vs R5 repeated STUN-source confirmation */
} smoke_lan_mdns_cache_t;

/* [CAM-D2-D3-RECOVERY-PROBE-R1-20260915]
 * Functional behavior is unchanged: these are read-only heap snapshots from
 * the existing persistent pump. C0/C1 bracket camera_h264_stream_start();
 * C2..C10 sample the following five seconds to show exactly when a transient 3KB
 * largest-block loss appears and when it coalesces again. */
static const uint16_t s_cam_start_probe_delay_ms[] = {0U, 10U, 50U, 100U, 250U, 500U, 1000U, 2000U, 5000U};

static void smoke_cam_start_probe_tick(smoke_sess_t *s, int64_t now_ms)
{
    if (!s || !s->cam_start_probe_armed || s->cam_start_probe_t0_ms <= 0) return;

    const size_t n = sizeof(s_cam_start_probe_delay_ms) / sizeof(s_cam_start_probe_delay_ms[0]);
    int64_t elapsed_ms = now_ms - s->cam_start_probe_t0_ms;
    if (elapsed_ms < 0) elapsed_ms = 0;

    while ((size_t)s->cam_start_probe_next < n &&
           elapsed_ms >= (int64_t)s_cam_start_probe_delay_ms[s->cam_start_probe_next]) {
        const uint8_t idx = s->cam_start_probe_next;
        const size_t free_now = heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
        const size_t largest_now = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
        const int delta_base = (int)largest_now - (int)s->cam_start_probe_base_largest;
        const int delta_prev = (int)largest_now - (int)s->cam_start_probe_prev_largest;

        SMK_DMA_DIAG("[DMA-DIAG CAM-START-PROBE] sid=%u C%u target=+%ums actual=+%lldms free=%u largest=%u delta_base=%d delta_prev=%d camera_active=%u",
             (unsigned)s->session_id, (unsigned)(idx + 2U),
             (unsigned)s_cam_start_probe_delay_ms[idx], (long long)elapsed_ms,
             (unsigned)free_now, (unsigned)largest_now,
             delta_base, delta_prev, (unsigned)camera_h264_stream_is_active());

        s->cam_start_probe_prev_largest = largest_now;
        s->cam_start_probe_next = (uint8_t)(idx + 1U);
    }

    if ((size_t)s->cam_start_probe_next >= n) {
        s->cam_start_probe_armed = false;
    }
}

/* [POST-IDLE-OFFER-PROBE-R1-20260915]
 * No new task/timer/semaphore is created.  A surviving session's already
 * persistent pump samples INTERNAL|DMA after the other viewer has been fully
 * freed.  This tells us whether the 26/24/21KB staircase occurs while idle,
 * before the next /offer handler even begins. */
static const uint16_t s_idle_dma_probe_delay_ms[] = {100U, 250U, 500U, 1000U, 2000U, 3000U};

static void smoke_idle_dma_probe_arm(smoke_sess_t *survivor, uint32_t departed_sid)
{
    if (!survivor || !survivor->running) return;

    const size_t free_now = heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
    const size_t largest_now = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
    survivor->idle_dma_probe_next = 0U;
    survivor->idle_dma_probe_t0_ms = (uint32_t)(esp_timer_get_time() / 1000);
    survivor->idle_dma_probe_departed_sid = departed_sid;
    survivor->idle_dma_probe_base_free = free_now;
    survivor->idle_dma_probe_base_largest = largest_now;
    survivor->idle_dma_probe_prev_largest = largest_now;
    __atomic_store_n(&survivor->idle_dma_probe_armed, true, __ATOMIC_RELEASE);

    SMK_DMA_DIAG("[DMA-DIAG POST-IDLE-PROBE] survivor_sid=%u departed_sid=%u I0 target=+0ms free=%u largest=%u",
         (unsigned)survivor->session_id, (unsigned)departed_sid,
         (unsigned)free_now, (unsigned)largest_now);
}

static void smoke_idle_dma_probe_tick(smoke_sess_t *s, uint32_t now_ms)
{
    if (!s || !__atomic_load_n(&s->idle_dma_probe_armed, __ATOMIC_ACQUIRE)) return;

    const size_t n = sizeof(s_idle_dma_probe_delay_ms) / sizeof(s_idle_dma_probe_delay_ms[0]);
    const uint32_t elapsed_ms = now_ms - s->idle_dma_probe_t0_ms;

    while ((size_t)s->idle_dma_probe_next < n &&
           elapsed_ms >= (uint32_t)s_idle_dma_probe_delay_ms[s->idle_dma_probe_next]) {
        const uint8_t idx = s->idle_dma_probe_next;
        const size_t free_now = heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
        const size_t largest_now = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
        const int delta_base = (int)largest_now - (int)s->idle_dma_probe_base_largest;
        const int delta_prev = (int)largest_now - (int)s->idle_dma_probe_prev_largest;

        SMK_DMA_DIAG("[DMA-DIAG POST-IDLE-PROBE] survivor_sid=%u departed_sid=%u I%u target=+%ums actual=+%ums free=%u largest=%u delta_base=%d delta_prev=%d",
             (unsigned)s->session_id, (unsigned)s->idle_dma_probe_departed_sid,
             (unsigned)(idx + 1U), (unsigned)s_idle_dma_probe_delay_ms[idx],
             (unsigned)elapsed_ms, (unsigned)free_now, (unsigned)largest_now,
             delta_base, delta_prev);

        s->idle_dma_probe_prev_largest = largest_now;
        s->idle_dma_probe_next = (uint8_t)(idx + 1U);
    }

    if ((size_t)s->idle_dma_probe_next >= n) {
        __atomic_store_n(&s->idle_dma_probe_armed, false, __ATOMIC_RELEASE);
    }
}

/* Offer-handler probe is stack-only and read-only.  O0 is the very first line
 * of h_offer_post(), before body malloc/cJSON parsing.  Later O-points bracket
 * ISP lazy restore, read_body(), cJSON, s_gate/reap, candidate rewrite, and
 * smoke_start_locked(). */
static size_t smoke_offer_dma_probe_log(const char *phase,
                                        size_t base_largest, size_t prev_largest)
{
    const size_t free_now = heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
    const size_t largest_now = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
    SMK_DMA_DIAG("[DMA-DIAG OFFER-EARLY-PROBE] %s free=%u largest=%u delta_base=%d delta_prev=%d",
         phase ? phase : "?", (unsigned)free_now, (unsigned)largest_now,
         (int)largest_now - (int)base_largest,
         (int)largest_now - (int)prev_largest);
    return largest_now;
}

/* [PERSISTENT-PUMP-PER-SLOT-R1-20260915]
 * Two WebRTC pump workers are created once from static .bss storage.  Session
 * start/stop only assigns or releases a worker; it never allocates/frees a
 * 12KB FreeRTOS stack from the INTERNAL/DMA heap.  This prevents viewer #2
 * from splitting the last ~31KB contiguous DMA block. */
_Static_assert(sizeof(StackType_t) == 1,
               "ESP-IDF task stack depth/buffer is byte-based on this target");
static StaticTask_t s_pump_tcb[SMOKE_MAX_SESSIONS];
static StackType_t  s_pump_stack[SMOKE_MAX_SESSIONS][SMOKE_PUMP_STACK];
static TaskHandle_t s_pump_task[SMOKE_MAX_SESSIONS] = {0};
static volatile smoke_sess_t *s_pump_sess[SMOKE_MAX_SESSIONS] = {0};
static bool s_pump_workers_ready = false;

static SemaphoreHandle_t s_gate = NULL;        /* primary/secondary lifecycle */
static SemaphoreHandle_t s_fanout_lock = NULL; /* camera callback target snapshot */
static smoke_sess_t     *s_active = NULL;      /* primary, s_gate protects */
static smoke_sess_t     *s_secondary = NULL;   /* second viewer, s_gate protects */
static smoke_sess_t     *s_fanout[SMOKE_MAX_SESSIONS] = {0}; /* s_fanout_lock protects */
static uint32_t          s_next_session_id = 1; /* s_gate protects */
/* [DUPVIEW-DMA-DIAG-R1-20260918] s_gate protected; prevents duplicate log spam. */
static uint32_t          s_dupdiag_last_active_sid = 0U;
static uint32_t          s_dupdiag_last_secondary_sid = 0U;
static size_t            s_dma_bb_floor_largest = 0U;       /* s_gate protects: confirmed floor */
static size_t            s_dma_bb_candidate_largest = 0U;   /* s_gate protects */
static uint32_t          s_dma_bb_candidate_since_ms = 0U;  /* s_gate protects */
static uint8_t           s_dma_bb_candidate_samples = 0U;   /* s_gate protects */
static bool              s_dma_bb_candidate_edge_logged = false; /* s_gate protects */
/* [DMA-BLACKBOX-R4-SAFE-WALK-20260919] one active-edge snapshot per full-idle epoch. */
static bool              s_dma_heapmap_edge_dumped = false;      /* s_gate protects */

/* [DMA-BLACKBOX-R2-20260919] final-viewer full-idle recovery sampler. */
static StaticTimer_t      s_dma_idle_timer_storage;
static TimerHandle_t      s_dma_idle_timer = NULL;
static uint8_t            s_dma_idle_stage = 0U;
static uint8_t            s_dma_idle_retry = 0U;
static uint32_t           s_dma_idle_departed_sid = 0U;
static uint32_t           s_dma_idle_t0_ms = 0U;
static size_t             s_dma_idle_base_largest = 0U;
static size_t             s_dma_idle_prev_largest = 0U;
/* [DMA-BLACKBOX-R7-IDLE-DEFER-20260919]
 * FreeRTOS software-timer callbacks run on the shared Tmr Svc task.  R6 proved
 * that recording/logging/heap-walking there can overflow its stack.  The
 * callback now only posts a notification; persistent pump slot 0 performs the
 * real idle sample on its existing 12KB static stack. */
static volatile bool      s_dma_idle_work_pending = false;

/* [DMA-PREOFFER-R8-20260919]
 * R7 proved that the large 31KB -> 23KB step can happen before h_offer_post()
 * runs, so status-driven sampling starts too late. Reuse the EXISTING static
 * timer and persistent pump0 stack to sample pre-offer DMA without creating
 * any task/stack/heap object. Tmr Svc remains notify-only. */
#define SMOKE_DMA_TIMER_MODE_NONE      0U
#define SMOKE_DMA_TIMER_MODE_PREOFFER  1U
#define SMOKE_DMA_TIMER_MODE_IDLE      2U
#define SMOKE_DMA_PREOFFER_INTERVAL_MS 250U
#define SMOKE_DMA_PREOFFER_WARMUP_MS   500U
#define SMOKE_DMA_PREOFFER_MAX_MS      15000U
#define SMOKE_DMA_PREOFFER_LOG_STEP    1024U
static volatile uint8_t   s_dma_timer_mode = SMOKE_DMA_TIMER_MODE_NONE;
static uint32_t           s_dma_preoffer_t0_ms = 0U;
static size_t             s_dma_preoffer_base_largest = 0U;
static size_t             s_dma_preoffer_prev_largest = 0U;
static bool               s_dma_r8_first_status_seen = false;

static bool              s_cert_prepared = false;
static volatile bool     s_audio_gate_running = false;
/* [CAM-AUDIO-GATE-STATIC-F2-20260919]
 * The old one-shot xTaskCreatePinnedToCore() allocated an exact 3072-byte
 * INTERNAL stack on every first media start. R9 measured the matching
 * largest-block step before the camera task's first instruction. Keep one
 * persistent static worker instead: no runtime heap stack alloc/free. */
static StaticTask_t       s_audio_gate_tcb;
/* [CAM-AUDIO-GATE-PSRAM-F3-20260919]
 * Keep the 3072-byte gate stack permanently in PSRAM. The TCB remains static
 * internal storage, but the large stack no longer occupies or fragments the
 * INTERNAL|DMA heap either at boot or at camera start. */
static StackType_t       *s_audio_gate_stack = NULL;
static TaskHandle_t       s_audio_gate_task_handle = NULL;

/* [LAN-DIRECT-ICE-ASYNC-R4-20260926]
 * Resolver worker has a persistent PSRAM stack and static TCB.  The tiny job
 * mailbox/cache are static data: no per-offer malloc/free and no second peer. */
static StaticTask_t       s_lan_mdns_tcb;
static StackType_t       *s_lan_mdns_stack = NULL;
static TaskHandle_t       s_lan_mdns_task_handle = NULL;
static smoke_lan_mdns_job_t s_lan_mdns_jobs[SMOKE_MAX_SESSIONS];
static smoke_lan_mdns_cache_t s_lan_mdns_cache;

/* [LAN-STUN-SOURCE-R5-20260926]
 * Same-LAN evidence captured at the esp_peer UDP transport boundary.  The
 * transport hook only publishes the source of clear STUN Binding Requests; it
 * never allocates, logs, modifies or consumes a packet.  The normal pump later
 * matches this endpoint to the browser's original .local host-candidate port. */
#define SMOKE_LAN_STUN_CONFIRM_HITS          2U
#define SMOKE_LAN_STUN_CONFIRM_WINDOW_MS  1500U
#define SMOKE_LAN_STUN_EVIDENCE_MAX_AGE_MS 1200U
typedef struct {
    uint8_t  ipv4[4];
    uint16_t port;
    uint8_t  hits;
    uint32_t last_ms;
    uint32_t serial;
} smoke_lan_stun_evidence_t;
static portMUX_TYPE s_lan_stun_mux = portMUX_INITIALIZER_UNLOCKED;
static smoke_lan_stun_evidence_t s_lan_stun_evidence;

static bool smoke_lan_stun_private_v4_bytes(const uint8_t ip[4])
{
    if (!ip) return false;
    if (ip[0] == 10U) return true;
    if (ip[0] == 172U && ip[1] >= 16U && ip[1] <= 31U) return true;
    if (ip[0] == 192U && ip[1] == 168U) return true;
    return false;
}

/* Strong definition for the weak observer declared by esp_peer transport/udp.c.
 * This callback executes on the existing peer pump path. Keep it bounded and
 * allocation/log/mutex-free. ICE itself still validates the candidate after it
 * is injected; this observer is discovery evidence, not an authentication step. */
void esp_peer_udp_rx_observer(const esp_peer_addr_t *addr,
                              const uint8_t *buf, int len)
{
    if (!addr || !buf || len < 20 || addr->family != AF_INET) return;
    /* STUN Binding Request: type=0x0001 and RFC5389 magic cookie. */
    if (buf[0] != 0x00U || buf[1] != 0x01U ||
        buf[4] != 0x21U || buf[5] != 0x12U ||
        buf[6] != 0xA4U || buf[7] != 0x42U) return;
    if (!smoke_lan_stun_private_v4_bytes(addr->ipv4) || addr->port == 0U) return;

    const uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);
    portENTER_CRITICAL(&s_lan_stun_mux);
    const bool same =
        s_lan_stun_evidence.port == addr->port &&
        memcmp(s_lan_stun_evidence.ipv4, addr->ipv4, 4) == 0 &&
        (uint32_t)(now_ms - s_lan_stun_evidence.last_ms) <=
            SMOKE_LAN_STUN_CONFIRM_WINDOW_MS;
    if (!same) {
        memcpy(s_lan_stun_evidence.ipv4, addr->ipv4, 4);
        s_lan_stun_evidence.port = addr->port;
        s_lan_stun_evidence.hits = 1U;
    } else if (s_lan_stun_evidence.hits < 255U) {
        s_lan_stun_evidence.hits++;
    }
    s_lan_stun_evidence.last_ms = now_ms;
    s_lan_stun_evidence.serial++;
    portEXIT_CRITICAL(&s_lan_stun_mux);
}

static bool smoke_lan_stun_snapshot(smoke_lan_stun_evidence_t *out)
{
    if (!out) return false;
    portENTER_CRITICAL(&s_lan_stun_mux);
    *out = s_lan_stun_evidence;
    portEXIT_CRITICAL(&s_lan_stun_mux);
    return out->hits != 0U;
}


/* [CAM-SHARED-OP-COORD-R6-20260916]
 * Cross-browser coordination for the shared single camera/H.264 pipeline.
 * The existing calibration lease now also covers an explicit user-requested
 * 1080p <-> 720p60 mode switch. Other camera pages observe the lock through
 * the existing /api/smoke/status URI and close ONLY their own session. For
 * auto-cal they reconnect after release; for a MANUAL profile switch they stay
 * disconnected until their own user presses Start again. This guarantees the
 * profile owner is the only live peer before camera/H264 restart.
 * No new task/timer/semaphore/heap buffer is added.
 *
 * kind=0 : 720 auto calibration
 * kind=1 : manual shared profile switch
 *
 * The lease prevents a crashed/closed owner page from leaving viewers paused
 * forever. The browser owner refreshes it every few seconds. */
#define SMOKE_AUTO_CAL_LEASE_MS 60000LL
static bool     s_auto_cal_active = false;               /* s_gate protects */
static char     s_auto_cal_owner_token[SMOKE_CLIENT_TOKEN_HEX_LEN + 1U] = {0};
static int64_t  s_auto_cal_lease_until_ms = 0;           /* s_gate protects */
static uint32_t s_auto_cal_generation = 0;               /* s_gate protects */
static bool     s_auto_cal_profile_switch = false;        /* false=cal, true=profile */
static camera_h264_profile_t s_auto_cal_target_profile = CAMERA_H264_PROFILE_720P_NATIVE;
/* [DUPVIEW-DMA-DIAG-R1-20260918] Read-only heap snapshots; no allocation. */
static size_t   s_camop_diag_base_free = 0U;
static size_t   s_camop_diag_base_largest = 0U;
static uint32_t s_camop_diag_generation = 0U;

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
#define ADAPT_GOOD_HEADROOM_KBPS   2600U
/* [CAM-1080-ADAPT-GOP-R2D-20260913]
 * availableIncomingBitrate is application-limited on some Chrome/Android paths:
 * while the LTE profile sends a low bitrate, the estimate can sit around 1-2Mbps even on a fast LAN.
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

/* [DUPVIEW-DMA-DIAG-R1-20260918] Diagnostic-only identifiers.
 * FNV-1a hash is stored in existing Flight Recorder arg fields so a saved
 * snapshot can prove whether two sessions came from the same browser id. */
static bool smoke_diag_id_valid(const char *id)
{
    if (!id || strlen(id) != SMOKE_DIAG_ID_HEX_LEN) return false;
    for (size_t i = 0; i < SMOKE_DIAG_ID_HEX_LEN; ++i) {
        if (!isxdigit((unsigned char)id[i])) return false;
    }
    return true;
}

static uint32_t smoke_diag_hash32(const char *id)
{
    uint32_t h = 2166136261u;
    if (!id) return 0U;
    for (size_t i = 0; id[i] != '\0'; ++i) {
        h ^= (uint8_t)id[i];
        h *= 16777619u;
    }
    return h;
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


/* s_gate must be held. */
static bool smoke_auto_cal_owner_locked(const char *token)
{
    return s_auto_cal_active &&
           smoke_client_token_valid(token) &&
           strcmp(s_auto_cal_owner_token, token) == 0;
}

/* s_gate must be held. */
static void smoke_auto_cal_clear_locked(const char *why)
{
    if (s_auto_cal_active) {
        const size_t diag_free = heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
        const size_t diag_largest = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
        SMK_DMA_DIAG("[DMA-LIFECYCLE-DIAG-R1] END kind=%s target=%s gen=%u reason=%s free=%u largest=%u delta_free=%d delta_largest=%d",
             s_auto_cal_profile_switch ? "profile" : "cal",
             s_auto_cal_target_profile == CAMERA_H264_PROFILE_720P_NATIVE ? "720p60" : "1080p",
             (unsigned)s_auto_cal_generation, why ? why : "?",
             (unsigned)diag_free, (unsigned)diag_largest,
             s_camop_diag_generation == s_auto_cal_generation ?
                 ((int)diag_free - (int)s_camop_diag_base_free) : 0,
             s_camop_diag_generation == s_auto_cal_generation ?
                 ((int)diag_largest - (int)s_camop_diag_base_largest) : 0);
        SMKW("[CAM-SHARED-OP-COORD-R6] release kind=%s target=%s owner=%.8s gen=%u reason=%s",
             s_auto_cal_profile_switch ? "profile" : "cal",
             s_auto_cal_target_profile == CAMERA_H264_PROFILE_720P_NATIVE ? "720p60" : "1080p",
             s_auto_cal_owner_token,
             (unsigned)s_auto_cal_generation,
             why ? why : "?");
    }
    s_auto_cal_active = false;
    s_auto_cal_owner_token[0] = '\0';
    s_auto_cal_lease_until_ms = 0;
    s_auto_cal_profile_switch = false;
    s_auto_cal_target_profile = CAMERA_H264_PROFILE_720P_NATIVE;
    s_camop_diag_base_free = 0U;
    s_camop_diag_base_largest = 0U;
    s_camop_diag_generation = 0U;
}

/* s_gate must be held. */
static void smoke_auto_cal_expire_locked(void)
{
    if (!s_auto_cal_active) return;
    const int64_t now_ms = esp_timer_get_time() / 1000LL;
    if (s_auto_cal_lease_until_ms > 0 && now_ms >= s_auto_cal_lease_until_ms) {
        smoke_auto_cal_clear_locked("lease_expired");
    }
}

/* s_gate must be held. */
static void smoke_auto_cal_refresh_locked(void)
{
    s_auto_cal_lease_until_ms =
        (esp_timer_get_time() / 1000LL) + SMOKE_AUTO_CAL_LEASE_MS;
}

/* ????????????? SDP ?틸 ????????????? */

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

/* [CAM-USER-GAIN-R23-20260923]
 * Per-profile production GAIN preference.
 *
 * NVS is intentionally owned here instead of the OV02C10 driver: the driver
 * only clamps the currently active sensor, while this layer knows which
 * WebRTC profile (1080 or native 720p60) is being selected.
 *
 * live_* is RAM preview state; saved_* is the last committed NVS state.
 * Therefore the user can preview AUTO/MANUAL gain without flash writes and
 * the existing "user settings save" button remains the only commit point. */
#define CAM_GAIN_NVS_NS "cam_gain"
#define CAM_GAIN_1080_MAX 191U
#define CAM_GAIN_720_MAX  223U

typedef struct {
    bool loaded;
    bool live_manual[2];
    uint16_t live_value[2];
    bool saved_manual[2];
    uint16_t saved_value[2];
} camera_gain_pref_t;

static camera_gain_pref_t s_camera_gain = {
    .loaded = false,
    .live_manual = { false, false },
    .live_value = { 100U, 191U },
    .saved_manual = { false, false },
    .saved_value = { 100U, 191U },
};

static int camera_gain_profile_index(camera_h264_profile_t profile)
{
    return profile == CAMERA_H264_PROFILE_720P_NATIVE ? 1 : 0;
}

static uint32_t camera_gain_profile_max(camera_h264_profile_t profile)
{
    return profile == CAMERA_H264_PROFILE_720P_NATIVE ?
           CAM_GAIN_720_MAX : CAM_GAIN_1080_MAX;
}

static void camera_gain_load_once(void)
{
    if (s_camera_gain.loaded) return;

    bool manual[2] = { false, false };
    uint16_t value[2] = { 100U, 191U };

    nvs_handle_t h;
    if (nvs_open(CAM_GAIN_NVS_NS, NVS_READONLY, &h) == ESP_OK) {
        uint8_t m = 0;
        uint16_t v = 0;
        if (nvs_get_u8(h, "m1080", &m) == ESP_OK) manual[0] = (m != 0U);
        if (nvs_get_u16(h, "v1080", &v) == ESP_OK) value[0] = v;
        if (nvs_get_u8(h, "m720", &m) == ESP_OK) manual[1] = (m != 0U);
        if (nvs_get_u16(h, "v720", &v) == ESP_OK) value[1] = v;
        nvs_close(h);
    }

    if (value[0] > CAM_GAIN_1080_MAX) value[0] = CAM_GAIN_1080_MAX;
    if (value[1] > CAM_GAIN_720_MAX) value[1] = CAM_GAIN_720_MAX;

    for (int i = 0; i < 2; ++i) {
        s_camera_gain.saved_manual[i] = manual[i];
        s_camera_gain.saved_value[i] = value[i];
        s_camera_gain.live_manual[i] = manual[i];
        s_camera_gain.live_value[i] = value[i];
    }
    s_camera_gain.loaded = true;

    ESP_LOGI(TAG,
             "[CAM-USER-GAIN-R23] NVS load 1080=%s/%u 720=%s/%u",
             manual[0] ? "MANUAL" : "AUTO", (unsigned)value[0],
             manual[1] ? "MANUAL" : "AUTO", (unsigned)value[1]);
}

static bool camera_gain_dirty(camera_h264_profile_t profile)
{
    camera_gain_load_once();
    const int i = camera_gain_profile_index(profile);
    if (s_camera_gain.live_manual[i] != s_camera_gain.saved_manual[i]) return true;
    return s_camera_gain.live_manual[i] &&
           s_camera_gain.live_value[i] != s_camera_gain.saved_value[i];
}

static esp_err_t camera_gain_preview_set(camera_h264_profile_t profile,
                                         bool manual,
                                         uint32_t gain_index)
{
    camera_gain_load_once();
    const uint32_t max_gain = camera_gain_profile_max(profile);
    if (gain_index > max_gain) return ESP_ERR_INVALID_ARG;

    const int i = camera_gain_profile_index(profile);
    s_camera_gain.live_manual[i] = manual;
    s_camera_gain.live_value[i] = (uint16_t)gain_index;

    if (camera_h264_stream_is_active() &&
        camera_h264_stream_get_profile() == profile) {
        return esp_video_isp_pipeline_user_gain_set(manual, gain_index);
    }
    return ESP_OK;
}

static esp_err_t camera_gain_apply_profile(camera_h264_profile_t profile)
{
    camera_gain_load_once();
    const int i = camera_gain_profile_index(profile);
    esp_err_t err = esp_video_isp_pipeline_user_gain_set(s_camera_gain.live_manual[i],
                                                        s_camera_gain.live_value[i]);
    ESP_LOGI(TAG, "[CAM-AE-GOV-R24] apply profile=%s mode=%s gain=%u ret=%s",
             profile == CAMERA_H264_PROFILE_720P_NATIVE ? "720p60" : "1080p",
             s_camera_gain.live_manual[i] ? "MANUAL" : "AUTO",
             (unsigned)s_camera_gain.live_value[i], esp_err_to_name(err));
    return err;
}

static esp_err_t camera_gain_save_profile(camera_h264_profile_t profile)
{
    camera_gain_load_once();
    const int i = camera_gain_profile_index(profile);

    nvs_handle_t h;
    esp_err_t err = nvs_open(CAM_GAIN_NVS_NS, NVS_READWRITE, &h);
    if (err != ESP_OK) return err;

    if (i == 1) {
        err = nvs_set_u8(h, "m720", s_camera_gain.live_manual[i] ? 1U : 0U);
        if (err == ESP_OK) err = nvs_set_u16(h, "v720", s_camera_gain.live_value[i]);
    } else {
        err = nvs_set_u8(h, "m1080", s_camera_gain.live_manual[i] ? 1U : 0U);
        if (err == ESP_OK) err = nvs_set_u16(h, "v1080", s_camera_gain.live_value[i]);
    }
    if (err == ESP_OK) err = nvs_commit(h);
    nvs_close(h);

    if (err == ESP_OK) {
        s_camera_gain.saved_manual[i] = s_camera_gain.live_manual[i];
        s_camera_gain.saved_value[i] = s_camera_gain.live_value[i];
        ESP_LOGI(TAG, "[CAM-USER-GAIN-R23] NVS save profile=%s mode=%s gain=%u",
                 i ? "720p60" : "1080p",
                 s_camera_gain.saved_manual[i] ? "MANUAL" : "AUTO",
                 (unsigned)s_camera_gain.saved_value[i]);
    }
    return err;
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

    /* GAIN follows the same per-profile activation points as ISP tune so
     * start/profile-switch/rollback cannot accidentally keep the other
     * profile's manual sensor gain. */
    esp_err_t gain_err = camera_gain_apply_profile(profile);
    if (gain_err != ESP_OK) {
        ESP_LOGW(TAG, "[CAM-AE-GOV-R24] profile apply failed profile=%s err=%s",
                 camera_h264_stream_profile_name(profile), esp_err_to_name(gain_err));
        if (err == ESP_OK) err = gain_err;
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

/* remote SDP ?의 a=candidate ?로그?(IP vs .local ?별) */
static void log_remote_candidates(const char *sdp)
{
    if (!sdp) return;
    const char *p = sdp;
    while ((p = strstr(p, "a=candidate:")) != NULL) {
        /* ?식: a=candidate:<foundation> <comp> <transport> <prio> <ADDRESS> <port> typ ... */
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
/* [DMA-BOUNDARY-R17F9-PSRAM-SIGNALING-20260919]
 * Signaling/HTTP scratch is not DMA hardware memory.  Keep it out of the
 * INTERNAL|DMA heap so repeated offer/answer cycles cannot split the camera's
 * largest contiguous DMA block.  Deliberately NO internal fallback here: an
 * exhausted PSRAM signaling request should fail cleanly instead of consuming
 * the camera/SDIO DMA reserve. */
static void *smoke_sig_psram_alloc(size_t size)
{
    if (size == 0) size = 1;
    return heap_caps_malloc(size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
}

static void *smoke_sig_psram_realloc(void *ptr, size_t size)
{
    if (size == 0) size = 1;
    return heap_caps_realloc(ptr, size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
}

static char *smoke_sig_psram_strdup(const char *src)
{
    if (!src) return NULL;
    size_t n = strlen(src) + 1U;
    char *dst = (char *)smoke_sig_psram_alloc(n);
    if (!dst) return NULL;
    memcpy(dst, src, n);
    return dst;
}

/* [LAN-DIRECT-ICE-MDNS-R1-20260926]
 * Browser-side filtering used to remove Chrome's .local host candidate before
 * this HTTP handler could resolve it.  The browser now sends the complete SDP.
 *
 * For a public/non-RFC1918 HTTP peer, keep the old safety behavior here:
 * remove only .local candidate lines, and only when at least one non-.local
 * alternative candidate is present.  This prevents esp_peer 1.5.4 from seeing
 * an unresolved mDNS hostname while retaining srflx/relay fallback.
 *
 * For a private HTTP peer, smoke_rewrite_lan_candidates_ipv4() below rewrites
 * the .local ADDRESS to the exact numeric HTTP peer and preserves its original
 * ICE UDP port, enabling the LAN host<->host pair.
 */
static bool smoke_candidate_line_addr_is_mdns(const char *line, const char *line_end)
{
    static const char prefix[] = "a=candidate:";
    if (!line || !line_end || line_end <= line) return false;
    if ((size_t)(line_end - line) < sizeof(prefix) - 1U ||
        strncmp(line, prefix, sizeof(prefix) - 1U) != 0) {
        return false;
    }

    const char *r = line + sizeof(prefix) - 1U;
    for (int field = 0; field <= 4; ++field) {
        while (r < line_end && *r == ' ') r++;
        const char *tok = r;
        while (r < line_end && *r != ' ') r++;
        if (field == 4) {
            size_t len = (size_t)(r - tok);
            return len >= 6U && strncasecmp(r - 6, ".local", 6) == 0;
        }
    }
    return false;
}

/* [LAN-DIRECT-ICE-ASYNC-R4-20260926]
 * Extract the first Chrome mDNS host candidate from the ORIGINAL browser SDP
 * in esp_peer message form: "candidate:..." (no leading "a=", no CRLF).
 * The complete line is retained so ADDRESS can later be replaced without
 * changing foundation/component/transport/priority/port/type/ufrag tokens. */
static bool smoke_extract_mdns_candidate_message(const char *sdp,
                                                  char *out,
                                                  size_t out_sz,
                                                  uint16_t *port_out)
{
    if (port_out) *port_out = 0;
    if (!sdp || !out || out_sz == 0U) return false;
    out[0] = '\0';

    const char *p = sdp;
    while ((p = strstr(p, "a=candidate:")) != NULL) {
        const char *nl = strchr(p, '\n');
        const char *line_end = nl ? nl : p + strlen(p);
        if (line_end > p && line_end[-1] == '\r') line_end--;

        const char *r = p + strlen("a=candidate:");
        const char *addr_start = NULL;
        const char *addr_end = NULL;
        const char *port_start = NULL;
        const char *port_end = NULL;
        for (int field = 0; field <= 5 && r < line_end; ++field) {
            while (r < line_end && *r == ' ') r++;
            const char *tok = r;
            while (r < line_end && *r != ' ') r++;
            if (field == 4) {
                addr_start = tok;
                addr_end = r;
            } else if (field == 5) {
                port_start = tok;
                port_end = r;
            }
        }

        if (!addr_start || !addr_end || !port_start || !port_end ||
            addr_end <= addr_start || port_end <= port_start) {
            p += strlen("a=candidate:");
            continue;
        }

        const size_t addr_len = (size_t)(addr_end - addr_start);
        const bool is_mdns = addr_len >= 6U &&
                             strncasecmp(addr_end - 6, ".local", 6) == 0;
        if (!is_mdns) {
            p = line_end;
            continue;
        }

        char port_buf[8] = {0};
        const size_t port_len = (size_t)(port_end - port_start);
        if (port_len == 0U || port_len >= sizeof(port_buf)) return false;
        memcpy(port_buf, port_start, port_len);
        char *port_ep = NULL;
        unsigned long port_ul = strtoul(port_buf, &port_ep, 10);
        if (!port_ep || *port_ep != '\0' || port_ul == 0UL || port_ul > 65535UL) {
            return false;
        }

        const char *cand_start = p + 2; /* skip "a=" */
        const size_t cand_len = (size_t)(line_end - cand_start);
        if (cand_len + 1U > out_sz) return false;
        memcpy(out, cand_start, cand_len);
        out[cand_len] = '\0';
        if (port_out) *port_out = (uint16_t)port_ul;
        return true;
    }
    return false;
}

/* Rewrite ADDRESS field #4 of one "candidate:..." message.  Only a .local
 * source and RFC1918 destination are accepted; every other byte is preserved. */
static bool smoke_candidate_message_rewrite_ipv4(const char *candidate,
                                                  const char *ipv4,
                                                  char *out,
                                                  size_t out_sz)
{
    static const char prefix[] = "candidate:";
    if (!candidate || !ipv4 || !out || out_sz == 0U ||
        strncmp(candidate, prefix, sizeof(prefix) - 1U) != 0 ||
        !smoke_ipv4_is_private_lan(ipv4)) {
        return false;
    }

    const char *line_end = candidate + strlen(candidate);
    const char *r = candidate + sizeof(prefix) - 1U;
    const char *addr_start = NULL;
    const char *addr_end = NULL;
    for (int field = 0; field <= 4 && r < line_end; ++field) {
        while (r < line_end && *r == ' ') r++;
        const char *tok = r;
        while (r < line_end && *r != ' ') r++;
        if (field == 4) {
            addr_start = tok;
            addr_end = r;
        }
    }
    if (!addr_start || !addr_end || addr_end <= addr_start) return false;
    const size_t old_len = (size_t)(addr_end - addr_start);
    if (old_len < 6U || strncasecmp(addr_end - 6, ".local", 6) != 0) return false;

    const size_t head_len = (size_t)(addr_start - candidate);
    const size_t tail_len = (size_t)(line_end - addr_end);
    const size_t ip_len = strlen(ipv4);
    const size_t need = head_len + ip_len + tail_len + 1U;
    if (need > out_sz) return false;

    memcpy(out, candidate, head_len);
    memcpy(out + head_len, ipv4, ip_len);
    memcpy(out + head_len + ip_len, addr_end, tail_len);
    out[need - 1U] = '\0';
    return true;
}

static char *smoke_strip_mdns_candidates_if_alternative(const char *sdp,
                                                         int *stripped)
{
    if (stripped) *stripped = 0;
    if (!sdp) return NULL;

    int mdns_n = 0;
    int other_n = 0;
    const char *p = sdp;
    while (*p) {
        const char *nl = strchr(p, '\n');
        const char *seg_end = nl ? nl + 1 : p + strlen(p);
        const char *line_end = nl ? nl : seg_end;
        if (line_end > p && line_end[-1] == '\r') line_end--;
        if (strncmp(p, "a=candidate:", strlen("a=candidate:")) == 0) {
            if (smoke_candidate_line_addr_is_mdns(p, line_end)) mdns_n++;
            else other_n++;
        }
        p = seg_end;
    }

    if (mdns_n == 0 || other_n == 0) return NULL;

    size_t cap = strlen(sdp) + 1U;
    char *out = (char *)smoke_sig_psram_alloc(cap);
    if (!out) return NULL;

    size_t n = 0;
    p = sdp;
    while (*p) {
        const char *nl = strchr(p, '\n');
        const char *seg_end = nl ? nl + 1 : p + strlen(p);
        const char *line_end = nl ? nl : seg_end;
        if (line_end > p && line_end[-1] == '\r') line_end--;

        const bool drop =
            strncmp(p, "a=candidate:", strlen("a=candidate:")) == 0 &&
            smoke_candidate_line_addr_is_mdns(p, line_end);
        if (drop) {
            if (stripped) (*stripped)++;
        } else {
            size_t seg_len = (size_t)(seg_end - p);
            memcpy(out + n, p, seg_len);
            n += seg_len;
        }
        p = seg_end;
    }
    out[n] = '\0';
    return out;
}


/* [LAN-DIRECT-ICE-MDNS-R3-20260926]
 * When this page is reached through a public reverse-proxy / NAT-loopback
 * address, getpeername() sees the proxy/public hop instead of the browser's
 * RFC1918 address.  Chrome still advertises the real host socket as a random
 * <uuid>.local mDNS candidate.  Resolve that one-shot name directly on the
 * ESP's LAN with a bounded BSD-socket mDNS query, then feed the resolved
 * private IPv4 back into the existing R1 address-only SDP rewrite.
 *
 * Safety:
 *   - bound 5353 + multicast receive, <=450 ms total timeout;
 *   - accept only RFC1918 IPv4 A records;
 *   - candidate UDP port / priority / type / ufrag are never changed;
 *   - if resolution fails, the exact R1 external fallback remains: strip the
 *     unresolved .local line only when srflx/relay alternatives exist.
 * No TURN/DTLS/SDP_ONLY/STOP-PC-FIRST policy is changed.
 */
static uint16_t smoke_dns_u16(const uint8_t *p)
{
    return (uint16_t)(((uint16_t)p[0] << 8) | (uint16_t)p[1]);
}

static size_t smoke_dns_encode_qname(uint8_t *dst, size_t cap, const char *name)
{
    if (!dst || !name || !name[0]) return 0;
    size_t n = 0;
    const char *p = name;
    while (*p) {
        const char *dot = strchr(p, '.');
        size_t lab = dot ? (size_t)(dot - p) : strlen(p);
        if (lab == 0 || lab > 63 || n + 1U + lab + 1U > cap) return 0;
        dst[n++] = (uint8_t)lab;
        memcpy(dst + n, p, lab);
        n += lab;
        if (!dot) break;
        p = dot + 1;
    }
    if (n + 1U > cap) return 0;
    dst[n++] = 0;
    return n;
}

static bool smoke_first_mdns_candidate_name(const char *sdp,
                                            char *out,
                                            size_t out_sz)
{
    if (!sdp || !out || out_sz < 7U) return false;
    out[0] = '\0';

    const char *p = sdp;
    while ((p = strstr(p, "a=candidate:")) != NULL) {
        const char *line_end = strpbrk(p, "\r\n");
        if (!line_end) line_end = p + strlen(p);

        const char *r = p + strlen("a=candidate:");
        for (int field = 0; field <= 4 && r < line_end; ++field) {
            while (r < line_end && *r == ' ') r++;
            const char *tok = r;
            while (r < line_end && *r != ' ') r++;
            if (field == 4) {
                size_t len = (size_t)(r - tok);
                if (len >= 6U &&
                    strncasecmp(r - 6, ".local", 6) == 0 &&
                    len + 1U <= out_sz) {
                    memcpy(out, tok, len);
                    out[len] = '\0';
                    return true;
                }
                break;
            }
        }
        p = line_end;
    }
    return false;
}

static bool smoke_dns_read_name(const uint8_t *pkt,
                                size_t len,
                                size_t *off,
                                char *out,
                                size_t out_sz)
{
    if (!pkt || !off || !out || out_sz < 2U || *off >= len) return false;

    size_t p = *off;
    size_t resume = 0;
    size_t n = 0;
    bool jumped = false;

    for (int guard = 0; guard < 128 && p < len; ++guard) {
        uint8_t c = pkt[p++];
        if (c == 0) {
            if (!jumped) *off = p;
            else *off = resume;
            out[n] = '\0';
            return true;
        }
        if ((c & 0xc0U) == 0xc0U) {
            if (p >= len) return false;
            uint16_t ptr = (uint16_t)(((uint16_t)(c & 0x3fU) << 8) | pkt[p]);
            p++;
            if ((size_t)ptr >= len) return false;
            if (!jumped) {
                resume = p;
                jumped = true;
            }
            p = (size_t)ptr;
            continue;
        }
        if ((c & 0xc0U) != 0 || c > 63U || p + c > len) return false;
        if (n != 0) {
            if (n + 1U >= out_sz) return false;
            out[n++] = '.';
        }
        if (n + c >= out_sz) return false;
        memcpy(out + n, pkt + p, c);
        n += c;
        p += c;
    }
    return false;
}

static bool smoke_mdns_packet_find_private_a(const uint8_t *resp,
                                              size_t len,
                                              const char *wanted_name,
                                              char out[16])
{
    if (!resp || len < 12U || !wanted_name || !out) return false;

    uint16_t qd = smoke_dns_u16(resp + 4);
    uint16_t an = smoke_dns_u16(resp + 6);
    uint16_t ns = smoke_dns_u16(resp + 8);
    uint16_t ar = smoke_dns_u16(resp + 10);
    size_t off = 12U;

    for (uint16_t i = 0; i < qd; ++i) {
        char qname[128];
        if (!smoke_dns_read_name(resp, len, &off, qname, sizeof(qname)) || off + 4U > len) {
            return false;
        }
        off += 4U;
    }

    uint32_t rr_total = (uint32_t)an + (uint32_t)ns + (uint32_t)ar;
    for (uint32_t i = 0; i < rr_total; ++i) {
        char rr_name[128];
        if (!smoke_dns_read_name(resp, len, &off, rr_name, sizeof(rr_name)) || off + 10U > len) {
            return false;
        }

        uint16_t type = smoke_dns_u16(resp + off);
        uint16_t klass = smoke_dns_u16(resp + off + 2U);
        uint16_t rdlen = smoke_dns_u16(resp + off + 8U);
        off += 10U;
        if (off + rdlen > len) return false;

        if (type == 1U && (klass & 0x7fffU) == 1U && rdlen == 4U &&
            strcasecmp(rr_name, wanted_name) == 0) {
            struct in_addr a;
            memcpy(&a.s_addr, resp + off, 4U);
            char ip[16] = {0};
            if (inet_ntop(AF_INET, &a, ip, sizeof(ip)) &&
                smoke_ipv4_is_private_lan(ip)) {
                snprintf(out, 16, "%s", ip);
                return true;
            }
        }
        off += rdlen;
    }
    return false;
}

static bool smoke_mdns_resolve_private_ipv4(const char *name,
                                            char out[16],
                                            uint32_t timeout_ms)
{
    if (!name || !out || timeout_ms == 0) return false;
    out[0] = '\0';

    uint8_t query[256];
    memset(query, 0, sizeof(query));
    /* mDNS header: ID=0, flags=0, QDCOUNT=1. */
    query[5] = 1;
    size_t qn = smoke_dns_encode_qname(query + 12, sizeof(query) - 12, name);
    if (qn == 0 || 12U + qn + 4U > sizeof(query)) return false;
    size_t qlen = 12U + qn;
    query[qlen++] = 0x00; query[qlen++] = 0x01; /* QTYPE A */
    query[qlen++] = 0x80; query[qlen++] = 0x01; /* QCLASS IN + QU */

    int fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (fd < 0) {
        SMK_WEBRTC_DIAG("[LAN-DIRECT-ICE-ASYNC-R4] mDNS socket failed name=%s errno=%d", name, errno);
        return false;
    }

    int reuse = 1;
    (void)setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    bool bound_5353 = false;
    struct sockaddr_in bind_addr;
    memset(&bind_addr, 0, sizeof(bind_addr));
    bind_addr.sin_family = AF_INET;
    bind_addr.sin_port = htons(5353);
    bind_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    if (bind(fd, (struct sockaddr *)&bind_addr, sizeof(bind_addr)) == 0) {
        bound_5353 = true;
    } else {
        SMK_WEBRTC_DIAG("[LAN-DIRECT-ICE-ASYNC-R4] mDNS bind 5353 failed name=%s errno=%d; legacy source-port fallback",
             name, errno);
    }

#ifdef IP_MULTICAST_TTL
    uint8_t ttl = 255;
    (void)setsockopt(fd, IPPROTO_IP, IP_MULTICAST_TTL, &ttl, sizeof(ttl));
#endif

#ifdef IP_ADD_MEMBERSHIP
    bool joined = false;
    if (bound_5353) {
        struct ip_mreq mreq;
        memset(&mreq, 0, sizeof(mreq));
        if (inet_pton(AF_INET, "224.0.0.251", &mreq.imr_multiaddr) == 1) {
            mreq.imr_interface.s_addr = htonl(INADDR_ANY);
            if (setsockopt(fd, IPPROTO_IP, IP_ADD_MEMBERSHIP,
                           &mreq, sizeof(mreq)) == 0) {
                joined = true;
            } else {
                SMK_WEBRTC_DIAG("[LAN-DIRECT-ICE-ASYNC-R4] mDNS multicast join failed name=%s errno=%d; unicast QU still enabled",
                     name, errno);
            }
        }
    }
#else
    bool joined = false;
#endif

    struct sockaddr_in dst;
    memset(&dst, 0, sizeof(dst));
    dst.sin_family = AF_INET;
    dst.sin_port = htons(5353);
    if (inet_pton(AF_INET, "224.0.0.251", &dst.sin_addr) != 1) {
        close(fd);
        return false;
    }

    int sent = sendto(fd, query, qlen, 0, (struct sockaddr *)&dst, sizeof(dst));
    if (sent != (int)qlen) {
        int e = errno;
#ifdef IP_DROP_MEMBERSHIP
        if (joined) {
            struct ip_mreq mreq;
            memset(&mreq, 0, sizeof(mreq));
            (void)inet_pton(AF_INET, "224.0.0.251", &mreq.imr_multiaddr);
            mreq.imr_interface.s_addr = htonl(INADDR_ANY);
            (void)setsockopt(fd, IPPROTO_IP, IP_DROP_MEMBERSHIP, &mreq, sizeof(mreq));
        }
#endif
        close(fd);
        SMK_WEBRTC_DIAG("[LAN-DIRECT-ICE-ASYNC-R4] mDNS send failed name=%s sent=%d errno=%d",
             name, sent, e);
        return false;
    }

    const int64_t start_us = esp_timer_get_time();
    const int64_t deadline_us = start_us + (int64_t)timeout_ms * 1000LL;
    bool resent = false;
    int packets = 0;

    while (esp_timer_get_time() < deadline_us) {
        int64_t remain_us = deadline_us - esp_timer_get_time();
        if (remain_us <= 0) break;
        uint32_t slice_ms = (uint32_t)((remain_us + 999LL) / 1000LL);
        if (slice_ms > 100U) slice_ms = 100U;
        if (slice_ms == 0U) slice_ms = 1U;

        struct timeval tv = {
            .tv_sec = (time_t)(slice_ms / 1000U),
            .tv_usec = (suseconds_t)((slice_ms % 1000U) * 1000U),
        };
        (void)setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

        uint8_t resp[768];
        struct sockaddr_in from;
        socklen_t from_len = sizeof(from);
        int nr = recvfrom(fd, resp, sizeof(resp), 0,
                          (struct sockaddr *)&from, &from_len);
        if (nr >= 12) {
            packets++;
            if (smoke_mdns_packet_find_private_a(resp, (size_t)nr, name, out)) {
                char src_ip[16] = {0};
                (void)inet_ntop(AF_INET, &from.sin_addr, src_ip, sizeof(src_ip));
#ifdef IP_DROP_MEMBERSHIP
                if (joined) {
                    struct ip_mreq mreq;
                    memset(&mreq, 0, sizeof(mreq));
                    (void)inet_pton(AF_INET, "224.0.0.251", &mreq.imr_multiaddr);
                    mreq.imr_interface.s_addr = htonl(INADDR_ANY);
                    (void)setsockopt(fd, IPPROTO_IP, IP_DROP_MEMBERSHIP, &mreq, sizeof(mreq));
                }
#endif
                close(fd);
                SMK_WEBRTC_INFO("[LAN-DIRECT-ICE-ASYNC-R4] mDNS resolved %s -> %s responder=%s packets=%d bind5353=%d join=%d",
                    name, out, src_ip[0] ? src_ip : "?", packets,
                    bound_5353 ? 1 : 0, joined ? 1 : 0);
                return true;
            }
        }

        if (!resent && esp_timer_get_time() - start_us >= (int64_t)timeout_ms * 500LL) {
            (void)sendto(fd, query, qlen, 0, (struct sockaddr *)&dst, sizeof(dst));
            resent = true;
        }
    }

#ifdef IP_DROP_MEMBERSHIP
    if (joined) {
        struct ip_mreq mreq;
        memset(&mreq, 0, sizeof(mreq));
        (void)inet_pton(AF_INET, "224.0.0.251", &mreq.imr_multiaddr);
        mreq.imr_interface.s_addr = htonl(INADDR_ANY);
        (void)setsockopt(fd, IPPROTO_IP, IP_DROP_MEMBERSHIP, &mreq, sizeof(mreq));
    }
#endif
    close(fd);

    SMK_WEBRTC_INFO("[LAN-DIRECT-ICE-ASYNC-R4] mDNS resolve miss name=%s timeout=%ums packets=%d bind5353=%d join=%d",
        name, (unsigned)timeout_ms, packets, bound_5353 ? 1 : 0, joined ? 1 : 0);
    return false;
}

static void smoke_lan_mdns_cache_store(const char *name, const char *ip, uint8_t source)
{
    if (!name || !ip || !smoke_ipv4_is_private_lan(ip)) return;
    if (source != SMOKE_LAN_CACHE_SRC_MDNS && source != SMOKE_LAN_CACHE_SRC_STUN) {
        source = SMOKE_LAN_CACHE_SRC_NONE;
    }
    uint32_t seq = __atomic_load_n(&s_lan_mdns_cache.seq, __ATOMIC_RELAXED);
    if (seq & 1U) seq++;
    __atomic_store_n(&s_lan_mdns_cache.seq, seq + 1U, __ATOMIC_SEQ_CST);
    snprintf(s_lan_mdns_cache.name, sizeof(s_lan_mdns_cache.name), "%s", name);
    snprintf(s_lan_mdns_cache.ip, sizeof(s_lan_mdns_cache.ip), "%s", ip);
    s_lan_mdns_cache.saved_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);
    s_lan_mdns_cache.source = source;
    __atomic_store_n(&s_lan_mdns_cache.seq, seq + 2U, __ATOMIC_SEQ_CST);
}

static bool smoke_lan_mdns_cache_lookup(const char *name,
                                        char ip_out[16],
                                        uint32_t *age_ms_out,
                                        uint8_t *source_out)
{
    if (age_ms_out) *age_ms_out = 0U;
    if (source_out) *source_out = SMOKE_LAN_CACHE_SRC_NONE;
    if (!name || !ip_out) return false;
    ip_out[0] = '\0';

    char cached_name[128];
    char cached_ip[16];
    uint32_t saved_ms = 0U;
    uint8_t cached_source = SMOKE_LAN_CACHE_SRC_NONE;
    for (int retry = 0; retry < 4; ++retry) {
        uint32_t seq1 = __atomic_load_n(&s_lan_mdns_cache.seq, __ATOMIC_ACQUIRE);
        if (seq1 & 1U) continue;
        memcpy(cached_name, s_lan_mdns_cache.name, sizeof(cached_name));
        memcpy(cached_ip, s_lan_mdns_cache.ip, sizeof(cached_ip));
        saved_ms = s_lan_mdns_cache.saved_ms;
        cached_source = s_lan_mdns_cache.source;
        uint32_t seq2 = __atomic_load_n(&s_lan_mdns_cache.seq, __ATOMIC_ACQUIRE);
        if (seq1 == seq2 && !(seq2 & 1U)) {
            cached_name[sizeof(cached_name) - 1U] = '\0';
            cached_ip[sizeof(cached_ip) - 1U] = '\0';
            const uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);
            const uint32_t age_ms = now_ms - saved_ms;
            if (cached_name[0] && cached_ip[0] &&
                strcasecmp(cached_name, name) == 0 &&
                age_ms <= SMOKE_LAN_MDNS_CACHE_TTL_MS &&
                smoke_ipv4_is_private_lan(cached_ip)) {
                snprintf(ip_out, 16, "%s", cached_ip);
                if (age_ms_out) *age_ms_out = age_ms;
                if (source_out) *source_out = cached_source;
                return true;
            }
            return false;
        }
    }
    return false;
}

static void smoke_lan_mdns_worker_task(void *arg)
{
    (void)arg;
    SMK_WEBRTC_DIAG("[LAN-DIRECT-ICE-ASYNC-R4] worker ready core=%d stack=%uB PSRAM timeout=%ums cache_ttl=%ums",
         xPortGetCoreID(), (unsigned)(SMOKE_LAN_MDNS_WORKER_STACK * sizeof(StackType_t)),
         (unsigned)SMOKE_LAN_MDNS_ASYNC_TIMEOUT_MS, (unsigned)SMOKE_LAN_MDNS_CACHE_TTL_MS);

    for (;;) {
        (void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        for (;;) {
            int picked = -1;
            for (uint32_t i = 0; i < SMOKE_MAX_SESSIONS; ++i) {
                uint32_t expected = SMOKE_LAN_MDNS_JOB_PENDING;
                if (__atomic_compare_exchange_n(&s_lan_mdns_jobs[i].state,
                                                &expected,
                                                SMOKE_LAN_MDNS_JOB_RUNNING,
                                                false,
                                                __ATOMIC_ACQ_REL,
                                                __ATOMIC_ACQUIRE)) {
                    picked = (int)i;
                    break;
                }
            }
            if (picked < 0) break;

            smoke_lan_mdns_job_t *job = &s_lan_mdns_jobs[picked];
            const uint32_t sid = job->session_id;
            char name[128];
            snprintf(name, sizeof(name), "%s", job->name);
            char ip[16] = {0};
            const bool hit = smoke_mdns_resolve_private_ipv4(
                name, ip, SMOKE_LAN_MDNS_ASYNC_TIMEOUT_MS);

            if (hit) {
                snprintf(job->ip, sizeof(job->ip), "%s", ip);
                smoke_lan_mdns_cache_store(name, ip, SMOKE_LAN_CACHE_SRC_MDNS);
            } else {
                job->ip[0] = '\0';
            }
            job->done_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);
            __atomic_store_n(&job->state,
                             hit ? SMOKE_LAN_MDNS_JOB_HIT : SMOKE_LAN_MDNS_JOB_MISS,
                             __ATOMIC_RELEASE);
            SMK_WEBRTC_INFO("[LAN-DIRECT-ICE-ASYNC-R4] sid=%u worker result=%s name=%s ip=%s slot=%d",
                (unsigned)sid, hit ? "HIT" : "MISS", name,
                hit ? ip : "-", picked);
        }
    }
}

static esp_err_t smoke_lan_mdns_worker_init(void)
{
    if (s_lan_mdns_task_handle) return ESP_OK;
    if (!s_lan_mdns_stack) {
        s_lan_mdns_stack = (StackType_t *)heap_caps_malloc(
            SMOKE_LAN_MDNS_WORKER_STACK * sizeof(StackType_t),
            MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!s_lan_mdns_stack) return ESP_ERR_NO_MEM;
    }

    const size_t free_before = heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
    const size_t largest_before = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
    s_lan_mdns_task_handle = xTaskCreateStaticPinnedToCore(
        smoke_lan_mdns_worker_task, "smoke_lan_mdns",
        SMOKE_LAN_MDNS_WORKER_STACK, NULL, SMOKE_LAN_MDNS_WORKER_PRIO,
        s_lan_mdns_stack, &s_lan_mdns_tcb, SMOKE_PUMP_CORE);
    if (!s_lan_mdns_task_handle) {
        heap_caps_free(s_lan_mdns_stack);
        s_lan_mdns_stack = NULL;
        return ESP_ERR_NO_MEM;
    }
    const size_t free_after = heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
    const size_t largest_after = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
    SMK_WEBRTC_DIAG("[LAN-DIRECT-ICE-ASYNC-R4] persistent worker created PSRAM-stack DMA free=%u->%u largest=%u->%u",
         (unsigned)free_before, (unsigned)free_after,
         (unsigned)largest_before, (unsigned)largest_after);
    return ESP_OK;
}

/* Caller holds s_gate.  Never reuse PENDING/RUNNING slots. Terminal slots are
 * reclaimable only when their old session is no longer active. */
static bool smoke_lan_mdns_sid_active_locked(uint32_t sid)
{
    return (s_active && s_active->session_id == sid) ||
           (s_secondary && s_secondary->session_id == sid);
}

static int smoke_lan_mdns_schedule_locked(uint32_t sid, const char *name)
{
    if (!s_lan_mdns_task_handle || !name || !name[0]) return -1;
    int slot = -1;
    for (uint32_t i = 0; i < SMOKE_MAX_SESSIONS; ++i) {
        uint32_t st = __atomic_load_n(&s_lan_mdns_jobs[i].state, __ATOMIC_ACQUIRE);
        if (st == SMOKE_LAN_MDNS_JOB_FREE ||
            ((st == SMOKE_LAN_MDNS_JOB_HIT || st == SMOKE_LAN_MDNS_JOB_MISS) &&
             !smoke_lan_mdns_sid_active_locked(s_lan_mdns_jobs[i].session_id))) {
            slot = (int)i;
            break;
        }
    }
    if (slot < 0) return -1;

    smoke_lan_mdns_job_t *job = &s_lan_mdns_jobs[slot];
    job->session_id = sid;
    snprintf(job->name, sizeof(job->name), "%s", name);
    job->ip[0] = '\0';
    job->done_ms = 0U;
    __atomic_store_n(&job->state, SMOKE_LAN_MDNS_JOB_PENDING, __ATOMIC_RELEASE);
    xTaskNotifyGive(s_lan_mdns_task_handle);
    return slot;
}

static void smoke_lan_mdns_poll_result(smoke_sess_t *s)
{
    if (!s || s->lan_mdns_job_slot == SMOKE_LAN_MDNS_JOB_NONE ||
        s->lan_mdns_job_slot >= SMOKE_MAX_SESSIONS) {
        return;
    }
    smoke_lan_mdns_job_t *job = &s_lan_mdns_jobs[s->lan_mdns_job_slot];
    uint32_t st = __atomic_load_n(&job->state, __ATOMIC_ACQUIRE);
    if ((st != SMOKE_LAN_MDNS_JOB_HIT && st != SMOKE_LAN_MDNS_JOB_MISS) ||
        job->session_id != s->session_id) {
        return;
    }

    if (st == SMOKE_LAN_MDNS_JOB_HIT && smoke_ipv4_is_private_lan(job->ip)) {
        snprintf(s->lan_mdns_resolved_ip, sizeof(s->lan_mdns_resolved_ip), "%s", job->ip);
        __atomic_store_n(&s->lan_mdns_candidate_ready, true, __ATOMIC_RELEASE);
        const bool pairing_open =
            __atomic_load_n(&s->lan_mdns_pairing_open, __ATOMIC_ACQUIRE);
        if (pairing_open &&
            !__atomic_load_n(&s->lan_mdns_inject_attempted, __ATOMIC_ACQUIRE)) {
            __atomic_store_n(&s->lan_mdns_inject_pending, true, __ATOMIC_RELEASE);
        }
        SMK_WEBRTC_DIAG("[LAN-DIRECT-ICE-ASYNC-R4] sid=%u RESULT HIT %s -> %s port=%u pairing=%d free=%u largest=%u",
             (unsigned)s->session_id, s->lan_mdns_name, s->lan_mdns_resolved_ip,
             (unsigned)s->lan_mdns_candidate_port, pairing_open ? 1 : 0,
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA),
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA));
    } else {
        SMK_WEBRTC_INFO("[LAN-DIRECT-ICE-ASYNC-R4] sid=%u RESULT MISS name=%s; TURN/srflx unchanged",
            (unsigned)s->session_id, s->lan_mdns_name);
    }

    __atomic_store_n(&job->state, SMOKE_LAN_MDNS_JOB_FREE, __ATOMIC_RELEASE);
    s->lan_mdns_job_slot = SMOKE_LAN_MDNS_JOB_NONE;
}

/* [LAN-STUN-SOURCE-R5-20260926]
 * Consume only recent, repeated same-LAN STUN Binding Requests whose source
 * UDP port exactly matches this session's original Chrome host-candidate port.
 * Two matching checks are enough to discover the address; esp_peer's own ICE
 * transaction remains the authority. R5F2 stores SRC_STUN cache only after the
 * PAIRED callback reports the exact learned IPv4:port as the selected pair. */
static void smoke_lan_stun_poll_result(smoke_sess_t *s)
{
    if (!s || !s->running || s->peer_connected || s->dtls_connecting ||
        !s->lan_mdns_candidate_template[0] || s->lan_mdns_candidate_port == 0U ||
        !__atomic_load_n(&s->lan_mdns_pairing_open, __ATOMIC_ACQUIRE) ||
        __atomic_load_n(&s->lan_mdns_candidate_ready, __ATOMIC_ACQUIRE) ||
        __atomic_load_n(&s->lan_mdns_inject_attempted, __ATOMIC_ACQUIRE)) {
        return;
    }

    smoke_lan_stun_evidence_t ev;
    if (!smoke_lan_stun_snapshot(&ev) ||
        ev.hits < SMOKE_LAN_STUN_CONFIRM_HITS ||
        ev.port != s->lan_mdns_candidate_port ||
        !smoke_lan_stun_private_v4_bytes(ev.ipv4)) {
        return;
    }
    const uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);
    const uint32_t age_ms = now_ms - ev.last_ms;
    if (age_ms > SMOKE_LAN_STUN_EVIDENCE_MAX_AGE_MS) return;

    char ip[16];
    snprintf(ip, sizeof(ip), "%u.%u.%u.%u",
             (unsigned)ev.ipv4[0], (unsigned)ev.ipv4[1],
             (unsigned)ev.ipv4[2], (unsigned)ev.ipv4[3]);
    snprintf(s->lan_mdns_resolved_ip, sizeof(s->lan_mdns_resolved_ip), "%s", ip);
    snprintf(s->lan_stun_confirmed_ip, sizeof(s->lan_stun_confirmed_ip), "%s", ip);
    s->lan_stun_confirmed_port = ev.port;
    /* R5F2: this is discovery evidence only.  Do NOT publish a trusted STUN
     * cache entry until ESP_PEER_STATE_PAIRED confirms this exact endpoint. */
    __atomic_store_n(&s->lan_stun_source_confirmed, true, __ATOMIC_RELEASE);
    __atomic_store_n(&s->lan_mdns_candidate_ready, true, __ATOMIC_RELEASE);
    __atomic_store_n(&s->lan_mdns_inject_pending, true, __ATOMIC_RELEASE);
    SMKW("[LAN-STUN-SOURCE-R5] sid=%u CONFIRMED source=%s:%u hits=%u age=%ums -> candidate pending free=%u largest=%u",
         (unsigned)s->session_id, ip, (unsigned)ev.port, (unsigned)ev.hits,
         (unsigned)age_ms,
         (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA),
         (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA));
}

/* Caller owns s->peer_mtx.  Safe to call both before and after main_loop():
 * before handles a resolver result that arrived between pump iterations;
 * after handles PAIRING published by the main_loop callback itself. */
static void smoke_lan_mdns_inject_locked(smoke_sess_t *s)
{
    if (!s || !s->peer || !s->running || s->peer_connected || s->dtls_connecting ||
        !__atomic_load_n(&s->lan_mdns_pairing_open, __ATOMIC_ACQUIRE) ||
        !__atomic_exchange_n(&s->lan_mdns_inject_pending, false, __ATOMIC_ACQ_REL) ||
        !__atomic_load_n(&s->lan_mdns_candidate_ready, __ATOMIC_ACQUIRE) ||
        __atomic_exchange_n(&s->lan_mdns_inject_attempted, true, __ATOMIC_ACQ_REL)) {
        return;
    }

    char candidate[SMOKE_CAND_LEN];
    const bool candidate_ok = smoke_candidate_message_rewrite_ipv4(
        s->lan_mdns_candidate_template, s->lan_mdns_resolved_ip,
        candidate, sizeof(candidate));
    const size_t free_before =
        heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
    const size_t largest_before =
        heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
    int inject_r = ESP_PEER_ERR_FAIL;
    if (candidate_ok) {
        esp_peer_msg_t msg = {
            .type = ESP_PEER_MSG_TYPE_CANDIDATE,
            .data = (uint8_t *)candidate,
            .size = (int)strlen(candidate),
        };
        inject_r = esp_peer_send_msg(s->peer, &msg);
    }
    s->lan_mdns_inject_result = inject_r;
    const size_t free_after =
        heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
    const size_t largest_after =
        heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
    SMK_WEBRTC_DIAG("[LAN-DIRECT-ICE-ASYNC-R4] sid=%u INJECT candidate_ok=%d r=%d target=%s:%u free=%u->%u largest=%u->%u delta_largest=%d",
         (unsigned)s->session_id, candidate_ok ? 1 : 0, inject_r,
         s->lan_mdns_resolved_ip, (unsigned)s->lan_mdns_candidate_port,
         (unsigned)free_before, (unsigned)free_after,
         (unsigned)largest_before, (unsigned)largest_after,
         (int)largest_after - (int)largest_before);
}

static char *smoke_rewrite_lan_candidates_ipv4(const char *sdp,
                                                const char *ipv4,
                                                int *rewritten)
{
    if (rewritten) *rewritten = 0;
    if (!sdp || !ipv4 || !ipv4[0] || !smoke_ipv4_is_private_lan(ipv4)) return NULL;

    const int candidate_count = count_sdp_candidate_lines(sdp);
    const bool force_single = (candidate_count == 1);
    char *copy = smoke_sig_psram_strdup(sdp);
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
            char *grown = (char *)smoke_sig_psram_realloc(copy, cur_len + grow + 1U);
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

/* local_sdp + ?집??candidate ??쳐 non-trickle SDP 문자?을 만든???출??free) */
static char *build_aggregated_sdp(smoke_sess_t *s)
{
    if (!s->local_sdp) return NULL;
    size_t need = strlen(s->local_sdp) + 64;
    for (int i = 0; i < s->cand_n; i++) need += strlen(s->cand[i]) + 3;
    need += strlen("a=end-of-candidates\r\n") + 4;
    char *out = (char *)smoke_sig_psram_alloc(need);
    if (!out) {
        SMKE("[DMA-BOUNDARY-R17F9] OOM aggregated SDP in PSRAM need=%u", (unsigned)need);
        return NULL;
    }
    size_t n = 0;
    n += snprintf(out + n, need - n, "%s", s->local_sdp);
    /* SDP ?이 개행?로 ?나?록 보정 */
    if (n >= 2 && !(out[n-1] == '\n')) { out[n++] = '\r'; out[n++] = '\n'; out[n] = '\0'; }
    for (int i = 0; i < s->cand_n; i++)
        n += snprintf(out + n, need - n, "%s\r\n", s->cand[i]);
    n += snprintf(out + n, need - n, "a=end-of-candidates\r\n");
    return out;
}

/* ????????????? esp_peer 콜백 (pump ?스??컨텍?트) ????????????? */

/* [WEBRTC-2CLIENT-FANOUT-AB] Session table is intentionally tiny.
 * Control/lifecycle uses s_gate; the camera callback never takes s_gate. */
static uint32_t smoke_session_count_locked(void)
{
    return (s_active ? 1U : 0U) + (s_secondary ? 1U : 0U);
}

/* [CAM-STALE-SLOT-VIEW0-FIX-R9-20260917]
 * Allocated peer slots and actually CONNECTED viewers are different states.
 * A not-yet-connected/stale slot must never be reported as "2 viewers". */
static uint32_t smoke_session_connected_count_locked(void)
{
    uint32_t n = 0U;
    if (s_active && s_active->peer_connected) n++;
    if (s_secondary && s_secondary->peer_connected) n++;
    return n;
}

/* [CAM-VIEWER-HEARTBEAT-R1-20260918]
 * CONNECTED is transport state; viewer count additionally requires a recent,
 * visible browser report with an advancing decoded-frame counter. Legacy
 * engineering clients that did not opt in retain the old CONNECTED meaning. */
static bool smoke_session_is_live_viewer_locked(const smoke_sess_t *s, uint32_t now_ms)
{
    if (!s || !s->peer_connected) return false;
    if (!s->viewer_hb_required) return true;
    const bool seen = __atomic_load_n(&s->viewer_hb_seen, __ATOMIC_ACQUIRE);
    const bool visible = __atomic_load_n(&s->viewer_hb_visible, __ATOMIC_ACQUIRE);
    const uint32_t last = __atomic_load_n(&s->viewer_hb_last_ms, __ATOMIC_ACQUIRE);
    /* [CAM-VIEWER-HB-SIGNED-AGE-R1-20260919]
     * The pump can sample now_ms immediately before the HTTP status task stores
     * a heartbeat one millisecond newer.  Unsigned subtraction then becomes
     * UINT32_MAX and falsely looks like a 49-day timeout.  A signed modular
     * delta rejects that tiny future timestamp while retaining 32-bit wrap. */
    const int32_t age_ms = (int32_t)(now_ms - last);
    return seen && visible && last != 0U &&
           age_ms >= 0 &&
           age_ms < (int32_t)SMOKE_VIEWER_HEARTBEAT_TIMEOUT_MS;
}

static uint32_t smoke_session_viewer_count_locked(void)
{
    const uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);
    uint32_t n = 0U;
    if (smoke_session_is_live_viewer_locked(s_active, now_ms)) n++;
    if (smoke_session_is_live_viewer_locked(s_secondary, now_ms)) n++;
    return n;
}

static bool smoke_viewer_hb_expired(const smoke_sess_t *s, uint32_t now_ms,
                                    uint32_t *age_ms_out)
{
    if (age_ms_out) *age_ms_out = 0U;
    if (!s || !s->viewer_hb_required || !s->running || !s->peer_connected) return false;
    const uint32_t last = __atomic_load_n(&s->viewer_hb_last_ms, __ATOMIC_ACQUIRE);
    const int32_t age_ms = (int32_t)(now_ms - last);
    if (last == 0U || age_ms < (int32_t)SMOKE_VIEWER_HEARTBEAT_TIMEOUT_MS) {
        return false;
    }
    if (age_ms_out) *age_ms_out = (uint32_t)age_ms;
    return true;
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


/* [FLIGHT-RECORDER-STAGE2-R1-20260916]
 * Locked helper is used only while s_gate is held. Peer callback helper never
 * dereferences s_active/s_secondary, so it cannot create another lifecycle
 * race while teardown is in progress. No allocation occurs in either path. */
static void smoke_fr_fill_locked(flight_recorder_ctx_t *ctx,
                                 smoke_sess_t *sess,
                                 camera_h264_profile_t requested_profile,
                                 const char *client_token,
                                 int32_t arg0, int32_t arg1)
{
    if (!ctx) return;
    memset(ctx, 0, sizeof(*ctx));
    ctx->sid = sess ? sess->session_id : 0U;
    ctx->active_sid = s_active ? s_active->session_id : 0U;
    ctx->secondary_sid = s_secondary ? s_secondary->session_id : 0U;
    ctx->peer_count = (uint8_t)smoke_session_count_locked();
    smoke_sess_t *list[SMOKE_MAX_SESSIONS] = {s_active, s_secondary};
    for (uint32_t i = 0; i < SMOKE_MAX_SESSIONS; ++i) {
        if (list[i] && list[i]->peer_connected) ctx->connected_count++;
    }
    ctx->fanout_count = (uint8_t)smoke_fanout_count();
    ctx->requested_profile = (uint8_t)requested_profile;
    if (sess) ctx->current_profile = (uint8_t)sess->profile;
    else if (s_active) ctx->current_profile = (uint8_t)s_active->profile;
    else ctx->current_profile = 0xFFU;
    if (sess) {
        ctx->session_running = sess->running;
        ctx->session_connected = sess->peer_connected;
        ctx->session_teardown = __atomic_load_n(&sess->teardown_started, __ATOMIC_ACQUIRE);
    }
    ctx->camera_active = camera_h264_stream_is_active();
    ctx->audio_active = camera_webrtc_audio_is_active();
    ctx->client_token = client_token ? client_token : (sess ? sess->client_token : NULL);
    ctx->arg0 = arg0;
    ctx->arg1 = arg1;
}

static void smoke_fr_record_locked(flight_recorder_event_t event,
                                   smoke_sess_t *sess,
                                   camera_h264_profile_t requested_profile,
                                   const char *client_token,
                                   const char *tag,
                                   int32_t arg0, int32_t arg1)
{
    flight_recorder_ctx_t ctx;
    smoke_fr_fill_locked(&ctx, sess, requested_profile, client_token, arg0, arg1);
    flight_recorder_record(event, &ctx, tag);
}

static void smoke_fr_trigger_locked(flight_recorder_event_t event,
                                    smoke_sess_t *sess,
                                    camera_h264_profile_t requested_profile,
                                    const char *client_token,
                                    const char *tag,
                                    int32_t arg0, int32_t arg1)
{
    flight_recorder_ctx_t ctx;
    smoke_fr_fill_locked(&ctx, sess, requested_profile, client_token, arg0, arg1);
    flight_recorder_trigger(event, &ctx, tag);
}

/* [DMA-OFFER-PROBE-R6-20260919]
 * O0..O4 execute before h_offer_post() owns s_gate. Do not inspect
 * s_active/s_secondary there: doing so would either race or require taking the
 * lifecycle gate and would perturb the exact allocation window we are trying
 * to measure. Record only allocator state plus camera/audio state; 0xFF means
 * peer/connected/fanout counts intentionally unavailable in the pre-gate
 * samples. No heap allocation is performed here. */
static void smoke_fr_offer_probe_unlocked(const char *tag, int32_t stage,
                                          size_t base_largest)
{
    flight_recorder_ctx_t ctx;
    memset(&ctx, 0, sizeof(ctx));
    ctx.peer_count = 0xFFU;
    ctx.connected_count = 0xFFU;
    ctx.fanout_count = 0xFFU;
    ctx.requested_profile = 0xFFU;
    ctx.current_profile = 0xFFU;
    ctx.camera_active = camera_h264_stream_is_active();
    ctx.audio_active = camera_webrtc_audio_is_active();
    ctx.arg0 = stage;
    const size_t now_largest =
        heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
    ctx.arg1 = (int32_t)now_largest - (int32_t)base_largest;
    flight_recorder_record(FR_EVENT_OFFER_PROBE, &ctx, tag);
}

static int32_t smoke_fr_offer_probe_delta(size_t base_largest)
{
    const size_t now_largest =
        heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
    return (int32_t)now_largest - (int32_t)base_largest;
}

/* [DMA-BLACKBOX-R4-SAFE-WALK-20260919]
 * R3 used heap_caps_dump(), which emits every heap block while the heap walker
 * is active and proved too expensive on this P4 build (Core0 interrupt-WDT).
 * R4 walks the same INTERNAL|DMA heaps but the callback performs NO logging,
 * allocation, lock, or other service call. It only copies >=2KB blocks into
 * a fixed static table. After heap_caps_walk() returns, a short bounded summary
 * is printed. This preserves the useful 4KB-block evidence without hundreds
 * of UART lines inside the heap walk. */
#define SMOKE_DMA_R4_MIN_BLOCK_BYTES 2048U
#define SMOKE_DMA_R4_MAX_BLOCKS      24U

typedef struct {
    intptr_t heap_start;
    intptr_t heap_end;
    void *ptr;
    size_t size;
    bool used;
} smoke_dma_r4_block_t;

typedef struct {
    uint32_t total_seen;
    uint32_t matched_seen;
    uint32_t dropped;
    uint32_t used_matched;
    uint32_t free_matched;
    smoke_dma_r4_block_t blocks[SMOKE_DMA_R4_MAX_BLOCKS];
} smoke_dma_r4_walk_t;

static smoke_dma_r4_walk_t s_dma_r4_walk;

static bool smoke_dma_r4_walk_cb(walker_heap_into_t heap_info,
                                 walker_block_info_t block_info,
                                 void *user_data)
{
    smoke_dma_r4_walk_t *w = (smoke_dma_r4_walk_t *)user_data;
    if (!w) return false;
    w->total_seen++;
    if (block_info.size < SMOKE_DMA_R4_MIN_BLOCK_BYTES) return true;

    w->matched_seen++;
    if (block_info.used) w->used_matched++;
    else w->free_matched++;

    if (w->matched_seen <= SMOKE_DMA_R4_MAX_BLOCKS) {
        smoke_dma_r4_block_t *e = &w->blocks[w->matched_seen - 1U];
        e->heap_start = heap_info.start;
        e->heap_end = heap_info.end;
        e->ptr = block_info.ptr;
        e->size = block_info.size;
        e->used = block_info.used;
    } else {
        w->dropped++;
    }
    return true;
}

/* [DMA-BB-EXPECTED-R3-20261010] DMA-OWNER-PROBE (diagnostic only)
 * Prints the stack start of a few long-lived service tasks and which >=2KB
 * INTERNAL|DMA block (from the R4 walk table just filled) contains it.
 * Purpose: PROVE (address equality) the owner of the idle 40960 -> 36864 step
 * (B23 4096 USED at the heap3 tail) instead of inferring it from size/timing.
 * Read-only: no allocation, no free, no task/stack change.  xTaskGetHandle()
 * is IDF-enabled (INCLUDE_xTaskGetHandle=1); names >= configMAX_TASK_NAME_LEN
 * are skipped because xTaskGetHandle() asserts on them. */
static void smoke_dma_owner_probe_locked(const char *reason)
{
    static const char *const names[] = {
        "mqtt_status", "mqtt_delay", "mqtt_task", "httpd", "weather",
    };
    const uint32_t emit = s_dma_r4_walk.matched_seen < SMOKE_DMA_R4_MAX_BLOCKS
                        ? s_dma_r4_walk.matched_seen : SMOKE_DMA_R4_MAX_BLOCKS;
    for (size_t n = 0; n < sizeof(names) / sizeof(names[0]); ++n) {
        if (strlen(names[n]) >= (size_t)configMAX_TASK_NAME_LEN) continue;
        TaskHandle_t h = xTaskGetHandle(names[n]);
        if (h == NULL) {
            SMKW("[DMA-OWNER-PROBE-R3] reason=%s task=%s handle=none",
                 reason ? reason : "?", names[n]);
            continue;
        }
        const uintptr_t stk = (uintptr_t)pxTaskGetStackStart(h);
        int hit = -1;
        uint32_t off = 0U;
        for (uint32_t i = 0; i < emit; ++i) {
            const smoke_dma_r4_block_t *e = &s_dma_r4_walk.blocks[i];
            const uintptr_t p = (uintptr_t)e->ptr;
            if (stk >= p && stk < p + e->size) {
                hit = (int)i;
                off = (uint32_t)(stk - p);
                break;
            }
        }
        if (hit >= 0) {
            const smoke_dma_r4_block_t *e = &s_dma_r4_walk.blocks[hit];
            SMKW("[DMA-OWNER-PROBE-R3] reason=%s task=%s stack=%p block=B%02d ptr=%p size=%u %s off=%u",
                 reason ? reason : "?", names[n], (void *)stk, hit,
                 e->ptr, (unsigned)e->size, e->used ? "USED" : "FREE",
                 (unsigned)off);
        } else {
            SMKW("[DMA-OWNER-PROBE-R3] reason=%s task=%s stack=%p block=none(<2KB-table/PSRAM/other)",
                 reason ? reason : "?", names[n], (void *)stk);
        }
    }
}

static void smoke_dma_heapmap_dump_locked(const char *reason,
                                          size_t reference_largest,
                                          size_t observed_largest)
{
    const uint32_t caps = MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA;
    multi_heap_info_t info = {0};
    heap_caps_get_info(&info, caps);
    const bool integrity_ok = heap_caps_check_integrity(caps, false);

    SMKW("[DMA-HEAPMAP-R4] BEGIN reason=%s ref=%u observed=%u free=%u largest=%u min=%u free_blocks=%u alloc_blocks=%u total_blocks=%u allocated=%u integrity=%u peers=%u connected=%u fanout=%u camera=%u audio=%u min_block=%u cap=%u",
         reason ? reason : "?",
         (unsigned)reference_largest, (unsigned)observed_largest,
         (unsigned)info.total_free_bytes, (unsigned)info.largest_free_block,
         (unsigned)info.minimum_free_bytes, (unsigned)info.free_blocks,
         (unsigned)info.allocated_blocks, (unsigned)info.total_blocks,
         (unsigned)info.total_allocated_bytes, integrity_ok ? 1U : 0U,
         (unsigned)smoke_session_count_locked(),
         (unsigned)smoke_session_connected_count_locked(),
         (unsigned)smoke_fanout_count(),
         camera_h264_stream_is_active() ? 1U : 0U,
         camera_webrtc_audio_is_active() ? 1U : 0U,
         (unsigned)SMOKE_DMA_R4_MIN_BLOCK_BYTES,
         (unsigned)SMOKE_DMA_R4_MAX_BLOCKS);

    memset(&s_dma_r4_walk, 0, sizeof(s_dma_r4_walk));
    heap_caps_walk(caps, smoke_dma_r4_walk_cb, &s_dma_r4_walk);

    const uint32_t emit = s_dma_r4_walk.matched_seen < SMOKE_DMA_R4_MAX_BLOCKS
                        ? s_dma_r4_walk.matched_seen : SMOKE_DMA_R4_MAX_BLOCKS;
    for (uint32_t i = 0; i < emit; ++i) {
        const smoke_dma_r4_block_t *e = &s_dma_r4_walk.blocks[i];
        SMKW("[DMA-HEAPMAP-R4] B%02u heap=%p..%p ptr=%p size=%u %s",
             (unsigned)i,
             (void *)e->heap_start, (void *)e->heap_end,
             e->ptr, (unsigned)e->size,
             e->used ? "USED" : "FREE");
    }

    multi_heap_info_t after = {0};
    heap_caps_get_info(&after, caps);
    SMKW("[DMA-HEAPMAP-R4] END reason=%s total_seen=%u matched=%u used=%u free=%u dropped=%u dma_free=%u largest=%u integrity=%u",
         reason ? reason : "?",
         (unsigned)s_dma_r4_walk.total_seen,
         (unsigned)s_dma_r4_walk.matched_seen,
         (unsigned)s_dma_r4_walk.used_matched,
         (unsigned)s_dma_r4_walk.free_matched,
         (unsigned)s_dma_r4_walk.dropped,
         (unsigned)after.total_free_bytes,
         (unsigned)after.largest_free_block,
         heap_caps_check_integrity(caps, false) ? 1U : 0U);
    smoke_dma_owner_probe_locked(reason); /* [DMA-BB-EXPECTED-R3-20261010] read-only */
}

/* [DMA-BB-EXPECTED-R3-20261010] EXPECTED_H264_DB_TMP classification.
 *
 * Hardware-proven (CAM-DMA-OWNER-R1B, S4 + WEATHER-PSRAM-STACK-R1 layout):
 *   1080 /dev/video11 CAPTURE STREAMON allocates db_tmp=15488 B INTERNAL at
 *   heap3 tail (0x4ff96d40) -> INTERNAL|DMA largest 36864 -> 31744 (the 32KB
 *   reserve island stays intact); STREAMOFF frees the same block.
 *
 * Only that exact confirmed transition, while a real 1080 stream is running
 * (camera running, opened 1920x1080, encoded_frames>0, selected+session
 * profile 1080, >=1 session/connected/fanout, candidate STARTED while 1080 was
 * already running) is logged as EXPECTED and does NOT call the flash trigger.
 *
 * It cannot hide a true failure:
 *  - floor becomes 31744 during the hold, so any further >=1KB drop
 *    (e.g. 31744 -> 28672) goes through the unchanged EDGE/PERSIST path;
 *  - the candidate is the MINIMUM seen, so a hidden deeper dip fails the
 *    exact-match test and stays DMA_DROP;
 *  - HOLD_END (1080 no longer running, or final-viewer IDLE0) restores the
 *    pre-hold floor 36864, so non-recovery after teardown confirms as a
 *    normal DMA_DROP (camera=0) with flash save;
 *  - idle 40960 -> 36864, 720p, camera-off and any other numbers are untouched.
 * No allocation/free, no H264/REF/WEB20/camera/WebRTC policy change. */
#define SMOKE_DMA_BB_EXP_1080_PREV_LARGEST 36864U
#define SMOKE_DMA_BB_EXP_1080_CONF_LARGEST 31744U
#define SMOKE_DMA_BB_EXP_1080_WIDTH        1920U
#define SMOKE_DMA_BB_EXP_1080_HEIGHT       1080U

static size_t   s_dma_bb_exp_base_floor = 0U;      /* s_gate; !=0 => hold active */
static uint32_t s_dma_bb_exp_hold_since_ms = 0U;   /* s_gate */
static uint32_t s_dma_bb_exp_count = 0U;           /* s_gate; accepted this boot */
static bool     s_dma_bb_cand_started_1080 = false; /* s_gate; set at candidate start */

static bool smoke_dma_bb_1080_running_now(void)
{
    return camera_h264_stream_is_active() &&
           camera_h264_stream_get_profile() == CAMERA_H264_PROFILE_1080P;
}

/* s_gate held. Ends an EXPECTED hold and restores the pre-hold floor. */
static void smoke_dma_bb_expected_hold_end_locked(const char *why, uint32_t now_ms)
{
    if (s_dma_bb_exp_base_floor == 0U) return;
    const size_t base = s_dma_bb_exp_base_floor;
    const size_t old_floor = s_dma_bb_floor_largest;
    const size_t largest_now = heap_caps_get_largest_free_block(
        MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
    s_dma_bb_exp_base_floor = 0U;
    if (base > s_dma_bb_floor_largest) {
        s_dma_bb_floor_largest = base;
        /* Any running candidate was measured against the lower hold floor;
         * restart it against the restored floor (re-detected next sample). */
        s_dma_bb_candidate_largest = 0U;
        s_dma_bb_candidate_since_ms = 0U;
        s_dma_bb_candidate_samples = 0U;
        s_dma_bb_candidate_edge_logged = false;
        s_dma_bb_cand_started_1080 = false;
        /* Allow one R4 heap map at the post-teardown EDGE (if any). */
        s_dma_heapmap_edge_dumped = false;
    }
    SMKW("[DMA-BB-EXPECTED-R3] HOLD_END why=%s floor %u -> %u largest_now=%u held_ms=%u camera=%u (non-recovery below floor now confirms as DMA_DROP)",
         why ? why : "?", (unsigned)old_floor, (unsigned)s_dma_bb_floor_largest,
         (unsigned)largest_now,
         (unsigned)(now_ms - s_dma_bb_exp_hold_since_ms),
         camera_h264_stream_is_active() ? 1U : 0U);
}

/* s_gate held; called at the top of every R2 sample. */
static void smoke_dma_bb_expected_hold_check_locked(uint32_t now_ms)
{
    if (s_dma_bb_exp_base_floor == 0U) return;
    if (smoke_dma_bb_1080_running_now()) return;
    smoke_dma_bb_expected_hold_end_locked("1080-not-running", now_ms);
}

/* s_gate held; called at the confirm point instead of the flash trigger only
 * when every condition holds. Returns true when classified EXPECTED. */
static bool __attribute__((noinline))
smoke_dma_bb_expected_try_locked(size_t previous, size_t confirmed,
                                 smoke_sess_t *sess,
                                 camera_h264_profile_t profile,
                                 uint32_t now_ms)
{
    if (s_dma_bb_exp_base_floor != 0U) return false;
    if (previous != SMOKE_DMA_BB_EXP_1080_PREV_LARGEST) return false;
    if (confirmed != SMOKE_DMA_BB_EXP_1080_CONF_LARGEST) return false;
    if (!s_dma_bb_cand_started_1080) return false;
    if (sess == NULL || profile != CAMERA_H264_PROFILE_1080P) return false;
    if (!smoke_dma_bb_1080_running_now()) return false;
    const unsigned peers = (unsigned)smoke_session_count_locked();
    const unsigned connected = (unsigned)smoke_session_connected_count_locked();
    const unsigned fanout = (unsigned)smoke_fanout_count();
    if (peers == 0U || connected == 0U || fanout == 0U) return false;

    camera_h264_stream_stats_t cs = {0};
    camera_h264_stream_get_stats(&cs);
    if (!cs.running ||
        cs.width != SMOKE_DMA_BB_EXP_1080_WIDTH ||
        cs.height != SMOKE_DMA_BB_EXP_1080_HEIGHT ||
        cs.encoded_frames == 0U) {
        return false;
    }

    const uint32_t age_ms = now_ms - s_dma_bb_candidate_since_ms;
    const unsigned samples = (unsigned)s_dma_bb_candidate_samples;
    s_dma_bb_exp_base_floor = previous;
    s_dma_bb_exp_hold_since_ms = now_ms;
    s_dma_bb_exp_count++;
    s_dma_bb_floor_largest = confirmed;
    s_dma_bb_candidate_largest = 0U;
    s_dma_bb_candidate_since_ms = 0U;
    s_dma_bb_candidate_samples = 0U;
    s_dma_bb_candidate_edge_logged = false;
    s_dma_bb_cand_started_1080 = false;
    SMKW("[DMA-BLACKBOX-EXPECTED] EXPECTED_H264_DB_TMP largest %u -> %u bytes samples=%u age_ms=%u peers=%u connected=%u fanout=%u camera=1 %ux%u enc_frames=%u audio=%u n=%u hold_base=%u flash_trigger=0",
         (unsigned)previous, (unsigned)confirmed, samples, (unsigned)age_ms,
         peers, connected, fanout,
         (unsigned)cs.width, (unsigned)cs.height, (unsigned)cs.encoded_frames,
         camera_webrtc_audio_is_active() ? 1U : 0U,
         (unsigned)s_dma_bb_exp_count, (unsigned)s_dma_bb_exp_base_floor);
    return true;
}

/* [DMA-BLACKBOX-R2-20260919]
 * Keep the confirmed floor separate from a possible lower candidate.  The
 * first >=2KB step is recorded immediately (DMA_EDGE), then R1's 3-sample/2s
 * rule decides whether it becomes DMA_DROP or recovers as DMA_RECOVER.
 * arg0=previous/reference largest bytes, arg1=new/current largest bytes. */
static void smoke_dma_blackbox_sample_locked(uint32_t now_ms)
{
    const size_t largest = heap_caps_get_largest_free_block(
        MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
    if (largest == 0U) return;
    smoke_dma_bb_expected_hold_check_locked(now_ms); /* [DMA-BB-EXPECTED-R3-20261010] */

    if (s_dma_bb_floor_largest == 0U) {
        s_dma_bb_floor_largest = largest;
        return;
    }

    if (largest + SMOKE_DMA_DROP_MIN_BYTES > s_dma_bb_floor_largest) {
        if (s_dma_bb_candidate_largest != 0U && s_dma_bb_candidate_edge_logged) {
            smoke_sess_t *sess = s_active ? s_active : s_secondary;
            camera_h264_profile_t profile = sess ? sess->profile : camera_h264_stream_get_profile();
            smoke_fr_record_locked(FR_EVENT_DMA_RECOVER, sess, profile,
                                   sess ? sess->client_token : NULL, "DMA_RECOV",
                                   (int32_t)s_dma_bb_candidate_largest, (int32_t)largest);
            SMKW("[DMA-BLACKBOX-R2] RECOVER candidate=%u -> largest=%u floor=%u age_ms=%u",
                 (unsigned)s_dma_bb_candidate_largest, (unsigned)largest,
                 (unsigned)s_dma_bb_floor_largest,
                 (unsigned)(now_ms - s_dma_bb_candidate_since_ms));
        }
        s_dma_bb_candidate_largest = 0U;
        s_dma_bb_candidate_since_ms = 0U;
        s_dma_bb_candidate_samples = 0U;
        s_dma_bb_candidate_edge_logged = false;
        return;
    }

    if (s_dma_bb_candidate_largest == 0U) {
        s_dma_bb_candidate_largest = largest;
        s_dma_bb_candidate_since_ms = now_ms;
        s_dma_bb_candidate_samples = 1U;
        s_dma_bb_cand_started_1080 = smoke_dma_bb_1080_running_now(); /* [DMA-BB-EXPECTED-R3-20261010] */
        if (largest + SMOKE_DMA_EDGE_MIN_BYTES <= s_dma_bb_floor_largest) {
            smoke_sess_t *sess = s_active ? s_active : s_secondary;
            camera_h264_profile_t profile = sess ? sess->profile : camera_h264_stream_get_profile();
            smoke_fr_record_locked(FR_EVENT_DMA_EDGE, sess, profile,
                                   sess ? sess->client_token : NULL, "DMA_EDGE",
                                   (int32_t)s_dma_bb_floor_largest, (int32_t)largest);
            s_dma_bb_candidate_edge_logged = true;
            SMKW("[DMA-BLACKBOX-R2] EDGE largest %u -> %u bytes peers=%u connected=%u fanout=%u camera=%u audio=%u",
                 (unsigned)s_dma_bb_floor_largest, (unsigned)largest,
                 (unsigned)smoke_session_count_locked(),
                 (unsigned)smoke_session_connected_count_locked(),
                 (unsigned)smoke_fanout_count(),
                 camera_h264_stream_is_active() ? 1U : 0U,
                 camera_webrtc_audio_is_active() ? 1U : 0U);
            if (!s_dma_heapmap_edge_dumped) {
                s_dma_heapmap_edge_dumped = true;
                smoke_dma_heapmap_dump_locked("EDGE", s_dma_bb_floor_largest, largest);
            }
        }
        return;
    }

    if (largest < s_dma_bb_candidate_largest) {
        s_dma_bb_candidate_largest = largest;
    }
    if (!s_dma_bb_candidate_edge_logged &&
        s_dma_bb_candidate_largest + SMOKE_DMA_EDGE_MIN_BYTES <= s_dma_bb_floor_largest) {
        smoke_sess_t *sess = s_active ? s_active : s_secondary;
        camera_h264_profile_t profile = sess ? sess->profile : camera_h264_stream_get_profile();
        smoke_fr_record_locked(FR_EVENT_DMA_EDGE, sess, profile,
                               sess ? sess->client_token : NULL, "DMA_EDGE",
                               (int32_t)s_dma_bb_floor_largest,
                               (int32_t)s_dma_bb_candidate_largest);
        s_dma_bb_candidate_edge_logged = true;
        SMKW("[DMA-BLACKBOX-R2] EDGE largest %u -> %u bytes (candidate deepened)",
             (unsigned)s_dma_bb_floor_largest,
             (unsigned)s_dma_bb_candidate_largest);
        if (!s_dma_heapmap_edge_dumped) {
            s_dma_heapmap_edge_dumped = true;
            smoke_dma_heapmap_dump_locked("EDGE", s_dma_bb_floor_largest,
                                           s_dma_bb_candidate_largest);
        }
    }
    if (s_dma_bb_candidate_samples < UINT8_MAX) s_dma_bb_candidate_samples++;

    if (s_dma_bb_candidate_samples < SMOKE_DMA_DROP_CONFIRM_SAMPLES ||
        (uint32_t)(now_ms - s_dma_bb_candidate_since_ms) < SMOKE_DMA_DROP_CONFIRM_MS) {
        return;
    }

    const size_t previous = s_dma_bb_floor_largest;
    const size_t confirmed = s_dma_bb_candidate_largest;
    smoke_sess_t *sess = s_active ? s_active : s_secondary;
    camera_h264_profile_t profile = sess ? sess->profile : camera_h264_stream_get_profile();
    /* [DMA-BB-EXPECTED-R3-20261010] proven 1080 db_tmp step: log only, no flash. */
    if (smoke_dma_bb_expected_try_locked(previous, confirmed, sess, profile, now_ms)) {
        return;
    }

    /* [DMA-BLACKBOX-R5-AUTO-PERSIST-20260919]
     * A confirmed >=2s DMA drop is the black-box event we cannot afford to
     * lose on the next reset. EDGE/RECOVER remain RAM-only to avoid flash
     * churn; only confirmed PERSIST freezes the ring and requests a flash save. */
    smoke_fr_trigger_locked(FR_EVENT_DMA_DROP, sess, profile,
                            sess ? sess->client_token : NULL, "DMA_DROP",
                            (int32_t)previous, (int32_t)confirmed);
    SMKW("[DMA-BLACKBOX-R2] PERSIST largest %u -> %u bytes samples=%u age_ms=%u peers=%u connected=%u fanout=%u camera=%u audio=%u flash_trigger=1",
         (unsigned)previous, (unsigned)confirmed,
         (unsigned)s_dma_bb_candidate_samples,
         (unsigned)(now_ms - s_dma_bb_candidate_since_ms),
         (unsigned)smoke_session_count_locked(),
         (unsigned)smoke_session_connected_count_locked(),
         (unsigned)smoke_fanout_count(),
         camera_h264_stream_is_active() ? 1U : 0U,
         camera_webrtc_audio_is_active() ? 1U : 0U);
    s_dma_bb_floor_largest = confirmed;
    s_dma_bb_candidate_largest = 0U;
    s_dma_bb_candidate_since_ms = 0U;
    s_dma_bb_candidate_samples = 0U;
    s_dma_bb_candidate_edge_logged = false;
}

/* Absolute samples after the final session has been freed. The one-shot static
 * timer is re-armed with the delta to the next target. */
static const uint16_t s_dma_idle_target_ms[] = {100U, 500U, 2000U, 5000U};
static const uint16_t s_dma_idle_delta_ms[]  = {100U, 400U, 1500U, 3000U};

static void smoke_dma_idle_record_locked(const char *tag, uint32_t target_ms,
                                         uint32_t actual_ms)
{
    const size_t largest = heap_caps_get_largest_free_block(
        MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
    smoke_sess_t *sess = s_active ? s_active : s_secondary;
    camera_h264_profile_t profile = sess ? sess->profile : camera_h264_stream_get_profile();
    smoke_fr_record_locked(FR_EVENT_DMA_IDLE, sess, profile,
                           sess ? sess->client_token : NULL, tag,
                           (int32_t)target_ms, (int32_t)actual_ms);
    SMKW("[DMA-BLACKBOX-IDLE-R2] departed_sid=%u %s target=+%ums actual=+%ums free=%u largest=%u delta_base=%d delta_prev=%d peers=%u connected=%u fanout=%u camera=%u audio=%u",
         (unsigned)s_dma_idle_departed_sid, tag ? tag : "IDLE?",
         (unsigned)target_ms, (unsigned)actual_ms,
         (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA),
         (unsigned)largest,
         (int)largest - (int)s_dma_idle_base_largest,
         (int)largest - (int)s_dma_idle_prev_largest,
         (unsigned)smoke_session_count_locked(),
         (unsigned)smoke_session_connected_count_locked(),
         (unsigned)smoke_fanout_count(),
         camera_h264_stream_is_active() ? 1U : 0U,
         camera_webrtc_audio_is_active() ? 1U : 0U);
    s_dma_idle_prev_largest = largest;
}


static void smoke_dma_r8_phase_log(const char *tag)
{
    /* Deliberately does not inspect s_active/s_secondary: several call sites
     * run before or outside s_gate. This probe must not perturb lifecycle. */
    SMK_DMA_DIAG("[DMA-PREOFFER-R8] %s free=%u largest=%u camera=%u audio=%u",
         tag ? tag : "?",
         (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA),
         (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA),
         camera_h264_stream_is_active() ? 1U : 0U,
         camera_webrtc_audio_is_active() ? 1U : 0U);
}

static void smoke_dma_preoffer_watch_stop(const char *reason)
{
    __atomic_store_n(&s_dma_timer_mode, SMOKE_DMA_TIMER_MODE_NONE, __ATOMIC_RELEASE);
    if (s_dma_idle_timer) {
        (void)xTimerStop(s_dma_idle_timer, 0);
    }
    SMK_DMA_DIAG("[DMA-PREOFFER-R8] STOP reason=%s elapsed=%ums free=%u largest=%u",
         reason ? reason : "?",
         (unsigned)((uint32_t)(esp_timer_get_time() / 1000ULL) - s_dma_preoffer_t0_ms),
         (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA),
         (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA));
}

static void smoke_dma_preoffer_worker_run(void)
{
    if (__atomic_load_n(&s_dma_timer_mode, __ATOMIC_ACQUIRE) !=
            SMOKE_DMA_TIMER_MODE_PREOFFER) {
        return;
    }

    if (!s_gate || xSemaphoreTake(s_gate, pdMS_TO_TICKS(100)) != pdTRUE) {
        if (s_dma_idle_timer &&
            __atomic_load_n(&s_dma_timer_mode, __ATOMIC_ACQUIRE) ==
                SMOKE_DMA_TIMER_MODE_PREOFFER) {
            (void)xTimerChangePeriod(s_dma_idle_timer,
                                     pdMS_TO_TICKS(SMOKE_DMA_PREOFFER_INTERVAL_MS), 0);
        }
        return;
    }

    const uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);
    const uint32_t elapsed_ms = now_ms - s_dma_preoffer_t0_ms;
    const size_t free_now =
        heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
    const size_t largest =
        heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);

    if (s_dma_preoffer_base_largest == 0U) {
        /* Wait for the rest of GOT_IP service registration/MQTT startup to
         * settle, then establish the pre-browser baseline. */
        s_dma_preoffer_base_largest = largest;
        s_dma_preoffer_prev_largest = largest;
        s_dma_bb_floor_largest = largest;
        s_dma_bb_candidate_largest = 0U;
        s_dma_bb_candidate_since_ms = 0U;
        s_dma_bb_candidate_samples = 0U;
        s_dma_bb_candidate_edge_logged = false;
        s_dma_heapmap_edge_dumped = false;
        SMK_DMA_DIAG("[DMA-PREOFFER-R8] BASELINE elapsed=%ums free=%u largest=%u",
             (unsigned)elapsed_ms, (unsigned)free_now, (unsigned)largest);
    } else {
        const size_t prev = s_dma_preoffer_prev_largest;
        const size_t diff = (largest > prev) ? (largest - prev) : (prev - largest);
        if (diff >= SMOKE_DMA_PREOFFER_LOG_STEP) {
            SMK_DMA_DIAG("[DMA-PREOFFER-R8] STEP elapsed=%ums free=%u largest=%u prev=%u delta=%d base=%u delta_base=%d peers=%u camera=%u audio=%u",
                 (unsigned)elapsed_ms, (unsigned)free_now, (unsigned)largest,
                 (unsigned)prev, (int)largest - (int)prev,
                 (unsigned)s_dma_preoffer_base_largest,
                 (int)largest - (int)s_dma_preoffer_base_largest,
                 (unsigned)smoke_session_count_locked(),
                 camera_h264_stream_is_active() ? 1U : 0U,
                 camera_webrtc_audio_is_active() ? 1U : 0U);
            s_dma_preoffer_prev_largest = largest;
        }

        /* Same R2/R4/R5 detector, now active before the first browser offer. */
        smoke_dma_blackbox_sample_locked(now_ms);
    }

    const bool session_started = smoke_session_count_locked() != 0U;
    const bool camera_started = camera_h264_stream_is_active();
    const bool timeout = elapsed_ms >= SMOKE_DMA_PREOFFER_MAX_MS;
    xSemaphoreGive(s_gate);

    /* R5 flash persistence must also work when no /status request exists yet. */
    (void)flight_recorder_service();

    if (session_started || camera_started || timeout) {
        smoke_dma_preoffer_watch_stop(session_started ? "session_started" :
                                      camera_started ? "camera_started" : "timeout");
        return;
    }

    if (s_dma_idle_timer &&
        __atomic_load_n(&s_dma_timer_mode, __ATOMIC_ACQUIRE) ==
            SMOKE_DMA_TIMER_MODE_PREOFFER) {
        (void)xTimerChangePeriod(s_dma_idle_timer,
                                 pdMS_TO_TICKS(SMOKE_DMA_PREOFFER_INTERVAL_MS), 0);
    }
}

static void smoke_dma_preoffer_watch_arm(void)
{
    if (!s_dma_idle_timer || !s_pump_workers_ready || !s_gate) return;
    if (xSemaphoreTake(s_gate, pdMS_TO_TICKS(100)) != pdTRUE) return;

    const size_t largest =
        heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
    s_dma_preoffer_t0_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);
    s_dma_preoffer_base_largest = 0U;  /* established after warmup on pump0 */
    s_dma_preoffer_prev_largest = largest;

    __atomic_store_n(&s_dma_idle_work_pending, false, __ATOMIC_RELEASE);
    __atomic_store_n(&s_dma_timer_mode, SMOKE_DMA_TIMER_MODE_PREOFFER,
                     __ATOMIC_RELEASE);
    xSemaphoreGive(s_gate);

    SMK_DMA_DIAG("[DMA-PREOFFER-R8] ARM current_free=%u current_largest=%u warmup=%ums interval=%ums max=%ums",
         (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA),
         (unsigned)largest,
         (unsigned)SMOKE_DMA_PREOFFER_WARMUP_MS,
         (unsigned)SMOKE_DMA_PREOFFER_INTERVAL_MS,
         (unsigned)SMOKE_DMA_PREOFFER_MAX_MS);
    (void)xTimerChangePeriod(s_dma_idle_timer,
                             pdMS_TO_TICKS(SMOKE_DMA_PREOFFER_WARMUP_MS), 0);
}

static void smoke_dma_idle_worker_run(void)
{
    const size_t n = sizeof(s_dma_idle_target_ms) / sizeof(s_dma_idle_target_ms[0]);
    if (s_dma_idle_stage >= n) return;

    /* This runs on persistent pump slot 0, not Tmr Svc, so normal lifecycle
     * work and the bounded R4 heap walk are safe for the timer-service stack. */
    if (!s_gate || xSemaphoreTake(s_gate, pdMS_TO_TICKS(250)) != pdTRUE) {
        if (++s_dma_idle_retry <= SMOKE_DMA_IDLE_MAX_RETRIES && s_dma_idle_timer) {
            (void)xTimerChangePeriod(s_dma_idle_timer,
                                     pdMS_TO_TICKS(SMOKE_DMA_IDLE_RETRY_MS), 0);
        } else {
            SMKW("[DMA-BLACKBOX-IDLE-R7] gate busy; abort stage=%u",
                 (unsigned)s_dma_idle_stage);
            s_dma_idle_stage = (uint8_t)n;
        }
        return;
    }
    s_dma_idle_retry = 0U;

    const uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);
    const uint32_t elapsed_ms = now_ms - s_dma_idle_t0_ms;
    const uint8_t idx = s_dma_idle_stage;
    char tag[12];
    snprintf(tag, sizeof(tag), "IDLE%u", (unsigned)(idx + 1U));

    if (smoke_session_count_locked() != 0U || camera_h264_stream_is_active()) {
        smoke_dma_idle_record_locked("IDLE_ABORT", s_dma_idle_target_ms[idx], elapsed_ms);
        s_dma_idle_stage = (uint8_t)n;
        __atomic_store_n(&s_dma_timer_mode, SMOKE_DMA_TIMER_MODE_NONE, __ATOMIC_RELEASE);
        xSemaphoreGive(s_gate);
        return;
    }

    smoke_dma_idle_record_locked(tag, s_dma_idle_target_ms[idx], elapsed_ms);
    s_dma_idle_stage = (uint8_t)(idx + 1U);
    const bool more = s_dma_idle_stage < n;

    /* A completed 5s full-idle recovery is a new clean baseline. */
    if (!more) {
        const size_t recovered = heap_caps_get_largest_free_block(
            MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
        smoke_dma_heapmap_dump_locked("IDLE5S", s_dma_bb_floor_largest, recovered);
        if (recovered > s_dma_bb_floor_largest + SMOKE_DMA_DROP_MIN_BYTES) {
            const size_t old_floor = s_dma_bb_floor_largest;
            smoke_sess_t *sess = s_active ? s_active : s_secondary;
            camera_h264_profile_t profile = sess ? sess->profile : camera_h264_stream_get_profile();
            smoke_fr_record_locked(FR_EVENT_DMA_RECOVER, sess, profile,
                                   sess ? sess->client_token : NULL, "IDLE_REARM",
                                   (int32_t)old_floor, (int32_t)recovered);
            SMKW("[DMA-BLACKBOX-R2] IDLE_REARM confirmed_floor %u -> %u bytes",
                 (unsigned)old_floor, (unsigned)recovered);
            s_dma_bb_floor_largest = recovered;
        }
        s_dma_bb_candidate_largest = 0U;
        s_dma_bb_candidate_since_ms = 0U;
        s_dma_bb_candidate_samples = 0U;
        s_dma_bb_candidate_edge_logged = false;
        s_dma_heapmap_edge_dumped = false;
        __atomic_store_n(&s_dma_timer_mode, SMOKE_DMA_TIMER_MODE_NONE, __ATOMIC_RELEASE);
    }
    xSemaphoreGive(s_gate);

    if (more && s_dma_idle_timer) {
        (void)xTimerChangePeriod(s_dma_idle_timer,
                                 pdMS_TO_TICKS(s_dma_idle_delta_ms[s_dma_idle_stage]), 0);
    }
}

static void smoke_dma_idle_timer_cb(TimerHandle_t timer)
{
    (void)timer;
    const uint8_t mode = __atomic_load_n(&s_dma_timer_mode, __ATOMIC_ACQUIRE);
    if (mode == SMOKE_DMA_TIMER_MODE_NONE) return;
    if (mode == SMOKE_DMA_TIMER_MODE_IDLE) {
        const size_t n = sizeof(s_dma_idle_target_ms) / sizeof(s_dma_idle_target_ms[0]);
        if (s_dma_idle_stage >= n) return;
    }

    /* Tmr Svc callback must stay tiny: no gate, heap walk, recorder, snprintf
     * or ESP_LOG. Wake the already-existing static pump worker instead. */
    if (__atomic_exchange_n(&s_dma_idle_work_pending, true, __ATOMIC_ACQ_REL)) {
        return;
    }

    if (s_pump_workers_ready && s_pump_task[0] != NULL) {
        xTaskNotifyGive(s_pump_task[0]);
    } else {
        __atomic_store_n(&s_dma_idle_work_pending, false, __ATOMIC_RELEASE);
    }
}

/* s_gate held; called after final session queues/semaphores/SDP are freed. */
static void smoke_dma_idle_probe_arm_locked(uint32_t departed_sid)
{
    /* [DMA-BLACKBOX-R7A-SCREEN-TRANSITION-SAFE-20260919]
     * Keep R8 pre-offer tracing, but do NOT revive the delayed full-idle
     * IDLE1..IDLE4 / IDLE5S series that caused Tmr Svc crashes in hardware.
     * Teardown records only synchronous IDLE0.
     */
    __atomic_store_n(&s_dma_timer_mode, SMOKE_DMA_TIMER_MODE_NONE, __ATOMIC_RELEASE);
    if (s_dma_idle_timer) {
        (void)xTimerStop(s_dma_idle_timer, 0);
    }
    s_dma_idle_departed_sid = departed_sid;
    s_dma_idle_t0_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);
    s_dma_idle_stage = (uint8_t)(sizeof(s_dma_idle_target_ms) / sizeof(s_dma_idle_target_ms[0]));
    s_dma_idle_retry = 0U;
    __atomic_store_n(&s_dma_idle_work_pending, false, __ATOMIC_RELEASE);
    s_dma_idle_base_largest = heap_caps_get_largest_free_block(
        MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
    s_dma_idle_prev_largest = s_dma_idle_base_largest;
    smoke_dma_idle_record_locked("IDLE0", 0U, 0U);

    /* [DMA-BB-EXPECTED-R3-20261010] final viewer gone: restore pre-hold floor first. */
    smoke_dma_bb_expected_hold_end_locked("IDLE0", s_dma_idle_t0_ms);

    /* [DMA-BLACKBOX-R8-IDLE0-REARM-20261004]
     * Logging/recorder only. After the final session is fully torn down,
     * IDLE0 is a safe synchronous point. If INTERNAL|DMA largest has
     * recovered above the detector's previously confirmed floor (for example
     * 16384 -> 31744), re-arm the R2/R5 edge/persist detector at that recovered
     * floor so a later 31744 -> 16384 event can be captured again.
     *
     * No allocation/free, no delayed timer series, no heap walk, and no
     * camera/WebRTC/audio/DMA policy change is performed here.
     */
    if (s_dma_idle_base_largest > s_dma_bb_floor_largest) {
        const size_t old_floor = s_dma_bb_floor_largest;
        s_dma_bb_floor_largest = s_dma_idle_base_largest;
        s_dma_bb_candidate_largest = 0U;
        s_dma_bb_candidate_since_ms = 0U;
        s_dma_bb_candidate_samples = 0U;
        s_dma_bb_candidate_edge_logged = false;
        s_dma_heapmap_edge_dumped = false;
        SMKW("[DMA-BLACKBOX-R8-IDLE0-REARM] floor %u -> %u bytes; next >=2KB drop armed",
             (unsigned)old_floor, (unsigned)s_dma_bb_floor_largest);
    }
    SMKW("[DMA-BLACKBOX-R7A-SCREEN-SAFE] departed_sid=%u delayed IDLE1..4/IDLE5S walk disabled",
         (unsigned)departed_sid);
}


static esp_err_t smoke_dma_idle_timer_init(void)
{
    if (s_dma_idle_timer) return ESP_OK;
    s_dma_idle_timer = xTimerCreateStatic("dma_idle_bb",
                                          pdMS_TO_TICKS(s_dma_idle_delta_ms[0]),
                                          pdFALSE, NULL,
                                          smoke_dma_idle_timer_cb,
                                          &s_dma_idle_timer_storage);
    if (!s_dma_idle_timer) return ESP_ERR_NO_MEM;
    SMKW("[DMA-BLACKBOX-R7-IDLE-DEFER] static timer callback=notify-only; idle work uses persistent pump0 stack");
    return ESP_OK;
}

static void smoke_fr_peer(flight_recorder_event_t event,
                          smoke_sess_t *sess,
                          const char *tag,
                          int32_t arg0, int32_t arg1,
                          bool trigger)
{
    if (!sess) return;
    flight_recorder_ctx_t ctx = {0};
    ctx.sid = sess->session_id;
    ctx.peer_count = 0xFFU;       /* callback intentionally avoids s_gate-owned table */
    ctx.connected_count = sess->peer_connected ? 1U : 0U;
    ctx.fanout_count = (uint8_t)smoke_fanout_count();
    ctx.requested_profile = (uint8_t)sess->profile;
    ctx.current_profile = (uint8_t)sess->profile;
    ctx.session_running = sess->running;
    ctx.session_connected = sess->peer_connected;
    ctx.session_teardown = __atomic_load_n(&sess->teardown_started, __ATOMIC_ACQUIRE);
    ctx.camera_active = camera_h264_stream_is_active();
    ctx.audio_active = camera_webrtc_audio_is_active();
    ctx.client_token = sess->client_token;
    ctx.arg0 = arg0;
    ctx.arg1 = arg1;
    if (trigger) flight_recorder_trigger(event, &ctx, tag);
    else flight_recorder_record(event, &ctx, tag);
}

static bool smoke_fanout_attach(smoke_sess_t *sess)
{
    if (!sess || !s_fanout_lock) return false;
    if (__atomic_load_n(&sess->teardown_started, __ATOMIC_ACQUIRE) || !sess->running) return false;
    if (xSemaphoreTake(s_fanout_lock, pdMS_TO_TICKS(50)) != pdTRUE) return false;
    /* Re-check under the fanout lock. This closes the race where teardown sets
     * the lifecycle fence while a late DTLS CONNECTED callback is waiting here. */
    if (__atomic_load_n(&sess->teardown_started, __ATOMIC_ACQUIRE) || !sess->running) {
        xSemaphoreGive(s_fanout_lock);
        return false;
    }
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

/* ????????????? [ASYNC-H264-TX] PSRAM SPSC queue ????????????? */

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
    if (!s || !data || len == 0 || !s->running || !s->peer_connected || !s->tx_lock ||
        __atomic_load_n(&s->teardown_started, __ATOMIC_ACQUIRE)) {
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
        if (sess && sess->running && sess->peer_connected &&
            !__atomic_load_n(&sess->teardown_started, __ATOMIC_ACQUIRE)) {
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
        !s->running || !s->peer_connected || !s->audio_lock ||
        __atomic_load_n(&s->teardown_started, __ATOMIC_ACQUIRE)) {
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
        if (sess && sess->running && sess->peer_connected &&
            !__atomic_load_n(&sess->teardown_started, __ATOMIC_ACQUIRE)) {
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

static void smoke_audio_start_after_video_schedule(void);

/* peer_mtx is already held. Keep audio_lock across this small SRTP send so
 * the producer cannot overwrite the tail packet while esp_peer uses it. */
static bool smoke_audio_send_one_locked(smoke_sess_t *s)
{
    if (!s || !s->audio_lock || !s->peer || !s->running || !s->peer_connected ||
        __atomic_load_n(&s->teardown_started, __ATOMIC_ACQUIRE)) {
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
    if (!s || !s->tx_lock || !s->peer || !s->running || !s->peer_connected ||
        __atomic_load_n(&s->teardown_started, __ATOMIC_ACQUIRE)) return false;

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

#if SMOKE_REVERSE_AUDIO_RX_PHASE1
/* [REV-AUDIO-RX-AB-PHASE1-20260917] Receive-only DIAGNOSTIC callbacks.
 * CONTEXT: both run from esp_peer_main_loop() while the persistent pump holds
 *          this session's peer_mtx. They therefore MUST NOT block, MUST NOT take
 *          peer_mtx, MUST NOT allocate, MUST NOT decode Opus, and MUST NOT touch
 *          the sound task / I2S. Phase 1 records counters + first-packet metadata
 *          and forwards each bounded payload to the preallocated Phase-2 queue.
 *          Packet size remains transport telemetry only; no speech classifier.
 *          Decode/I2S never execute in this callback. */
static int smoke_on_audio_info(esp_peer_audio_stream_info_t *info, void *ctx)
{
    smoke_sess_t *s = (smoke_sess_t *)ctx;
    if (!s || !info) return 0;
    if (__atomic_load_n(&s->teardown_started, __ATOMIC_ACQUIRE)) return 0;

    __atomic_add_fetch(&s->rx_audio_info_count, 1U, __ATOMIC_RELAXED);
    s->rx_audio_first_sr = info->sample_rate;
    s->rx_audio_first_ch = info->channel;
    if (!s->rx_info_logged) {
        s->rx_info_logged = true;
        SMK_WEBRTC_DIAG("[RX-AB] RXA3 first_on_audio_info sid=%u codec=%d sr=%u ch=%u free=%u largest=%u",
             (unsigned)s->session_id, (int)info->codec,
             (unsigned)info->sample_rate, (unsigned)info->channel,
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA),
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA));
    }
    return 0;
}

static int smoke_on_audio_data(esp_peer_audio_frame_t *frame, void *ctx)
{
    smoke_sess_t *s = (smoke_sess_t *)ctx;
    if (!s || !frame) return 0;
    if (__atomic_load_n(&s->teardown_started, __ATOMIC_ACQUIRE)) return 0;

    const uint32_t sz = (frame->size > 0) ? (uint32_t)frame->size : 0U;
    const uint32_t n  = __atomic_add_fetch(&s->rx_audio_pkt_count, 1U, __ATOMIC_RELAXED);
    __atomic_add_fetch(&s->rx_audio_bytes, sz, __ATOMIC_RELAXED);
    if (sz > s->rx_audio_max_size) s->rx_audio_max_size = sz;
    if (s->rx_audio_min_size == 0U || sz < s->rx_audio_min_size) s->rx_audio_min_size = sz;

    if (!s->rx_first_data_logged) {
        s->rx_first_data_logged = true;
        s->rx_audio_first_size = sz;
        SMK_WEBRTC_DIAG("[RX-AB] RXA4 first_on_audio_data sid=%u size=%u pts=%u free=%u largest=%u",
             (unsigned)s->session_id, (unsigned)sz, (unsigned)frame->pts,
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA),
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA));
    }
    /* One-shot steady summary near ~5s of 20ms audio (50 pkt/s * 5). No per-packet log. */
    if (!s->rx_steady_logged && n >= 250U) {
        s->rx_steady_logged = true;
        SMK_WEBRTC_DIAG("[RX-AB] RXA5 steady sid=%u pkts=%u bytes=%u min=%u max=%u sr=%u ch=%u free=%u largest=%u",
             (unsigned)s->session_id, (unsigned)n,
             (unsigned)s->rx_audio_bytes, (unsigned)s->rx_audio_min_size,
             (unsigned)s->rx_audio_max_size,
             (unsigned)s->rx_audio_first_sr, (unsigned)s->rx_audio_first_ch,
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA),
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA));
    }
    /* [REV-AUDIO-SPEAKER-PHASE2-R1-20260917]
     * Callback remains bounded/non-blocking: sound_talkback_submit_opus() only
     * claims a preallocated PSRAM slot + memcpy. Decode and I2S run elsewhere. */
    const uint32_t sr = s->rx_audio_first_sr ? s->rx_audio_first_sr : CAM_WEBRTC_AUDIO_SAMPLE_RATE;
    const uint8_t ch = s->rx_audio_first_ch ? s->rx_audio_first_ch : CAM_WEBRTC_AUDIO_CHANNELS;
    (void)sound_talkback_submit_opus(s->session_id, frame->data, (size_t)sz,
                                     frame->pts, sr, ch);
    return 0;
}
#endif /* SMOKE_REVERSE_AUDIO_RX_PHASE1 */

static int smoke_on_state(esp_peer_state_t st, void *ctx)
{
    smoke_sess_t *sess = (smoke_sess_t *)ctx;
    if (!sess) return 0;

    /* [LATE-CONNECTED-UAF-GUARD-R1-20260915]
     * Teardown can overlap the tail of esp_peer_main_loop(). In the observed
     * crash, CLIENT-ORPHAN-RECLAIM entered teardown while DTLS was CONNECTING,
     * then a late CONNECTED callback re-attached the soon-to-be-freed session
     * and restarted camera/audio. The audio producer later dereferenced a
     * deleted audio_lock (0xA5 poison). Once teardown starts, lifecycle is
     * monotonic: ignore every subsequent peer state callback for this session. */
    if (__atomic_load_n(&sess->teardown_started, __ATOMIC_ACQUIRE)) {
        SMKW("[LATE-CONNECTED-UAF-GUARD-R1] sid=%u ignore peer state=%d teardown=1",
             (unsigned)sess->session_id, (int)st);
        smoke_fr_peer(FR_EVENT_LATE_CALLBACK, sess, "LATE_CB", (int32_t)st, 0, true);
        return 0;
    }

    switch (st) {
    case ESP_PEER_STATE_CANDIDATE_GATHERING:
        SMK("sid=%u ICE state=gathering", (unsigned)sess->session_id);
        smoke_fr_peer(FR_EVENT_ICE_GATHERING, sess, "ICE_GATH", 0, 0, false);
        break;
    case ESP_PEER_STATE_PAIRING:
        SMK("sid=%u ICE state=pairing", (unsigned)sess->session_id);
        smoke_fr_peer(FR_EVENT_ICE_PAIRING, sess, "ICE_PAIR", 0, 0, false);
        __atomic_store_n(&sess->lan_mdns_pairing_open, true, __ATOMIC_RELEASE);
        if (__atomic_load_n(&sess->lan_mdns_candidate_ready, __ATOMIC_ACQUIRE) &&
            !__atomic_load_n(&sess->lan_mdns_inject_attempted, __ATOMIC_ACQUIRE)) {
            __atomic_store_n(&sess->lan_mdns_inject_pending, true, __ATOMIC_RELEASE);
            SMK_WEBRTC_DIAG("[LAN-DIRECT-ICE-ASYNC-R4] sid=%u PAIRING arm one-shot %s:%u free=%u largest=%u",
                 (unsigned)sess->session_id, sess->lan_mdns_resolved_ip,
                 (unsigned)sess->lan_mdns_candidate_port,
                 (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA),
                 (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA));
        }
        break;
    case ESP_PEER_STATE_PAIRED: {
        __atomic_store_n(&sess->lan_mdns_pairing_open, false, __ATOMIC_RELEASE);
        __atomic_store_n(&sess->lan_mdns_inject_pending, false, __ATOMIC_RELEASE);
        SMK("sid=%u ICE state=paired", (unsigned)sess->session_id);
        smoke_fr_peer(FR_EVENT_ICE_PAIRED, sess, "ICE_PAIRED", 0, 0, false);
        esp_peer_addr_t addr;
        if (sess->peer && esp_peer_get_paired_addr(sess->peer, &addr) == ESP_PEER_ERR_NONE) {
            SMK("sid=%u remote=%u.%u.%u.%u:%u",
                (unsigned)sess->session_id,
                addr.ipv4[0], addr.ipv4[1], addr.ipv4[2], addr.ipv4[3], addr.port);
            /* [LAN-STUN-TRUST-R5F2-20260927]
             * Repeated STUN-source evidence is only a discovery hint.  Promote
             * it to the 30s SRC_STUN cache after esp_peer has actually selected
             * that exact IPv4:port pair (therefore after normal ICE validation). */
            if (__atomic_exchange_n(&sess->lan_stun_source_confirmed, false,
                                    __ATOMIC_ACQ_REL)) {
                char paired_ip[16];
                snprintf(paired_ip, sizeof(paired_ip), "%u.%u.%u.%u",
                         (unsigned)addr.ipv4[0], (unsigned)addr.ipv4[1],
                         (unsigned)addr.ipv4[2], (unsigned)addr.ipv4[3]);
                const bool exact_stun_pair =
                    addr.family == AF_INET &&
                    sess->lan_stun_confirmed_ip[0] != '\0' &&
                    strcmp(paired_ip, sess->lan_stun_confirmed_ip) == 0 &&
                    addr.port == sess->lan_stun_confirmed_port;
                if (exact_stun_pair && sess->lan_mdns_name[0]) {
                    smoke_lan_mdns_cache_store(sess->lan_mdns_name,
                                               sess->lan_stun_confirmed_ip,
                                               SMOKE_LAN_CACHE_SRC_STUN);
                    SMKW("[LAN-STUN-TRUST-R5F2] sid=%u PAIRED-VERIFIED cache-store name=%s endpoint=%s:%u",
                         (unsigned)sess->session_id, sess->lan_mdns_name,
                         sess->lan_stun_confirmed_ip,
                         (unsigned)sess->lan_stun_confirmed_port);
                } else {
                    SMKW("[LAN-STUN-TRUST-R5F2] sid=%u NO-CACHE paired=%s:%u learned=%s:%u family=%d",
                         (unsigned)sess->session_id, paired_ip, (unsigned)addr.port,
                         sess->lan_stun_confirmed_ip[0] ? sess->lan_stun_confirmed_ip : "-",
                         (unsigned)sess->lan_stun_confirmed_port, (int)addr.family);
                }
            }
            if (__atomic_load_n(&sess->lan_mdns_inject_attempted, __ATOMIC_ACQUIRE)) {
                SMK_WEBRTC_DIAG("[LAN-DIRECT-ICE-ASYNC-R4] sid=%u PAIRED remote=%u.%u.%u.%u:%u inject_r=%d free=%u largest=%u",
                     (unsigned)sess->session_id,
                     addr.ipv4[0], addr.ipv4[1], addr.ipv4[2], addr.ipv4[3], addr.port,
                     sess->lan_mdns_inject_result,
                     (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA),
                     (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA));
            }
        }
        break;
    }
    case ESP_PEER_STATE_CONNECTING: {
        __atomic_store_n(&sess->lan_mdns_pairing_open, false, __ATOMIC_RELEASE);
        __atomic_store_n(&sess->lan_mdns_inject_pending, false, __ATOMIC_RELEASE);
        int64_t now_ms = esp_timer_get_time() / 1000;
        if (!sess->dtls_connecting) {
            sess->dtls_connecting = true;
            sess->dtls_start_ms = now_ms;
            SMK("sid=%u ICE state=connecting (DTLS) watchdog=%ums",
                (unsigned)sess->session_id, (unsigned)SMOKE_DTLS_CONNECT_TIMEOUT_MS);
            smoke_fr_peer(FR_EVENT_DTLS_CONNECTING, sess, "DTLS_CONN", 0, 0, false);
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
        smoke_fr_peer(FR_EVENT_CONNECTED, sess, "CONNECTED", (int32_t)dtls_ms, (int32_t)viewers, false);
        SMK_DMA_DIAG("[DMA-DIAG D0 after_dtls_srtp_ready] free=%u largest=%u",
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA),
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA));
        if (__atomic_load_n(&sess->lan_mdns_inject_attempted, __ATOMIC_ACQUIRE)) {
            SMK_WEBRTC_DIAG("[LAN-DIRECT-ICE-ASYNC-R4] sid=%u DTLS ready inject_r=%d free=%u largest=%u",
                 (unsigned)sess->session_id, sess->lan_mdns_inject_result,
                 (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA),
                 (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA));
        }

        if (viewers == 1U && !camera_h264_stream_is_active()) {
            const size_t cam_start_free_before = heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
            const size_t cam_start_largest_before = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
            SMK_DMA_DIAG("[DMA-DIAG CAM-START-PROBE] sid=%u C0 before_camera_start free=%u largest=%u",
                 (unsigned)sess->session_id,
                 (unsigned)cam_start_free_before, (unsigned)cam_start_largest_before);

            /* [CAM-AE-RESUME-FASTLOCK-R50F1-20260926]
             * Fresh/same-profile WebRTC start: restore the last RAM-only physical
             * AE if available, otherwise cold-seed and let bounded FAST-LOCK close
             * the initial error. This also fixes 720 reconnects where the sensor
             * restarted at Gain index 1 while geometry stayed 1288x728. */
            esp_video_isp_pipeline_ae_warm_set_restore_enabled(true);
            esp_video_isp_pipeline_ae_fastlock_prepare_next_seed(true);
            esp_err_t cr = camera_h264_stream_start(smoke_fanout_h264, NULL);
            if (cr != ESP_OK) {
                esp_video_isp_pipeline_ae_fastlock_prepare_next_seed(false);
            }

            const size_t cam_start_free_after = heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
            const size_t cam_start_largest_after = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
            SMK_DMA_DIAG("[DMA-DIAG CAM-START-PROBE] sid=%u C1 after_camera_start ret=%s free=%u largest=%u delta_free=%d delta_largest=%d",
                 (unsigned)sess->session_id, esp_err_to_name(cr),
                 (unsigned)cam_start_free_after, (unsigned)cam_start_largest_after,
                 (int)cam_start_free_after - (int)cam_start_free_before,
                 (int)cam_start_largest_after - (int)cam_start_largest_before);

            if (cr == ESP_OK) {
                sess->cam_start_probe_base_free = cam_start_free_before;
                sess->cam_start_probe_base_largest = cam_start_largest_before;
                sess->cam_start_probe_prev_largest = cam_start_largest_after;
                sess->cam_start_probe_t0_ms = esp_timer_get_time() / 1000;
                sess->cam_start_probe_next = 0U;
                sess->cam_start_probe_armed = true;
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
            smoke_audio_start_after_video_schedule();
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
        smoke_fr_peer(FR_EVENT_CONNECT_FAILED, sess, "CONN_FAIL", 0, 0, false);
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
        smoke_fr_peer(FR_EVENT_DISCONNECTED, sess, "DISCONN", 0, 0, false);
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
        smoke_fr_peer(FR_EVENT_CLOSED, sess, "CLOSED", 0, 0, false);
        if (sess->running) {
            sess->auto_cleanup_requested = true;
            sess->running = false;
        }
        break;
#if SMOKE_REVERSE_AUDIO_RX_PHASE1
    case ESP_PEER_STATE_REMOTE_AUDIO_TRACK_ADDED:
        if (!sess->rx_track_probe_logged) {
            sess->rx_track_probe_logged = true;
            SMK_WEBRTC_DIAG("[RX-AB] RXA2 remote_audio_track_added sid=%u free=%u largest=%u",
                 (unsigned)sess->session_id,
                 (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA),
                 (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA));
        }
        break;
#endif
    default:
        break;
    }
    return 0;
}

static int smoke_on_msg(esp_peer_msg_t *info, void *ctx)
{
    smoke_sess_t *s = (smoke_sess_t *)ctx;
    if (!s || __atomic_load_n(&s->teardown_started, __ATOMIC_ACQUIRE)) return 0;
    if (!info || !info->data || info->size <= 0) return 0;

    if (info->type == ESP_PEER_MSG_TYPE_SDP) {
        char *sdp = (char *)smoke_sig_psram_alloc((size_t)info->size + 1U);
        if (!sdp) {
            SMKE("[DMA-BOUNDARY-R17F9] OOM capturing local SDP in PSRAM size=%u; internal fallback forbidden",
                 (unsigned)info->size);
            return 0;
        }
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
        /* info->data ??"candidate:..." ??SDP ?인? "a=candidate:..." */
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

/* ????????????? pump ?스??(Core0) ????????????? */

static bool smoke_teardown_locked(smoke_sess_t *sess); /* forward: s_gate held */

/* [PERSISTENT-PUMP-PER-SLOT-R1-20260915]
 * After a persistent pump unwinds from esp_peer_main_loop and signals done,
 * the same worker may perform the safe session cleanup by session_id.  No
 * transient cleanup task/stack is created, avoiding another heap fragment. */
static void smoke_auto_cleanup_sid(uint32_t sid)
{
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
}

static void smoke_pump(void *arg)
{
    const uint32_t slot = (uint32_t)(uintptr_t)arg;
    if (slot >= SMOKE_MAX_SESSIONS) {
        SMKE("[PERSIST-PUMP] invalid worker slot=%u", (unsigned)slot);
        vTaskSuspend(NULL);
        return;
    }

    SMK("[PERSIST-PUMP] worker ready slot=%u core=%d stack=%u",
        (unsigned)slot, xPortGetCoreID(), (unsigned)SMOKE_PUMP_STACK);

    for (;;) {
        (void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        bool did_dma_timer_work = false;
        if (slot == 0U &&
            __atomic_exchange_n(&s_dma_idle_work_pending, false, __ATOMIC_ACQ_REL)) {
            const uint8_t mode =
                __atomic_load_n(&s_dma_timer_mode, __ATOMIC_ACQUIRE);
            if (mode == SMOKE_DMA_TIMER_MODE_PREOFFER) {
                smoke_dma_preoffer_worker_run();
            } else if (mode == SMOKE_DMA_TIMER_MODE_IDLE) {
                smoke_dma_idle_worker_run();
            }
            did_dma_timer_work = true;
        }

        smoke_sess_t *s = (smoke_sess_t *)s_pump_sess[slot];
        if (!s) {
            if (!did_dma_timer_work) {
                SMKW("[PERSIST-PUMP] slot=%u woke without session", (unsigned)slot);
            }
            continue;
        }

        const uint32_t sid = s->session_id;
        SMK("sid=%u pump task started core=%d stack=%d persistent_slot=%u",
            (unsigned)sid, xPortGetCoreID(), SMOKE_PUMP_STACK, (unsigned)slot);

        while (s->running) {
            bool did_tx = false;
            smoke_lan_stun_poll_result(s);
            smoke_lan_mdns_poll_result(s);
            if (xSemaphoreTake(s->peer_mtx, pdMS_TO_TICKS(100)) == pdTRUE) {
                /* [LAN-DIRECT-ICE-ASYNC-R4-20260926]
                 * Try pending async candidate immediately if PAIRING was already
                 * open, then run the agent, then retry once for a PAIRING callback
                 * or resolver result published during main_loop. */
                smoke_lan_mdns_inject_locked(s);
                if (s->peer) esp_peer_main_loop(s->peer);
                smoke_lan_stun_poll_result(s);
                smoke_lan_mdns_poll_result(s);
                smoke_lan_mdns_inject_locked(s);

                /* Send one previously-copied H.264 AU after main_loop. The camera
                 * is free to queue the next encoded frame while this SRTP/UDP send
                 * is in progress. */
                if (s->peer && s->running && s->peer_connected) {
                    /* [CAM-AUDIO-PUMP-JITTER-FIX-V1-20260902]
                     * Audio is 20ms/50fps while H264 send can occupy peer_mtx for
                     * tens of ms. Preserve audio continuity by draining a bounded
                     * backlog first, then send exactly one video AU. */
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

            /* [CAM-D2-D3-RECOVERY-PROBE-R1-20260915] Read-only sampling from
             * the already-running persistent pump; no new task/timer/heap alloc. */
            smoke_cam_start_probe_tick(s, now_ms);
            smoke_idle_dma_probe_tick(s, (uint32_t)now_ms);

            /* [DTLS-FAST-RECOVERY-CD78] timeout only after unwinding peer_mtx. */
            /* Browser liveness is independent of ICE/DTLS CONNECTED. A stale
             * page can leave media flowing forever, so the pump enforces the
             * heartbeat deadline even when no HTTP request arrives. */
            uint32_t hb_age_ms = 0U;
            if (smoke_viewer_hb_expired(s, (uint32_t)now_ms, &hb_age_ms)) {
                SMKW("[CAM-VIEWER-HB-SIGNED-AGE-R1] sid=%u pump timeout age=%ums -> safe cleanup",
                     (unsigned)sid, (unsigned)hb_age_ms);
                __atomic_store_n(&s->viewer_hb_visible, false, __ATOMIC_RELEASE);
                s->auto_cleanup_requested = true;
                s->running = false;
            }

            if (s->running && !s->peer_connected && s->dtls_connecting &&
                s->dtls_start_ms > 0 &&
                now_ms - s->dtls_start_ms >= (int64_t)SMOKE_DTLS_CONNECT_TIMEOUT_MS) {
                int64_t elapsed = now_ms - s->dtls_start_ms;
                SMKE("sid=%u [DTLS-WATCHDOG] timeout after %lldms -> safe auto cleanup",
                     (unsigned)sid, (long long)elapsed);
                s->dtls_connecting = false;
                s->dtls_start_ms = 0;
                s->auto_cleanup_requested = true;
                s->running = false;
            }

            vTaskDelay(pdMS_TO_TICKS((did_tx || smoke_tx_count(s) > 0 || smoke_audio_count(s) > 0) ? 1 : 10));
        }

        const bool need_auto_cleanup = s->auto_cleanup_requested;
        const UBaseType_t hwm = uxTaskGetStackHighWaterMark(NULL);
        SMK_DMA_DIAG("[DMA-DIAG PUMP-PROBE] sid=%u pump_stack_hwm configured=%u high_watermark=%u free=%u largest=%u persistent_slot=%u",
             (unsigned)sid,
             (unsigned)SMOKE_PUMP_STACK,
             (unsigned)hwm,
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA),
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA),
             (unsigned)slot);

        /* Publish worker-idle before giving done.  start/teardown are serialized by
         * s_gate, so a new session cannot reuse this worker until teardown has
         * consumed done.  No s dereference is allowed after xSemaphoreGive(). */
        s_pump_sess[slot] = NULL;
        s->pump = NULL;
        s->pump_slot = SMOKE_PUMP_SLOT_NONE;
        xSemaphoreGive(s->done);

        if (need_auto_cleanup) {
            SMK("sid=%u peer ended -> safe auto cleanup via persistent slot=%u",
                (unsigned)sid, (unsigned)slot);
            smoke_auto_cleanup_sid(sid);
        }
    }
}

static esp_err_t smoke_pump_workers_init(void)
{
    if (s_pump_workers_ready) return ESP_OK;

    const size_t free_before = heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
    const size_t largest_before = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
    static const char *const names[SMOKE_MAX_SESSIONS] = {"smoke_pump0", "smoke_pump1"};

    for (uint32_t i = 0; i < SMOKE_MAX_SESSIONS; ++i) {
        s_pump_sess[i] = NULL;
        s_pump_task[i] = xTaskCreateStaticPinnedToCore(
            smoke_pump, names[i], SMOKE_PUMP_STACK, (void *)(uintptr_t)i,
            SMOKE_PUMP_PRIO, s_pump_stack[i], &s_pump_tcb[i], SMOKE_PUMP_CORE);
        if (!s_pump_task[i]) {
            SMKE("[PERSIST-PUMP] worker create failed slot=%u", (unsigned)i);
            return ESP_ERR_NO_MEM;
        }
    }

    s_pump_workers_ready = true;
    SMKW("[PERSIST-PUMP] workers ready count=%u stack_each=%u dma=%u/%u -> %u/%u (static stack/TCB; no heap stack alloc)",
         (unsigned)SMOKE_MAX_SESSIONS, (unsigned)SMOKE_PUMP_STACK,
         (unsigned)free_before, (unsigned)largest_before,
         (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA),
         (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA));
    return ESP_OK;
}

/* s_gate must be held. */
static int smoke_pump_slot_acquire(smoke_sess_t *sess)
{
    if (!sess || !s_pump_workers_ready) return -1;
    for (uint32_t i = 0; i < SMOKE_MAX_SESSIONS; ++i) {
        if (s_pump_task[i] && s_pump_sess[i] == NULL) {
            s_pump_sess[i] = sess;
            sess->pump = s_pump_task[i];
            sess->pump_slot = (uint8_t)i;
            return (int)i;
        }
    }
    return -1;
}


static void smoke_audio_start_after_video_schedule(void);

/* [R21 OWNER-ONLY PROFILE SWITCH — CURRENT STABLE]
 * Before restarting the shared camera/H264 producer, every OTHER viewer must
 * finish exact-SID teardown. The requesting owner's peer/ICE/DTLS/SRTP/audio,
 * persistent pump, PSRAM TX slots, and persistent H.264 memory stay resident.
 *
 * s_gate MUST be held. Apply is allowed only when exactly one allocated and
 * connected session remains and it is the lease owner. Only TX ring metadata
 * is flushed; PSRAM slot payloads remain allocated. Historical R21 log names
 * are retained for continuity.
 */
static bool smoke_tx_video_flush_resident_locked(smoke_sess_t *sess)
{
    if (!sess || !sess->tx_lock || !sess->peer_mtx) return false;

    if (xSemaphoreTake(sess->peer_mtx, pdMS_TO_TICKS(1500)) != pdTRUE) {
        SMKW("[DMA-BOUNDARY-R21] sid=%u queue flush peer mutex busy",
             (unsigned)sess->session_id);
        return false;
    }

    bool ok = false;
    if (xSemaphoreTake(sess->tx_lock, pdMS_TO_TICKS(250)) == pdTRUE) {
        const uint32_t dropped = sess->tx_count;
        for (uint32_t i = 0; i < SMOKE_TX_QUEUE_DEPTH; ++i) {
            sess->tx_slots[i].len = 0;
            sess->tx_slots[i].pts_ms = 0;
        }
        sess->tx_head = 0U;
        sess->tx_tail = 0U;
        sess->tx_count = 0U;
        xSemaphoreGive(sess->tx_lock);
        SMKW("[DMA-BOUNDARY-R21] sid=%u owner H264 queue flushed=%u slots_kept=%u",
             (unsigned)sess->session_id, (unsigned)dropped,
             (unsigned)SMOKE_TX_QUEUE_DEPTH);
        ok = true;
    } else {
        SMKW("[DMA-BOUNDARY-R21] sid=%u queue flush tx mutex busy",
             (unsigned)sess->session_id);
    }

    xSemaphoreGive(sess->peer_mtx);
    return ok;
}

static esp_err_t smoke_profile_switch_owner_only_locked(camera_h264_profile_t target,
                                                       bool ae_warm_restore)
{
    const uint32_t peers = smoke_session_count_locked();
    const uint32_t connected = smoke_session_connected_count_locked();
    const camera_h264_profile_t old_profile = camera_h264_stream_get_profile();
    const size_t free0 = heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
    const size_t largest0 =
        heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
    const bool safe_1080_transition =
        (old_profile == CAMERA_H264_PROFILE_720P_NATIVE &&
         target == CAMERA_H264_PROFILE_1080P);
    const bool audio_was_active =
        safe_1080_transition && camera_webrtc_audio_is_active();

    if (peers != 1U || connected != 1U) {
        SMKW("[DMA-BOUNDARY-R21] apply blocked: owner-only invariant peers=%u connected=%u target=%s",
             (unsigned)peers, (unsigned)connected,
             camera_h264_stream_profile_name(target));
        return ESP_ERR_INVALID_STATE;
    }
    if (!camera_h264_stream_is_active()) {
        SMKW("[DMA-BOUNDARY-R21] apply rejected: camera inactive target=%s",
             camera_h264_stream_profile_name(target));
        return ESP_ERR_INVALID_STATE;
    }
    if (old_profile == target) {
        if (s_active) s_active->profile = target;
        if (s_secondary) s_secondary->profile = target;
        SMKW("[DMA-BOUNDARY-R21] no-op profile already=%s peers=%u largest=%u",
             camera_h264_stream_profile_name(target), (unsigned)connected,
             (unsigned)largest0);
        return ESP_OK;
    }

    SMKW("[DMA-BOUNDARY-R21] begin owner-only %s->%s peers=%u dma=%u/%u camera=1 audio=%u refpool=%u",
         camera_h264_stream_profile_name(old_profile),
         camera_h264_stream_profile_name(target),
         (unsigned)connected, (unsigned)free0, (unsigned)largest0,
         camera_webrtc_audio_is_active() ? 1U : 0U,
         esp_h264_enc_hw_ref_pool_ready() ? 1U : 0U);

    /* [CAM-AE-PROFILE-WARM-RESTORE-R48-20260924]
     * Save REAL sensor Exposure/Gain while the OLD profile is still live.
     * A save failure must never block the proven owner-only switch: the target
     * will use its cold seed if no valid RAM slot exists. */
    /* [CAM-AE-RESUME-FASTLOCK-R50F1-20260926]
     * Normal profile switch keeps R48F1 SAVE + RESTORE, and now also arms the
     * bounded FAST-LOCK so a first/cold target reaches the neighborhood quickly.
     *
     * Auto-cal switch keeps R50's READ-ONLY target restore policy. Never save
     * outgoing 1080 while it may still be walking; leaving a live 720 view for
     * the reference still snapshots that physical 720 state before stop. */
    esp_video_isp_pipeline_ae_warm_set_restore_enabled(true);
    if (ae_warm_restore) {
        esp_err_t ae_save_r = esp_video_isp_pipeline_ae_warm_save_profile(
            old_profile == CAMERA_H264_PROFILE_720P_NATIVE);
        if (ae_save_r != ESP_OK) {
            SMKW("[CAM-AE-WARM-R48F1] pre-switch SAVE skipped/failed profile=%s err=%s; OLD slot invalidated, continuing",
                 camera_h264_stream_profile_name(old_profile),
                 esp_err_to_name(ae_save_r));
        }
    } else {
        if (old_profile == CAMERA_H264_PROFILE_720P_NATIVE &&
                target == CAMERA_H264_PROFILE_1080P) {
            esp_err_t ae_save_r =
                esp_video_isp_pipeline_ae_warm_save_profile(true);
            if (ae_save_r == ESP_OK) {
                SMKW("[CAM-AE-FASTLOCK-R50F1] captured live 720 before 1080 reference; target slots remain read-only during internal switches");
            } else {
                SMKW("[CAM-AE-FASTLOCK-R50F1] live 720 capture failed err=%s; FAST-LOCK fallback remains available",
                     esp_err_to_name(ae_save_r));
            }
        }
        SMKW("[CAM-AE-FASTLOCK-R50F1] auto-cal switch %s->%s: target warm RESTORE allowed, outgoing 1080 SAVE skipped, FAST-LOCK armed after seed",
             camera_h264_stream_profile_name(old_profile),
             camera_h264_stream_profile_name(target));
    }

    /* Stop ONLY the shared camera/H264 producer. Audio + both SEND_RECV peers
     * deliberately remain live. camera_h264_stream_stop() also waits for the
     * producer cleanup/REQBUFS(0) to finish before we reuse the PSRAM rings. */
    esp_err_t stop_r = camera_h264_stream_stop(4500);
    if (stop_r != ESP_OK) {
        SMKE("[DMA-BOUNDARY-R21] camera stop failed %s", esp_err_to_name(stop_r));
        return stop_r;
    }

    if ((s_active && !smoke_tx_video_flush_resident_locked(s_active)) ||
        (s_secondary && !smoke_tx_video_flush_resident_locked(s_secondary))) {
        SMKE("[DMA-BOUNDARY-R21] resident queue flush failed; restoring %s",
             camera_h264_stream_profile_name(old_profile));
        (void)camera_h264_stream_set_profile(old_profile);
        (void)isp_tune_apply_profile(old_profile);
        (void)camera_color_stage2_apply_profile(
            old_profile == CAMERA_H264_PROFILE_720P_NATIVE);
        esp_err_t rr = camera_h264_stream_start(smoke_fanout_h264, NULL);
        SMKW("[DMA-BOUNDARY-R21] rollback after flush failure start=%s",
             esp_err_to_name(rr));
        return ESP_ERR_TIMEOUT;
    }

    /* [CAM-1080-SAFE-SWITCH-R1-20260928]
     * 720->1080 needs three ~3.1 MB camera buffers. The failing trace showed
     * REQBUFS ENOMEM while owner peer/audio/ref-pool stayed resident, then
     * success after those resources were released by full teardown. Keep
     * peer/ICE/DTLS, but temporarily release audio + H264 ref pool before
     * the large camera REQBUFS. */
    if (safe_1080_transition) {
        const size_t ps_free0 = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
        const size_t ps_largest0 = heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM);

        if (audio_was_active) {
            camera_webrtc_audio_request_stop();
            esp_err_t ar = camera_webrtc_audio_stop(1500);
            SMKW("[CAM-1080-SAFE-SWITCH-R1] audio temporary stop=%s",
                 esp_err_to_name(ar));
            if (ar != ESP_OK) {
                SMKE("[CAM-1080-SAFE-SWITCH-R1] audio did not stop; aborting switch");
                esp_err_t rr = camera_h264_stream_start(smoke_fanout_h264, NULL);
                SMKW("[CAM-1080-SAFE-SWITCH-R1] rollback 720 camera start=%s",
                     esp_err_to_name(rr));
                return ar;
            }
        }

        bool ref_released = false;
        if (esp_h264_enc_hw_ref_pool_ready()) {
            ref_released = esp_h264_enc_hw_ref_pool_release();
        }

        const size_t ps_free1 = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
        const size_t ps_largest1 = heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM);
        SMKW("[CAM-1080-SAFE-SWITCH-R1] pre-REQBUFS release audio=%u ref=%u psram=%u/%u->%u/%u",
             audio_was_active ? 1U : 0U, ref_released ? 1U : 0U,
             (unsigned)ps_free0, (unsigned)ps_largest0,
             (unsigned)ps_free1, (unsigned)ps_largest1);
    }

    esp_err_t set_r = camera_h264_stream_set_profile(target);
    if (set_r != ESP_OK) {
        SMKE("[DMA-BOUNDARY-R21] set target profile failed %s; restoring %s",
             esp_err_to_name(set_r), camera_h264_stream_profile_name(old_profile));
        (void)camera_h264_stream_set_profile(old_profile);
        (void)isp_tune_apply_profile(old_profile);
        (void)camera_color_stage2_apply_profile(
            old_profile == CAMERA_H264_PROFILE_720P_NATIVE);
        esp_err_t rr = camera_h264_stream_start(smoke_fanout_h264, NULL);
        if (audio_was_active && rr == ESP_OK) {
            smoke_audio_start_after_video_schedule();
        }
        SMKW("[DMA-BOUNDARY-R21] rollback after set failure start=%s",
             esp_err_to_name(rr));
        return set_r;
    }

    esp_err_t tune_r = isp_tune_apply_profile(target);
    if (tune_r != ESP_OK) {
        SMKW("[DMA-BOUNDARY-R21] target tune apply=%s; continuing",
             esp_err_to_name(tune_r));
    }
    esp_err_t color_r = camera_color_stage2_apply_profile(
        target == CAMERA_H264_PROFILE_720P_NATIVE);
    if (color_r != ESP_OK) {
        SMKW("[DMA-BOUNDARY-R21] target Color2 apply=%s; continuing",
             esp_err_to_name(color_r));
    }

    /* Keep the per-peer monotonic PTS translator. A camera restart resets raw
     * PTS, and smoke_send_h264() already detects that wrap/restart and rebases
     * it onto the existing peer timeline. */
    if (s_active) s_active->profile = target;
    if (s_secondary) s_secondary->profile = target;
    smoke_adapt_reset_primary_locked();

    /* [CAM-AE-RESUME-FASTLOCK-R50F1-20260926]
     * Every deliberate target start consumes exactly one FAST-LOCK marker.
     * Warm restore still happens first. If it is already in 75..83, FAST-LOCK
     * performs zero writes; a cold/stale state gets at most two predictive jumps. */
    esp_video_isp_pipeline_ae_fastlock_prepare_next_seed(true);
    esp_err_t start_r = camera_h264_stream_start(smoke_fanout_h264, NULL);
    if (start_r != ESP_OK) {
        SMKE("[DMA-BOUNDARY-R21] target camera start failed %s; rollback %s",
             esp_err_to_name(start_r),
             camera_h264_stream_profile_name(old_profile));

        /* Target start may have failed before consuming the one-shot marker. */
        esp_video_isp_pipeline_ae_fastlock_prepare_next_seed(false);
        if (s_active) s_active->profile = old_profile;
        if (s_secondary) s_secondary->profile = old_profile;
        (void)camera_h264_stream_set_profile(old_profile);
        (void)isp_tune_apply_profile(old_profile);
        (void)camera_color_stage2_apply_profile(
            old_profile == CAMERA_H264_PROFILE_720P_NATIVE);
        esp_err_t rollback_r = camera_h264_stream_start(smoke_fanout_h264, NULL);
        if (audio_was_active && rollback_r == ESP_OK) {
            smoke_audio_start_after_video_schedule();
        }
        SMKW("[DMA-BOUNDARY-R21] rollback camera start=%s",
             esp_err_to_name(rollback_r));
        return start_r;
    }

    if (audio_was_active) {
        smoke_audio_start_after_video_schedule();
        SMKW("[CAM-1080-SAFE-SWITCH-R1] 1080 camera started; audio restart scheduled after first video");
    }

    const size_t free1 = heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
    const size_t largest1 =
        heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
    SMKW("[DMA-BOUNDARY-R21] PASS owner-only %s->%s peers=%u owner-peer/ICE/DTLS/audio/refpool=KEPT dma=%u/%u->%u/%u delta_largest=%d",
         camera_h264_stream_profile_name(old_profile),
         camera_h264_stream_profile_name(target),
         (unsigned)connected,
         (unsigned)free0, (unsigned)largest0,
         (unsigned)free1, (unsigned)largest1,
         (int)largest1 - (int)largest0);

    smoke_fr_record_locked(FR_EVENT_BROWSER_MARK, s_active, target,
                           s_active ? s_active->client_token : NULL,
                           "SW_R21_ONE", (int32_t)old_profile, (int32_t)target);
    return ESP_OK;
}

static bool smoke_teardown_locked(smoke_sess_t *sess)
{
    if (!sess) return true;

    uint32_t sid = sess->session_id;
    smoke_fr_record_locked(FR_EVENT_TEARDOWN_BEGIN, sess, sess->profile,
                           sess->client_token, "TD_BEGIN", 0, 0);
    /* [LATE-CONNECTED-UAF-GUARD-R1-20260915]
     * Publish the teardown fence before touching fanout/shared media. running=0
     * makes the persistent pump unwind after any in-flight main_loop callback;
     * callbacks themselves also check teardown_started and cannot re-attach. */
    __atomic_store_n(&sess->teardown_started, true, __ATOMIC_RELEASE);
    sound_talkback_session_closed(sid); /* non-blocking; drains queued PCM then releases owner/mic mute */
    sess->running = false;
    sess->peer_connected = false;
    sess->dtls_connecting = false;
    sess->dtls_start_ms = 0;
    smoke_fanout_detach(sess);

    uint32_t viewers_left = smoke_fanout_count();
    /* Shared-resource teardown is gated by SESSION EXISTENCE, not fanout
     * count, because a connecting peer is allocated before DTLS fanout attach.
     * The final-viewer path below may also release the idle PSRAM H.264
     * reference staging pool; despite the historical HANDOVER-DMA log name,
     * that release is NOT part of R21 owner-only in-place profile switching.
     * Keep this known-good last-session lifecycle unchanged. */
    const bool other_session =
        (s_active    && s_active    != sess) ||
        (s_secondary && s_secondary != sess);
    if (!other_session && camera_h264_stream_is_active()) {
        /* [CAM-AE-RESUME-FASTLOCK-R50F1-20260926]
         * Preserve the last physical AE across an ordinary same-profile WebRTC
         * disconnect/reconnect. Do not let an interrupted auto-cal teardown
         * overwrite its read-only reference policy. SAVE failure is non-fatal. */
        if (!s_auto_cal_active) {
            const bool profile_720 =
                camera_h264_stream_get_profile() == CAMERA_H264_PROFILE_720P_NATIVE;
            esp_err_t ae_save_r =
                esp_video_isp_pipeline_ae_warm_save_profile(profile_720);
            if (ae_save_r == ESP_OK) {
                SMKW("[CAM-AE-RESUME-R50F1] teardown SAVE profile=%s before same-profile restart",
                     profile_720 ? "720p60-native" : "1080p");
            } else {
                SMKW("[CAM-AE-RESUME-R50F1] teardown SAVE skipped/failed profile=%s err=%s; cold FAST-LOCK remains available",
                     profile_720 ? "720p60-native" : "1080p",
                     esp_err_to_name(ae_save_r));
            }
        }
        camera_h264_stream_request_stop();
        esp_err_t cam_stop = camera_h264_stream_stop(4500);
        if (cam_stop != ESP_OK) {
            SMK("sid=%u teardown deferred: camera stopping (%s)",
                (unsigned)sid, esp_err_to_name(cam_stop));
            return false;
        }
    }
    if (!other_session && esp_h264_enc_hw_ref_pool_ready()) {
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
    if (!other_session) {
        camera_webrtc_audio_request_stop();
        esp_err_t aud_stop = camera_webrtc_audio_stop(1500);
        SMK_DMA_DIAG("[DMA-DIAG D8 after_audio_stop] sid=%u result=%s free=%u largest=%u",
             (unsigned)sid, esp_err_to_name(aud_stop),
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA),
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA));
        if (aud_stop != ESP_OK) {
            SMKW("sid=%u audio stop wait=%s; task retains ownership until exit",
                 (unsigned)sid, esp_err_to_name(aud_stop));
        }
    }

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
#if SMOKE_REVERSE_AUDIO_RX_PHASE1
            SMK_WEBRTC_DIAG("[RX-AB] RXA6 before_esp_peer_close sid=%u rx_info=%u rx_pkts=%u bytes=%u min=%u max=%u free=%u largest=%u",
                 (unsigned)sid,
                 (unsigned)sess->rx_audio_info_count, (unsigned)sess->rx_audio_pkt_count,
                 (unsigned)sess->rx_audio_bytes, (unsigned)sess->rx_audio_min_size,
                 (unsigned)sess->rx_audio_max_size,
                 (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA),
                 (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA));
#endif
            int r = esp_peer_close(sess->peer);
            SMK("sid=%u esp_peer_close r=%d", (unsigned)sid, r);
            SMK_DMA_DIAG("[DMA-DIAG D9 after_esp_peer_close] sid=%u result=%d free=%u largest=%u",
                 (unsigned)sid, r,
                 (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA),
                 (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA));
#if SMOKE_REVERSE_AUDIO_RX_PHASE1
            SMK_WEBRTC_DIAG("[RX-AB] RXA7 after_esp_peer_close sid=%u free=%u largest=%u",
                 (unsigned)sid,
                 (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA),
                 (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA));
#endif
            sess->peer = NULL;
        }
        xSemaphoreGive(sess->peer_mtx);
    } else if (sess->peer) {
        SMKE("sid=%u teardown deferred: peer exists without mutex", (unsigned)sid);
        return false;
    }

    /* A callback that was already executing when teardown_started became true
     * may have completed an older attach before observing the fence. Heal that
     * edge before waiting refs/freeing locks; new attaches are now rejected. */
    smoke_fanout_detach(sess);
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

    /* Snapshot the surviving session before freeing sess.  Arm the idle probe
     * only after this session's semaphores/queues/SDP have been released, so I0
     * represents the post-teardown steady state rather than teardown itself. */
    smoke_sess_t *survivor = s_active;

    smoke_fr_record_locked(FR_EVENT_TEARDOWN_DONE, sess, sess->profile,
                           sess->client_token, "TD_DONE", (int32_t)viewers_left, 0);

    smoke_tx_pool_free(sess);
    if (sess->sdp_ready) vSemaphoreDelete(sess->sdp_ready);
    if (sess->done)      vSemaphoreDelete(sess->done);
    if (sess->peer_mtx)  vSemaphoreDelete(sess->peer_mtx);
    free(sess->local_sdp);
    free(sess);

    if (survivor && survivor->running) {
        smoke_idle_dma_probe_arm(survivor, sid);
    } else {
        /* [DMA-BLACKBOX-R2-20260919] full viewer=0 recovery */
        smoke_dma_idle_probe_arm_locked(sid);
    }

    SMK("sid=%u session torn down viewers_left=%u",
        (unsigned)sid, (unsigned)viewers_left);
    return true;
}

/* [CAM-AUDIO-AFTER-VIDEO-R1-20260913]
 * Start Opus only after the camera/H.264 path has produced its first access
 * unit. Starting I2S immediately after DTLS can temporarily split the small
 * INTERNAL|DMA heap and make the camera restart guard see largest=21KB. */
static void smoke_audio_gate_run_once(void)
{
    uint32_t waited_ms = 0;
    bool ready = false;
    camera_h264_stream_stats_t cs = {0};

    while (waited_ms <= SMOKE_AUDIO_GATE_TIMEOUT_MS) {
        if (smoke_fanout_count() == 0U) {
            SMK("[CAM-AUDIO-AFTER-VIDEO] abort: no viewers");
            return;
        }
        if (camera_webrtc_audio_is_active()) {
            return;
        }
        if (camera_h264_stream_is_active()) {
            camera_h264_stream_get_stats(&cs);
            if (cs.encoded_frames > 0U && cs.last_errno == 0) {
                ready = true;
                break;
            }
            if (cs.last_errno != 0) {
                SMKW("[CAM-AUDIO-AFTER-VIDEO] camera failed before audio: stage=%s errno=%d",
                     cs.last_stage, cs.last_errno);
                return;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(SMOKE_AUDIO_GATE_POLL_MS));
        waited_ms += SMOKE_AUDIO_GATE_POLL_MS;
    }

    if (!ready) {
        SMKW("[CAM-AUDIO-AFTER-VIDEO] timeout waiting video; audio not started");
        return;
    }

    SMK_DMA_DIAG("[DMA-DIAG D1 before_audio_start_after_video] free=%u largest=%u",
         (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA),
         (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA));
    esp_err_t ar = camera_webrtc_audio_start(smoke_fanout_opus, NULL);
    if (ar == ESP_OK) {
        SMK("camera audio start requested after video: ES8311 -> Opus %uHz mono",
            (unsigned)CAM_WEBRTC_AUDIO_SAMPLE_RATE);
    } else {
        SMKW("camera audio start failed after video: %s; video remains active",
             esp_err_to_name(ar));
    }
}

static void smoke_audio_gate_task(void *arg)
{
    (void)arg;
    SMKW("[CAM-AUDIO-GATE-PSRAM-F3] worker ready core=%d stack=%uB PSRAM",
         xPortGetCoreID(), (unsigned)(SMOKE_AUDIO_GATE_STACK * sizeof(StackType_t)));
    for (;;) {
        (void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (!__atomic_load_n(&s_audio_gate_running, __ATOMIC_ACQUIRE)) {
            continue;
        }
        smoke_audio_gate_run_once();
        __atomic_store_n(&s_audio_gate_running, false, __ATOMIC_RELEASE);
    }
}

static esp_err_t smoke_audio_gate_worker_init(void)
{
    if (s_audio_gate_task_handle) return ESP_OK;

    if (!s_audio_gate_stack) {
        s_audio_gate_stack = (StackType_t *)heap_caps_malloc(
            SMOKE_AUDIO_GATE_STACK * sizeof(StackType_t),
            MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!s_audio_gate_stack) {
            SMKE("[CAM-AUDIO-GATE-PSRAM-F3] persistent PSRAM stack alloc failed bytes=%u",
                 (unsigned)(SMOKE_AUDIO_GATE_STACK * sizeof(StackType_t)));
            return ESP_ERR_NO_MEM;
        }
    }

    s_audio_gate_task_handle = xTaskCreateStaticPinnedToCore(
        smoke_audio_gate_task, "smoke_aud_gate",
        SMOKE_AUDIO_GATE_STACK, NULL, SMOKE_AUDIO_GATE_PRIO,
        s_audio_gate_stack, &s_audio_gate_tcb, SMOKE_PUMP_CORE);
    if (!s_audio_gate_task_handle) {
        SMKE("[CAM-AUDIO-GATE-PSRAM-F3] persistent worker create failed");
        heap_caps_free(s_audio_gate_stack);
        s_audio_gate_stack = NULL;
        return ESP_ERR_NO_MEM;
    }
    SMKW("[CAM-AUDIO-GATE-PSRAM-F3] persistent worker created stack=%uB PSRAM; no runtime INTERNAL stack allocation",
         (unsigned)(SMOKE_AUDIO_GATE_STACK * sizeof(StackType_t)));
    return ESP_OK;
}

static void smoke_audio_start_after_video_schedule(void)
{
    if (camera_webrtc_audio_is_active() ||
        __atomic_load_n(&s_audio_gate_running, __ATOMIC_ACQUIRE)) {
        return;
    }
    if (!s_audio_gate_task_handle) {
        SMKW("[CAM-AUDIO-AFTER-VIDEO] persistent gate worker unavailable; audio not started");
        return;
    }
    __atomic_store_n(&s_audio_gate_running, true, __ATOMIC_RELEASE);
    xTaskNotifyGive(s_audio_gate_task_handle);
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
        const int64_t now_ms = esp_timer_get_time() / 1000;

        if (s_active) {
            const bool camera_orphan =
                s_active->running &&
                s_active->media_sent > 0U &&
                !cam_active;
            const bool connect_stale =
                s_active->running &&
                !s_active->peer_connected &&
                s_active->start_ms > 0 &&
                (now_ms - s_active->start_ms) >= (int64_t)SMOKE_SESSION_CONNECT_GRACE_MS;
            uint32_t heartbeat_age_ms = 0U;
            const bool heartbeat_stale =
                smoke_viewer_hb_expired(s_active, (uint32_t)now_ms,
                                        &heartbeat_age_ms);
            if (!s_active->running || camera_orphan || connect_stale || heartbeat_stale) {
                if (heartbeat_stale) {
                    SMKW("[CAM-VIEWER-HB-SIGNED-AGE-R1] sid=%u primary timeout age=%ums -> stop pump",
                        (unsigned)s_active->session_id,
                         (unsigned)heartbeat_age_ms);
                    __atomic_store_n(&s_active->viewer_hb_visible, false, __ATOMIC_RELEASE);
                    s_active->auto_cleanup_requested = true;
                    s_active->running = false;
                } else if (camera_orphan) {
                    SMK("sid=%u [STALE-CAM-REAP] primary running but camera inactive "
                        "media_sent=%u -> force reap",
                        (unsigned)s_active->session_id,
                        (unsigned)s_active->media_sent);
                    s_active->auto_cleanup_requested = false;
                    s_active->running = false;
                } else if (connect_stale) {
                    SMK("sid=%u [STALE-CONNECT-REAP] primary never connected in %lldms -> stop pump",
                        (unsigned)s_active->session_id,
                        (long long)(now_ms - s_active->start_ms));
                    s_active->auto_cleanup_requested = true;
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
            const bool connect_stale =
                s_secondary->running &&
                !s_secondary->peer_connected &&
                s_secondary->start_ms > 0 &&
                (now_ms - s_secondary->start_ms) >= (int64_t)SMOKE_SESSION_CONNECT_GRACE_MS;
            uint32_t heartbeat_age_ms = 0U;
            const bool heartbeat_stale =
                smoke_viewer_hb_expired(s_secondary, (uint32_t)now_ms,
                                        &heartbeat_age_ms);
            if (!s_secondary->running || camera_orphan || connect_stale || heartbeat_stale) {
                if (heartbeat_stale) {
                    SMKW("[CAM-VIEWER-HB-SIGNED-AGE-R1] sid=%u secondary timeout age=%ums -> stop pump",
                        (unsigned)s_secondary->session_id,
                         (unsigned)heartbeat_age_ms);
                    __atomic_store_n(&s_secondary->viewer_hb_visible, false, __ATOMIC_RELEASE);
                    s_secondary->auto_cleanup_requested = true;
                    s_secondary->running = false;
                } else if (camera_orphan) {
                    SMK("sid=%u [STALE-CAM-REAP] secondary running but camera inactive "
                        "media_sent=%u -> force reap",
                        (unsigned)s_secondary->session_id,
                        (unsigned)s_secondary->media_sent);
                    s_secondary->auto_cleanup_requested = false;
                    s_secondary->running = false;
                } else if (connect_stale) {
                    SMK("sid=%u [STALE-CONNECT-REAP] secondary never connected in %lldms -> stop pump",
                        (unsigned)s_secondary->session_id,
                        (long long)(now_ms - s_secondary->start_ms));
                    s_secondary->auto_cleanup_requested = true;
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
        const size_t prof_free0 = heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
        const size_t prof_largest0 = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
        SMK_DMA_DIAG("[DMA-LIFECYCLE-DIAG-R1] PROFILE0 before_set profile=%s free=%u largest=%u",
             camera_h264_stream_profile_name(profile),
             (unsigned)prof_free0, (unsigned)prof_largest0);
        esp_err_t pr = camera_h264_stream_set_profile(profile);
        SMK_DMA_DIAG("[DMA-LIFECYCLE-DIAG-R1] PROFILE1 after_set profile=%s ret=%s free=%u largest=%u delta_free=%d delta_largest=%d",
             camera_h264_stream_profile_name(profile), esp_err_to_name(pr),
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA),
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA),
             (int)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA) - (int)prof_free0,
             (int)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA) - (int)prof_largest0);
        if (pr != ESP_OK) {
            SMKE("camera profile select failed profile=%s err=%s",
                 camera_h264_stream_profile_name(profile), esp_err_to_name(pr));
            return pr;
        }

        esp_err_t tr = isp_tune_apply_profile(profile);
        SMK_DMA_DIAG("[DMA-LIFECYCLE-DIAG-R1] PROFILE2 after_isp profile=%s ret=%s free=%u largest=%u delta_free=%d delta_largest=%d",
             camera_h264_stream_profile_name(profile), esp_err_to_name(tr),
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA),
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA),
             (int)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA) - (int)prof_free0,
             (int)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA) - (int)prof_largest0);
        if (tr != ESP_OK) {
            SMKW("camera profile tune apply failed profile=%s err=%s; continuing",
                 camera_h264_stream_profile_name(profile), esp_err_to_name(tr));
        }

        /* [CAM-COLOR2-PER-PROFILE-20260910]
         * Apply Hue/WB from the same selected profile in backend, so correct
         * color does not depend on browser request timing. */
        esp_err_t c2r = camera_color_stage2_apply_profile(
                            profile == CAMERA_H264_PROFILE_720P_NATIVE);
        SMK_DMA_DIAG("[DMA-LIFECYCLE-DIAG-R1] PROFILE3 after_color2 profile=%s ret=%s free=%u largest=%u delta_free=%d delta_largest=%d",
             camera_h264_stream_profile_name(profile), esp_err_to_name(c2r),
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA),
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA),
             (int)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA) - (int)prof_free0,
             (int)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA) - (int)prof_largest0);
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
        SMKE("[DMA-BOUNDARY-R17F9] session PSRAM OOM size=%u; internal fallback forbidden",
             (unsigned)sizeof(smoke_sess_t));
        return ESP_ERR_NO_MEM;
    }
    SMK("session alloc=PSRAM size=%u [DMA-BOUNDARY-R17F9]", (unsigned)sizeof(smoke_sess_t));

    uint32_t sid = s_next_session_id++;
    if (sid == 0U) sid = s_next_session_id++;
    sess->session_id = sid;
    sess->pump_slot = SMOKE_PUMP_SLOT_NONE;
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
    /* [CAM-ICE-ALL-R1-20260928]
     * Production policy is always ALL. TURN remains configured as fallback,
     * but previous-path history can no longer force a relay-only session. */
    cfg.ice_trans_policy    = ESP_PEER_ICE_TRANS_POLICY_ALL;
    SMK("sid=%u [CAM-ICE-ALL-R1] ESP ICE policy=ALL",
        (unsigned)sess->session_id);
    cfg.server_lists        = s_smoke_ice_servers;
    cfg.server_num          = (uint8_t)(sizeof(s_smoke_ice_servers) / sizeof(s_smoke_ice_servers[0]));
    cfg.video_info.codec    = ESP_PEER_VIDEO_CODEC_H264;
    uint32_t video_w = 0, video_h = 0, video_fps = 0;
    smoke_profile_video_info(profile, &video_w, &video_h, &video_fps);
    cfg.video_info.width    = video_w;
    cfg.video_info.height   = video_h;
    cfg.video_info.fps      = video_fps;
    cfg.video_dir           = ESP_PEER_MEDIA_DIR_SEND_ONLY;
#if SMOKE_REVERSE_AUDIO_RX_PHASE1
    /* [REV-AUDIO-RX-AB-PHASE1-20260917] Enable inbound audio (browser mic -> ESP)
     * so peer_default builds the receive path; video stays SEND_ONLY (untouched).
     * The inbound Opus payload is discarded in Phase 1 (see smoke_on_audio_data). */
    cfg.audio_dir           = ESP_PEER_MEDIA_DIR_SEND_RECV;
    cfg.on_audio_info       = smoke_on_audio_info;
    cfg.on_audio_data       = smoke_on_audio_data;
    /* Bound ONLY the new audio receive jitter. All other rtp_cfg fields stay 0
     * => library defaults, so the existing SEND path (send_pool_size 400kB etc.)
     * and the video receive jitter (unused, video is SEND_ONLY) are unchanged. */
    peer_default_cfg.rtp_cfg.audio_recv_jitter.cache_size = SMOKE_RX_AUDIO_JITTER_BYTES;
    /* [DMA-BOUNDARY-R18A-DUAL-SENDRECV-20260919]
     * Keep BOTH viewers SEND_RECV. Bound peer-default candidate bookkeeping to the
     * same 12-candidate ceiling already used by this application. This does not
     * remove STUN/TURN/relay or reverse-audio; it only prevents the opaque default
     * peer from sizing candidate control storage to its larger library default. */
    peer_default_cfg.max_candidates = SMOKE_MAX_CAND;
#else
    cfg.audio_dir           = ESP_PEER_MEDIA_DIR_SEND_ONLY;
#endif
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

    SMKW("[DMA-BOUNDARY-R18A-DUAL-SENDRECV-20260919] sid=%u SEND_RECV=1 jitter=%uB max_candidates=%u peer-wrapper=PSRAM dtls-ctx=PSRAM",
         (unsigned)sid, (unsigned)SMOKE_RX_AUDIO_JITTER_BYTES,
         (unsigned)peer_default_cfg.max_candidates);

    const size_t peer_open_free_before = heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
    const size_t peer_open_largest_before = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
    SMK_DMA_DIAG("[DMA-DIAG PUMP-PROBE] sid=%u P0 before_esp_peer_open free=%u largest=%u",
         (unsigned)sid,
         (unsigned)peer_open_free_before,
         (unsigned)peer_open_largest_before);
#if SMOKE_REVERSE_AUDIO_RX_PHASE1
    SMK_WEBRTC_DIAG("[REV-AUDIO-RX-AB-PHASE1-R13-R1-20260917] [RX-AB] RXA0 before_esp_peer_open sid=%u audio_dir=SEND_RECV jitter_cache=%uB free=%u largest=%u",
         (unsigned)sid, (unsigned)SMOKE_RX_AUDIO_JITTER_BYTES,
         (unsigned)peer_open_free_before, (unsigned)peer_open_largest_before);
#endif
    int r = esp_peer_open(&cfg, esp_peer_get_default_impl(), &sess->peer);
    smoke_dma_probe_log(sid, "P1 after_esp_peer_open",
                        peer_open_free_before, peer_open_largest_before);
#if SMOKE_REVERSE_AUDIO_RX_PHASE1
    SMK_WEBRTC_DIAG("[RX-AB] RXA1 after_esp_peer_open sid=%u r=%d free=%u largest=%u",
         (unsigned)sid, r,
         (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA),
         (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA));
#endif
    if (r != ESP_PEER_ERR_NONE || !sess->peer) {
        SMKE("sid=%u esp_peer_open failed r=%d", (unsigned)sid, r);
        goto fail;
    }

    sess->running = true;
    if (secondary) s_secondary = sess;
    else s_active = sess;

    const size_t pump_create_free_before = heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
    const size_t pump_create_largest_before = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
    SMK_DMA_DIAG("[DMA-DIAG PUMP-PROBE] sid=%u P2 before_persistent_pump_assign stack=%u free=%u largest=%u",
         (unsigned)sid,
         (unsigned)SMOKE_PUMP_STACK,
         (unsigned)pump_create_free_before,
         (unsigned)pump_create_largest_before);
    const int pump_slot = smoke_pump_slot_acquire(sess);
    if (pump_slot < 0) {
        smoke_dma_probe_log(sid, "P3 persistent_pump_no_slot",
                            pump_create_free_before, pump_create_largest_before);
        SMKE("sid=%u persistent pump slot unavailable", (unsigned)sid);
        if (s_active == sess) s_active = NULL;
        if (s_secondary == sess) s_secondary = NULL;
        sess->running = false;
        esp_peer_close(sess->peer);
        sess->peer = NULL;
        goto fail;
    }
    smoke_dma_probe_log(sid, "P3 after_persistent_pump_assign",
                        pump_create_free_before, pump_create_largest_before);
    SMK("sid=%u persistent pump assigned slot=%d stack=%u",
        (unsigned)sid, pump_slot, (unsigned)SMOKE_PUMP_STACK);
    xTaskNotifyGive(sess->pump);

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

/* peer ?출 helper (mutex) */
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

/* ????????????? HTTP ?틸 ????????????? */

static char *read_body(httpd_req_t *req)
{
    int total = req->content_len;
    if (HA_VERBOSE_DIAG_LOGS) SMK("HTTP body len=%d (max=%d)", total, SMOKE_BODY_MAX);
    if (total <= 0 || total > SMOKE_BODY_MAX) return NULL;

    size_t dma_before = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
    char *buf = (char *)smoke_sig_psram_alloc((size_t)total + 1U);
    if (!buf) {
        SMKE("[DMA-BOUNDARY-R17F9] HTTP body PSRAM OOM len=%d; internal fallback forbidden", total);
        return NULL;
    }
    size_t dma_after = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
    SMKW("[DMA-BOUNDARY-R17F9] HTTP body -> PSRAM len=%d dma_largest=%u->%u delta=%d",
         total, (unsigned)dma_before, (unsigned)dma_after,
         (int)dma_after - (int)dma_before);
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

/* SDP ?성 ??+ candidate ?집????친 SDP 반환(?출??free). ?패 ??NULL */
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
        const bool lan_verified = s->lan_relay_wait_bypass;
        if (lan_verified) {
            SMKW("[LAN-RELAY-WAIT-R5F2] sid=%u SKIP legacy TURN relay wait reason=paired-verified-stun-cache relay=%d",
                 (unsigned)s->session_id, relay_ready ? 1 : 0);
        } else {
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
            SMK("sid=%u [TURN-RELAY-EMIT-AB] wait end=%dms relay=%d lan_verified=%d",
                (unsigned)s->session_id, waited_ms, relay_ready ? 1 : 0, lan_verified ? 1 : 0);
        }
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

/* ????????????? HTTP ?들??????????????? */

/* TEST1: POST /api/smoke/offer  {mode:"A"|"B", type:"offer", sdp}
 *   ESP = answerer. ?? 강제 ?음. esp_peer 가 고른 answer setup ??관?반환. */
/* [ANDROID-ICE-LITE-AB] Production signaling path:
 * Browser creates offer (full ICE) -> ESP answers as ICE Lite.
 * This also avoids the known esp_peer browser-answer mid=255 parser path. */
static esp_err_t h_offer_post(httpd_req_t *req)
{
    const size_t offer_probe_base_largest =
        heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
    size_t offer_probe_prev_largest = offer_probe_base_largest;
    offer_probe_prev_largest = smoke_offer_dma_probe_log(
        "O0 handler_entry", offer_probe_base_largest, offer_probe_prev_largest);
    smoke_fr_offer_probe_unlocked("O0_ENTRY", 0, offer_probe_base_largest);

    isp_tune_restore_saved_settings_once();
    offer_probe_prev_largest = smoke_offer_dma_probe_log(
        "O1 after_isp_restore", offer_probe_base_largest, offer_probe_prev_largest);
    smoke_fr_offer_probe_unlocked("O1_ISP", 1, offer_probe_base_largest);

    char *body = read_body(req);
    offer_probe_prev_largest = smoke_offer_dma_probe_log(
        "O2 after_read_body", offer_probe_base_largest, offer_probe_prev_largest);
    smoke_fr_offer_probe_unlocked("O2_BODY", 2, offer_probe_base_largest);
    if (!body) return send_err(req, "400 Bad Request", "bad_body");

    cJSON *in = cJSON_Parse(body);
    free(body);
    offer_probe_prev_largest = smoke_offer_dma_probe_log(
        "O3 after_json_parse_body_free", offer_probe_base_largest, offer_probe_prev_largest);
    smoke_fr_offer_probe_unlocked("O3_CJSON", 3, offer_probe_base_largest);
    if (!in) return send_err(req, "400 Bad Request", "bad_json");

    const cJSON *jsdp = cJSON_GetObjectItem(in, "sdp");
    const cJSON *jmode = cJSON_GetObjectItem(in, "mode");
    const cJSON *jprofile = cJSON_GetObjectItem(in, "profile");
    const cJSON *jclient_token = cJSON_GetObjectItem(in, "client_token");
    const cJSON *jdiag_browser_id = cJSON_GetObjectItem(in, "diag_browser_id");
    const cJSON *jdiag_page_id = cJSON_GetObjectItem(in, "diag_page_id");
    const cJSON *jdiag_offer_seq = cJSON_GetObjectItem(in, "diag_offer_seq");
    const cJSON *jviewer_heartbeat = cJSON_GetObjectItem(in, "viewer_heartbeat");
    const cJSON *jesp_relay_only = cJSON_GetObjectItem(in, "esp_relay_only");
    const bool esp_relay_only_requested = cJSON_IsTrue(jesp_relay_only);
    /* [CAM-ICE-ALL-R1-20260928]
     * Legacy clients may still send esp_relay_only=true. Never force the ESP
     * side to RELAY: ALL keeps direct/STUN and TURN candidates together. */
    const bool esp_relay_only = false;
    if (esp_relay_only_requested) {
        SMKW("[CAM-ICE-ALL-R1] legacy esp_relay_only request ignored; policy=ALL");
    }
    const bool viewer_heartbeat = cJSON_IsTrue(jviewer_heartbeat);
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
    const char *diag_browser_id = NULL;
    const char *diag_page_id = NULL;
    uint32_t diag_offer_seq = 0U;
    if (jdiag_browser_id) {
        if (!cJSON_IsString(jdiag_browser_id) ||
            !smoke_diag_id_valid(jdiag_browser_id->valuestring)) {
            cJSON_Delete(in);
            return send_err(req, "400 Bad Request", "bad_diag_browser_id");
        }
        diag_browser_id = jdiag_browser_id->valuestring;
    }
    if (jdiag_page_id) {
        if (!cJSON_IsString(jdiag_page_id) ||
            !smoke_diag_id_valid(jdiag_page_id->valuestring)) {
            cJSON_Delete(in);
            return send_err(req, "400 Bad Request", "bad_diag_page_id");
        }
        diag_page_id = jdiag_page_id->valuestring;
    }
    if (jdiag_offer_seq && cJSON_IsNumber(jdiag_offer_seq) &&
        jdiag_offer_seq->valuedouble > 0 && jdiag_offer_seq->valuedouble <= 4294967295.0) {
        diag_offer_seq = (uint32_t)jdiag_offer_seq->valuedouble;
    }
    if (!cJSON_IsString(jsdp) || !jsdp->valuestring[0]) {
        cJSON_Delete(in);
        return send_err(req, "400 Bad Request", "no_sdp");
    }

    char remote_setup[24];
    parse_setup(jsdp->valuestring, remote_setup, sizeof(remote_setup));
    offer_probe_prev_largest = smoke_offer_dma_probe_log(
        "O4 after_validate_parse", offer_probe_base_largest, offer_probe_prev_largest);
    smoke_fr_offer_probe_unlocked("O4_VALID", 4, offer_probe_base_largest);

    if (xSemaphoreTake(s_gate, pdMS_TO_TICKS(500)) != pdTRUE) {
        cJSON_Delete(in);
        return send_err(req, "409 Conflict", "session_gate_busy");
    }
    smoke_reap_stale_locked();
    smoke_fr_record_locked(FR_EVENT_OFFER_GATE, NULL, profile, client_token,
                           "OFFER", (int32_t)mode, esp_relay_only ? 1 : 0);

    /* [CAM-SHARED-OP-COORD-R6-20260916]
     * While one browser owns calibration or a shared profile switch, only that
     * exact page token may open replacement sessions. Other pages pause their
     * own sessions and auto-resume after the owner releases the lease. */
    smoke_auto_cal_expire_locked();
    if (s_auto_cal_active) {
        if (!smoke_auto_cal_owner_locked(client_token)) {
            const char *busy_err = s_auto_cal_profile_switch
                ? "profile_switch_in_progress"
                : "calibration_in_progress";
            xSemaphoreGive(s_gate);
            cJSON_Delete(in);
            return send_err(req, "409 Conflict", busy_err);
        }
        smoke_auto_cal_refresh_locked();
    }

    /* A retry from the SAME browser page may arrive after its previous /offer
     * created an ESP session but the HTTP answer (and therefore session_id) was
     * lost. Reclaim only that exact token while holding s_gate, BEFORE normal
     * viewer-count/primary_connecting checks. Never touch another viewer. */
    if (client_token) {
        smoke_sess_t *owned = smoke_find_client_token_locked(client_token);
        if (owned) {
            if (owned->peer_connected) {
                smoke_fr_trigger_locked(FR_EVENT_CLIENT_SESSION_ACTIVE, owned, profile,
                                        client_token, "CLIENT_ACT", 0, 0);
                xSemaphoreGive(s_gate);
                cJSON_Delete(in);
                (void)flight_recorder_service();
                return send_err(req, "409 Conflict", "client_session_active");
            }
            SMK("sid=%u [CLIENT-ORPHAN-RECLAIM] exact token %.8s -> teardown",
                (unsigned)owned->session_id, client_token);
            smoke_fr_record_locked(FR_EVENT_ORPHAN_RECLAIM, owned, profile, client_token,
                                   "ORPHAN", 0, 0);
            owned->auto_cleanup_requested = false;
            if (!smoke_teardown_locked(owned)) {
                smoke_fr_trigger_locked(FR_EVENT_ORPHAN_RECLAIM, owned, profile, client_token,
                                        "ORPH_DEFER", 1, 0);
                xSemaphoreGive(s_gate);
                cJSON_Delete(in);
                (void)flight_recorder_service();
                return send_err(req, "409 Conflict", "client_orphan_cleanup_deferred");
            }
        }
    }

    uint32_t count = smoke_session_count_locked();
    if (count >= SMOKE_MAX_SESSIONS) {
        const uint32_t connected_count = smoke_session_connected_count_locked();
        const bool reap_pending =
            (s_active && !s_active->running) ||
            (s_secondary && !s_secondary->running);
        const bool truly_full = (connected_count >= SMOKE_MAX_SESSIONS);
        const char *gate_tag = reap_pending ? "MAX_REAP" :
                               (truly_full ? "MAX_VIEW" : "MAX_SLOT");
        smoke_fr_trigger_locked(FR_EVENT_MAX_VIEWERS, NULL, profile, client_token,
                                gate_tag, (int32_t)count, (int32_t)connected_count);
        SMKW("[CAM-STALE-SLOT-VIEW0-FIX-R9-20260917] offer gate slots=%u connected=%u "
             "reap_pending=%u -> %s",
             (unsigned)count, (unsigned)connected_count,
             reap_pending ? 1U : 0U,
             reap_pending ? "reaping_in_progress" :
             (truly_full ? "max_2_viewers" : "session_slots_busy"));
        xSemaphoreGive(s_gate);
        cJSON_Delete(in);
        (void)flight_recorder_service();
        return send_err(req, "409 Conflict",
                        reap_pending ? "reaping_in_progress" :
                        (truly_full ? "max_2_viewers" : "session_slots_busy"));
    }
    if (s_active) {
        if (s_active->profile != profile) {
            smoke_fr_record_locked(FR_EVENT_PROFILE_IN_USE, s_active, profile, client_token,
                                    "PROF_USE", (int32_t)s_active->profile, (int32_t)profile);
            xSemaphoreGive(s_gate);
            cJSON_Delete(in);
            (void)flight_recorder_service();
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

    offer_probe_prev_largest = smoke_offer_dma_probe_log(
        "O5 after_gate_reap_guards", offer_probe_base_largest, offer_probe_prev_largest);
    smoke_fr_record_locked(FR_EVENT_OFFER_PROBE, NULL, profile, client_token,
                           "O5_GATE", 5, smoke_fr_offer_probe_delta(offer_probe_base_largest));
    /* If the pre-offer timer was stopped by this HTTP request, take one final
     * gate-safe sample against the R8 baseline before creating a session. */
    if (s_dma_preoffer_base_largest != 0U) {
        smoke_dma_blackbox_sample_locked(
            (uint32_t)(esp_timer_get_time() / 1000ULL));
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
    int mdns_stripped = 0;
    const int remote_candidate_count = count_sdp_candidate_lines(jsdp->valuestring);
    const bool http_peer_ok =
        remote_candidate_count > 0 && smoke_http_peer_ipv4(req, http_peer_ip);
    const bool http_peer_private =
        http_peer_ok && smoke_ipv4_is_private_lan(http_peer_ip);

    char lan_mdns_name[128] = {0};
    char lan_mdns_candidate_template[SMOKE_CAND_LEN] = {0};
    char lan_cached_ip[16] = {0};
    uint16_t lan_mdns_candidate_port = 0U;
    uint32_t lan_cache_age_ms = 0U;
    uint8_t lan_cache_source = SMOKE_LAN_CACHE_SRC_NONE;
    bool lan_async_needed = false;
    bool lan_cached_trickle_ready = false;

    if (http_peer_private) {
        remote_sdp_owned = smoke_rewrite_lan_candidates_ipv4(
            jsdp->valuestring, http_peer_ip, &lan_rewritten);
        if (remote_sdp_owned && lan_rewritten > 0) {
            remote_sdp = remote_sdp_owned;
            SMK_WEBRTC_INFO("[LAN-DIRECT-ICE-ASYNC-R4] private HTTP peer=%s candidates=%d mdns_rewrote=%d",
                http_peer_ip, remote_candidate_count, lan_rewritten);
            log_remote_candidates(remote_sdp);
        } else if (remote_candidate_count == 1) {
            SMK_WEBRTC_INFO("[LAN-DIRECT-ICE-ASYNC-R4] single candidate already numeric for HTTP peer=%s",
                http_peer_ip);
        }
    } else {
        const bool have_mdns = smoke_first_mdns_candidate_name(
            jsdp->valuestring, lan_mdns_name, sizeof(lan_mdns_name));
        const bool have_template = have_mdns && smoke_extract_mdns_candidate_message(
            jsdp->valuestring, lan_mdns_candidate_template,
            sizeof(lan_mdns_candidate_template), &lan_mdns_candidate_port);
        const bool cache_hit = have_template && smoke_lan_mdns_cache_lookup(
            lan_mdns_name, lan_cached_ip, &lan_cache_age_ms, &lan_cache_source);
        lan_async_needed = have_template && !cache_hit;

        if (cache_hit) {
            remote_sdp_owned = smoke_rewrite_lan_candidates_ipv4(
                jsdp->valuestring, lan_cached_ip, &lan_rewritten);
            if (remote_sdp_owned && lan_rewritten > 0) {
                remote_sdp = remote_sdp_owned;
                SMK_WEBRTC_INFO("[LAN-DIRECT-ICE-ASYNC-R4] CACHE-HIT public HTTP peer=%s mdns=%s -> %s age=%ums rewrote=%d source=%s",
                    http_peer_ok ? http_peer_ip : "(unknown)", lan_mdns_name,
                    lan_cached_ip, (unsigned)lan_cache_age_ms, lan_rewritten,
                    lan_cache_source == SMOKE_LAN_CACHE_SRC_STUN ? "STUN" :
                    (lan_cache_source == SMOKE_LAN_CACHE_SRC_MDNS ? "mDNS" : "unknown"));
                log_remote_candidates(remote_sdp);
            } else {
                lan_cached_trickle_ready = true;
                SMK_WEBRTC_DIAG("[LAN-DIRECT-ICE-ASYNC-R4] CACHE-HIT rewrite allocation failed; use PAIRING trickle %s -> %s",
                     lan_mdns_name, lan_cached_ip);
            }
        }

        if (remote_sdp == jsdp->valuestring) {
            remote_sdp_owned = smoke_strip_mdns_candidates_if_alternative(
                jsdp->valuestring, &mdns_stripped);
            if (remote_sdp_owned && mdns_stripped > 0) {
                remote_sdp = remote_sdp_owned;
                if (lan_cached_trickle_ready) lan_async_needed = false;
                SMK_WEBRTC_INFO("[LAN-DIRECT-ICE-ASYNC-R4] public/non-private HTTP peer=%s mdns=%s stripped=%d mode=%s; NO PREWAIT",
                    http_peer_ok ? http_peer_ip : "(unknown)",
                    have_mdns ? lan_mdns_name : "NONE", mdns_stripped,
                    lan_cached_trickle_ready ? "CACHE-TRICKLE" :
                    (lan_async_needed ? "ASYNC" : "TURN-FALLBACK"));
            }
        }
        if (lan_async_needed && mdns_stripped == 0 && remote_sdp == jsdp->valuestring) {
            SMK_WEBRTC_INFO("[LAN-DIRECT-ICE-ASYNC-R4] public/non-private HTTP peer=%s mdns=%s kept in SDP; async resolver still scheduled; NO PREWAIT",
                http_peer_ok ? http_peer_ip : "(unknown)", lan_mdns_name);
        }
    }

    offer_probe_prev_largest = smoke_offer_dma_probe_log(
        "O6 before_smoke_start", offer_probe_base_largest, offer_probe_prev_largest);
    smoke_fr_record_locked(FR_EVENT_OFFER_PROBE, NULL, profile, client_token,
                           "O6_START", 6, smoke_fr_offer_probe_delta(offer_probe_base_largest));

    smoke_sess_t *sess = NULL;
    s_handover_dual_relay_ab2_next = esp_relay_only;
    esp_err_t sr0 = smoke_start_locked(false, mode, profile, &sess);
    s_handover_dual_relay_ab2_next = false;
    if (sr0 != ESP_OK || !sess) {
        if (sr0 == ESP_ERR_INVALID_ARG) {
            smoke_fr_record_locked(FR_EVENT_PROFILE_IN_USE, s_active, profile, client_token,
                                    "PROF_START", s_active ? (int32_t)s_active->profile : -1, (int32_t)profile);
        }
        xSemaphoreGive(s_gate);
        free(remote_sdp_owned);
        cJSON_Delete(in);
        if (sr0 == ESP_ERR_INVALID_ARG) {
            (void)flight_recorder_service();
            return send_err(req, "409 Conflict", "profile_in_use");
        }
        if (sr0 == ESP_ERR_INVALID_STATE)
            return send_err(req, "409 Conflict", "primary_connecting");
        if (sr0 == ESP_ERR_NO_MEM)
            return send_err(req, "503 Service Unavailable", "peer_resource_exhausted");
        return send_err(req, "503 Service Unavailable", "peer_open_failed");
    }

    sess->lan_mdns_job_slot = SMOKE_LAN_MDNS_JOB_NONE;
    sess->lan_mdns_inject_result = ESP_PEER_ERR_FAIL;
    /* [LAN-RELAY-WAIT-R5F2-20260927]
     * Only an exact-name SRC_STUN cache entry promoted after ICE PAIRED verified
     * the learned endpoint may bypass the legacy local-relay wait. mDNS-only
     * cache entries retain the old wait, preserving TURN fallback semantics. */
    sess->lan_relay_wait_bypass =
        (!http_peer_private && lan_cache_source == SMOKE_LAN_CACHE_SRC_STUN &&
         lan_cached_ip[0] != '\0');
    if (sess->lan_relay_wait_bypass) {
        SMKW("[LAN-RELAY-WAIT-R5F2] sid=%u paired-verified STUN-cache %s age=%ums -> relay wait bypass armed",
             (unsigned)sess->session_id, lan_cached_ip, (unsigned)lan_cache_age_ms);
    }
    if (!http_peer_private && lan_mdns_candidate_template[0]) {
        snprintf(sess->lan_mdns_name, sizeof(sess->lan_mdns_name), "%s", lan_mdns_name);
        snprintf(sess->lan_mdns_candidate_template,
                 sizeof(sess->lan_mdns_candidate_template), "%s",
                 lan_mdns_candidate_template);
        sess->lan_mdns_candidate_port = lan_mdns_candidate_port;

        if (lan_cached_trickle_ready) {
            snprintf(sess->lan_mdns_resolved_ip, sizeof(sess->lan_mdns_resolved_ip), "%s",
                     lan_cached_ip);
            __atomic_store_n(&sess->lan_mdns_candidate_ready, true, __ATOMIC_RELEASE);
            SMK_WEBRTC_DIAG("[LAN-DIRECT-ICE-ASYNC-R4] sid=%u CACHE-TRICKLE ready %s:%u age=%ums free=%u largest=%u",
                 (unsigned)sess->session_id, sess->lan_mdns_resolved_ip,
                 (unsigned)sess->lan_mdns_candidate_port, (unsigned)lan_cache_age_ms,
                 (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA),
                 (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA));
        } else if (lan_async_needed) {
            const int job_slot = smoke_lan_mdns_schedule_locked(
                sess->session_id, sess->lan_mdns_name);
            if (job_slot >= 0) {
                sess->lan_mdns_job_slot = (uint8_t)job_slot;
                SMK_WEBRTC_DIAG("[LAN-DIRECT-ICE-ASYNC-R4] sid=%u ASYNC scheduled name=%s port=%u slot=%d free=%u largest=%u",
                     (unsigned)sess->session_id, sess->lan_mdns_name,
                     (unsigned)sess->lan_mdns_candidate_port, job_slot,
                     (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA),
                     (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA));
            } else {
                SMK_WEBRTC_DIAG("[LAN-DIRECT-ICE-ASYNC-R4] sid=%u ASYNC unavailable/busy; TURN/srflx fallback unchanged",
                     (unsigned)sess->session_id);
            }
        }
    }

    if (client_token) {
        snprintf(sess->client_token, sizeof(sess->client_token), "%s", client_token);
        SMK("sid=%u [CLIENT-TOKEN] bound %.8s",
            (unsigned)sess->session_id, client_token);
    }
    if (diag_browser_id)
        snprintf(sess->diag_browser_id, sizeof(sess->diag_browser_id), "%s", diag_browser_id);
    if (diag_page_id)
        snprintf(sess->diag_page_id, sizeof(sess->diag_page_id), "%s", diag_page_id);
    sess->diag_offer_seq = diag_offer_seq;
    sess->viewer_hb_required = viewer_heartbeat;
    if (viewer_heartbeat) {
        const uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);
        __atomic_store_n(&sess->viewer_hb_last_ms, now_ms, __ATOMIC_RELEASE);
        __atomic_store_n(&sess->viewer_hb_last_frames, 0U, __ATOMIC_RELEASE);
        __atomic_store_n(&sess->viewer_hb_seen, false, __ATOMIC_RELEASE);
        __atomic_store_n(&sess->viewer_hb_visible, false, __ATOMIC_RELEASE);
        SMK_WEBRTC_DIAG("[CAM-VIEWER-HEARTBEAT-R1] sid=%u armed timeout=%ums page=%s",
             (unsigned)sess->session_id,
             (unsigned)SMOKE_VIEWER_HEARTBEAT_TIMEOUT_MS,
             sess->diag_page_id[0] ? sess->diag_page_id : "-");
    }
    SMK_WEBRTC_DIAG("[DUPVIEW-DIAG-R1] OPEN sid=%u token=%.8s browser=%s page=%s offer_seq=%u",
         (unsigned)sess->session_id, sess->client_token,
         sess->diag_browser_id[0] ? sess->diag_browser_id : "-",
         sess->diag_page_id[0] ? sess->diag_page_id : "-",
         (unsigned)sess->diag_offer_seq);
    smoke_fr_record_locked(FR_EVENT_SESSION_OPEN, sess, profile, client_token,
                           "OPEN", (int32_t)mode, 0);

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
    smoke_fr_record_locked(FR_EVENT_OFFER_PROBE, sess, profile, NULL,
                           "O7_CJDEL", 7, smoke_fr_offer_probe_delta(offer_probe_base_largest));
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

/* TEST2: GET /api/smoke/offer?mode=C   ??ESP = offerer. offer ?성/반환. */
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
    smoke_fr_record_locked(FR_EVENT_OFFER_GATE, NULL, profile, NULL,
                           "OFFER_GET", (int32_t)mode, 0);

    smoke_auto_cal_expire_locked();
    if (s_auto_cal_active) {
        xSemaphoreGive(s_gate);
        return send_err(req, "409 Conflict", "calibration_in_progress");
    }

    uint32_t count = smoke_session_count_locked();
    if (count >= SMOKE_MAX_SESSIONS) {
        const uint32_t connected_count = smoke_session_connected_count_locked();
        const bool reap_pending =
            (s_active && !s_active->running) ||
            (s_secondary && !s_secondary->running);
        const bool truly_full = (connected_count >= SMOKE_MAX_SESSIONS);
        smoke_fr_trigger_locked(FR_EVENT_MAX_VIEWERS, NULL, profile, NULL,
                                reap_pending ? "MAX_REAP_G" :
                                (truly_full ? "MAX_VIEW_G" : "MAX_SLOT_G"),
                                (int32_t)count, (int32_t)connected_count);
        xSemaphoreGive(s_gate);
        (void)flight_recorder_service();
        return send_err(req, "409 Conflict",
                        reap_pending ? "reaping_in_progress" :
                        (truly_full ? "max_2_viewers" : "session_slots_busy"));
    }

    /* The second ESP-offerer peer joins the existing single encoder. It must
     * use the same camera profile, and the first stream must already be LIVE. */
    if (s_active) {
        if (s_active->profile != profile) {
            smoke_fr_record_locked(FR_EVENT_PROFILE_IN_USE, s_active, profile, NULL,
                                    "PROF_USE_G", (int32_t)s_active->profile, (int32_t)profile);
            xSemaphoreGive(s_gate);
            (void)flight_recorder_service();
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
        if (sr0 == ESP_ERR_INVALID_ARG) {
            smoke_fr_record_locked(FR_EVENT_PROFILE_IN_USE, s_active, profile, NULL,
                                    "PROF_ST_G", s_active ? (int32_t)s_active->profile : -1, (int32_t)profile);
        }
        xSemaphoreGive(s_gate);
        if (sr0 == ESP_ERR_INVALID_ARG) {
            (void)flight_recorder_service();
            return send_err(req, "409 Conflict", "profile_in_use");
        }
        if (sr0 == ESP_ERR_INVALID_STATE)
            return send_err(req, "409 Conflict", "primary_connecting");
        if (sr0 == ESP_ERR_NO_MEM)
            return send_err(req, "503 Service Unavailable", "peer_resource_exhausted");
        return send_err(req, "503 Service Unavailable", "peer_open_failed");
    }

    smoke_fr_record_locked(FR_EVENT_SESSION_OPEN, sess, profile, NULL,
                           "OPEN_GET", (int32_t)mode, 0);

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

/* /api/smoke/offer ?스?치: GET(mode=C) vs POST */
static esp_err_t h_offer(httpd_req_t *req)
{
    if (__atomic_load_n(&s_dma_timer_mode, __ATOMIC_ACQUIRE) ==
            SMOKE_DMA_TIMER_MODE_PREOFFER) {
        smoke_dma_preoffer_watch_stop("offer_handler_entry");
    }
    if (req->method == HTTP_GET)  return h_offer_get(req);
    if (req->method == HTTP_POST) return h_offer_post(req);
    return send_err(req, "405 Method Not Allowed", "method");
}

/* TEST2: POST /api/smoke/answer  {mode:"C", type:"answer", sdp}  ??browser answer 주입 */
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

    const bool tune_dirty = use_live &&
        ((shown_enabled != saved_enabled) ||
         (shown_enabled && saved_enabled &&
          (shown_sat != saved_sat ||
           shown_con != saved_con ||
           shown_gamma != saved_gamma ||
           shown_sharp != saved_sharp)));

    camera_gain_load_once();
    const int gain_i = camera_gain_profile_index(profile);
    const bool gain_manual = s_camera_gain.live_manual[gain_i];
    const uint32_t gain_value = s_camera_gain.live_value[gain_i];
    const bool saved_gain_manual = s_camera_gain.saved_manual[gain_i];
    const uint32_t saved_gain_value = s_camera_gain.saved_value[gain_i];
    const bool gain_dirty = camera_gain_dirty(profile);

    uint32_t req_exp = 0, req_gain = 0, applied_exp = 0, applied_gain = 0;
    uint32_t left_ms = 0, writes = 0, group_writes = 0, exp_writes = 0, gain_writes = 0;
    uint32_t last_path = 0, suppressed = 0;
    bool hold_active = false;
    ov02c10_ae_diag_get(&req_exp, &req_gain, &applied_exp, &applied_gain,
                        &left_ms, &writes, &group_writes, &exp_writes, &gain_writes,
                        &last_path, &suppressed, &hold_active);
    const int current_gain = stream_same_profile ? (int)applied_gain : -1;
    const int current_exposure = stream_same_profile ? (int)applied_exp : -1;
    const bool dirty = tune_dirty || gain_dirty;

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

    cJSON_AddBoolToObject(t, "gain_manual", gain_manual);
    cJSON_AddNumberToObject(t, "gain_value", gain_value);
    cJSON_AddNumberToObject(t, "gain_current", current_gain);
    cJSON_AddNumberToObject(t, "exposure_current", current_exposure);
    cJSON_AddNumberToObject(t, "gain_max", camera_gain_profile_max(profile));
    cJSON_AddBoolToObject(t, "saved_gain_manual", saved_gain_manual);
    cJSON_AddNumberToObject(t, "saved_gain_value", saved_gain_value);
    cJSON_AddBoolToObject(t, "gain_dirty", gain_dirty);
    cJSON_AddBoolToObject(t, "tune_dirty", tune_dirty);

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

    /* [OV02C10-AE-ALLPATH-DIAG-R2-20260922]
     * POST /api/smoke/isp diagnostic commands:
     *   {"ae_diag":"watch","ms":15000}                 -> log IPA requested/applied exp+gain
     *   {"ae_diag":"hold","ms":10000}                  -> freeze sensor exp+gain temporarily
     *   {"ae_diag":"release"}                            -> end full hold immediately
     *   {"ae_diag":"gain_force",...}                  -> R18 production: disabled
     *   {"ae_diag":"gain_auto"}                          -> release gain clamp, return AGC
     *   {"ae_diag":"status"}                             -> read counters/current values
     * This branch returns before normal colour-tune validation and never saves
     * anything to NVS. */
    const cJSON *jae = cJSON_GetObjectItem(in, "ae_diag");
    if (cJSON_IsString(jae) && jae->valuestring) {
        /* [CAM-AUTOCAL-AE-HOLD-R14-20260922]
         * R2 kept cmd as a pointer owned by `in`, then deleted `in` before
         * cJSON_AddStringToObject() below. That explains the occasional
         * garbage `ae_diag` text observed in PowerShell. Keep a tiny local
         * copy; command semantics and hold timing are unchanged. */
        char cmd_copy[16] = {0};
        strncpy(cmd_copy, jae->valuestring, sizeof(cmd_copy) - 1U);
        const char *cmd = cmd_copy;
        const cJSON *jms = cJSON_GetObjectItem(in, "ms");
        uint32_t ms = cJSON_IsNumber(jms) && jms->valuedouble > 0.0 ?
                      (uint32_t)jms->valuedouble : 0U;
        esp_err_t ar = ESP_OK;
        if (strcmp(cmd, "watch") == 0) {
            if (ms == 0U) ms = 15000U;
            ar = ov02c10_ae_diag_watch_ms(ms);
        } else if (strcmp(cmd, "hold") == 0) {
            if (ms == 0U) ms = 10000U;
            ar = ov02c10_ae_diag_hold_ms(ms);
        } else if (strcmp(cmd, "release") == 0) {
            ov02c10_ae_diag_release();
        } else if (strcmp(cmd, "gain_force") == 0) {
            /* [CAM-AUTOCAL-720-AE-SETTLE-R18-20260922]
             * R15/R16 manual gain force was diagnostic-only. Production R18
             * rejects stale cached-page attempts so AUTO convergence cannot be
             * silently pinned at 223 before calibration HOLD. */
            cJSON_Delete(in);
            return send_err(req, "409 Conflict", "gain_force_disabled_r18");
        } else if (strcmp(cmd, "gain_auto") == 0) {
            ov02c10_ae_diag_gain_force_release();
        } else if (strcmp(cmd, "status") != 0) {
            cJSON_Delete(in);
            return send_err(req, "400 Bad Request", "bad_ae_diag_command");
        }
        if (ar == ESP_ERR_INVALID_ARG) {
            cJSON_Delete(in);
            return send_err(req, "400 Bad Request", "ae_diag_arg_out_of_range");
        }
        if (ar != ESP_OK) {
            cJSON_Delete(in);
            return send_err(req, "503 Service Unavailable", "ae_diag_failed");
        }
        uint32_t req_exp = 0, req_gain = 0, applied_exp = 0, applied_gain = 0;
        uint32_t left_ms = 0, writes = 0, group_writes = 0, exp_writes = 0, gain_writes = 0;
        uint32_t last_path = 0, suppressed = 0;
        uint32_t gain_force_value = 0, gain_force_left_ms = 0, gain_force_suppressed = 0;
        bool hold_active = false, gain_force_active = false;
        ov02c10_ae_diag_get(&req_exp, &req_gain, &applied_exp, &applied_gain,
                            &left_ms, &writes, &group_writes, &exp_writes, &gain_writes,
                            &last_path, &suppressed, &hold_active);
        ov02c10_ae_diag_gain_force_get(&gain_force_value, &gain_force_left_ms,
                                       &gain_force_suppressed, &gain_force_active);
        cJSON_Delete(in);
        cJSON *out = cJSON_CreateObject();
        if (!out) return send_err(req, "503 Service Unavailable", "oom");
        cJSON_AddBoolToObject(out, "ok", true);
        cJSON_AddStringToObject(out, "ae_diag", cmd);
        cJSON_AddBoolToObject(out, "hold_active", hold_active);
        cJSON_AddNumberToObject(out, "hold_left_ms", left_ms);
        cJSON_AddNumberToObject(out, "req_exp", req_exp);
        cJSON_AddNumberToObject(out, "req_gain", req_gain);
        cJSON_AddNumberToObject(out, "applied_exp", applied_exp);
        cJSON_AddNumberToObject(out, "applied_gain", applied_gain);
        cJSON_AddNumberToObject(out, "writes", writes);
        cJSON_AddNumberToObject(out, "group_writes", group_writes);
        cJSON_AddNumberToObject(out, "exp_writes", exp_writes);
        cJSON_AddNumberToObject(out, "gain_writes", gain_writes);
        cJSON_AddNumberToObject(out, "last_path", last_path);
        cJSON_AddStringToObject(out, "last_path_name",
                                last_path == 1 ? "EXP_VAL" :
                                last_path == 2 ? "EXP_US" :
                                last_path == 3 ? "GAIN" :
                                last_path == 4 ? "GROUP" : "NONE");
        cJSON_AddNumberToObject(out, "suppressed", suppressed);
        cJSON_AddBoolToObject(out, "gain_force_active", gain_force_active);
        cJSON_AddNumberToObject(out, "gain_force_value", gain_force_value);
        cJSON_AddNumberToObject(out, "gain_force_left_ms", gain_force_left_ms);
        cJSON_AddNumberToObject(out, "gain_force_suppressed", gain_force_suppressed);
        bool user_gain_manual = false;
        uint32_t user_gain_value = 0U;
        esp_video_isp_pipeline_user_gain_get(&user_gain_manual, &user_gain_value);
        cJSON_AddBoolToObject(out, "user_gain_manual", user_gain_manual);
        cJSON_AddNumberToObject(out, "user_gain_value", user_gain_value);
        esp_err_t e = send_json(req, "200 OK", out);
        cJSON_Delete(out);
        return e;
    }

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

    camera_gain_load_once();
    const int gain_i = camera_gain_profile_index(profile);
    bool gain_update = false;
    bool requested_gain_manual = s_camera_gain.live_manual[gain_i];
    uint32_t requested_gain_value = s_camera_gain.live_value[gain_i];
    const cJSON *jgain_manual = cJSON_GetObjectItem(in, "gain_manual");
    const cJSON *jgain_value = cJSON_GetObjectItem(in, "gain_value");
    if (jgain_manual || jgain_value) {
        if (!cJSON_IsBool(jgain_manual) || !cJSON_IsNumber(jgain_value)) {
            cJSON_Delete(in);
            return send_err(req, "400 Bad Request", "missing_gain_values");
        }
        requested_gain_manual = cJSON_IsTrue(jgain_manual);
        if (jgain_value->valuedouble < 0.0 ||
            jgain_value->valuedouble > (double)camera_gain_profile_max(profile)) {
            cJSON_Delete(in);
            return send_err(req, "400 Bad Request", "gain_out_of_range");
        }
        requested_gain_value = (uint32_t)jgain_value->valuedouble;
        gain_update = true;
    }

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

    if (gain_update) {
        esp_err_t gain_err = camera_gain_preview_set(profile,
                                                     requested_gain_manual,
                                                     requested_gain_value);
        if (gain_err == ESP_ERR_INVALID_ARG) {
            return send_err(req, "400 Bad Request", "gain_out_of_range");
        }
        if (gain_err != ESP_OK) {
            return send_err(req, "503 Service Unavailable", "gain_apply_failed");
        }
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

        esp_err_t gain_save_err = camera_gain_save_profile(profile);
        if (gain_save_err != ESP_OK) {
            ESP_LOGE(TAG, "[CAM-USER-GAIN-R23] NVS save FAILED profile=%s err=%s",
                     profile == CAMERA_H264_PROFILE_720P_NATIVE ? "720p60" : "1080p",
                     esp_err_to_name(gain_save_err));
            return send_err(req, "503 Service Unavailable", "gain_nvs_save_failed");
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

/* POST /api/smoke/stop ???성 ?션 ?리 */
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
/* [CAM-1080-ADAPT-GOP-R2-20260912] ---------- ESP-authoritative 1080 Adaptive GOP ---------- */
static const char *smoke_adapt_state_name(adapt_gop_state_t st)
{
    switch (st) {
    case ADAPT_GOP5_STABLE: return "LTE";
    case ADAPT_GOP1_STABLE: return "GOOD";
    case ADAPT_PROBING_GOP1: return "PROBING_GOOD";
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
    if (target_gop != 5U && target_gop != 15U) return ESP_ERR_INVALID_ARG;
    if (camera_h264_stream_get_profile() != CAMERA_H264_PROFILE_1080P) return ESP_ERR_NOT_SUPPORTED;
    const uint32_t old_gop = camera_h264_stream_get_gop();
    const uint32_t old_qp = camera_h264_stream_get_qp();
    const uint32_t target_qp = (target_gop == 5U) ? 30U : 32U;
    if (old_qp == target_qp && old_gop == target_gop) return ESP_OK;
    if (!camera_h264_stream_is_active()) return ESP_ERR_INVALID_STATE;

    SMKW("[1080-NET-PROFILE] switch reason=%s qp/gop %u/%u -> %u/%u viewers=%u",
         reason ? reason : "?", (unsigned)old_qp, (unsigned)old_gop,
         (unsigned)target_qp, (unsigned)target_gop, (unsigned)smoke_fanout_count());

    esp_err_t gr = camera_h264_stream_set_1080_net_profile(target_qp, target_gop);
    if (gr != ESP_OK) {
        SMKE("[1080-NET-PROFILE] live switch qp/gop %u/%u -> %u/%u failed=%s; actual=%u/%u",
             (unsigned)old_qp, (unsigned)old_gop, (unsigned)target_qp, (unsigned)target_gop,
             esp_err_to_name(gr), (unsigned)camera_h264_stream_get_qp(), (unsigned)camera_h264_stream_get_gop());
        return gr;
    }

    if (camera_h264_stream_get_qp() != target_qp || camera_h264_stream_get_gop() != target_gop) {
        SMKE("[1080-NET-PROFILE] live switch ACK mismatch target=%u/%u actual=%u/%u",
             (unsigned)target_qp, (unsigned)target_gop, (unsigned)camera_h264_stream_get_qp(), (unsigned)camera_h264_stream_get_gop());
        return ESP_FAIL;
    }

    s_adapt_switch_count++;
    smoke_adapt_clear_windows_locked();
    SMK("[1080-NET-PROFILE] switch PASS state=%s qp=%u gop=%u no camera/H264 restart; peer/ICE/DTLS/audio/PTS kept",
        (target_qp == 30U) ? "GOOD" : "LTE", (unsigned)target_qp, (unsigned)target_gop);
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
        const bool strong_avail_hint = ps->adapt_avail_kbps >= ADAPT_GOOD_HEADROOM_KBPS;
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
    esp_err_t rr = smoke_adapt_switch_locked(15U, reason);
    if (rr != ESP_OK) {
        /* Fail closed without lying about current encoder state. */
        s_adapt_state = ADAPT_GOP1_STABLE;
        s_adapt_probe_until_ms = 0;
        s_adapt_cooldown_until_ms = now_ms + 30000LL;
        SMKE("[CAM-1080-ADAPT-GOP-R2-20260912] probe fallback to LTE failed=%s; holding actual GOOD profile", esp_err_to_name(rr));
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
        if (joined && camera_h264_stream_get_gop() == 5U) {
            if (smoke_adapt_switch_locked(15U, "viewer_join") == ESP_OK) {
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
        if (s_adapt_resume_state == ADAPT_PROBING_GOP1 && camera_h264_stream_get_gop() == 5U) {
            s_adapt_state = ADAPT_PROBING_GOP1;
            s_adapt_probe_until_ms = now_ms + ADAPT_PROBE_MS;
            smoke_adapt_clear_windows_locked();
            SMK_WEBRTC_INFO("[CAM-1080-ADAPT-GOP-R2-20260912] network freeze ended -> resume GOOD probe");
        } else {
            s_adapt_state = (camera_h264_stream_get_gop() == 5U) ?
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
                SMK("[CAM-1080-ADAPT-GOP-R2-20260912] GOOD probe PASS -> GOOD");
            } else {
                smoke_adapt_fail_probe_locked("probe_failed", now_ms);
            }
        }
        return;
    }

    if (gop == 5U) {
        s_adapt_state = ADAPT_GOP1_STABLE;
        if (smoke_adapt_any_congested_locked(&cs, now_ms, false)) {
            if (smoke_adapt_switch_locked(15U, "congested_3of4") == ESP_OK) {
                s_adapt_state = ADAPT_GOP5_STABLE;
                s_adapt_cooldown_until_ms = now_ms + ADAPT_COOLDOWN_MS;
            }
        }
        return;
    }

    s_adapt_state = ADAPT_GOP5_STABLE;
    if (now_ms >= s_adapt_cooldown_until_ms && smoke_adapt_all_headroom_locked(&cs, now_ms)) {
        if (smoke_adapt_switch_locked(5U, "headroom_probe") == ESP_OK) {
            s_adapt_state = ADAPT_PROBING_GOP1;
            s_adapt_probe_until_ms = now_ms + ADAPT_PROBE_MS;
            SMK_WEBRTC_INFO("[CAM-1080-ADAPT-GOP-R2-20260912] GOOD probe start duration=%lldms", (long long)ADAPT_PROBE_MS);
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
     * the current low-rate LTE stream.  Clean delivery + low ESP send latency
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
        SMK_WEBRTC_INFO("[CAM-1080-ADAPT-GOP-R2-20260912] sid=%u metrics state=%s gop=%u browser=%u.%02u enc=%u.%02u ratio=%u "
            "loss=%u.%02u%% jitter=%ums avail=%uk(%s) tx=%uk rx=%uk send_ema=%uus hwm=%u/%u drop=%u hist=0x%x good=%u",
            (unsigned)sid, smoke_adapt_state_name(s_adapt_state), (unsigned)camera_h264_stream_get_gop(),
            (unsigned)(browser_fps_x100 / 100U), (unsigned)(browser_fps_x100 % 100U),
            (unsigned)(cs.encode_fps_x100 / 100U), (unsigned)(cs.encode_fps_x100 % 100U),
            (unsigned)ratio, (unsigned)(ps->adapt_loss_ema_bp / 100U),
            (unsigned)(ps->adapt_loss_ema_bp % 100U), (unsigned)ps->adapt_jitter_ema_ms,
            (unsigned)ps->adapt_avail_kbps, ps->adapt_avail_kbps ? "known" : "unknown",
            (unsigned)cs.h264_tx_kbps, (unsigned)ps->adapt_rx_kbps, (unsigned)ps->adapt_send_ema_us,
            (unsigned)hwm_window, (unsigned)SMOKE_TX_QUEUE_DEPTH,
            (unsigned)drop_delta, (unsigned)ps->adapt_bad_hist, (unsigned)ps->adapt_good_streak);
        ps->adapt_last_log_ms = now_ms;
    }

    smoke_adapt_evaluate_locked(0U);
}


/* [FLIGHT-RECORDER-STAGE2-R1-20260916]
 * Reuse the existing /api/smoke/status GET instead of consuming two more
 * httpd URI-handler slots.  This is deliberate: the current server was sized
 * tightly, and expanding max_uri_handlers could add internal-heap pressure. */
static void smoke_fr_add_status_json(cJSON *out)
{
    if (!out) return;
    flight_recorder_status_t st;
    flight_recorder_get_status(&st);
    cJSON *fr = cJSON_CreateObject();
    if (!fr) return;
    cJSON_AddBoolToObject(fr, "enabled", st.enabled);
    cJSON_AddBoolToObject(fr, "flash_ready", st.flash_ready);
    cJSON_AddBoolToObject(fr, "pending", st.pending);
    cJSON_AddBoolToObject(fr, "frozen", st.frozen);
    cJSON_AddBoolToObject(fr, "saved", st.saved_valid);
    cJSON_AddNumberToObject(fr, "saved_seq", st.saved_seq);
    cJSON_AddNumberToObject(fr, "saved_trigger", st.saved_trigger);
    cJSON_AddStringToObject(fr, "saved_trigger_name",
                            flight_recorder_event_name((uint16_t)st.saved_trigger));
    cJSON_AddNumberToObject(fr, "saved_trigger_ms", st.saved_trigger_ms);
    cJSON_AddNumberToObject(fr, "saved_records", st.saved_records);
    cJSON_AddNumberToObject(fr, "ring_records", st.ring_records);
    cJSON_AddNumberToObject(fr, "dropped_frozen", st.dropped_while_frozen);
    cJSON_AddNumberToObject(fr, "last_save_result", st.last_save_result);
    cJSON_AddNumberToObject(fr, "partition_size", st.partition_size);
    cJSON_AddItemToObject(out, "flight_recorder", fr);
}

static void smoke_fr_add_compact_status_json(cJSON *out)
{
    if (!out) return;
    flight_recorder_status_t st;
    flight_recorder_get_status(&st);
    unsigned flags = 0U;
    if (st.enabled) flags |= 1U;
    if (st.flash_ready) flags |= 2U;
    if (st.pending) flags |= 4U;
    if (st.frozen) flags |= 8U;
    if (st.saved_valid) flags |= 16U;
    char buf[96];
    snprintf(buf, sizeof(buf), "%u,%u,%u,%u,%u,%ld",
             flags, (unsigned)st.saved_seq, (unsigned)st.saved_trigger,
             (unsigned)st.saved_records, (unsigned)st.ring_records,
             (long)st.last_save_result);
    cJSON_AddStringToObject(out, "fr", buf);
}

static bool smoke_fr_tag_valid(const char *tag)
{
    if (!tag || !tag[0]) return false;
    size_t n = strlen(tag);
    if (n > 10U) return false;
    for (size_t i = 0; i < n; ++i) {
        const unsigned char c = (unsigned char)tag[i];
        if (!(isalnum(c) || c == '_' || c == '-')) return false;
    }
    return true;
}

static esp_err_t smoke_fr_send_action_status(httpd_req_t *req, esp_err_t action)
{
    cJSON *out = cJSON_CreateObject();
    if (!out) return send_err(req, "503 Service Unavailable", "oom");
    cJSON_AddBoolToObject(out, "ok", action == ESP_OK);
    cJSON_AddNumberToObject(out, "result", action);
    smoke_fr_add_status_json(out);
    esp_err_t e = send_json(req, action == ESP_OK ? "200 OK" : "500 Internal Server Error", out);
    cJSON_Delete(out);
    return e;
}

static void smoke_add_float9_csv(cJSON *obj, const char *name,
                                 const float values[9])
{
    /* [CAM-AUTOCAL-SPATIAL-R1-20260918]
     * A compact string avoids creating 27 extra cJSON number nodes in the
     * status path. The browser parses these three fixed nine-value fields. */
    if (!obj || !name || !values) return;
    char csv[128];
    size_t used = 0;
    csv[0] = '\0';
    for (size_t i = 0; i < 9U && used < sizeof(csv); ++i) {
        int n = snprintf(csv + used, sizeof(csv) - used,
                         i ? ",%.3f" : "%.3f", (double)values[i]);
        if (n < 0 || (size_t)n >= sizeof(csv) - used) return;
        used += (size_t)n;
    }
    cJSON_AddStringToObject(obj, name, csv);
}


/* [FACE-AI-SCAN-GEOM-DUMP-R21-20261001]
 * Reuse the already-registered /api/smoke/status URI: this server is already
 * sized tightly for URI handlers, so diagnostics MUST NOT consume another slot.
 * Special requests:
 *   ?ai_geom=new|low|old   switch SCAN geometry only
 *   ?ai_capture=1      arm one-shot full-width calibration snapshot
 *   ?ai_frame=1        download last snapshot as planar YUV420 160x120
 *   ?ai_diag=1         compact diagnostic JSON
 * Normal status polling is untouched when no ai_* key is present.
 */
static bool smoke_ai_diag_try_handle(httpd_req_t *req,
                                     const char *qbuf,
                                     esp_err_t *ret_out)
{
    if (!req || !qbuf || !ret_out) return false;

    char geom[12] = {0};
    char capture[8] = {0};
    char frame[8] = {0};
    char diag[8] = {0};
    const bool have_geom =
        httpd_query_key_value(qbuf, "ai_geom", geom, sizeof(geom)) == ESP_OK;
    const bool have_capture =
        httpd_query_key_value(qbuf, "ai_capture", capture, sizeof(capture)) == ESP_OK;
    const bool have_frame =
        httpd_query_key_value(qbuf, "ai_frame", frame, sizeof(frame)) == ESP_OK;
    const bool have_diag =
        httpd_query_key_value(qbuf, "ai_diag", diag, sizeof(diag)) == ESP_OK;

    if (!have_geom && !have_capture && !have_frame && !have_diag) return false;

    if (have_geom) {
        if (strcmp(geom, "new") == 0 || strcmp(geom, "160") == 0) {
            face_ai_tap_diag_set_scan_geometry(FACE_AI_SCAN_GEOM_NEW160);
        } else if (strcmp(geom, "low") == 0 || strcmp(geom, "160") == 0 || strcmp(geom, "mid") == 0 || strcmp(geom, "192") == 0 || strcmp(geom, "wide") == 0 || strcmp(geom, "224") == 0) {
            face_ai_tap_diag_set_scan_geometry(FACE_AI_SCAN_GEOM_WIDE224);
        } else if (strcmp(geom, "old") == 0) {
            face_ai_tap_diag_set_scan_geometry(FACE_AI_SCAN_GEOM_OLD);
        } else {
            *ret_out = send_err(req, "400 Bad Request", "bad_ai_geom");
            return true;
        }
    }

    if (have_capture && strcmp(capture, "1") == 0) {
        if (!face_ai_tap_diag_request_capture()) {
            *ret_out = send_err(req, "503 Service Unavailable", "ai_capture_alloc_failed");
            return true;
        }
    }

    if (have_frame && strcmp(frame, "1") == 0) {
        const size_t bytes = 160U * 120U * 3U / 2U; /* R28 max: FOLLOW remains 160x120 */
        uint8_t *tmp = (uint8_t *)heap_caps_malloc(
            bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!tmp) {
            *ret_out = send_err(req, "503 Service Unavailable", "ai_frame_tmp_alloc_failed");
            return true;
        }

        face_ai_tap_diag_frame_meta_t m = {0};
        if (!face_ai_tap_diag_copy_frame(tmp, bytes, &m)) {
            heap_caps_free(tmp);
            *ret_out = send_err(req, "409 Conflict", "ai_frame_not_ready");
            return true;
        }

        char h_seq[16];
        char h_src[32];
        char h_frame[24];
        char h_active[40];
        snprintf(h_seq, sizeof(h_seq), "%u", (unsigned)m.seq);
        snprintf(h_src, sizeof(h_src), "%ux%u", (unsigned)m.src_w, (unsigned)m.src_h);
        snprintf(h_frame, sizeof(h_frame), "%ux%u", (unsigned)m.frame_w, (unsigned)m.frame_h);
        snprintf(h_active, sizeof(h_active), "%u,%u,%u,%u",
                 (unsigned)m.active_x, (unsigned)m.active_y,
                 (unsigned)m.active_w, (unsigned)m.active_h);

        httpd_resp_set_type(req, "application/octet-stream");
        httpd_resp_set_hdr(req, "Cache-Control", "no-store");
        httpd_resp_set_hdr(req, "X-AI-Seq", h_seq);
        httpd_resp_set_hdr(req, "X-AI-Source", h_src);
        httpd_resp_set_hdr(req, "X-AI-Frame", h_frame);
        httpd_resp_set_hdr(req, "X-AI-Active", h_active);
        const char *gname = (m.geometry_mode == FACE_AI_SCAN_GEOM_WIDE224)
                                ? "low160x90"
                                : (m.geometry_mode == FACE_AI_SCAN_GEOM_NEW160)
                                    ? "new160x90" : "old_r10a1";
        httpd_resp_set_hdr(req, "X-AI-Geometry", gname);
        httpd_resp_set_hdr(req, "X-AI-Format", "YUV420P-Y-U-V");

        const size_t actual_bytes=(size_t)m.frame_w*m.frame_h*3U/2U;
        esp_err_t ret = ESP_OK;
        size_t off = 0;
        while (off < actual_bytes) {
            size_t n = actual_bytes - off;
            if (n > 1024U) n = 1024U;
            ret = httpd_resp_send_chunk(req, (const char *)tmp + off, n);
            if (ret != ESP_OK) break;
            off += n;
            if ((off & 4095U) == 0U && off < bytes) vTaskDelay(1);
        }
        if (ret == ESP_OK) ret = httpd_resp_send_chunk(req, NULL, 0);
        heap_caps_free(tmp);
        *ret_out = ret;
        return true;
    }

    cJSON *o = cJSON_CreateObject();
    if (!o) {
        *ret_out = send_err(req, "503 Service Unavailable", "json_alloc_failed");
        return true;
    }
    cJSON_AddBoolToObject(o, "ok", true);
    cJSON_AddStringToObject(o, "scan_geometry",
                           face_ai_tap_diag_scan_geometry_name());
    cJSON_AddNumberToObject(o, "capture_seq",
                           (double)face_ai_tap_diag_capture_seq());
    cJSON_AddNumberToObject(o, "frame_bytes",
                           (double)face_ai_tap_diag_frame_bytes());
    cJSON_AddStringToObject(o, "frame_format", "YUV420P-Y-U-V");
    *ret_out = send_json(req, "200 OK", o);
    cJSON_Delete(o);
    return true;
}

/* [FACE-OWNER-R1-20261015]
 * OWNER evidence for the browser label, piggybacked on the EXISTING status
 * heartbeat (no new URI/poll). Format in face_ai_detect.h:
 * "face_own": owner,box_seq,pass_seq,sim_x1000,pass_age_ms,why,passes,fails,breaks
 * Read-only snapshot; separate noinline helper keeps the frame out of the caller. */
static __attribute__((noinline)) void smoke_status_add_face_own(cJSON *out)
{
    char s[96];
    s[0] = '\0';
    if (face_ai_detect_owner_compact(s, sizeof(s)) <= 0 || s[0] == '\0') {
        cJSON_AddStringToObject(out, "face_own", "0");
        return;
    }
    cJSON_AddStringToObject(out, "face_own", s);
}

/* [FACE-OWNER-ORQ-R1-20261016] Browser hybrid OWNER (?ftbownhyb=1) one-shot MFN
 * re-verification, carried on the EXISTING status GET (no new URI/poll):
 *   request : "orq=<8 hex, non-zero>"   (sent only while that page waits for an answer)
 *   reply   : "face_orq": "<ack>,<face_ai_detect_owner_request_compact>"
 *             ack '-' = no request in this GET, else Q/D/B/R (see face_ai_detect.h).
 * Accepted ONLY from a page whose client token currently owns a camera session
 * (self_sid != 0, checked under s_gate by the caller). Results are reported only to
 * the same token hash, so another client can neither inject nor borrow a result.
 * This handler only queues; MFN runs later in the AI task through its normal gates.
 * Pages that never send orq (all default URLs) get no "face_orq" field at all. */
static __attribute__((noinline)) void smoke_status_add_face_orq(cJSON *out, const char *qbuf,
                                                                const char *client_token,
                                                                bool client_token_ok,
                                                                uint32_t self_sid)
{
    if (!out || !client_token_ok || !client_token || !client_token[0]) return;
    const uint32_t ch = smoke_diag_hash32(client_token);
    char ack = '-';
    char rid_txt[12] = {0};
    if (qbuf && qbuf[0] &&
        httpd_query_key_value(qbuf, "orq", rid_txt, sizeof(rid_txt)) == ESP_OK) {
        bool hex_ok = (strlen(rid_txt) == 8U);
        for (size_t i = 0; hex_ok && i < 8U; ++i) {
            if (!isxdigit((unsigned char)rid_txt[i])) hex_ok = false;
        }
        const uint32_t rid = hex_ok ? (uint32_t)strtoul(rid_txt, NULL, 16) : 0U;
        if (!hex_ok || rid == 0U || self_sid == 0U || ch == 0U) {
            ack = 'R';
        } else {
            ack = face_ai_detect_owner_request(rid, ch);
        }
    }
    char body[80];
    body[0] = '\0';
    (void)face_ai_detect_owner_request_compact(ch, body, sizeof(body));
    if (ack == '-' && (body[0] == '\0' || !strcmp(body, "0"))) return; /* nothing for this page */
    char s[96];
    snprintf(s, sizeof(s), "%c,%s", ack, body[0] ? body : "0");
    cJSON_AddStringToObject(out, "face_orq", s);
}

/* [WEB-SFACE-MFN-SEP-R1-20261019] read-only proof that the web camera does not use ESP
 * MobileFaceNet: role, MFN load/inference/ORQ/OWNER-verdict counters (format in
 * face_ai_detect.h). Same EXISTING status heartbeat, no new URI/poll, no decision. */
static __attribute__((noinline)) void smoke_status_add_face_sep(cJSON *out)
{
    char s[160];
    s[0] = '\0';
    if (face_ai_detect_mfn_sep_compact(s, sizeof(s)) <= 0 || s[0] == '\0') {
        cJSON_AddStringToObject(out, "face_sep", "0");
        return;
    }
    cJSON_AddStringToObject(out, "face_sep", s);
}

/* [FACE-TRACK-DIAG-R2-20261007]
 * Diagnostic piggyback on the EXISTING status heartbeat (no new URI/poll).
 * "face_dg": follow,scan,pub,hold,n0,sub,mfn,mfn_ms,bootv,
 *            hint_valid,hx0,hy0,hx1,hy1,hint_age_ms
 * Separate noinline helper so its ~0.2 KB frame exists only during the call. */
static __attribute__((noinline)) void smoke_status_add_face_dg(cJSON *out)
{
    face_ai_track_diag_t dg = {0};
    face_ai_search_hint_t hn = {0};
    char s[192];
    if (!face_ai_detect_get_track_diag(&dg)) {
        cJSON_AddStringToObject(out, "face_dg", "0");
        return;
    }
    const bool hv = face_ai_detect_get_search_hint(&hn) && hn.valid;
    snprintf(s, sizeof(s), "%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%u,%u,%ld,%ld,%ld,%ld,%lu",
             (unsigned long)dg.follow, (unsigned long)dg.scan, (unsigned long)dg.pub,
             (unsigned long)dg.hold, (unsigned long)dg.n0, (unsigned long)dg.sub,
             (unsigned long)dg.mfn, (unsigned long)dg.mfn_ms, (unsigned)dg.bootv,
             hv ? 1U : 0U,
             (long)(hv ? hn.x0 : 0), (long)(hv ? hn.y0 : 0),
             (long)(hv ? hn.x1 : 0), (long)(hv ? hn.y1 : 0),
             (unsigned long)(hv ? hn.age_ms : 0U));
    cJSON_AddStringToObject(out, "face_dg", s);
}

/* [FACE-EV-R3C1-20261008]
 * ESP spatial evidence for the browser FT2 validator, piggybacked on the EXISTING
 * status heartbeat (no new URI, no new poll, no socket). Two compact strings:
 * "face_ev"  : v,run,seq,path,hits,pb,src,span_ms,step_x100,cap_age_ms,
 *              p_valid,px0,py0,px1,py1,p_age_ms,p_cap_age_ms,p_score,p_src  | "0"
 * "face_hist": seq,x0,y0,x1,y1,score,cap_age_ms,src[;...]  (newest first, <=3) | "0"
 * One reused 192-byte stack buffer, only during this noinline call; cJSON copies. */
static __attribute__((noinline)) void smoke_status_add_face_ev(cJSON *out)
{
    face_ai_evidence_t ev;
    char s[192];
    if (!face_ai_detect_get_evidence(&ev) || !ev.valid) {
        cJSON_AddStringToObject(out, "face_ev", "0");
        cJSON_AddStringToObject(out, "face_hist", "0");
        return;
    }
    snprintf(s, sizeof(s),
             "1,%lu,%lu,%c,%u,%u,%c,%lu,%u,%lu,%u,%ld,%ld,%ld,%ld,%lu,%lu,%u,%c",
             (unsigned long)ev.run, (unsigned long)ev.seq,
             ev.path ? ev.path : '-', (unsigned)ev.hits, (unsigned)ev.pb,
             ev.src ? ev.src : '-',
             (unsigned long)ev.span_ms, (unsigned)ev.step_x100,
             (unsigned long)ev.cap_age_ms,
             ev.p_valid ? 1U : 0U,
             (long)ev.px0, (long)ev.py0, (long)ev.px1, (long)ev.py1,
             (unsigned long)ev.p_age_ms, (unsigned long)ev.p_cap_age_ms,
             (unsigned)ev.p_score_x1000, ev.p_src ? ev.p_src : '-');
    cJSON_AddStringToObject(out, "face_ev", s);
    size_t n = 0;
    s[0] = '\0';
    for (unsigned i = 0; i < (unsigned)ev.hist_n && i < 3U; ++i) {
        const face_ai_ev_hist_t *h = &ev.hist[i];
        const int w = snprintf(s + n, sizeof(s) - n, "%s%lu,%ld,%ld,%ld,%ld,%u,%lu,%c",
                               i ? ";" : "",
                               (unsigned long)h->seq, (long)h->x0, (long)h->y0,
                               (long)h->x1, (long)h->y1, (unsigned)h->score_x1000,
                               (unsigned long)h->cap_age_ms, h->src ? h->src : '-');
        if (w < 0 || (size_t)w >= sizeof(s) - n) break;   /* never emit a truncated entry */
        n += (size_t)w;
    }
    if (n == 0) { s[0] = '0'; s[1] = '\0'; }
    else s[n] = '\0';
    cJSON_AddStringToObject(out, "face_hist", s);
}

/* [FACE-MOT-R4-20261008]
 * SHADOW motion tracker snapshot on the EXISTING status heartbeat (no new URI/poll).
 * "face_mot": 1,aseq,tseq,age_ms,frame_age_ms,ncc_x100,x0,y0,x1,y1   (valid estimate)
 *           | 0,why,aseq,age_ms                                      (no valid estimate)
 * Measurement only; the browser logs it and never displays it in R4-1. */
static __attribute__((noinline)) void smoke_status_add_face_mot(cJSON *out)
{
    face_ai_mot_t mt;
    char s[112];
    const bool ok = face_ai_tap_get_mot(&mt);
    if (ok && mt.valid) {
        snprintf(s, sizeof(s), "1,%lu,%lu,%lu,%lu,%d,%ld,%ld,%ld,%ld",
                 (unsigned long)mt.aseq, (unsigned long)mt.tseq,
                 (unsigned long)mt.age_ms, (unsigned long)mt.frame_age_ms,
                 (int)mt.ncc_x100,
                 (long)mt.x0, (long)mt.y0, (long)mt.x1, (long)mt.y1);
    } else {
        const char w = (mt.why >= '!' && mt.why <= '~') ? mt.why : '-';
        snprintf(s, sizeof(s), "0,%c,%lu,%lu", (mt.alive && w == '-') ? 'e' : w,
                 (unsigned long)mt.aseq, (unsigned long)mt.age_ms);
    }
    cJSON_AddStringToObject(out, "face_mot", s);
}

static esp_err_t h_status_get(httpd_req_t *req)
{

    if (!s_dma_r8_first_status_seen) {
        s_dma_r8_first_status_seen = true;
        smoke_dma_r8_phase_log("STATUS0 first_status_entry");
    }
    /* Callback-origin triggers are flushed by the already-existing HTTP task;
     * no new FreeRTOS task/stack/semaphore is created by the recorder. */
    (void)flight_recorder_service();
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
    char status_client_token[SMOKE_CLIENT_TOKEN_HEX_LEN + 1U] = {0};
    bool status_client_token_ok = false;
    bool viewer_hb_present = false, viewer_hb_valid = false, viewer_hb_visible = false;
    uint32_t viewer_hb_sid = 0U, viewer_hb_frames = 0U;
    char viewer_hb_page[SMOKE_DIAG_ID_HEX_LEN + 1U] = {0};
    /* [CAM-1080-ADAPT-GOP-R2-20260912] Compact browser report carried on the existing status GET. */
    bool adapt_report_present = false;
    bool adapt_report_valid = false;
    bool adapt_report_freeze = false;
    uint32_t adapt_freeze_ms = 0;
    uint32_t adapt_sid = 0, adapt_bfps_x100 = 0, adapt_loss_bp = 0;
    uint32_t adapt_jitter_ms = 0, adapt_avail_kbps = 0, adapt_rx_kbps = 0;
    if (httpd_req_get_url_query_str(req, qbuf, sizeof(qbuf)) == ESP_OK) {
        esp_err_t ai_diag_ret = ESP_OK;
        if (smoke_ai_diag_try_handle(req, qbuf, &ai_diag_ret)) return ai_diag_ret;

        if (httpd_query_key_value(qbuf, "ct", status_client_token,
                                  sizeof(status_client_token)) == ESP_OK) {
            status_client_token_ok = smoke_client_token_valid(status_client_token);
            if (!status_client_token_ok) status_client_token[0] = '\0';
        }

        /* [CAM-VIEWER-HEARTBEAT-R1-20260918]
         * Compact heartbeat carried on the existing status GET:
         * vhs=session id, vhp=page id, vh=1 visible/0 hidden,
         * vhf=browser framesDecoded. */
        char hb_sid_txt[16] = {0}, hb_state_txt[4] = {0}, hb_frames_txt[16] = {0};
        if (httpd_query_key_value(qbuf, "vh", hb_state_txt, sizeof(hb_state_txt)) == ESP_OK) {
            viewer_hb_present = true;
            char *sid_end = NULL, *frames_end = NULL;
            const unsigned long sid_v =
                (httpd_query_key_value(qbuf, "vhs", hb_sid_txt, sizeof(hb_sid_txt)) == ESP_OK)
                    ? strtoul(hb_sid_txt, &sid_end, 10) : 0UL;
            const unsigned long frames_v =
                (httpd_query_key_value(qbuf, "vhf", hb_frames_txt, sizeof(hb_frames_txt)) == ESP_OK)
                    ? strtoul(hb_frames_txt, &frames_end, 10) : 0UL;
            const bool page_ok =
                httpd_query_key_value(qbuf, "vhp", viewer_hb_page,
                                      sizeof(viewer_hb_page)) == ESP_OK &&
                smoke_diag_id_valid(viewer_hb_page);
            const bool state_ok = !strcmp(hb_state_txt, "0") || !strcmp(hb_state_txt, "1");
            viewer_hb_visible = !strcmp(hb_state_txt, "1");
            viewer_hb_valid = status_client_token_ok && page_ok && state_ok &&
                sid_end && *sid_end == '\0' && sid_v > 0UL && sid_v <= UINT32_MAX &&
                frames_end && *frames_end == '\0' && frames_v <= UINT32_MAX;
            if (viewer_hb_valid) {
                viewer_hb_sid = (uint32_t)sid_v;
                viewer_hb_frames = (uint32_t)frames_v;
            }
        }

        /* [CAM-SHARED-OP-COORD-R6-20260916]
         * Reuse the existing status URI for two cooperative operations:
         *   kind=cal     : 720 auto calibration
         *   kind=profile : shared 1080p/720p60 mode switch
         * R22A additionally lets the calibration owner perform an owner-only
         * in-lease profile apply while keeping kind=cal for the whole sequence.
         * Browsers still close ONLY their own session. The server lock merely
         * serializes the shared-camera owner and blocks foreign replacement
         * offers until the operation ends. */
        /* [CAM-AUTOCAL-CALOP-BUF-R22A1-20260920]
         * Longest command is "apply_cal_profile" (17 chars + NUL).
         * 16 bytes made httpd_query_key_value() reject/truncate the key value,
         * so the request fell through as a plain status GET and the browser
         * interpreted calibration.active/owner as false ("ownership lost"). */
        char cal_op[24] = {0};
        if (httpd_query_key_value(qbuf, "cal", cal_op, sizeof(cal_op)) == ESP_OK) {
            char cal_token[SMOKE_CLIENT_TOKEN_HEX_LEN + 1U] = {0};
            bool have_token =
                httpd_query_key_value(qbuf, "ct", cal_token, sizeof(cal_token)) == ESP_OK;
            if (have_token && !smoke_client_token_valid(cal_token))
                return send_err(req, "400 Bad Request", "bad_client_token");

            /* [CAM-AUTOCAL-OWNER-COUNT-FIX-R1-20260918]
             * Browser supplies its current sid when it has one. It is only a
             * hint until validated against the exact client token under s_gate. */
            char cal_sid_txt[16] = {0};
            uint32_t cal_requested_sid = 0U;
            if (httpd_query_key_value(qbuf, "sid", cal_sid_txt,
                                      sizeof(cal_sid_txt)) == ESP_OK) {
                char *endp = NULL;
                unsigned long v = strtoul(cal_sid_txt, &endp, 10);
                if (endp && *endp == '\0' && v > 0UL && v <= UINT32_MAX)
                    cal_requested_sid = (uint32_t)v;
            }

            bool requested_profile_switch = false;
            camera_h264_profile_t requested_target = CAMERA_H264_PROFILE_720P_NATIVE;
            if (strcmp(cal_op, "begin") == 0) {
                char kind_txt[16] = {0};
                if (httpd_query_key_value(qbuf, "kind", kind_txt, sizeof(kind_txt)) == ESP_OK) {
                    if (strcmp(kind_txt, "profile") == 0) {
                        requested_profile_switch = true;
                    } else if (strcmp(kind_txt, "cal") != 0) {
                        return send_err(req, "400 Bad Request", "bad_camera_operation_kind");
                    }
                }

                if (requested_profile_switch) {
                    char target_txt[32] = {0};
                    if (httpd_query_key_value(qbuf, "target", target_txt,
                                              sizeof(target_txt)) != ESP_OK) {
                        return send_err(req, "400 Bad Request", "missing_target_profile");
                    }
                    const bool target_720 =
                        !strcasecmp(target_txt, "720") ||
                        !strcasecmp(target_txt, "720p") ||
                        !strcasecmp(target_txt, "720p-highspeed") ||
                        !strcasecmp(target_txt, "720p60") ||
                        !strcasecmp(target_txt, "720p60-native") ||
                        !strcasecmp(target_txt, "1288x728");
                    const bool target_1080 =
                        !strcasecmp(target_txt, "1080") ||
                        !strcasecmp(target_txt, "1080p") ||
                        !strcasecmp(target_txt, "1920x1080");
                    if (!target_720 && !target_1080) {
                        return send_err(req, "400 Bad Request", "bad_target_profile");
                    }
                    requested_target = target_720
                        ? CAMERA_H264_PROFILE_720P_NATIVE
                        : CAMERA_H264_PROFILE_1080P;
                }
            }

            if (xSemaphoreTake(s_gate, pdMS_TO_TICKS(500)) != pdTRUE)
                return send_err(req, "409 Conflict", "session_gate_busy");

            smoke_auto_cal_expire_locked();
            const int64_t now_ms = esp_timer_get_time() / 1000LL;
            const char *cal_error = NULL;
            const char *cal_error_status = "409 Conflict";

            if (strcmp(cal_op, "begin") == 0) {
                if (!have_token) {
                    cal_error = "bad_client_token";
                    cal_error_status = "400 Bad Request";
                } else if (s_auto_cal_active && !smoke_auto_cal_owner_locked(cal_token)) {
                    cal_error = "camera_operation_busy";
                } else {
                    if (!s_auto_cal_active) {
                        s_auto_cal_active = true;
                        s_auto_cal_profile_switch = requested_profile_switch;
                        s_auto_cal_target_profile = requested_target;
                        snprintf(s_auto_cal_owner_token, sizeof(s_auto_cal_owner_token),
                                 "%s", cal_token);
                        s_auto_cal_generation++;
                        if (s_auto_cal_generation == 0U) s_auto_cal_generation = 1U;
                        s_camop_diag_base_free = heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
                        s_camop_diag_base_largest = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
                        s_camop_diag_generation = s_auto_cal_generation;
                        SMK_DMA_DIAG("[DMA-LIFECYCLE-DIAG-R1] BEGIN kind=%s target=%s gen=%u free=%u largest=%u",
                             s_auto_cal_profile_switch ? "profile" : "cal",
                             s_auto_cal_target_profile == CAMERA_H264_PROFILE_720P_NATIVE
                                 ? "720p60" : "1080p",
                             (unsigned)s_auto_cal_generation,
                             (unsigned)s_camop_diag_base_free,
                             (unsigned)s_camop_diag_base_largest);
                        SMKW("[CAM-SHARED-OP-COORD-R6] begin kind=%s target=%s owner=%.8s gen=%u viewers=%u",
                             s_auto_cal_profile_switch ? "profile" : "cal",
                             s_auto_cal_target_profile == CAMERA_H264_PROFILE_720P_NATIVE
                                 ? "720p60" : "1080p",
                             s_auto_cal_owner_token,
                             (unsigned)s_auto_cal_generation,
                             (unsigned)smoke_session_count_locked());
                    } else if (s_auto_cal_profile_switch != requested_profile_switch ||
                               (requested_profile_switch &&
                                s_auto_cal_target_profile != requested_target)) {
                        cal_error = "camera_operation_owner_busy";
                    }
                    if (!cal_error) smoke_auto_cal_refresh_locked();
                }
            } else if (strcmp(cal_op, "apply") == 0) {
                /* R21 apply: fail closed until the lease owner is the only
                 * allocated+connected session. Other viewers must complete
                 * exact-SID teardown first. The owner's transport and all
                 * persistent H.264 memory remain resident during the switch. */
                if (!have_token || !smoke_auto_cal_owner_locked(cal_token)) {
                    cal_error = s_auto_cal_active ? "camera_operation_not_owner"
                                                  : "camera_operation_not_active";
                } else if (!s_auto_cal_profile_switch) {
                    cal_error = "camera_operation_not_profile";
                } else {
                    smoke_reap_stale_locked();
                    smoke_sess_t *owner_sess = smoke_find_client_token_locked(cal_token);
                    const uint32_t pcount = smoke_session_count_locked();
                    const uint32_t ccount = smoke_session_connected_count_locked();
                    const bool owner_ready = owner_sess && owner_sess->running &&
                                             owner_sess->peer_connected &&
                                             !__atomic_load_n(&owner_sess->teardown_started, __ATOMIC_ACQUIRE);
                    if (!owner_ready || pcount != 1U || ccount != 1U) {
                        SMKW("[DMA-BOUNDARY-R21] apply waiting owner=%.8s owner_ready=%u peers=%u connected=%u largest=%u",
                             cal_token, owner_ready ? 1U : 0U,
                             (unsigned)pcount, (unsigned)ccount,
                             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA));
                        cal_error = "profile_inplace_busy";
                    } else {
                        smoke_auto_cal_refresh_locked();
                        SMKW("[DMA-BOUNDARY-R21] other viewers gone; owner sid=%u target=%s peers=1 connected=1 largest=%u",
                             (unsigned)owner_sess->session_id,
                             camera_h264_stream_profile_name(s_auto_cal_target_profile),
                             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA));
                        esp_err_t swr =
                            smoke_profile_switch_owner_only_locked(s_auto_cal_target_profile, true);
                        if (swr != ESP_OK) {
                            cal_error = "profile_inplace_failed";
                            cal_error_status = "503 Service Unavailable";
                        }
                    }
                }
            } else if (strcmp(cal_op, "apply_cal_profile") == 0) {
                /* [CAM-AUTOCAL-INLEASE-PROFILE-R22A-20260920]
                 * Auto-cal already owns the shared-camera lease. Permit that
                 * SAME owner to switch 1080<->720 without replacing the lease
                 * with kind=profile. This preserves calibration exclusivity,
                 * owner WebRTC/ICE/DTLS/SEND_RECV/audio and the resident H264
                 * pools, while still requiring every other connected session
                 * to be gone before camera/H264 restart. */
                if (!have_token || !smoke_auto_cal_owner_locked(cal_token)) {
                    cal_error = s_auto_cal_active ? "camera_operation_not_owner"
                                                  : "camera_operation_not_active";
                } else if (s_auto_cal_profile_switch) {
                    cal_error = "camera_operation_not_calibration";
                } else {
                    char target_txt[32] = {0};
                    if (httpd_query_key_value(qbuf, "target", target_txt,
                                              sizeof(target_txt)) != ESP_OK) {
                        cal_error = "missing_target_profile";
                        cal_error_status = "400 Bad Request";
                    } else {
                        const bool target_720 =
                            !strcasecmp(target_txt, "720") ||
                            !strcasecmp(target_txt, "720p") ||
                            !strcasecmp(target_txt, "720p-highspeed") ||
                            !strcasecmp(target_txt, "720p60") ||
                            !strcasecmp(target_txt, "720p60-native") ||
                            !strcasecmp(target_txt, "1288x728");
                        const bool target_1080 =
                            !strcasecmp(target_txt, "1080") ||
                            !strcasecmp(target_txt, "1080p") ||
                            !strcasecmp(target_txt, "1920x1080");
                        if (!target_720 && !target_1080) {
                            cal_error = "bad_target_profile";
                            cal_error_status = "400 Bad Request";
                        } else {
                            const camera_h264_profile_t cal_target =
                                target_720 ? CAMERA_H264_PROFILE_720P_NATIVE
                                           : CAMERA_H264_PROFILE_1080P;
                            smoke_reap_stale_locked();
                            smoke_sess_t *owner_sess =
                                smoke_find_client_token_locked(cal_token);
                            const uint32_t pcount = smoke_session_count_locked();
                            const uint32_t ccount =
                                smoke_session_connected_count_locked();
                            const bool owner_ready =
                                owner_sess && owner_sess->running &&
                                owner_sess->peer_connected &&
                                !__atomic_load_n(&owner_sess->teardown_started,
                                                 __ATOMIC_ACQUIRE);
                            if (!owner_ready || pcount != 1U || ccount != 1U) {
                                SMKW("[DMA-BOUNDARY-R22A-CAL-PROFILE] waiting owner=%.8s owner_ready=%u peers=%u connected=%u target=%s largest=%u",
                                     cal_token, owner_ready ? 1U : 0U,
                                     (unsigned)pcount, (unsigned)ccount,
                                     camera_h264_stream_profile_name(cal_target),
                                     (unsigned)heap_caps_get_largest_free_block(
                                         MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA));
                                cal_error = "cal_profile_inplace_busy";
                            } else {
                                smoke_auto_cal_refresh_locked();
                                SMKW("[DMA-BOUNDARY-R22A-CAL-PROFILE] begin owner_sid=%u %s->%s peers=1 connected=1 largest=%u",
                                     (unsigned)owner_sess->session_id,
                                     camera_h264_stream_profile_name(
                                         camera_h264_stream_get_profile()),
                                     camera_h264_stream_profile_name(cal_target),
                                     (unsigned)heap_caps_get_largest_free_block(
                                         MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA));
                                esp_err_t swr =
                                    smoke_profile_switch_owner_only_locked(cal_target, false);
                                if (swr != ESP_OK) {
                                    cal_error = "cal_profile_inplace_failed";
                                    cal_error_status = "503 Service Unavailable";
                                } else {
                                    SMKW("[DMA-BOUNDARY-R22A-CAL-PROFILE] PASS owner_sid=%u target=%s largest=%u",
                                         (unsigned)owner_sess->session_id,
                                         camera_h264_stream_profile_name(cal_target),
                                         (unsigned)heap_caps_get_largest_free_block(
                                             MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA));
                                }
                            }
                        }
                    }
                }
            } else if (strcmp(cal_op, "heartbeat") == 0) {
                if (!have_token || !smoke_auto_cal_owner_locked(cal_token)) {
                    cal_error = s_auto_cal_active ? "camera_operation_not_owner"
                                                  : "camera_operation_not_active";
                } else {
                    smoke_auto_cal_refresh_locked();
                }
            } else if (strcmp(cal_op, "end") == 0) {
                if (s_auto_cal_active) {
                    if (!have_token || !smoke_auto_cal_owner_locked(cal_token)) {
                        cal_error = "camera_operation_not_owner";
                    } else {
                        smoke_auto_cal_clear_locked("owner_end");
                    }
                }
            } else if (strcmp(cal_op, "state") != 0) {
                cal_error = "bad_cal_op";
                cal_error_status = "400 Bad Request";
            }

            bool cal_active = s_auto_cal_active;
            bool cal_owner = cal_active && have_token &&
                             smoke_auto_cal_owner_locked(cal_token);
            uint32_t cal_generation = s_auto_cal_generation;
            uint32_t cal_peer_count = smoke_session_count_locked();
            uint32_t cal_connected_count = smoke_session_connected_count_locked();

            /* [CAM-AUTOCAL-OWNER-COUNT-FIX-R1-20260918]
             * Server-authoritative ownership. First validate the browser's
             * sid+token pair; if sid is absent/stale, fall back to exact token.
             * This makes allocated stale slots harmless: only CONNECTED other
             * sessions block calibration/profile switching. */
            smoke_sess_t *cal_self_sess = NULL;
            if (have_token && cal_requested_sid != 0U) {
                smoke_sess_t *by_sid = smoke_find_session_locked(cal_requested_sid);
                if (by_sid && by_sid->client_token[0] &&
                    strcmp(by_sid->client_token, cal_token) == 0) {
                    cal_self_sess = by_sid;
                }
            }
            if (!cal_self_sess && have_token)
                cal_self_sess = smoke_find_client_token_locked(cal_token);

            const uint32_t cal_self_sid = cal_self_sess ? cal_self_sess->session_id : 0U;
            const bool cal_self_connected = cal_self_sess && cal_self_sess->peer_connected;
            const uint32_t cal_other_peer_count =
                cal_peer_count - (cal_self_sess ? 1U : 0U);
            const uint32_t cal_other_connected_count =
                cal_connected_count - (cal_self_connected ? 1U : 0U);

            if (strcmp(cal_op, "begin") == 0 ||
                (cal_requested_sid != 0U && cal_requested_sid != cal_self_sid)) {
                SMKW("[CAM-AUTOCAL-OWNER-COUNT-FIX-R1] op=%s token=%.8s req_sid=%u self_sid=%u self_conn=%u peers=%u conn=%u other_peers=%u other_conn=%u",
                     cal_op, have_token ? cal_token : "",
                     (unsigned)cal_requested_sid, (unsigned)cal_self_sid,
                     cal_self_connected ? 1U : 0U,
                     (unsigned)cal_peer_count, (unsigned)cal_connected_count,
                     (unsigned)cal_other_peer_count,
                     (unsigned)cal_other_connected_count);
            }

            int64_t cal_lease_ms = cal_active
                ? (s_auto_cal_lease_until_ms - now_ms) : 0LL;
            if (cal_lease_ms < 0) cal_lease_ms = 0;
            bool cal_profile_switch = s_auto_cal_profile_switch;
            camera_h264_profile_t cal_target_profile = s_auto_cal_target_profile;
            xSemaphoreGive(s_gate);

            if (cal_error) return send_err(req, cal_error_status, cal_error);

            cJSON *cal_out = cJSON_CreateObject();
            cJSON *cal = cJSON_CreateObject();
            if (!cal_out || !cal) {
                cJSON_Delete(cal_out);
                cJSON_Delete(cal);
                return send_err(req, "503 Service Unavailable", "oom");
            }
            cJSON_AddBoolToObject(cal_out, "ok", true);
            cJSON_AddBoolToObject(cal, "active", cal_active);
            cJSON_AddBoolToObject(cal, "owner", cal_owner);
            cJSON_AddNumberToObject(cal, "generation", cal_generation);
            cJSON_AddNumberToObject(cal, "peer_count", cal_peer_count);
            cJSON_AddNumberToObject(cal, "connected_count", cal_connected_count);
            cJSON_AddNumberToObject(cal, "self_sid", cal_self_sid);
            cJSON_AddBoolToObject(cal, "self_connected", cal_self_connected);
            cJSON_AddNumberToObject(cal, "other_peer_count", cal_other_peer_count);
            cJSON_AddNumberToObject(cal, "other_connected_count", cal_other_connected_count);
            cJSON_AddNumberToObject(cal, "lease_ms", (double)cal_lease_ms);
            cJSON_AddStringToObject(cal, "kind", cal_profile_switch ? "profile" : "cal");
            cJSON_AddStringToObject(
                cal, "target",
                cal_target_profile == CAMERA_H264_PROFILE_720P_NATIVE
                    ? "720p-highspeed" : "1080p");
            cJSON_AddItemToObject(cal_out, "calibration", cal);
            esp_err_t cal_send = send_json(req, "200 OK", cal_out);
            cJSON_Delete(cal_out);
            return cal_send;
        }

        /* Flight-recorder controls deliberately reuse this existing GET URI.
         * They return immediately and never enter the camera benchmark/tune path. */
        char flight_op[16] = {0};
        if (httpd_query_key_value(qbuf, "flight", flight_op, sizeof(flight_op)) == ESP_OK) {
            if (strcmp(flight_op, "download") == 0) {
                return flight_recorder_http_download(req);
            }
            if (strcmp(flight_op, "clear") == 0) {
                return smoke_fr_send_action_status(req, flight_recorder_clear());
            }
            if (strcmp(flight_op, "save") == 0 || strcmp(flight_op, "mark") == 0) {
                char fsid[16] = {0}, fprofile[24] = {0}, ftag[16] = {0};
                char fa0[16] = {0}, fa1[16] = {0};
                uint32_t sid = 0U;
                int32_t arg0 = 0, arg1 = 0;
                if (httpd_query_key_value(qbuf, "fsid", fsid, sizeof(fsid)) == ESP_OK)
                    sid = (uint32_t)strtoul(fsid, NULL, 10);
                camera_h264_profile_t fprof = camera_h264_stream_get_profile();
                if (httpd_query_key_value(qbuf, "fprofile", fprofile, sizeof(fprofile)) == ESP_OK)
                    fprof = smoke_profile_from_text(fprofile);
                const char *tag = "WEB";
                if (httpd_query_key_value(qbuf, "ftag", ftag, sizeof(ftag)) == ESP_OK &&
                    smoke_fr_tag_valid(ftag)) tag = ftag;
                if (httpd_query_key_value(qbuf, "fa0", fa0, sizeof(fa0)) == ESP_OK)
                    arg0 = (int32_t)strtol(fa0, NULL, 10);
                if (httpd_query_key_value(qbuf, "fa1", fa1, sizeof(fa1)) == ESP_OK)
                    arg1 = (int32_t)strtol(fa1, NULL, 10);

                if (xSemaphoreTake(s_gate, pdMS_TO_TICKS(500)) != pdTRUE)
                    return send_err(req, "409 Conflict", "session_gate_busy");
                smoke_sess_t *sess = smoke_find_session_locked(sid);
                if (strcmp(flight_op, "save") == 0) {
                    smoke_fr_trigger_locked(FR_EVENT_MANUAL_SAVE, sess, fprof,
                                            sess ? sess->client_token : NULL,
                                            tag, arg0, arg1);
                } else {
                    smoke_fr_record_locked(FR_EVENT_BROWSER_MARK, sess, fprof,
                                           sess ? sess->client_token : NULL,
                                           tag, arg0, arg1);
                }
                xSemaphoreGive(s_gate);
                esp_err_t action = (strcmp(flight_op, "save") == 0)
                    ? flight_recorder_service() : ESP_OK;
                return smoke_fr_send_action_status(req, action);
            }
            return send_err(req, "400 Bad Request", "bad_flight_op");
        }

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
    uint32_t peer_count = 0, peer_connected_count = 0, viewer_count = 0;
    bool status_self_viewer_alive = false;
    uint32_t status_self_sid = 0U; /* [PTT-PRESENCE-R4-20260918] */
    bool cal_status_active = false, cal_status_owner = false;
    bool cal_status_profile_switch = false;
    camera_h264_profile_t cal_status_target_profile = CAMERA_H264_PROFILE_720P_NATIVE;
    uint32_t cal_status_generation = 0;
    int64_t cal_status_lease_ms = 0;
    uint32_t media_sent = 0, media_drop = 0;
    uint32_t tx_q = 0, tx_hwm = 0, tx_enqueued = 0, tx_qdrop = 0, tx_oversize = 0;
    uint32_t tx_copy_last = 0, tx_copy_max = 0, tx_send_last = 0, tx_send_max = 0;
    if (xSemaphoreTake(s_gate, pdMS_TO_TICKS(200)) == pdTRUE) {
        /* [CAM-STALE-SLOT-VIEW0-FIX-R9-20260917]
         * Status polling is already periodic while a browser is waiting/playing.
         * Let it service the existing stale-session reaper too, so an abandoned
         * CONNECTING slot can age out without requiring another /offer first. */
        /* Apply an exact sid+token+page heartbeat before timeout reaping.
         * A repeated frame counter never refreshes the deadline. */
        if (viewer_hb_present && viewer_hb_valid) {
            smoke_sess_t *hb_sess = smoke_find_session_locked(viewer_hb_sid);
            if (hb_sess && hb_sess->viewer_hb_required &&
                hb_sess->client_token[0] &&
                strcmp(hb_sess->client_token, status_client_token) == 0 &&
                hb_sess->diag_page_id[0] &&
                strcmp(hb_sess->diag_page_id, viewer_hb_page) == 0) {
                const uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);
                const bool was_visible =
                    __atomic_load_n(&hb_sess->viewer_hb_visible, __ATOMIC_ACQUIRE);
                if (!viewer_hb_visible) {
                    __atomic_store_n(&hb_sess->viewer_hb_visible, false, __ATOMIC_RELEASE);
                    __atomic_store_n(&hb_sess->viewer_hb_last_ms, now_ms, __ATOMIC_RELEASE);
                    if (was_visible) {
                        SMK_WEBRTC_DIAG("[CAM-VIEWER-HEARTBEAT-R1] sid=%u hidden grace=%ums",
                             (unsigned)hb_sess->session_id,
                             (unsigned)SMOKE_VIEWER_HEARTBEAT_TIMEOUT_MS);
                    }
                } else {
                    const uint32_t prev_frames =
                        __atomic_load_n(&hb_sess->viewer_hb_last_frames, __ATOMIC_ACQUIRE);
                    if (viewer_hb_frames > prev_frames) {
                        __atomic_store_n(&hb_sess->viewer_hb_last_frames,
                                         viewer_hb_frames, __ATOMIC_RELEASE);
                        __atomic_store_n(&hb_sess->viewer_hb_last_ms, now_ms,
                                         __ATOMIC_RELEASE);
                        __atomic_store_n(&hb_sess->viewer_hb_seen, true,
                                         __ATOMIC_RELEASE);
                        __atomic_store_n(&hb_sess->viewer_hb_visible, true,
                                         __ATOMIC_RELEASE);
                        if (!was_visible) {
                            SMK_WEBRTC_DIAG("[CAM-VIEWER-HEARTBEAT-R1] sid=%u visible frames=%u",
                                 (unsigned)hb_sess->session_id,
                                 (unsigned)viewer_hb_frames);
                        }
                    }
                }
                status_self_viewer_alive = smoke_session_is_live_viewer_locked(
                    hb_sess, (uint32_t)(esp_timer_get_time() / 1000ULL));
            }
        }
        smoke_reap_stale_locked();
        smoke_auto_cal_expire_locked();
        cal_status_active = s_auto_cal_active;
        cal_status_owner = cal_status_active && status_client_token_ok &&
                           smoke_auto_cal_owner_locked(status_client_token);
        cal_status_generation = s_auto_cal_generation;
        cal_status_profile_switch = s_auto_cal_profile_switch;
        cal_status_target_profile = s_auto_cal_target_profile;
        if (cal_status_active) {
            cal_status_lease_ms = s_auto_cal_lease_until_ms -
                                  (esp_timer_get_time() / 1000LL);
            if (cal_status_lease_ms < 0) cal_status_lease_ms = 0;
        }

        smoke_sess_t *list[SMOKE_MAX_SESSIONS] = {s_active, s_secondary};
        peer_count = smoke_session_count_locked();
        peer_connected_count = smoke_session_connected_count_locked();
        viewer_count = smoke_session_viewer_count_locked();
        peer_active = (peer_count != 0U);
        if (status_client_token_ok) {
            smoke_sess_t *self_sess = smoke_find_client_token_locked(status_client_token);
            if (self_sess) status_self_sid = self_sess->session_id;
        }

        /* [DUPVIEW-DMA-DIAG-R1-20260918]
         * Status polling runs under s_gate, so this is the safest point to
         * snapshot BOTH connected sessions without introducing a callback UAF.
         * Record once per 2-viewer episode. Flight Recorder gets two rows:
         * arg0=browser_id hash, arg1=page_id hash. Same arg0 on DUP-A/DUP-S
         * proves the two sessions came from the same persisted browser id. */
        if (peer_connected_count >= 2U && s_active && s_secondary &&
            s_active->peer_connected && s_secondary->peer_connected) {
            if (s_dupdiag_last_active_sid != s_active->session_id ||
                s_dupdiag_last_secondary_sid != s_secondary->session_id) {
                esp_peer_addr_t a_addr = {0}, b_addr = {0};
                const bool a_addr_ok = s_active->peer &&
                    esp_peer_get_paired_addr(s_active->peer, &a_addr) == ESP_PEER_ERR_NONE;
                const bool b_addr_ok = s_secondary->peer &&
                    esp_peer_get_paired_addr(s_secondary->peer, &b_addr) == ESP_PEER_ERR_NONE;
                const int64_t start_delta = s_secondary->start_ms - s_active->start_ms;
                SMK_WEBRTC_DIAG("[DUPVIEW-DIAG-R1] TWO_CONNECTED "
                     "A{sid=%u tok=%.8s browser=%s page=%s seq=%u r=%u %u.%u.%u.%u:%u} "
                     "S{sid=%u tok=%.8s browser=%s page=%s seq=%u r=%u %u.%u.%u.%u:%u} "
                     "start_delta_ms=%lld dma_free=%u dma_largest=%u",
                     (unsigned)s_active->session_id, s_active->client_token,
                     s_active->diag_browser_id[0] ? s_active->diag_browser_id : "-",
                     s_active->diag_page_id[0] ? s_active->diag_page_id : "-",
                     (unsigned)s_active->diag_offer_seq, a_addr_ok ? 1U : 0U,
                     a_addr.ipv4[0], a_addr.ipv4[1], a_addr.ipv4[2], a_addr.ipv4[3], a_addr.port,
                     (unsigned)s_secondary->session_id, s_secondary->client_token,
                     s_secondary->diag_browser_id[0] ? s_secondary->diag_browser_id : "-",
                     s_secondary->diag_page_id[0] ? s_secondary->diag_page_id : "-",
                     (unsigned)s_secondary->diag_offer_seq, b_addr_ok ? 1U : 0U,
                     b_addr.ipv4[0], b_addr.ipv4[1], b_addr.ipv4[2], b_addr.ipv4[3], b_addr.port,
                     (long long)start_delta,
                     (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA),
                     (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA));
                smoke_fr_record_locked(FR_EVENT_BROWSER_MARK, s_active, s_active->profile,
                                       s_active->client_token, "DUP-A",
                                       (int32_t)smoke_diag_hash32(s_active->diag_browser_id),
                                       (int32_t)smoke_diag_hash32(s_active->diag_page_id));
                smoke_fr_record_locked(FR_EVENT_BROWSER_MARK, s_secondary, s_secondary->profile,
                                       s_secondary->client_token, "DUP-S",
                                       (int32_t)smoke_diag_hash32(s_secondary->diag_browser_id),
                                       (int32_t)smoke_diag_hash32(s_secondary->diag_page_id));
                s_dupdiag_last_active_sid = s_active->session_id;
                s_dupdiag_last_secondary_sid = s_secondary->session_id;
            }
        } else if (peer_connected_count < 2U) {
            s_dupdiag_last_active_sid = 0U;
            s_dupdiag_last_secondary_sid = 0U;
        }

        /* [DMA-BLACKBOX-R2-20260919] edge + persistence/recovery on existing status-poll cadence. */
        smoke_dma_blackbox_sample_locked(
            (uint32_t)(esp_timer_get_time() / 1000ULL));

        for (uint32_t i = 0; i < SMOKE_MAX_SESSIONS; ++i) {
            smoke_sess_t *ps = list[i];
            if (!ps) continue;
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

    /* [DMA-BLACKBOX-R5-AUTO-PERSIST-20260919]
     * h_status_get() already services pending recorder writes at entry. A DMA
     * PERSIST trigger can be raised later in this same request while s_gate is
     * held, so service once more after releasing s_gate. With no pending save
     * this is only a cheap state check; with PERSIST it commits immediately. */
    (void)flight_recorder_service();

    camera_h264_stream_stats_t cs = {0};
    camera_h264_stream_get_stats(&cs);
    cJSON_AddBoolToObject(out, "peer_active", peer_active);
    cJSON_AddBoolToObject(out, "peer_connected", peer_connected);
    cJSON_AddNumberToObject(out, "peer_count", peer_count);
    cJSON_AddNumberToObject(out, "peer_connected_count", peer_connected_count);
    cJSON_AddNumberToObject(out, "viewer_count", viewer_count);
    cJSON_AddBoolToObject(out, "self_viewer_alive", status_self_viewer_alive);
    cJSON_AddNumberToObject(out, "viewer_heartbeat_timeout_ms",
                           SMOKE_VIEWER_HEARTBEAT_TIMEOUT_MS);
    cJSON_AddNumberToObject(out, "peer_pending_count",
                           peer_count >= peer_connected_count
                               ? (peer_count - peer_connected_count) : 0U);
    cJSON_AddNumberToObject(out, "max_peers", SMOKE_MAX_SESSIONS);
    /* [PTT-PRESENCE-R4-20260918]
     * Read-only UI telemetry. Never changes talkback ownership or media flow. */
    const bool talkback_active = sound_talkback_is_active();
    const uint32_t talkback_owner_sid = sound_talkback_owner_session();
    const bool talkback_owner_self = talkback_active && status_self_sid != 0U &&
                                     talkback_owner_sid == status_self_sid;
    const bool talkback_busy = talkback_active && talkback_owner_sid != 0U &&
                               !talkback_owner_self;
    cJSON_AddBoolToObject(out, "talkback_active", talkback_active);
    cJSON_AddBoolToObject(out, "talkback_owner_self", talkback_owner_self);
    cJSON_AddBoolToObject(out, "talkback_busy", talkback_busy);
    /* Compact string avoids a nested cJSON object on the hot 1.2 s path.
     * Layout:
     * active,owner,generation,peer_count,connected_count,lease_ms,kind,target
     * kind: 0=cal,1=profile | target: 0=1080p,1=720p60 */
    char cal_compact[96];
    snprintf(cal_compact, sizeof(cal_compact), "%u,%u,%u,%u,%u,%lld,%u,%u",
             cal_status_active ? 1U : 0U,
             cal_status_owner ? 1U : 0U,
             (unsigned)cal_status_generation,
             (unsigned)peer_count,
             (unsigned)peer_connected_count,
             (long long)cal_status_lease_ms,
             cal_status_profile_switch ? 1U : 0U,
             cal_status_target_profile == CAMERA_H264_PROFILE_720P_NATIVE ? 1U : 0U);
    cJSON_AddStringToObject(out, "cal", cal_compact);
    smoke_fr_add_compact_status_json(out);
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
    cJSON_AddNumberToObject(cam, "tx_kbps", cs.h264_tx_kbps);
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

    /* [FACE-AI-OVERLAY-STATUS-MERGE-R5-20260929]
     * Piggyback face metadata on the EXISTING 1.2 s status/heartbeat request.
     * One compact cJSON string node only: no extra URI, no extra HTTP poll,
     * no early return before viewer heartbeat processing.
     * Format valid,seq,src_w,src_h,x0,y0,x1,y1,score_x1000,age_ms */
    face_ai_box_t fb = {0};
    char face_compact[128];
    if (face_ai_detect_get_latest(&fb) && fb.valid) {
        snprintf(face_compact, sizeof(face_compact),
                 "1,%u,%u,%u,%ld,%ld,%ld,%ld,%u,%u",
                 (unsigned)fb.seq, (unsigned)fb.src_w, (unsigned)fb.src_h,
                 (long)fb.x0, (long)fb.y0, (long)fb.x1, (long)fb.y1,
                 (unsigned)fb.score_x1000, (unsigned)fb.age_ms);
    } else {
        strcpy(face_compact, "0");
    }
    cJSON_AddStringToObject(out, "face", face_compact);

    /* [FACE-MFN-PREP-GATE-R1E-20261003]
     * Reuse the existing status heartbeat: no new endpoint/poll. Compact format
     * active,done,total. Read-only; profile switching remains browser-owned. */
    face_ai_prepare_status_t fp = {0};
    char face_prep_compact[24];
    if (face_ai_detect_get_prepare_status(&fp)) {
        snprintf(face_prep_compact, sizeof(face_prep_compact), "%u,%u,%u",
                 fp.active ? 1U : 0U, (unsigned)fp.done, (unsigned)fp.total);
    } else {
        strcpy(face_prep_compact, "0,0,0");
    }
    cJSON_AddStringToObject(out, "face_prep", face_prep_compact);

    /* [FACE-LEARN-COLOR-D3R3-20261006]
     * Same existing 1.2 s heartbeat, no new URI and no new poll.
     * Compact format state,done,total. */
    face_ai_learn_status_t fl = {0};
    char face_learn_compact[24];
    if (face_ai_detect_get_learn_status(&fl)) {
        snprintf(face_learn_compact, sizeof(face_learn_compact), "%u,%u,%u",
                 (unsigned)fl.state, (unsigned)fl.done, (unsigned)fl.total);
    } else {
        strcpy(face_learn_compact, "0,0,3");
    }
    cJSON_AddStringToObject(out, "face_learn", face_learn_compact);
    smoke_status_add_face_dg(out); /* [FACE-TRACK-DIAG-R2-20261007] */
    smoke_status_add_face_ev(out); /* [FACE-EV-R3C1-20261008] */
    smoke_status_add_face_mot(out); /* [FACE-MOT-R4-20261008] shadow tracker (log only) */
    smoke_status_add_face_own(out); /* [FACE-OWNER-R1-20261015] browser OWNER label evidence */
    smoke_status_add_face_orq(out, qbuf, status_client_token, status_client_token_ok,
                              status_self_sid); /* [FACE-OWNER-ORQ-R1-20261016] */
    smoke_status_add_face_sep(out); /* [WEB-SFACE-MFN-SEP-R1-20261019] proof counters */

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
            cJSON_AddNumberToObject(measure, "y_p95", cm.y_p95);
            cJSON_AddNumberToObject(measure, "y_clip_low_bp", cm.y_clip_low_bp);
            cJSON_AddNumberToObject(measure, "y_clip_high_bp", cm.y_clip_high_bp);
            cJSON_AddNumberToObject(measure, "u_mean", cm.u_mean);
            cJSON_AddNumberToObject(measure, "v_mean", cm.v_mean);
            cJSON_AddNumberToObject(measure, "u_abs", cm.u_abs);
            cJSON_AddNumberToObject(measure, "v_abs", cm.v_abs);
            cJSON_AddNumberToObject(measure, "neutral_count", cm.neutral_count);
            cJSON_AddNumberToObject(measure, "neutral_u_mean", cm.neutral_u_mean);
            cJSON_AddNumberToObject(measure, "neutral_v_mean", cm.neutral_v_mean);
            smoke_add_float9_csv(measure, "zone_y", cm.zone_y_mean);
            smoke_add_float9_csv(measure, "zone_u", cm.zone_u_mean);
            smoke_add_float9_csv(measure, "zone_v", cm.zone_v_mean);
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
/* [HTTP-PAGE-DMA-F6-GZIP-20260919]
 * F5 pacing recovered part of the pre-camera DMA floor but a persistent
 * 31KB -> 23KB drop can still happen while the camera page is being delivered.
 *
 * F6 changes the production page transport, not the camera/WebRTC engine:
 *   - serve a pre-compressed camera_live.html.gz when the browser accepts gzip
 *   - pace the much smaller gzip payload in 1KB chunks
 *   - pause one RTOS tick after every 2KB
 *   - use a build-specific ETag so repeat page loads return 304 with no body
 *   - fall back to the original flash-resident HTML if gzip is unavailable
 *
 * No heap copy buffer is allocated by this handler. Both raw and gzip payloads
 * are linked as flash-resident EMBED_FILES.
 */
#define SMOKE_CAMERA_PAGE_GZ_CHUNK_BYTES 1024U
#define SMOKE_CAMERA_PAGE_GZ_YIELD_BYTES 2048U
#define SMOKE_CAMERA_PAGE_RAW_CHUNK_BYTES 1024U
#define SMOKE_CAMERA_PAGE_RAW_YIELD_BYTES 2048U

static bool smoke_req_accepts_gzip(httpd_req_t *req)
{
    size_t n = httpd_req_get_hdr_value_len(req, "Accept-Encoding");
    if (n == 0 || n > 255) return false;

    char enc[256];
    if (httpd_req_get_hdr_value_str(req, "Accept-Encoding", enc, sizeof(enc)) != ESP_OK) {
        return false;
    }
    return strstr(enc, "gzip") != NULL;
}

static bool smoke_camera_page_etag_matches(httpd_req_t *req, const char *etag)
{
    size_t n = httpd_req_get_hdr_value_len(req, "If-None-Match");
    if (n == 0 || n > 95) return false;

    char inm[96];
    if (httpd_req_get_hdr_value_str(req, "If-None-Match", inm, sizeof(inm)) != ESP_OK) {
        return false;
    }
    return strcmp(inm, etag) == 0;
}

static esp_err_t smoke_send_flash_paced(httpd_req_t *req,
                                        const uint8_t *data,
                                        size_t len,
                                        size_t chunk_bytes,
                                        size_t yield_bytes,
                                        const char *tag)
{
    size_t off = 0;
    size_t since_yield = 0;
    const size_t largest0 = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);

    ESP_LOGW(TAG,
             "[HTTP-PAGE-DMA-F6-GZIP-20260919] %s begin len=%u chunk=%u largest=%u",
             tag, (unsigned)len, (unsigned)chunk_bytes, (unsigned)largest0);

    while (off < len) {
        size_t n = len - off;
        if (n > chunk_bytes) n = chunk_bytes;

        esp_err_t ret = httpd_resp_send_chunk(req, (const char *)data + off, n);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG,
                     "[HTTP-PAGE-DMA-F6-GZIP-20260919] %s chunk failed off=%u/%u ret=%s",
                     tag, (unsigned)off, (unsigned)len, esp_err_to_name(ret));
            return ret;
        }

        off += n;
        since_yield += n;
        if (since_yield >= yield_bytes && off < len) {
            since_yield = 0;
            vTaskDelay(1);
        }
    }

    esp_err_t ret = httpd_resp_send_chunk(req, NULL, 0);
    const size_t largest1 = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
    ESP_LOGW(TAG,
             "[HTTP-PAGE-DMA-F6-GZIP-20260919] %s end ret=%s largest=%u->%u delta=%d",
             tag, esp_err_to_name(ret), (unsigned)largest0, (unsigned)largest1,
             (int)largest1 - (int)largest0);
    return ret;
}

static esp_err_t h_camera_page_get(httpd_req_t *req)
{
    smoke_dma_r8_phase_log("PAGE0 camera_get_entry");
    isp_tune_restore_saved_settings_once();
    smoke_dma_r8_phase_log("PAGE1 after_isp_restore");

    /* [CAM-DMA-BOUNDARY-R16F8-20260919]
     * First camera-page visit creates the esp_video/ISP controller BEFORE the
     * browser can issue status/color/WebRTC follow-up requests.  The controller
     * is then kept resident by R12 for both 720 and 1080, so these INTERNAL/DMA
     * allocations are never returned/repositioned on later mode switches.
     * Failure is deliberately fail-open: the normal START path can still try. */
    {
        const size_t b0 = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
        esp_err_t br = camera_h264_dma_boundary_bootstrap();
        const size_t b1 = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
        if (br == ESP_OK) {
            ESP_LOGW(TAG,
                     "[CAM-DMA-BOUNDARY-R16F8] page bootstrap OK largest=%u->%u delta=%d",
                     (unsigned)b0, (unsigned)b1, (int)b1 - (int)b0);
        } else {
            ESP_LOGW(TAG,
                     "[CAM-DMA-BOUNDARY-R16F8] page bootstrap fail-open ret=%s largest=%u->%u",
                     esp_err_to_name(br), (unsigned)b0, (unsigned)b1);
        }
    }
    smoke_dma_r8_phase_log("PAGE1B after_camera_boundary_bootstrap");

    const size_t raw_len = (size_t)(s_camera_live_html_end - s_camera_live_html_start);
    const size_t gz_len = (size_t)(s_camera_live_html_gz_end - s_camera_live_html_gz_start);
    const bool gzip_ok = smoke_req_accepts_gzip(req) && gz_len > 0;

    char etag[80];
    snprintf(etag, sizeof(etag), "\"cam-%s-%s-%u\"",
             __DATE__, __TIME__, (unsigned)(gzip_ok ? gz_len : raw_len));

    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
    httpd_resp_set_hdr(req, "ETag", etag);
    httpd_resp_set_hdr(req, "Vary", "Accept-Encoding");

    if (smoke_camera_page_etag_matches(req, etag)) {
        httpd_resp_set_status(req, "304 Not Modified");
        esp_err_t ret = httpd_resp_send(req, NULL, 0);
        ESP_LOGW(TAG,
                 "[HTTP-PAGE-DMA-F6-GZIP-20260919] 304 etag hit gzip=%d raw=%u gz=%u largest=%u",
                 gzip_ok ? 1 : 0, (unsigned)raw_len, (unsigned)gz_len,
                 (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA));
        smoke_dma_r8_phase_log("PAGE2 after_html_send");
        return ret;
    }

    esp_err_t ret;
    if (gzip_ok) {
        httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
        ret = smoke_send_flash_paced(req,
                                     s_camera_live_html_gz_start, gz_len,
                                     SMOKE_CAMERA_PAGE_GZ_CHUNK_BYTES,
                                     SMOKE_CAMERA_PAGE_GZ_YIELD_BYTES,
                                     "gzip");
    } else {
        ret = smoke_send_flash_paced(req,
                                     s_camera_live_html_start, raw_len,
                                     SMOKE_CAMERA_PAGE_RAW_CHUNK_BYTES,
                                     SMOKE_CAMERA_PAGE_RAW_YIELD_BYTES,
                                     "raw-fallback");
    }

    ESP_LOGW(TAG,
             "[HTTP-PAGE-DMA-F6-GZIP-20260919] page ret=%s gzip=%d raw=%u gz=%u ratio_permille=%u",
             esp_err_to_name(ret), gzip_ok ? 1 : 0,
             (unsigned)raw_len, (unsigned)gz_len,
             (unsigned)(raw_len ? ((gz_len * 1000U) / raw_len) : 0U));

    smoke_dma_r8_phase_log("PAGE2 after_html_send");
    return ret;
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
/* ????????????? ?록 ????????????? */


/* [SFACE-DB-R1A] Reserve existing WebRTC session mutex THROUGH the write/erase.
 * A new offer cannot start between an idle check and commit. The web task
 * owns this mutex until sface_db_flash_end(); no calls on camera workers.
 * No change to WebRTC/TURN/owner verdicts. */
bool sface_db_flash_begin(void)
{
    if (!s_gate || camera_h264_stream_is_active() || camera_webrtc_audio_is_active()) return false;
    if (xSemaphoreTake(s_gate, pdMS_TO_TICKS(150)) != pdTRUE) return false;
    if (s_active != NULL || s_secondary != NULL ||
        camera_h264_stream_is_active() || camera_webrtc_audio_is_active()) {
        xSemaphoreGive(s_gate);
        return false;
    }
    return true;  /* gate intentionally held until sface_db_flash_end */
}

void sface_db_flash_end(void)
{
    xSemaphoreGive(s_gate);
}

esp_err_t web_smoke_register(httpd_handle_t server)
{
    if (!server) return ESP_ERR_INVALID_ARG;
    /* [CAM-AUDIO-CRACKLE-PEERLOG-DIAG-V2-20260902]
     * A/B diagnostic only.
     * Suppress PEER_DEF INFO/DEBUG bursts while preserving WARN/ERROR.
     * No media, queue, RTP, H264, ICE, TURN, gain or codec behavior changes. */
    esp_log_level_set("PEER_DEF", ESP_LOG_WARN);
    esp_log_level_set("AGENT", ESP_LOG_WARN);
    esp_log_level_set("DTLS", ESP_LOG_WARN);
    if (HA_LOG_DIAG_BOOT_BANNERS) {
        ESP_LOGW(TAG, "[AUDIO-CRACKLE-DIAG] PEER_DEF/AGENT/DTLS -> WARN (log policy)");
    }
    ESP_LOGI(TAG, "[LOG-COMPACT-R1] compact logs active: boot=%d dma=%d webrtc=%d face=%d camera=%d; LCD diagnostic remains enabled",
             HA_LOG_DIAG_BOOT_BANNERS, HA_LOG_DIAG_DMA_VERBOSE,
             HA_LOG_DIAG_WEBRTC_VERBOSE, HA_LOG_DIAG_FACE_VERBOSE,
             HA_LOG_DIAG_CAMERA_VERBOSE);
    if (HA_LOG_DIAG_BOOT_BANNERS) {
        ESP_LOGW(TAG, "[LATE-CONNECTED-UAF-GUARD-R1-20260915] active build=%s %s",
                 __DATE__, __TIME__);
        ESP_LOGW(TAG, "[FLIGHT-RECORDER-STAGE2-R1-20260916] active build=%s %s",
                 __DATE__, __TIME__);
        ESP_LOGW(TAG, "[DUPVIEW-DMA-DIAG-R1-20260918] log-only diagnostic active build=%s %s",
                 __DATE__, __TIME__);
        ESP_LOGW(TAG, "[DMA-BLACKBOX-R2-20260919] edge/persist/recover + idle 0/100/500/2000/5000ms active build=%s %s",
                 __DATE__, __TIME__);
        ESP_LOGW(TAG, "[DMA-BLACKBOX-R4-SAFE-WALK-20260919] bounded >=2KB INTERNAL|DMA walk at EDGE and full-idle +5s active build=%s %s",
                 __DATE__, __TIME__);
        ESP_LOGW(TAG, "[DMA-BLACKBOX-R5-AUTO-PERSIST-20260919] confirmed PERSIST -> immediate flight-recorder flash save active build=%s %s",
                 __DATE__, __TIME__);
        ESP_LOGW(TAG, "[DMA-OFFER-PROBE-R6-20260919] flight-recorder O0..O7 /offer allocation window active build=%s %s",
                 __DATE__, __TIME__);
        ESP_LOGW(TAG, "[DMA-BLACKBOX-R7-IDLE-DEFER-20260919] Tmr Svc notify-only; idle work on persistent pump0 active build=%s %s",
                 __DATE__, __TIME__);
        ESP_LOGW(TAG, "[DMA-BLACKBOX-R7A-SCREEN-TRANSITION-SAFE-20260919] delayed full-idle timer series disabled; IDLE0 only build=%s %s",
                 __DATE__, __TIME__);
        ESP_LOGW(TAG, "[DMA-PREOFFER-R8A-R7A-SAFE-20260919] service/page/pre-offer 250ms DMA watch active build=%s %s",
                 __DATE__, __TIME__);
        ESP_LOGW(TAG, "[CAM-AUDIO-GATE-PSRAM-F3-20260919] 3072B audio-gate stack is persistent PSRAM; runtime INTERNAL stack allocation removed build=%s %s",
                 __DATE__, __TIME__);
        ESP_LOGW(TAG, "[HTTP-PAGE-DMA-F6-GZIP-20260919] gzip+ETag+pacing active build=%s %s",
                 __DATE__, __TIME__);
        ESP_LOGW(TAG, "[CAM-DMA-BOUNDARY-R16F8-20260919] first camera-page visit bootstraps esp_video/ISP once; controller stays resident across modes build=%s %s",
                 __DATE__, __TIME__);
        ESP_LOGW(TAG, "[DMA-BOUNDARY-R17F9-PSRAM-SIGNALING-20260919] HTTP body/SDP/candidate/session scratch forced to PSRAM; INTERNAL fallback forbidden build=%s %s",
                 __DATE__, __TIME__);
        ESP_LOGW(TAG, "[DMA-BOUNDARY-R21-DROP-OTHER-BEFORE-SWITCH-20260920] profile owner keeps its esp_peer/ICE/DTLS/SEND_RECV/audio/refpool; every other viewer exact-SID teardown completes first build=%s %s",
                 __DATE__, __TIME__);
        ESP_LOGW(TAG, "[LAN-DIRECT-ICE-ASYNC-R4-20260926] no-prewait async mDNS + 30s RAM cache + PAIRING trickle active build=%s %s",
                 __DATE__, __TIME__);
    
        ESP_LOGW(TAG, "[CAM-AUTO-CAL-PAUSE-RESUME-R3-20260916] active build=%s %s",
                 __DATE__, __TIME__);
        ESP_LOGW(TAG, "[FACE-AI-SCAN-LOW160-R28-20261001] default LOW160x90 + hidden status-URI fallback active active build=%s %s",
                 __DATE__, __TIME__);
    }


    /* Do not call ISP restore here: registration runs inside GOT_IP/sys_evt. */
    smoke_dma_r8_phase_log("REG0 register_entry");
    if (!s_gate) {
        s_gate = xSemaphoreCreateMutex();
        if (!s_gate) return ESP_ERR_NO_MEM;
    }
    if (!s_fanout_lock) {
        s_fanout_lock = xSemaphoreCreateMutex();
        if (!s_fanout_lock) return ESP_ERR_NO_MEM;
    }
    smoke_dma_r8_phase_log("REG1 after_lifecycle_mutexes");
    {
        esp_err_t fre = flight_recorder_init();
        if (fre != ESP_OK) {
            SMKW("[FLIGHT-RECORDER-STAGE2-R1] init=%s; WebRTC continues without recorder",
                 esp_err_to_name(fre));
        }
    }
    smoke_dma_r8_phase_log("REG2 after_flight_recorder");
    {
        esp_err_t de = smoke_dma_idle_timer_init();
        if (de != ESP_OK) {
            SMKW("[DMA-BLACKBOX-R2] idle timer init=%s; edge recorder continues",
                 esp_err_to_name(de));
        }
    }
    smoke_dma_r8_phase_log("REG3 after_static_timer");
    { esp_err_t pe = smoke_pump_workers_init(); if (pe != ESP_OK) return pe; }
    smoke_dma_r8_phase_log("REG4 after_pump_workers");
    {
        esp_err_t le = smoke_lan_mdns_worker_init();
        if (le != ESP_OK) {
            SMK_WEBRTC_DIAG("[LAN-DIRECT-ICE-ASYNC-R4] worker init=%s; direct-LAN async discovery disabled, TURN fallback remains",
                 esp_err_to_name(le));
        }
    }
    smoke_dma_r8_phase_log("REG4B after_lan_mdns_worker");
    { esp_err_t ge = smoke_audio_gate_worker_init(); if (ge != ESP_OK) return ge; }
    { esp_err_t ae = camera_webrtc_audio_init(); if (ae != ESP_OK) return ae; }
    smoke_dma_r8_phase_log("REG5 after_audio_init");
    /* DTLS ?증?는 ?smoke ?션?서 lazy ?성?다. 부??경로?는 비용??주? ?는?? */

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
    smoke_dma_r8_phase_log("REG6 after_smoke_uri_handlers");
    /* [CAM-COLOR-STAGE2-20260901] one extra POST URI */
    e |= camera_color_stage2_register(server);
    { const esp_err_t se = sface_db_register(server); /* opt-in, nonfatal to WebRTC */
      if (se != ESP_OK) ESP_LOGW(TAG, "SFACE-DB optional HTTP registration failed: %s", esp_err_to_name(se)); }
    smoke_dma_r8_phase_log("REG7 after_color_handler");
    if (e != ESP_OK) {
        ESP_LOGW(TAG, "smoke uri register incomplete (max_uri_handlers ?유 ?인)");
    } else {
        ESP_LOGI(TAG, "camera WebRTC registered: max_viewers=%u one-camera/one-H264 fanout (esp_peer %s)", (unsigned)SMOKE_MAX_SESSIONS, SMOKE_ESP_PEER_VER);
    }
    smoke_dma_r8_phase_log("REG8 register_complete");
    if (e == ESP_OK) {
        smoke_dma_preoffer_watch_arm();
    }
    return e;
}
