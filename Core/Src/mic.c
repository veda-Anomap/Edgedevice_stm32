#include "mic.h"
#include <string.h>
#include <math.h>

/* Interleaved frame: [L, R, L, R, ...] */
#define ADC_BUF_LEN 200U
#ifdef HAL_ADC_MODULE_ENABLED
static uint16_t adc_buffer[ADC_BUF_LEN];
#endif

#ifdef HAL_I2S_MODULE_ENABLED
/* INMP441 DMA input: 16-bit halfword stream from I2S peripheral */
#define I2S_DMA_WORD_LEN     200U
#define I2S_DMA_HALFWORD_LEN (I2S_DMA_WORD_LEN * 2U)
static uint16_t i2s_buffer[I2S_DMA_HALFWORD_LEN];
static int32_t dc_offset_l = 0;
static int32_t dc_offset_r = 0;
extern I2S_HandleTypeDef hi2s2;
#endif

/* Direction decision gates */
#define SOUND_TH       390U
#define ALPHA_DIV      8U
#define SWITCH_HOLDOFF 200U
#define MIC_MID_U16    32768U
#define DC_TRACK_SHIFT 9U
#define NOISE_GATE_TH  40

/* Sliding windows and SNR gates */
#define SIG_WIN         12U
#define NOISE_WIN       64U
#define SNR_TH_Q8      384U
#define NOISE_FREEZE_Q8 320U
#define DIR_RATIO_TH_Q8 384U

/* Stage-2 TDOA preprocess: VAD + 50% overlap frame */
#define TDOA_HALF_N               64U
#define TDOA_FRAME_N              (TDOA_HALF_N * 2U)
#define TDOA_FS_HZ                16000U
#define TDOA_LAG_MAX              8
#define TDOA_VAD_MEANABS_TH       260U
#define TDOA_COARSE_MAX_DEG_X10   750
#define TDOA_MIC_DISTANCE_MM      120.0f
#define TDOA_SOUND_SPEED_MM_S     343000.0f
#define TDOA_PHAT_EPS             1.0e-9f
#define TDOA_PI_F                 3.14159265358979323846f
#define TDOA_LAG_MARGIN_SAMPLES   1
#define TDOA_PROCESS_PERIOD_MS    20U
#define TDOA_STALE_INVALID_MS     300U
/* Stage-4 stabilization: confidence hysteresis + hold + EMA + slew limit */
#define TDOA_CONF_ON_Q8           336U
#define TDOA_CONF_OFF_Q8          272U
#define TDOA_VALID_HOLD_MS        120U
#define TDOA_EMA_ALPHA_Q8         64U
#define TDOA_MAX_STEP_DEG_X10     80
#define TDOA_ANGLE_CLAMP_DEG_X10  900

/* Shared states updated in ISR context */
static volatile uint32_t lvlL = 0U, lvlR = 0U;
static volatile uint32_t peakL = 0U, peakR = 0U;
static volatile uint32_t adcAvgL = 0U, adcAvgR = 0U;
static volatile uint32_t noiseL = 1U, noiseR = 1U;
static volatile uint32_t snrL_q8 = 0U, snrR_q8 = 0U;
static volatile uint8_t noise_ready = 0U;

/* Stage-1 TDOA skeleton runtime states */
static volatile uint8_t tdoa_enabled = 0U;
static volatile mic_tdoa_debug_t tdoa_dbg = {0};
static int16_t tdoa_acc_l[TDOA_HALF_N];
static int16_t tdoa_acc_r[TDOA_HALF_N];
static uint16_t tdoa_acc_idx = 0U;
static int16_t tdoa_ready_l[TDOA_HALF_N];
static int16_t tdoa_ready_r[TDOA_HALF_N];
static volatile uint32_t tdoa_ready_seq = 0U;
static uint32_t tdoa_seen_seq = 0U;
static int16_t tdoa_prev_l[TDOA_HALF_N];
static int16_t tdoa_prev_r[TDOA_HALF_N];
static uint8_t tdoa_prev_valid = 0U;
static uint8_t tdoa_track_valid = 0U;
static uint8_t tdoa_ema_init = 0U;
static int32_t tdoa_ema_angle_x10 = 0;
static uint32_t tdoa_last_good_ms = 0U;
static uint32_t tdoa_last_proc_ms = 0U;
static uint32_t tdoa_last_result_ms = 0U;
static float tdoa_hann[TDOA_FRAME_N];
static uint8_t tdoa_hann_ready = 0U;

/* Direction state */
static char detectLR = '-';
static uint32_t last_switch_ms = 0U;
static volatile uint8_t gate_sound_dbg = 0U;
static volatile uint8_t gate_snr_dbg = 0U;
static volatile uint8_t gate_ratio_dbg = 0U;
static volatile uint32_t diff_dbg = 0U;

/* Ring buffers for O(1) moving average updates */
static uint16_t sig_histL[SIG_WIN];
static uint16_t sig_histR[SIG_WIN];
static uint16_t noise_histL[NOISE_WIN];
static uint16_t noise_histR[NOISE_WIN];
static uint16_t sig_idx = 0U, sig_count = 0U;
static uint16_t noise_idx = 0U, noise_count = 0U;
static uint32_t sig_sumL = 0U, sig_sumR = 0U;
static uint32_t noise_sumL = 0U, noise_sumR = 0U;

static inline uint32_t u32_abs_diff(uint32_t a, uint32_t b)
{
    return (a > b) ? (a - b) : (b - a);
}

static inline uint32_t q8_ratio(uint32_t num, uint32_t den)
{
    if (den == 0U) den = 1U;
    return (num << 8) / den;
}

static inline uint16_t u16_sat_u32(uint32_t v)
{
    return (v > 65535U) ? 65535U : (uint16_t)v;
}

static inline uint32_t u32_abs_i16(int16_t v)
{
    return (v >= 0) ? (uint32_t)v : (uint32_t)(-(int32_t)v);
}

static inline int32_t tdoa_coarse_angle_x10_from_lag(int32_t lag)
{
    if (lag > TDOA_LAG_MAX) lag = TDOA_LAG_MAX;
    if (lag < -TDOA_LAG_MAX) lag = -TDOA_LAG_MAX;
    return (lag * TDOA_COARSE_MAX_DEG_X10) / TDOA_LAG_MAX;
}

static inline int32_t tdoa_clamp_i32(int32_t v, int32_t min_v, int32_t max_v)
{
    if (v < min_v) return min_v;
    if (v > max_v) return max_v;
    return v;
}

static int32_t tdoa_effective_lag_max(void)
{
    /* Physical max delay in samples: fs * d / c, plus small search margin. */
    const float lag_phy = ((float)TDOA_FS_HZ * TDOA_MIC_DISTANCE_MM) / TDOA_SOUND_SPEED_MM_S;
    int32_t lag = (int32_t)(lag_phy + 0.5f) + TDOA_LAG_MARGIN_SAMPLES;
    if (lag < 1) lag = 1;
    if (lag > TDOA_LAG_MAX) lag = TDOA_LAG_MAX;
    return lag;
}

static void tdoa_prepare_hann(void)
{
    if (tdoa_hann_ready != 0U) return;

    const float den = (float)(TDOA_FRAME_N - 1U);
    for (uint32_t n = 0U; n < TDOA_FRAME_N; n++) {
        tdoa_hann[n] = 0.5f - (0.5f * cosf((2.0f * TDOA_PI_F * (float)n) / den));
    }
    tdoa_hann_ready = 1U;
}

static void tdoa_fft_c128_inplace(float *xr, float *xi, uint8_t inverse)
{
    const uint32_t N = TDOA_FRAME_N;
    uint32_t j = 0U;

    /* bit-reversal permutation */
    for (uint32_t i = 1U; i < N; i++) {
        uint32_t bit = (N >> 1U);
        while (j & bit) {
            j ^= bit;
            bit >>= 1U;
        }
        j ^= bit;
        if (i < j) {
            const float tr = xr[i];
            const float ti = xi[i];
            xr[i] = xr[j];
            xi[i] = xi[j];
            xr[j] = tr;
            xi[j] = ti;
        }
    }

    /* radix-2 Cooley-Tukey FFT (O(N log N)) */
    for (uint32_t len = 2U; len <= N; len <<= 1U) {
        const float ang = (inverse != 0U ? 2.0f : -2.0f) * TDOA_PI_F / (float)len;
        const float wlen_r = cosf(ang);
        const float wlen_i = sinf(ang);
        const uint32_t half = (len >> 1U);

        for (uint32_t i = 0U; i < N; i += len) {
            float wr = 1.0f;
            float wi = 0.0f;
            for (uint32_t k = 0U; k < half; k++) {
                const uint32_t u = i + k;
                const uint32_t v = u + half;

                const float tr = (wr * xr[v]) - (wi * xi[v]);
                const float ti = (wr * xi[v]) + (wi * xr[v]);

                const float ur = xr[u];
                const float ui = xi[u];

                xr[u] = ur + tr;
                xi[u] = ui + ti;
                xr[v] = ur - tr;
                xi[v] = ui - ti;

                const float nwr = (wr * wlen_r) - (wi * wlen_i);
                wi = (wr * wlen_i) + (wi * wlen_r);
                wr = nwr;
            }
        }
    }

    if (inverse != 0U) {
        const float inv_n = 1.0f / (float)N;
        for (uint32_t i = 0U; i < N; i++) {
            xr[i] *= inv_n;
            xi[i] *= inv_n;
        }
    }
}

static void tdoa_fft_real_windowed_128(const int16_t *x, const float *win, float *xr, float *xi)
{
    const uint32_t N = TDOA_FRAME_N;
    for (uint32_t n = 0U; n < N; n++) {
        float xn = (float)x[n];
        if (win != NULL) {
            xn *= win[n];
        }
        xr[n] = xn;
        xi[n] = 0.0f;
    }
    tdoa_fft_c128_inplace(xr, xi, 0U);
}

static void tdoa_ifft_phat_to_corr_abs(float *gr, float *gi, float *corr_abs)
{
    tdoa_fft_c128_inplace(gr, gi, 1U);

    for (int32_t lag = -TDOA_LAG_MAX; lag <= TDOA_LAG_MAX; lag++) {
        const uint32_t idx = (lag >= 0) ? (uint32_t)lag : (uint32_t)(TDOA_FRAME_N + lag);
        corr_abs[lag + TDOA_LAG_MAX] = fabsf(gr[idx]);
    }
}

static inline void ring_push_pair(uint16_t *histL,
                                  uint16_t *histR,
                                  uint16_t len,
                                  uint16_t *idx,
                                  uint16_t *count,
                                  uint32_t *sumL,
                                  uint32_t *sumR,
                                  uint16_t vL,
                                  uint16_t vR)
{
    if (*count == len) {
        *sumL -= histL[*idx];
        *sumR -= histR[*idx];
    } else {
        (*count)++;
    }

    histL[*idx] = vL;
    histR[*idx] = vR;
    *sumL += vL;
    *sumR += vR;
    *idx = (uint16_t)((*idx + 1U) % len);
}

static void mic_reset_state(void)
{
    detectLR = '-';
    last_switch_ms = HAL_GetTick();

    lvlL = lvlR = 0U;
    peakL = peakR = 0U;
    adcAvgL = adcAvgR = 0U;
    noiseL = noiseR = 1U;
    snrL_q8 = snrR_q8 = 0U;
    noise_ready = 0U;

    gate_sound_dbg = gate_snr_dbg = gate_ratio_dbg = 0U;
    diff_dbg = 0U;
    tdoa_enabled = 0U;
    tdoa_dbg = (mic_tdoa_debug_t){0};
    tdoa_acc_idx = 0U;
    tdoa_ready_seq = 0U;
    tdoa_seen_seq = 0U;
    tdoa_prev_valid = 0U;
    tdoa_track_valid = 0U;
    tdoa_ema_init = 0U;
    tdoa_ema_angle_x10 = 0;
    tdoa_last_good_ms = 0U;
    tdoa_last_proc_ms = 0U;
    tdoa_last_result_ms = 0U;
    tdoa_hann_ready = 0U;
    memset(tdoa_acc_l, 0, sizeof(tdoa_acc_l));
    memset(tdoa_acc_r, 0, sizeof(tdoa_acc_r));
    memset(tdoa_ready_l, 0, sizeof(tdoa_ready_l));
    memset(tdoa_ready_r, 0, sizeof(tdoa_ready_r));
    memset(tdoa_prev_l, 0, sizeof(tdoa_prev_l));
    memset(tdoa_prev_r, 0, sizeof(tdoa_prev_r));

    sig_idx = sig_count = 0U;
    noise_idx = noise_count = 0U;
    sig_sumL = sig_sumR = 0U;
    noise_sumL = noise_sumR = 0U;

    for (uint32_t i = 0U; i < SIG_WIN; i++) {
        sig_histL[i] = 0U;
        sig_histR[i] = 0U;
    }
    for (uint32_t i = 0U; i < NOISE_WIN; i++) {
        noise_histL[i] = 0U;
        noise_histR[i] = 0U;
    }

#ifdef HAL_I2S_MODULE_ENABLED
    dc_offset_l = 0;
    dc_offset_r = 0;
#endif
}

static void mic_process_interleaved_u16(const uint16_t *buffer, uint32_t total_count)
{
    const uint32_t samples_per_ch = total_count / 2U;

    uint32_t max_magL = 0U, max_magR = 0U;
    uint32_t sum_magL = 0U, sum_magR = 0U;
    uint32_t sum_rawL = 0U, sum_rawR = 0U;

    for (uint32_t i = 0U; i < total_count; i += 2U) {
        const uint32_t rawL = buffer[i];
        const uint32_t rawR = buffer[i + 1U];
        const uint32_t magL = u32_abs_diff(rawL, MIC_MID_U16);
        const uint32_t magR = u32_abs_diff(rawR, MIC_MID_U16);

        sum_rawL += rawL;
        sum_rawR += rawR;
        sum_magL += magL;
        sum_magR += magR;

        if (magL > max_magL) max_magL = magL;
        if (magR > max_magR) max_magR = magR;
    }

    const uint32_t frame_magL = sum_magL / samples_per_ch;
    const uint32_t frame_magR = sum_magR / samples_per_ch;

    adcAvgL = sum_rawL / samples_per_ch;
    adcAvgR = sum_rawR / samples_per_ch;

    ring_push_pair(sig_histL, sig_histR, SIG_WIN,
                   &sig_idx, &sig_count, &sig_sumL, &sig_sumR,
                   (uint16_t)frame_magL, (uint16_t)frame_magR);

    const uint32_t sig_avgL = (sig_count > 0U) ? (sig_sumL / sig_count) : 0U;
    const uint32_t sig_avgR = (sig_count > 0U) ? (sig_sumR / sig_count) : 0U;

    uint8_t freeze_noise = 0U;
    if (noise_count > 0U) {
        const uint32_t cur_noiseL = noise_sumL / noise_count;
        const uint32_t cur_noiseR = noise_sumR / noise_count;
        const uint32_t frame_snrL_q8 = q8_ratio(frame_magL, cur_noiseL);
        const uint32_t frame_snrR_q8 = q8_ratio(frame_magR, cur_noiseR);
        if (frame_snrL_q8 >= NOISE_FREEZE_Q8 || frame_snrR_q8 >= NOISE_FREEZE_Q8) {
            freeze_noise = 1U;
        }
    }

    if (!freeze_noise || noise_count < (NOISE_WIN / 4U)) {
        ring_push_pair(noise_histL, noise_histR, NOISE_WIN,
                       &noise_idx, &noise_count, &noise_sumL, &noise_sumR,
                       (uint16_t)frame_magL, (uint16_t)frame_magR);
    }

    noiseL = (noise_count > 0U) ? (noise_sumL / noise_count) : 1U;
    noiseR = (noise_count > 0U) ? (noise_sumR / noise_count) : 1U;
    noise_ready = (noise_count >= NOISE_WIN) ? 1U : 0U;
    snrL_q8 = q8_ratio(sig_avgL, noiseL);
    snrR_q8 = q8_ratio(sig_avgR, noiseR);

    peakL = max_magL;
    peakR = max_magR;

    lvlL = lvlL + (uint32_t)(((int32_t)sig_avgL - (int32_t)lvlL) / (int32_t)ALPHA_DIV);
    lvlR = lvlR + (uint32_t)(((int32_t)sig_avgR - (int32_t)lvlR) / (int32_t)ALPHA_DIV);
}

void mic_init(void)
{
    mic_reset_state();

#ifdef HAL_I2S_MODULE_ENABLED
    (void)HAL_I2S_Receive_DMA(&hi2s2, i2s_buffer, I2S_DMA_WORD_LEN);
#else
    /* I2S disabled: no input start */
#endif
}

#ifdef HAL_ADC_MODULE_ENABLED
void mic_on_dma_complete(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance != ADC1) return;
    mic_process_interleaved_u16(adc_buffer, ADC_BUF_LEN);
}
#endif

#ifdef HAL_I2S_MODULE_ENABLED
static inline int32_t inmp441_sign_extend24(uint32_t x24)
{
    if ((x24 & 0x00800000U) != 0U) {
        x24 |= 0xFF000000U;
    }
    return (int32_t)x24;
}

static inline int32_t inmp441_unpack_s24(uint16_t hi, uint16_t lo)
{
    /* INMP441: 24-bit signed sample packed in MSB side of 32-bit slot */
    const uint32_t raw32 = ((uint32_t)hi << 16) | (uint32_t)lo;
    const uint32_t x24 = (raw32 >> 8) & 0x00FFFFFFU;
    return inmp441_sign_extend24(x24);
}

static void mic_on_i2s_rx_segment(const uint16_t *src, uint32_t halfword_len)
{
    if (src == NULL || halfword_len < 4U) return;
    if ((halfword_len & 0x3U) != 0U) return;

    static uint16_t frame_u16[I2S_DMA_HALFWORD_LEN / 2U];
    uint32_t out = 0U;

    /* halfword stream layout: [L_hi L_lo R_hi R_lo ...] */
    for (uint32_t i = 0U; (i + 3U) < halfword_len; i += 4U) {
        int32_t l16 = inmp441_unpack_s24(src[i], src[i + 1U]) >> 8;
        int32_t r16 = inmp441_unpack_s24(src[i + 2U], src[i + 3U]) >> 8;

        /* Slowly track DC bias and remove it from each channel. */
        dc_offset_l += (l16 - dc_offset_l) >> DC_TRACK_SHIFT;
        dc_offset_r += (r16 - dc_offset_r) >> DC_TRACK_SHIFT;
        l16 -= dc_offset_l;
        r16 -= dc_offset_r;

        /* Keep ungated samples for TDOA phase path. */
        int32_t l16_tdoa = l16;
        int32_t r16_tdoa = r16;

        /* Gate very small values only for level/direction envelope path. */
        if (l16 > -NOISE_GATE_TH && l16 < NOISE_GATE_TH) l16 = 0;
        if (r16 > -NOISE_GATE_TH && r16 < NOISE_GATE_TH) r16 = 0;

        if (l16 > 32767) l16 = 32767;
        if (l16 < -32768) l16 = -32768;
        if (r16 > 32767) r16 = 32767;
        if (r16 < -32768) r16 = -32768;
        if (l16_tdoa > 32767) l16_tdoa = 32767;
        if (l16_tdoa < -32768) l16_tdoa = -32768;
        if (r16_tdoa > 32767) r16_tdoa = 32767;
        if (r16_tdoa < -32768) r16_tdoa = -32768;

        frame_u16[out++] = (uint16_t)(l16 + 32768);
        frame_u16[out++] = (uint16_t)(r16 + 32768);

        /* Stage-2: collect fixed-size half block for overlap TDOA frame build */
        tdoa_acc_l[tdoa_acc_idx] = (int16_t)l16_tdoa;
        tdoa_acc_r[tdoa_acc_idx] = (int16_t)r16_tdoa;
        tdoa_acc_idx++;
        if (tdoa_acc_idx >= TDOA_HALF_N) {
            memcpy(tdoa_ready_l, tdoa_acc_l, sizeof(tdoa_ready_l));
            memcpy(tdoa_ready_r, tdoa_acc_r, sizeof(tdoa_ready_r));
            tdoa_ready_seq++;
            tdoa_acc_idx = 0U;
        }
    }
    mic_process_interleaved_u16(frame_u16, out);
}

void mic_on_i2s_rx_half_complete(I2S_HandleTypeDef *hi2s)
{
    if (hi2s != &hi2s2) return;
    mic_on_i2s_rx_segment(i2s_buffer, I2S_DMA_HALFWORD_LEN / 2U);
}

void mic_on_i2s_rx_complete(I2S_HandleTypeDef *hi2s)
{
    if (hi2s != &hi2s2) return;
    mic_on_i2s_rx_segment(&i2s_buffer[I2S_DMA_HALFWORD_LEN / 2U], I2S_DMA_HALFWORD_LEN / 2U);
}
#endif

char mic_process(uint32_t now_ms, uint32_t motor_lock_until_ms)
{
    const uint32_t sigL = lvlL;
    const uint32_t sigR = lvlR;
    const uint32_t sL = snrL_q8;
    const uint32_t sR = snrR_q8;
    const uint8_t nReady = noise_ready;

    diff_dbg = u32_abs_diff(sigL, sigR);

    const uint8_t gate_sound = ((sigL >= SOUND_TH || sigR >= SOUND_TH) ? 1U : 0U);
    const uint8_t gate_snr = ((sL >= SNR_TH_Q8 || sR >= SNR_TH_Q8) ? 1U : 0U);
    const uint32_t win = (sL > sR) ? sL : sR;
    const uint32_t lose = (sL > sR) ? sR : sL;
    const uint8_t gate_ratio = (((win << 8) >= (lose * DIR_RATIO_TH_Q8)) ? 1U : 0U);

    gate_sound_dbg = gate_sound;
    gate_snr_dbg = gate_snr;
    gate_ratio_dbg = gate_ratio;

    if ((int32_t)(now_ms - motor_lock_until_ms) >= 0) {
        if (nReady && gate_sound && gate_snr && gate_ratio) {
            const char newDir = (sL > sR) ? 'L' : 'R';
            if (newDir != detectLR && (now_ms - last_switch_ms >= SWITCH_HOLDOFF)) {
                detectLR = newDir;
                last_switch_ms = now_ms;
            }
        }
    }

    return detectLR;
}

uint8_t mic_is_calibrated(void)
{
    return 1U;
}

void mic_get_debug(mic_debug_t *out)
{
    if (out == NULL) return;

    out->adc_avg_l = adcAvgL;
    out->adc_avg_r = adcAvgR;
    out->peak_l = peakL;
    out->peak_r = peakR;
    out->sig_l = lvlL;
    out->sig_r = lvlR;
    out->noise_l = noiseL;
    out->noise_r = noiseR;
    out->snr_l_q8 = snrL_q8;
    out->snr_r_q8 = snrR_q8;
    out->diff = diff_dbg;
    out->gate_sound = gate_sound_dbg;
    out->gate_snr = gate_snr_dbg;
    out->gate_ratio = gate_ratio_dbg;
    out->noise_ready = noise_ready;
    out->detect_dir = detectLR;
}

void mic_tdoa_enable(uint8_t enable)
{
    tdoa_enabled = (enable != 0U) ? 1U : 0U;
    if (tdoa_enabled == 0U) {
        tdoa_dbg.vad_pass = 0U;
        tdoa_dbg.valid = 0U;
        tdoa_track_valid = 0U;
        tdoa_ema_init = 0U;
        tdoa_last_proc_ms = 0U;
        tdoa_last_result_ms = 0U;
    }
}

void mic_tdoa_process(uint32_t now_ms)
{
    static int16_t cur_l[TDOA_HALF_N];
    static int16_t cur_r[TDOA_HALF_N];
    static int16_t frm_l[TDOA_FRAME_N];
    static int16_t frm_r[TDOA_FRAME_N];
    static float x1r[TDOA_FRAME_N], x1i[TDOA_FRAME_N];
    static float x2r[TDOA_FRAME_N], x2i[TDOA_FRAME_N];
    static float gr[TDOA_FRAME_N], gi[TDOA_FRAME_N];
    static float corr_abs[(TDOA_LAG_MAX * 2) + 1];

    if (tdoa_enabled == 0U) {
        tdoa_dbg.vad_pass = 0U;
        tdoa_dbg.valid = 0U;
        return;
    }

    /* Stage-8: throttle heavy GCC-PHAT cadence to reduce CPU load. */
    if ((now_ms - tdoa_last_proc_ms) < TDOA_PROCESS_PERIOD_MS) {
        if ((tdoa_dbg.valid != 0U) && ((now_ms - tdoa_last_result_ms) > TDOA_STALE_INVALID_MS)) {
            tdoa_dbg.valid = 0U;
            tdoa_track_valid = 0U;
            tdoa_ema_init = 0U;
        }
        return;
    }
    tdoa_last_proc_ms = now_ms;

    tdoa_prepare_hann();

    uint32_t seq0 = 0U;
    uint32_t seq1 = 0U;
    do {
        seq0 = tdoa_ready_seq;
        if (seq0 == tdoa_seen_seq) {
            return; /* no new half block yet */
        }
        memcpy(cur_l, tdoa_ready_l, sizeof(cur_l));
        memcpy(cur_r, tdoa_ready_r, sizeof(cur_r));
        seq1 = tdoa_ready_seq;
    } while (seq0 != seq1);
    tdoa_seen_seq = seq1;

    if (tdoa_prev_valid == 0U) {
        memcpy(tdoa_prev_l, cur_l, sizeof(tdoa_prev_l));
        memcpy(tdoa_prev_r, cur_r, sizeof(tdoa_prev_r));
        tdoa_prev_valid = 1U;
        tdoa_dbg.valid = 0U;
        return;
    }

    memcpy(frm_l, tdoa_prev_l, sizeof(tdoa_prev_l));
    memcpy(frm_r, tdoa_prev_r, sizeof(tdoa_prev_r));
    memcpy(&frm_l[TDOA_HALF_N], cur_l, sizeof(cur_l));
    memcpy(&frm_r[TDOA_HALF_N], cur_r, sizeof(cur_r));
    memcpy(tdoa_prev_l, cur_l, sizeof(tdoa_prev_l));
    memcpy(tdoa_prev_r, cur_r, sizeof(tdoa_prev_r));

    uint32_t mean_abs = 0U;
    for (uint32_t i = 0U; i < TDOA_FRAME_N; i++) {
        mean_abs += u32_abs_i16(frm_l[i]);
        mean_abs += u32_abs_i16(frm_r[i]);
    }
    mean_abs /= (2U * TDOA_FRAME_N);

    tdoa_dbg.vad_pass = (mean_abs >= TDOA_VAD_MEANABS_TH) ? 1U : 0U;
    if (tdoa_dbg.vad_pass == 0U) {
        tdoa_dbg.valid = 0U;
        tdoa_dbg.lag_samples = 0;
        tdoa_dbg.tau_us = 0;
        tdoa_dbg.alpha_deg_x10 = 0;
        tdoa_dbg.peak_main = 0U;
        tdoa_dbg.peak_second = 0U;
        tdoa_dbg.confidence_q8 = 0U;
        return;
    }

    /* Stage-3: GCC-PHAT pipeline (FFT -> PHAT normalize -> IFFT -> lag scan) */
    tdoa_fft_real_windowed_128(frm_l, tdoa_hann, x1r, x1i);
    tdoa_fft_real_windowed_128(frm_r, tdoa_hann, x2r, x2i);

    const int32_t lag_max = tdoa_effective_lag_max();
    for (uint32_t k = 0U; k < TDOA_FRAME_N; k++) {
        /* G = X1 * conj(X2) */
        const float cr = x1r[k] * x2r[k] + x1i[k] * x2i[k];
        const float ci = x1i[k] * x2r[k] - x1r[k] * x2i[k];
        const float mag = sqrtf(cr * cr + ci * ci) + TDOA_PHAT_EPS;
        gr[k] = cr / mag;
        gi[k] = ci / mag;
    }

    tdoa_ifft_phat_to_corr_abs(gr, gi, corr_abs);

    int32_t best_lag = 0;
    float best_peak = 0.0f;
    float second_peak = 0.0f;

    for (int32_t lag = -lag_max; lag <= lag_max; lag++) {
        const float a = corr_abs[lag + TDOA_LAG_MAX];

        if (a > best_peak) {
            second_peak = best_peak;
            best_peak = a;
            best_lag = lag;
        } else if (a > second_peak) {
            second_peak = a;
        }
    }

    float p = 0.0f;
    if ((best_lag > -lag_max) && (best_lag < lag_max)) {
        const float a = corr_abs[best_lag - 1 + TDOA_LAG_MAX];
        const float b = corr_abs[best_lag + TDOA_LAG_MAX];
        const float c = corr_abs[best_lag + 1 + TDOA_LAG_MAX];
        const float den = (a - (2.0f * b) + c);
        if (fabsf(den) > 1.0e-9f) {
            p = 0.5f * (a - c) / den;
            if (p > 0.5f) p = 0.5f;
            if (p < -0.5f) p = -0.5f;
        }
    }

    const float lag_f = (float)best_lag + p;
    const float tau_us_f = lag_f * (1000000.0f / (float)TDOA_FS_HZ);
    const float tau_max_us = (TDOA_MIC_DISTANCE_MM / TDOA_SOUND_SPEED_MM_S) * 1000000.0f;
    float x = tau_us_f / tau_max_us;
    if (x > 1.0f) x = 1.0f;
    if (x < -1.0f) x = -1.0f;
    const float alpha_rad = asinf(x);
    const float alpha_deg_x10_f = alpha_rad * (1800.0f / TDOA_PI_F);
    const int32_t alpha_raw_x10 = (int32_t)alpha_deg_x10_f;

    tdoa_dbg.lag_samples = best_lag;
    tdoa_dbg.tau_us = (int32_t)tau_us_f;
    tdoa_dbg.alpha_deg_x10 = alpha_raw_x10;

    /* compress peaks for log readability */
    const uint32_t p_main = (uint32_t)(best_peak * 1000.0f);
    const uint32_t p_second = (uint32_t)(second_peak * 1000.0f);
    tdoa_dbg.peak_main = u16_sat_u32(p_main);
    tdoa_dbg.peak_second = u16_sat_u32(p_second);

    const float conf_q8_f = (best_peak * 256.0f) / (second_peak + 1.0e-6f);
    const uint32_t conf_q8 = (conf_q8_f >= 65535.0f) ? 65535U : (uint32_t)conf_q8_f;
    tdoa_dbg.confidence_q8 = (uint16_t)conf_q8;

    /* keep previous coarse fallback compatibility if angle is NaN */
    if (!isfinite((double)alpha_deg_x10_f)) {
        tdoa_dbg.alpha_deg_x10 = tdoa_coarse_angle_x10_from_lag(best_lag);
    }

    /* Stage-4: confidence hysteresis + hold window */
    uint8_t conf_good = 0U;
    if (tdoa_track_valid != 0U) {
        conf_good = (tdoa_dbg.confidence_q8 >= TDOA_CONF_OFF_Q8) ? 1U : 0U;
    } else {
        conf_good = (tdoa_dbg.confidence_q8 >= TDOA_CONF_ON_Q8) ? 1U : 0U;
    }

    if ((tdoa_dbg.vad_pass != 0U) && (conf_good != 0U)) {
        tdoa_last_good_ms = now_ms;
        if (tdoa_track_valid == 0U) {
            tdoa_track_valid = 1U;
            tdoa_ema_init = 0U;
        }
    } else if (tdoa_track_valid != 0U) {
        if ((now_ms - tdoa_last_good_ms) > TDOA_VALID_HOLD_MS) {
            tdoa_track_valid = 0U;
        }
    }

    /* Stage-4: angle smoothing (slew-limited EMA) */
    if (tdoa_track_valid != 0U) {
        int32_t raw = tdoa_dbg.alpha_deg_x10;
        raw = tdoa_clamp_i32(raw, -TDOA_ANGLE_CLAMP_DEG_X10, TDOA_ANGLE_CLAMP_DEG_X10);

        if (tdoa_ema_init == 0U) {
            tdoa_ema_angle_x10 = raw;
            tdoa_ema_init = 1U;
        } else if ((tdoa_dbg.vad_pass != 0U) && (conf_good != 0U)) {
            int32_t delta = raw - tdoa_ema_angle_x10;
            delta = tdoa_clamp_i32(delta, -TDOA_MAX_STEP_DEG_X10, TDOA_MAX_STEP_DEG_X10);
            tdoa_ema_angle_x10 += (delta * (int32_t)TDOA_EMA_ALPHA_Q8) / 256;
        }

        tdoa_dbg.alpha_deg_x10 = tdoa_ema_angle_x10;
        tdoa_dbg.valid = 1U;
        tdoa_last_result_ms = now_ms;
    } else {
        tdoa_dbg.valid = 0U;
    }
}

uint8_t mic_tdoa_is_valid(void)
{
    if (tdoa_enabled == 0U) return 0U;
    return tdoa_dbg.valid;
}

void mic_get_tdoa_debug(mic_tdoa_debug_t *out)
{
    if (out == NULL) return;
    *out = tdoa_dbg;
}

