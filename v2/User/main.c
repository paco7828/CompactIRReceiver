/*
 * CH32V003A4M6 - IR receiver / logger  (MounRiver Studio II)
 * PA1 IRM3638 | PC1 SDA | PC2 SCL | PC3 CONFIRM | PC4 CANCEL
 * PC6 SD CMD | PC7 SD DAT0 | PD4 SD CLK | PD6 SD DAT3(CS) | PD1 SWIO (untouched)
 * buf[512] = OLED framebuffer AND SD sector buffer (shared, RAM is 2 KB)
 */
#include "ch32v00x.h"
#include <string.h>

extern uint32_t SystemCoreClock;

#define RAW_MAX 320
#define MIN_ENTRIES 12
#define IR_GAP_US 15000
#define OLED_ADDR 0x3C
#define DEBOUNCE_MS 40
#define ANIM_MS 400
#define FEEDBACK_MS 3000
#define FAT_DATE 0x5C21

static uint32_t tpms;

static inline uint32_t now (void) { return SysTick->CNT; }

static uint8_t elapsed (uint32_t t0, uint32_t ms) { return (uint32_t)(now() - t0) >= ms * tpms; }

static void delay_ms (uint32_t ms) {
    uint32_t t0 = now();
    while (!elapsed (t0, ms));
}

static void delay_us (uint32_t us) {
    uint32_t t0 = now(), n = us * (tpms / 1000);
    while ((uint32_t)(now() - t0) < n);
}

/* ---------- IR RX ---------- */
static volatile uint16_t raw[RAW_MAX];
static volatile uint16_t rawn, rx_last;
static volatile uint8_t rx_started;

void EXTI7_0_IRQHandler (void) __attribute__ ((interrupt ("WCH-Interrupt-fast")));

void EXTI7_0_IRQHandler (void) {
    uint16_t t = TIM2->CNT;
    uint8_t low = !(GPIOA->INDR & (1u << 1));
    EXTI->INTFR = (1u << 1);
    if (!rx_started) {
        if (low) {
            rx_started = 1;
            rx_last = t;
            rawn = 0;
        }
    } else {
        uint16_t d = t - rx_last;
        rx_last = t;
        if (rawn < RAW_MAX)
            raw[rawn++] = d;
    }
}

static void rx_start (void) {
    rx_started = 0;
    rawn = 0;
    EXTI->INTFR = (1u << 1);
    EXTI->INTENR |= (1u << 1);
}

static void rx_stop (void) { EXTI->INTENR &= ~(1u << 1); }

static uint8_t rx_poll (void) {
    uint16_t l, c;
    if (!rx_started)
        return 0;
    l = rx_last;
    c = TIM2->CNT;
    if ((uint16_t)(c - l) < IR_GAP_US)
        return 0;
    rx_stop();
    if (rawn >= MIN_ENTRIES)
        return 1;
    rx_start();
    return 0;
}

static uint8_t cap_digits;

static uint32_t decode_ir (void) {
    uint16_t n = rawn, s = 0, i, mn[2] = {0xFFFF, 0xFFFF}, mx[2] = {0, 0};
    uint8_t k, nb = 0;
    uint32_t v = 0;

    if (raw[0] > 2000)
        s = 2;
    for (i = s; i < n; i++) {
        uint16_t d = raw[i];
        k = (i - s) & 1;
        if (d < mn[k])
            mn[k] = d;
        if (d > mx[k])
            mx[k] = d;
    }
    if (n > s + 3) {
        if (mx[1] > mn[1] + (mn[1] >> 1))
            k = 1;
        else if (mx[0] > mn[0] + (mn[0] >> 1))
            k = 0;
        else
            k = 2;
        if (k < 2) {
            uint16_t thr = mn[k] + (mn[k] >> 1);
            for (i = s + k; i < n; i += 2) {
                v = (v << 1) | (raw[i] > thr);
                nb++;
            }
            cap_digits = (nb >= 32) ? 8 : 2 * ((nb + 7) >> 3);
            return v;
        }
    }
    {
        uint16_t base = 0xFFFF;
        v = 2166136261u;
        for (i = 0; i < n; i++)
            if (raw[i] < base)
                base = raw[i];
        if (base < 50)
            base = 50;
        for (i = 0; i < n; i++) {
            uint32_t q = ((uint32_t)raw[i] * 2 + base) / (2u * base);
            v = (v ^ q) * 16777619u;
        }
    }
    cap_digits = 8;
    return v;
}

/* ---------- GPIO ---------- */
static void gpio_cfg (GPIO_TypeDef *g, uint8_t pin, uint8_t cfg) {
    g->CFGLR = (g->CFGLR & ~(0xFu << (pin * 4))) | ((uint32_t)cfg << (pin * 4));
}

#define CFG_IPU 0x8
#define CFG_OUT 0x1
#define CFG_AFOD 0xD

/* ---------- OLED (I2C1) ---------- */
static uint8_t buf[512];
#define fb buf

static void i2c_recover (void) {
    uint8_t i;
    I2C1->CTLR1 = 0;                                        /* PE off */
    gpio_cfg (GPIOC, 1, 0x5);
    gpio_cfg (GPIOC, 2, 0x5);                               /* GPIO open-drain */
    GPIOC->BSHR = (1u << 1) | (1u << 2);                    /* release SDA, SCL */
    delay_us (10);
    for (i = 0; i < 9 && !(GPIOC->INDR & (1u << 1)); i++) { /* SDA stuck low */
        GPIOC->BCR = 1u << 2;
        delay_us (10);
        GPIOC->BSHR = 1u << 2;
        delay_us (10);
    }
    GPIOC->BCR = 1u << 1;
    delay_us (10); /* STOP condition */
    GPIOC->BSHR = 1u << 1;
    delay_us (10);
    gpio_cfg (GPIOC, 1, CFG_AFOD);
    gpio_cfg (GPIOC, 2, CFG_AFOD);
    I2C1->CTLR2 = SystemCoreClock / 1000000;
    I2C1->CKCFGR = 0x8000 | (SystemCoreClock / (3 * 400000));
    I2C1->CTLR1 = 1;
}

static uint8_t i2c_wait (uint16_t mask) {
    uint16_t t = 60000;
    while (!(I2C1->STAR1 & mask))
        if (!--t)
            return 0;
    return 1;
}

static uint8_t i2c_begin (void) {
    uint16_t t = 60000;
    while (I2C1->STAR2 & 2)
        if (!--t)
            return 0;
    I2C1->CTLR1 |= (1u << 8);
    if (!i2c_wait (1u << 0))
        goto fail;
    I2C1->DATAR = OLED_ADDR << 1;
    if (!i2c_wait (1u << 1))
        goto fail;
    (void)I2C1->STAR2;
    return 1;
fail:
    I2C1->CTLR1 |= (1u << 9);
    return 0;
}

static void i2c_tx (uint8_t d) {
    if (i2c_wait (1u << 7))
        I2C1->DATAR = d;
}

static void i2c_stop (void) {
    i2c_wait (1u << 2);
    I2C1->CTLR1 |= (1u << 9);
}

static uint8_t oled_cmds (const uint8_t *c, uint8_t n) {
    if (!i2c_begin()) {
        i2c_recover();
        return 0;
    }
    i2c_tx (0x00);
    while (n--) i2c_tx (*c++);
    i2c_stop();
    return 1;
}

static void oled_init (void) {
    static const uint8_t init[] = {
        0xAE, 0xD5, 0x80, 0xA8, 0x1F, 0xD3, 0x00, 0x40, 0x8D, 0x14, 0x20, 0x00,
        0xA1, 0xC8, 0xDA, 0x02, 0x81, 0x8F, 0xD9, 0xF1, 0xDB, 0x40, 0xA4, 0xA6, 0xAF};
    uint8_t i;
    for (i = 0; i < 5; i++) {
        i2c_recover();
        if (oled_cmds (init, sizeof (init)))
            break;
        delay_ms (50);
    }
    delay_ms (10);
    oled_cmds (init, sizeof (init)); /* second pass: harmless, catches a late reset release */
}

static void oled_flush (void) {
    static const uint8_t win[] = {0x21, 0, 127, 0x22, 0, 3};
    uint16_t i;
    oled_cmds (win, sizeof (win));
    if (!i2c_begin())
        return;
    i2c_tx (0x40);
    for (i = 0; i < 512; i++) i2c_tx (fb[i]);
    i2c_stop();
}

/* ---------- graphics ---------- */
static const char FCH[] = " !.:0123456789ABCDEFILNRSTXacdeghilnorstvwx";
static const uint8_t FNT[][5] = {
    {0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x5F, 0x00, 0x00},
    {0x00, 0x00, 0x60, 0x60, 0x00},
    {0x00, 0x36, 0x36, 0x00, 0x00},
    {0x3E, 0x51, 0x49, 0x45, 0x3E},
    {0x00, 0x42, 0x7F, 0x40, 0x00},
    {0x42, 0x61, 0x51, 0x49, 0x46},
    {0x21, 0x41, 0x45, 0x4B, 0x31},
    {0x18, 0x14, 0x12, 0x7F, 0x10},
    {0x27, 0x45, 0x45, 0x45, 0x39},
    {0x3C, 0x4A, 0x49, 0x49, 0x30},
    {0x01, 0x71, 0x09, 0x05, 0x03},
    {0x36, 0x49, 0x49, 0x49, 0x36},
    {0x06, 0x49, 0x49, 0x29, 0x1E},
    /* A B C D E F I L N R S T X */
    {0x7E, 0x11, 0x11, 0x11, 0x7E},
    {0x7F, 0x49, 0x49, 0x49, 0x36},
    {0x3E, 0x41, 0x41, 0x41, 0x22},
    {0x7F, 0x41, 0x41, 0x41, 0x3E},
    {0x7F, 0x49, 0x49, 0x49, 0x41},
    {0x7F, 0x09, 0x09, 0x09, 0x01},
    {0x00, 0x41, 0x7F, 0x41, 0x00},
    {0x7F, 0x40, 0x40, 0x40, 0x40},
    {0x7F, 0x04, 0x08, 0x10, 0x7F},
    {0x7F, 0x09, 0x19, 0x29, 0x46},
    {0x46, 0x49, 0x49, 0x49, 0x31},
    {0x03, 0x01, 0x7F, 0x01, 0x03},
    {0x63, 0x14, 0x08, 0x14, 0x63},
    /* a c d e g h i l n o r s t v w x */
    {0x20, 0x54, 0x54, 0x54, 0x78},
    {0x38, 0x44, 0x44, 0x44, 0x20},
    {0x38, 0x44, 0x44, 0x48, 0x7F},
    {0x38, 0x54, 0x54, 0x54, 0x18},
    {0x18, 0xA4, 0xA4, 0xA4, 0x7C},
    {0x7F, 0x08, 0x04, 0x04, 0x78},
    {0x00, 0x44, 0x7D, 0x40, 0x00},
    {0x00, 0x41, 0x7F, 0x40, 0x00},
    {0x7C, 0x08, 0x04, 0x04, 0x78},
    {0x38, 0x44, 0x44, 0x44, 0x38},
    {0x7C, 0x08, 0x04, 0x04, 0x08},
    {0x48, 0x54, 0x54, 0x54, 0x24},
    {0x04, 0x3F, 0x44, 0x40, 0x20},
    {0x1C, 0x20, 0x40, 0x20, 0x1C},
    {0x3C, 0x40, 0x30, 0x40, 0x3C},
    {0x44, 0x28, 0x10, 0x28, 0x44}
};

static void px (int16_t x, int16_t y) {
    if ((uint16_t)x < 128 && (uint16_t)y < 32)
        fb[x + ((y >> 3) << 7)] |= 1u << (y & 7);
}

static void glyph (int16_t x, int16_t y, char c, uint8_t sz) {
    const char *p = FCH;
    uint8_t i = 0, col, row, dx, dy, b;
    while (*p && *p != c) {
        p++;
        i++;
    }
    if (!*p)
        return;
    for (col = 0; col < 5; col++) {
        b = FNT[i][col];
        for (row = 0; row < 8; row++)
            if (b & (1u << row))
                for (dy = 0; dy < sz; dy++)
                    for (dx = 0; dx < sz; dx++) px (x + col * sz + dx, y + row * sz + dy);
    }
}

static void text_c (int16_t y, const char *s, uint8_t sz, uint8_t gap) {
    uint8_t adv = 5 * sz + gap, n = strlen (s);
    int16_t x = (128 - (n * adv - gap)) / 2;
    while (*s) {
        glyph (x, y, *s++, sz);
        x += adv;
    }
}

static void circle (int16_t cx, int16_t cy, int16_t r, uint8_t fill) {
    int16_t dx, dy, o = r * r + r, in = (r - 1) * (r - 1) + (r - 1);
    for (dy = -r; dy <= r; dy++)
        for (dx = -r; dx <= r; dx++) {
            int16_t d = dx * dx + dy * dy;
            if (d <= o && (fill || d > in))
                px (cx + dx, cy + dy);
        }
}

static void line (int16_t x0, int16_t y0, int16_t x1, int16_t y1) {
    int16_t dx = x1 > x0 ? x1 - x0 : x0 - x1, dy = y1 > y0 ? y0 - y1 : y1 - y0;
    int16_t sx = x0 < x1 ? 1 : -1, sy = y0 < y1 ? 1 : -1, e = dx + dy, e2;
    for (;;) {
        px (x0, y0);
        if (x0 == x1 && y0 == y1)
            break;
        e2 = 2 * e;
        if (e2 >= dy) {
            e += dy;
            x0 += sx;
        }
        if (e2 <= dx) {
            e += dx;
            y0 += sy;
        }
    }
}

static void icon_check (int16_t x, int16_t y) {
    uint8_t k;
    for (k = 0; k < 2; k++) {
        line (x, y + 4 + k, x + 2, y + 6 + k);
        line (x + 2, y + 6 + k, x + 7, y + 1 + k);
    }
}

static void icon_cross (int16_t x, int16_t y) {
    uint8_t k;
    for (k = 0; k < 2; k++) {
        line (x + k, y, x + 6 + k, y + 6);
        line (x + 6 + k, y, x + k, y + 6);
    }
}

/* ---------- screens ---------- */
static void scr_title (void) {
    memset (fb, 0, 512);
    text_c (8, "IR Receiver", 2, 1);
    oled_flush();
}

static void scr_listening (uint8_t st) {
    uint8_t i;
    memset (fb, 0, 512);
    text_c (3, "Listening", 1, 1);
    for (i = 0; i < 3; i++) circle (48 + i * 16, 22, 4, i == st);
    oled_flush();
}

static void scr_msg (const char *s) {
    memset (fb, 0, 512);
    text_c (8, s, 2, 2);
    oled_flush();
}

static uint32_t cap_val;

static void scr_captured (void) {
    char h[11], l[20];
    uint8_t i, d = cap_digits, p = 0;
    uint16_t n = rawn;

    h[0] = '0';
    h[1] = 'x';
    for (i = 0; i < d; i++) {
        uint8_t nib = (cap_val >> ((d - 1 - i) * 4)) & 0xF;
        h[2 + i] = nib < 10 ? '0' + nib : 'A' + nib - 10;
    }
    h[2 + d] = 0;

    strcpy (l, "raw length: ");
    p = 12;
    if (n >= 100)
        l[p++] = '0' + n / 100;
    if (n >= 10)
        l[p++] = '0' + (n / 10) % 10;
    l[p++] = '0' + n % 10;
    l[p] = 0;

    memset (fb, 0, 512);
    text_c (2, h, 2, 2);
    text_c (19, l, 1, 1);
    icon_check (0, 24);
    icon_cross (120, 24);
    oled_flush();
}

/* ---------- SD (bit-banged SPI mode) ---------- */
#define CS_H() (GPIOD->BSHR = (1u << 6))
#define CS_L() (GPIOD->BCR = (1u << 6))
#define SCK_H() (GPIOD->BSHR = (1u << 4))
#define SCK_L() (GPIOD->BCR = (1u << 4))
#define MOSI_H() (GPIOC->BSHR = (1u << 6))
#define MOSI_L() (GPIOC->BCR = (1u << 6))
#define MISO() ((GPIOC->INDR >> 7) & 1u)
#define ACMD(n) (0x80 | (n))

static uint8_t spi_slow, sd_hc, sd_err; /* sd_err: 1 = read failed, 2 = write failed */
#define SD_DLY 3                        /* data phase ~1 MHz */

static inline void spi_dly (void) {
    uint8_t n = spi_slow;
    while (n--) __asm__ volatile ("nop");
}

static uint8_t spi_xfer (uint8_t d) {
    uint8_t i;
    for (i = 0; i < 8; i++) {
        if (d & 0x80)
            MOSI_H();
        else
            MOSI_L();
        d <<= 1;
        spi_dly();
        SCK_H();
        spi_dly();
        d |= MISO();
        SCK_L();
    }
    return d;
}

static void sd_end (void) {
    CS_H();
    spi_xfer (0xFF);
}

static uint8_t sd_wait (void) {
    uint32_t t0 = now();
    do {
        if (spi_xfer (0xFF) == 0xFF)
            return 1;
    } while (!elapsed (t0, 500));
    return 0;
}

static uint8_t sd_cmd (uint8_t c, uint32_t a) {
    uint8_t r, n;
    if (c & 0x80) {
        c &= 0x7F;
        r = sd_cmd (55, 0);
        if (r > 1)
            return r;
    }
    CS_H();
    spi_xfer (0xFF);
    CS_L();
    spi_xfer (0xFF);
    if (!sd_wait())
        return 0xFF;
    spi_xfer (0x40 | c);
    spi_xfer (a >> 24);
    spi_xfer (a >> 16);
    spi_xfer (a >> 8);
    spi_xfer (a);
    spi_xfer (c == 0 ? 0x95 : c == 8 ? 0x87
                                     : 0x01);
    for (n = 10; n; n--) {
        r = spi_xfer (0xFF);
        if (!(r & 0x80))
            break;
    }
    return r;
}

static uint8_t sd_init (void) {
    uint8_t i, r, o[4];
    uint32_t t0;
    sd_hc = 0;
    spi_slow = 20;
    CS_H();
    for (i = 0; i < 10; i++) spi_xfer (0xFF);
    if (sd_cmd (0, 0) != 1)
        goto fail;
    t0 = now();
    if (sd_cmd (8, 0x1AA) == 1) {
        for (i = 0; i < 4; i++) o[i] = spi_xfer (0xFF);
        if (o[2] != 1 || o[3] != 0xAA)
            goto fail;
        while ((r = sd_cmd (ACMD (41), 1UL << 30)) != 0)
            if (r > 1 || elapsed (t0, 1000))
                goto fail;
        if (sd_cmd (58, 0) != 0)
            goto fail;
        for (i = 0; i < 4; i++) o[i] = spi_xfer (0xFF);
        sd_hc = (o[0] & 0x40) != 0;
    } else {
        while ((r = sd_cmd (ACMD (41), 0)) != 0)
            if (r > 1 || elapsed (t0, 1000))
                goto fail;
        if (sd_cmd (16, 512) != 0)
            goto fail;
    }
    sd_end();
    spi_slow = SD_DLY;
    return 1;
fail:
    sd_end();
    return 0;
}

static uint8_t sd_read (uint32_t lba, uint8_t *b) {
    uint16_t i;
    uint32_t t0;
    if (!sd_hc)
        lba <<= 9;
    if (sd_cmd (17, lba) != 0)
        goto fail;
    t0 = now();
    do { i = spi_xfer (0xFF); } while (i == 0xFF && !elapsed (t0, 200));
    if (i != 0xFE)
        goto fail;
    for (i = 0; i < 512; i++) b[i] = spi_xfer (0xFF);
    spi_xfer (0xFF);
    spi_xfer (0xFF);
    sd_end();
    return 1;
fail:
    sd_end();
    sd_err = 1;
    return 0;
}

static uint8_t sd_write (uint32_t lba, const uint8_t *b) {
    uint16_t i;
    if (!sd_hc)
        lba <<= 9;
    if (sd_cmd (24, lba) != 0)
        goto fail;
    spi_xfer (0xFF);
    spi_xfer (0xFE);
    for (i = 0; i < 512; i++) spi_xfer (b[i]);
    spi_xfer (0xFF);
    spi_xfer (0xFF);
    if ((spi_xfer (0xFF) & 0x1F) != 0x05)
        goto fail;
    if (!sd_wait())
        goto fail;
    sd_end();
    return 1;
fail:
    sd_end();
    sd_err = 2;
    return 0;
}

/* ---------- minimal FAT32 (root dir, 8.3 names) ---------- */
static uint32_t fat_lba, data_lba, root_cl, fat_sz, max_cl, free_hint;
static uint8_t spc, nfats;

static uint16_t rd16 (const uint8_t *p) { return p[0] | (p[1] << 8); }

static uint32_t rd32 (const uint8_t *p) { return rd16 (p) | ((uint32_t)rd16 (p + 2) << 16); }

static void wr16 (uint8_t *p, uint16_t v) {
    p[0] = v;
    p[1] = v >> 8;
}

static void wr32 (uint8_t *p, uint32_t v) {
    wr16 (p, v);
    wr16 (p + 2, v >> 16);
}

static uint32_t cl_lba (uint32_t cl) { return data_lba + (cl - 2) * spc; }

enum { FM_OK,
       FM_IO,
       FM_FMT };

static uint8_t is_fat32_vbr (void) {
    return rd16 (buf + 510) == 0xAA55 && buf[0x52] == 'F' && buf[0x53] == 'A' && buf[0x54] == 'T' &&
           buf[0x55] == '3' && buf[0x56] == '2';
}

static uint8_t fat_mount (void) {
    uint32_t base = 0, tot, cand[4];
    uint8_t i;
    if (!sd_read (0, buf))
        return FM_IO;
    if (!is_fat32_vbr()) {
        if (rd16 (buf + 510) != 0xAA55)
            return FM_FMT;
        for (i = 0; i < 4; i++) cand[i] = buf[0x1BE + i * 16 + 4] ? rd32 (buf + 0x1BE + i * 16 + 8) : 0;
        for (i = 0; i < 4; i++) {
            if (!cand[i])
                continue;
            if (!sd_read (cand[i], buf))
                return FM_IO;
            if (is_fat32_vbr()) {
                base = cand[i];
                break;
            }
        }
        if (i == 4)
            return FM_FMT;
    }
    if (rd16 (buf + 11) != 512 || rd16 (buf + 17) != 0)
        return FM_FMT;
    spc = buf[13];
    nfats = buf[16];
    fat_sz = rd32 (buf + 36);
    root_cl = rd32 (buf + 44);
    fat_lba = base + rd16 (buf + 14);
    data_lba = fat_lba + nfats * fat_sz;
    tot = rd16 (buf + 19) ? rd16 (buf + 19) : rd32 (buf + 32);
    if (!spc || !nfats || !fat_sz || root_cl < 2)
        return FM_FMT;
    max_cl = (tot - (data_lba - base)) / spc + 2;
    free_hint = 2;
    return FM_OK;
}

static uint32_t fat_get (uint32_t cl) {
    if (!sd_read (fat_lba + (cl >> 7), buf))
        return 0x0FFFFFFFu;
    return rd32 (buf + ((cl & 127) << 2)) & 0x0FFFFFFFu;
}

static uint8_t fat_set (uint32_t cl, uint32_t v) {
    uint32_t lba = fat_lba + (cl >> 7);
    uint16_t o = (cl & 127) << 2;
    uint8_t k;
    if (!sd_read (lba, buf))
        return 0;
    wr32 (buf + o, (rd32 (buf + o) & 0xF0000000u) | (v & 0x0FFFFFFFu));
    for (k = 0; k < nfats; k++)
        if (!sd_write (lba + k * fat_sz, buf))
            return 0;
    return 1;
}

static uint32_t alloc_cluster (void) {
    uint32_t cl, s, cur = 0xFFFFFFFFu;
    uint8_t pass;
    for (pass = 0; pass < 2; pass++)
        for (cl = pass ? 2 : free_hint; cl < max_cl; cl++) {
            s = cl >> 7;
            if (s != cur) {
                if (!sd_read (fat_lba + s, buf))
                    return 0;
                cur = s;
            }
            if ((rd32 (buf + ((cl & 127) << 2)) & 0x0FFFFFFFu) == 0) {
                free_hint = cl + 1;
                return cl;
            }
        }
    return 0;
}

static uint32_t next_idx, sl_lba;
static uint16_t sl_off;

static uint8_t dir_scan (void) {
    uint32_t cl = root_cl, last = root_cl, lba, mx = 0, v;
    uint8_t s, k, found = 0, end = 0;
    uint16_t o;
    while (!end && cl >= 2 && cl < 0x0FFFFFF8u) {
        for (s = 0; s < spc && !end; s++) {
            lba = cl_lba (cl) + s;
            if (!sd_read (lba, buf))
                return 0;
            for (o = 0; o < 512; o += 32) {
                uint8_t *e = buf + o;
                if (e[0] == 0x00 || e[0] == 0xE5) {
                    if (!found) {
                        found = 1;
                        sl_lba = lba;
                        sl_off = o;
                    }
                    if (e[0] == 0x00) {
                        end = 1;
                        break;
                    }
                    continue;
                }
                if (e[0] == 'S' && e[1] == 'I' && e[2] == 'G' && e[8] == 'T' && e[9] == 'X' &&
                    e[10] == 'T' && (e[11] & 0x0F) != 0x0F) {
                    v = 0;
                    for (k = 3; k < 8 && e[k] >= '0' && e[k] <= '9'; k++) v = v * 10 + (e[k] - '0');
                    if (k == 8 && v > mx)
                        mx = v;
                }
            }
        }
        if (!end) {
            last = cl;
            cl = fat_get (cl);
        }
    }
    next_idx = mx + 1;
    if (!found) {
        cl = alloc_cluster();
        if (!cl || !fat_set (cl, 0x0FFFFFFFu) || !fat_set (last, cl))
            return 0;
        memset (buf, 0, 512);
        for (s = 0; s < spc; s++)
            if (!sd_write (cl_lba (cl) + s, buf))
                return 0;
        sl_lba = cl_lba (cl);
        sl_off = 0;
    }
    return next_idx <= 99999;
}

static uint32_t w_lba, w_size;
static uint16_t w_pos;
static uint8_t w_sec, w_err;

static void emit (uint8_t c) {
    if (w_sec >= spc)
        return; /* one cluster max: truncate */
    buf[w_pos++] = c;
    w_size++;
    if (w_pos == 512) {
        if (!sd_write (w_lba + w_sec, buf))
            w_err = 1;
        w_sec++;
        w_pos = 0;
    }
}

static void emit_dec (uint32_t v) {
    char t[6];
    uint8_t n = 0;
    do {
        t[n++] = '0' + v % 10;
        v /= 10;
    } while (v);
    while (n) emit (t[--n]);
}

static uint8_t file_write (uint32_t cl) {
    uint16_t i;
    uint8_t d;
    w_lba = cl_lba (cl);
    w_sec = 0;
    w_pos = 0;
    w_size = 0;
    w_err = 0;
    for (d = cap_digits; d; d--) {
        uint8_t nib = (cap_val >> ((d - 1) * 4)) & 0xF;
        emit (nib < 10 ? '0' + nib : 'A' + nib - 10);
    }
    emit (';');
    emit_dec (rawn);
    emit (';');
    emit ('{');
    for (i = 0; i < rawn; i++) {
        if (i)
            emit (',');
        emit_dec (raw[i]);
    }
    emit ('}');
    emit ('\r');
    emit ('\n');
    if (w_pos && w_sec < spc) {
        memset (buf + w_pos, 0, 512 - w_pos);
        if (!sd_write (w_lba + w_sec, buf))
            w_err = 1;
    }
    return !w_err;
}

static uint8_t file_create (void) {
    uint32_t cl, idx;
    uint8_t k, *e;
    if (!dir_scan())
        return 0;
    idx = next_idx;
    cl = alloc_cluster();
    if (!cl || !file_write (cl) || !fat_set (cl, 0x0FFFFFFFu))
        return 0;
    if (!sd_read (sl_lba, buf))
        return 0;
    e = buf + sl_off;
    memset (e, 0, 32);
    memcpy (e, "SIG00000TXT", 11);
    for (k = 7; k >= 3; k--) {
        e[k] = '0' + idx % 10;
        idx /= 10;
    }
    e[11] = 0x20;
    wr16 (e + 16, FAT_DATE);
    wr16 (e + 18, FAT_DATE);
    wr16 (e + 24, FAT_DATE);
    wr16 (e + 20, cl >> 16);
    wr16 (e + 26, cl);
    wr32 (e + 28, w_size);
    if (!sd_write (sl_lba, buf) || !sd_read (sl_lba, buf))
        return 0;
    e = buf + sl_off;
    if (memcmp (e, "SIG", 3) || rd32 (e + 28) != w_size) {
        sd_err = 2;
        return 0;
    }
    return 1;
}

enum { SV_OK,
       SV_NOCARD,
       SV_FMT,
       SV_ERR };

static uint8_t save_signal (void) {
    uint8_t r;
    sd_err = 0;
    if (!sd_init())
        return SV_NOCARD;
    r = fat_mount();
    if (r == FM_IO)
        return SV_ERR;
    if (r == FM_FMT)
        return SV_FMT;
    return file_create() ? SV_OK : SV_ERR;
}

/* ---------- buttons ---------- */
typedef struct {
    uint8_t last, stable;
    uint32_t t;
} btn_t;

static uint8_t btn_press (btn_t *b, uint8_t level) {
    if (level != b->last) {
        b->last = level;
        b->t = now();
    } else if (level != b->stable && elapsed (b->t, DEBOUNCE_MS)) {
        b->stable = level;
        if (!level)
            return 1;
    }
    return 0;
}

/* ---------- main ---------- */
enum { M_TITLE,
       M_LISTENING,
       M_CAPTURED,
       M_FEEDBACK };

int main (void) {
    btn_t bOk = {1, 1, 0}, bCn = {1, 1, 0};
    uint8_t mode = M_TITLE, st = 0, ok, cn;
    uint32_t t_anim = 0, t_fb = 0;

    SysTick->CTLR = 5; /* free-running, HCLK */
    tpms = SystemCoreClock / 1000;

    RCC->APB2PCENR |= (1u << 0) | (1u << 2) | (1u << 4) | (1u << 5); /* AFIO, GPIOA, C, D */
    RCC->APB1PCENR |= (1u << 0) | (1u << 21);                        /* TIM2, I2C1 */

    gpio_cfg (GPIOA, 1, CFG_IPU);
    GPIOA->BSHR = 1u << 1;
    gpio_cfg (GPIOC, 3, CFG_IPU);
    GPIOC->BSHR = 1u << 3;
    gpio_cfg (GPIOC, 4, CFG_IPU);
    GPIOC->BSHR = 1u << 4;
    gpio_cfg (GPIOC, 1, CFG_AFOD);
    gpio_cfg (GPIOC, 2, CFG_AFOD);
    gpio_cfg (GPIOC, 6, CFG_OUT);
    gpio_cfg (GPIOC, 7, CFG_IPU);
    GPIOC->BSHR = 1u << 7;
    gpio_cfg (GPIOD, 4, CFG_OUT);
    gpio_cfg (GPIOD, 6, CFG_OUT);
    CS_H();
    SCK_L();

    TIM2->PSC = SystemCoreClock / 1000000 - 1; /* 1 us tick */
    TIM2->ATRLR = 0xFFFF;
    TIM2->SWEVGR = 1;
    TIM2->CTLR1 = 1;

    AFIO->EXTICR &= ~(3u << 2); /* EXTI1 <- PA1 */
    EXTI->RTENR |= (1u << 1);
    EXTI->FTENR |= (1u << 1);
    EXTI->INTENR &= ~(1u << 1);
    NVIC_EnableIRQ (EXTI7_0_IRQn);

    delay_ms (200);
    oled_init();
    scr_title();

    for (;;) {
        ok = btn_press (&bOk, (GPIOC->INDR >> 3) & 1);
        cn = btn_press (&bCn, (GPIOC->INDR >> 4) & 1);

        switch (mode) {
        case M_TITLE:
            if (ok) {
                st = 0;
                t_anim = now();
                rx_start();
                scr_listening (st);
                mode = M_LISTENING;
            }
            break;

        case M_LISTENING:
            if (ok || cn) {
                rx_stop();
                scr_title();
                mode = M_TITLE;
                break;
            }
            if (elapsed (t_anim, ANIM_MS)) {
                st = (st + 1) % 3;
                t_anim = now();
                scr_listening (st);
            }
            if (rx_poll()) {
                cap_val = decode_ir();
                scr_captured();
                mode = M_CAPTURED;
            }
            break;

        case M_CAPTURED:
            if (ok) {
                scr_msg ("Saving...");
                switch (save_signal()) {
                case SV_OK: scr_msg ("Saved!"); break;
                case SV_NOCARD: scr_msg ("No SD card"); break;
                case SV_FMT: scr_msg ("Not FAT32"); break;
                default: scr_msg (sd_err == 2 ? "SD wr err" : "SD rd err"); break;
                }
                t_fb = now();
                mode = M_FEEDBACK;
            } else if (cn) {
                scr_msg ("Cancel X");
                t_fb = now();
                mode = M_FEEDBACK;
            }
            break;

        case M_FEEDBACK:
            if (elapsed (t_fb, FEEDBACK_MS)) {
                scr_title();
                mode = M_TITLE;
            }
            break;
        }
    }
}