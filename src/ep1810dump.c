/* SPDX-License-Identifier: GPL-3.0-or-later */
/*
 * ep1810dump -- Altera Classic EPLD (EP1810 / EP1800I, EP910 / EP900I,
 * EP610 / EP600I) fuse map decompiler. Copyright (C) DARKSOFT
 *
 * Reads a JEDEC file or a POF. Writes the equations of every logic cell, the pin
 * use, a design and assignment file for MAX+PLUS II with every pin and logic cell
 * in place, and a behavioural Verilog description.
 *
 * The parts (tables in tables.h):
 *   - A logic cell has ten product terms. A term is a row of fuses, two per
 *     signal: complement first, then true. Fuse 0 = connected. All fuses 0: the
 *     unused term, never true. All fuses 1: constant 1.
 *   - Terms 0-7 are OR-ed. Term 8 clears the register. Term 9 is the output
 *     enable, or the register clock when the cell takes its clock from the array.
 *   - Five architecture fuses per cell:
 *       0  0 = OR inverted
 *       1  term 9 is the clock (1) or the output enable (0)
 *       2  T register (1) or D register (0)
 *       3  the cell's own pair of array lines carries the cell (1) or its pin (0)
 *       4  registered (1) or combinational (0) output
 *   - Registers start at 0. The global clock of a cell is the clock pin of its
 *     half (EP610, EP910) or quadrant (EP1810), rising edge.
 *
 * Exit codes: 0 done, 3 refused by name (nothing is written), 2 usage, 1 error.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <setjmp.h>
#ifdef _WIN32
#include <io.h>
#include <fcntl.h>
#endif
#include "tables.h"
#include "selftest_data.h"

#define VERSION "0.93"

#if defined(__MINGW64__)
#define SETJMP(b) _setjmp((b), NULL)
#else
#define SETJMP(b) setjmp(b)
#endif

/* ---------------------------------------------------------------- refusals */

static jmp_buf refuse_jmp;
static char refuse_name[64];
static char refuse_detail[256];

static void refuse(const char *name, const char *fmt, ...)
{
    va_list ap;
    strncpy(refuse_name, name, sizeof refuse_name - 1);
    refuse_name[sizeof refuse_name - 1] = 0;
    refuse_detail[0] = 0;
    if (fmt) {
        va_start(ap, fmt);
        vsnprintf(refuse_detail, sizeof refuse_detail, fmt, ap);
        va_end(ap);
    }
    longjmp(refuse_jmp, 1);
}

static void die(const char *what)
{
    fprintf(stderr, "ep1810dump: %s\n", what);
    exit(1);
}

/* ---------------------------------------------------------------- text buffer */

typedef struct { char *s; size_t n, cap; } Text;

static void text_need(Text *t, size_t extra)
{
    if (t->n + extra + 1 > t->cap) {
        size_t c = t->cap ? t->cap * 2 : 4096;
        while (c < t->n + extra + 1) c *= 2;
        t->s = (char *)realloc(t->s, c);
        if (!t->s) die("out of memory");
        t->cap = c;
    }
}

static void text_put(Text *t, const char *s)
{
    size_t l = strlen(s);
    text_need(t, l);
    memcpy(t->s + t->n, s, l);
    t->n += l;
    t->s[t->n] = 0;
}

static void text_printf(Text *t, const char *fmt, ...)
{
    char buf[1024];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof buf, fmt, ap);
    va_end(ap);
    text_put(t, buf);
}

static void text_free(Text *t) { free(t->s); t->s = NULL; t->n = t->cap = 0; }

/* ---------------------------------------------------------------- input files */

#define MAXFUSE 42490

typedef struct {
    int is_pof;
    char device[40];        /* empty when the file names none */
    int security;
    int nfuse;
    unsigned char fuse[MAXFUSE];
} FuseFile;

static int is_ws(int c) { return c == ' ' || c == '\t' || c == '\r' || c == '\n'; }
static int is_sp(int c) { return is_ws(c) || c == 11 || c == 12; }
static int is_hex(int c) { return (c >= '0' && c <= '9') || (c >= 'A' && c <= 'F') || (c >= 'a' && c <= 'f'); }
static int hexval(int c) { return c <= '9' ? c - '0' : (c | 32) - 'a' + 10; }

static const Part *find_part(const char *name)
{
    int i;
    for (i = 0; i < NPART; i++)
        if (strcmp(PART[i].name, name) == 0) return &PART[i];
    return NULL;
}

/* strip blanks at both ends of [*a, *b) */
static void strip(const unsigned char **a, const unsigned char **b)
{
    while (*a < *b && is_ws(**a)) (*a)++;
    while (*b > *a && is_ws((*b)[-1])) (*b)--;
}

/*
 * JEDEC: STX, header text, fields ending with '*', ETX, four hex digits (sum of
 * all bytes from STX to ETX; 0000 means "not given").
 */
static void read_jed(const unsigned char *d, size_t len, FuseFile *out)
{
    size_t i, nstx = 0, netx = 0, stx = 0, etx = 0;
    const unsigned char *p, *end, *f, *fe;
    static unsigned char given[MAXFUSE];
    long qf = -1;
    int deflt = -1, csum = -1, first = 1;
    unsigned sum = 0;

    for (i = 0; i < len; i++) {
        if (d[i] == 2) { if (!nstx) stx = i; nstx++; }
        if (d[i] == 3) { if (!netx) etx = i; netx++; }
    }
    if (nstx == 0) refuse("JED_NO_STX", NULL);
    if (nstx != 1) refuse("JED_STX_COUNT", NULL);
    if (netx == 0) refuse("JED_NO_ETX", NULL);
    if (netx != 1) refuse("JED_ETX_COUNT", NULL);
    if (etx < stx) refuse("JED_NO_ETX", NULL);
    if (len - (etx + 1) < 4) refuse("JED_BAD_TRAILER", NULL);
    for (i = 0; i < 4; i++)
        if (!is_hex(d[etx + 1 + i])) refuse("JED_BAD_TRAILER", NULL);
    for (i = etx + 5; i < len; i++)
        if (!is_ws(d[i]) && d[i] != 0) refuse("JED_BAD_TRAILER", NULL);
    {
        unsigned want = 0;
        for (i = 0; i < 4; i++) want = want * 16 + (unsigned)hexval(d[etx + 1 + i]);
        for (i = stx; i <= etx; i++) sum += d[i];
        sum &= 0xFFFF;
        if (want != 0 && want != sum) refuse("JED_CHECKSUM", "file %04X computed %04X", want, sum);
    }
    p = d + stx + 1;
    end = d + etx;
    /* the text after the last '*' must be blank */
    {
        const unsigned char *q = end;
        while (q > p && q[-1] != '*') q--;
        for (f = q; f < end; f++)
            if (!is_ws(*f)) refuse("JED_UNTERMINATED_FIELD", NULL);
        end = q;                /* end now points just after the last '*', or at p when there is none */
    }
    out->is_pof = 0;
    out->device[0] = 0;
    out->security = 0;
    out->nfuse = 0;
    while (p < end) {
        f = p;
        while (p < end && *p != '*') p++;
        fe = p;
        p++;                    /* skip '*' */
        if (first) { first = 0; continue; }     /* header text */
        while (f < fe && is_ws(*f)) f++;
        if (f == fe) continue;
        switch (*f) {
        case 'Q':
            if (f + 1 < fe && f[1] == 'F') {
                const unsigned char *a = f + 2, *b = fe;
                long v = 0;
                strip(&a, &b);
                if (a == b || b - a > 7) refuse("JED_QF_VALUE", NULL);
                for (; a < b; a++) {
                    if (*a < '0' || *a > '9') refuse("JED_QF_VALUE", NULL);
                    v = v * 10 + (*a - '0');
                }
                if (qf >= 0) refuse("JED_QF_REPEATED", NULL);
                if (v < 1 || v > MAXFUSE) refuse("JED_QF_VALUE", NULL);
                qf = v;
                memset(out->fuse, 0, (size_t)qf);
                memset(given, 0, (size_t)qf);
            } else if (f + 1 < fe && (f[1] == 'P' || f[1] == 'V')) {
                /* pin count, vector count: not used */
            } else {
                refuse("JED_FIELD_UNKNOWN", "Q");
            }
            break;
        case 'N':
            if (f + 2 < fe && f[1] == 'D' && (f[2] == ' ' || f[2] == '\t')) {
                /* "ND <device>": exactly one word */
                const unsigned char *a = f + 3, *b = fe, *q;
                size_t l;
                while (a < b && is_sp(*a)) a++;
                while (b > a && is_sp(b[-1])) b--;
                if (a == b) refuse("JED_ND_VALUE", NULL);
                l = (size_t)(b - a);
                if (l > 32) refuse("JED_ND_VALUE", NULL);
                for (q = a; q < b; q++)
                    if (*q < 33 || *q > 126) refuse("JED_ND_VALUE", NULL);
                memcpy(out->device, a, l);
                out->device[l] = 0;
            }
            break;
        case 'G': {
            const unsigned char *a = f + 1, *b = fe;
            strip(&a, &b);
            if (b - a != 1 || (*a != '0' && *a != '1')) refuse("JED_G_VALUE", NULL);
            out->security = *a - '0';
            break;
        }
        case 'F': {
            const unsigned char *a = f + 1, *b = fe;
            strip(&a, &b);
            if (b - a != 1 || (*a != '0' && *a != '1')) refuse("JED_F_VALUE", NULL);
            deflt = *a - '0';
            break;
        }
        case 'L': {
            const unsigned char *a = f + 1;
            long addr = 0;
            int nd = 0;
            if (qf < 0) refuse("JED_QF_MISSING", NULL);
            while (a < fe && *a >= '0' && *a <= '9') { addr = addr * 10 + (*a - '0'); a++; nd++; if (nd > 7) break; }
            if (nd == 0 || nd > 7) refuse("JED_L_ADDRESS", NULL);
            for (; a < fe; a++) {
                if (is_ws(*a)) continue;
                if (*a != '0' && *a != '1') refuse("JED_FUSE_CHAR", NULL);
                if (addr >= qf) refuse("JED_FUSE_RANGE", NULL);
                out->fuse[addr] = (unsigned char)(*a - '0');
                given[addr] = 1;
                addr++;
            }
            break;
        }
        case 'C': {
            const unsigned char *a = f + 1, *b = fe;
            strip(&a, &b);
            if (b - a != 4 || !is_hex(a[0]) || !is_hex(a[1]) || !is_hex(a[2]) || !is_hex(a[3])) refuse("JED_C_VALUE", NULL);
            csum = (hexval(a[0]) << 12) | (hexval(a[1]) << 8) | (hexval(a[2]) << 4) | hexval(a[3]);
            break;
        }
        default:
            if (*f >= 33 && *f <= 126) refuse("JED_FIELD_UNKNOWN", "%c", *f);
            refuse("JED_FIELD_UNKNOWN", "0x%02x", *f);
        }
    }
    if (qf < 0) refuse("JED_QF_MISSING", NULL);
    {
        long missing = 0, k;
        for (k = 0; k < qf; k++) if (!given[k]) missing++;
        if (missing) {
            if (deflt < 0) refuse("JED_FUSES_UNDEFINED", "%ld of %ld", missing, qf);
            for (k = 0; k < qf; k++) if (!given[k]) out->fuse[k] = (unsigned char)deflt;
        }
        if (csum >= 0) {
            unsigned s = 0;
            for (k = 0; k < qf; k++) if (out->fuse[k]) s += 1u << (k & 7);
            s &= 0xFFFF;
            if ((int)s != csum) refuse("JED_FUSE_CHECKSUM", "file %04X computed %04X", (unsigned)csum, s);
        }
    }
    out->nfuse = (int)qf;
}

static unsigned crc16(const unsigned char *d, size_t n)
{
    unsigned c = 0xFFFF;
    size_t i;
    int k;
    for (i = 0; i < n; i++) {
        c ^= d[i];
        for (k = 0; k < 8; k++) c = (c & 1) ? (c >> 1) ^ 0x8408 : c >> 1;
    }
    return c ^ 0xFFFF;
}

static unsigned le16(const unsigned char *p) { return p[0] | (p[1] << 8); }
static unsigned long le32(const unsigned char *p) { return p[0] | (p[1] << 8) | ((unsigned long)p[2] << 16) | ((unsigned long)p[3] << 24); }

/*
 * POF: "POF\0", version, packet count, then packets (tag, length, body):
 * 1 tool name, 2 device name, 3 title, 5 security, 7 one matrix of array fuses per
 * group of eight logic cells, 6 the architecture fuses, 8 CRC-16 of all before it.
 */
static void read_pof(const unsigned char *d, size_t len, FuseFile *out)
{
    unsigned long n, k;
    size_t o = 12, npk = 0, i7 = 0;
    static int *tag;
    static const unsigned char **body;
    static size_t *blen, *boff;
    int i, cnt[9], fi;
    const unsigned char *b1 = NULL, *b2 = NULL, *b3 = NULL, *b5 = NULL, *b6 = NULL, *b8 = NULL;
    size_t l1 = 0, l2 = 0, l3 = 0, l5 = 0, l6 = 0, l8 = 0, crc_off = 0, j;
    const Part *part;
    const Family *F;
    static unsigned char image[6 * 10 * 88];
    size_t nimage = 0;

    if (len < 4 || memcmp(d, "POF\0", 4) != 0) refuse("POF_MAGIC", NULL);
    if (len < 12) refuse("POF_TRUNCATED", NULL);
    if (le32(d + 4) != 0x00010000UL) refuse("POF_VERSION", NULL);
    n = le32(d + 8);
    /* a packet takes at least six bytes, so the file length bounds the packet list */
    tag = (int *)realloc(tag, (len / 6 + 1) * sizeof *tag);
    body = (const unsigned char **)realloc((void *)body, (len / 6 + 1) * sizeof *body);
    blen = (size_t *)realloc(blen, (len / 6 + 1) * sizeof *blen);
    boff = (size_t *)realloc(boff, (len / 6 + 1) * sizeof *boff);
    if (!tag || !body || !blen || !boff) die("out of memory");
    for (k = 0; k < n; k++) {
        unsigned long ln;
        if (o + 6 > len) refuse("POF_TRUNCATED", NULL);
        ln = le32(d + o + 2);
        if (ln > len - o - 6) refuse("POF_TRUNCATED", NULL);
        tag[npk] = (int)le16(d + o);
        body[npk] = d + o + 6;
        blen[npk] = (size_t)ln;
        boff[npk] = o;
        npk++;
        o += 6 + (size_t)ln;
    }
    if (o != len) refuse("POF_TRAILING_BYTES", NULL);
    memset(cnt, 0, sizeof cnt);
    for (j = 0; j < npk; j++) {
        int t = tag[j];
        if (!(t == 1 || t == 2 || t == 3 || t == 5 || t == 6 || t == 7 || t == 8)) refuse("POF_TAG_UNKNOWN", "%d", t);
    }
    for (j = 0; j < npk; j++) cnt[tag[j]]++;
    for (i = 0; i < 5; i++) {
        static const int must[5] = {1, 2, 5, 6, 8};
        if (cnt[must[i]] != 1) refuse("POF_TAG_COUNT", "tag %d", must[i]);
    }
    if (cnt[3] > 1) refuse("POF_TAG_COUNT", "tag 3");
    for (j = 0; j < npk; j++) {
        switch (tag[j]) {
        case 1: b1 = body[j]; l1 = blen[j]; break;
        case 2: b2 = body[j]; l2 = blen[j]; break;
        case 3: b3 = body[j]; l3 = blen[j]; break;
        case 5: b5 = body[j]; l5 = blen[j]; break;
        case 6: b6 = body[j]; l6 = blen[j]; break;
        case 8: b8 = body[j]; l8 = blen[j]; crc_off = boff[j]; break;
        default: break;
        }
    }
    /* texts: one NUL, at the end */
    {
        const unsigned char *tb[3];
        size_t tl[3];
        int tt[3], nt = 0, z;
        tb[nt] = b1; tl[nt] = l1; tt[nt++] = 1;
        tb[nt] = b2; tl[nt] = l2; tt[nt++] = 2;
        if (b3) { tb[nt] = b3; tl[nt] = l3; tt[nt++] = 3; }
        for (i = 0; i < nt; i++) {
            z = 0;
            for (j = 0; j < tl[i]; j++) if (tb[i][j] == 0) z++;
            if (tl[i] == 0 || tb[i][tl[i] - 1] != 0 || z != 1) refuse("POF_TEXT", "tag %d", tt[i]);
        }
        if (l2 - 1 > 32) refuse("POF_TEXT", "tag 2");
        for (j = 0; j + 1 < l2; j++) if (b2[j] < 33 || b2[j] > 126) refuse("POF_TEXT", "tag 2");
    }
    part = find_part((const char *)b2);
    if (!part) refuse("POF_DEVICE_UNKNOWN", "%s", (const char *)b2);
    F = &FAMILY[part->family];
    /* packet order: 1 2 [3] 5 7.. 6 8 */
    {
        int want[16], nw = 0, ok = 1;
        want[nw++] = 1; want[nw++] = 2;
        if (b3) want[nw++] = 3;
        want[nw++] = 5;
        for (i = 0; i < F->ntag7; i++) want[nw++] = 7;
        want[nw++] = 6; want[nw++] = 8;
        if ((size_t)nw != npk) ok = 0;
        for (i = 0; ok && i < nw; i++) if (want[i] != tag[i]) ok = 0;
        if (!ok) refuse("POF_TAG_ORDER", NULL);
    }
    if (l5 != 2 || !((b5[0] == 0 || b5[0] == 1) && b5[1] == 0)) refuse("POF_SECURITY_VALUE", NULL);
    if (l8 != 2) refuse("POF_CRC_LENGTH", NULL);
    {
        unsigned crc = le16(b8), calc = crc16(d, crc_off + 6);
        if (crc != calc) refuse("POF_CRC", "file %04X computed %04X", crc, calc);
    }
    fi = 0;
    for (j = 0; j < npk; j++) {
        const int *e;
        if (tag[j] != 7) continue;
        e = F->tag7 + 5 * fi++;
        if (blen[j] < 12) refuse("POF_MATRIX_HEADER", NULL);
        if ((int)le16(body[j]) != e[0] || (int)le16(body[j] + 2) != e[1] || (int)le16(body[j] + 4) != e[2] ||
            (int)le16(body[j] + 6) != e[3] || (int)le16(body[j] + 8) != e[4] || le16(body[j] + 10) != 0)
            refuse("POF_MATRIX_HEADER", NULL);
        if (blen[j] - 12 != (size_t)(e[2] * e[3])) refuse("POF_MATRIX_SIZE", NULL);
        memcpy(image + nimage, body[j] + 12, blen[j] - 12);
        nimage += blen[j] - 12;
    }
    (void)i7;
    if (l6 < 8) refuse("POF_ARCH_HEADER", NULL);
    if (le32(b6) != (unsigned long)F->tag6_first || (int)le16(b6 + 4) != F->tag6_count || le16(b6 + 6) != 1) refuse("POF_ARCH_HEADER", NULL);
    if (l6 - 8 != (size_t)((F->tag6_count + 7) / 8)) refuse("POF_ARCH_SIZE", NULL);
    {
        int mc, pt, line;
        for (mc = 0; mc < F->N; mc++)
            for (pt = 0; pt < 10; pt++)
                for (line = 0; line < F->W; line++) {
                    long a = F->bitbase[mc] + 8L * F->W * pt + 8L * line;
                    out->fuse[(mc * 10 + pt) * F->W + line] = (unsigned char)((image[a >> 3] >> (a & 7)) & 1);
                }
        for (i = 0; i < F->tag6_count; i++)
            out->fuse[F->tag6_first + i] = (unsigned char)((b6[8 + (i >> 3)] >> (i & 7)) & 1);
    }
    out->is_pof = 1;
    strcpy(out->device, (const char *)b2);
    out->security = b5[0];
    out->nfuse = F->QF;
}

static void read_any(const unsigned char *d, size_t len, FuseFile *out)
{
    if (len >= 4 && memcmp(d, "POF\0", 4) == 0) read_pof(d, len, out);
    else read_jed(d, len, out);
}

/* ---------------------------------------------------------------- netlist */

#define MAXLIT 44
#define MAXCELL 48
#define MAXVP 69

typedef struct { unsigned char kind, n, pol; } Lit;     /* kind 'C' logic cell feedback, 'P' pin record */
typedef struct { int n; Lit l[MAXLIT]; } Term;

enum { UNUSED, INPUT, LOGIC };
enum { REG_NONE, REG_D, REG_T };
enum { CLK_NONE, CLK_GLOBAL, CLK_TERM, CLK_NEVER };
enum { OE_ON, OE_OFF, OE_TERM };
enum { R_NONE, R_IN, R_OUT, R_BIDIR, R_BURIED, R_INBURIED };

typedef struct {
    int cls;
    int nterms;
    Term terms[8];
    int inv, reg;
    int clk, clk_vp;
    Term clkt;
    int has_clr;
    Term clrt;
    int oe;
    Term oet;
    int fbpin;
} Cell;

typedef struct {
    const Family *F;
    int mode;                       /* package mode */
    const char *device;
    int security, turbo, is_pof;
    Cell c[MAXCELL + 1];
    /* view */
    int nlogic, logic[MAXCELL];
    unsigned char read_pin[MAXVP], read_cell[MAXCELL + 1], gclk[MAXVP];
    int cellpin[MAXVP];             /* logic cell of a pin record, or 0 */
    int role[MAXVP];
} Net;

static int lit_cmp(const void *a, const void *b)
{
    const Lit *x = (const Lit *)a, *y = (const Lit *)b;
    if (x->kind != y->kind) return x->kind < y->kind ? -1 : 1;
    if (x->n != y->n) return x->n < y->n ? -1 : 1;
    return (int)x->pol - (int)y->pol;
}

/* class of one product term: '1' no line connected, '0' every line connected,
 * 'P' a product, 'X' a product holding a signal together with its complement */
static int term_class(const unsigned char *bits, int W)
{
    int i, ones = 0, both = 0;
    for (i = 0; i < W; i++) ones += bits[i];
    if (ones == W) return '1';
    if (ones == 0) return '0';
    for (i = 0; i < W; i += 2) if (!bits[i] && !bits[i + 1]) both = 1;
    return both ? 'X' : 'P';
}

static int pattern_known(const char *sig)
{
    int i;
    for (i = 0; i < NPATTERN; i++) if (strcmp(PATTERN[i], sig) == 0) return 1;
    return 0;
}

static void term_lits(const Family *F, int mc, const unsigned char *bits, const unsigned char *k3of, Term *t)
{
    int i, q = F->cell_quad[mc], half = F->W / 2;
    t->n = 0;
    for (i = 0; i < half; i++) {
        int c = bits[2 * i], tr = bits[2 * i + 1], kind, n;
        Lit *l;
        if (c && tr) continue;
        kind = F->pair_kind[q * half + i];
        n = F->pair_n[q * half + i];
        l = &t->l[t->n++];
        l->pol = (unsigned char)(tr == 0 ? 1 : 0);
        if (kind == 1) { l->kind = 'P'; l->n = (unsigned char)n; }
        else if (kind == 4) { l->kind = 'P'; l->n = F->cell_vp[n]; }
        else if (kind == 3) { l->kind = 'C'; l->n = (unsigned char)n; }
        else if (k3of[n] == 1) { l->kind = 'C'; l->n = (unsigned char)n; }
        else { l->kind = 'P'; l->n = F->cell_vp[n]; }
    }
    qsort(t->l, (size_t)t->n, sizeof(Lit), lit_cmp);
}

static void check_sources(const Net *N, int m, const Term *t)
{
    int i;
    for (i = 0; i < t->n; i++) {
        int src = t->l[i].kind == 'C' ? t->l[i].n : N->cellpin[t->l[i].n];
        if (t->l[i].kind == 'C' && N->c[src].cls != LOGIC) refuse("SIGNAL_FROM_UNUSED_CELL", "LC%d reads LC%d", m, src);
        if (t->l[i].kind == 'P' && src && N->c[src].cls == UNUSED) refuse("SIGNAL_FROM_UNUSED_CELL", "LC%d reads the pin of LC%d", m, src);
    }
}

static void decode(const FuseFile *ff, const Family *F, Net *N)
{
    const unsigned char *fz = ff->fuse;
    unsigned char k3of[MAXCELL + 1];
    unsigned char used_in[MAXVP], used_gp[4];
    int mc, t, i, half = F->W / 2;

    memset(N->c, 0, sizeof N->c);
    N->F = F;
    for (mc = 1; mc <= F->N; mc++) k3of[mc] = fz[F->A + 5 * (mc - 1) + 3];
    memset(used_in, 0, sizeof used_in);
    memset(used_gp, 0, sizeof used_gp);
    for (i = 0; i < MAXVP; i++) N->cellpin[i] = 0;
    for (mc = 1; mc <= F->N; mc++) N->cellpin[F->cell_vp[mc]] = mc;
    for (mc = 1; mc <= F->N; mc++) {
        Cell *c = &N->c[mc];
        const unsigned char *row = fz + (long)(mc - 1) * 10 * F->W;
        const unsigned char *k = fz + F->A + 5 * (mc - 1);
        const unsigned char *kd = F->kdef + 5 * mc;
        int cl[10], glob = F->cell_global[mc], all1 = 1, any1 = 0, anyother = 0, isdef, isin;
        char sig[64], logic;
        for (t = 0; t < 10; t++) {
            cl[t] = term_class(row + t * F->W, F->W);
            if (cl[t] != '1') all1 = 0;
        }
        for (t = 0; t < 8; t++) { if (cl[t] == '1') any1 = 1; else anyother = 1; }
        logic = (char)(!anyother ? 'U' : (any1 ? 'V' : 'L'));
        for (t = 0; t < 10; t++)
            if (cl[t] == 'X') refuse("TERM_SIGNAL_AND_COMPLEMENT", "LC%d term %d", mc, t);
        sprintf(sig, "%s %c %d%d%d%d%d %c %c %c", F->name, glob ? 'G' : 'L', k[0], k[1], k[2], k[3], k[4], logic, cl[8], cl[9]);
        if (!pattern_known(sig)) {
            if (glob && k[3] == 1) refuse("GLOBAL_CELL_BRANCH_UNSEEN", "LC%d", mc);
            if (k[0] == 0 && k[4] == 1) refuse("INVERSION_WITH_REGISTER_UNSEEN", "LC%d", mc);
            refuse("CELL_PATTERN_UNSEEN", "LC%d %s", mc, sig + strlen(F->name) + 1);
        }
        isdef = memcmp(k, kd, 5) == 0;
        if (all1 && isdef) { c->cls = UNUSED; continue; }
        isin = 1;
        for (t = 0; t < 9; t++) if (cl[t] != '1') isin = 0;
        if (cl[9] != '0') isin = 0;
        if (glob) { if (!(k[0] && k[1] && k[2] && k[3] && k[4])) isin = 0; }
        else { if (!(k[0] == 1 && k[1] == 0 && k[2] == 1 && k[3] == 0 && k[4] == 1)) isin = 0; }
        if (isin) { c->cls = INPUT; continue; }
        c->cls = LOGIC;
        c->nterms = 0;
        for (t = 0; t < 8; t++) {
            if (cl[t] == '0') continue;
            if (cl[t] == '1') c->terms[c->nterms].n = 0;
            else term_lits(F, mc, row + t * F->W, k3of, &c->terms[c->nterms]);
            c->nterms++;
        }
        c->inv = (k[0] == 0);
        c->reg = k[4] == 0 ? REG_NONE : (k[2] == 1 ? REG_T : REG_D);
        c->has_clr = 0;
        if (c->reg != REG_NONE && cl[8] != '0') {
            c->has_clr = 1;
            if (cl[8] == '1') c->clrt.n = 0;
            else term_lits(F, mc, row + 8 * F->W, k3of, &c->clrt);
        }
        c->clk = CLK_NONE;
        if (k[1] == 1) {
            if (c->reg != REG_NONE) {
                if (cl[9] == '0') c->clk = CLK_NEVER;
                else {
                    c->clk = CLK_TERM;
                    if (cl[9] == '1') c->clkt.n = 0;
                    else term_lits(F, mc, row + 9 * F->W, k3of, &c->clkt);
                }
            }
            c->oe = OE_ON;
        } else {
            if (c->reg != REG_NONE) { c->clk = CLK_GLOBAL; c->clk_vp = F->cell_clk[mc]; }
            if (cl[9] == '0') c->oe = OE_OFF;
            else if (cl[9] == '1') c->oe = OE_ON;
            else { c->oe = OE_TERM; term_lits(F, mc, row + 9 * F->W, k3of, &c->oet); }
        }
        c->fbpin = !(glob || k[3] == 1);
        /* lines in use, for the device-level fuses */
        for (t = 0; t < 10; t++) {
            if (cl[t] != 'P') continue;
            for (i = 0; i < half; i++) {
                const unsigned char *b = row + t * F->W + 2 * i;
                int kind = F->pair_kind[F->cell_quad[mc] * half + i], n = F->pair_n[F->cell_quad[mc] * half + i];
                if (b[0] && b[1]) continue;
                if (kind == 1) used_in[n] = 1;
                if (kind == 4) used_gp[F->cell_quad[n]] = 1;
            }
        }
    }
    /* every signal read must come from a pin or from a logic cell in use */
    for (mc = 1; mc <= F->N; mc++) {
        const Cell *c = &N->c[mc];
        if (c->cls != LOGIC) continue;
        for (t = 0; t < c->nterms; t++) check_sources(N, mc, &c->terms[t]);
        if (c->has_clr) check_sources(N, mc, &c->clrt);
        if (c->clk == CLK_TERM) check_sources(N, mc, &c->clkt);
        if (c->oe == OE_TERM) check_sources(N, mc, &c->oet);
    }
    if (fz[F->turbo0] != fz[F->turbo1]) refuse("TURBO_COPIES_DIFFER", "%d%d", fz[F->turbo0], fz[F->turbo1]);
    N->turbo = fz[F->turbo0];
    for (i = 0; i < F->nck; i++) {
        int vp = F->ck_vp[i], v = fz[F->ck_fuse[i]];
        if (v != used_in[vp])
            refuse("CLOCK_PIN_DATA_FUSE_MISMATCH", "fuse %d = %d, pin record %d %s in the array", F->ck_fuse[i], v, vp, used_in[vp] ? "used" : "not used");
    }
    for (i = 0; i < F->nbus; i++) {
        int q = F->bus_q[i], v = fz[F->bus_fuse[i]];
        if (v != used_gp[q])
            refuse("GLOBAL_BUS_FUSE_MISMATCH", "fuse %d = %d, bus pairs of quadrant %c %s", F->bus_fuse[i], v, "ABCD"[q], used_gp[q] ? "used" : "not used");
    }
}

/* ---------------------------------------------------------------- view: names and pin roles */

static const char *pin_name(const Net *N, int vp)
{
    return (N->mode == N->F->mode_a ? N->F->pin_a : N->F->pin_b)[vp];
}

static void see(Net *N, const Term *t)
{
    int i;
    for (i = 0; i < t->n; i++) {
        if (t->l[i].kind == 'P') N->read_pin[t->l[i].n] = 1;
        else N->read_cell[t->l[i].n] = 1;
    }
}

static void make_view(Net *N)
{
    const Family *F = N->F;
    int m, t, vp;
    N->nlogic = 0;
    memset(N->read_pin, 0, sizeof N->read_pin);
    memset(N->read_cell, 0, sizeof N->read_cell);
    memset(N->gclk, 0, sizeof N->gclk);
    for (m = 1; m <= F->N; m++) {
        Cell *c = &N->c[m];
        if (c->cls != LOGIC) continue;
        N->logic[N->nlogic++] = m;
        for (t = 0; t < c->nterms; t++) see(N, &c->terms[t]);
        if (c->oe == OE_TERM) see(N, &c->oet);
        if (c->reg != REG_NONE) {
            if (c->has_clr) see(N, &c->clrt);
            if (c->clk == CLK_TERM) see(N, &c->clkt);
            else if (c->clk == CLK_GLOBAL) N->gclk[c->clk_vp] = 1;
        }
    }
    for (vp = 0; vp < MAXVP; vp++) {
        N->role[vp] = R_NONE;
        if ((N->read_pin[vp] || N->gclk[vp]) && (!N->cellpin[vp] || N->c[N->cellpin[vp]].cls != LOGIC)) N->role[vp] = R_IN;
    }
    for (m = 1; m <= F->N; m++) {
        Cell *c = &N->c[m];
        vp = F->cell_vp[m];
        if (c->cls == INPUT) N->role[vp] = R_IN;
        else if (c->cls == LOGIC) {
            if (c->oe == OE_ON) N->role[vp] = R_OUT;
            else if (c->oe == OE_OFF) N->role[vp] = N->read_pin[vp] ? R_INBURIED : R_BURIED;
            else N->role[vp] = N->read_pin[vp] ? R_BIDIR : R_OUT;
        }
    }
}

/* package pin order: by number, or for grid names (PGA) by letter then number */
static const Net *order_net;
static int order_cmp(const void *a, const void *b)
{
    const char *x = pin_name(order_net, *(const int *)a), *y = pin_name(order_net, *(const int *)b);
    int dx = (x[0] >= '0' && x[0] <= '9'), dy = (y[0] >= '0' && y[0] <= '9');
    if (dx != dy) return dx ? -1 : 1;
    if (dx) return atoi(x) - atoi(y);
    if (x[0] != y[0]) return x[0] < y[0] ? -1 : 1;
    return atoi(x + 1) - atoi(y + 1);
}

/* pin records whose role passes the mask (bit r set = role r wanted), in package order */
static int ordered(const Net *N, unsigned mask, int *out)
{
    int vp, n = 0;
    for (vp = 1; vp <= N->F->nvp; vp++)
        if (N->role[vp] != R_NONE && (mask & (1u << N->role[vp]))) out[n++] = vp;
    order_net = N;
    qsort(out, (size_t)n, sizeof(int), order_cmp);
    return n;
}

/* ---------------------------------------------------------------- RACING registers */

typedef struct { unsigned long long p[2]; unsigned long long c; } SigSet;

static void set_or(SigSet *a, const SigSet *b) { a->p[0] |= b->p[0]; a->p[1] |= b->p[1]; a->c |= b->c; }
static int set_eq(const SigSet *a, const SigSet *b) { return a->p[0] == b->p[0] && a->p[1] == b->p[1] && a->c == b->c; }

/*
 * What a signal depends on, followed through combinational cells and through pins.
 * cellsup[m]: what the value of combinational cell m depends on; pinsup[m]: what the
 * level on the pin of logic cell m depends on. Both grow together to a fixed point,
 * so loops (combinational feedback, cells whose output enables read each other's
 * pins) are harmless.
 */
static SigSet cellsup[MAXCELL + 1], pinsup[MAXCELL + 1];

static void lit_support(const Net *N, const Term *t, SigSet *out)
{
    int i;
    for (i = 0; i < t->n; i++) {
        int kind = t->l[i].kind, n = t->l[i].n, m;
        if (kind == 'C') {
            const Cell *c = &N->c[n];
            if (c->cls != LOGIC) continue;
            if (c->reg != REG_NONE) out->c |= 1ULL << n;
            else set_or(out, &cellsup[n]);
            continue;
        }
        m = N->cellpin[n];
        if (!m || N->c[m].cls != LOGIC) out->p[n >> 6] |= 1ULL << (n & 63);
        else set_or(out, &pinsup[m]);
    }
}

static void put_name(Text *o, const Net *N, int kind, int n, int pol, const char *neg)
{
    if (!pol) text_put(o, neg);
    if (kind == 'P') text_printf(o, "p%s", pin_name(N, n));
    else text_printf(o, "lc%d", n);
}

static void racing(const Net *N, Text *o)
{
    int changed = 1, i, t, m, n, first;
    memset(cellsup, 0, sizeof cellsup);
    memset(pinsup, 0, sizeof pinsup);
    while (changed) {
        changed = 0;
        for (i = 0; i < N->nlogic; i++) {
            const Cell *c;
            SigSet s, ps;
            int vp;
            m = N->logic[i];
            c = &N->c[m];
            vp = N->F->cell_vp[m];
            if (c->reg == REG_NONE) {
                memset(&s, 0, sizeof s);
                for (t = 0; t < c->nterms; t++) lit_support(N, &c->terms[t], &s);
                if (!set_eq(&s, &cellsup[m])) { cellsup[m] = s; changed = 1; }
            }
            memset(&ps, 0, sizeof ps);
            if (c->oe != OE_ON) ps.p[vp >> 6] |= 1ULL << (vp & 63);
            if (c->oe != OE_OFF) {
                if (c->reg != REG_NONE) ps.c |= 1ULL << m;
                else set_or(&ps, &cellsup[m]);
                if (c->oe == OE_TERM) lit_support(N, &c->oet, &ps);
            }
            if (!set_eq(&ps, &pinsup[m])) { pinsup[m] = ps; changed = 1; }
        }
    }
    for (i = 0; i < N->nlogic; i++) {
        const Cell *c;
        SigSet ck, data;
        m = N->logic[i];
        c = &N->c[m];
        if (c->reg == REG_NONE) continue;
        memset(&ck, 0, sizeof ck);
        memset(&data, 0, sizeof data);
        if (c->clk == CLK_GLOBAL) ck.p[c->clk_vp >> 6] |= 1ULL << (c->clk_vp & 63);
        else if (c->clk == CLK_TERM) lit_support(N, &c->clkt, &ck);
        for (t = 0; t < c->nterms; t++) lit_support(N, &c->terms[t], &data);
        if (c->has_clr) lit_support(N, &c->clrt, &data);
        if (c->reg == REG_T) data.c |= 1ULL << m;      /* the next state of a T register depends on its own output */
        ck.p[0] &= data.p[0]; ck.p[1] &= data.p[1]; ck.c &= data.c;
        if (!ck.p[0] && !ck.p[1] && !ck.c) continue;
        text_printf(o, "RACING lc%d: clock and data or clear share ", m);
        first = 1;
        for (n = 1; n <= MAXCELL; n++)
            if (ck.c & (1ULL << n)) { if (!first) text_put(o, ", "); first = 0; put_name(o, N, 'C', n, 1, ""); }
        for (n = 0; n < MAXVP; n++)
            if (ck.p[n >> 6] & (1ULL << (n & 63))) { if (!first) text_put(o, ", "); first = 0; put_name(o, N, 'P', n, 1, ""); }
        text_put(o, "\n");
    }
}

/* ---------------------------------------------------------------- report */

static const char *pkg_name(const Net *N) { return N->mode == N->F->mode_a ? N->F->pkg_a : N->F->pkg_b; }

static void rep_prod(Text *o, const Net *N, const Term *t)
{
    int i;
    if (t->n == 0) { text_put(o, "VCC"); return; }
    for (i = 0; i < t->n; i++) {
        if (i) text_put(o, " & ");
        put_name(o, N, t->l[i].kind, t->l[i].n, t->l[i].pol, "!");
    }
}

static void report(const Net *N, Text *o)
{
    const Family *F = N->F;
    static const char *const role_name[] = {"", "in", "out", "bidir", "buried", "in+buried"};
    int list[MAXVP], n, i, t, m, unused = 0;
    char cellname[16];
    text_printf(o, "Device   : %s\n", N->device);
    text_printf(o, "Family   : %s, %d logic cells, package %s\n", F->name, F->N, pkg_name(N));
    text_printf(o, "Fuses    : %d (%s)\n", F->QF, N->is_pof ? "POF" : "JEDEC");
    text_printf(o, "Security : %s\n", N->security ? "on" : "off");
    text_printf(o, "Turbo    : %s\n", N->turbo ? "on" : "off");
    text_put(o, "\nPINS\n");
    n = ordered(N, ~0u, list);
    for (i = 0; i < n; i++) {
        m = N->cellpin[list[i]];
        if (m) sprintf(cellname, "LC%d", m); else strcpy(cellname, "-");
        text_printf(o, "%-5s %-10s %-5s p%s\n", pin_name(N, list[i]), role_name[N->role[list[i]]], cellname, pin_name(N, list[i]));
    }
    text_put(o, "\nEQUATIONS\n");
    for (i = 0; i < N->nlogic; i++) {
        const Cell *c;
        m = N->logic[i];
        c = &N->c[m];
        text_printf(o, "lc%d (LC%d, pin %s): %s%s\n", m, m, pin_name(N, F->cell_vp[m]),
                    c->reg == REG_NONE ? "combinational" : (c->reg == REG_D ? "D register" : "T register"), c->inv ? ", inverted" : "");
        text_printf(o, "  %-3s = ", c->reg == REG_NONE ? "out" : (c->reg == REG_D ? "d" : "t"));
        if (c->nterms == 0) text_put(o, "GND");
        for (t = 0; t < c->nterms; t++) {
            if (t) text_put(o, "\n      # ");
            rep_prod(o, N, &c->terms[t]);
        }
        text_put(o, "\n");
        if (c->reg != REG_NONE) {
            if (c->clk == CLK_GLOBAL) text_printf(o, "  clk = global p%s\n", pin_name(N, c->clk_vp));
            else if (c->clk == CLK_NEVER) text_put(o, "  clk = GND\n");
            else { text_put(o, "  clk = "); rep_prod(o, N, &c->clkt); text_put(o, "\n"); }
            if (!c->has_clr) text_put(o, "  clr = never\n");
            else { text_put(o, "  clr = "); rep_prod(o, N, &c->clrt); text_put(o, "\n"); }
        }
        if (c->oe == OE_ON) text_put(o, "  oe  = always\n");
        else if (c->oe == OE_OFF) text_put(o, "  oe  = never\n");
        else { text_put(o, "  oe  = "); rep_prod(o, N, &c->oet); text_put(o, "\n"); }
        text_printf(o, "  feedback line carries the %s\n", c->fbpin ? "pin" : "cell");
    }
    text_put(o, "\nNOTES\n");
    racing(N, o);
    for (m = 1; m <= F->N; m++) if (N->c[m].cls == UNUSED) unused++;
    text_printf(o, "unused logic cells: %d of %d\n", unused, F->N);
}

/* ---------------------------------------------------------------- Verilog description */

static void v_lit(Text *o, const Net *N, const Lit *l)
{
    if (!l->pol) text_put(o, "~");
    if (l->kind == 'C') text_printf(o, "lc%d", l->n);
    else if (N->role[l->n] == R_IN) text_printf(o, "p%s", pin_name(N, l->n));
    else text_printf(o, "n_p%s", pin_name(N, l->n));
}

static void v_prod(Text *o, const Net *N, const Term *t)
{
    int i;
    if (t->n == 0) { text_put(o, "1'b1"); return; }
    for (i = 0; i < t->n; i++) { if (i) text_put(o, " & "); v_lit(o, N, &t->l[i]); }
}

static void v_sop(Text *o, const Net *N, const Cell *c)
{
    int t;
    if (c->nterms == 0) { text_put(o, "1'b0"); return; }
    for (t = 0; t < c->nterms; t++) {
        if (t) text_put(o, " | ");
        text_put(o, "(");
        v_prod(o, N, &c->terms[t]);
        text_put(o, ")");
    }
}

static void verilog(const Net *N, const char *top, Text *o)
{
    const Family *F = N->F;
    int list[MAXVP], n, i, m, first = 1;
    text_printf(o, "module %s(", top);
    n = ordered(N, 1u << R_IN, list);
    for (i = 0; i < n; i++) { text_printf(o, "%sinput p%s", first ? "" : ", ", pin_name(N, list[i])); first = 0; }
    n = ordered(N, (1u << R_BIDIR) | (1u << R_INBURIED), list);
    for (i = 0; i < n; i++) { text_printf(o, "%sinput p%s_in", first ? "" : ", ", pin_name(N, list[i])); first = 0; }
    for (i = 0; i < N->nlogic; i++) {
        const char *p = pin_name(N, F->cell_vp[N->logic[i]]);
        text_printf(o, "%soutput p%s, output p%s__oe", first ? "" : ", ", p, p);
        first = 0;
    }
    text_put(o, ");\n");
    for (i = 0; i < N->nlogic; i++) {
        const Cell *c;
        const char *p;
        int vp;
        m = N->logic[i];
        c = &N->c[m];
        vp = F->cell_vp[m];
        p = pin_name(N, vp);
        if (c->reg == REG_NONE) {
            text_printf(o, "  wire lc%d = ", m);
            if (c->inv) text_put(o, "~(");
            v_sop(o, N, c);
            if (c->inv) text_put(o, ")");
            text_put(o, ";\n");
        } else {
            text_printf(o, "  wire ck%d = ", m);
            if (c->clk == CLK_GLOBAL) text_printf(o, "p%s", pin_name(N, c->clk_vp));
            else if (c->clk == CLK_NEVER) text_put(o, "1'b0");
            else v_prod(o, N, &c->clkt);
            text_put(o, ";\n");
            text_printf(o, "  reg lc%d = 1'b0;\n", m);
            if (c->has_clr) {
                text_printf(o, "  wire cl%d = ", m);
                v_prod(o, N, &c->clrt);
                text_put(o, ";\n");
                text_printf(o, "  always @(posedge ck%d or posedge cl%d) if (cl%d) lc%d <= 1'b0; else lc%d <= ", m, m, m, m, m);
            } else text_printf(o, "  always @(posedge ck%d) lc%d <= ", m, m);
            if (c->reg == REG_T) text_printf(o, "lc%d ^ (", m);
            if (c->inv) text_put(o, "~(");
            v_sop(o, N, c);
            if (c->inv) text_put(o, ")");
            if (c->reg == REG_T) text_put(o, ")");
            text_put(o, ";\n");
        }
        text_printf(o, "  assign p%s__oe = ", p);
        if (c->oe == OE_ON) text_put(o, "1'b1");
        else if (c->oe == OE_OFF) text_put(o, "1'b0");
        else v_prod(o, N, &c->oet);
        text_put(o, ";\n");
        text_printf(o, "  assign p%s = lc%d;\n", p, m);
        if (N->role[vp] == R_BIDIR || N->role[vp] == R_INBURIED) text_printf(o, "  wire n_p%s = p%s__oe ? lc%d : p%s_in;\n", p, p, m, p);
        else text_printf(o, "  wire n_p%s = lc%d;\n", p, m);
    }
    text_put(o, "endmodule\n");
}

/* ---------------------------------------------------------------- MAX+PLUS II design and assignment files */

/* Assignment file: the device, one line per pin or buried cell, automatic global clock off
   (a product-term clock stays a product term), and a synthesis style KEEP with two settings:
   no minimisation (every product term stays as read) and the Turbo Bit as read. */

static void a_lit(Text *o, const Net *N, const Lit *l)
{
    int m;
    if (!l->pol) text_put(o, "!");
    if (l->kind == 'C') { text_printf(o, "lc%d", l->n); return; }
    m = N->cellpin[l->n];
    if (m && N->role[l->n] == R_OUT) text_printf(o, "lc%d", m);
    else text_printf(o, "p%s", pin_name(N, l->n));
}

static void a_prod(Text *o, const Net *N, const Term *t)
{
    int i;
    if (t->n == 0) { text_put(o, "VCC"); return; }
    for (i = 0; i < t->n; i++) { if (i) text_put(o, " & "); a_lit(o, N, &t->l[i]); }
}

static void a_ports(Text *o, const Net *N, unsigned mask, const char *kind)
{
    int list[MAXVP], n, i;
    n = ordered(N, mask, list);
    if (!n) return;
    text_put(o, "\t");
    for (i = 0; i < n; i++) text_printf(o, "%sp%s", i ? ", " : "", pin_name(N, list[i]));
    text_printf(o, " : %s;\n", kind);
}

static void ahdl(const Net *N, const char *project, Text *tdf, Text *acf)
{
    const Family *F = N->F;
    int list[MAXVP], n, i, m, t;
    for (m = 1; m <= F->N; m++)
        if (N->c[m].cls == LOGIC && N->role[F->cell_vp[m]] == R_INBURIED)
            refuse("EMIT_PIN_READ_BESIDE_BURIED_CELL", "pin %s", pin_name(N, F->cell_vp[m]));
    text_printf(tdf, "SUBDESIGN %s\n(\n", project);
    a_ports(tdf, N, 1u << R_IN, "INPUT");
    a_ports(tdf, N, 1u << R_OUT, "OUTPUT");
    a_ports(tdf, N, 1u << R_BIDIR, "BIDIR");
    text_put(tdf, ")\n");
    if (N->nlogic) text_put(tdf, "VARIABLE\n");
    for (i = 0; i < N->nlogic; i++) {
        m = N->logic[i];
        text_printf(tdf, "\tlc%d : %s;\n", m, N->c[m].reg == REG_NONE ? "LCELL" : (N->c[m].reg == REG_D ? "DFF" : "TFF"));
    }
    text_put(tdf, "BEGIN\n");
    for (i = 0; i < N->nlogic; i++) {
        const Cell *c;
        const char *p;
        m = N->logic[i];
        c = &N->c[m];
        p = pin_name(N, F->cell_vp[m]);
        if (c->reg == REG_NONE) text_printf(tdf, "\tlc%d = ", m);
        else text_printf(tdf, "\tlc%d.%s = ", m, c->reg == REG_D ? "d" : "t");
        if (c->inv) text_put(tdf, "!(");
        if (c->nterms == 0) text_put(tdf, "GND");
        for (t = 0; t < c->nterms; t++) {
            if (t) text_put(tdf, "\n\t\t# ");
            a_prod(tdf, N, &c->terms[t]);
        }
        if (c->inv) text_put(tdf, ")");
        text_put(tdf, ";\n");
        if (c->reg != REG_NONE) {
            if (c->clk == CLK_GLOBAL) text_printf(tdf, "\tlc%d.clk = GLOBAL(p%s);\n", m, pin_name(N, c->clk_vp));
            else if (c->clk == CLK_NEVER) text_printf(tdf, "\tlc%d.clk = GND;\n", m);
            else { text_printf(tdf, "\tlc%d.clk = ", m); a_prod(tdf, N, &c->clkt); text_put(tdf, ";\n"); }
            if (c->has_clr) { text_printf(tdf, "\tlc%d.clrn = !(", m); a_prod(tdf, N, &c->clrt); text_put(tdf, ");\n"); }
        }
        if (c->oe == OE_ON) text_printf(tdf, "\tp%s = lc%d;\n", p, m);
        else if (c->oe == OE_TERM) { text_printf(tdf, "\tp%s = TRI(lc%d, ", p, m); a_prod(tdf, N, &c->oet); text_put(tdf, ");\n"); }
    }
    text_put(tdf, "END;\n");
    text_printf(acf, "CHIP %s\nBEGIN\n\tDEVICE = %s;\n", project, N->device);
    n = ordered(N, ~0u, list);
    for (i = 0; i < n; i++) {
        int r = N->role[list[i]];
        if (r == R_BURIED) text_printf(acf, "\t|lc%d : LOCATION = LC%d;\n", N->cellpin[list[i]], N->cellpin[list[i]]);
        else text_printf(acf, "\t|p%s : %s_PIN = %s;\n", pin_name(N, list[i]), r == R_IN ? "INPUT" : (r == R_OUT ? "OUTPUT" : "BIDIR"), pin_name(N, list[i]));
    }
    text_put(acf, "END;\n\nGLOBAL_PROJECT_SYNTHESIS_ASSIGNMENT_OPTIONS\nBEGIN\n\tAUTO_GLOBAL_CLOCK = OFF;\n\tSTYLE = KEEP;\nEND;\n\n");
    if (N->security) text_put(acf, "GLOBAL_PROJECT_DEVICE_OPTIONS\nBEGIN\n\tSECURITY_BIT = ON;\nEND;\n\n");
    text_printf(acf, "DEFINE_LOGIC_SYNTHESIS_STYLE KEEP.CLASSIC\nBEGIN\n\tMINIMIZATION = NONE;\n\tTURBO_BIT = %s;\nEND;\n", N->turbo ? "ON" : "OFF");
}

/* ---------------------------------------------------------------- commands */

static FuseFile file_a, file_b;
static Net net;

/* read a file's bytes, name its device and family */
static const Part *load(const unsigned char *d, size_t len, const char *dopt, FuseFile *ff, const char **device)
{
    const Part *part, *fp;
    const char *dev;
    size_t i;
    read_any(d, len, ff);
    if (dopt) {
        if (strlen(dopt) > 32) refuse("DEVICE_UNKNOWN", "bad name");
        for (i = 0; dopt[i]; i++)
            if ((unsigned char)dopt[i] < 33 || (unsigned char)dopt[i] > 126) refuse("DEVICE_UNKNOWN", "bad name");
    }
    dev = dopt ? dopt : (ff->device[0] ? ff->device : NULL);
    if (!dev) refuse("DEVICE_NOT_NAMED", "the file names no device, give one with -d");
    part = find_part(dev);
    if (!part) refuse("DEVICE_UNKNOWN", "%s", dev);
    if (dopt && ff->device[0]) {
        fp = find_part(ff->device);
        if (fp && fp->family != part->family) refuse("DEVICE_FAMILY_MISMATCH", "the file is for %s", ff->device);
    }
    if (ff->nfuse != FAMILY[part->family].QF)
        refuse("FUSE_COUNT", "%d fuses in the file, %s has %d", ff->nfuse, dev, FAMILY[part->family].QF);
    *device = part->name;
    return part;
}

static void decode_bytes(const unsigned char *d, size_t len, const char *dopt)
{
    const char *device;
    const Part *part = load(d, len, dopt, &file_a, &device);
    decode(&file_a, &FAMILY[part->family], &net);
    net.mode = part->mode;
    net.device = device;
    net.security = file_a.security;
    net.is_pof = file_a.is_pof;
    make_view(&net);
}

static void do_decode(const unsigned char *d, size_t len, const char *dopt, Text *o)
{
    decode_bytes(d, len, dopt);
    report(&net, o);
}

static void do_emit(const unsigned char *d, size_t len, const char *dopt, const char *name, Text *tdf, Text *acf, Text *v, Text *rep)
{
    decode_bytes(d, len, dopt);
    ahdl(&net, name, tdf, acf);
    verilog(&net, name, v);
    report(&net, rep);
}

static void where(const Family *F, int i, char *buf)
{
    if (i < F->A) sprintf(buf, "LC%d term %d line %d", i / (10 * F->W) + 1, (i / F->W) % 10, i % F->W);
    else if (i - F->A < 5 * F->N) sprintf(buf, "LC%d architecture %d", (i - F->A) / 5 + 1, (i - F->A) % 5);
    else strcpy(buf, "device");
}

static void do_dump(const unsigned char *d, size_t len, const char *dopt, Text *o)
{
    const char *device;
    const Part *part = load(d, len, dopt, &file_a, &device);
    const Family *F = &FAMILY[part->family];
    const unsigned char *fz = file_a.fuse;
    int mc, t, i;
    char line[96];
    text_printf(o, "Device   : %s\nFuses    : %d (%s)\nSecurity : %s\n\n", device, F->QF, file_a.is_pof ? "POF" : "JEDEC", file_a.security ? "on" : "off");
    for (mc = 1; mc <= F->N; mc++) {
        const unsigned char *k = fz + F->A + 5 * (mc - 1);
        text_printf(o, "LC%d pin %s architecture %d%d%d%d%d\n", mc, (part->mode == F->mode_a ? F->pin_a : F->pin_b)[F->cell_vp[mc]], k[0], k[1], k[2], k[3], k[4]);
        for (t = 0; t < 10; t++) {
            for (i = 0; i < F->W; i++) line[i] = (char)('0' + fz[(long)(mc - 1) * 10 * F->W + t * F->W + i]);
            line[F->W] = 0;
            text_printf(o, " %d %s\n", t, line);
        }
    }
    text_printf(o, "device fuses %d-%d: ", F->A + 5 * F->N, F->QF - 1);
    for (i = F->A + 5 * F->N; i < F->QF; i++) text_printf(o, "%d", fz[i]);
    text_put(o, "\n");
}

static void do_diff(const unsigned char *da, size_t la, const unsigned char *db, size_t lb, Text *o)
{
    const Family *F = NULL;
    int i, n = 0;
    char buf[64];
    read_any(da, la, &file_a);
    read_any(db, lb, &file_b);
    if (file_a.nfuse != file_b.nfuse) refuse("DIFF_FUSE_COUNT", "%d and %d fuses", file_a.nfuse, file_b.nfuse);
    for (i = 0; i < NFAMILY; i++) if (FAMILY[i].QF == file_a.nfuse) F = &FAMILY[i];
    if (!F) refuse("FUSE_COUNT", "%d fuses", file_a.nfuse);
    for (i = 0; i < F->QF; i++) {
        if (file_a.fuse[i] == file_b.fuse[i]) continue;
        n++;
        where(F, i, buf);
        text_printf(o, "fuse %d (%s): %d -> %d\n", i, buf, file_a.fuse[i], file_b.fuse[i]);
    }
    if (file_a.security != file_b.security) text_printf(o, "security: %d -> %d\n", file_a.security, file_b.security);
    text_printf(o, "differing fuses: %d of %d\n", n, F->QF);
}

static void do_devices(Text *o)
{
    int i;
    for (i = 0; i < NPART; i++) {
        const Family *F = &FAMILY[PART[i].family];
        text_printf(o, "%-18s %-7s %s\n", PART[i].name, F->name, PART[i].mode == F->mode_a ? F->pkg_a : F->pkg_b);
    }
}

/* ---------------------------------------------------------------- built-in checks */

static unsigned long long fnv(unsigned long long h, const char *s, size_t n)
{
    size_t i;
    for (i = 0; i < n; i++) { h ^= (unsigned char)s[i]; h *= 1099511628211ULL; }
    return h;
}
#define FNV0 14695981039346656037ULL

/* build a JEDEC file as MAX+PLUS II writes it */
static size_t write_jed(const char *device, int security, const unsigned char *fz, int nfuse, int W, unsigned char *out)
{
    size_t n = 0;
    int i;
    unsigned sum = 0;
    n += (size_t)sprintf((char *)out + n, "\x02" "self-test\r\n*\r\nQF %d *\r\nND %s *\r\nN written by ep1810dump *\r\nG%d*\r\nL0 \r\n", nfuse, device, security);
    for (i = 0; i < nfuse; i++) {
        out[n++] = (unsigned char)('0' + fz[i]);
        if (i % W == W - 1 || i == nfuse - 1) { out[n++] = '\r'; out[n++] = '\n'; }
    }
    out[n++] = '*';
    out[n++] = 3;
    for (i = 0; i < (int)n; i++) sum += out[i];
    n += (size_t)sprintf((char *)out + n, "%04X\r\n", sum & 0xFFFF);
    return n;
}

static void put16(unsigned char *p, unsigned v) { p[0] = (unsigned char)(v & 255); p[1] = (unsigned char)(v >> 8); }
static void put32(unsigned char *p, unsigned long v) { put16(p, (unsigned)(v & 0xFFFF)); put16(p + 2, (unsigned)(v >> 16)); }

/* build a POF as MAX+PLUS II writes it (bits of the matrix bytes that hold no fuse are 1) */
static size_t write_pof(const char *device, int security, const unsigned char *fz, unsigned char *out)
{
    const Part *part = find_part(device);
    const Family *F = &FAMILY[part->family];
    static unsigned char image[6 * 10 * 88], arch[32];
    static const char creator[] = "ep1810dump";
    size_t n = 12, na = (size_t)((F->tag6_count + 7) / 8), l;
    int mc, pt, line, i;
    unsigned crc;
    memset(image, 0xFF, sizeof image);
    memset(arch, 0xFF, sizeof arch);
    for (mc = 0; mc < F->N; mc++)
        for (pt = 0; pt < 10; pt++)
            for (line = 0; line < F->W; line++) {
                long a = F->bitbase[mc] + 8L * F->W * pt + 8L * line;
                if (!fz[(mc * 10 + pt) * F->W + line]) image[a >> 3] &= (unsigned char)~(1u << (a & 7));
            }
    for (i = 0; i < F->tag6_count; i++)
        if (!fz[F->tag6_first + i]) arch[i >> 3] &= (unsigned char)~(1u << (i & 7));
    memcpy(out, "POF\0", 4);
    put32(out + 4, 0x00010000UL);
    put32(out + 8, (unsigned long)(5 + F->ntag7));
    l = sizeof creator; put16(out + n, 1); put32(out + n + 2, (unsigned long)l); memcpy(out + n + 6, creator, l); n += 6 + l;
    l = strlen(device) + 1; put16(out + n, 2); put32(out + n + 2, (unsigned long)l); memcpy(out + n + 6, device, l); n += 6 + l;
    put16(out + n, 5); put32(out + n + 2, 2); put16(out + n + 6, (unsigned)security); n += 8;
    for (i = 0; i < F->ntag7; i++) {
        const int *e = F->tag7 + 5 * i;
        l = (size_t)(10 * F->W);
        put16(out + n, 7); put32(out + n + 2, (unsigned long)(12 + l));
        put16(out + n + 6, (unsigned)e[0]); put16(out + n + 8, (unsigned)e[1]); put16(out + n + 10, (unsigned)e[2]);
        put16(out + n + 12, (unsigned)e[3]); put16(out + n + 14, (unsigned)e[4]); put16(out + n + 16, 0);
        memcpy(out + n + 18, image + (size_t)i * l, l);
        n += 18 + l;
    }
    put16(out + n, 6); put32(out + n + 2, (unsigned long)(8 + na));
    put32(out + n + 6, (unsigned long)F->tag6_first); put16(out + n + 10, (unsigned)F->tag6_count); put16(out + n + 12, 1);
    memcpy(out + n + 14, arch, na);
    n += 14 + na;
    put16(out + n, 8); put32(out + n + 2, 2);
    n += 6;
    crc = crc16(out, n);
    put16(out + n, crc);
    return n + 2;
}

/*
 * One built-in check: a design given as fuses (run-length coded in selftest_data.h),
 * wrapped into a JEDEC or POF file in memory, changed as the case says, then run
 * through one command. The expected result is a hash of the text written, or the
 * name of the refusal.
 */
static unsigned char st_fuse[MAXFUSE], st_file[60000], st_file2[60000];

static void st_expand(const SelfDesign *D)
{
    int i, n = 0, v = D->first;
    for (i = 0; i < D->nruns; i++) {
        memset(st_fuse + n, v, (size_t)D->runs[i]);
        n += D->runs[i];
        v ^= 1;
    }
}

static size_t st_build(const SelfCase *C, const SelfDesign *D, int pof, unsigned char *out)
{
    const Family *F = &FAMILY[find_part(D->device)->family];
    return pof ? write_pof(D->device, C->security, st_fuse, out) : write_jed(D->device, C->security, st_fuse, F->QF, F->W, out);
}

static int st_run(const SelfCase *C, int planted)
{
    const SelfDesign *D = &SELF_DESIGN[C->design];
    int clean = planted && C->refusal[0];       /* a refusal case run without its defect */
    const char *dopt = clean ? NULL : C->dopt;
    size_t n;
    volatile size_t n2 = 0;
    Text a = {0, 0, 0}, b = {0, 0, 0}, c = {0, 0, 0}, d = {0, 0, 0};
    volatile unsigned long long h = FNV0;
    const char *volatile got = "";
    int ok;
    st_expand(D);
    if (C->flip >= 0 && !clean) st_fuse[C->flip] ^= 1;
    if (C->cmd == 3) n2 = st_build(C, D, 0, st_file2);
    if (planted && !C->refusal[0] && C->plant >= 0) st_fuse[C->plant] ^= 1;
    n = st_build(C, D, C->cmd == 3 ? 1 : C->pof, st_file);
    if (C->damage == 1 && !clean) {
        if (C->pof) st_file[n - 1] ^= 0xFF;
        else st_file[n - 3] = (unsigned char)(st_file[n - 3] == '0' ? '1' : '0');
    }
    if (SETJMP(refuse_jmp)) {
        got = refuse_name;
    } else {
        switch (C->cmd) {
        case 0: do_decode(st_file, n, dopt, &a); break;
        case 1: do_emit(st_file, n, dopt, "design", &a, &b, &c, &d); break;
        case 2: do_dump(st_file, n, dopt, &a); break;
        default: do_diff(st_file2, n2, st_file, n, &a); break;
        }
        if (a.s) h = fnv(h, a.s, a.n);
        if (b.s) h = fnv(h, b.s, b.n);
        if (c.s) h = fnv(h, c.s, c.n);
        if (d.s) h = fnv(h, d.s, d.n);
    }
    if (C->refusal[0]) ok = strcmp(got, C->refusal) == 0;
    else ok = (got[0] == 0 && h == C->hash);
    text_free(&a); text_free(&b); text_free(&c); text_free(&d);
    return ok;
}

static int selftest(int fail)
{
    int i, pass = 0, bad = 0;
    for (i = 0; i < NSELFCASE; i++) {
        int ok = st_run(&SELF_CASE[i], fail);
        if (!fail) {
            printf("selftest %d %s: %s\n", i + 1, SELF_CASE[i].name, ok ? "ok" : "FAILED");
            if (ok) pass++; else bad++;
        } else {
            printf("selftest --fail %d %s: %s\n", i + 1, SELF_CASE[i].name, ok ? "NOT DETECTED" : "failed as planned");
            if (!ok) pass++; else bad++;
        }
    }
    if (!fail) printf("selftest: %d passed, %d failed\n", pass, bad);
    else printf("selftest --fail: %d of %d planted defects detected\n", pass, NSELFCASE);
    return bad ? 1 : 0;
}

/* ---------------------------------------------------------------- main */

static const char USAGE[] =
    "usage: ep1810dump decode <file> [-d DEVICE] [-o FILE]\n"
    "       ep1810dump emit <file> [-d DEVICE] [-n NAME] -o DIR\n"
    "       ep1810dump dump <file> [-d DEVICE] [-o FILE]\n"
    "       ep1810dump diff <file A> <file B> [-o FILE]\n"
    "       ep1810dump devices\n"
    "       ep1810dump selftest [--fail]\n"
    "       ep1810dump --version\n";

static unsigned char *slurp(const char *path, size_t *len)
{
    FILE *f = fopen(path, "rb");
    unsigned char *buf = NULL;
    size_t n = 0, cap = 0, r;
    if (!f) { fprintf(stderr, "ep1810dump: cannot read %s\n", path); return NULL; }
    for (;;) {
        if (n + 65536 > cap) {
            cap = cap ? cap * 2 : 131072;
            buf = (unsigned char *)realloc(buf, cap);
            if (!buf) die("out of memory");
        }
        r = fread(buf + n, 1, 65536, f);
        n += r;
        if (r < 65536) break;
    }
    if (ferror(f)) { fclose(f); free(buf); fprintf(stderr, "ep1810dump: cannot read %s\n", path); return NULL; }
    fclose(f);
    *len = n;
    return buf;
}

static int spill(const char *path, const Text *t)
{
    FILE *f = fopen(path, "wb");
    if (!f) return 0;
    if (t->n && fwrite(t->s, 1, t->n, f) != t->n) { fclose(f); return 0; }
    return fclose(f) == 0;
}

static int name_ok(const char *s)
{
    size_t i, l = strlen(s);
    if (l < 1 || l > 32) return 0;
    if (!((s[0] >= 'A' && s[0] <= 'Z') || (s[0] >= 'a' && s[0] <= 'z'))) return 0;
    for (i = 0; i < l; i++)
        if (!((s[i] >= 'A' && s[i] <= 'Z') || (s[i] >= 'a' && s[i] <= 'z') || (s[i] >= '0' && s[i] <= '9') || s[i] == '_')) return 0;
    return 1;
}

int main(int argc, char **argv)
{
    const char *cmd, *pos[4], *od = NULL, *oo = NULL, *on = NULL;
    int npos = 0, i, fail = 0, nopt = 0;
    unsigned char *da = NULL, *db = NULL;
    size_t la = 0, lb = 0;
    Text out = {0, 0, 0}, t2 = {0, 0, 0}, t3 = {0, 0, 0}, t4 = {0, 0, 0};

#ifdef _WIN32
    _setmode(_fileno(stdout), _O_BINARY);
    _setmode(_fileno(stderr), _O_BINARY);
#endif
    fprintf(stderr, "ep1810 decoder tool by Darksoft v%s. Use at your own risk.\n", VERSION);
    if (argc == 2 && strcmp(argv[1], "--version") == 0) { printf("ep1810dump %s\n", VERSION); return 0; }
    if (argc < 2) { fputs(USAGE, stderr); return 2; }
    cmd = argv[1];
    for (i = 2; i < argc; i++) {
        const char *a = argv[i];
        if (strcmp(a, "-d") == 0 || strcmp(a, "-o") == 0 || strcmp(a, "-n") == 0) {
            const char **slot = a[1] == 'd' ? &od : (a[1] == 'o' ? &oo : &on);
            if (i + 1 >= argc || *slot) { fputs(USAGE, stderr); return 2; }
            *slot = argv[++i];
            nopt++;
        } else if (strcmp(a, "--fail") == 0 && strcmp(cmd, "selftest") == 0) fail = 1;
        else if (a[0] == '-') { fputs(USAGE, stderr); return 2; }
        else if (npos < 4) pos[npos++] = a;
        else npos++;
    }
    if (strcmp(cmd, "devices") == 0 && npos == 0 && nopt == 0) {
        do_devices(&out);
    } else if (strcmp(cmd, "selftest") == 0 && npos == 0 && nopt == 0) {
        return selftest(fail);
    } else if ((strcmp(cmd, "decode") == 0 || strcmp(cmd, "dump") == 0) && npos == 1 && !on) {
        da = slurp(pos[0], &la);
        if (!da) return 1;
        if (SETJMP(refuse_jmp)) goto refused;
        if (cmd[1] == 'e') do_decode(da, la, od, &out); else do_dump(da, la, od, &out);
    } else if (strcmp(cmd, "diff") == 0 && npos == 2 && !od && !on) {
        da = slurp(pos[0], &la);
        db = slurp(pos[1], &lb);
        if (!da || !db) return 1;
        if (SETJMP(refuse_jmp)) goto refused;
        do_diff(da, la, db, lb, &out);
    } else if (strcmp(cmd, "emit") == 0 && npos == 1 && oo) {
        const char *volatile name = on ? on : "design";
        static const char *const ext[4] = {".tdf", ".acf", ".v", ""};
        const Text *tx[4];
        char path[1200];
        if (!name_ok(name)) { fputs("ep1810dump: the name must be a letter followed by letters, digits or _ (at most 32)\n", stderr); return 2; }
        da = slurp(pos[0], &la);
        if (!da) return 1;
        if (SETJMP(refuse_jmp)) goto refused;
        do_emit(da, la, od, name, &out, &t2, &t3, &t4);
        tx[0] = &out; tx[1] = &t2; tx[2] = &t3; tx[3] = &t4;
        if (strlen(oo) > 1000) { fprintf(stderr, "ep1810dump: cannot write into %s\n", oo); return 1; }
        for (i = 0; i < 4; i++) {
            if (i < 3) sprintf(path, "%s/%s%s", oo, name, ext[i]);
            else sprintf(path, "%s/REPORT.txt", oo);
            if (!spill(path, tx[i])) { fprintf(stderr, "ep1810dump: cannot write into %s\n", oo); return 1; }
        }
        return 0;
    } else {
        fputs(USAGE, stderr);
        return 2;
    }
    if (oo) {
        if (!spill(oo, &out)) { fprintf(stderr, "ep1810dump: cannot write %s\n", oo); return 1; }
    } else if (out.n) {
        fwrite(out.s, 1, out.n, stdout);
    }
    return 0;
refused:
    if (refuse_detail[0]) fprintf(stderr, "ep1810dump: refused: %s (%s)\n", refuse_name, refuse_detail);
    else fprintf(stderr, "ep1810dump: refused: %s\n", refuse_name);
    return 3;
}
