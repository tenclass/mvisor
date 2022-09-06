/* -*- Mode: C; c-basic-offset: 4; indent-tabs-mode: nil -*- */
/*
   Copyright (C) 2009 Red Hat, Inc.

   This library is free software; you can redistribute it and/or
   modify it under the terms of the GNU Lesser General Public
   License as published by the Free Software Foundation; either
   version 2.1 of the License, or (at your option) any later version.

   This library is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
   Lesser General Public License for more details.

   You should have received a copy of the GNU Lesser General Public
   License along with this library; if not, see <http://www.gnu.org/licenses/>.
*/
#include "config.h"

#define COMPRESS_IMP

#if defined(ONE_BYTE) || defined(FOUR_BYTE)
#  define FARGS_DECL(...) (Encoder *encoder, Channel *channel_a, __VA_ARGS__)
#  define FARGS_CALL(...) (encoder, channel_a, __VA_ARGS__)
#  define UNCOMPRESS_PIX_START(row) do { } while (0)
#  define SET_a(pix, val) ((pix)->a = val)
#  define GET_a(pix) ((pix)->a)
#  define SAME_PIXEL(p1, p2) (GET_a(p1) == GET_a(p2))
#  define COPY_PIXEL(dest, src) \
    SET_a(dest, GET_a(src));
#  define DECLARE_STATE_VARIABLES \
    CommonState *state = &channel_a->state
#  define DECLARE_CHANNEL_VARIABLES \
    BYTE * const correlate_row_a = channel_a->correlate_row
#  define APPLY_ALL_COMP(macro, ...) \
    macro(a, ## __VA_ARGS__)
#else
#  define FARGS_DECL(...) (Encoder *encoder, __VA_ARGS__)
#  define FARGS_CALL(...) (encoder, __VA_ARGS__)
#  define SAME_PIXEL(p1, p2)                               \
    (GET_r(p1) == GET_r(p2) && GET_g(p1) == GET_g(p2) &&   \
     GET_b(p1) == GET_b(p2))
#  define COPY_PIXEL(dest, src) \
    SET_r(dest, GET_r(src)); \
    SET_g(dest, GET_g(src)); \
    SET_b(dest, GET_b(src))
#  define DECLARE_STATE_VARIABLES \
    CommonState *state = &encoder->rgb_state
#  define DECLARE_CHANNEL_VARIABLES \
    Channel * const channel_r = encoder->channels; \
    Channel * const channel_g = channel_r + 1; \
    Channel * const channel_b = channel_g + 1; \
    BYTE * const correlate_row_r = channel_r->correlate_row; \
    BYTE * const correlate_row_g = channel_g->correlate_row; \
    BYTE * const correlate_row_b = channel_b->correlate_row
#  define APPLY_ALL_COMP(macro, ...) \
    macro(r, ## __VA_ARGS__); \
    macro(g, ## __VA_ARGS__); \
    macro(b, ## __VA_ARGS__)
#endif

#ifdef ONE_BYTE
#undef ONE_BYTE
#define FNAME(name) quic_one_##name
#define PIXEL one_byte_t
#define BPC 8
#endif

#ifdef FOUR_BYTE
#undef FOUR_BYTE
#define FNAME(name) quic_four_##name
#define PIXEL four_bytes_t
#define BPC 8
#endif

#ifdef QUIC_RGB32
#undef QUIC_RGB32
#define PIXEL rgb32_pixel_t
#define FNAME(name) quic_rgb32_##name
#define BPC 8
#define SET_r(pix, val) ((pix)->r = val)
#define GET_r(pix) ((pix)->r)
#define SET_g(pix, val) ((pix)->g = val)
#define GET_g(pix) ((pix)->g)
#define SET_b(pix, val) ((pix)->b = val)
#define GET_b(pix) ((pix)->b)
#define UNCOMPRESS_PIX_START(pix) ((pix)->pad = 0)
#endif

#ifdef QUIC_RGB24
#undef QUIC_RGB24
#define PIXEL rgb24_pixel_t
#define FNAME(name) quic_rgb24_##name
#define BPC 8
#define SET_r(pix, val) ((pix)->r = val)
#define GET_r(pix) ((pix)->r)
#define SET_g(pix, val) ((pix)->g = val)
#define GET_g(pix) ((pix)->g)
#define SET_b(pix, val) ((pix)->b = val)
#define GET_b(pix) ((pix)->b)
#define UNCOMPRESS_PIX_START(pix)
#endif

#ifdef QUIC_RGB16
#undef QUIC_RGB16
#define PIXEL rgb16_pixel_t
#define FNAME(name) quic_rgb16_##name
#define BPC 5
#define SET_r(pix, val) (*(pix) = (*(pix) & ~(0x1f << 10)) | ((val) << 10))
#define GET_r(pix) ((*(pix) >> 10) & 0x1f)
#define SET_g(pix, val) (*(pix) = (*(pix) & ~(0x1f << 5)) | ((val) << 5))
#define GET_g(pix) ((*(pix) >> 5) & 0x1f)
#define SET_b(pix, val) (*(pix) = (*(pix) & ~0x1f) | (val))
#define GET_b(pix) (*(pix) & 0x1f)
#define UNCOMPRESS_PIX_START(pix) (*(pix) = 0)
#endif

#ifdef QUIC_RGB16_TO_32
#undef QUIC_RGB16_TO_32
#define PIXEL rgb32_pixel_t
#define FNAME(name) quic_rgb16_to_32_##name
#define BPC 5
#undef COMPRESS_IMP
#define SET_r(pix, val) ((pix)->r = ((val) << 3) | (((val) & 0x1f) >> 2))
#define GET_r(pix) ((pix)->r >> 3)
#define SET_g(pix, val) ((pix)->g = ((val) << 3) | (((val) & 0x1f) >> 2))
#define GET_g(pix) ((pix)->g >> 3)
#define SET_b(pix, val) ((pix)->b = ((val) << 3) | (((val) & 0x1f) >> 2))
#define GET_b(pix) ((pix)->b >> 3)
#define UNCOMPRESS_PIX_START(pix) ((pix)->pad = 0)
#endif

#define FNAME_DECL(name) FNAME(name) FARGS_DECL
#define FNAME_CALL(name) FNAME(name) FARGS_CALL

#if BPC == 5
#  define golomb_coding golomb_coding_5bpc
#  define golomb_decoding golomb_decoding_5bpc
#  define update_model update_model_5bpc
#  define find_bucket find_bucket_5bpc
#  define family family_5bpc
#  define BPC_MASK 0x1fU
#elif BPC == 8
#  define golomb_coding golomb_coding_8bpc
#  define golomb_decoding golomb_decoding_8bpc
#  define update_model update_model_8bpc
#  define find_bucket find_bucket_8bpc
#  define family family_8bpc
#  define BPC_MASK 0xffU
#else
#  error BPC must be 5 or 8
#endif

#define _PIXEL_A(channel, curr) ((unsigned int)GET_##channel((curr) - 1))
#define _PIXEL_B(channel, prev) ((unsigned int)GET_##channel(prev))

/*  a  */

#define DECORRELATE_0(channel, curr, bpc_mask)\
    family.xlatU2L[(unsigned)((int)GET_##channel(curr) - (int)_PIXEL_A(channel, curr)) & bpc_mask]

#define CORRELATE_0(channel, curr, correlate, bpc_mask)\
    ((family.xlatL2U[correlate] + _PIXEL_A(channel, curr)) & bpc_mask)


/*  (a+b)/2  */
#define DECORRELATE(channel, prev, curr, bpc_mask, r)                                          \
    r = family.xlatU2L[(unsigned)((int)GET_##channel(curr) - (int)((_PIXEL_A(channel, curr) +  \
    _PIXEL_B(channel, prev)) >> 1)) & bpc_mask]

#define CORRELATE(channel, prev, curr, correlate, bpc_mask, r)                                  \
    SET_##channel(r, ((family.xlatL2U[correlate] +                                              \
         (int)((_PIXEL_A(channel, curr) + _PIXEL_B(channel, prev)) >> 1)) & bpc_mask))


#define COMPRESS_ONE_ROW0_0(channel)                                    \
    correlate_row_##channel[0] = family.xlatU2L[GET_##channel(cur_row)];\
    golomb_coding(encoder, correlate_row_##channel[0],                  \
                  find_bucket(channel_##channel,                        \
                              correlate_row_##channel[-1])->bestcode)

#define COMPRESS_ONE_ROW0(channel, index)                                               \
    correlate_row_##channel[index] = DECORRELATE_0(channel, &cur_row[index], bpc_mask); \
    golomb_coding(encoder, correlate_row_##channel[index],                              \
                  find_bucket(channel_##channel,                                        \
                              correlate_row_##channel[index - 1])->bestcode)

#define UPDATE_MODEL_COMP(channel, index)                                                   \
    update_model(state, find_bucket(channel_##channel, correlate_row_##channel[index - 1]), \
                 correlate_row_##channel[index])
#define UPDATE_MODEL(index) APPLY_ALL_COMP(UPDATE_MODEL_COMP, index)

#define RLE_PRED_IMP                                                                \
if (SAME_PIXEL(&prev_row[i - 1], &prev_row[i])) {                                   \
    if (run_index != i && i > 2 && SAME_PIXEL(&cur_row[i - 1], &cur_row[i - 2])) {  \
        goto do_run;                                                                \
    }                                                                               \
}

#ifdef COMPRESS_IMP

static void FNAME_DECL(compress_row0_seg)(int i,
                                          const PIXEL * const cur_row,
                                          const int end,
                                          const unsigned int waitmask,
                                          const unsigned int bpc_mask)
{
    DECLARE_STATE_VARIABLES;
    DECLARE_CHANNEL_VARIABLES;
    int stopidx;

    spice_assert(end - i > 0);

    if (i == 0) {
        APPLY_ALL_COMP(COMPRESS_ONE_ROW0_0);

        if (state->waitcnt) {
            state->waitcnt--;
        } else {
            state->waitcnt = (tabrand(&state->tabrand_seed) & waitmask);
            UPDATE_MODEL(0);
        }
        stopidx = ++i + state->waitcnt;
    } else {
        stopidx = i + state->waitcnt;
    }

    while (stopidx < end) {
        for (; i <= stopidx; i++) {
            APPLY_ALL_COMP(COMPRESS_ONE_ROW0, i);
        }

        UPDATE_MODEL(stopidx);
        stopidx = i + (tabrand(&state->tabrand_seed) & waitmask);
    }

    for (; i < end; i++) {
        APPLY_ALL_COMP(COMPRESS_ONE_ROW0, i);
    }
    state->waitcnt = stopidx - end;
}

#undef COMPRESS_ONE_ROW0_0
#undef COMPRESS_ONE_ROW0

static void FNAME_DECL(compress_row0)(const PIXEL *cur_row, unsigned int width)
{
    DECLARE_STATE_VARIABLES;
    const unsigned int bpc_mask = BPC_MASK;
    int pos = 0;

    while ((DEFwmimax > (int)state->wmidx) && (state->wmileft <= width)) {
        if (state->wmileft) {
            FNAME_CALL(compress_row0_seg)(pos, cur_row, pos + state->wmileft,
                                          bppmask[state->wmidx], bpc_mask);
            width -= state->wmileft;
            pos += state->wmileft;
        }

        state->wmidx++;
        set_wm_trigger(state);
        state->wmileft = DEFwminext;
    }

    if (width) {
        FNAME_CALL(compress_row0_seg)(pos, cur_row, pos + width,
                                      bppmask[state->wmidx], bpc_mask);
        if (DEFwmimax > (int)state->wmidx) {
            state->wmileft -= width;
        }
    }

    spice_assert((int)state->wmidx <= DEFwmimax);
    spice_assert(state->wmidx <= 32);
    spice_assert(DEFwminext > 0);
}

#define COMPRESS_ONE_0(channel) \
    correlate_row_##channel[0] = family.xlatU2L[(unsigned)((int)GET_##channel(cur_row) -              \
                                                          (int)GET_##channel(prev_row) ) & bpc_mask]; \
    golomb_coding(encoder, correlate_row_##channel[0],                                                \
                  find_bucket(channel_##channel, correlate_row_##channel[-1])->bestcode)

#define COMPRESS_ONE(channel, index)                                                                   \
     DECORRELATE(channel, &prev_row[index], &cur_row[index],bpc_mask, correlate_row_##channel[index]); \
     golomb_coding(encoder, correlate_row_##channel[index],                                            \
                   find_bucket(channel_##channel, correlate_row_##channel[index - 1])->bestcode)

static void FNAME_DECL(compress_row_seg)(int i,
                                         const PIXEL * const prev_row,
                                         const PIXEL * const cur_row,
                                         const int end,
                                         const unsigned int waitmask,
                                         const unsigned int bpc_mask)
{
    DECLARE_STATE_VARIABLES;
    DECLARE_CHANNEL_VARIABLES;
    int stopidx;
    int run_index = 0;
    int run_size;

    spice_assert(end - i > 0);

    if (i == 0) {
        APPLY_ALL_COMP(COMPRESS_ONE_0);

        if (state->waitcnt) {
            state->waitcnt--;
        } else {
            state->waitcnt = (tabrand(&state->tabrand_seed) & waitmask);
            UPDATE_MODEL(0);
        }
        stopidx = ++i + state->waitcnt;
    } else {
        stopidx = i + state->waitcnt;
    }
    for (;;) {
        while (stopidx < end) {
            for (; i <= stopidx; i++) {
                RLE_PRED_IMP;
                APPLY_ALL_COMP(COMPRESS_ONE, i);
            }

            UPDATE_MODEL(stopidx);
            stopidx = i + (tabrand(&state->tabrand_seed) & waitmask);
        }

        for (; i < end; i++) {
            RLE_PRED_IMP;
            APPLY_ALL_COMP(COMPRESS_ONE, i);
        }
        state->waitcnt = stopidx - end;

        return;

do_run:
        run_index = i;
        state->waitcnt = stopidx - i;
        run_size = 0;

        while (SAME_PIXEL(&cur_row[i], &cur_row[i - 1])) {
            run_size++;
            if (++i == end) {
                encode_state_run(encoder, state, run_size);
                return;
            }
        }
        encode_state_run(encoder, state, run_size);
        stopidx = i + state->waitcnt;
    }
}

static void FNAME_DECL(compress_row)(const PIXEL * const prev_row,
                                     const PIXEL * const cur_row,
                                     unsigned int width)

{
    DECLARE_STATE_VARIABLES;
    const unsigned int bpc_mask = BPC_MASK;
    unsigned int pos = 0;

    while ((DEFwmimax > (int)state->wmidx) && (state->wmileft <= width)) {
        if (state->wmileft) {
            FNAME_CALL(compress_row_seg)(pos, prev_row, cur_row,
                                         pos + state->wmileft, bppmask[state->wmidx],
                                         bpc_mask);
            width -= state->wmileft;
            pos += state->wmileft;
        }

        state->wmidx++;
        set_wm_trigger(state);
        state->wmileft = DEFwminext;
    }

    if (width) {
        FNAME_CALL(compress_row_seg)(pos, prev_row, cur_row, pos + width,
                                     bppmask[state->wmidx], bpc_mask);
        if (DEFwmimax > (int)state->wmidx) {
            state->wmileft -= width;
        }
    }

    spice_assert((int)state->wmidx <= DEFwmimax);
    spice_assert(state->wmidx <= 32);
    spice_assert(DEFwminext > 0);
}

#endif

#define UNCOMPRESS_ONE_ROW0_0(channel)                                                                      \
    correlate_row_##channel[0] = (BYTE)golomb_decoding(find_bucket(channel_##channel,                       \
                                                                   correlate_row_##channel[-1])->bestcode,  \
                                                       encoder->io_word, &codewordlen);                     \
    SET_##channel(&cur_row[0], (BYTE)family.xlatL2U[correlate_row_##channel[0]]);                           \
    decode_eatbits(encoder, codewordlen);

#define UNCOMPRESS_ONE_ROW0(channel)                                                                           \
    correlate_row_##channel[i] = (BYTE)golomb_decoding(find_bucket(channel_##channel,                          \
                                                                   correlate_row_##channel[i - 1])->bestcode,  \
                                                       encoder->io_word, &codewordlen);                        \
    SET_##channel(&cur_row[i], CORRELATE_0(channel, &cur_row[i], correlate_row_##channel[i],                   \
                  bpc_mask));                                                                                  \
    decode_eatbits(encoder, codewordlen);

static void FNAME_DECL(uncompress_row0_seg)(int i,
                                            PIXEL * const cur_row,
                                            const int end,
                                            const unsigned int waitmask,
                                            const unsigned int bpc_mask)
{
    DECLARE_STATE_VARIABLES;
    DECLARE_CHANNEL_VARIABLES;
    int stopidx;

    spice_assert(end - i > 0);

    if (i == 0) {
        unsigned int codewordlen;

        UNCOMPRESS_PIX_START(&cur_row[i]);
        APPLY_ALL_COMP(UNCOMPRESS_ONE_ROW0_0);

        if (state->waitcnt) {
            --state->waitcnt;
        } else {
            state->waitcnt = (tabrand(&state->tabrand_seed) & waitmask);
            UPDATE_MODEL(0);
        }
        stopidx = ++i + state->waitcnt;
    } else {
        stopidx = i + state->waitcnt;
    }

    while (stopidx < end) {
        for (; i <= stopidx; i++) {
            unsigned int codewordlen;

            UNCOMPRESS_PIX_START(&cur_row[i]);
            APPLY_ALL_COMP(UNCOMPRESS_ONE_ROW0);
        }
        UPDATE_MODEL(stopidx);
        stopidx = i + (tabrand(&state->tabrand_seed) & waitmask);
    }

    for (; i < end; i++) {
        unsigned int codewordlen;

        UNCOMPRESS_PIX_START(&cur_row[i]);
        APPLY_ALL_COMP(UNCOMPRESS_ONE_ROW0);
    }
    state->waitcnt = stopidx - end;
}

static void FNAME_DECL(uncompress_row0)(PIXEL * const cur_row,
                                        unsigned int width)

{
    DECLARE_STATE_VARIABLES;
    const unsigned int bpc_mask = BPC_MASK;
    unsigned int pos = 0;

    while ((DEFwmimax > (int)state->wmidx) && (state->wmileft <= width)) {
        if (state->wmileft) {
            FNAME_CALL(uncompress_row0_seg)(pos, cur_row,
                                            pos + state->wmileft,
                                            bppmask[state->wmidx],
                                            bpc_mask);
            pos += state->wmileft;
            width -= state->wmileft;
        }

        state->wmidx++;
        set_wm_trigger(state);
        state->wmileft = DEFwminext;
    }

    if (width) {
        FNAME_CALL(uncompress_row0_seg)(pos, cur_row, pos + width,
                                        bppmask[state->wmidx], bpc_mask);
        if (DEFwmimax > (int)state->wmidx) {
            state->wmileft -= width;
        }
    }

    spice_assert((int)state->wmidx <= DEFwmimax);
    spice_assert(state->wmidx <= 32);
    spice_assert(DEFwminext > 0);
}

#define UNCOMPRESS_ONE_0(channel)                                                                          \
    correlate_row_##channel[0] = (BYTE)golomb_decoding(find_bucket(channel_##channel,                      \
                                                                   correlate_row_##channel[-1])->bestcode, \
                                                       encoder->io_word, &codewordlen);                    \
    SET_##channel(&cur_row[0], (family.xlatL2U[correlate_row_##channel[0]] +                               \
                  GET_##channel(prev_row)) & bpc_mask);                                                    \
    decode_eatbits(encoder, codewordlen);

#define UNCOMPRESS_ONE(channel)                                                                               \
    correlate_row_##channel[i] = (BYTE)golomb_decoding(find_bucket(channel_##channel,                         \
                                                                   correlate_row_##channel[i - 1])->bestcode, \
                                                       encoder->io_word, &codewordlen);                       \
    CORRELATE(channel, &prev_row[i], &cur_row[i], correlate_row_##channel[i], bpc_mask,                       \
              &cur_row[i]);                                                                                   \
    decode_eatbits(encoder, codewordlen);

static void FNAME_DECL(uncompress_row_seg)(const PIXEL * const prev_row,
                                           PIXEL * const cur_row,
                                           int i,
                                           const int end,
                                           const unsigned int bpc_mask)
{
    DECLARE_STATE_VARIABLES;
    DECLARE_CHANNEL_VARIABLES;
    const unsigned int waitmask = bppmask[state->wmidx];
    int stopidx;
    int run_index = 0;
    int run_end;

    spice_assert(end - i > 0);

    if (i == 0) {
        unsigned int codewordlen;

        UNCOMPRESS_PIX_START(&cur_row[i]);
        APPLY_ALL_COMP(UNCOMPRESS_ONE_0);

        if (state->waitcnt) {
            --state->waitcnt;
        } else {
            state->waitcnt = (tabrand(&state->tabrand_seed) & waitmask);
            UPDATE_MODEL(0);
        }
        stopidx = ++i + state->waitcnt;
    } else {
        stopidx = i + state->waitcnt;
    }
    for (;;) {
        while (stopidx < end) {
            for (; i <= stopidx; i++) {
                unsigned int codewordlen;
                RLE_PRED_IMP;
                UNCOMPRESS_PIX_START(&cur_row[i]);
                APPLY_ALL_COMP(UNCOMPRESS_ONE);
            }

            UPDATE_MODEL(stopidx);

            stopidx = i + (tabrand(&state->tabrand_seed) & waitmask);
        }

        for (; i < end; i++) {
            unsigned int codewordlen;
            RLE_PRED_IMP;
            UNCOMPRESS_PIX_START(&cur_row[i]);
            APPLY_ALL_COMP(UNCOMPRESS_ONE);
        }

        state->waitcnt = stopidx - end;

        return;

do_run:
        state->waitcnt = stopidx - i;
        run_index = i;
        run_end = decode_state_run(encoder, state);
        if (run_end < 0 || run_end > (end - i)) {
            encoder->usr->error(encoder->usr, "wrong RLE\n");
        }
        run_end += i;

        for (; i < run_end; i++) {
            UNCOMPRESS_PIX_START(&cur_row[i]);
            COPY_PIXEL(&cur_row[i], &cur_row[i - 1]);
        }

        if (i == end) {
            return;
        }

        stopidx = i + state->waitcnt;
    }
}

static void FNAME_DECL(uncompress_row)(const PIXEL * const prev_row,
                                       PIXEL * const cur_row,
                                       unsigned int width)

{
    DECLARE_STATE_VARIABLES;
    const unsigned int bpc_mask = BPC_MASK;
    unsigned int pos = 0;

    while ((DEFwmimax > (int)state->wmidx) && (state->wmileft <= width)) {
        if (state->wmileft) {
            FNAME_CALL(uncompress_row_seg)(prev_row, cur_row, pos,
                                           pos + state->wmileft, bpc_mask);
            pos += state->wmileft;
            width -= state->wmileft;
        }

        state->wmidx++;
        set_wm_trigger(state);
        state->wmileft = DEFwminext;
    }

    if (width) {
        FNAME_CALL(uncompress_row_seg)(prev_row, cur_row, pos,
                                       pos + width, bpc_mask);
        if (DEFwmimax > (int)state->wmidx) {
            state->wmileft -= width;
        }
    }

    spice_assert((int)state->wmidx <= DEFwmimax);
    spice_assert(state->wmidx <= 32);
    spice_assert(DEFwminext > 0);
}

#undef PIXEL
#undef FARGS_CALL
#undef FARGS_DECL
#undef FNAME
#undef FNAME_CALL
#undef FNAME_DECL
#undef _PIXEL_A
#undef _PIXEL_B
#undef SAME_PIXEL
#undef RLE_PRED_IMP
#undef UPDATE_MODEL
#undef DECORRELATE_0
#undef DECORRELATE
#undef COMPRESS_ONE_0
#undef COMPRESS_ONE
#undef CORRELATE_0
#undef CORRELATE
#undef UNCOMPRESS_ONE_ROW0_0
#undef UNCOMPRESS_ONE_ROW0
#undef UNCOMPRESS_ONE_0
#undef UNCOMPRESS_ONE
#undef golomb_coding
#undef golomb_decoding
#undef update_model
#undef find_bucket
#undef family
#undef BPC
#undef BPC_MASK
#undef COMPRESS_IMP
#undef SET_r
#undef GET_r
#undef SET_g
#undef GET_g
#undef SET_b
#undef GET_b
#undef SET_a
#undef GET_a
#undef UNCOMPRESS_PIX_START
#undef UPDATE_MODEL_COMP
#undef APPLY_ALL_COMP
#undef DECLARE_STATE_VARIABLES
#undef DECLARE_CHANNEL_VARIABLES
#undef COPY_PIXEL
