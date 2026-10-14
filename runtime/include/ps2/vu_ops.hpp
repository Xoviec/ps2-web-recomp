#pragma once

#include "cpu.hpp"
#include "fpu.hpp"
#include "vu.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <vector>

#define PS2_VU_INLINE inline __attribute__((always_inline))

namespace ps2::vu {

void kick(int unit, std::uint32_t address);
std::uint32_t top();
std::uint32_t itop();
void interpret(int unit, std::uint32_t pc);

using ProgramFn = void (*)(Vu0Regs &r, std::uint8_t *mem);

struct Program {
    std::uint64_t key;
    ProgramFn function;
};

void register_programs(const Program *programs, std::uint32_t count);

inline std::uint32_t lower_major(std::uint32_t lo) { return lo >> 25; }
inline bool is_branch(std::uint32_t lo) {
    const std::uint32_t op = lower_major(lo);
    return op == 0x20 || op == 0x21 || op == 0x24 || op == 0x25 || (op >= 0x28 && op <= 0x2F && op != 0x2A && op != 0x2B);
}
inline bool is_jump_register(std::uint32_t lo) { return lower_major(lo) == 0x24 || lower_major(lo) == 0x25; }
inline bool is_unconditional(std::uint32_t lo) { return lower_major(lo) == 0x20 || lower_major(lo) == 0x21; }
inline bool links(std::uint32_t lo) { return lower_major(lo) == 0x21 || lower_major(lo) == 0x25; }
inline std::uint32_t branch_target(std::uint32_t lo, std::uint32_t pc, std::uint32_t mask) {
    const std::int32_t imm11 = static_cast<std::int32_t>(lo << 21) >> 21;
    return (pc + 8u + static_cast<std::uint32_t>(imm11) * 8u) & mask;
}

inline std::vector<std::uint32_t> reachable(const std::uint8_t *code, std::uint32_t mask, std::uint32_t entry) {
    std::vector<std::uint8_t> seen((mask + 1) / 8, 0);
    std::vector<std::uint32_t> work{entry & mask}, out;
    while (!work.empty()) {
        std::uint32_t pc = work.back();
        work.pop_back();
        int stop_after = -1;
        for (int guard = 0; guard < 4096; ++guard) {
            if (seen[pc >> 3]) break;
            seen[pc >> 3] = 1;
            out.push_back(pc);
            if (stop_after == 0) break;
            if (stop_after > 0) --stop_after;
            std::uint32_t lo, hi;
            std::memcpy(&lo, code + pc, 4);
            std::memcpy(&hi, code + pc + 4, 4);
            if (hi & 0x40000000u) stop_after = 0;
            if (!(hi & 0x80000000u) && is_branch(lo)) {
                if (!is_jump_register(lo)) work.push_back(branch_target(lo, pc, mask));
                if (links(lo)) work.push_back((pc + 16u) & mask);
                if (is_unconditional(lo) || is_jump_register(lo)) stop_after = 0;
            }
            pc = (pc + 8u) & mask;
        }
    }
    std::vector<std::uint32_t> sorted(out);
    std::sort(sorted.begin(), sorted.end());
    return sorted;
}

inline std::uint64_t program_key(const std::uint8_t *code, const std::vector<std::uint32_t> &pcs, std::uint32_t entry) {
    std::uint64_t h = 0xCBF29CE484222325ull ^ entry;
    const auto mix = [&](std::uint32_t v) {
        for (int i = 0; i < 4; ++i) {
            h ^= (v >> (i * 8)) & 0xFFu;
            h *= 0x100000001B3ull;
        }
    };
    for (std::uint32_t pc : pcs) {
        std::uint32_t lo, hi;
        std::memcpy(&lo, code + pc, 4);
        std::memcpy(&hi, code + pc + 4, 4);
        mix(pc);
        mix(lo);
        mix(hi);
    }
    return h;
}

}

namespace ps2::vuops {

PS2_VU_INLINE float fin(std::uint32_t x) {
    const std::uint32_t e = x & 0x7F800000u;
    if (!e) x &= 0x80000000u;
    else if (e == 0x7F800000u) x = (x & 0x80000000u) | 0x7F7FFFFFu;
    float f;
    std::memcpy(&f, &x, 4);
    return f;
}

PS2_VU_INLINE std::uint32_t fout(float f) {
    std::uint32_t x;
    std::memcpy(&x, &f, 4);
    const std::uint32_t e = x & 0x7F800000u;
    if (!e) return x & 0x80000000u;
    if (e == 0x7F800000u) return (x & 0x80000000u) | 0x7F7FFFFFu;
    return x;
}

PS2_VU_INLINE float norm(float f) { return fin(fout(f)); }
// The VU adder rounds toward zero. Microcode relies on it: x + 2^23 - 2^23 is floor(x), not
// round(x) (Burnout 3's particle sine range reduction). Round to nearest, then step one ulp
// back toward zero when the exact error (TwoSum) shows the sum was rounded away from zero.
PS2_VU_INLINE float chop_add(float a, float b) {
    const float s = a + b, bv = s - a, e = (a - (s - bv)) + (b - bv);
    std::uint32_t si, ei;
    std::memcpy(&si, &s, 4);
    std::memcpy(&ei, &e, 4);
    if (e != 0.0f && ((si ^ ei) & 0x80000000u)) --si;
    float r;
    std::memcpy(&r, &si, 4);
    return r;
}
PS2_VU_INLINE std::uint32_t add(std::uint32_t a, std::uint32_t b) { return fout(chop_add(fin(a), fin(b))); }
PS2_VU_INLINE std::uint32_t sub(std::uint32_t a, std::uint32_t b) { return fout(chop_add(fin(a), -fin(b))); }
PS2_VU_INLINE std::uint32_t mul(std::uint32_t a, std::uint32_t b) { return fout(fin(a) * fin(b)); }
PS2_VU_INLINE std::uint32_t madd(std::uint32_t acc, std::uint32_t a, std::uint32_t b) { return fout(chop_add(fin(acc), norm(fin(a) * fin(b)))); }
PS2_VU_INLINE std::uint32_t msub(std::uint32_t acc, std::uint32_t a, std::uint32_t b) { return fout(chop_add(fin(acc), -norm(fin(a) * fin(b)))); }
PS2_VU_INLINE std::uint32_t max(std::uint32_t a, std::uint32_t b) { return fp::max(a, b); }
PS2_VU_INLINE std::uint32_t min(std::uint32_t a, std::uint32_t b) { return fp::min(a, b); }

typedef std::uint32_t v4u __attribute__((vector_size(16)));
typedef std::int32_t v4i __attribute__((vector_size(16)));
typedef float v4f __attribute__((vector_size(16)));

PS2_VU_INLINE v4u vload(const Reg128 &r) {
    v4u v;
    std::memcpy(&v, &r, 16);
    return v;
}
PS2_VU_INLINE void vstore(Reg128 &r, v4u v) { std::memcpy(&r, &v, 16); }
PS2_VU_INLINE v4u vsplat(std::uint32_t x) { return v4u{x, x, x, x}; }
PS2_VU_INLINE v4u vsel(v4u old, v4u now, v4u mask) { return (now & mask) | (old & ~mask); }
PS2_VU_INLINE v4u vfix(v4u x) {
    const v4i clamped = __builtin_elementwise_min((v4i)(x & 0x7FFFFFFFu), (v4i){0x7F7FFFFF, 0x7F7FFFFF, 0x7F7FFFFF, 0x7F7FFFFF});
    const v4i normal = (v4f)clamped >= (v4f)(v4i){0x00800000, 0x00800000, 0x00800000, 0x00800000};
    return (x & 0x80000000u) | (v4u)(clamped & normal);
}
PS2_VU_INLINE v4f vin(v4u x) { return (v4f)vfix(x); }
PS2_VU_INLINE v4u vout(v4f f) { return vfix((v4u)f); }
PS2_VU_INLINE v4f vnorm(v4f f) { return (v4f)vfix((v4u)f); }
// Lane-wise chop_add: a sum rounded away from zero steps one ulp back.
PS2_VU_INLINE v4f vchop_add(v4f a, v4f b) {
    const v4f s = a + b, bv = s - a, e = (a - (s - bv)) + (b - bv);
    const v4i away = (e != 0.0f) & (((v4i)e ^ (v4i)s) < 0);
    return (v4f)((v4i)s + away);
}
// The n variants take operands that already went through vfix.
PS2_VU_INLINE v4u vaddn(v4u a, v4u b) { return vout(vchop_add((v4f)a, (v4f)b)); }
PS2_VU_INLINE v4u vsubn(v4u a, v4u b) { return vout(vchop_add((v4f)a, -(v4f)b)); }
PS2_VU_INLINE v4u vmuln(v4u a, v4u b) { return vout((v4f)a * (v4f)b); }
PS2_VU_INLINE v4u vmaddn(v4u acc, v4u a, v4u b) { return vout(vchop_add((v4f)acc, vnorm((v4f)a * (v4f)b))); }
PS2_VU_INLINE v4u vmsubn(v4u acc, v4u a, v4u b) { return vout(vchop_add((v4f)acc, -vnorm((v4f)a * (v4f)b))); }
PS2_VU_INLINE v4u vadd(v4u a, v4u b) { return vaddn(vfix(a), vfix(b)); }
PS2_VU_INLINE v4u vsub(v4u a, v4u b) { return vsubn(vfix(a), vfix(b)); }
PS2_VU_INLINE v4u vmul(v4u a, v4u b) { return vmuln(vfix(a), vfix(b)); }
PS2_VU_INLINE v4u vmadd(v4u acc, v4u a, v4u b) { return vmaddn(vfix(acc), vfix(a), vfix(b)); }
PS2_VU_INLINE v4u vmsub(v4u acc, v4u a, v4u b) { return vmsubn(vfix(acc), vfix(a), vfix(b)); }
// Raw sign-magnitude order, no flush: MAXi/MINIi load integer constants through I.
PS2_VU_INLINE v4i vkey(v4u x) {
    const v4i s = (v4i)x;
    return s ^ ((s >> 31) & 0x7FFFFFFF);
}
PS2_VU_INLINE v4u vmax(v4u a, v4u b) {
    const v4u ge = (v4u)(vkey(a) >= vkey(b));
    return (a & ge) | (b & ~ge);
}
PS2_VU_INLINE v4u vmin(v4u a, v4u b) {
    const v4u le = (v4u)(vkey(a) <= vkey(b));
    return (a & le) | (b & ~le);
}
PS2_VU_INLINE v4u vabs(v4u a) { return a & 0x7FFFFFFFu; }
PS2_VU_INLINE v4u vftoi(v4u x, int fraction) {
    const v4f f = (v4f)vfix(x) * static_cast<float>(1u << fraction);
    const v4i big = f >= 2147483648.0f;
    const v4i small = f < -2147483648.0f;
    const v4i out = big | small;
    const v4i t = __builtin_convertvector((v4f)((v4i)f & ~out), v4i);
    return (v4u)((t & ~out) | (big & 0x7FFFFFFF) | (small & (v4i)vsplat(0x80000000u)));
}
PS2_VU_INLINE v4u vitof(v4u x, int fraction) {
    const v4i v = (v4i)x;
    v4f f = __builtin_convertvector(v, v4f);
    if (__builtin_reduce_or((v4i)(x + 0x01000000u > 0x02000000u))) {
        const v4i big = f >= 2147483648.0f;
        const v4i back = __builtin_convertvector((v4f)((v4i)f & ~big), v4i);
        const v4i over = big | ((v >= 0) & (back > v)) | ((v < 0) & (back < v));
        f = (v4f)((v4i)f + over);
    }
    return (v4u)(f * (1.0f / static_cast<float>(1u << fraction)));
}

PS2_VU_INLINE void flags(Vu0Regs &r, const Reg128 &v, unsigned dest) {
    std::uint32_t mac = 0;
    for (int i = 0; i < 4; ++i) {
        if (!(dest & (8u >> i))) continue;
        const std::uint32_t bit = 3u - static_cast<std::uint32_t>(i);
        const std::uint32_t x = v.w[i];
        if (!(x & 0x7F800000u)) mac |= 1u << bit;
        if (x & 0x80000000u) mac |= 1u << (bit + 4);
        if ((x & 0x7FFFFFFFu) == 0x7FFFFFFFu) mac |= 1u << (bit + 12);
    }
    r.mac = mac;
    std::uint32_t status = r.status & 0xFF0u;
    if (mac & 0x000Fu) status |= 0x1u;
    if (mac & 0x00F0u) status |= 0x2u;
    if (mac & 0x0F00u) status |= 0x4u;
    if (mac & 0xF000u) status |= 0x8u;
    status |= (status & 0xFu) << 6;
    r.status = (status & ~0x30u) | (r.status & 0x30u);
}

PS2_VU_INLINE std::uint32_t clip_value(const Vu0Regs &r, const Reg128 &s, std::uint32_t w) {
    const std::int32_t lim = fp::key(w & fp::kAbs);
    std::uint32_t f = 0;
    for (int i = 0; i < 3; ++i) {
        const std::int32_t x = fp::key(s.w[i]);
        if (x > lim) f |= 1u << (i * 2);
        if (x < -lim) f |= 2u << (i * 2);
    }
    return ((r.clip << 6) | f) & 0xFFFFFFu;
}

PS2_VU_INLINE void clip(Vu0Regs &r, const Reg128 &s, std::uint32_t w) {
    const std::int32_t lim = fp::key(w & fp::kAbs);
    std::uint32_t f = 0;
    for (int i = 0; i < 3; ++i) {
        const std::int32_t x = fp::key(s.w[i]);
        if (x > lim) f |= 1u << (i * 2);
        if (x < -lim) f |= 2u << (i * 2);
    }
    r.clip = ((r.clip << 6) | f) & 0xFFFFFFu;
}

struct FlagPipe {
    std::uint32_t t = 16;
    std::uint32_t mac_ts[8] = {}, mac_v[8] = {}, status_v[8] = {}, clip_ts[8] = {}, clip_v[8] = {};
    unsigned mh = 0, ch = 0;
    // FDIV (Q) and EFU (P) results land `latency` cycles after issue; software-pipelined
    // microcode reads the previous value until then. qt/pt: ready time, 0 when idle.
    std::uint32_t qt = 0, pt = 0, qv = 0, pv = 0;

    void start(const Vu0Regs &r) {
        for (int i = 0; i < 8; ++i) {
            mac_ts[i] = clip_ts[i] = 0;
            mac_v[i] = r.mac;
            status_v[i] = r.status;
            clip_v[i] = r.clip;
        }
        qt = r.qwait ? t + r.qwait : 0;
        pt = r.pwait ? t + r.pwait : 0;
        qv = r.qnext;
        pv = r.pnext;
    }
    PS2_VU_INLINE void sync_q(Vu0Regs &r) {
        if (qt && t >= qt) {
            r.q = qv;
            qt = 0;
        }
    }
    PS2_VU_INLINE void sync_p(Vu0Regs &r) {
        if (pt && t >= pt) {
            r.p = pv;
            pt = 0;
        }
    }
    PS2_VU_INLINE void wait_q(Vu0Regs &r) {
        if (qt) r.q = qv;
        qt = 0;
    }
    PS2_VU_INLINE void wait_p(Vu0Regs &r) {
        if (pt) r.p = pv;
        pt = 0;
    }
    // A second issue stalls until the first lands; starting from its ready time stands in for the stall.
    PS2_VU_INLINE void issue_q(Vu0Regs &r, std::uint32_t v, std::uint32_t latency) {
        if (qt > t) t = qt;  // the pair stalls until the unit frees: time moves on
        wait_q(r);
        qv = v;
        qt = t + latency;
    }
    PS2_VU_INLINE void issue_p(Vu0Regs &r, std::uint32_t v, std::uint32_t latency) {
        if (pt > t) t = pt;  // the pair stalls until the unit frees: time moves on
        wait_p(r);
        pv = v;
        pt = t + latency;
    }
    // The E-bit flushes both units, so the next program starts with Q and P settled.
    PS2_VU_INLINE void finish(Vu0Regs &r) {
        wait_q(r);
        wait_p(r);
        r.qwait = r.pwait = 0;
    }
    // Leaving for another entry mid-program (jr, unsupported op) is not a program boundary:
    // hand what is still in flight to the next run's start().
    void park(Vu0Regs &r) {
        sync_q(r);
        sync_p(r);
        r.qwait = static_cast<std::uint16_t>(qt ? qt - t : 0);
        r.pwait = static_cast<std::uint16_t>(pt ? pt - t : 0);
        r.qnext = qv;
        r.pnext = pv;
    }
    PS2_VU_INLINE void set_mac(std::uint32_t mac, std::uint32_t status) {
        const unsigned i = mh++ & 7u;
        mac_ts[i] = t;
        mac_v[i] = mac;
        status_v[i] = status;
    }
    PS2_VU_INLINE void set_clip(std::uint32_t clip, std::uint32_t at) {
        const unsigned i = ch++ & 7u;
        clip_ts[i] = at;
        clip_v[i] = clip;
    }
    PS2_VU_INLINE void force_mac(std::uint32_t mac, std::uint32_t status) {
        set_mac(mac, status);
        mac_ts[(mh - 1) & 7u] = t - 4u;
    }
    PS2_VU_INLINE unsigned mac_slot() const {
        for (unsigned k = 1; k <= 8; ++k) {
            const unsigned i = (mh - k) & 7u;
            if (mac_ts[i] + 4u <= t) return i;
        }
        return (mh - 8) & 7u;
    }
    PS2_VU_INLINE std::uint32_t mac() const { return mac_v[mac_slot()]; }
    PS2_VU_INLINE std::uint32_t status() const { return status_v[mac_slot()]; }
    PS2_VU_INLINE std::uint32_t clip() const {
        for (unsigned k = 1; k <= 8; ++k) {
            const unsigned i = (ch - k) & 7u;
            if (clip_ts[i] + 3u <= t) return clip_v[i];
        }
        return clip_v[(ch - 8) & 7u];
    }
};

PS2_VU_INLINE std::uint8_t *qword(std::uint8_t *mem, std::uint32_t index) { return mem + (index & 1023u) * 16u; }
PS2_VU_INLINE void set_vi(Vu0Regs &r, unsigned i, std::uint32_t v) {
    if (i) r.vi[i] = static_cast<std::uint16_t>(v);
}
PS2_VU_INLINE void set_vi(std::uint16_t &dst, std::uint32_t v) { dst = static_cast<std::uint16_t>(v); }
PS2_VU_INLINE void drop_vi(std::uint32_t) {}

PS2_VU_INLINE std::uint32_t f2u(float f) {
    std::uint32_t v;
    std::memcpy(&v, &f, 4);
    return v;
}

PS2_VU_INLINE float u2f(std::uint32_t v) {
    if (!(v & 0x7F800000u)) v &= 0x80000000u;
    if ((v & 0x7F800000u) == 0x7F800000u) v = (v & 0x80000000u) | 0x7F7FFFFFu;
    float f;
    std::memcpy(&f, &v, 4);
    return f;
}

inline std::uint32_t efu_value(const Vu0Regs &r, unsigned index, unsigned fs, unsigned fsf) {
    switch (index) {
    case 0x70: case 0x71: case 0x72: case 0x73: {
        const float x = u2f(r.vf[fs].w[0]), y = u2f(r.vf[fs].w[1]), z = u2f(r.vf[fs].w[2]);
        const float sum = x * x + y * y + z * z;
        float p = sum;
        if (index == 0x71) p = sum != 0.0f ? 1.0f / sum : 3.4e38f;
        if (index == 0x72) p = std::sqrt(sum);
        if (index == 0x73) p = sum != 0.0f ? 1.0f / std::sqrt(sum) : 3.4e38f;
        return f2u(p);
    }
    case 0x74: return f2u(std::atan2(u2f(r.vf[fs].w[1]), u2f(r.vf[fs].w[0])));
    case 0x75: return f2u(std::atan2(u2f(r.vf[fs].w[2]), u2f(r.vf[fs].w[0])));
    case 0x76: return f2u(u2f(r.vf[fs].w[0]) + u2f(r.vf[fs].w[1]) + u2f(r.vf[fs].w[2]) + u2f(r.vf[fs].w[3]));
    case 0x78: return f2u(std::sqrt(std::fabs(u2f(r.vf[fs].w[fsf]))));
    case 0x79: { const float v = std::sqrt(std::fabs(u2f(r.vf[fs].w[fsf]))); return f2u(v != 0.0f ? 1.0f / v : 3.4e38f); }
    case 0x7A: { const float v = u2f(r.vf[fs].w[fsf]); return f2u(v != 0.0f ? 1.0f / v : 3.4e38f); }
    case 0x7C: return f2u(std::sin(u2f(r.vf[fs].w[fsf])));
    case 0x7D: return f2u(std::atan(u2f(r.vf[fs].w[fsf])));
    case 0x7E: return f2u(std::exp(-u2f(r.vf[fs].w[fsf])));
    default: return r.p;
    }
}

// EFU latencies by index & 15: ESADD ERSADD ELENG ERLENG EATANxy EATANxz ESUM - ESQRT ERSQRT ERCPR - ESIN EATAN EEXP.
inline constexpr std::uint8_t kEfuLatency[16] = {11, 18, 18, 24, 54, 54, 12, 0, 12, 18, 12, 0, 29, 54, 44, 0};

PS2_VU_INLINE void efu(Vu0Regs &r, FlagPipe &fl, unsigned index, unsigned fs, unsigned fsf) {
    fl.issue_p(r, efu_value(r, index, fs, fsf), kEfuLatency[index & 15u]);
}

PS2_VU_INLINE void qdiv(Vu0Regs &r, FlagPipe &fl, std::uint32_t s, std::uint32_t t) {
    const fp::Result x = fp::div(s, t);
    r.status = (r.status & ~0x30u) | (x.invalid ? 0x20u : 0u);
    fl.issue_q(r, x.value, 7);
}
PS2_VU_INLINE void qsqrt(Vu0Regs &r, FlagPipe &fl, std::uint32_t t) { fl.issue_q(r, fp::sqrt(t), 7); }
PS2_VU_INLINE void qrsqrt(Vu0Regs &r, FlagPipe &fl, std::uint32_t s, std::uint32_t t) {
    const fp::Result x = fp::rsqrt(s, t);
    r.status = (r.status & ~0x30u) | (x.invalid ? 0x20u : 0u);
    fl.issue_q(r, x.value, 13);
}

}
