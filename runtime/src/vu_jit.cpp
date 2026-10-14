#include "vu_jit.hpp"

#include "ps2/vu_ops.hpp"

#include <cstddef>
#include <cstring>
#include <map>
#include <set>

namespace ps2::vu::jit {

const char *const kImportNames[kImports] = {
    "clip_value", "clip_post", "set_clip", "start", "mac", "status", "clip", "force_mac", "mul", "msub", "to_fixed", "from_fixed",
    "div", "sqrt", "rsqrt", "efu", "kick", "top", "itop", "rand_next", "interpret", "pipe",
};

namespace {

constexpr std::uint32_t kMask = 16383u;
constexpr int kAcc = 32;

struct ImportType {
    std::uint8_t params;
    bool result;
};

constexpr ImportType kImportTypes[kImports] = {
    {3, true}, {3, false}, {3, false}, {2, false}, {1, true}, {1, true}, {1, true}, {2, false}, {2, true}, {3, true}, {2, true}, {2, true},
    {3, false}, {2, false}, {3, false}, {4, false}, {1, false}, {0, true}, {0, true}, {1, true}, {1, false}, {2, false},
};

enum Local : std::uint32_t {
    kR, kMem, kPend, kPendj, kEnding, kJpc, kPcw, kUclip, kUx, kUy, kUz, kUw, kTmp, kTmp2, kMacv, kStat, kSlot, kI32Locals,
    kUv = kI32Locals, kV0, kV1, kV2, kV3, kV4, kShadow0
};
constexpr std::uint32_t kI32Count = kI32Locals - 2;
constexpr std::uint32_t kVecTemps = kShadow0 - kUv;

constexpr std::uint32_t kOne = 0x3F800000u;

std::uint32_t vf_off(unsigned reg) { return static_cast<std::uint32_t>(offsetof(Vu0Regs, vf) + reg * 16u); }
std::uint32_t reg_off(int reg) { return reg == kAcc ? static_cast<std::uint32_t>(offsetof(Vu0Regs, acc)) : vf_off(static_cast<unsigned>(reg)); }
std::uint32_t vi_off(unsigned i) { return static_cast<std::uint32_t>(offsetof(Vu0Regs, vi) + i * 2u); }
constexpr std::uint32_t kOffQ = offsetof(Vu0Regs, q), kOffP = offsetof(Vu0Regs, p), kOffI = offsetof(Vu0Regs, i), kOffR = offsetof(Vu0Regs, r);
constexpr std::uint32_t kOffMac = offsetof(Vu0Regs, mac);
constexpr std::uint32_t kOffMh = offsetof(vuops::FlagPipe, mh), kOffMacTs = offsetof(vuops::FlagPipe, mac_ts), kOffMacV = offsetof(vuops::FlagPipe, mac_v),
                        kOffStatusV = offsetof(vuops::FlagPipe, status_v), kOffT = offsetof(vuops::FlagPipe, t);
constexpr std::uint32_t kOffStatus = offsetof(Vu0Regs, status), kOffClip = offsetof(Vu0Regs, clip), kOffTpc = offsetof(Vu0Regs, tpc);

bool lane(unsigned dest, int i) { return dest & (8u >> i); }

struct Out {
    std::vector<std::uint8_t> b;
    void byte(std::uint8_t v) { b.push_back(v); }
    void u32(std::uint32_t v) {
        do {
            std::uint8_t c = v & 0x7Fu;
            v >>= 7;
            if (v) c |= 0x80u;
            b.push_back(c);
        } while (v);
    }
    void s32(std::int32_t v) {
        for (;;) {
            const std::uint8_t c = static_cast<std::uint8_t>(v & 0x7F);
            v >>= 7;
            if ((v == 0 && !(c & 0x40)) || (v == -1 && (c & 0x40))) {
                b.push_back(c);
                return;
            }
            b.push_back(c | 0x80u);
        }
    }
    void name(const char *s) {
        u32(static_cast<std::uint32_t>(std::strlen(s)));
        b.insert(b.end(), s, s + std::strlen(s));
    }
    void section(std::uint8_t id, const Out &body) {
        byte(id);
        u32(static_cast<std::uint32_t>(body.b.size()));
        b.insert(b.end(), body.b.begin(), body.b.end());
    }
};

struct Code : Out {
    std::uint32_t depth = 0;
    void simd(std::uint32_t o) {
        byte(0xFD);
        u32(o);
    }
    void i32c(std::uint32_t v) {
        byte(0x41);
        s32(static_cast<std::int32_t>(v));
    }
    void get(std::uint32_t l) {
        byte(0x20);
        u32(l);
    }
    void set(std::uint32_t l) {
        byte(0x21);
        u32(l);
    }
    void tee(std::uint32_t l) {
        byte(0x22);
        u32(l);
    }
    void load(std::uint32_t off) {
        byte(0x28);
        u32(2);
        u32(off);
    }
    void load16(std::uint32_t off) {
        byte(0x2F);
        u32(1);
        u32(off);
    }
    void store(std::uint32_t off) {
        byte(0x36);
        u32(2);
        u32(off);
    }
    void store16(std::uint32_t off) {
        byte(0x3B);
        u32(1);
        u32(off);
    }
    void vload(std::uint32_t off) {
        simd(0x00);
        u32(4);
        u32(off);
    }
    void vstore(std::uint32_t off) {
        simd(0x0B);
        u32(4);
        u32(off);
    }
    void vconst(std::uint32_t a, std::uint32_t b2, std::uint32_t c, std::uint32_t d) {
        simd(0x0C);
        for (std::uint32_t w : {a, b2, c, d})
            for (int k = 0; k < 4; ++k) byte(static_cast<std::uint8_t>(w >> (k * 8)));
    }
    void vsplatc(std::uint32_t v) { vconst(v, v, v, v); }
    void call(std::uint32_t f) {
        byte(0x10);
        u32(f);
    }
    void tail(std::uint32_t f) {
        byte(0x12);
        u32(f);
    }
    void op(std::uint8_t o) { byte(o); }
    void open_if() {
        byte(0x04);
        byte(0x40);
        ++depth;
    }
    void end() {
        byte(0x0B);
        --depth;
    }
};

enum : std::uint8_t {
    kOpDrop = 0x1A, kOpEqz = 0x45, kOpEq = 0x46, kOpNe = 0x47, kOpLtS = 0x48, kOpGtS = 0x4A, kOpLeS = 0x4C, kOpGeS = 0x4E,
    kOpAdd = 0x6A, kOpSub = 0x6B, kOpMul = 0x6C, kOpAnd = 0x71, kOpOr = 0x72, kOpXor = 0x73, kOpShl = 0x74, kOpShrU = 0x76, kOpExt16 = 0xC1,
    kOpReturn = 0x0F, kOpBr = 0x0C, kOpUnreachable = 0x00,
};

enum : std::uint32_t {
    kVSplat = 0x11, kVExtract = 0x1B, kI32x4Eq = 0x37, kI32x4LtS = 0x39, kI32x4LeS = 0x3D, kI32x4GeS = 0x3F, kF32x4Ge = 0x46,
    kVAnd = 0x4E, kVOr = 0x50, kVXor = 0x51, kVBitselect = 0x52, kI32x4MinS = 0xB6, kF32x4Add = 0xE4, kF32x4Sub = 0xE5, kF32x4Mul = 0xE6,
    kF32x4Ne = 0x42, kF32x4Neg = 0xE1, kI32x4Add = 0xAE,
};

enum Kind { Nop, Add, Sub, Madd, Msub, Max, Min, Mul, Abs, Ftoi, Itof, Clip, Opmula, Opmsub };
enum Src { Vec, Bc, Q, I };

struct Up {
    Kind kind = Nop;
    Src src = Vec;
    int target = -1;
    bool flags = false;
    int frac = 0;
    unsigned dest = 0, ft = 0, fs = 0, bc = 0;
};

Up decode_upper(std::uint32_t hi) {
    Up u;
    u.dest = (hi >> 21) & 15u;
    u.ft = (hi >> 16) & 31u;
    u.fs = (hi >> 11) & 31u;
    u.bc = hi & 3u;
    const unsigned fd = (hi >> 6) & 31u;
    const unsigned funct = hi & 63u;
    static const int kFrac[4] = {0, 4, 12, 15};
    const auto set = [&](Kind k, Src s, int t, bool f) {
        u.kind = k;
        u.src = s;
        u.target = t;
        u.flags = f;
    };
    if (funct < 0x3C) {
        switch (funct) {
        case 0x00: case 0x01: case 0x02: case 0x03: set(Add, Bc, static_cast<int>(fd), true); break;
        case 0x04: case 0x05: case 0x06: case 0x07: set(Sub, Bc, static_cast<int>(fd), true); break;
        case 0x08: case 0x09: case 0x0A: case 0x0B: set(Madd, Bc, static_cast<int>(fd), true); break;
        case 0x0C: case 0x0D: case 0x0E: case 0x0F: set(Msub, Bc, static_cast<int>(fd), true); break;
        case 0x10: case 0x11: case 0x12: case 0x13: set(Max, Bc, static_cast<int>(fd), false); break;
        case 0x14: case 0x15: case 0x16: case 0x17: set(Min, Bc, static_cast<int>(fd), false); break;
        case 0x18: case 0x19: case 0x1A: case 0x1B: set(Mul, Bc, static_cast<int>(fd), true); break;
        case 0x1C: set(Mul, Q, static_cast<int>(fd), true); break;
        case 0x1D: set(Max, I, static_cast<int>(fd), false); break;
        case 0x1E: set(Mul, I, static_cast<int>(fd), true); break;
        case 0x1F: set(Min, I, static_cast<int>(fd), false); break;
        case 0x20: set(Add, Q, static_cast<int>(fd), true); break;
        case 0x21: set(Madd, Q, static_cast<int>(fd), true); break;
        case 0x22: set(Add, I, static_cast<int>(fd), true); break;
        case 0x23: set(Madd, I, static_cast<int>(fd), true); break;
        case 0x24: set(Sub, Q, static_cast<int>(fd), true); break;
        case 0x25: set(Msub, Q, static_cast<int>(fd), true); break;
        case 0x26: set(Sub, I, static_cast<int>(fd), true); break;
        case 0x27: set(Msub, I, static_cast<int>(fd), true); break;
        case 0x28: set(Add, Vec, static_cast<int>(fd), true); break;
        case 0x29: set(Madd, Vec, static_cast<int>(fd), true); break;
        case 0x2A: set(Mul, Vec, static_cast<int>(fd), true); break;
        case 0x2B: set(Max, Vec, static_cast<int>(fd), false); break;
        case 0x2C: set(Sub, Vec, static_cast<int>(fd), true); break;
        case 0x2D: set(Msub, Vec, static_cast<int>(fd), true); break;
        case 0x2E: set(Opmsub, Vec, static_cast<int>(fd), true); break;
        case 0x2F: set(Min, Vec, static_cast<int>(fd), false); break;
        default: break;
        }
    } else {
        const unsigned index = ((hi >> 6) & 31u) * 4u + (funct & 3u);
        switch (index) {
        case 0x00: case 0x01: case 0x02: case 0x03: set(Add, Bc, kAcc, true); break;
        case 0x04: case 0x05: case 0x06: case 0x07: set(Sub, Bc, kAcc, true); break;
        case 0x08: case 0x09: case 0x0A: case 0x0B: set(Madd, Bc, kAcc, true); break;
        case 0x0C: case 0x0D: case 0x0E: case 0x0F: set(Msub, Bc, kAcc, true); break;
        case 0x10: case 0x11: case 0x12: case 0x13: set(Itof, Vec, static_cast<int>(u.ft), false); u.frac = kFrac[index & 3u]; break;
        case 0x14: case 0x15: case 0x16: case 0x17: set(Ftoi, Vec, static_cast<int>(u.ft), false); u.frac = kFrac[index & 3u]; break;
        case 0x18: case 0x19: case 0x1A: case 0x1B: set(Mul, Bc, kAcc, true); break;
        case 0x1C: set(Mul, Q, kAcc, true); break;
        case 0x1D: set(Abs, Vec, static_cast<int>(u.ft), false); break;
        case 0x1E: set(Mul, I, kAcc, true); break;
        case 0x1F: set(Clip, Vec, -1, false); break;
        case 0x20: set(Add, Q, kAcc, true); break;
        case 0x21: set(Madd, Q, kAcc, true); break;
        case 0x22: set(Add, I, kAcc, true); break;
        case 0x23: set(Madd, I, kAcc, true); break;
        case 0x24: set(Sub, Q, kAcc, true); break;
        case 0x25: set(Msub, Q, kAcc, true); break;
        case 0x26: set(Sub, I, kAcc, true); break;
        case 0x27: set(Msub, I, kAcc, true); break;
        case 0x28: set(Add, Vec, kAcc, true); break;
        case 0x29: set(Madd, Vec, kAcc, true); break;
        case 0x2A: set(Mul, Vec, kAcc, true); break;
        case 0x2C: set(Sub, Vec, kAcc, true); break;
        case 0x2D: set(Msub, Vec, kAcc, true); break;
        case 0x2E: set(Opmula, Vec, kAcc, true); break;
        default: break;
        }
    }
    if (u.kind == Opmula || u.kind == Opmsub) u.dest = 0xE;
    return u;
}

bool arithmetic(Kind k) { return k == Add || k == Sub || k == Mul || k == Madd || k == Msub; }
bool vector(Kind k) { return arithmetic(k) || k == Max || k == Min || k == Abs; }

bool active(const Up &u, bool flags_live) {
    if (u.kind == Nop || u.kind == Clip) return false;
    return !(u.dest == 0 || (u.target == 0 && !(u.flags && flags_live)));
}

bool sets_flags(std::uint32_t hi) {
    const Up u = decode_upper(hi);
    return active(u, true) && u.flags;
}

std::vector<int> shadow_reads(std::uint32_t hi) {
    const Up u = decode_upper(hi);
    std::vector<int> regs;
    if (!active(u, true) || !arithmetic(u.kind)) return regs;
    if ((u.src == Vec || u.src == Bc) && u.ft) regs.push_back(static_cast<int>(u.ft));
    if (u.fs) regs.push_back(static_cast<int>(u.fs));
    if (u.kind == Madd || u.kind == Msub) regs.push_back(kAcc);
    return regs;
}

struct LowerInfo {
    int writes = -1;
    bool reads_flags = false, reads_status = false;
    bool branch = false, jump = false;
    std::uint32_t target = 0;
    bool supported = true;
};

LowerInfo analyze_lower(std::uint32_t w, std::uint32_t pc) {
    LowerInfo l;
    const unsigned dest = (w >> 21) & 15u, ft = (w >> 16) & 31u;
    const std::int32_t imm11 = static_cast<std::int32_t>(w << 21) >> 21;
    if ((w >> 25) == 0x40u) {
        const unsigned funct = w & 63u;
        if (funct < 0x3C) {
            l.supported = funct == 0x30 || funct == 0x31 || funct == 0x32 || funct == 0x34 || funct == 0x35;
            return l;
        }
        const unsigned index = ((w >> 6) & 31u) * 4u + (funct & 3u);
        switch (index) {
        case 0x30: case 0x31: case 0x34: case 0x36: case 0x3D: case 0x40: case 0x41: case 0x64:
            if (ft) l.writes = static_cast<int>(ft);
            break;
        case 0x35: case 0x37: case 0x38: case 0x39: case 0x3A: case 0x3B: case 0x3C: case 0x3E: case 0x3F: case 0x42: case 0x43:
        case 0x68: case 0x69: case 0x6C: case 0x70: case 0x71: case 0x72: case 0x73: case 0x74: case 0x75: case 0x76: case 0x78:
        case 0x79: case 0x7A: case 0x7B: case 0x7C: case 0x7D: case 0x7E:
            break;
        default: l.supported = false; break;
        }
        (void)dest;
        return l;
    }
    const std::uint32_t target = (pc + 8u + static_cast<std::uint32_t>(imm11) * 8u) & kMask;
    switch (w >> 25) {
    case 0x00: if (ft) l.writes = static_cast<int>(ft); break;
    case 0x01: case 0x04: case 0x05: case 0x08: case 0x09: case 0x10: case 0x11: case 0x12: case 0x13: case 0x1C: break;
    case 0x14: case 0x15: case 0x16: case 0x17: l.reads_flags = l.reads_status = true; break;
    case 0x18: case 0x1A: case 0x1B: l.reads_flags = true; break;
    case 0x20: case 0x21: case 0x28: case 0x29: case 0x2C: case 0x2D: case 0x2E: case 0x2F: l.branch = true; l.target = target; break;
    case 0x24: case 0x25: l.jump = true; break;
    default: l.supported = false; break;
    }
    return l;
}

class Translator {
public:
    Translator(const std::uint8_t *code, std::uint32_t entry, const Layout &layout) : code_(code), entry_(entry & kMask), layout_(layout) {}

    bool run(std::vector<std::uint8_t> &out);

private:
    const std::uint8_t *code_;
    std::uint32_t entry_;
    Layout layout_;
    Code c_;
    std::vector<std::uint32_t> pcs_;
    std::set<std::uint32_t> in_;
    std::map<std::uint32_t, LowerInfo> lowers_;
    std::map<std::uint32_t, bool> live_in_;
    std::map<int, std::uint32_t> shadow_;
    std::uint32_t case_index_ = 0;

    std::uint32_t word(std::uint32_t at) const {
        std::uint32_t v;
        std::memcpy(&v, code_ + (at & kMask), 4);
        return v;
    }

    std::vector<std::int64_t> successors(std::uint32_t pc) {
        std::vector<std::int64_t> s{static_cast<std::int64_t>((pc + 8u) & kMask)};
        const std::uint32_t prev = (pc - 8u) & kMask;
        if (in_.count(prev)) {
            const LowerInfo &pl = lowers_[prev];
            if (pl.branch) s.push_back(pl.target);
            if (pl.jump || (word(prev + 4) & 0x40000000u)) s.push_back(-1);
        }
        return s;
    }

    bool out_live(std::uint32_t pc) {
        for (std::int64_t s : successors(pc))
            if (s < 0 || !in_.count(static_cast<std::uint32_t>(s)) || live_in_[static_cast<std::uint32_t>(s)]) return true;
        return false;
    }

    void fl() { c_.i32c(layout_.flags); }
    void pipe(PipeOp op) {
        c_.get(kR);
        c_.i32c(op);
        c_.call(kPipe);
    }
    std::uint32_t top_depth() const { return static_cast<std::uint32_t>(pcs_.size()) - case_index_ + c_.depth; }

    void vfix() {
        c_.tee(kV0);
        c_.vsplatc(0x7FFFFFFFu);
        c_.simd(kVAnd);
        c_.vsplatc(0x7F7FFFFFu);
        c_.simd(kI32x4MinS);
        c_.tee(kV1);
        c_.get(kV1);
        c_.vsplatc(0x00800000u);
        c_.simd(kF32x4Ge);
        c_.simd(kVAnd);
        c_.get(kV0);
        c_.vsplatc(0x80000000u);
        c_.simd(kVAnd);
        c_.simd(kVOr);
    }

    // a b -> a + b rounded toward zero, like the VU adder (vuops::chop_add): round to nearest,
    // then step one ulp back where the TwoSum error shows the sum went away from zero.
    void chop_add() {
        c_.set(kV1);
        c_.tee(kV0);
        c_.get(kV1);
        c_.simd(kF32x4Add);
        c_.tee(kV2);
        c_.get(kV0);
        c_.simd(kF32x4Sub);
        c_.set(kV3);
        c_.get(kV0);
        c_.get(kV2);
        c_.get(kV3);
        c_.simd(kF32x4Sub);
        c_.simd(kF32x4Sub);
        c_.get(kV1);
        c_.get(kV3);
        c_.simd(kF32x4Sub);
        c_.simd(kF32x4Add);
        c_.set(kV4);
        c_.get(kV2);
        c_.get(kV4);
        c_.vsplatc(0);
        c_.simd(kF32x4Ne);
        c_.get(kV4);
        c_.get(kV2);
        c_.simd(kVXor);
        c_.vsplatc(0);
        c_.simd(kI32x4LtS);
        c_.simd(kVAnd);
        c_.simd(kI32x4Add);
    }

    void vkey_of(std::uint32_t local) {
        c_.get(local);
        c_.get(local);
        c_.vsplatc(0);
        c_.simd(kI32x4LtS);
        c_.vsplatc(0x7FFFFFFFu);
        c_.simd(kVAnd);
        c_.simd(kVXor);
    }

    void raw_vec(unsigned reg) {
        if (!reg) {
            c_.vconst(0, 0, 0, kOne);
            return;
        }
        c_.get(kR);
        c_.vload(vf_off(reg));
    }

    void raw_lane(unsigned reg, unsigned l) {
        if (!reg) {
            c_.i32c(l == 3 ? kOne : 0u);
            return;
        }
        c_.get(kR);
        c_.load(vf_off(reg) + l * 4u);
    }

    void mem_lane(unsigned reg, unsigned l) {
        c_.get(kR);
        c_.load(vf_off(reg) + l * 4u);
    }

    void shadow_vec(unsigned reg) {
        if (!reg) {
            c_.vconst(0, 0, 0, kOne);
            return;
        }
        c_.get(shadow_.at(static_cast<int>(reg)));
    }

    void vi(unsigned i) {
        c_.get(kR);
        c_.load16(vi_off(i));
    }

    template <typename F>
    void set_vi(unsigned i, F value) {
        if (i) c_.get(kR);
        value();
        if (i) c_.store16(vi_off(i));
        else c_.op(kOpDrop);
    }

    template <typename F>
    void qword(F index) {
        c_.get(kMem);
        index();
        c_.i32c(1023);
        c_.op(kOpAnd);
        c_.i32c(4);
        c_.op(kOpShl);
        c_.op(kOpAdd);
    }

    void lanes_copy(unsigned dest, unsigned to) {
        if (dest == 0xF) {
            c_.get(kR);
            c_.get(kTmp);
            c_.vload(0);
            c_.vstore(vf_off(to));
            return;
        }
        for (unsigned l = 0; l < 4; ++l) {
            if (!lane(dest, static_cast<int>(l))) continue;
            c_.get(kR);
            c_.get(kTmp);
            c_.load(l * 4u);
            c_.store(vf_off(to) + l * 4u);
        }
    }

    void lanes_store(unsigned dest, unsigned from) {
        if (dest == 0xF) {
            c_.get(kTmp);
            c_.get(kR);
            c_.vload(vf_off(from));
            c_.vstore(0);
            return;
        }
        for (unsigned l = 0; l < 4; ++l) {
            if (!lane(dest, static_cast<int>(l))) continue;
            c_.get(kTmp);
            mem_lane(from, l);
            c_.store(l * 4u);
        }
    }

    void lanes_fill(unsigned dest, unsigned to, std::uint32_t value_local) {
        for (unsigned l = 0; l < 4; ++l) {
            if (!lane(dest, static_cast<int>(l))) continue;
            c_.get(kR);
            c_.get(value_local);
            c_.store(vf_off(to) + l * 4u);
        }
    }

    void lane_mask(unsigned d) {
        c_.vconst(lane(d, 0) ? ~0u : 0u, lane(d, 1) ? ~0u : 0u, lane(d, 2) ? ~0u : 0u, lane(d, 3) ? ~0u : 0u);
    }

    void flags_inline(unsigned d) {
        const auto bits = [&](unsigned shift) {
            c_.vconst(lane(d, 0) ? 8u << shift : 0u, lane(d, 1) ? 4u << shift : 0u, lane(d, 2) ? 2u << shift : 0u, lane(d, 3) ? 1u << shift : 0u);
        };
        c_.set(kV0);
        c_.get(kV0);
        c_.vsplatc(0x7F800000u);
        c_.simd(kVAnd);
        c_.vsplatc(0);
        c_.simd(kI32x4Eq);
        bits(0);
        c_.simd(kVAnd);
        c_.get(kV0);
        c_.vsplatc(0);
        c_.simd(kI32x4LtS);
        bits(4);
        c_.simd(kVAnd);
        c_.simd(kVOr);
        c_.get(kV0);
        c_.vsplatc(0x7FFFFFFFu);
        c_.simd(kVAnd);
        c_.vsplatc(0x7FFFFFFFu);
        c_.simd(kI32x4Eq);
        bits(12);
        c_.simd(kVAnd);
        c_.simd(kVOr);
        c_.tee(kV1);
        for (std::uint8_t l = 0; l < 4; ++l) {
            if (l) c_.get(kV1);
            c_.simd(kVExtract);
            c_.byte(l);
            if (l) c_.op(kOpOr);
        }
        c_.set(kMacv);
        c_.get(kR);
        c_.get(kMacv);
        c_.store(kOffMac);
        c_.get(kR);
        c_.load(kOffStatus);
        c_.i32c(0xFF0u);
        c_.op(kOpAnd);
        const std::pair<std::uint32_t, std::uint32_t> groups[3] = {{0xFu, 0}, {0xF0u, 1}, {0xF000u, 3}};
        for (const auto &[mask, shift] : groups) {
            c_.get(kMacv);
            c_.i32c(mask);
            c_.op(kOpAnd);
            c_.i32c(0);
            c_.op(kOpNe);
            if (shift) {
                c_.i32c(shift);
                c_.op(kOpShl);
            }
            c_.op(kOpOr);
        }
        c_.tee(kStat);
        c_.get(kStat);
        c_.i32c(0xFu);
        c_.op(kOpAnd);
        c_.i32c(6);
        c_.op(kOpShl);
        c_.op(kOpOr);
        c_.set(kStat);
        c_.get(kR);
        c_.get(kStat);
        c_.store(kOffStatus);
        fl();
        fl();
        c_.load(kOffMh);
        c_.tee(kSlot);
        c_.i32c(1);
        c_.op(kOpAdd);
        c_.store(kOffMh);
        c_.get(kSlot);
        c_.i32c(7);
        c_.op(kOpAnd);
        c_.i32c(2);
        c_.op(kOpShl);
        c_.tee(kSlot);
        fl();
        c_.load(kOffT);
        c_.store(layout_.flags + kOffMacTs);
        c_.get(kSlot);
        c_.get(kMacv);
        c_.store(layout_.flags + kOffMacV);
        c_.get(kSlot);
        c_.get(kStat);
        c_.store(layout_.flags + kOffStatusV);
    }

    void upper_pre(const Up &u, bool flags_live);
    void upper_post(const Up &u, bool flags_live);
    bool lower(std::uint32_t w, std::uint32_t pc);
    void branch_to(std::uint32_t pc_local_value_known, bool from_local);
};

void Translator::upper_pre(const Up &u, bool flags_live) {
    if (u.kind == Clip) {
        c_.get(kR);
        c_.i32c(u.fs);
        c_.i32c(u.ft);
        c_.call(kClipValue);
        c_.set(kUclip);
        return;
    }
    if (!active(u, flags_live)) return;
    if (u.src == Q) pipe(kSyncQ);
    if (arithmetic(u.kind)) {
        if (u.kind == Madd || u.kind == Msub) c_.get(shadow_.at(kAcc));
        shadow_vec(u.fs);
        switch (u.src) {
        case Vec: shadow_vec(u.ft); break;
        case Bc:
            if (u.ft) {
                c_.get(shadow_.at(static_cast<int>(u.ft)));
                c_.simd(kVExtract);
                c_.byte(static_cast<std::uint8_t>(u.bc));
                c_.simd(kVSplat);
            } else {
                c_.vsplatc(u.bc == 3 ? kOne : 0u);
            }
            break;
        case Q:
            c_.get(kR);
            c_.load(kOffQ);
            c_.simd(kVSplat);
            vfix();
            break;
        default:
            c_.get(kR);
            c_.load(kOffI);
            c_.simd(kVSplat);
            vfix();
            break;
        }
        switch (u.kind) {
        case Add: chop_add(); vfix(); break;
        case Sub:
            c_.simd(kF32x4Neg);
            chop_add();
            vfix();
            break;
        case Mul: c_.simd(kF32x4Mul); vfix(); break;
        case Madd:
            c_.simd(kF32x4Mul);
            vfix();
            chop_add();
            vfix();
            break;
        default:
            c_.simd(kF32x4Mul);
            vfix();
            c_.simd(kF32x4Neg);
            chop_add();
            vfix();
            break;
        }
        c_.set(kUv);
        return;
    }
    if (vector(u.kind)) {
        raw_vec(u.fs);
        if (u.kind == Abs) {
            c_.vsplatc(0x7FFFFFFFu);
            c_.simd(kVAnd);
            c_.set(kUv);
            return;
        }
        c_.set(kV2);
        switch (u.src) {
        case Vec: raw_vec(u.ft); break;
        case Bc:
            raw_lane(u.ft, u.bc);
            c_.simd(kVSplat);
            break;
        case Q:
            c_.get(kR);
            c_.load(kOffQ);
            c_.simd(kVSplat);
            break;
        default:
            c_.get(kR);
            c_.load(kOffI);
            c_.simd(kVSplat);
            break;
        }
        c_.set(kV3);
        c_.get(kV2);
        c_.get(kV3);
        vkey_of(kV2);
        vkey_of(kV3);
        c_.simd(u.kind == Max ? kI32x4GeS : kI32x4LeS);
        c_.simd(kVBitselect);
        c_.set(kUv);
        return;
    }
    static const unsigned a[3] = {1, 2, 0}, b[3] = {2, 0, 1};
    for (unsigned l = 0; l < 4; ++l) {
        if (!lane(u.dest, static_cast<int>(l))) continue;
        switch (u.kind) {
        case Ftoi:
            raw_lane(u.fs, l);
            c_.i32c(static_cast<std::uint32_t>(u.frac));
            c_.call(kToFixed);
            break;
        case Itof:
            mem_lane(u.fs, l);
            c_.i32c(static_cast<std::uint32_t>(u.frac));
            c_.call(kFromFixed);
            break;
        case Opmula:
            raw_lane(u.fs, a[l]);
            raw_lane(u.ft, b[l]);
            c_.call(kMul);
            break;
        default:
            c_.get(kR);
            c_.load(static_cast<std::uint32_t>(offsetof(Vu0Regs, acc)) + l * 4u);
            raw_lane(u.fs, a[l]);
            raw_lane(u.ft, b[l]);
            c_.call(kMsub);
            break;
        }
        c_.set(kUx + l);
    }
}

void Translator::upper_post(const Up &u, bool flags_live) {
    if (u.kind == Clip) {
        c_.get(kR);
        fl();
        c_.get(kUclip);
        c_.call(kClipPost);
        return;
    }
    if (!active(u, flags_live)) return;
    const std::uint32_t dst = reg_off(u.target < 0 ? 0 : u.target);
    if (vector(u.kind)) {
        if (u.target == 0) {
            c_.get(kUv);
            flags_inline(u.dest);
            return;
        }
        c_.get(kR);
        if (u.dest == 0xF) {
            c_.get(kUv);
        } else {
            c_.get(kUv);
            c_.get(kR);
            c_.vload(dst);
            lane_mask(u.dest);
            c_.simd(kVBitselect);
        }
        c_.vstore(dst);
        if (u.flags && flags_live) {
            c_.get(kUv);
            flags_inline(u.dest);
        }
        return;
    }
    if (u.target == 0) {
        c_.i32c(layout_.scratch);
        c_.vconst(0, 0, 0, 0);
        c_.vstore(0);
        for (unsigned l = 0; l < 4; ++l) {
            if (!lane(u.dest, static_cast<int>(l))) continue;
            c_.i32c(layout_.scratch);
            c_.get(kUx + l);
            c_.store(l * 4u);
        }
        c_.i32c(layout_.scratch);
        c_.vload(0);
        flags_inline(u.dest);
        return;
    }
    for (unsigned l = 0; l < 4; ++l) {
        if (!lane(u.dest, static_cast<int>(l))) continue;
        c_.get(kR);
        c_.get(kUx + l);
        c_.store(dst + l * 4u);
    }
    if (u.flags && flags_live) {
        c_.get(kR);
        c_.vload(dst);
        flags_inline(u.dest);
    }
}

bool Translator::lower(std::uint32_t w, std::uint32_t pc) {
    const unsigned dest = (w >> 21) & 15u, it = (w >> 16) & 15u, is = (w >> 11) & 15u, id = (w >> 6) & 15u;
    const unsigned ft = (w >> 16) & 31u, fs = (w >> 11) & 31u;
    const std::int32_t imm11 = static_cast<std::int32_t>(w << 21) >> 21;
    const auto binary = [&](unsigned out, unsigned a, unsigned b, std::uint8_t op) {
        set_vi(out, [&] {
            vi(a);
            vi(b);
            c_.op(op);
        });
    };
    if ((w >> 25) == 0x40u) {
        const unsigned funct = w & 63u;
        if (funct < 0x3C) {
            switch (funct) {
            case 0x30: binary(id, is, it, kOpAdd); break;
            case 0x31: binary(id, is, it, kOpSub); break;
            case 0x32:
                set_vi(it, [&] {
                    vi(is);
                    c_.i32c(static_cast<std::uint32_t>(static_cast<std::int32_t>(((w >> 6) & 31u) << 27) >> 27));
                    c_.op(kOpAdd);
                });
                break;
            case 0x34: binary(id, is, it, kOpAnd); break;
            case 0x35: binary(id, is, it, kOpOr); break;
            default: return false;
            }
            return true;
        }
        const unsigned index = ((w >> 6) & 31u) * 4u + (funct & 3u);
        const unsigned fsf = (w >> 21) & 3u, ftf = (w >> 23) & 3u;
        switch (index) {
        case 0x30:
            if (ft)
                for (unsigned l = 0; l < 4; ++l) {
                    if (!lane(dest, static_cast<int>(l))) continue;
                    c_.get(kR);
                    mem_lane(fs, l);
                    c_.store(vf_off(ft) + l * 4u);
                }
            break;
        case 0x31:
            if (ft) {
                c_.get(kR);
                c_.vload(vf_off(fs));
                c_.set(kV2);
                for (unsigned l = 0; l < 4; ++l) {
                    if (!lane(dest, static_cast<int>(l))) continue;
                    c_.get(kR);
                    c_.get(kV2);
                    c_.simd(kVExtract);
                    c_.byte(static_cast<std::uint8_t>((l + 1) & 3u));
                    c_.store(vf_off(ft) + l * 4u);
                }
            }
            break;
        case 0x34:
            qword([&] { vi(is); });
            c_.set(kTmp);
            if (ft) lanes_copy(dest, ft);
            set_vi(is, [&] {
                vi(is);
                c_.i32c(1);
                c_.op(kOpAdd);
            });
            break;
        case 0x35:
            qword([&] { vi(it); });
            c_.set(kTmp);
            lanes_store(dest, fs);
            set_vi(it, [&] {
                vi(it);
                c_.i32c(1);
                c_.op(kOpAdd);
            });
            break;
        case 0x36:
            set_vi(is, [&] {
                vi(is);
                c_.i32c(1);
                c_.op(kOpSub);
            });
            qword([&] { vi(is); });
            c_.set(kTmp);
            if (ft) lanes_copy(dest, ft);
            break;
        case 0x37:
            set_vi(it, [&] {
                vi(it);
                c_.i32c(1);
                c_.op(kOpSub);
            });
            qword([&] { vi(it); });
            c_.set(kTmp);
            lanes_store(dest, fs);
            break;
        case 0x38:
            c_.get(kR);
            mem_lane(fs, fsf);
            mem_lane(ft, ftf);
            c_.call(kDiv);
            break;
        case 0x39:
            c_.get(kR);
            mem_lane(ft, ftf);
            c_.call(kSqrt);
            break;
        case 0x3A:
            c_.get(kR);
            mem_lane(fs, fsf);
            mem_lane(ft, ftf);
            c_.call(kRsqrt);
            break;
        case 0x3B: pipe(kWaitQ); break;
        case 0x3C: set_vi(it, [&] { mem_lane(fs, fsf); }); break;
        case 0x3D:
            if (ft) {
                vi(is);
                c_.op(kOpExt16);
                c_.set(kTmp);
                lanes_fill(dest, ft, kTmp);
            }
            break;
        case 0x3E:
            for (unsigned l = 0; l < 4; ++l)
                if (lane(dest, static_cast<int>(l))) {
                    set_vi(it, [&] {
                        qword([&] { vi(is); });
                        c_.load16(l * 4u);
                    });
                    break;
                }
            break;
        case 0x3F:
            qword([&] { vi(is); });
            c_.set(kTmp);
            vi(it);
            c_.set(kTmp2);
            for (unsigned l = 0; l < 4; ++l) {
                if (!lane(dest, static_cast<int>(l))) continue;
                c_.get(kTmp);
                c_.get(kTmp2);
                c_.store(l * 4u);
            }
            break;
        case 0x40:
            c_.get(kR);
            c_.get(kR);
            c_.load(kOffR);
            c_.call(kRandNext);
            c_.store(kOffR);
            if (ft) {
                c_.get(kR);
                c_.load(kOffR);
                c_.i32c(kOne);
                c_.op(kOpOr);
                c_.set(kTmp);
                lanes_fill(dest, ft, kTmp);
            }
            break;
        case 0x41:
            if (ft) {
                c_.get(kR);
                c_.load(kOffR);
                c_.i32c(0x7FFFFFu);
                c_.op(kOpAnd);
                c_.i32c(kOne);
                c_.op(kOpOr);
                c_.set(kTmp);
                lanes_fill(dest, ft, kTmp);
            }
            break;
        case 0x42:
            c_.get(kR);
            mem_lane(fs, fsf);
            c_.i32c(0x7FFFFFu);
            c_.op(kOpAnd);
            c_.store(kOffR);
            break;
        case 0x43:
            c_.get(kR);
            c_.get(kR);
            c_.load(kOffR);
            mem_lane(fs, fsf);
            c_.op(kOpXor);
            c_.i32c(0x7FFFFFu);
            c_.op(kOpAnd);
            c_.store(kOffR);
            break;
        case 0x64:
            if (ft) {
                pipe(kSyncP);
                c_.get(kR);
                c_.load(kOffP);
                c_.set(kTmp);
                lanes_fill(dest, ft, kTmp);
            }
            break;
        case 0x68: set_vi(it, [&] { c_.call(kTop); }); break;
        case 0x69: set_vi(it, [&] { c_.call(kItop); }); break;
        case 0x6C:
            vi(is);
            c_.call(kKick);
            break;
        case 0x70: case 0x71: case 0x72: case 0x73: case 0x74: case 0x75: case 0x76: case 0x78: case 0x79: case 0x7A: case 0x7C: case 0x7D: case 0x7E:
            c_.get(kR);
            c_.i32c(index);
            c_.i32c(fs);
            c_.i32c(fsf);
            c_.call(kEfu);
            break;
        case 0x7B: pipe(kWaitP); break;
        default: return false;
        }
        return true;
    }
    const std::uint32_t imm15 = ((w >> 10) & 0x7800u) | (w & 0x7FFu);
    const std::uint32_t imm12 = ((w >> 10) & 0x800u) | (w & 0x7FFu);
    const std::uint32_t imm24 = w & 0xFFFFFFu;
    const auto status = [&] {
        fl();
        c_.call(kStatus);
    };
    const auto mac = [&] {
        fl();
        c_.call(kMac);
    };
    const auto clip = [&] {
        fl();
        c_.call(kClip);
    };
    const auto offset = [&](unsigned reg) {
        vi(reg);
        c_.i32c(static_cast<std::uint32_t>(imm11));
        c_.op(kOpAdd);
    };
    switch (w >> 25) {
    case 0x00:
        qword([&] { offset(is); });
        c_.set(kTmp);
        if (ft) lanes_copy(dest, ft);
        break;
    case 0x01:
        qword([&] { offset(it); });
        c_.set(kTmp);
        lanes_store(dest, fs);
        break;
    case 0x04:
        for (unsigned l = 0; l < 4; ++l)
            if (lane(dest, static_cast<int>(l))) {
                set_vi(it, [&] {
                    qword([&] { offset(is); });
                    c_.load16(l * 4u);
                });
                break;
            }
        break;
    case 0x05:
        qword([&] { offset(is); });
        c_.set(kTmp);
        vi(it);
        c_.set(kTmp2);
        for (unsigned l = 0; l < 4; ++l) {
            if (!lane(dest, static_cast<int>(l))) continue;
            c_.get(kTmp);
            c_.get(kTmp2);
            c_.store(l * 4u);
        }
        break;
    case 0x08:
        set_vi(it, [&] {
            vi(is);
            c_.i32c(imm15);
            c_.op(kOpAdd);
        });
        break;
    case 0x09:
        set_vi(it, [&] {
            vi(is);
            c_.i32c(imm15);
            c_.op(kOpSub);
        });
        break;
    case 0x10:
        set_vi(1, [&] {
            clip();
            c_.i32c(0xFFFFFFu);
            c_.op(kOpAnd);
            c_.i32c(imm24);
            c_.op(kOpEq);
        });
        break;
    case 0x11:
        c_.get(kR);
        c_.i32c(imm24);
        c_.store(kOffClip);
        fl();
        c_.i32c(imm24);
        fl();
        c_.load(0);
        c_.i32c(4);
        c_.op(kOpSub);
        c_.call(kSetClip);
        break;
    case 0x12:
        set_vi(1, [&] {
            clip();
            c_.i32c(imm24);
            c_.op(kOpAnd);
            c_.i32c(0);
            c_.op(kOpNe);
        });
        break;
    case 0x13:
        set_vi(1, [&] {
            clip();
            c_.i32c(imm24);
            c_.op(kOpOr);
            c_.i32c(0xFFFFFFu);
            c_.op(kOpAnd);
            c_.i32c(0xFFFFFFu);
            c_.op(kOpEq);
        });
        break;
    case 0x14:
        set_vi(it, [&] {
            status();
            c_.i32c(0xFFFu);
            c_.op(kOpAnd);
            c_.i32c(imm12);
            c_.op(kOpEq);
        });
        break;
    case 0x15:
        c_.get(kR);
        status();
        c_.i32c(0x3Fu);
        c_.op(kOpAnd);
        c_.i32c(imm12 & 0xFC0u);
        c_.op(kOpOr);
        c_.store(kOffStatus);
        fl();
        c_.get(kR);
        c_.call(kForceMac);
        break;
    case 0x16:
        set_vi(it, [&] {
            status();
            c_.i32c(imm12);
            c_.op(kOpAnd);
        });
        break;
    case 0x17:
        set_vi(it, [&] {
            status();
            c_.i32c(imm12);
            c_.op(kOpOr);
            c_.i32c(0xFFFu);
            c_.op(kOpAnd);
        });
        break;
    case 0x18:
        set_vi(it, [&] {
            mac();
            c_.i32c(0xFFFFu);
            c_.op(kOpAnd);
            vi(is);
            c_.op(kOpEq);
        });
        break;
    case 0x1A:
        set_vi(it, [&] {
            mac();
            vi(is);
            c_.op(kOpAnd);
        });
        break;
    case 0x1B:
        set_vi(it, [&] {
            mac();
            vi(is);
            c_.op(kOpOr);
            c_.i32c(0xFFFFu);
            c_.op(kOpAnd);
        });
        break;
    case 0x1C:
        set_vi(it, [&] {
            clip();
            c_.i32c(0xFFFu);
            c_.op(kOpAnd);
        });
        break;
    case 0x20:
        c_.i32c(1);
        c_.set(kPend);
        break;
    case 0x21:
        set_vi(it, [&] { c_.i32c((pc + 16u) / 8u); });
        c_.i32c(1);
        c_.set(kPend);
        break;
    case 0x24:
    case 0x25:
        vi(is);
        c_.i32c(8);
        c_.op(kOpMul);
        c_.i32c(kMask);
        c_.op(kOpAnd);
        c_.set(kJpc);
        if ((w >> 25) == 0x25) set_vi(it, [&] { c_.i32c((pc + 16u) / 8u); });
        c_.i32c(1);
        c_.set(kPendj);
        break;
    case 0x28:
    case 0x29:
        vi(is);
        vi(it);
        c_.op((w >> 25) == 0x28 ? kOpEq : kOpNe);
        c_.set(kPend);
        break;
    case 0x2C: case 0x2D: case 0x2E: case 0x2F: {
        static const std::uint8_t kCompare[4] = {kOpLtS, kOpGtS, kOpLeS, kOpGeS};
        vi(is);
        c_.op(kOpExt16);
        c_.i32c(0);
        c_.op(kCompare[(w >> 25) - 0x2C]);
        c_.set(kPend);
        break;
    }
    default: return false;
    }
    return true;
}

bool Translator::run(std::vector<std::uint8_t> &out) {
    pcs_ = reachable(code_, kMask, entry_);
    if (pcs_.empty()) return false;
    in_.insert(pcs_.begin(), pcs_.end());
    bool any_status = false, any_flags = false;
    for (std::uint32_t pc : pcs_) {
        LowerInfo l;
        if (!(word(pc + 4) & 0x80000000u)) l = analyze_lower(word(pc), pc);
        any_status = any_status || l.reads_status;
        any_flags = any_flags || l.reads_flags;
        lowers_[pc] = l;
    }
    for (std::uint32_t pc : pcs_) live_in_[pc] = false;
    for (bool changed = true; changed;) {
        changed = false;
        for (auto it = pcs_.rbegin(); it != pcs_.rend(); ++it) {
            const std::uint32_t pc = *it;
            const bool out = out_live(pc);
            bool reader_near = false;
            for (std::uint32_t k = 1; k <= 3; ++k) {
                const std::uint32_t q = (pc + 8u * k) & kMask;
                if (in_.count(q) && (lowers_[q].reads_flags || lowers_[q].branch || lowers_[q].jump)) reader_near = true;
            }
            const bool kills = sets_flags(word(pc + 4)) && !reader_near;
            const bool v = lowers_[pc].reads_flags || (!kills && out);
            if (v != live_in_[pc]) {
                live_in_[pc] = v;
                changed = true;
            }
        }
    }
    std::set<int> shadowed;
    for (std::uint32_t pc : pcs_)
        for (int reg : shadow_reads(word(pc + 4))) shadowed.insert(reg);
    std::uint32_t next_local = kShadow0;
    for (int reg : shadowed) shadow_[reg] = next_local++;

    fl();
    c_.get(kR);
    c_.call(kStart);
    for (const auto &[reg, local] : shadow_) {
        c_.get(kR);
        c_.vload(reg_off(reg));
        vfix();
        c_.set(local);
    }
    c_.i32c(entry_ >> 3);
    c_.set(kPcw);
    const std::uint32_t n = static_cast<std::uint32_t>(pcs_.size());
    c_.byte(0x03);
    c_.byte(0x40);
    c_.byte(0x02);
    c_.byte(0x40);
    for (std::uint32_t k = 0; k < n; ++k) {
        c_.byte(0x02);
        c_.byte(0x40);
    }
    std::map<std::uint32_t, std::uint32_t> case_of;
    for (std::uint32_t k = 0; k < n; ++k) case_of[pcs_[k]] = k;
    c_.get(kPcw);
    c_.byte(0x0E);
    c_.u32(2048);
    for (std::uint32_t p = 0; p < 2048; ++p) {
        const auto it = case_of.find(p * 8u);
        c_.u32(it == case_of.end() ? n : it->second);
    }
    c_.u32(n);

    for (std::uint32_t k = 0; k < n; ++k) {
        c_.byte(0x0B);
        case_index_ = k;
        const std::uint32_t pc = pcs_[k];
        const std::uint32_t lo = word(pc), hi = word(pc + 4);
        const Up u = decode_upper(hi);
        const bool flags_live = any_status ? true : (any_flags && out_live(pc));
        const LowerInfo &li = lowers_[pc];

        fl();
        fl();
        c_.load(0);
        c_.i32c(1);
        c_.op(kOpAdd);
        c_.store(0);
        upper_pre(u, flags_live);
        if (hi & 0x80000000u) {
            c_.get(kR);
            c_.i32c(lo);
            c_.store(kOffI);
        } else if (!li.supported || !lower(lo, pc)) {
            // The interpreter reruns this instruction, so hand over the clock from before it.
            fl();
            fl();
            c_.load(kOffT);
            c_.i32c(1);
            c_.op(kOpSub);
            c_.store(kOffT);
            c_.i32c(pc);
            c_.tail(kInterpret);
        }
        upper_post(u, flags_live);
        const int lower_write = (hi & 0x80000000u) || !li.supported ? -1 : li.writes;
        if (lower_write >= 0 && shadow_.count(lower_write)) {
            c_.get(kR);
            c_.vload(reg_off(lower_write));
            vfix();
            c_.set(shadow_.at(lower_write));
        }
        const bool target_active = active(u, flags_live) && u.target > 0;
        if (target_active && u.target != lower_write && shadow_.count(u.target)) {
            const bool normalized = vector(u.kind) && arithmetic(u.kind);
            if (!normalized) {
                c_.get(kR);
                c_.vload(reg_off(u.target));
                vfix();
            } else if (u.dest == 0xF) {
                c_.get(kUv);
            } else {
                c_.get(kUv);
                c_.get(shadow_.at(u.target));
                lane_mask(u.dest);
                c_.simd(kVBitselect);
            }
            c_.set(shadow_.at(u.target));
        }
        if (hi & 0x40000000u) {
            c_.i32c(1);
            c_.set(kEnding);
        }

        const std::uint32_t prev = (pc - 8u) & kMask;
        if (in_.count(prev)) {
            const LowerInfo &pl = lowers_[prev];
            if (word(prev + 4) & 0x40000000u) {
                c_.get(kEnding);
                c_.open_if();
                pipe(kFinish);
                c_.get(kR);
                c_.i32c((pc + 8u) & kMask);
                c_.store(kOffTpc);
                c_.op(kOpReturn);
                c_.end();
            }
            if (pl.branch) {
                c_.get(kPend);
                c_.open_if();
                c_.i32c(0);
                c_.set(kPend);
                c_.i32c(pl.target >> 3);
                c_.set(kPcw);
                c_.byte(kOpBr);
                c_.u32(top_depth());
                c_.end();
            }
            if (pl.jump) {
                c_.get(kPendj);
                c_.open_if();
                c_.i32c(0);
                c_.set(kPendj);
                c_.get(kJpc);
                c_.i32c(3);
                c_.op(kOpShrU);
                c_.set(kPcw);
                c_.byte(kOpBr);
                c_.u32(top_depth());
                c_.end();
            }
        }
        const std::uint32_t next = (pc + 8u) & kMask;
        if (!in_.count(next)) {
            c_.i32c(next);
            c_.tail(kInterpret);
        }
    }
    c_.byte(0x0B);
    c_.get(kPcw);
    c_.i32c(3);
    c_.op(kOpShl);
    c_.tail(kInterpret);
    c_.byte(0x0B);
    c_.byte(kOpUnreachable);
    c_.byte(0x0B);

    Out m;
    m.b = {0x00, 0x61, 0x73, 0x6D, 0x01, 0x00, 0x00, 0x00};
    std::vector<std::pair<std::uint8_t, bool>> types;
    std::vector<std::uint32_t> import_type(kImports);
    for (std::uint32_t i = 0; i < kImports; ++i) {
        const std::pair<std::uint8_t, bool> t{kImportTypes[i].params, kImportTypes[i].result};
        auto found = std::find(types.begin(), types.end(), t);
        if (found == types.end()) {
            types.push_back(t);
            found = types.end() - 1;
        }
        import_type[i] = static_cast<std::uint32_t>(found - types.begin());
    }
    const std::uint32_t run_type = static_cast<std::uint32_t>(types.size());
    Out ts;
    ts.u32(run_type + 1);
    for (const auto &[params, result] : types) {
        ts.byte(0x60);
        ts.u32(params);
        for (std::uint8_t p = 0; p < params; ++p) ts.byte(0x7F);
        ts.u32(result ? 1 : 0);
        if (result) ts.byte(0x7F);
    }
    ts.byte(0x60);
    ts.u32(2);
    ts.byte(0x7F);
    ts.byte(0x7F);
    ts.u32(0);
    m.section(1, ts);

    Out is;
    is.u32(kImports + 1);
    is.name("env");
    is.name("memory");
    is.byte(0x02);
    is.byte(0x03);
    is.u32(1);
    is.u32(65536);
    for (std::uint32_t i = 0; i < kImports; ++i) {
        is.name("env");
        is.name(kImportNames[i]);
        is.byte(0x00);
        is.u32(import_type[i]);
    }
    m.section(2, is);

    Out fs;
    fs.u32(1);
    fs.u32(run_type);
    m.section(3, fs);

    Out es;
    es.u32(1);
    es.name("run");
    es.byte(0x00);
    es.u32(kImports);
    m.section(7, es);

    Out body;
    body.u32(2);
    body.u32(kI32Count);
    body.byte(0x7F);
    body.u32(kVecTemps + static_cast<std::uint32_t>(shadow_.size()));
    body.byte(0x7B);
    body.b.insert(body.b.end(), c_.b.begin(), c_.b.end());
    Out cs;
    cs.u32(1);
    cs.u32(static_cast<std::uint32_t>(body.b.size()));
    cs.b.insert(cs.b.end(), body.b.begin(), body.b.end());
    m.section(10, cs);

    out = std::move(m.b);
    return true;
}

}

bool translate(const std::uint8_t *code, std::uint32_t entry, const Layout &layout, std::vector<std::uint8_t> &out) {
    Translator t(code, entry, layout);
    return t.run(out);
}

}
