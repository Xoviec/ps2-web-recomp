#include "ps2/override.hpp"
#include "ps2/generated.hpp"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

namespace ps2 {

bool g_overrides_armed = false;
std::uint8_t g_override_bits[kRamSize / 32];

namespace {

struct Override {
    std::uint32_t address;
    const char *name;
    OverrideFn fn;
    bool armed;
    std::uint64_t calls;
    std::uint64_t verified;
    std::uint64_t mismatches;
};

std::vector<Override> &registry() {
    static std::vector<Override> overrides;
    return overrides;
}

// The address call_original() is entering: its hook lets the translated code run once.
std::uint32_t g_bypass = 0;

// PS2_OVERRIDE_VERIFY=1: every call of an armed override runs twice from the same snapshot, once
// through the override and once through the translated code, and the results are compared: all
// of RAM, scratchpad and VU0 data memory, the VU0 registers, the registers a caller may read (v0,
// s0-s7, gp, sp, fp, ra, f0, pc) and the cycles charged. Both runs get a large cycle budget so
// neither reaches an interrupt check. The state is then restored and the translated code runs for
// real, so the game always continues exactly as without the override. Only for overrides whose
// callees touch nothing but memory and registers (no syscalls, no hardware registers).
bool g_verify = false;

struct Snapshot {
    std::unique_ptr<std::uint8_t[]> ram{new std::uint8_t[kRamSize]};
    std::uint8_t spr[kScratchpadSize];
    std::uint8_t vu0[4096];
    Cpu cpu;

    void take(const Cpu &c) {
        std::memcpy(ram.get(), g_ram, kRamSize);
        std::memcpy(spr, g_spr, sizeof spr);
        std::memcpy(vu0, vu0_data(), sizeof vu0);
        cpu = c;
    }
    void restore(Cpu &c) const {
        std::memcpy(g_ram, ram.get(), kRamSize);
        std::memcpy(g_spr, spr, sizeof spr);
        std::memcpy(vu0_data(), vu0, sizeof vu0);
        c = cpu;
    }
};

// Prints what differs between the override's result `a` and the translated code's (live state).
bool same_result(const Override &o, const Snapshot &a, const Cpu &c, std::uint64_t call) {
    int shown = 0;
    auto report = [&](const char *what, std::uint64_t at, std::uint64_t mine, std::uint64_t theirs) {
        if (shown++ < 8)
            std::fprintf(stderr, "[override] %s call %llu: %s 0x%llX override 0x%llX translated 0x%llX\n", o.name,
                         static_cast<unsigned long long>(call), what, static_cast<unsigned long long>(at),
                         static_cast<unsigned long long>(mine), static_cast<unsigned long long>(theirs));
    };
    for (std::uint32_t at = 0; at < kRamSize; at += 4096) {
        if (!std::memcmp(a.ram.get() + at, g_ram + at, 4096)) continue;
        for (std::uint32_t k = at; k < at + 4096; ++k)
            if (a.ram[k] != g_ram[k]) report("ram byte", k, a.ram[k], g_ram[k]);
    }
    for (std::uint32_t k = 0; k < kScratchpadSize; ++k)
        if (a.spr[k] != g_spr[k]) report("scratchpad byte", k, a.spr[k], g_spr[k]);
    for (std::uint32_t k = 0; k < 4096; ++k)
        if (a.vu0[k] != vu0_data()[k]) report("vu0 data byte", k, a.vu0[k], vu0_data()[k]);
    if (std::memcmp(&a.cpu.vu0, &c.vu0, sizeof c.vu0)) report("vu0 registers", 0, 0, 0);
    for (int r : {2, 16, 17, 18, 19, 20, 21, 22, 23, 28, 29, 30, 31})
        for (int h = 0; h < 2; ++h)
            if (a.cpu.r[r].d[h] != c.r[r].d[h]) report(h ? "gpr (upper half)" : "gpr", r, a.cpu.r[r].d[h], c.r[r].d[h]);
    if (a.cpu.f[0] != c.f[0]) report("f0", 0, a.cpu.f[0], c.f[0]);
    if (a.cpu.pc != c.pc) report("pc", 0, a.cpu.pc, c.pc);
    if (a.cpu.budget != c.budget) report("cycles left", 0, static_cast<std::uint32_t>(a.cpu.budget), static_cast<std::uint32_t>(c.budget));
    return shown == 0;
}

void verify(Override &o, Cpu &c, std::uint32_t address, std::uint32_t depth) {
    static Snapshot before, mine;
    constexpr std::int32_t kPlenty = 1 << 30;
    before.take(c);
    c.budget = kPlenty;
    o.fn(c, depth);
    mine.take(c);
    before.restore(c);
    c.budget = kPlenty;
    call_original(c, address, depth);
    ++o.verified;
    if (!same_result(o, mine, c, o.calls)) ++o.mismatches;
    before.restore(c);
}

bool selected(const Override &o, const std::string &list) {
    std::size_t at = 0;
    while (at <= list.size()) {
        std::size_t end = list.find(',', at);
        if (end == std::string::npos) end = list.size();
        const std::string item = list.substr(at, end - at);
        if (item == "all" || item == o.name) return true;
        if (!item.empty() && std::strtoul(item.c_str(), nullptr, 16) == o.address) return true;
        at = end + 1;
    }
    return false;
}

}

bool register_override(std::uint32_t address, const char *name, OverrideFn fn) {
    registry().push_back({address, name, fn, false, 0, 0, 0});
    return true;
}

void init_overrides() {
    const char *env = std::getenv("PS2_OVERRIDES");
    const std::string list = env ? env : "";
    for (Override &o : registry()) {
        o.armed = !list.empty() && selected(o, list) && o.address < kRamSize && !(o.address & 3u);
        if (!o.armed) continue;
        if (override_armed(o.address)) {
            std::fprintf(stderr, "[override] %s not armed: another override already replaces 0x%08X\n", o.name, o.address);
            o.armed = false;
            continue;
        }
        g_override_bits[o.address >> 5] |= static_cast<std::uint8_t>(1u << ((o.address >> 2) & 7u));
        g_overrides_armed = true;
        std::fprintf(stderr, "[override] %s at 0x%08X armed\n", o.name, o.address);
    }
    g_verify = g_overrides_armed && std::getenv("PS2_OVERRIDE_VERIFY");
    if (g_verify) std::fprintf(stderr, "[override] verify mode: overrides are compared, the translated code runs\n");
    if (!list.empty() && !g_overrides_armed) std::fprintf(stderr, "[override] PS2_OVERRIDES=%s armed nothing\n", list.c_str());
}

bool enter_override(Cpu &c, std::uint32_t address, std::uint32_t depth) {
    if (g_bypass == address) {
        g_bypass = 0;
        return false;
    }
    for (Override &o : registry())
        if (o.address == address && o.armed) {
            ++o.calls;
            if (g_verify) {
                verify(o, c, address, depth);
                return false;
            }
            o.fn(c, depth);
            return true;
        }
    return false;
}

void override_stats() {
    for (const Override &o : registry())
        if (o.armed) {
            std::fprintf(stderr, "[override] %s: %llu calls\n", o.name, static_cast<unsigned long long>(o.calls));
            if (g_verify)
                std::fprintf(stderr, "[override] %s: verified %llu calls, %llu mismatches\n", o.name,
                             static_cast<unsigned long long>(o.verified), static_cast<unsigned long long>(o.mismatches));
        }
}

void call_original(Cpu &c, std::uint32_t address, std::uint32_t depth) {
    g_bypass = address;
    c.pc = address;
    dispatch(c, 0, depth);
    g_bypass = 0;
}

bool call_function(Cpu &c, std::uint32_t target, std::uint32_t ret, std::uint32_t depth) {
    c.r[31].sd[0] = static_cast<std::int32_t>(ret);
    c.pc = target;
    if (depth >= kMaxDepth) return false;
    dispatch(c, 0, depth + 1);
    return c.pc == ret;
}

}
