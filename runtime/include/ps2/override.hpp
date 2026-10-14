#pragma once

// Function overrides: replace a translated game function with hand-written C++.
//
// With the CMake option PS2_OVERRIDES_DIR set, every *.cpp in that folder is linked into the
// runner and the generated code is compiled with PS2_OVERRIDES, which turns on the
// PS2_OVERRIDE_HOOK(address) that eerecomp puts at the start of every function. An override is
// registered with PS2_OVERRIDE(address, Name) and stays off until the environment variable
// PS2_OVERRIDES names it (comma separated names or hex addresses, or "all").
//
// An override runs in place of the translated function, on the same CPU state and memory: it
// reads its arguments from the registers, writes its results back, and returns to the caller by
// setting c.pc to the return address (normally c.r[31]) exactly as the translated code does.
// call_original() runs the translated function instead, so an override that only calls it must
// leave everything bit-identical. Without PS2_OVERRIDES the hooks compile to nothing.
//
// PS2_OVERRIDE_VERIFY=1 runs each armed override and the translated function from the same
// snapshot, reports any difference in memory, VU0 state, caller-visible registers or cycles
// charged, then lets the translated code run for real (see verify() in override.cpp).
//
// The hook sits on the function's label, so a branch from inside the function back to its own
// first instruction would enter the override again; do not arm an override on such a function.

#include "cpu.hpp"
#include "memory.hpp"

#include <cstdint>

namespace ps2 {

using OverrideFn = void (*)(Cpu &c, std::uint32_t depth);

bool register_override(std::uint32_t address, const char *name, OverrideFn fn);
// Arms the overrides PS2_OVERRIDES selects; register_units() calls it once.
void init_overrides();
// Prints how often each armed override ran; the runner calls it when it stops.
void override_stats();

extern bool g_overrides_armed;
extern std::uint8_t g_override_bits[kRamSize / 32];

inline bool override_armed(std::uint32_t address) {
    return g_overrides_armed && address < kRamSize && ((g_override_bits[address >> 5] >> ((address >> 2) & 7u)) & 1u);
}

// Runs the override armed at `address`; false when the translated code should run instead.
bool enter_override(Cpu &c, std::uint32_t address, std::uint32_t depth);

// Runs the translated function at `address` with the current registers, at the same call depth,
// as if the override were not there. Returns with c.pc set the way the translated code left it.
void call_original(Cpu &c, std::uint32_t address, std::uint32_t depth);

// Calls the game function at `target` the way a JAL at `ret - 8` does: ra = ret, then the
// function. True when it returned normally; false when it unwound (a thread switch or the
// host stack limit), in which case the caller must return at once and the game resumes later
// at `ret` in the translated code, so the registers it expects there must already be in place.
bool call_function(Cpu &c, std::uint32_t target, std::uint32_t ret, std::uint32_t depth);

}

#define PS2_OVERRIDE(address, fn) \
    [[maybe_unused]] static const bool ps2_override_registered_##fn = ::ps2::register_override((address), #fn, &(fn))
