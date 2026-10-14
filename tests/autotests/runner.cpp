// Headless runner for ps2autotests: boots a recompiled test ELF and sends what the
// test prints through the fileio RPC server (fd 1/2) to stdout.
#include "ps2/system.hpp"
#include "ps2/memory.hpp"
#include "iop/sif.hpp"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

void register_generated_code();
void register_generated_vu();

namespace {

std::uint32_t rd32(std::uint32_t a) { std::uint32_t v; std::memcpy(&v, ps2::g_ram + (a & 0x1FFFFFFu), 4); return v; }

bool fileio(std::uint32_t method, std::uint32_t args, std::uint32_t, std::uint32_t ret, std::uint32_t) {
    std::int32_t result = 0;
    switch (method) {
    case 0: result = 3; break;  // open
    case 1: result = 0; break;  // close
    case 3: {                   // write {fd, ptr, size, mis, aligned[16]}
        const std::uint32_t fd = rd32(args), ptr = rd32(args + 4), size = rd32(args + 8);
        if (fd == 1 || fd == 2) {
            const char *text = reinterpret_cast<const char *>(ps2::g_ram + (ptr & 0x1FFFFFFu));
            std::fwrite(text, 1, size, stdout);
            std::fflush(stdout);
            // Tests park their threads after printing the end marker instead of exiting.
            if (std::string(text, size).find("-- TEST END") != std::string::npos) ps2::halt("test end");
        }
        result = static_cast<std::int32_t>(size);
        break;
    }
    default: result = 0; break;
    }
    std::memcpy(ps2::g_ram + (ret & 0x1FFFFFFu), &result, 4);
    return true;
}

}

int main(int argc, char **argv) {
    if (argc < 2) { std::fprintf(stderr, "usage: autotest <elf> [--frames n] [--misses file] [--trace-syscalls] [--trace-threads]\n"); return 2; }
    ps2::g_options.max_frames = 1200;
    for (int i = 2; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--frames" && i + 1 < argc) ps2::g_options.max_frames = std::strtoull(argv[++i], nullptr, 10);
        else if (a == "--misses" && i + 1 < argc) ps2::g_options.misses_path = argv[++i];
        else if (a == "--trace-syscalls") ps2::g_options.trace_syscalls = true;
        else if (a == "--trace-threads") ps2::g_options.trace_threads = true;
    }
    register_generated_code();
    register_generated_vu();
    std::uint32_t entry = 0;
    if (!ps2::load_elf(argv[1], entry)) { std::fprintf(stderr, "cannot load %s\n", argv[1]); return 1; }
    std::string name = argv[1]; name = name.substr(name.find_last_of('/') + 1);
    ps2::boot(entry, {"host:" + name});
    ps2::sif::register_hle_server(0x80000001u, fileio);
    return ps2::run();
}
