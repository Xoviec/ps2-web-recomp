#include "gif.hpp"

#include "../dmac.hpp"
#include "gs.hpp"
#include "queue.hpp"

#include "ps2/state.hpp"

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <vector>

namespace ps2::gif {

namespace {

struct Path {
    bool active = false;
    std::uint32_t nloop = 0, loop = 0, nreg = 0, reg = 0, flg = 0;
    std::uint64_t regs = 0;
    bool eop = false;
    bool address_data = false;
};

Path g_paths[4];
std::uint32_t g_stat = 0;
std::uint64_t g_tags[4][4];

template <typename Sink>
void walk(Path &p, const std::uint8_t *data, std::uint32_t qwc, Sink &sink) {
    for (std::uint32_t i = 0; i < qwc; ++i) {
        Reg128 q;
        std::memcpy(&q, data + static_cast<std::size_t>(i) * 16, 16);
        if (!p.active) {
            p.nloop = static_cast<std::uint32_t>(q.d[0]) & 0x7FFFu;
            p.eop = (q.d[0] >> 15) & 1u;
            p.flg = static_cast<std::uint32_t>(q.d[0] >> 58) & 3u;
            p.nreg = static_cast<std::uint32_t>(q.d[0] >> 60) & 0xFu;
            if (!p.nreg) p.nreg = 16;
            p.regs = q.d[1];
            p.loop = 0;
            p.reg = 0;
            sink.tag(p, q);
            p.active = p.nloop != 0;
            continue;
        }
        switch (p.flg) {
        case 0:
            sink.packed(static_cast<std::uint32_t>(p.regs >> (p.reg * 4)) & 0xFu, q);
            if (++p.reg >= p.nreg) {
                p.reg = 0;
                if (++p.loop >= p.nloop) p.active = false;
            }
            break;
        case 1:
            for (int half = 0; half < 2 && p.active; ++half) {
                sink.reglist(static_cast<std::uint32_t>(p.regs >> (p.reg * 4)) & 0xFu, q.d[half]);
                if (++p.reg >= p.nreg) {
                    p.reg = 0;
                    if (++p.loop >= p.nloop) p.active = false;
                }
            }
            break;
        default: {
            const std::uint32_t run = std::min(qwc - i, p.nloop - p.loop);
            sink.image(data + static_cast<std::size_t>(i) * 16, run);
            p.loop += run;
            i += run - 1;
            if (p.loop >= p.nloop) p.active = false;
            break;
        }
        }
    }
}

struct GsSink {
    int path;
    void tag(const Path &p, const Reg128 &q) {
        ++g_tags[path][p.flg];
        gs::g_state.q = 1.0f;
        if (((q.d[0] >> 46) & 1u) && p.flg == 0) gs::write(0x00, (q.d[0] >> 47) & 0x7FFu);
    }
    void packed(std::uint32_t reg, const Reg128 &q) { gs::write_packed(reg, q); }
    void reglist(std::uint32_t reg, std::uint64_t value) { gs::write(reg, value); }
    void image(const std::uint8_t *data, std::uint32_t qwords) { gs::write_image(data, qwords); }
};

void scan(Path &p, const std::uint8_t *data, std::uint32_t qwc) {
    std::uint32_t i = 0;
    while (i < qwc) {
        Reg128 q;
        if (!p.active) {
            std::memcpy(&q, data + static_cast<std::size_t>(i) * 16, 16);
            p.nloop = static_cast<std::uint32_t>(q.d[0]) & 0x7FFFu;
            p.flg = static_cast<std::uint32_t>(q.d[0] >> 58) & 3u;
            p.nreg = static_cast<std::uint32_t>(q.d[0] >> 60) & 0xFu;
            if (!p.nreg) p.nreg = 16;
            p.regs = q.d[1];
            p.loop = 0;
            p.reg = 0;
            p.address_data = false;
            if (p.flg == 0)
                for (std::uint32_t r = 0; r < p.nreg; ++r) p.address_data = p.address_data || ((p.regs >> (r * 4)) & 0xFu) == 0xEu;
            p.active = p.nloop != 0;
            ++i;
            continue;
        }
        if (p.flg == 0 && p.address_data) {
            for (; i < qwc && p.active; ++i) {
                const std::uint8_t *at = data + static_cast<std::size_t>(i) * 16;
                if (((p.regs >> (p.reg * 4)) & 0xFu) == 0xEu && at[8] >= 0x50 && at[8] <= 0x62) {
                    std::uint64_t value;
                    std::memcpy(&value, at, 8);
                    if (at[8] >= 0x60) gs::signal(at[8], value);
                    else if (at[8] <= 0x53) gs::note_transfer(at[8], value);
                }
                if (++p.reg >= p.nreg) {
                    p.reg = 0;
                    if (++p.loop >= p.nloop) p.active = false;
                }
            }
            continue;
        }
        const std::uint64_t done = static_cast<std::uint64_t>(p.loop) * p.nreg + p.reg;
        const std::uint64_t left = static_cast<std::uint64_t>(p.nloop) * p.nreg - done;
        std::uint64_t take, used;
        if (p.flg == 0) {
            take = used = std::min<std::uint64_t>(left, qwc - i);
        } else if (p.flg == 1) {
            take = std::min<std::uint64_t>((left + 1) / 2, qwc - i);
            used = std::min<std::uint64_t>(take * 2, left);
        } else {
            take = std::min<std::uint64_t>(p.nloop - p.loop, qwc - i);
            p.loop += static_cast<std::uint32_t>(take);
            if (p.loop >= p.nloop) p.active = false;
            i += static_cast<std::uint32_t>(take);
            continue;
        }
        p.loop = static_cast<std::uint32_t>((done + used) / p.nreg);
        p.reg = static_cast<std::uint32_t>((done + used) % p.nreg);
        if (used == left) p.active = false;
        i += static_cast<std::uint32_t>(take);
    }
}

Path g_scan[4];

// PATH3 data the GIF holds while VIF1's MSKPATH3 masks the path. The mask only stops PATH3
// between packets, once an EOP tag's data is through: games chain a group of uploads into
// one packet and let exactly one group in per unmask.
struct Fifo {
    std::vector<std::uint8_t> data;
    std::size_t head = 0;
    std::uint32_t left = 0; // data qwords still due under the current tag
    bool between = true, last = false, masked = false;
    std::uint32_t queued() const { return static_cast<std::uint32_t>((data.size() - head) / 16); }
};

// With the VU thread up each side keeps one, fed in the same order: the EE's follows its
// shadow VIF1 for channel 2's completion and GIF_STAT, the VU thread's holds the data for
// the GS so it lands between the right PATH1/PATH2 draws.
Fifo g_fifo[2];

bool on_vu_thread() { return gs::queue::role() == gs::queue::Role::Vu; }
Fifo &fifo() { return g_fifo[on_vu_thread() ? 1 : 0]; }

void send(int path, const std::uint8_t *data, std::uint32_t qwc) {
    if (!qwc) return;
    if (!gs::queue::active()) {
        render(path, data, qwc);
        return;
    }
    if (!on_vu_thread()) scan(g_scan[path], data, qwc);
    // the VU thread's copy of PATH3 goes to the GS
    if (path == 3 && gs::queue::vu_active() && !on_vu_thread()) return;
    gs::queue::post(gs::queue::kGif, static_cast<std::uint32_t>(path), qwc, data, qwc * 16);
}

// Qwords of `data` PATH3 sends before it can next stop: through the end of the current
// packet, or all of it.
std::uint32_t advance(Fifo &f, const std::uint8_t *data, std::uint32_t qwc, bool all) {
    std::uint32_t i = 0;
    while (i < qwc) {
        if (!f.left) {
            std::uint64_t tag;
            std::memcpy(&tag, data + static_cast<std::size_t>(i) * 16, 8);
            const std::uint32_t nloop = static_cast<std::uint32_t>(tag) & 0x7FFFu, flg = static_cast<std::uint32_t>(tag >> 58) & 3u;
            const std::uint32_t nreg = (static_cast<std::uint32_t>(tag >> 60) & 0xFu) ? static_cast<std::uint32_t>(tag >> 60) & 0xFu : 16u;
            f.left = flg == 0 ? nloop * nreg : flg == 1 ? (nloop * nreg + 1) / 2 : nloop;
            f.last = (tag >> 15) & 1u;
            ++i;
        } else {
            const std::uint32_t take = std::min(f.left, qwc - i);
            f.left -= take;
            i += take;
        }
        f.between = !f.left && f.last;
        if (f.between && !all) break;
    }
    return i;
}

void release(Fifo &f, bool all) {
    const std::uint32_t n = advance(f, f.data.data() + f.head, f.queued(), all);
    if (!n) return;
    send(3, f.data.data() + f.head, n);
    f.head += static_cast<std::size_t>(n) * 16;
    if (f.head < f.data.size()) return;
    f.data.clear();
    f.head = 0;
    if (!on_vu_thread()) dmac::resume(2);
}

void push(const std::uint8_t *data, std::uint32_t qwc) {
    Fifo &f = fifo();
    if (!f.queued()) {
        const std::uint32_t n = f.masked && f.between ? 0 : advance(f, data, qwc, !f.masked);
        send(3, data, n);
        data += static_cast<std::size_t>(n) * 16;
        qwc -= n;
    }
    f.data.insert(f.data.end(), data, data + static_cast<std::size_t>(qwc) * 16);
    // data already waiting while unmasked: the unmask that let one packet through was a while ago
    if (!f.masked) release(f, true);
}

}

void reset() {
    Fifo &f = fifo();
    const bool held = f.queued() != 0;
    f = Fifo{};
    if (!on_vu_thread()) {
        g_stat = 0;
        if (held) dmac::resume(2);
    }
    if (gs::queue::active()) {
        if (!on_vu_thread())
            for (Path &p : g_scan) p = Path{};
        gs::queue::post(gs::queue::kGifReset, 0, 0, nullptr, 0);
        return;
    }
    render_reset();
}

void render_reset() {
    for (Path &p : g_paths) p = Path{};
}

void render(int path, const std::uint8_t *data, std::uint32_t qwc) {
    GsSink sink{path};
    walk(g_paths[path], data, qwc, sink);
}

void transfer(int path, const std::uint8_t *data, std::uint32_t qwc) {
    if (path != 3) {
        send(path, data, qwc);
        return;
    }
    if (gs::queue::vu_active() && !on_vu_thread()) gs::queue::vu().post(gs::queue::kGif, 3, qwc, data, qwc * 16);
    push(data, qwc);
}

void scan_signals(int path, const std::uint8_t *data, std::uint32_t qwc) { scan(g_scan[path], data, qwc); }

bool busy(int path) { return g_paths[path].active; }

void stats() {
    for (int p = 1; p < 4; ++p)
        std::fprintf(stderr, "[gif] path %d tags: packed %llu reglist %llu image %llu\n", p,
                     static_cast<unsigned long long>(g_tags[p][0]), static_cast<unsigned long long>(g_tags[p][1]),
                     static_cast<unsigned long long>(g_tags[p][2] + g_tags[p][3]));
}

std::uint32_t read(std::uint32_t a) {
    // M3P, and FQC counting what a masked PATH3 has left waiting in the FIFO
    const Fifo &f = g_fifo[0];
    if ((a & 0xF0u) == 0x20u) return (g_stat & ~2u) | (f.masked ? 2u : 0u) | (std::min(f.queued(), 16u) << 24);
    return 0;
}

void write(std::uint32_t a, std::uint32_t v) {
    if ((a & 0xF0u) == 0x00u && (v & 1u)) reset();
    if ((a & 0xF0u) == 0x10u) g_stat = (g_stat & ~1u) | (v & 1u);
}

void set_path3_masked(bool masked) {
    Fifo &f = fifo();
    f.masked = masked;
    // the GIF arbitrates as soon as the mask drops, so a packet goes out before VIF1 can mask again
    if (!masked) release(f, false);
}

void path3_flush() {
    Fifo &f = fifo();
    if (!f.masked) release(f, true);
}

void vif1_idle() {
    if (gs::queue::vu_active() && !on_vu_thread()) gs::queue::vu().post(gs::queue::kPath3Flush, 0, 0, nullptr, 0);
    path3_flush();
}

bool path3_held() { return g_fifo[0].queued() != 0; }

}

namespace ps2::gif {

void state(ps2::state::Stream &s) {
    s.tag("gif");
    s.pod(g_paths);
    s.pod(g_stat);
    s.pod(g_tags);
    s.pod(g_scan);
    Fifo &f = g_fifo[0];
    f.data.erase(f.data.begin(), f.data.begin() + static_cast<std::ptrdiff_t>(f.head));
    f.head = 0;
    s.vec(f.data);
    s.pod(f.left);
    s.pod(f.between);
    s.pod(f.last);
    s.pod(f.masked);
    // the VU thread has caught up (sync_vu_thread), so its copy matches the EE's
    if (s.loading()) g_fifo[1] = f;
}

}
