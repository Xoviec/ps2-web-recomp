#!/usr/bin/env python3
"""PPM helpers for scripts/regress.sh, standard library only.

  regress_img.py png  in.ppm out.png         convert a frame dump for viewing
  regress_img.py diff ref.ppm new.ppm out.png  write a diff image, print the differing pixel count

The diff image shows the reference dimmed to a quarter, with every pixel that
differs painted in full red. Frames of different sizes count as all different.
"""
import struct
import sys
import zlib


def read_ppm(path):
    with open(path, "rb") as f:
        data = f.read()
    fields, pos = [], 0
    while len(fields) < 4:
        while data[pos:pos + 1].isspace():
            pos += 1
        if data[pos:pos + 1] == b"#":
            pos = data.index(b"\n", pos) + 1
            continue
        end = pos
        while not data[end:end + 1].isspace():
            end += 1
        fields.append(data[pos:end])
        pos = end
    if fields[0] != b"P6":
        raise SystemExit(f"{path}: not a binary PPM")
    w, h = int(fields[1]), int(fields[2])
    pixels = data[pos + 1:pos + 1 + w * h * 3]
    return w, h, pixels


def write_png(path, w, h, rgb):
    raw = b"".join(b"\0" + rgb[y * w * 3:(y + 1) * w * 3] for y in range(h))

    def chunk(kind, body):
        return struct.pack(">I", len(body)) + kind + body + struct.pack(">I", zlib.crc32(kind + body) & 0xFFFFFFFF)

    with open(path, "wb") as f:
        f.write(b"\x89PNG\r\n\x1a\n")
        f.write(chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0)))
        f.write(chunk(b"IDAT", zlib.compress(raw, 6)))
        f.write(chunk(b"IEND", b""))


def main(argv):
    if len(argv) == 4 and argv[1] == "png":
        w, h, rgb = read_ppm(argv[2])
        write_png(argv[3], w, h, rgb)
        return 0
    if len(argv) == 5 and argv[1] == "diff":
        rw, rh, ref = read_ppm(argv[2])
        nw, nh, new = read_ppm(argv[3])
        if (rw, rh) != (nw, nh):
            print(max(rw * rh, nw * nh))
            write_png(argv[4], nw, nh, new)
            return 0
        out = bytearray(len(ref))
        count = 0
        for i in range(0, len(ref), 3):
            if ref[i:i + 3] == new[i:i + 3]:
                out[i], out[i + 1], out[i + 2] = ref[i] >> 2, ref[i + 1] >> 2, ref[i + 2] >> 2
            else:
                out[i] = 255
                count += 1
        write_png(argv[4], rw, rh, bytes(out))
        print(count)
        return 0
    sys.stderr.write(__doc__)
    return 2


if __name__ == "__main__":
    sys.exit(main(sys.argv))
