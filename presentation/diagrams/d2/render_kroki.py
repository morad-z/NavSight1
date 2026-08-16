#!/usr/bin/env python3
"""Render the NavSight D2 diagrams to PNG + SVG via the public Kroki service.

For each *.d2 file in this directory it:
  - builds the Kroki GET URL (zlib-deflate + base64url of the source),
  - downloads PNG and SVG renders into ./render/,
  - records the public URLs (which Canva can ingest) in kroki_urls.json.

Run:  python render_kroki.py
"""
import zlib, base64, json, os, urllib.request, urllib.error

HERE = os.path.dirname(os.path.abspath(__file__))
RENDER = os.path.join(HERE, "render")
os.makedirs(RENDER, exist_ok=True)


def kroki_url(dtype, fmt, source):
    data = zlib.compress(source.encode("utf-8"), 9)
    enc = base64.urlsafe_b64encode(data).decode("ascii")
    return f"https://kroki.io/{dtype}/{fmt}/{enc}"


def fetch(url, dest):
    req = urllib.request.Request(url, headers={"User-Agent": "navsight-kroki/1.0"})
    with urllib.request.urlopen(req, timeout=90) as r:
        body = r.read()
        with open(dest, "wb") as f:
            f.write(body)
        return r.status, r.headers.get("Content-Type", ""), len(body)


def try_fetch(url, dest, rec, tag):
    try:
        st, ct, n = fetch(url, dest)
        rec[tag + "_status"] = st
        rec[tag + "_ctype"] = ct
        rec[tag + "_bytes"] = n
    except urllib.error.HTTPError as e:
        try:
            msg = e.read().decode("utf-8", "replace")[:600]
        except Exception:
            msg = ""
        rec[tag + "_error"] = f"HTTP {e.code}: {msg}"
    except Exception as e:  # noqa: BLE001
        rec[tag + "_error"] = str(e)


def main():
    files = sorted(f for f in os.listdir(HERE) if f.endswith(".d2"))
    out = []
    for fn in files:
        name = fn[:-3]
        with open(os.path.join(HERE, fn), encoding="utf-8") as fh:
            src = fh.read()
        rec = {
            "file": fn,
            "name": name,
            "png_url": kroki_url("d2", "png", src),
            "svg_url": kroki_url("d2", "svg", src),
        }
        rec["png_url_len"] = len(rec["png_url"])
        try_fetch(rec["png_url"], os.path.join(RENDER, name + ".png"), rec, "png")
        try_fetch(rec["svg_url"], os.path.join(RENDER, name + ".svg"), rec, "svg")
        out.append(rec)
        summary = {k: v for k, v in rec.items() if not k.endswith("_url")}
        print(json.dumps(summary))
    with open(os.path.join(HERE, "kroki_urls.json"), "w", encoding="utf-8") as fh:
        json.dump(out, fh, indent=2)
    print("WROTE", os.path.join(HERE, "kroki_urls.json"))


if __name__ == "__main__":
    main()
