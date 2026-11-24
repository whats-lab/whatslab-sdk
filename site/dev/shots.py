import sys
import time

from playwright.sync_api import sync_playwright

BASE = sys.argv[1]
OUT = sys.argv[2]
DESKTOP = {"width": 1440, "height": 900}
PHONE = {"width": 390, "height": 844}
CASES = [
    ("desktop_default", "", DESKTOP, "light", None),
    ("pinch_mid", "#hand=tesollo_dg5f_s&motion=pinch_index&side=right", DESKTOP, "light", 0.5),
    ("phone", "#hand=tesollo_dg5f_s&motion=fist&side=right", PHONE, "light", 0.6),
    ("dark", "#hand=shadow_hand&motion=v_pose&side=left", DESKTOP, "dark", 0.7),
]
ARGS = ["--use-angle=swiftshader", "--enable-unsafe-swiftshader", "--ignore-gpu-blocklist"]
SCRUB = "(e, v) => { e.value = v; e.dispatchEvent(new Event('input')); }"
ENCODED = (
    "performance.getEntriesByType('resource')"
    ".reduce((a, e) => a + (e.encodedBodySize || 0), 0)"
)

with sync_playwright() as p:
    browser = p.chromium.launch(args=ARGS)
    for name, hash_, vp, scheme, pos in CASES[: int(sys.argv[3]) if len(sys.argv) > 3 else None]:
        page = browser.new_page(viewport=vp, color_scheme=scheme)
        errors = []
        page.on(
            "console",
            lambda m, e=errors: m.type in ("error", "warning") and e.append(f"{m.type}: {m.text}"),
        )
        page.on("pageerror", lambda x, e=errors: e.append(f"pageerror: {x}"))
        t0 = time.time()
        page.goto(BASE + "index.html" + hash_)
        page.wait_for_function("window.__firstRender", timeout=120000)
        page.wait_for_timeout(1500)
        if pos is not None:
            page.click("#play")
            page.eval_on_selector("#scrub", SCRUB, int(pos * 1000))
            page.wait_for_timeout(500)
        page.screenshot(path=f"{OUT}/{name}.png")
        fr = page.evaluate("window.__firstRender")
        size = page.evaluate(ENCODED)
        print(
            f"{name}: firstRender={fr} ms wall={time.time() - t0:.1f}s "
            f"encoded={size / 1e6:.2f}MB errors={errors}"
        )
        page.close()
    browser.close()
