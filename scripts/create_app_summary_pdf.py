from pathlib import Path
from textwrap import wrap

from PIL import Image, ImageDraw, ImageFont


ROOT = Path(__file__).resolve().parents[1]
OUTPUT_DIR = ROOT / "output" / "pdf"
TMP_DIR = ROOT / "tmp" / "pdfs"
PDF_PATH = OUTPUT_DIR / "navsight_app_summary.pdf"
PNG_PATH = TMP_DIR / "navsight_app_summary_preview.png"


def load_font(name: str, size: int) -> ImageFont.FreeTypeFont | ImageFont.ImageFont:
    candidates = [
        Path("C:/Windows/Fonts") / name,
        Path("C:/Windows/Fonts") / "arial.ttf",
    ]
    for candidate in candidates:
        if candidate.exists():
            return ImageFont.truetype(str(candidate), size=size)
    return ImageFont.load_default()


PAGE_W = 1700
PAGE_H = 2200
MARGIN_X = 110
TOP = 90
CONTENT_W = PAGE_W - (MARGIN_X * 2)
GAP = 22
SECTION_GAP = 34
BOX_GAP = 28

BG = "#F7F5EF"
INK = "#111111"
MUTED = "#4E4A43"
ACCENT = "#204C45"
ACCENT_SOFT = "#D7E6E0"
BOX = "#FFFCF6"
LINE = "#D7D0C4"

TITLE_FONT = load_font("segoeuib.ttf", 56)
SUBTITLE_FONT = load_font("segoeui.ttf", 26)
H2_FONT = load_font("segoeuib.ttf", 28)
BODY_FONT = load_font("segoeui.ttf", 24)
BODY_BOLD = load_font("segoeuib.ttf", 24)
SMALL_FONT = load_font("segoeui.ttf", 20)


SUMMARY = {
    "what_it_is": (
        "NavSight is an Android navigation app that combines camera, IMU, "
        "and map services to estimate movement and guide a user on-device. "
        "Repo evidence shows a Jetpack Compose UI, native C++ visual-inertial "
        "processing, Google Maps rendering, and route guidance built around "
        "the estimated position."
    ),
    "who_its_for": (
        "Primary persona: an Android user navigating in GPS-challenged spaces "
        "such as dense urban areas, tunnels, or similar environments."
    ),
    "features": [
        "Shows a live camera preview with AR-style overlays and motion status.",
        "Tracks device motion with visual-inertial odometry via camera plus IMU.",
        "Requests an initial location, then draws position and path on Google Maps.",
        "Searches destinations with Places autocomplete and starts route guidance.",
        "Updates turn prompts, ETA, and remaining distance during navigation.",
        "Snaps estimated positions to roads with a cached Google Roads API layer.",
        "Exports path JSON and can record simulation datasets for later analysis.",
    ],
    "architecture": [
        "Compose UI in MainActivity renders camera, overlays, map, search, and nav banners.",
        "NavSightViewModel orchestrates app state and connects UI to repositories/services.",
        "SensorRepository reads accelerometer, gyroscope, camera frames, and initial location.",
        "NativeBridge JNI forwards sensor/frame data to C++ VisionModule and IMUPreintegrator.",
        "Native code returns VioData; ViewModel converts local meters to map coordinates.",
        "RoadSnapper soft-snaps recent positions to roads; NavigationManager uses snapped points for routing progress.",
    ],
    "how_to_run": [
        "Set GOOGLE_MAPS_API_KEY in local.properties.",
        "Open the project in Android Studio with the Android SDK installed.",
        "Build and run the app module on an Android device or emulator with camera/location support.",
        "Alternative CLI from repo root: gradlew.bat installDebug.",
    ],
    "footer": "Built from repo evidence in app/, tests/, Gradle files, and project docs.",
}


def draw_wrapped(draw: ImageDraw.ImageDraw, text: str, font, x: int, y: int, fill: str, width: int, line_gap: int = 8):
    avg_char_px = max(font.size * 0.52, 10)
    max_chars = max(24, int(width / avg_char_px))
    lines = wrap(text, width=max_chars)
    for line in lines:
        draw.text((x, y), line, font=font, fill=fill)
        y += font.size + line_gap
    return y


def bullet_block(draw: ImageDraw.ImageDraw, items: list[str], x: int, y: int, width: int, font, bullet_fill: str):
    bullet_x = x
    text_x = x + 28
    text_width = width - 28
    for item in items:
        draw.text((bullet_x, y), "-", font=font, fill=bullet_fill)
        y = draw_wrapped(draw, item, font, text_x, y, INK, text_width, line_gap=7)
        y += 8
    return y


def section(draw: ImageDraw.ImageDraw, title: str, x: int, y: int, width: int, height: int):
    draw.rounded_rectangle(
        (x, y, x + width, y + height),
        radius=26,
        fill=BOX,
        outline=LINE,
        width=2,
    )
    draw.text((x + 28, y + 20), title, font=H2_FONT, fill=ACCENT)


def main():
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    TMP_DIR.mkdir(parents=True, exist_ok=True)

    img = Image.new("RGB", (PAGE_W, PAGE_H), BG)
    draw = ImageDraw.Draw(img)

    draw.rounded_rectangle((70, 60, PAGE_W - 70, PAGE_H - 60), radius=36, outline=LINE, width=3)
    draw.rounded_rectangle((MARGIN_X, TOP, PAGE_W - MARGIN_X, TOP + 170), radius=30, fill=ACCENT, outline=ACCENT, width=2)
    draw.text((MARGIN_X + 34, TOP + 24), "NavSight App Summary", font=TITLE_FONT, fill="white")
    draw.text(
        (MARGIN_X + 36, TOP + 96),
        "One-page repo-backed overview",
        font=SUBTITLE_FONT,
        fill="#DCE9E5",
    )

    y = TOP + 210

    left_x = MARGIN_X
    col_w = CONTENT_W // 2 - 14
    right_x = left_x + col_w + 28

    left_box_h = 360
    right_box_h = 360
    section(draw, "What It Is", left_x, y, col_w, left_box_h)
    section(draw, "Who It's For", right_x, y, col_w, right_box_h)

    inner_y = y + 72
    draw.text((left_x + 28, inner_y), "Overview", font=BODY_BOLD, fill=MUTED)
    draw_wrapped(draw, SUMMARY["what_it_is"], BODY_FONT, left_x + 28, inner_y + 36, INK, col_w - 56, line_gap=8)

    draw.text((right_x + 28, inner_y), "Primary user", font=BODY_BOLD, fill=MUTED)
    draw_wrapped(draw, SUMMARY["who_its_for"], BODY_FONT, right_x + 28, inner_y + 36, INK, col_w - 56, line_gap=8)

    y += left_box_h + BOX_GAP

    features_h = 510
    section(draw, "What It Does", MARGIN_X, y, CONTENT_W, features_h)
    bullet_block(draw, SUMMARY["features"], MARGIN_X + 28, y + 74, CONTENT_W - 56, BODY_FONT, ACCENT)

    y += features_h + BOX_GAP

    arch_h = 360
    run_h = 270
    section(draw, "How It Works", MARGIN_X, y, CONTENT_W, arch_h)
    bullet_block(draw, SUMMARY["architecture"], MARGIN_X + 28, y + 74, CONTENT_W - 56, BODY_FONT, ACCENT)

    y += arch_h + BOX_GAP
    section(draw, "How To Run", MARGIN_X, y, CONTENT_W, run_h)
    bullet_block(draw, SUMMARY["how_to_run"], MARGIN_X + 28, y + 74, CONTENT_W - 56, BODY_FONT, ACCENT)

    footer_y = PAGE_H - 120
    draw.rounded_rectangle((MARGIN_X, footer_y - 16, PAGE_W - MARGIN_X, footer_y + 34), radius=18, fill=ACCENT_SOFT)
    draw.text((MARGIN_X + 18, footer_y - 2), SUMMARY["footer"], font=SMALL_FONT, fill=ACCENT)

    if y + run_h > footer_y - 40:
        raise RuntimeError("Layout overflow detected; content does not fit on one page.")

    img.save(PNG_PATH, "PNG")
    img.save(PDF_PATH, "PDF", resolution=300.0)

    print(PDF_PATH)
    print(PNG_PATH)


if __name__ == "__main__":
    main()
