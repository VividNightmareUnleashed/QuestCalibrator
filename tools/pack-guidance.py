"""Pack completed Blender renders into the embedded calibration PNG atlases."""

import json
from pathlib import Path

from PIL import Image

ROOT = Path(__file__).resolve().parents[1]

for setup in ("headset", "handheld", "contact"):
    frame_size, columns, rows = ((320, 192), 16, 30) if setup == "handheld" else ((192, 168), 24, 35)
    folder = ROOT / "x64/guidance/frames" / setup
    manifest = json.loads((folder / "complete.json").read_text(encoding="utf-8"))
    if manifest != {"frames": columns * rows, "size": list(frame_size), "fps": 60}:
        raise ValueError(f"Incomplete or incompatible render: {folder}")
    atlas = Image.new("RGBA", (frame_size[0] * columns, frame_size[1] * rows))
    for frame in range(columns * rows):
        path = folder / f"{frame:03}.png"
        with Image.open(path) as image:
            if image.size != frame_size or image.mode != "RGBA":
                raise ValueError(f"Unexpected frame format: {path}")
            atlas.paste(image, ((frame % columns) * frame_size[0],
                               (frame // columns) * frame_size[1]))
    destination = ROOT / "Overlay/assets" / f"guide-{setup}.png"
    destination.parent.mkdir(parents=True, exist_ok=True)
    atlas.save(destination, optimize=True)
    print(destination, destination.stat().st_size)
