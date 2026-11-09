"""Import an Ultralytics NDJSON export using existing local JPEGs (no downloads)."""
import argparse
import json
import math
from pathlib import Path

from PIL import Image

ROOT = Path(__file__).resolve().parent
DATA = ROOT / "datasets"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("export", type=Path)
    parser.add_argument("--apply", action="store_true", help="Write labels and apply exported splits")
    args = parser.parse_args()
    records = [json.loads(line) for line in args.export.read_text(encoding="utf-8-sig").splitlines() if line.strip()]
    header = records[0]
    if header.get("type") != "dataset" or header.get("task") != "detect" or set(header.get("class_names", {})) != {"0"}:
        raise SystemExit("Expected a single-class object-detection dataset with class ID 0.")
    changes, seen = [], set()
    counts = {"train": 0, "valid": 0}
    for record in records[1:]:
        if record.get("type") != "image":
            raise SystemExit("Unexpected NDJSON record type")
        name = record["file"]
        if Path(name).name != name or "/" in name or "\\" in name or name in seen:
            raise SystemExit(f"Invalid or duplicate image name: {name}")
        seen.add(name)
        split = {"train": "train", "val": "valid"}.get(record["split"])
        if split is None:
            raise SystemExit(f"Unsupported split for {name}")
        candidates = [DATA / s / "images" / name for s in ("train", "valid")]
        sources = [p for p in candidates if p.exists()]
        if len(sources) != 1:
            raise SystemExit(f"Expected exactly one local image for {name}, found {len(sources)}")
        source = sources[0]
        with Image.open(source) as image:
            image.verify()
            w, h = image.size
        if abs(w / h - record["width"] / record["height"]) > 0.0001:
            raise SystemExit(f"Aspect ratio differs from annotated export: {name}")
        rows = []
        for box in record.get("annotations", {}).get("boxes", []):
            if len(box) != 5 or box[0] != 0 or not all(math.isfinite(v) for v in box):
                raise SystemExit(f"Invalid outlet box in {name}")
            _, x, y, bw, bh = box
            if not (0 <= x <= 1 and 0 <= y <= 1 and 0 < bw <= 1 and 0 < bh <= 1):
                raise SystemExit(f"Invalid normalized coordinates in {name}")
            if min(x-bw/2, y-bh/2) < -0.0001 or max(x+bw/2, y+bh/2) > 1.0001:
                raise SystemExit(f"Box extends outside {name}")
            rows.append("0 " + " ".join(f"{v:.8f}" for v in box[1:]))
        text = "\n".join(rows) + ("\n" if rows else "")
        target = DATA / split / "images" / name
        label = DATA / split / "labels" / (Path(name).stem + ".txt")
        other_split = "valid" if split == "train" else "train"
        other_label = DATA / other_split / "labels" / label.name
        if other_label.exists() or (label.exists() and label.read_text() != text):
            raise SystemExit(f"Existing annotation conflict for {name}; review before replacing")
        changes.append((source, target, label, text))
        counts[split] += 1
    print(f"Export: {counts['train']} train, {counts['valid']} validation images; class 0 -> outlet")
    if not args.apply:
        print("Checks passed. Add --apply to import.")
        return
    for source, target, label, text in changes:
        target.parent.mkdir(parents=True, exist_ok=True)
        label.parent.mkdir(parents=True, exist_ok=True)
        # Both paths are fixed beneath this workspace's datasets directory.
        if source != target:
            source.rename(target)
        if not label.exists():
            label.write_text(text, encoding="utf-8")
    # Preserve annotations and splits without retaining expiring signed image URLs.
    clean_header = {k: v for k, v in header.items() if k != "url"}
    clean_header["class_names"] = {"0": "outlet"}
    clean = [clean_header] + [{k: v for k, v in r.items() if k != "url"} for r in records[1:]]
    archive = DATA / "imports" / "electricaloutlets.ndjson"
    archive.parent.mkdir(parents=True, exist_ok=True)
    archive.write_text("\n".join(json.dumps(r) for r in clean) + "\n", encoding="utf-8")
    print("Imported labels and splits. Original 24 training pairs retained.")


if __name__ == "__main__":
    main()
