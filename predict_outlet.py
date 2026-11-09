"""Run the trained outlet detector on a photo, including over SSH on a Pi."""
import argparse
import os
from pathlib import Path

ROOT = Path(__file__).resolve().parent
os.environ.setdefault("YOLO_CONFIG_DIR", str(ROOT))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("image", type=Path)
    parser.add_argument("--model", type=Path, default=ROOT / "models" / "best.pt")
    parser.add_argument("--conf", type=float, default=0.25)
    args = parser.parse_args()
    for path in (args.image, args.model):
        if not path.is_file():
            raise SystemExit(f"File not found: {path}")
    if not 0 <= args.conf <= 1:
        raise SystemExit("--conf must be between 0 and 1")
    from ultralytics import YOLO

    model = YOLO(str(args.model))
    if model.task != "detect" or model.names != {0: "outlet"}:
        raise SystemExit(f"Expected the trained outlet detector, got {model.task}: {model.names}")
    result = model.predict(
        source=str(args.image), imgsz=640, conf=args.conf, device="cpu",
        save=True, show=False, project=str(ROOT / "results"), name="outlet_test",
    )[0]
    print(f"Detected {len(result.boxes)} outlet(s) at confidence >= {args.conf}")
    for box in result.boxes:
        print(f"  confidence={float(box.conf[0]):.3f}, box_xyxy={box.xyxy[0].tolist()}")
    print(f"Annotated photo folder: {result.save_dir}")


if __name__ == "__main__":
    main()
