"""Train an outlet detector after adding an independent labeled validation set."""
import argparse
import os
from pathlib import Path

ROOT = Path(__file__).resolve().parent
os.environ.setdefault("YOLO_CONFIG_DIR", str(ROOT))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--epochs", type=int, default=50)
    parser.add_argument("--check-only", action="store_true")
    parser.add_argument("--device", default=None, help="cpu or GPU index, e.g. 0")
    args = parser.parse_args()
    extensions = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}
    for split in ("train", "valid"):
        folder = ROOT / "datasets" / split
        images = [p for p in (folder / "images").glob("*") if p.suffix.lower() in extensions]
        if not images:
            raise SystemExit(f"No {split} images found in {folder / 'images'}. Add labeled images before training.")
        for image in images:
            label = folder / "labels" / f"{image.stem}.txt"
            if not label.exists():
                raise SystemExit(f"Missing label: {label}. Use an empty label file for a verified negative image.")
            for line in label.read_text().splitlines():
                if not line.strip():
                    continue
                values = list(map(float, line.split()))
                if (len(values) != 5 or values[0] != 0
                        or not all(0 <= x <= 1 for x in values[1:])
                        or values[3] <= 0 or values[4] <= 0):
                    raise SystemExit(f"Invalid YOLO outlet annotation: {label}")
        print(f"{split}: {len(images)} images with label files", flush=True)

    if args.check_only:
        print("Dataset checks passed.")
        return
    import torch
    from ultralytics import YOLO

    model = YOLO("yolo26n.pt")
    model.train(
        data=str(ROOT / "datasets" / "data.yaml"),
        epochs=args.epochs,
        imgsz=640,
        batch=4,
        workers=0,
        device=args.device if args.device is not None else (0 if torch.cuda.is_available() else "cpu"),
        project=str(ROOT / "runs" / "outlet_detection"),
        name="baseline",
        seed=42,
        patience=15,
    )
    print(f"Trained model: {model.trainer.best}")


if __name__ == "__main__":
    main()
