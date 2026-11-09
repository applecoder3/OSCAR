# Outlet detector: training and Raspberry Pi prediction

The Ultralytics platform export was imported using the local JPEG files. Its
16 training / 6 validation split is preserved. Together with the original 24
training images, the dataset has 40 training images (37 boxes, 5 backgrounds)
and 6 validation images (4 boxes, 2 backgrounds). Class 0 is named `outlet`.
Empty label files explicitly identify reviewed background images.

## Completed baseline results

The initial run completed 46 epochs, stopping after 15 epochs without improvement.
The selected checkpoint is from epoch 31 (5,374,405 bytes). Final validation:
precision 0.991, recall 1.000, mAP50 0.995, mAP50-95 0.852. These numbers describe
only six validation photos containing four labeled outlet assemblies; they are
not an estimate of reliability across unfamiliar rooms or Pi-camera conditions.

CPU prediction on the laptop successfully loaded this checkpoint, but detected
zero outlets at confidence 0.25 in both `outlet-focus-test.jpg` and the supplied
`outlet-test.jpg` from Downloads. Both photos are visibly blurry. The result
demonstrates a real-world miss despite the strong small-set validation metrics.
No model tuning was performed against these test photos. Pi-side runtime and
latency still need to be checked on the actual device.

Three platform images were resized to 3072 x 4096; their local originals are
4284 x 5712. Both have the same aspect ratio, and the normalized bounding boxes
were visually checked against the local images. No image downloads are needed.
The sanitized export in `datasets/imports/electricaloutlets.ndjson` retains
annotations and split assignments without expiring signed image URLs.

## Train on Windows

Open PowerShell in the Rover project folder:

```powershell
.\env\Scripts\python.exe train_outlets.py --check-only
.\env\Scripts\python.exe train_outlets.py --epochs 50
```

This fine-tunes pretrained YOLO26n at input size 640 using batch size 4, seed 42,
and early stopping patience 15. It selects CUDA if available, otherwise CPU.
The initial run saves its best checkpoint to
`runs/outlet_detection/baseline/weights/best.pt`. Repeated runs use a numbered
directory; use the path printed at completion. The initial `yolo26n.pt` is the
generic pretrained model, not the trained outlet checkpoint.

The six-image validation set is too small for a robust accuracy claim and
includes multiple views of the same outlet. Keep a separate set of Pi-camera
images for testing; do not train on these examples when measuring generalization.

## Copy to the Pi

In the Pi SSH terminal:

```bash
mkdir -p ~/rover/models
```

In Windows PowerShell in the Rover folder, replace `PI_ADDRESS` with your Pi's
hostname or IP. The Pi username observed in the installation log is `oscar`.

```powershell
scp .\runs\outlet_detection\baseline\weights\best.pt oscar@PI_ADDRESS:/home/oscar/rover/models/best.pt
scp .\predict_outlet.py oscar@PI_ADDRESS:/home/oscar/rover/predict_outlet.py
```

Only the trained checkpoint and prediction script are needed, not the training
dataset, dataset YAML, NDJSON, or Windows virtual environment.

## Predict on an existing Pi photo

In the Pi SSH terminal, substitute your actual photo path:

```bash
cd ~/rover
source .venv/bin/activate
python predict_outlet.py /home/oscar/outlet-focus-test.jpg
```

The script uses CPU inference and saves an annotated photo without opening a
preview window. It prints the result folder, detection count and confidences.
The input photo is not modified and needs no bounding-box labels for prediction.

For the first prediction run, copy the result back from Windows PowerShell:

```powershell
scp oscar@PI_ADDRESS:/home/oscar/rover/results/outlet_test/outlet-focus-test.jpg "$env:USERPROFILE\Downloads\outlet-prediction.jpg"
```

Use the printed output folder for subsequent runs (which receive a number).
Laptop inference timing is not a Raspberry Pi performance measurement.
