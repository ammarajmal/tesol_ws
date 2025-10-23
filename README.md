# tesol

## Displacement analysis utilities

The `analysis/compare_displacements.py` script compares the x-axis displacement
measurements produced by multiple cameras with a reference Laser Displacement
Sensor (LDS) signal. The tool resamples every input onto a common time base,
aggregates the camera measurements, and computes accuracy metrics in both the
time and frequency domains.

### Requirements

* Python 3.8 or newer
* NumPy (`pip install numpy`)
* Matplotlib (optional, for plotting: `pip install matplotlib`)

### Usage

```bash
python3 analysis/compare_displacements.py \
    --camera-files cam1.csv cam2.csv cam3.csv \
    --lds-file lds.csv \
    --time-column stamp \
    --value-column x \
    --lds-time-column stamp \
    --lds-value-column displacement \
    --time-scale 1e-9 \
    --output-dir results --plot --save-resampled
```

Arguments of interest:

* `--time-scale` and `--lds-time-scale` convert timestamps to seconds (handy for
  ROS bag exports recorded in nanoseconds).
* `--resample-rate` overrides the automatically detected sample rate.
* `--aggregate` controls how the per-camera signals are combined (`mean` or
  `median`).
* `--output-dir` stores JSON summaries, optional plots, and the resampled data
  when `--save-resampled` is provided.

The script prints a JSON summary that includes mean absolute error (MAE), root
mean square error (RMSE), bias, peak error, correlation coefficients, and
frequency-domain comparisons such as dominant frequency differences.
