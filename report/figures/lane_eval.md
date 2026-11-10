# Offline lane evaluation

Exclude |true offset| > 1.5 m. MAEs use detected frames. Both sign metrics use |true offset| > 0.2 m; missed detections count as wrong offset signs. Steering includes loss decay. Controller resets per town/drive; retained frames run in tick order. Overall pools frames.

| Split | Town | N | Detection | Offset MAE (m) | Heading MAE (deg) | Offset sign | Steering sign |
|---|---|---:|---:|---:|---:|---:|---:|
| tune | Town01 | 186 | 82.8% | 0.345 | 23.79 | 56.7% | 82.3% |
| tune | Town02_Opt | 186 | 84.9% | 0.190 | 2.84 | 70.5% | 79.5% |
| tune | Town03_Opt | 186 | 86.0% | 0.154 | 2.54 | 83.5% | 85.2% |
| tune | Town04 | 186 | 100.0% | 0.110 | 1.67 | 100.0% | 97.4% |
| tune | Town05_Opt | 186 | 83.9% | 0.126 | 2.33 | 82.3% | 88.7% |
| tune | Town10HD | 186 | 72.6% | 0.591 | 7.94 | 43.1% | 61.0% |
| tune | Overall | 1116 | 85.0% | 0.240 | 6.60 | 71.9% | 82.2% |

| Split | Town | N | Detection | Offset MAE (m) | Heading MAE (deg) | Offset sign | Steering sign |
|---|---|---:|---:|---:|---:|---:|---:|
| test | Town01 | 93 | 92.5% | 0.350 | 123.73 | 34.5% | 38.2% |
| test | Town02_Opt | 93 | 88.2% | 0.190 | 2.56 | 75.0% | 81.7% |
| test | Town03_Opt | 93 | 95.7% | 0.138 | 2.63 | 91.1% | 83.9% |
| test | Town04 | 93 | 90.3% | 0.251 | 4.36 | 81.2% | 97.1% |
| test | Town05_Opt | 93 | 90.3% | 0.147 | 0.84 | 81.8% | 80.3% |
| test | Town10HD | 93 | 90.3% | 0.517 | 9.50 | 36.5% | 52.4% |
| test | Overall | 558 | 91.2% | 0.265 | 24.20 | 67.2% | 73.2% |

## Target assessment

Final drive-2 evaluation followed tuning only on drives 0?1; no CARLA was started.
Overall detection >=85% and offset MAE <=0.30 m pass. Offset sign >=85% and
steering sign >=90% fail. All towns pass detection. Town01 and Town10HD fail
MAE and both sign targets; Town02_Opt and Town05_Opt fail both sign targets;
Town03_Opt fails steering sign; Town04 fails offset sign. No town meets all four.
Town01 heading error remains unresolved (123.73 degrees). Tune overlays also
show junction cross-road edges surviving single-side fallback in Town10HD.

Changes: explicit pair vanishing-point/centre/40%-width checks, width-prior-led
pair ranking, median pixel-width learning over 31 accepted pairs per sequence,
stricter single-side fallback, and five-estimate median plus 0.15/update steering
rate limit after warmup. Heading authority is bounded relative to offset after
the first valid update. Startup response and public API are preserved.

Validation: 244 tests passed using `.venv/Scripts/python.exe -m pytest -q -p no:cacheprovider --basetemp=.test_tmp_lt2`.
