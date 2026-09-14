# QP/GOP Tuning Notes

## Final Values

| Use Case | QP | GOP | Why |
|---|---:|---:|---|
| 720p60 | 39 | 8 | Keeps bandwidth low while preserving high browser FPS |
| 1080 Wi-Fi | 30 | 5 | Best visual quality target after fixed-QP delta sync |
| 1080 LTE around 1 Mbps | 32 | 15 | Stable measured result with zero loss and zero queue drop |

## Why H.264 Matters Here

At 1080p over a 1 Mbps class link, MJPEG-style frame-by-frame compression would
usually collapse into very low FPS or very poor image quality. H.264 survives
because most frames are P-frames and only encode motion/residuals between IDR
frames.

## Fixed-QP Delta Sync

The encoder may be created with an initial QP, then switched live to another QP.
If the hardware QP changes but the slice header still signals the old QP, the
decoder reconstructs with the wrong quantization assumption. This showed up as
dark moving soot-like artifacts.

The fix:

```text
decoder SliceQPY = PPS pic_init_qp + slice_qp_delta
slice_qp_delta   = live_fixed_qp - qp_init
hardware QP      = live_fixed_qp
```

This allows live QP switching without reopening the encoder.
