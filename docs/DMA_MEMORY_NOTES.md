# DMA Memory Notes

The camera pipeline is sensitive to the largest contiguous DMA-capable internal
SRAM block, not just total free memory.

## Practical Rule

Start the video pipeline first, wait until the first H.264 frame is produced,
then start audio. This avoids audio/I2S DMA allocations fragmenting the memory
needed by camera and H.264 buffers.

## Observed Behavior

- Starting audio too early can reduce the largest DMA block.
- Delaying audio until after first video frame avoids restart/reopen behavior.
- The UI exposes `DMA` as the current largest DMA block in KB.

## Future Improvement

Audio DMA buffer sizes can likely be reduced further, but the video-first start
order should remain. The target is not just low total memory use; the goal is to
preserve enough contiguous DMA space during reconnects, profile switches, and
camera calibration.
