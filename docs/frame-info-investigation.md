# Investigation: `FrameNr` from `pl_exp_get_oldest_frame_ex` and false dropped frames

## Summary

In continuous acquisition with a `CIRC_NO_OVERWRITE` circular buffer, the `FRAME_INFO`
returned by `pl_exp_get_oldest_frame_ex` describes the **newest** frame that has arrived in
the buffer, not the oldest frame whose pointer is returned. The frame *data* pointer is
correct (oldest frame, in order); only the frame *info* is wrong.

Because the driver originally detected dropped frames from gaps in that `FrameNr`, it reported
drops whenever it fell behind by more than one frame, even though no frames were lost. The
driver now takes `FrameNr` from the EOF callback instead, which fires once per frame, in order.

## Symptoms

1. **Externally triggered run:** 18010 hardware triggers were sent at 80 Hz and exactly 18010
   frames were received, yet `DroppedFrames_RBV` reported a non-zero count.
2. **Continuous run:** `DroppedFrames_RBV` rose steadily, by 1 at the same timestamp as each
   `PendingFrames_RBV` value of 1:

   ```
   cam1:DroppedFrames_RBV 2026-10-16 15:42:21.437010 132
   cam1:DroppedFrames_RBV 2026-10-16 15:42:21.460507 133
   cam1:DroppedFrames_RBV 2026-10-16 15:42:21.484584 134
   ```

   ```
   cam1:PendingFrames_RBV 2026-10-16 15:42:21.437002 1
   cam1:PendingFrames_RBV 2026-10-16 15:42:21.439218 0
   cam1:PendingFrames_RBV 2026-10-16 15:42:21.460500 1
   cam1:PendingFrames_RBV 2026-10-16 15:42:21.462113 0
   cam1:PendingFrames_RBV 2026-10-16 15:42:21.484577 1
   cam1:PendingFrames_RBV 2026-10-16 15:42:21.486177 0
   ```

## Original drop-detection code

```cpp
if (PV_OK != pl_exp_get_oldest_frame_ex(this->cameraContext->hcam, &pFrame, pFrameInfo)) {
    ...
}

frameNr = pFrameInfo->FrameNr;
if (lastFrameNr >= 0 && pFrameInfo->FrameNr > lastFrameNr + 1) {
    int missed = pFrameInfo->FrameNr - lastFrameNr - 1;
    droppedFrames += missed;
    setIntegerParam(KTX_DroppedFrames, droppedFrames);
    WARN_ARGS("Dropped %d frame(s) between frame %d and frame %d", missed,
              lastFrameNr, pFrameInfo->FrameNr);
}
lastFrameNr = pFrameInfo->FrameNr;
```

## What the PVCam documentation says

From `pvcam.h`, `pl_exp_get_oldest_frame_ex`:

> Additionally, this function updates the values in the variable pointed to by `frame_info`
> with the data collected at the time of frame reception by the device driver (e.g. frame
> counter value).

That reads as if `FRAME_INFO` describes the returned (oldest) frame. The test below shows
otherwise.

## Test setup

The EOF callback was changed to queue each frame's `FrameNr` in arrival order, and the
acquisition thread pops one entry for each frame it reads:

```cpp
static void PV_DECL newFrameCallback(FRAME_INFO *pFrameInfo, CameraContext *ctx) {
    if (ctx == nullptr) return;
    {
        std::lock_guard<std::mutex> lock(ctx->eofEvent.mutex);
        ctx->pendingFrameNrs.push_back(pFrameInfo != nullptr ? pFrameInfo->FrameNr : -1);
    }
    ctx->eofEvent.cond.notify_all();
}
```

```cpp
// in waitForEofEvent()
*frameNr = this->cameraContext->pendingFrameNrs.front();
this->cameraContext->pendingFrameNrs.pop_front();
*pendingFrames = (uns32) this->cameraContext->pendingFrameNrs.size();
```

A temporary debug print was added after each successful `pl_exp_get_oldest_frame_ex` call.
It shows the callback `FrameNr` (`eof`), the `FrameNr` from `get_oldest_frame_ex`
(`oldest`), the number of frames still queued (`pending`), the `FRAME_INFO` timestamp, and
the index of the returned frame in the circular buffer (`slot`):

```cpp
printf("FRAMENR_DEBUG eof=%d oldest=%d pending=%u ts=%lld slot=%ld%s\n", (int) frameNr,
       (int) pFrameInfo->FrameNr, (unsigned) pendingFrames,
       (long long) pFrameInfo->TimeStamp,
       (long) (((uns8 *) pFrame - (uns8 *) this->frameBuffer) / frameBufferSize),
       (frameNr != pFrameInfo->FrameNr) ? " MISMATCH" : "");
```

## Results

### Run 1: `FrameNr` comparison

```
FRAMENR_DEBUG eof=4306 oldest=4315 pending=9 ts=1622820 MISMATCH
FRAMENR_DEBUG eof=4307 oldest=4315 pending=8 ts=1622820 MISMATCH
FRAMENR_DEBUG eof=4308 oldest=4315 pending=7 ts=1622820 MISMATCH
FRAMENR_DEBUG eof=4309 oldest=4316 pending=7 ts=1622950 MISMATCH
FRAMENR_DEBUG eof=4310 oldest=4316 pending=6 ts=1622950 MISMATCH
FRAMENR_DEBUG eof=4311 oldest=4317 pending=6 ts=1623070 MISMATCH
FRAMENR_DEBUG eof=4312 oldest=4317 pending=5 ts=1623070 MISMATCH
FRAMENR_DEBUG eof=4313 oldest=4318 pending=5 ts=1623200 MISMATCH
FRAMENR_DEBUG eof=4314 oldest=4318 pending=4 ts=1623200 MISMATCH
FRAMENR_DEBUG eof=4315 oldest=4318 pending=3 ts=1623200 MISMATCH
FRAMENR_DEBUG eof=4316 oldest=4319 pending=3 ts=1623320 MISMATCH
FRAMENR_DEBUG eof=4317 oldest=4319 pending=2 ts=1623320 MISMATCH
FRAMENR_DEBUG eof=4318 oldest=4319 pending=1 ts=1623320 MISMATCH
FRAMENR_DEBUG eof=4319 oldest=4320 pending=1 ts=1623450 MISMATCH
```

On every line, `oldest = eof + pending`, which is the most recent frame that has arrived:

| eof  | pending | eof + pending | oldest |
|------|---------|---------------|--------|
| 4306 | 9       | 4315          | 4315   |
| 4307 | 8       | 4315          | 4315   |
| 4309 | 7       | 4316          | 4316   |
| 4313 | 5       | 4318          | 4318   |
| 4319 | 1       | 4320          | 4320   |

The `FRAME_INFO` timestamp also repeats whenever `oldest` repeats, so the whole structure
belongs to the newest frame, not just the `FrameNr` field. The `eof` sequence is continuous
(4306 → 4319), so no frames were actually lost.

### Run 2: is the frame data also from the newest frame?

```
FRAMENR_DEBUG eof=2566 oldest=2566 pending=0 ts=985700 slot=15
FRAMENR_DEBUG eof=2567 oldest=2567 pending=0 ts=985830 slot=16
FRAMENR_DEBUG eof=2568 oldest=2573 pending=5 ts=986580 slot=17 MISMATCH
FRAMENR_DEBUG eof=2569 oldest=2573 pending=4 ts=986580 slot=18 MISMATCH
FRAMENR_DEBUG eof=2570 oldest=2574 pending=4 ts=986710 slot=19 MISMATCH
FRAMENR_DEBUG eof=2571 oldest=2574 pending=3 ts=986710 slot=20 MISMATCH
FRAMENR_DEBUG eof=2572 oldest=2575 pending=3 ts=986830 slot=21 MISMATCH
FRAMENR_DEBUG eof=2573 oldest=2575 pending=2 ts=986830 slot=22 MISMATCH
FRAMENR_DEBUG eof=2574 oldest=2576 pending=2 ts=986950 slot=23 MISMATCH
FRAMENR_DEBUG eof=2575 oldest=2576 pending=1 ts=986950 slot=24 MISMATCH
FRAMENR_DEBUG eof=2576 oldest=2576 pending=0 ts=986950 slot=25
FRAMENR_DEBUG eof=2577 oldest=2577 pending=0 ts=987080 slot=26
FRAMENR_DEBUG eof=2578 oldest=2578 pending=0 ts=987200 slot=27
FRAMENR_DEBUG eof=2579 oldest=2579 pending=0 ts=987330 slot=28
FRAMENR_DEBUG eof=2580 oldest=2580 pending=0 ts=987450 slot=29
FRAMENR_DEBUG eof=2581 oldest=2581 pending=0 ts=987580 slot=30
FRAMENR_DEBUG eof=2582 oldest=2582 pending=0 ts=987710 slot=31
FRAMENR_DEBUG eof=2583 oldest=2583 pending=0 ts=987830 slot=32
```

- `slot` increases by exactly one on every line, including the lines where `oldest` repeats.
  The returned pointer always moves to the next frame in the buffer.
- With a 50-frame buffer, `slot == (eof - 1) % 50` on every line (e.g. `eof=2568` →
  `2567 % 50 = 17`). The returned data is the frame identified by the callback `FrameNr`.
- When `pending == 0`, `eof == oldest`, so the problem only appears when the driver is more
  than one frame behind.

## Conclusion

| Item returned by `pl_exp_get_oldest_frame_ex` | Correct? |
|-----------------------------------------------|----------|
| Frame data pointer                            | Yes, oldest unretrieved frame, in order |
| `FRAME_INFO` (`FrameNr`, `TimeStamp`)         | No, describes the newest frame in the buffer |

Gaps in `FRAME_INFO.FrameNr` from `pl_exp_get_oldest_frame_ex` do not mean lost frames. With
`UniqueIdSource=Camera`, the old code would also have given consecutive arrays duplicate
`UniqueId` values whenever the driver fell behind.

## Resulting driver behaviour

- The EOF callback queues each frame's `FrameNr`, and the acquisition thread takes one per
  frame read. That value is used for both drop detection and `NDArray::uniqueId`.
- `PendingFrames_RBV` is the length of that queue.
- `DroppedFrames_RBV` counts gaps in the callback `FrameNr` sequence.

```cpp
// get_oldest_frame_ex fills FRAME_INFO for the newest frame, so use the EOF callback's FrameNr
if (lastFrameNr >= 0 && frameNr > lastFrameNr + 1) {
    int missed = frameNr - lastFrameNr - 1;
    droppedFrames += missed;
    setIntegerParam(KTX_DroppedFrames, droppedFrames);
    WARN_ARGS("Dropped %d frame(s) between frame %d and frame %d", missed,
              lastFrameNr, frameNr);
}
lastFrameNr = frameNr;
```

## Related note on `CIRC_NO_OVERWRITE`

Per `pvcam.h`, in `CIRC_NO_OVERWRITE` mode the oldest frame is still overwritten when the
buffer overruns; PVCam reports this as an error on the next `pl_exp_get_oldest_frame_ex`
call. Buffer overruns therefore show up as that error, not as `FrameNr` gaps, and they are
not currently counted in `DroppedFrames_RBV`.
