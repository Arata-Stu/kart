# Stream indices also select bits in seen_mask.
uint8 LEFT_IMAGE=0
uint8 RIGHT_IMAGE=1
uint8 LEFT_INFO=2
uint8 RIGHT_INFO=3
uint8 stream
int64 stamp_ns
uint64 sequence
int64 interval_ns
uint64 expected_frames_missing
uint8 seen_mask
