Header header
int64[] ids
float32[] data
float32[] distances

# int64[] ids consists of combined tracker and trajectory ids
# to decompose do:
# trajectory_id = ids[i] >> 32
# tracker_id = ids[i] % 32