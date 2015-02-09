# This message assigns a list trajectory ids for every object
# 'len(offsets)-1' is the number of objects
# 'ids' are stored in continues order
# to access all ids of the ith object, iterate as:
# for(j=offsets[i]; j<offsets[i+1]; ++j)
#   do_something( ids[j] )

Header header
int32[] offsets
int32[] labels
int64[] ids 

# int64[] ids consists of combined tracker and trajectory ids
# to decompose do:
# trajectory_id = ids[i] >> 32
# tracker_id = ids[i] % 32
