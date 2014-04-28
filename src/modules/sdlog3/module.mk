# This is James' hack of a new data logging app.
#
#

#
# sdlog3 Application
#

MODULE_COMMAND  = sdlog3
# The main thread only buffers to RAM, needs a high priority
MODULE_PRIORITY = "SCHED_PRIORITY_MAX-30"

SRCS = sdlog3.c \
       logbuffer.c
