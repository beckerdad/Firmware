

#
# Attitude estimator (Madgwick gradient descent)
#

MODULE_COMMAND	 = attitude_estimator_madgwick

SRCS		 = attitude_estimator_madgwick_main.cpp \
		   attitude_estimator_madgwick_params.c \
		   madgwick.c \
		   quaternion.c \
		   codegen/eye.c \
		   codegen/attitudeKalmanfilter.c \
		   codegen/mrdivide.c \
		   codegen/rdivide.c \
		   codegen/attitudeKalmanfilter_initialize.c \
		   codegen/attitudeKalmanfilter_terminate.c \
		   codegen/rt_nonfinite.c \
		   codegen/rtGetInf.c \
		   codegen/rtGetNaN.c \
		   codegen/norm.c \
		   codegen/cross.c
