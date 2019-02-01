def deg2rad(deg):
    return deg * 3.1415 / 180

DEBUG = True

CTE_COEFF = 100
EPSI_COEF = 100
SPEED_COEFF = 2
ACC_COEFF = 2
STEER_COEFF = 2

CONSEC_ACC_COEFF = 5
CONSEC_STEER_COEFF = 5

Lf = 0.325  # TODO(MD): consult with Lukasz
THROTTLE_LOWER_BOUND = 5  # TODO(MD): consult with Lukasz
THROTTLE_UPPER_BOUND = 15  # TODO(MD): consult with Lukasz
STEER_BOUND = deg2rad(25)  # TODO(MD): consult with Lukasz

TARGET_SPEED = 1

# When fitting a degree=3 polynomial to the waypoints we're using
# (STEPS_POLY * 3) points ahead to fit it (impacts smoothness)
STEPS_POLY = 15

# For the optimizer
STEPS_AHEAD = 20
TIME_STEP = 0.1
TOLERANCE = 0.1
# Higher tolerance means that the minimization ends after
#  fewer iterations (and it has a higher chance of converging)
#  BUT the of the cost function can be sub-optimal, and thus
#  the actuators that correspond to that cost function
