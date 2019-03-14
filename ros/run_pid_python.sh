DEBUG=True

Kp_cte=1.0
Ki_cte=0.0
Kd_cte=5.0

Kp_ePsi=1.0
Ki_ePsi=0.0
Kd_ePsi=5.0

Lf=0.325

POLY_DEGREE=3
POLY_STEPS=30

rosrun pid pid_in_python.py \
    $DEBUG \
    $Kp_cte \
    $Ki_cte \
    $Kd_cte \
    $Kp_ePsi \
    $Ki_ePsi \
    $Kd_ePsi \
    $Lf \
    $POLY_DEGREE \
    $POLY_STEPS
