STEPS_AHEAD=20
DT=0.1

LATENCY=0.01

CTE_COEFF=100
EPSI_COEF=100

SPEED_COEFF= 2
ACC_COEFF=2
STEER_COEFF=2

CONSEC_ACC_COEFF=5
CONSEC_STEER_COEFF=5

rosrun mpc mpc_node_cpp \
    $STEPS_AHEAD \
    $DT \
    $LATENCY \
    $CTE_COEFF \
    $EPSI_COEF \
    $SPEED_COEFF \
    $ACC_COEFF \
    $STEER_COEFF \
    $CONSEC_ACC_COEFF \
    $CONSEC_STEER_COEFF