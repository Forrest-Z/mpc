STEPS_AHEAD=20
DT=0.1
LATENCY=0.01

rosrun mpc mpc_node_cpp \
    $STEPS_AHEAD \
    $DT \
    $LATENCY
