
genprotos() {
        echo "Generating $1"
        protoc -I=$2 --cpp_out=$3 $2/$1
}

SRC_DIR=.
DST_DIR=.

genprotos "Point.proto" $SRC_DIR $DST_DIR
genprotos "Twist.proto" $SRC_DIR $DST_DIR
genprotos "SteeringModeType.proto" $SRC_DIR $DST_DIR
genprotos "Telecommand.proto" $SRC_DIR $DST_DIR
genprotos "Telemetry.proto" $SRC_DIR $DST_DIR
genprotos "AllWheelState.proto" $SRC_DIR $DST_DIR
genprotos "AllWheelControl.proto" $SRC_DIR $DST_DIR
