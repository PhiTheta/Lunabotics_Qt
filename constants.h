#ifndef CONSTANTS_H
#define CONSTANTS_H

#define CONN_INCOMING_ADDR  "192.168.218.1"
#define CONN_OUTGOING_ADDR  "192.168.218.138"

#define PID_KP              "20"
#define PID_KI              "0.1"
#define PID_KD              "1.5"
#define FEEDBACK_MIN_OFFSET "0.05"
#define FEEDBACK_MULTIPLIER "0.25"
#define FEEDFORWARD_MIN_OFFSET "0.05"
#define FEEDFORWARD_FRACTION "0.5"


#define LOCAL_PORT  44326
#define REMOTE_PORT 49203

#define DEFAULT_LINEAR_SPEED_LIMIT  0.2
#define DEFAULT_BEZIER_SEGMENTS 20
#define DEFAULT_ANGLE_ACCURACY 0.05
#define DEFAULT_DISTANCE_ACCURACY 0.25

#define OCCUPANCY_THRESHOLD     80



//Settings names
#define SETTINGS_VELOCITY   "steering.v"
#define SETTINGS_BEZIER     "steering.bezier"
#define SETTINGS_ANGLE_ACC  "steering.angle"
#define SETTINGS_DIST_ACC   "steering.distance"
#define SETTINGS_IP         "ip"
#define SETTING_PID_P       "p"
#define SETTING_PID_I       "i"
#define SETTING_PID_D       "d"
#define SETTING_FEEDBACK_MIN_OFFSET "feedback.min_offset"
#define SETTING_FEEDBACK_MULTIPLIER "feedback.multiplier"
#define SETTING_FEEDFORWARD_MIN_OFFSET "feedforward.min_offset"
#define SETTING_FEEDFORWARD_FRACTION "feedforward.fraction"


#endif // CONSTANTS_H
