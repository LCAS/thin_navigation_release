
include "hokuyo.inc"

define crobot position(
    size [0.25 0.25 0.25]
    origin [0 0 0 0]
    gui_nose 1
    drive "diff"
#size [0.20 0.20 0.20]
    
    
    hokuyo_utm30( pose [0.05 0 -0.1 0] )

    # Report error-free position in world coordinates
    #localization "gps"
    #localization_origin [ 0 0 0 0 ]

    # Some more realistic localization error
    localization "odom"
    odom_error [ 0.0 0.0 0.0 0.1 ]

)
