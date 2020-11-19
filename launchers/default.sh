#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------


# NOTE: Use the variable DT_REPO_PATH to know the absolute path to your code
# NOTE: Use `dt-exec COMMAND` to run the main process (blocking process)

# launching app

# decide which package to launch using environment variables input at runtime

if [ "$LAUNCH_PACKAGE" = "encoder_localization" ]
then
    echo "default.sh: Launching encoder_localization package!"
    dt-exec roslaunch encoder_localization encoder_localization.launch veh:="$VEHICLE_NAME"
elif [ "$LAUNCH_PACKAGE" = "at_localization" ]
then
    echo "default.sh: Launching at_localization package!"
    dt-exec roslaunch at_localization at_localization.launch veh:="$VEHICLE_NAME"
elif [ "$LAUNCH_PACKAGE" = "fused_localization" ]
then
    echo "default.sh: Launching fused_localization package!"
    dt-exec roslaunch fused_localization fused_localization.launch veh:="$VEHICLE_NAME"
else
    echo "default.sh: Error: Please input a valid package to launch as the value of LAUNCH_PACKAGE in the form 
    'dts devel run -- --env LAUNCH_PACKAGE=<package>'
where <package> is encoder_localization, at_localization or fused_localization.
You entered: $LAUNCH_PACKAGE"
    exit 0
fi

# ----------------------------------------------------------------------------
# YOUR CODE ABOVE THIS LINE

# wait for app to end
dt-launchfile-join
