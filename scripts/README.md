# Kite_ROS
This project aims to provide open source software to for ar robot controlled flying traction kite utilising python, ROS and OpenCV.  The kite will initially be tethered to a wooden frame with two line kite hooked up to an
arm with a central bearing to allow control.  The navigation approach will be to do this the way people fly kites
ie by seeing where the kite is and steering as desired

The main documentation for the project, the plan and the decision logs for this are being setup
at: http://www.netdecisionmaking.com/kite_ros

We are still setting this up and will add documentation and hopefully a few pictures as we go.

The initial steps are to get a webcam hooked up to see where the kite is and use open cv to detect the kite, the
direction of travel and the current angle of the kite body.  This information and analysis will be published as two
ROS messages one with the position and angles of the kite and another image message of the completed frames with
annotation included.

Separately we will collect information from the variable resistors one rotary will establish the angle of the arm and
two others which are linear will at least confirm whether the lines are taut or slack - depending on how well these can
be sprung and tensioned we may be able to get a more detailed measure of the force on each string .

The desired flight path and flight mode will also be available via ROS and this may be configurable either within the
motion detection module or separately.  It seems the above would be sufficient for data collection and we then look
to manually fly the kite and collect a bunch of data into the Ros bag file format.  We will then look at both
traditional and machine learning based approaches to setting the desired angle of the rotating arm,

We still need to figure out the best method of getting motor control over the arm.  There are some challenges
here as it seems we may require rather powerful servo(s) as there can be very strong forces involved particularly if
flying in strong winds.  Potentially some sort of ratchet arrangement may help protect the servo and reduce the work
requirements - however this would also bring additional complexity to the solution.
