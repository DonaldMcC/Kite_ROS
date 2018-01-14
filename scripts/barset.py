#  This module will cover setting the bar angle based on desired route
#   their will be a number of situations
#   1 park mode - aim is just to keep the kiteangle to zero so we adjust bar angle accordingly
#   2 wiggle mode - aim will I think be to get kiteangle to specified value and once there we change so
#     esentially a variation of 1 with targetangle not being zeor
#   3 xwind phase - here there are some options as to how best to proceed - if we have clear direction may use that
#     if no direction then we could calculate and use targetangle
#   4 turn phase - when starting we will have a maximum setting on the base and will just use that
#   5 once we are nearing end of turn we should switch to angle setting again
#     if kite not found we will need to look at best plan - if high then we would aim to park and there should be
#     a value of the base that has been calculated and lets us do this - if left or right then we should be in a
#     turn phase but recovery will be tricky - if low we probalby just need to go to base angle and hope for the best
#
#  so inputs will be phase, kitefound, position, targetangle, target point and we will somehow need to establish
#  when coming out of a turn - let's assume we don't need PID style control yet as wind is dynamic and there is no real
#  stability 