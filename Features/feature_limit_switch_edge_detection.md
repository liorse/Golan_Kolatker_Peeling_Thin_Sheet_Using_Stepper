there is a problem with the limit switches.

the problems description is as follow:

when the instrument returns home and the home limit switch is turned on. it remains turned until the motor moves out of home. but the motor can't move because it is on. 

the solution is to detect a rising or a falling edge, whatever is needed and not query the current limit switch position.

FIXED: replaced level-detection with falling-edge + 2 ms debounce for the safety abort
in MOVING / MOVING_TO_START / PEELING states. The fix applies symmetrically to both the
home (X) and far-end (Y) limit switches.

Implementation: `limitXPrev`/`limitYPrev` track the switch state from the previous loop
iteration; `limitXStableAt`/`limitYStableAt` record the timestamp of the falling edge.
The abort fires only when the switch has been continuously LOW for ≥ 2 ms AND was HIGH
on the preceding iteration (`xNewPress` / `yNewPress` flags). Once fired the timer is
cleared so the abort does not re-trigger for the same press.

HOMING and CAL_HOMING states retain level-detection (no change): those states are only
entered when the motor is away from the home switch, so a LOW reading always means a
fresh contact.
