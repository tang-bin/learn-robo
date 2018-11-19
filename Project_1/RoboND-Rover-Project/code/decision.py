import numpy as np


# This is where you can build a decision tree for determining throttle, brake and steer
# commands based on the output of the perception_step() function
def decision_step(Rover):

    # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!


    
    degs = Rover.nav_angles * 180/np.pi
    avg = np.mean(degs)

    
    rightThreshold = -30
    leftCount = np.sum(degs > rightThreshold)
    rightCount = np.sum(degs <= rightThreshold)

    if rightCount > 100 and leftCount > 4 * rightCount:
        rotation = -20
    else:
        if avg < 10 and avg > -10:
            avg = 0

        arr = [[], [], []]

        for deg in degs:
            if deg < -17:
                arr[2].append(deg)
            elif deg > 17:
                arr[0].append(deg)
            else:
                arr[1].append(deg)

        leftLen = len(arr[0])
        midLen = len(arr[1])
        rightLen = len(arr[2])

        if (rightLen > midLen * 2 and leftLen > midLen * 2) or (avg == 0 and rightLen > midLen and rightLen > leftLen):
            avg = np.mean(arr[2])

        rotation = np.clip(avg, -30, 30)

    move = np.sqrt((Rover.pos[0] - Rover.prevX) ** 2 + (Rover.pos[1] - Rover.prevY) ** 2)

    print("MOVE:", move)

    if move < 0.01:
        Rover.stuck += 1
    else:
        Rover.stuck = 0

    Rover.prevX = Rover.pos[0]
    Rover.prevY = Rover.pos[1]

    print("STUCK", Rover.stuck)
    if Rover.stuck > 100:
        if Rover.mode != "fallback":
            Rover.mode = "fallback"
            Rover.fallbackCount = 60

    if Rover.mode == "fallback":
        Rover.throttle = -1
        Rover.brake = 0
        Rover.steer = -20 * (80 - Rover.fallbackCount) / 80
        Rover.fallbackCount -= 1
        if Rover.fallbackCount <= 0:
            Rover.mode = "forward"
            Rover.stuck = 0
    # Example:
    # Check if we have vision data to make decisions with
    elif Rover.nav_angles is not None:
        # Check for Rover.mode status
        if Rover.mode == 'forward':
            # Check the extent of navigable terrain
            if len(Rover.nav_angles) >= Rover.stop_forward:
                # If mode is forward, navigable terrain looks good
                # and velocity is below max, then throttle
                if Rover.vel < Rover.max_vel:
                    # Set throttle value to throttle setting
                    Rover.throttle = Rover.throttle_set
                else:  # Else coast
                    Rover.throttle = 0
                Rover.brake = 0
                # Set steering to average angle clipped to the range +/- 15
                Rover.steer = rotation
            # If there's a lack of navigable terrain pixels then go to 'stop' mode
            elif len(Rover.nav_angles) < Rover.stop_forward:
                    # Set mode to "stop" and hit the brakes!
                Rover.throttle = 0
                # Set brake to stored brake value
                Rover.brake = Rover.brake_set
                Rover.steer = rotation
                Rover.mode = 'stop'

        # If we're already in "stop" mode then make different decisions
        elif Rover.mode == 'stop':
            # If we're in stop mode but still moving keep braking
            if Rover.vel > 0.2:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0
            # If we're not moving (vel < 0.2) then do something else
            elif Rover.vel <= 0.2:
                # Now we're stopped and we have vision data to see if there's a path forward
                if len(Rover.nav_angles) < Rover.go_forward:
                    Rover.throttle = 0
                    # Release the brake to allow turning
                    Rover.brake = 0
                    # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                    if Rover.turnDir is not None:
                        Rover.steer = -15 if Rover.turnDir == 'right' else 15
                    else:
                        Rover.steer = -15 if rotation < 0 else 15  # Could be more clever here about which way to turn
                        Rover.turnDir = "right" if Rover.steer < 0 else "left"
                # If we're stopped but see sufficient navigable terrain in front then go!
                if len(Rover.nav_angles) >= Rover.go_forward:
                    # Set throttle back to stored value
                    Rover.throttle = Rover.throttle_set
                    # Release the brake
                    Rover.brake = 0
                    # Set steer to mean angle
                    Rover.steer = rotation
                    Rover.mode = 'forward'
                    
                    Rover.turnDir = None
    # Just to make the rover do something
    # even if no modifications have been made to the code
    else:
        Rover.throttle = Rover.throttle_set
        Rover.steer = 0
        Rover.brake = 0

    # If in a state where want to pickup a rock send pickup command
    if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
        Rover.send_pickup = True

    return Rover
