def MoveSlowly(anglestart,clawstart,angleend,clawend,totaltime):
    import numpy as np
    import time
    from MoveTo import MoveTo
    
    time_per_waypoint = .05
    number_of_waypoints = int(totaltime/time_per_waypoint)
    claw_matrix = np.zeros(number_of_waypoints)
    anglematrix = np.zeros((4,number_of_waypoints))
    
    clawdist = clawend - clawstart
    clawdist_per = clawdist/(number_of_waypoints - 1)
    for k in np.arange(number_of_waypoints):
        claw_matrix[k] = clawstart + (clawdist_per*k)
    
    for i in [0,1,2,3]:
        dist = angleend[i] - anglestart[i]
        dist_per = dist/(number_of_waypoints-1)
        for j in np.arange(number_of_waypoints):
            anglematrix[i,j] = anglestart[i] + (dist_per*j)
    
    for index in np.arange(number_of_waypoints):
        currentangle = anglematrix[:,index]
        j1 = currentangle[0]
        j2 = currentangle[1]
        j3 = currentangle[2]
        j4 = currentangle[3]
        claw = claw_matrix[index]
        MoveTo(np.array([j1,j2,j3,j4]),claw)
        time.sleep(time_per_waypoint)
    return