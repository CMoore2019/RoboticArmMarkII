def catapult(startpos,landpos):
    # Inputs: np.array([startpos_x,startpos_y]), np.array([landposx,landposy])
    import numpy as np
    import time
    from IK import IK
    from FK import FK
    from MoveTo import MoveTo
    from workspacetest import workspacetest
    from FindBasisVectors import FindBasisVectors
    from FindFastAngles import FindFastAngles
    from MoveSlowly import MoveSlowly
    
    H1 = 6.25
    H2 = 7.25
    H3 = 6
    g = 9.81*39.37 # in/sec**2
    
    # Do the launch trajectory math
    
    landpos_x = landpos[0]
    landpos_y = landpos[1]
    if landpos_x == 0:
        if landpos_y >= 0:
            theta1 = np.pi/2
        else:
            theta1 = -np.pi/2
    elif landpos_x > 0:
        theta1 = np.arctan(landpos_y/landpos_x)
    else:
        theta1 = np.arctan(landpos_y/landpos_x)+np.pi
    theta1 = theta1*(180/np.pi)
    theta2 = -90
    theta2b = -45
    if theta1 > 135 or theta1 < -135:
        theta2 = -theta2
        theta2b = -theta2b
        if theta1 > 135:
            theta1 = theta1 - 180
        else:
            theta1 = theta1 + 180
    
    distance = np.linalg.norm(landpos)
    initialheight = H1+((H2+H3)/np.sqrt(2))
    initialdistance = -((H2+H3)/np.sqrt(2))
    initialvelocity = np.sqrt((g*(distance-initialdistance)**2)/(initialheight+distance-initialdistance))
    omega = initialvelocity/(H2+H3)
    throwtime = (np.pi/4)/omega
    
    # Move the robot
    home = np.array([0,0,0,0])
    MoveTo(home,0)
    #xv,yv,zv = FindBasisVectors(startpos[0],startpos[1],4) #2.25
    #if xv is None:
    #    print("Start Position Outside Range")
    #    return
    #Tstart = np.mat([[xv[0],yv[0],zv[0],startpos[0]],[xv[1],yv[1],zv[1],startpos[1]],[xv[2],yv[2],zv[2],4],[0,0,0,1]]) #2.25

    thetastart = FindFastAngles(startpos[0],startpos[1],4.5)
    MoveSlowly(home,0,thetastart,150,2)
    MoveSlowly(thetastart,150,thetastart,97,1)
    thetahover = FindFastAngles(startpos[0],startpos[1],5.5)
    MoveSlowly(thetastart,97,thetahover,97,1)
    thetaforlaunch = np.array([theta1,theta2,0,thetahover[3]])
    MoveSlowly(thetahover,97,thetaforlaunch,97,3)
    MoveSlowly(thetaforlaunch,97,np.array([theta1,theta2b,theta2,thetahover[3]]),97,2)
    time.sleep(3)
    MoveTo(np.array([theta1,0,0,thetahover[3]]),97)
    time.sleep(.1)
    MoveTo(np.array([theta1,-theta2b,-theta2/2,thetahover[3]]),99)
    #thetarelease = np.array([theta1,theta2b,0,thetahover[3]])
    #MoveSlowly(thetaforlaunch,97,thetarelease,110,throwtime)
    
    #xv1,yv1,zv1 = FindBasisVectors(startpos[0],startpos[1],5)
    #Thover = np.mat([[xv1[0],yv1[0],zv1[0],startpos[0]],[xv1[1],yv1[1],zv1[2],startpos[1]],[xv1[2],yv1[2],zv1[2],5],[0,0,0,1]])
    #hoveroverstart,randomcheck1 = IK(Thover,thetastart)
    #if randomcheck1 == 1:
    #    print("Path Issue Detected")
    #     MoveTo(thetastart,150)
    #    MoveTo(home,150)
    return    