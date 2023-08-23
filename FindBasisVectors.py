def FindBasisVectors(x,y,z):
    import numpy as np
    from FK import FK
    
    H1 = 6.25
    H2 = 7.25
    H3 = 6
    # Find Z vector
    array2center = np.array([x,y,z])-np.array([0,0,H1])
    thirdside = np.linalg.norm(array2center)
    if thirdside > H2+H3:
        print("Error. Move projectile closer.")
        return
    elbowangle = np.pi - np.arccos((H2**2 + H3**2 - thirdside**2)/(2*H2*H3))
    shouldersmallangle = np.arccos((thirdside**2 + H2**2 - H3**2)/(2*thirdside*H2))
    
    array2peak = np.array([x,y,z])-np.array([0,0,H1+thirdside])
    longside = np.linalg.norm(array2peak)
    shoulderbigangle = np.arccos((thirdside**2 + thirdside**2 - longside**2)/(2*(thirdside**2)))
    shoulderangle = shoulderbigangle-shouldersmallangle
    wristangle = np.pi/2
    
    if x == 0:
        if y >= 0:
            baseangle = np.pi/2
        else:
            baseangle = -np.pi/2
    elif x > 0:
        baseangle = np.arctan(y/x)
    else:
        baseangle = np.arctan(y/x)+np.pi
    
    if baseangle > 135*(np.pi/180):
        baseangle = baseangle - np.pi
        shoulderangle = -shoulderangle
        elbowangle = -elbowangle
        wristangle = -wristangle
        
    elif baseangle < -135*(np.pi/180):
        baseangle = baseangle + np.pi
        shoulderangle = -shoulderangle
        elbowangle = -elbowangle
        wristangle = -wristangle
    
    joints = (np.array([baseangle,shoulderangle,elbowangle,wristangle]))*(180/np.pi)
    Tout = FK(joints)
    print(joints)
    
    xv = np.array([Tout[0,0],Tout[1,0],Tout[2,0]])
    yv = np.array([Tout[0,1],Tout[1,1],Tout[2,1]])
    zv = np.array([Tout[0,2],Tout[1,2],Tout[2,2]])
    return xv,yv,zv