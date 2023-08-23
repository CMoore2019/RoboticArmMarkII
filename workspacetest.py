def workspacetest(T):
    import numpy as np
    # Checks for collisions and if the position and orientation of the end effector are attainable
    H1 = 6.25
    H2 = 7.25
    H3 = 6
    
    pointtocheck_x = T[0,3]-(H3*T[0,2])
    pointtocheck_y = T[1,3]-(H3*T[1,2])
    pointtocheck_z = T[2,3]-(H3*T[2,2])
    pointtocheck = np.array([pointtocheck_x,pointtocheck_y,pointtocheck_z])
    centersphereorigin = np.array([0,0,H1])
    vectorbetween = pointtocheck - centersphereorigin
    if abs((np.linalg.norm(vectorbetween))-H2) > 0.001:
        return False
    if T[2,3] < 0:
        return False
    clawrailextreme1_z = T[2,3]+(1.75*T[2,0])
    clawrailextreme2_z = T[2,3]-(1.75*T[2,0])
    if clawrailextreme1_z < 0.75 or clawrailextreme2_z < 0.75: #Claw rail hits ground while closed
        return False
    clawextreme_z = T[2,3]+(1.25*T[2,2])
    if clawextreme_z < 0: # Claw Pincher hits ground while closed
        return False
    return True