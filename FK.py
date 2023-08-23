def FK(positionvector):
    # Forward Kinematics Function for Mark II robot, returns end effector transformation matrix
    # Input can be 4x1 column vector or 1x4 row vector [theta1,theta2,theta3,theta4]
    # Input must be in degrees
    import numpy as np
    from workspacetest import workspacetest
    
    # helper functions
    def Rot(axis,angle):
        if axis == 'x':
            Rmatrix = np.mat([[1,0,0,0],[0,np.cos(angle),-np.sin(angle),0],[0,np.sin(angle),np.cos(angle),0],[0,0,0,1]])
        else:
            Rmatrix = np.mat([[np.cos(angle),-np.sin(angle),0,0],[np.sin(angle),np.cos(angle),0,0],[0,0,1,0],[0,0,0,1]])
        return Rmatrix
    
    def Trans(axis,distance):
        if axis == 'x':
            Tmatrix = np.mat([[1,0,0,distance],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
        else:
            Tmatrix = np.mat([[1,0,0,0],[0,1,0,0],[0,0,1,distance],[0,0,0,1]])
        return Tmatrix
    
    if isinstance(positionvector,list):
        j1 = positionvector[0]*(np.pi/180)
        j2 = positionvector[1]*(np.pi/180)
        j3 = positionvector[2]*(np.pi/180)
        j4 = positionvector[3]*(np.pi/180)
    elif positionvector.shape == (4,1):
        j1 = positionvector[0,0]*(np.pi/180)
        j2 = positionvector[1,0]*(np.pi/180)
        j3 = positionvector[2,0]*(np.pi/180)
        j4 = positionvector[3,0]*(np.pi/180)
    else:
        j1 = positionvector[0]*(np.pi/180)
        j2 = positionvector[1]*(np.pi/180)
        j3 = positionvector[2]*(np.pi/180)
        j4 = positionvector[3]*(np.pi/180)

    # Check Joint Limits
    if j1 > 135*(np.pi/180) or j1 < -135*(np.pi/180):
        return
    if j2 > 105*(np.pi/180) or j2 < -105*(np.pi/180):
        return
    if j3 > 115*(np.pi/180) or j3 < -115*(np.pi/180):
        return
    if j4 > 90*(np.pi/180) or j4 < -90*(np.pi/180):
        return
    
    # Arm Geometry (in inches)
    H1 = 6.25 # Get actual values from SolidWorks
    H2 = 7.25
    H3 = 6
    
    phi = np.array([j1+(np.pi),j2+(np.pi/2),j3+(np.pi/2),j4])
    d = np.array([H1,0,0,H3])
    a = np.array([0,H2,0,0])
    alpha = np.array([(np.pi/2),0,(np.pi/2),0])
    
    T01 = Rot('z',phi[0])@(Trans('z',d[0])@(Trans('x',a[0])@Rot('x',alpha[0])))
    T12 = Rot('z',phi[1])@(Trans('z',d[1])@(Trans('x',a[1])@Rot('x',alpha[1])))
    T23 = Rot('z',phi[2])@(Trans('z',d[2])@(Trans('x',a[2])@Rot('x',alpha[2])))
    T34 = Rot('z',phi[3])@(Trans('z',d[3])@(Trans('x',a[3])@Rot('x',alpha[3])))
    
    T04 = T01@(T12@(T23@T34))
    
    if workspacetest(T04) == False:
        print("WARNING: Configuration is Outside of Workspace")
    
    return T04