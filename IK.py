def IK(T,theta0):
    # Inverse Kinematics Function for Mark II robot, returns joint array [theta1,theta2,theta3,theta4]
    # theta0 must be row vector in degrees
    # returns two outputs, a theta vector and a random order marker.  If there is an issue, the first output
    # will be 1 instead of a theta vector.  If a new random thetaguess needed to be used, the second output will be switched to 1.
    import numpy as np
    import random
    from FK import FK
    from workspacetest import workspacetest
    
    def SKS(W):
        W_sks = np.mat([[0,-W[2],W[1],W[3]],[W[2],0,-W[0],W[4]],[-W[1],W[0],0,W[5]],[0,0,0,0]])
        return W_sks
    
    def MatrixLog(Matrix):
        R_ = np.mat([[Matrix[0,0],Matrix[0,1],Matrix[0,2]],[Matrix[1,0],Matrix[1,1],Matrix[1,2]],[Matrix[2,0],Matrix[2,1],Matrix[2,2]]])
        d_ = np.mat([[Matrix[0,3]],[Matrix[1,3]],[Matrix[2,3]]])
        
        if np.linalg.norm(R_ - np.identity(3)) < 0.0001:
            LogAnswer = np.mat([[0,0,0,d_[0,0]],[0,0,0,d_[1,0]],[0,0,0,d_[2,0]],[0,0,0,0]])
            print("Case 0")
        elif abs(np.trace(R_) + 1.) < 0.0001:
            theta_ = np.pi
            if abs(R_[1,1] + 1.) < 0.0001:
                w1_ = (1./np.sqrt(2*(1.+R_[2,2])))*(R_[0,2])
                w2_ = (1./np.sqrt(2*(1.+R_[2,2])))*(R_[1,2])
                w3_ = (1./np.sqrt(2*(1.+R_[2,2])))*(1.+R_[2,2])
                print("Case 1a")
            else:
                w1_ = (1./np.sqrt(2*(1.+R_[1,1])))*(R_[0,1])
                w2_ = (1./np.sqrt(2*(1.+R_[1,1])))*(1.+R_[1,1])
                w3_ = (1./np.sqrt(2*(1.+R_[1,1])))*(R_[2,1])
                print("Case 1b")
            wsks_ = np.mat([[0,-w3_,w2_],[w3_,0,-w1_],[-w2_,w1_,0]])
            omega_theta = wsks_*theta_
            over_theta_ = 1./theta_
            v_theta = ((over_theta_*np.identity(3))-((wsks_)*.5)+((over_theta_ - (.5/np.tan(theta_*.5)))*(wsks_@wsks_)))@(d_*theta_)
            LogAnswer = np.concatenate((omega_theta,v_theta),axis=1)
            bottomrow = np.array([[0,0,0,0]])
            LogAnswer = np.concatenate((LogAnswer,bottomrow),axis=0)
        else:
            theta_ = np.arccos((np.trace(R_)-1.)/2)
            wsks_ = (1./(2*np.sin(theta_)))*(R_ - R_.T)
            omega_theta = wsks_*theta_
            over_theta_ = 1./theta_
            v_theta = ((over_theta_*np.identity(3))-((wsks_)*.5)+((over_theta_ - (.5/np.tan(theta_*.5)))*(wsks_@wsks_)))@(d_*theta_)
            LogAnswer = np.concatenate((omega_theta,v_theta),axis=1)
            bottomrow = np.array([[0,0,0,0]])
            LogAnswer = np.concatenate((LogAnswer,bottomrow),axis=0)
            print("Case 2")
        return LogAnswer
    
    def MatrixExp(B_i,theta_i):
        w_i = np.mat([[B_i[0,0]],[B_i[1,0]],[B_i[2,0]]])
        v_i = np.mat([[B_i[3,0]],[B_i[4,0]],[B_i[5,0]]])
        w_i_sks = np.mat([[0,-w_i[2,0],w_i[1,0]],[w_i[2,0],0,-w_i[0,0]],[-w_i[1,0],w_i[0,0],0]])
        G_theta = (np.identity(3)*theta_i) + ((1.-np.cos(theta_i))*(w_i_sks)) + ((theta_i - np.sin(theta_i))*(w_i_sks@w_i_sks))
        exp_theta = np.identity(3) + (w_i_sks*np.sin(theta_i)) + ((w_i_sks@w_i_sks)*(1.-np.cos(theta_i)))
        MatrixAns = np.concatenate((exp_theta,(G_theta@v_i)),axis=1)
        bottomrow = np.array([[0,0,0,1]])
        MatrixAns = np.concatenate((MatrixAns,bottomrow),axis=0)
        return MatrixAns
    
    def AdJ(t):
        dR = (np.mat([[0,-t[2,3],t[1,3]],[t[2,3],0,-t[0,3]],[-t[1,3],t[0,3],0]]))@(np.mat([[t[0,0],t[0,1],t[0,2]],[t[1,0],t[1,1],t[1,2]],[t[2,0],t[2,1],t[2,2]]]))
        AdJreturn = np.mat([[t[0,0],t[0,1],t[0,2],0,0,0],[t[1,0],t[1,1],t[1,2],0,0,0],[t[2,0],t[2,1],t[2,2],0,0,0],[dR[0,0],dR[0,1],dR[0,2],t[0,0],t[0,1],t[0,2]],[dR[1,0],dR[1,1],dR[1,2],t[1,0],t[1,1],t[1,2]],[dR[2,0],dR[2,1],dR[2,2],t[2,0],t[2,1],t[2,2]]])
        return AdJreturn
    
    # Check for proper input T
    if workspacetest(T) == False:
        print("Configuration Not in Workspace")
        return 1,0
    
    # Arm Geometry from FK.py (in inches)
    H1 = 6.25
    H2 = 7.25
    H3 = 6
    
    # Joint screw axes in the world frame, using the same convention as in FK.py
    w1 = np.array([0,0,1])
    w2 = np.array([0,1,0])
    w3 = np.array([0,1,0])
    w4 = np.array([1,0,0])
    q1 = np.array([0,0,0])
    q2 = np.array([0,0,H1])
    q3 = np.array([0,0,H1+H2])
    q4 = np.array([0,0,0])
    v1 = np.cross(-w1,q1)
    v2 = np.cross(-w2,q2)
    v3 = np.cross(-w3,q3)
    v4 = np.cross(-w4,q4)
    S1 = np.concatenate((w1,v1),axis=None)
    S2 = np.concatenate((w2,v2),axis=None)
    S3 = np.concatenate((w3,v3),axis=None)
    S4 = np.concatenate((w4,v4),axis=None)
    
    # Transformation between the world and body frames when at home position (M)
    M = np.mat([[1,0,0,0],[0,1,0,0],[0,0,1,H1+H2+H3],[0,0,0,1]])
    
    # Bracketed form of the joint screw axes in the world frame
    S1_sks = SKS(S1)
    S2_sks = SKS(S2)
    S3_sks = SKS(S3)
    S4_sks = SKS(S4)
    
    # Bracketed form
    Minv = np.linalg.inv(M)
    B1_sks = Minv@(S1_sks@M)
    B2_sks = Minv@(S2_sks@M)
    B3_sks = Minv@(S3_sks@M)
    B4_sks = Minv@(S4_sks@M)
    
    # Vector form of the screw axes in the body frame
    B1 = np.mat([[B1_sks[2,1]],[B1_sks[0,2]],[B1_sks[1,0]],[B1_sks[0,3]],[B1_sks[1,3]],[B1_sks[2,3]]])
    B2 = np.mat([[B2_sks[2,1]],[B2_sks[0,2]],[B2_sks[1,0]],[B2_sks[0,3]],[B2_sks[1,3]],[B2_sks[2,3]]])
    B3 = np.mat([[B3_sks[2,1]],[B3_sks[0,2]],[B3_sks[1,0]],[B3_sks[0,3]],[B3_sks[1,3]],[B3_sks[2,3]]])
    B4 = np.mat([[B4_sks[2,1]],[B4_sks[0,2]],[B4_sks[1,0]],[B4_sks[0,3]],[B4_sks[1,3]],[B4_sks[2,3]]])
    
    # Now we iterate
    
    # Initialization
    i = 0
    theta_new = np.mat([[theta0[0]],[theta0[1]],[theta0[2]],[theta0[3]]])
    Ew = .01
    Ev = .01
    
    # Set [V_b] = log(T^{-1}_{ab}(theta^i)T_{ab}
    Tsb = FK(theta_new)
    theta_new = theta_new*(np.pi/180)
    if Tsb is None:
        print("Initial Theta Guess is Outside Joint Limits")
        return 1,0
    V_b_sks = MatrixLog((np.linalg.inv(Tsb))@T)
    wb = np.mat([[V_b_sks[2,1]],[V_b_sks[0,2]],[V_b_sks[1,0]]])
    vb = np.mat([[V_b_sks[0,3]],[V_b_sks[1,3]],[V_b_sks[2,3]]])
    
    while np.linalg.norm(wb) > Ew or np.linalg.norm(vb) > Ev:
        
        # 1. Calculate exp([B]theta) for each of the body screw axes and the current theta
        expB1 = MatrixExp(B1,theta_new[0,0])
        expB2 = MatrixExp(B2,theta_new[1,0])
        expB3 = MatrixExp(B3,theta_new[2,0])
        expB4 = MatrixExp(B4,theta_new[3,0])
        
        # 2. Calculate the columns of the Jacobian using the above values and the body screw axes
        Jb4 = B4
        buffer = np.linalg.inv(expB4)
        Jb3 = (AdJ(buffer))@B3
        buffer = buffer@(np.linalg.inv(expB3))
        Jb2 = (AdJ(buffer))@B2
        buffer = buffer@(np.linalg.inv(expB2))
        Jb1 = (AdJ(buffer))@B1
        
        # 3. Calculate the forward kinematics as a function of the current angles. using the body form of the product of exponentials formula
        TS4 = M@(expB1@(expB2@(expB3@(expB4))))
        T4S = np.linalg.inv(TS4)
        
        # 4. Calculate T_{bd}, the desired end-effector position in the body frame, using above values
        Tbd = T4S@T
        
        # 5. Calculate the matrix logarithm of T_{bd}, using the algorithm given in the book for doing so
        Tbd_log = MatrixLog(Tbd)
        
        # 6. Set [V_b] using the above result
        V_b_sks = Tbd_log
        
        # 7. Convert [V_b] to V_b
        wb = np.mat([V_b_sks[2,1],V_b_sks[0,2],V_b_sks[1,0]])
        vb = np.mat([V_b_sks[0,3],V_b_sks[1,3],V_b_sks[2,3]])
        V_b = np.mat([[wb[0,0]],[wb[0,1]],[wb[0,2]],[vb[0,0]],[vb[0,1]],[vb[0,2]]])
        
        # 8. Update the current guess for the angles, using the above values
        Jb = np.column_stack((Jb1,Jb2,Jb3,Jb4))
        theta_new = theta_new + ((np.linalg.pinv(Jb))@V_b)
        # Iterate
        i = i + 1
        #print(i - 1)
        #print(theta_new*(180/np.pi))
    
    # Return the angles after the iteration has converged
    if any(abs(theta_new)) >= 2*np.pi:
        theta_new = np.mod(theta_new,2*np.pi)
    # Check Joint Limits
    randomchecker = 0
    Joint1Out = (theta_new[0,0] > 135*(np.pi/180) or theta_new[0,0] < -135*(np.pi/180))
    Joint2Out = (theta_new[1,0] > 105*(np.pi/180) or theta_new[1,0] < -105*(np.pi/180))
    Joint3Out = (theta_new[2,0] > 115*(np.pi/180) or theta_new[2,0] < -115*(np.pi/180))
    Joint4Out = (theta_new[3,0] > 90*(np.pi/180) or theta_new[3,0] < -90*(np.pi/180))
    if Joint1Out or Joint2Out or Joint3Out or Joint4Out:
        thetaguess = np.array([(270*(random.random()))-135,(210*(random.random()))-105,(230*(random.random()))-115,(180*(random.random()))-90]) # in degrees
        theta_new, randomchecker = IK(T,thetaguess)
        randomchecker = 1
        return theta_new,randomchecker
    
    
    thetas = (theta_new)*(180/np.pi) # Return in degrees
    return thetas,randomchecker