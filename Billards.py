import numpy as np
from matplotlib import pyplot as plt, patches
from matplotlib.animation import FuncAnimation 

def main():
    # Initial Conditions
    r = 0.05
    
    # Test Case 1:
    print("Test Case 1: ")
    xR, yR, uR, vR = 0.75, 5*r, -0.1, 0.5  # Red Ball Initial Values
    xB, yB, uB, vB = 0.25, 5.5*r, 0.11, 0.2  # Blue Ball Initial Values
    run_simulation_test(r, xR, yR, uR, vR, xB, yB, uB, vB)
    
    # Test Case 2:
    print("Test Case 2: ")
    xR, yR, uR, vR = 0.75, 0.5, -0.5, 0  # Red Ball Initial Values
    xB, yB, uB, vB = 0.25, 0.5, 0.5, 0  # Blue Ball Initial Values
    run_simulation_test(r, xR, yR, uR, vR, xB, yB, uB, vB)
    
    # Test Case 3:
    print("Test Case 3: ")
    xR, yR, uR, vR = 0.5, 0.5, 0.5, 0.5  # Red Ball Initial Values
    xB, yB, uB, vB = 0.05, 0.05, 0, 0  # Blue Ball Initial Values
    run_simulation_test(r, xR, yR, uR, vR, xB, yB, uB, vB)

def run_simulation_test(r, xR, yR, uR, vR, xB, yB, uB, vB):
     # initializing a figure in 
    # which the graph will be plotted
    fig = plt.figure() 

    # marking the x-axis and y-axis
    board = plt.axes(xlim =(0, 1), ylim =(0, 1)) 
    
    t = 0
    tfinal = 50
    dt = 0.02
    
    while t < tfinal:
        # draw the red and blue balls on the board
        board.set_facecolor("forestgreen")
        board.set_aspect(1)
        redBall = plt.Circle((xR, yR), radius=r, color='red')
        board.add_patch(redBall)
        blueBall = plt.Circle((xB, yB), radius=r, color='blue')
        board.add_patch(blueBall)
        
        plt.draw()
        plt.pause(0.005)
        redBall.remove()
        blueBall.remove()
        
        if (t + dt > tfinal):
            dt = tfinal - 1 
        
        xR = xR + (dt)*(uR)
        yR = yR + (dt)*(vR)
        xB = xB + (dt)*(uB)
        yB = yB + (dt)*(vB)
        
        dtnew = dt # If there is no collision of any kind, then the next iteration will just continue with the original dt.
        dtnewList = [] # List of all the possible dtnews. Must determine the minimum dtnew to see which event will happen first
        RdtnewRW = dt
        RdtnewLW = dt
        RdtnewTW = dt
        RdtnewBW = dt
        BdtnewRW = dt
        BdtnewLW = dt
        BdtnewTW = dt
        BdtnewBW = dt
        dtnewCollide = dt
        
        # Red Ball Case:
        x = xR
        y = yR
        u = uR
        v = vR
        # RIGHT Wall
        if ((x + r) > 1):
            RdtnewRW = (1 - r - x)/u
            dtnewList.append(RdtnewRW)
        # LEFT Wall
        if ((x - r) < 0):
            RdtnewLW = (x - r)/u
            dtnewList.append(RdtnewLW)
        # TOP Wall
        if ((y + r) > 1):
            RdtnewTW = (1 - r - y)/v
            dtnewList.append(RdtnewTW)
        # BOTTOM Wall
        if ((y - r) < 0):
            RdtnewBW = (y - r)/v
            dtnewList.append(RdtnewBW)
            
        # Blue Ball Case:
        x = xB
        y = yB
        u = uB
        v = vB
        # RIGHT Wall
        if ((x + r) > 1):
            BdtnewRW = (1 - r - x)/u
            dtnewList.append(BdtnewRW)
        # LEFT Wall
        if ((x - r) < 0):
            BdtnewLW = (x - r)/u
            dtnewList.append(BdtnewLW)
        # TOP Wall
        if ((y + r) > 1):
            BdtnewTW = (1 - r - y)/v
            dtnewList.append(BdtnewTW)
        # BOTTOM Wall
        if ((y - r) < 0):
            BdtnewBW = (y - r)/v
            dtnewList.append(BdtnewBW)
            
        # Ball-Ball Collision
        length = np.sqrt((xR - xB)**2 + (yR - yB)**2)
        if (length <= (2*r)):
            relative_distance = np.sqrt((xR - xB)**2 + (yR - yB)**2)
            relative_velocity = np.sqrt((uR - uB)**2 + (vR - vB)**2)
            dtnewCollide = (relative_distance - (2*r))/relative_velocity
            dtnewList.append(dtnewCollide)
        
        if (len(dtnewList) != 0) :  
            dtnew = min(dtnewList[0 : len(dtnewList)])  
            
            # Red Ball Wall Collisions:
            x = xR
            y = yR
            u = uR
            v = vR
            if (dtnew == RdtnewRW) :
                print("Red Ball:")
                xR, yR, uR, vR = rightWallCollision(r, x, y, u, v, dtnew)
            if (dtnew == RdtnewLW) :
                print("Red Ball:")
                xR, yR, uR, vR = leftWallCollision(r, x, y, u, v, dtnew)
            if (dtnew == RdtnewTW) :
                print("Red Ball:")
                xR, yR, uR, vR = topWallCollision(r, x, y, u, v, dtnew)
            if (dtnew == RdtnewBW) :
                print("Red Ball:")
                xR, yR, uR, vR = bottomWallCollision(r, x, y, u, v, dtnew)
            
            # Blue Ball Wall Collisions:            
            x = xB
            y = yB
            u = uB
            v = vB
            if (dtnew == BdtnewRW) :
                print("Blue Ball:")
                xB, yB, uB, vB = rightWallCollision(r, x, y, u, v, dtnew)
            if (dtnew == BdtnewLW) :
                print("Blue Ball:")
                xB, yB, uB, vB = leftWallCollision(r, x, y, u, v, dtnew)
            if (dtnew == BdtnewTW) :
                print("Blue Ball:")
                xB, yB, uB, vB = topWallCollision(r, x, y, u, v, dtnew)
            if (dtnew == BdtnewBW) :
                print("Blue Ball:")
                xB, yB, uB, vB = bottomWallCollision(r, x, y, u, v, dtnew)
            
            # Red and Blue Ball Collision Case:
            if (dtnew == dtnewCollide) :
                print("BOTH Balls:")
                xR, yR, uR, vR, xB, yB, uB, vB = objectsCollisionCase(r, xR, yR, uR, vR, xB, yB, uB, vB, dtnew, length)
    
        t = t + dtnew
        
    plt.show(block=False)
    plt.close('all')
        
def rightWallCollision(r, x, y, u, v, dtnew):
    alpha = 0.8
    beta = 0.98
    x = 1 - r
    y = y + (dtnew)*(v)
    u = -alpha * u
    v = beta * v
    print ("x: ", x, "y: ", y, "u", u, "v", v)
    return x, y, u, v

def leftWallCollision(r, x, y, u, v, dtnew):
    alpha = 0.8
    beta = 0.98
    x = r
    y = y + (dtnew)*(v)
    u = -alpha * u
    v = beta * v
    print ("x: ", x, "y: ", y, "u", u, "v", v)
    return x, y, u, v

def topWallCollision(r, x, y, u, v, dtnew):
    alpha = 0.8
    beta = 0.98
    x = x + (dtnew)*(u)
    y = 1 - r
    u = alpha * u
    v = -beta * v
    print ("x: ", x, "y: ", y, "u", u, "v", v)
    return x, y, u, v

def bottomWallCollision(r, x, y, u, v, dtnew):
    alpha = 0.8
    beta = 0.98
    x = x + (dtnew)*(u)
    y = r
    u = alpha * u
    v = -beta * v
    print ("x: ", x, "y: ", y, "u", u, "v", v)
    return x, y, u, v
        
def objectsCollisionCase(r, xR, yR, uR, vR, xB, yB, uB, vB, dtnew, length): 
    xR = xR + (dtnew)*(uR)
    yR = yR + (dtnew)*(vR)
    xB = xB + (dtnew)*(uB)
    yB = yB + (dtnew)*(vB)
    
    VR_in = np.array([uR, vR])
    VB_in = np.array([uB, vB])
    
    n1 = (xR - xB)/length
    n2 = (yR - yB)/length
    normal = np.array([n1, n2])
    
    t1 = n2
    t2 = -n1
    tangential = np.array([t1, t2])
    
    VR_out =  (VB_in @ normal) * normal + (VR_in @ tangential) * tangential
    VB_out =  (VR_in @ normal) * normal + (VB_in @ tangential) * tangential
    
    uR = VR_out[0]
    vR = VR_out[1]
    uB = VB_out[0]
    vB = VB_out[1]
                
    print ("xR: ", xR, "yR: ", yR, "uR", uR, "vR", vR, "xB: ", xB, "yB: ", yB, "uB", uB, "vB", vB)
    return xR, yR, uR, vR, xB, yB, uB, vB

if __name__ == "__main__":
    main()