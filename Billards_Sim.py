import numpy as np
from matplotlib import pyplot as plt, patches
from matplotlib.animation import FuncAnimation 

def main():
    # initializing a figure in 
    # which the graph will be plotted
    fig = plt.figure() 

    # marking the x-axis and y-axis
    board = plt.axes(xlim =(0, 1), ylim =(0, 1)) 

    r = 0.05
    xR = 0.75
    yR = 5*r
    uR = -0.1
    vR = 0.5
    xB = 0.25
    yB = 5.5*r
    uB = 0.11
    vB = 0.2
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
        plt.pause(0.01)
        redBall.remove()
        blueBall.remove()
        
        if (t + dt > tfinal):
            dt = tfinal - 1 
        
        xR = xR + (dt)*(uR)
        yR = yR + (dt)*(vR)
        xB = xB + (dt)*(uB)
        yB = yB + (dt)*(vB)
        
        # Red Ball Case:
        x = xR
        y = yR
        u = uR
        v = vR
        xR, yR, uR, vR = wallCollisionCase(r, x, y, u, v)
        
        # Blue Ball Case:
        x = xB
        y = yB
        u = uB
        v = vB
        xB, yB, uB, vB = wallCollisionCase(r, x, y, u, v)
        
        xR, yR, uR, vR, xB, yB, uB, vB = objectsCollisionCase(r, xR, yR, uR, vR, xB, yB, uB, vB, dt)
    
        t = t + dt
    plt.show()
        
def wallCollisionCase(r, x, y, u, v):
    alpha = 0.8
    beta = 0.98
    
    # RIGHT Wall
    if ((x + r) > 1):
        dtnew = (1 - r - x)/u
        x = 1 - r
        y = y + (dtnew)*(v)
        u = -alpha * u
        v = beta * v
    # LEFT Wall
    if ((x - r) < 0):
        dtnew = (x - r)/u
        x = r
        y = y + (dtnew)*(v)
        u = -alpha * u
        v = beta * v
    # TOP Wall
    if ((y + r) > 1):
        dtnew = (1 - r - y)/v
        x = x + (dtnew)*(u)
        y = 1 - r
        u = alpha * u
        v = -beta * v
    # BOTTOM Wall
    if ((y - r) < 0):
        dtnew = (y - r)/v
        x = x + (dtnew)*(u)
        y = r
        u = alpha * u
        v = -beta * v
        
    return x, y, u, v
        
def objectsCollisionCase(r, xR, yR, uR, vR, xB, yB, uB, vB, dt):
    # # Update the position
    # xR = xR + (dt)*(uR)
    # yR = yR + (dt)*(vR)
    # xB = xB + (dt)*(uB)
    # yB = yB + (dt)*(vB)
    
    # Detect a Collision
    length = np.sqrt((xR - xB)**2 + (yR - yB)**2)
    if (length <= (2*r)):
        relative_distance = np.sqrt((xR - xB)**2 + (yR - yB)**2)
        relative_velocity = np.sqrt((uR - uB)**2 + (vR - vB)**2)
        
        dtnew = (relative_distance - (2*r))/relative_velocity
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
                
    return xR, yR, uR, vR, xB, yB, uB, vB

# # Test Case 6 (Corner Case): 
    # print("Test Case 6: ")
    # xR, yR, uR, vR = 0.5, 0.5, 0.5, 0.5  # Red Ball Initial Values
    # xB, yB, uB, vB = 0.95, 0.05, 0, 0  # Blue Ball Initial Values
    # alpha, beta = 0.8, 0.8
    # t, tfinal, dt = 0, 20, 0.02
    # run_simulation_test(r, xR, yR, uR, vR, xB, yB, uB, vB, alpha, beta, t, tfinal, dt)

# # RIGHT Wall
# if ((x + r) > 1):
#     BdtnewRW = (1 - r - x)/u
#     dtnewList.append(BdtnewRW)
# # LEFT Wall
# if ((x - r) < 0):
#     BdtnewLW = (x - r)/u
#     dtnewList.append(BdtnewLW)
# # TOP Wall
# if ((y + r) > 1):
#     BdtnewTW = (1 - r - y)/v
#     dtnewList.append(BdtnewTW)
# # BOTTOM Wall
# if ((y - r) < 0):
#     BdtnewBW = (y - r)/v
#     dtnewList.append(BdtnewBW)
        
# if (len(dtnewList) != 0) :  
#     dtnew = dtnewList[0]
#     for i in range(len(dtnewList) - 1):
#         dtnew = min(dtnewList[i], dtnew)
        
# # Red Ball Wall Motion
# # If the red ball hits the RIGHT wall
# if ((xR + r) > 1):
#     dtnew = (1 - r - xR)/uR
#     xR = 1 - r
#     yR = yR + (dtnew)*(vR)
#     uR = -alpha * uR
#     vR = beta * vR
# # If the red ball hits the LEFT wall
# if ((xR - r) < 0):
#     dtnew = (xR - r)/uR
#     xR = r
#     yR = yR + (dtnew)*(vR)
#     uR = -alpha * uR
#     vR = beta * vR
# # If the red ball hits the TOP wall
# if ((yR + r) > 1):
#     dtnew = (1 - r - yR)/vR
#     xR = xR + (dtnew)*(uR)
#     yR = 1 - r
#     uR = alpha * uR
#     vR = -beta * vR
# # If the red ball hits the BOTTOM wall
# if ((yR - r) < 0):
#     dtnew = (yR - r)/vR
#     xR = xR + (dtnew)*(uR)
#     yR = r
#     uR = alpha * uR
#     vR = -beta * vR
    
# # Blue Ball Wall Motion
# # If the blue ball hits the RIGHT wall
# if ((xB + r) > 1):
#     dtnew = (1 - r - xB)/uB
#     xB = 1 - r
#     yB = yB + (dtnew)*(vB)
#     uB = -alpha * uB
#     vB = beta * vB
# # If the blue ball hits the LEFT wall
# if ((xB - r) < 0):
#     dtnew = (xB - r)/uB
#     xB = r
#     yB = yB + (dtnew)*(vB)
#     uB = -alpha * uB
#     vB = beta * vB
# # If the blue ball hits the TOP wall
# if ((yB + r) > 1):
#     dtnew = (1 - r - yB)/vB
#     xB = xB + (dtnew)*(uB)
#     yB = 1 - r
#     uB = alpha * uB
#     vB = -beta * vB
# # If the blue ball hits the BOTTOM wall
# if ((yB - r) < 0):
#     dtnew = (yB - r)/vB
#     xB = xB + (dtnew)*(uB)
#     yB = r
#     uB = alpha * uB
#     vB = -beta * vB

if __name__ == "__main__":
    main()