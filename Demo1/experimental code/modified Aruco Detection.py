# import the necessary packages
import cv2
import numpy as np
import glob
import math
#Defining Global Variables
quad=0
quad_change_flag=0

#Camera Set Up
def calibrate():
    CHECKERBOARD = (7,7)
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    
    # Creating vector to store vectors of 3D points for each checkerboard image
    objpoints = []
    # Creating vector to store vectors of 2D points for each checkerboard image
    imgpoints = [] 
    
    
    # Defining the world coordinates for 3D points
    objp = np.zeros((1, CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
    objp[0,:,:2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
    
    # Extracting path of individual image stored in a given directory
    images = glob.glob('C:/Users/Taveion/Python Files/*.jpg')
    for fname in images:
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        # Find the chess board corners
        # If desired number of corners are found in the image then ret = true
        ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE)
        
        """
        If desired number of corner are detected,
        we refine the pixel coordinates and display 
        them on the images of checker board
        """
        if ret == True:
            objpoints.append(objp)
            # refining pixel coordinates for given 2d points.
            corners2 = cv2.cornerSubPix(gray, corners, (11,11),(-1,-1), criteria)
            
            imgpoints.append(corners2)
    
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
    return(mtx,dist)

#Function that handles the detection of the aruco markers
def aruco(pic,w,l,mtx,dist):
    current_quad=0
    ang=0
    #generate a grayscale image from the given input file
    pic=cv2.cvtColor(pic,cv2.COLOR_BGR2GRAY)
    
    #Create the aruco dictionary for the given tag type
    aruco_dict=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_1000)
    
    #Create aruco paramters
    params=cv2.aruco.DetectorParameters()
    
    #detect function
    (corners,ids,rejected)=cv2.aruco.detectMarkers(pic,aruco_dict,parameters=params)
    
    #Loop that goes through and grabs each corner of the detected marker
    if len(corners) > 0:
        ids=ids.flatten()
        markers=zip(corners,ids)
        for(c,i) in markers:
            #Reshaping
            corners=c.reshape((4,2))
            (t_left,t_right,b_right,b_left)=corners
            t_left=(int(t_left[0]),int(t_left[1]))
            t_right=(int(t_right[0]),int(t_right[1]))
            b_right=(int(b_right[0]),int(b_right[1]))
            b_left=(int(b_left[0]),int(b_left[1]))
            
            #drawing the box for each marker
            cv2.line(pic,t_left,t_right,(0,255,0),2)
            cv2.line(pic,t_left,b_left,(0,255,0),2)
            cv2.line(pic,t_right,b_right,(0,255,0),2)
            cv2.line(pic,b_left,b_right,(0,255,0),2)
            
            #adding the marker id
            cv2.putText(pic,str(i),(t_left[0],t_left[1]-15),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,0,255),2)
            
            #Calculation the distance and degree offset

            #Center of Object
            cx=int((t_left[0]+b_right[0])*0.5)
            cy=int((t_left[1]+b_right[1])*0.5)

            fov=68.5/2
            ratio=((w-(0.5*(t_right[0]+t_left[0])))/w)
            ang=int(fov*ratio)
            cv2.putText(pic,str(ang),(b_right[0],b_right[1]+15),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,0,255),2)
            #cv2.putText(pic,str(ang),(t_right[0],t_right[1]-15),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,0,255),2)
            #finding focal lens to pixel proportion
            
            
            h,  w = pic.shape[:2]
            newcameramtx,roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))
            r1=newcameramtx.dot([cx,cy,1.0])
            r2=newcameramtx.dot([w/2,h/2,1.0])
            cosine=r1.dot(r2) / (np.linalg.norm(r1) * np.linalg.norm(r2))
            angle=int(math.degrees(np.arccos(cosine)))
            cv2.putText(pic,str(angle),(t_right[0],t_right[1]-15),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,0,255),2)

            
            #Finding Quadrent
            
            #If to find Quadrent
            if cx < (w) and cy < (l):
                current_quad=2
            elif cx > (w) and cy > (l):
                current_quad=4
            elif cx < (w) and cy > (l):
                current_quad=3
            elif cx > (w) and cy < (l):
                current_quad=1


            #Update global quad and change quad flag variable
            global quad,quad_change_flag    
            if current_quad != quad:
                quad=current_quad
                quad_change_flag=1    
            
            
    #Return processed picture and the quadrent that the object is in
    return(pic,current_quad,ang)


#Start video and detection
def start():
#Grab Global Variables
    global quad,quad_change_flag

#Start and Calibrate Video Capture
    cam_cal=calibrate()
    cap=cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FPS,24)
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE,0.25)
    #cap.set(cv2.CAP_PROP_EXPOSURE,5)

    #loop to keep getting and processing frames
    while True:
        #Getting fram
        ret,frame=cap.read()
        #Running detection on the frame
        width=0.5*cap.get(3)
        length=0.5*cap.get(4)
        a = aruco(frame,width,length,cam_cal[0],cam_cal[1])
        #Show processed image
        cv2.imshow("Live Feed",a[0])
        #wating for the user to hit q and stop the stream
        if cv2.waitKey(1) == ord('q'):
            break
        if quad_change_flag:
            print("Change Quadrant to: ",a[1])
            quad_change_flag=0
    #Release the caputre and destory all the created windows
    cap.release()
    cv2.destroyAllWindows()

start()