# import the necessary packages
import cv2

#Function that handles the detection of the aruco markers
def aruco(pic,w,l):
    quad=0
    #generate a grayscale image from the given input file
    pic=cv2.cvtColor(pic,cv2.COLOR_BGR2GRAY)
    
    #Create the aruco dictionary for the given tag type
    aruco_dict=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)
    
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
            fov=53.5/2
            ratio=((w-(0.5*(t_right[0]+t_left[0])))/w)
            cv2.putText(pic,str(fov*ratio),(t_right[0],t_right[1]-15),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,0,255),2)
            
            #Finding Quadrent
            
            #Center of Object
            cx=int((t_left[0]+b_right[0])*0.5)
            cy=int((t_left[1]+b_right[1])*0.5)
            #If to find Quadrent
            if cx < (w) and cy < (l):
                quad=2
            elif cx > (w) and cy > (l):
                quad=4
            elif cx < (w) and cy > (l):
                quad=3
            elif cx > (w) and cy < (l):
                quad=1
                
            
            
    #Return processed picture and the quadrent that the object is in
    return(pic,quad)


#Start video and detection
def start():
#Start and Calibrate Video Capture
    cap=cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FPS,24)
    #cap.set(cv2.CAP_PROP_AUTO_EXPOSURE,0.25)
    #cap.set(cv2.CAP_PROP_EXPOSURE,5)

    #loop to keep getting and processing frames
    while True:
        #Getting fram
        ret,frame=cap.read()
        #Running detection on the frame
        width=0.5*cap.get(3)
        length=0.5*cap.get(4)
        a = aruco(frame,width,length)
        #Show processed image
        cv2.imshow("Live Feed",a[0])
        #wating for the user to hit q and stop the stream
        if cv2.waitKey(1) == ord('q'):
            break
        
    #Release the caputre and destory all the created windows
    cap.release()
    cv2.destroyAllWindows()    

start()