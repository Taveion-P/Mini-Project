# import the necessary packages
import time
import cv2

def takepic(file):
    cap=cv2.VideoCapture(0)
    ret,frame=cap.read()
   # allow the camera to warmup
    time.sleep(0.1)
    pic=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
   # grab an image from the camera
    print("Capturing Image...")

   # save the image to the disk
    print("Saving image "+file)
    
    cv2.imwrite("c:/Users/Taveion/Python Files/"+file,pic)
    
    #Create and image object from the file and dispaly it to the screen
    cv2.imshow("Picture",pic)
    #Wait for the user to hit esc and then destroy all created windows
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    cap.release()
       
#Main Stuff and Things    
file = input("Please Enter a file name: ")
for i in range(10):
    pic=file + str(i) +".jpg"
    takepic(pic)
