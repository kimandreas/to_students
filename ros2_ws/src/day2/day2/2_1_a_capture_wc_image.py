#This program runs on Window due to WSL's limitation on integrating USB Camera

import cv2
import os
#import time

save_directory = "img_capture"  #save direectory is set to be under the current directory

def capture_image():

    os.makedirs(save_directory, exist_ok=True)

    # del_img = input("Do you want to delete the previous images: (y/n) ")
    # if del_img == 'y':
    #     file_name = f'{save_directory}\*.*'
    #     os.remove(file_name)
    #     print(f"{file_name} has been deleted.")

    file_prefix = input("Enter a file prefix to use : ")
    file_prefix = f'{file_prefix}_'
    print(file_prefix)
    
    image_count = 0
    cap = cv2.VideoCapture(0)   #PC Camera
    # cap = cv2.VideoCapture(1)   #USB Camera
    

    while True:
        ret, frame = cap.read()
        
        cv2.imshow("Webcam", frame)

        key = cv2.waitKey(1)
        if key == ord('c'):

            # change the filename when multiple people are capturing images
            # ex: obj1_img_{image_count}.jpg
            # then all images and txt files generated can be combined for execution of step 3
            file_name = f'{save_directory}/{file_prefix}img_{image_count}.jpg'  
            
            cv2.imwrite(file_name, frame)
            print(f"Image saved. name:{file_name}")
            image_count += 1
        
        elif key == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

def main():
    capture_image()

if __name__ == "__main__":
    main()

