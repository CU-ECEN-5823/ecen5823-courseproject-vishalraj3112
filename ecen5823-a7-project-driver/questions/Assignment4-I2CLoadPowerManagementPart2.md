Please include your answers to the questions below with your submission, entering into the space below each question
See [Mastering Markdown](https://guides.github.com/features/mastering-markdown/) for github markdown formatting if desired.

*Be sure to take measurements with logging disabled to ensure your logging logic is not impacting current/time measurements.*

*Please include screenshots of the profiler window detailing each current measurement captured.  See the file Instructions to add screenshots in assignment.docx in the ECEN 5823 Student Public Folder.* 

1. What is the average current per period?
   Answer: 17.11uA
   <br>Screenshot:  
   ![Avg_current_per_period](https://github.com/CU-ECEN-5823/ecen5823-assignment4-vishalraj3112/blob/master/screenshots/Avg_current_per_period.jpg)  

2. What is the average current when the Si7021 is Powered Off?
   Answer: 5.52uA
   <br>Screenshot:  
   ![Avg_current_LPM_Off](https://github.com/CU-ECEN-5823/ecen5823-assignment4-vishalraj3112/blob/master/screenshots/Avg_current_LPM_Off.jpg)  

3. What is the average current when the Si7021 is Powered On?
   Answer: 325.94uA
   <br>Screenshot:
   ![Avg_current_LPM_On](https://github.com/CU-ECEN-5823/ecen5823-assignment4-vishalraj3112/blob/master/screenshots/Avg_current_LPM_On.jpg)  

4. How long is the Si7021 Powered On for 1 temperature reading?
   Answer: 101.9ms
   <br>Screenshot:  
   ![duration_lpm_on](https://github.com/CU-ECEN-5823/ecen5823-assignment4-vishalraj3112/blob/master/screenshots/duration_lpm_on.jpg)  

5. Compute what the total operating time of your design for assignment 4 would be in hours, assuming a 1000mAh battery power supply?
   Answer: The total operating time is: (battery_capacity)/(average current consumption in each cycle) = 1000/17.11uA = 58445.35 hours
   
6. How has the power consumption performance of your design changed since the previous assignment?
   Answer: The average current consumption reduction is about 140uA(155uA in A3 to 17uA in A4) from pervious assignment or almost 90% reduction! The On time current reduction is about
           92% from 4.58mA to 325.94uA. This shows that interrupt based delay and I2C transfers are much more energy efficient that simple polling based methods.
   
7. Describe how you tested your code for EM1 during I2C transfers.
   Answer:  It is not possible to see the transition of the MCU to EM1 in energy profiler since it is in EM1 for a very short duration of time, just before I2C transfer starts and after the
            I2C transfer ends, hence I put the code in debug mode and stepped through it to verify MCU going in EM1.
