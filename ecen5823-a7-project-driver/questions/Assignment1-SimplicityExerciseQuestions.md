Please include your answers to the questions below with your submission, entering into the space below each question
See [Mastering Markdown](https://guides.github.com/features/mastering-markdown/) for github markdown formatting if desired.

**1. How much current does the system draw (instantaneous measurement) when a single LED is on with the GPIO pin set to StrongAlternateStrong?**
   Answer: Approximately 5.39mA when LED is On, 4.76mA when LED is Off.


**2. How much current does the system draw (instantaneous measurement) when a single LED is on with the GPIO pin set to WeakAlternateWeak?**
   Answer: Approximately 5.25mA when LED is On, 4.77mA when LED is Off.


**3. Is there a meaningful difference in current between the answers for question 1 and 2? Please explain your answer, 
referencing the [Mainboard Schematic](https://www.silabs.com/documents/public/schematic-files/WSTK-Main-BRD4001A-A01-schematic.pdf) and [AEM Accuracy](https://www.silabs.com/documents/login/user-guides/ug279-brd4104a-user-guide.pdf) section of the user's guide where appropriate. Extra credit is avilable for this question and depends on your answer.**
   Answer: There is a small difference between the current for the strong drive strength and weak drive strength of about 140uA which is insignificant. However, there is a difference in the rise and fall times:
           For StrongAlternateStrong setting: Rise time: 300uS, Fall time: 300uS.
           For WeakAlternateWeak setting: Rise time: 400uS, Fall time: 400uS. 
           This difference is due of difference in slew rate for the two drive strengths as given in:(https://www.silabs.com/documents/public/application-notes/an0012-efm32-gpio.pdf). 
           Slew rate = dv/dt, hence for the strong drive setting the slew rate is higher as compared to weak drive strength, hence the StrongAlternateStrong setting should be used for 
           higher slew rate.


**4. With the WeakAlternateWeak drive strength setting, what is the average current for 1 complete on-off cycle for 1 LED with an on-off duty cycle of 50% (approximately 1 sec on, 1 sec off)?**
   Answer: With WeakAlternateWeak, the average current drawn for one LED is 5.02mA.


**5. With the WeakAlternateWeak drive strength setting, what is the average current for 1 complete on-off cycle for 2 LEDs (both on at the time same and both off at the same time) with an on-off duty cycle of 50% (approximately 1 sec on, 1 sec off)?**
   Answer: With WeakAlternateWeak, the average current drawn by 2 LEDs is 5.36mA. (same on and off period, 50% duty cycle).


