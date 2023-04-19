Please include your answers to the questions below with your submission, entering into the space below each question
See [Mastering Markdown](https://guides.github.com/features/mastering-markdown/) for github markdown formatting if desired.

*Be sure to take measurements with logging disabled to ensure your logging logic is not impacting current/time measurements.*

*Please include screenshots of the profiler window detailing each current measurement captured.  See the file Instructions to add screenshots in assignment.docx in the ECEN 5823 Student Public Folder.*

1. Provide screen shot verifying the Advertising period matches the values required for the assignment.
   <br>Screenshot: 250ms
   ![advertising_period](https://github.com/CU-ECEN-5823/ecen5823-assignment5-vishalraj3112/blob/master/screenshots/advertising_period.jpg)  

2. What is the average current between advertisements?
   Answer: 3.99uA
   <br>Screenshot: 
   ![avg_current_between_advertisements](https://github.com/CU-ECEN-5823/ecen5823-assignment5-vishalraj3112/blob/master/screenshots/avg_current_between_advertisements.jpg)  

3. What is the peak current of an advertisement? 
   Answer: 26.37mA
   <br>Screenshot: 
   ![peak_current_of_advertisement](https://github.com/CU-ECEN-5823/ecen5823-assignment5-vishalraj3112/blob/master/screenshots/peak_current_of_advertisement.jpg)  

4. Provide screen shot showing the connection interval settings. Do they match the values you set in your slave(server) code or the master's(client) values?.
   Answer: The connection interval parameters are: connection interval: 30ms, Latency - 0ms, Timeout - 720ms. These, values are totally different from what we tried setting on the
   server side.
   <br>Screenshot: 
   ![connection_interval](https://github.com/CU-ECEN-5823/ecen5823-assignment5-vishalraj3112/blob/master/screenshots/connection_interval.jpg)  

5. What is the average current between connection intervals?
   Answer: 2.22uA
   <br>Screenshot:
   ![avg_current_between_connection_intervals](https://github.com/CU-ECEN-5823/ecen5823-assignment5-vishalraj3112/blob/master/screenshots/avg_current_between_connection_intervals.jpg)  

6. If possible, provide screen shot verifying the slave latency matches what was reported when you logged the values from event = gecko_evt_le_connection_parameters_id. 
   Answer: Slave Latency - 15.50ms. As the Slave latency set by my phone is 0ms. The data once queued(temperature reading) will be sent immediately in the next cycle without any latency,
   which in our case is 15.50ms, just after calculation and write gatt DB operation completes and data is transmitted in the next cycle.
   <br>Screenshot:  
   ![slave_latency](https://github.com/CU-ECEN-5823/ecen5823-assignment5-vishalraj3112/blob/master/screenshots/slave_latency.jpg)  

7. What is the peak current of a data transmission when the phone is connected and placed next to the Blue Gecko? 
   Answer: 10.78mA
   <br>Screenshot:
   ![peak_current_phone_next_to](https://github.com/CU-ECEN-5823/ecen5823-assignment5-vishalraj3112/blob/master/screenshots/peak_current_phone_next_to.jpg)  
   
8. What is the peak current of a data transmission when the phone is connected and placed approximately 20 feet away from the Blue Gecko? 
   Answer: 18.85mA
   <br>Screenshot:  
   ![peak_current_phone_20ft_away](https://github.com/CU-ECEN-5823/ecen5823-assignment5-vishalraj3112/blob/master/screenshots/peak_current_phone_20ft_away.jpg)  
   