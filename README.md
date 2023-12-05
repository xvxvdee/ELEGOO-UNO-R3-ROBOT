# ELEGOO-UNO-R3-ROBOT

To-do:
* [ ] Debug code changes made by Deandra
  * [ ] AMR stuck in Stop() (until object is detected) even if flame is not detected anymore 
* [x] ~~Edge Detection~~
  * [x] ~~Remove delays in edge detection algorithm~~
* [x] Object Avoidance
  * [x] Remove delays in object avoidance algorithm
  * [ ] Add 3 ultrasonic sensors to fix object detection blind spots
* [ ] Line Following
  * [ ] Fix line following algorithm, figure out why sometimes it doesn't follow lines properly
* [ ] Flame Detection and Extinguishing
  * [ ] Add 2 flame IR sensors OR attach 5-way flame sensor module to fix flame detection blind spots
  * [ ] Attach fan (DC Motor + Propellor)
  * [ ] Code logic for the fan (ON when middle == LOW, OFF when middle == HIGH) 
  * [ ] Add ```maxFlameTurnDuration``` to prevent infinite donuts in flame detection mode
* [ ] Defense Mechanism
  * [ ] Add space in the back of the chassis for our candle
  * [ ] Figure out a defense mechanism to prevent getting eliminated by other AMRs
