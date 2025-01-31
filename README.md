# roboVision project

A personalized voice-controlled robot assistant. The base use-case scenarios:
1. _Search_: The user asks the robot to find an object -> robot navigates around to find this object -> robot reports back the search status and information about the found object
2. _Delivery_: The user asks the robot to bring the object -> robot performs _Search_ to find an object -> if the search is successful grabs the object and delivers it to the user
3. _Sort_: The user asks the robot to sort objects into different places by some criteria -> robot performs _Search_ to find sorting places -> robot performs _Delivery_ to sort all objects

I can build a LEGO Mindstorm robot (because I already have this one) with:
- a differential drive using two large motors for wheels and a small rotating wheel for stability
- a smartphone used as a webcam
- a gripper controlled by one medium motor
- a radar to get an approximation of object pose
- a bumper that presses a button if the collision is very near
- a possibility to use a color sensor
  
[Detailed plan and work-in-progress](https://github.com/users/CatUnderTheLeaf/projects/2/views/4)

---
Current state after `Iteration2`:
- run `ngrok http --domain=<YOUR-DOMAIN-NAME> 5000` to locally run a web server with a public HTTPS URL and a flask app to receive Alexa intents
- run `ros2 launch bringup main.launch.py` (do not forget to source the workspace) to run all ROS files and additionally MQTT broker for communication with the robot
- run `main.py` on the legobot to launch MQTT client to send and receive messages from ROS
- activate your skill with the invocation name in '[Alexa developer console](https://developer.amazon.com/alexa/console/ask)/YOUR-SKILL-NAME/test'
- use skill commands like `drive`, `rotate`, `pause`, or `stop`
- the intent is sent to a flask app, which also works as a ROS action client and sends a request to the action server
- action server has hardcoded execution callbacks depending on the intent (it will be changed later)

| <video src="https://github.com/user-attachments/assets/40ea0960-a278-4102-85f5-d0d1ccc59fb1" /> | <video src="https://github.com/user-attachments/assets/5e04d0ea-d2f7-4472-a6c3-ddfea5f584ae" /> |
| :---:     | :---:   |
