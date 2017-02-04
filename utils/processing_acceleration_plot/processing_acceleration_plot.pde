 // Graphing sketch. Modified demo

 import processing.serial.*;
 
 Serial myPort;        // The serial port
 int xPos = 1;         // horizontal position of the graph
 float ax = 0;        //Acceleration in x
 float ay = 0;
 float az = 0;
 
 //Stored accelerations
 float axPrev = 0;
 float ayPrev = 0;
 float azPrev = 0;

 float aMax = 20000;
 float aMin = -20000;
 
 void setup () {
 // set the window size:
 size(800, 600);        
 
 // List all the available serial ports
 println(Serial.list());
 // I know that the first port in the serial list on my mac
 // is always my  Arduino, so I open Serial.list()[0].
 // Open whatever port is the one you're using.
 myPort = new Serial(this, Serial.list()[0], 38400);
 // don't generate a serialEvent() unless you get a newline character:
 myPort.bufferUntil('\n');

 // set inital background:
 background(0);
 }
 void draw () {
 // everything happens in the serialEvent()
 }
 
 void serialEvent (Serial myPort) {
  //Store and reset  variables
  axPrev = ax;
  ayPrev = ay;
  azPrev = az;
  
   
 // get the ASCII string:
 String inString = myPort.readStringUntil('\n');

 if (inString != null) {

 // split acceleration values ax, ay, az
 String[] accelerationList = split(inString,';');
 
 //Check if correct, otherwize ignore
 if (accelerationList.length == 3){
   
   ax = float(accelerationList[0]);
   ax = map(ax,aMin,aMax,0,height);
 
   ay = float(accelerationList[1]);
   ay = map(ay,aMin,aMax,0,height);
 
   az = float(accelerationList[2]);
   az = map(az,aMin,aMax,0,height);
  
   //draw accelerations
   stroke(100,100,100);
   line(0,height/2,width,height/2);
   stroke(255,0,0);
   line(xPos-1, height-axPrev, xPos, height - ax);
   stroke(0,255,0);
   line(xPos-1, height-ayPrev, xPos, height - ay);
   stroke(0,0,255);
   line(xPos-1, height-azPrev, xPos, height - az);

   
 }
 
 
 


 
 // at the edge of the screen, go back to the beginning:
 if (xPos >= width) {
 xPos = 0;
 background(0);
 }
 else {
 // increment the horizontal position:
 xPos++;
 }
 }
 }
 
