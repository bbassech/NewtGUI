import processing.serial.*;
import java.util.List;
import java.util.ArrayList;
import java.awt.AWTException;
import java.awt.Robot;
import java.awt.PointerInfo;
import java.awt.MouseInfo;
import java.awt.Point;
import java.awt.event.KeyEvent;
import controlP5.*;
import papaya.*;

int r,g,b;    // Used to color background
Serial port;  // The serial port object
Robot robot; //Create object from Robot class

int myColor = color(244,243,103);
int c1, c2;
float n, n1;
int guiScale = 3; //adjusts size of GUI for different resolution screens

int i = 0;  //This is an incrementing counter that increments everytime something is read from serialEvent
int mode = 0; //This variable is changed when keys are pressed and used by serialEvent() to check what to do with incoming data
int j = 0; //This is an incrementer used for populated arrays with a fixed number of samples (neutralY, leftSamplesY, etc)

//Declare variables for calculating rotation angles and visualizing on cube
float xZero=1640; //center value of x channel. Determined with controller resting "housing down" on a table
float yZero=2010; //center value of y channel
float zZero=2050; // center value of z channel
float Scale=1/300.0; //sensitivity (g/mV) of accelerometer

float signal=0; //initialize variable for signal angle
float pitch=0; //initialize variable for pitch angle
float neutralSignal=0; //variable for the angle of neutral postion
float leftThresh=0; //variable for the angle of the left threshold
float rightThresh=0; //variable for the angle of the right threshold
int[] rxheading = {0, 0}; //Used for telling GUI and Emulator when threshold has been reached

//Initialize float lists for all acceleration channels. Values will be appended 
//to these lists everytime a non null string is written to the port
FloatList pot_vals;
FloatList x_vals;
FloatList y_vals;
FloatList z_vals;

//initialize variables to store the x,y,z accelerations when thresholds are set
float[] neutralX = new float[10]; //used to collect 10 x values for neutral...
float[] neutralY = new float[10];
float[] neutralZ = new float[10];
float[] leftSamplesX = new float[10]; //Used to store 10 x values for left position to average for a threshold
float[] leftSamplesY = new float[10]; //Used to store 10 y values for left position to average for a threshold
float[] leftSamplesZ = new float[10]; //Used to store 10 z values for left position to average for a threshold
float[] rightSamplesX = new float[10]; //Used to store 10 x values for right position to average for a threshold
float[] rightSamplesY = new float[10]; //Used to store 10 y values for right position to average for a threshold
float[] rightSamplesZ = new float[10]; //Used to store 10 y values for right position to average for a threshold

// For Mouse control
int x = 0;     // Mouse pointer position
int y = 0;     // Mouse pointer position
int new_x = 0; // New mouse pointer position
int new_y = 0; // New mouse pointer position
int increment = 4*guiScale; // pixel increment to move mouse position by
int old_rxheading = 0;  // Previous rxheading value...

int x_window_limit = 1;
int y_window_limit = 0;


// GUI
ControlP5 cp5;
DropdownList ddl_ports;             // available serial ports as a DropdownList
controlP5.Button b_emulator_on;
controlP5.Button b_controlmode; //Button for switching control mode
controlP5.Button b_keyboard_mouse;
controlP5.Button b_heading_left;
controlP5.Button b_heading_stop;
controlP5.Button b_heading_right;
controlP5.Button b_collect_left;
controlP5.Button b_collect_right;
controlP5.Button b_collect_neutral;

controlP5.Textlabel t_port_status;
controlP5.Textlabel t_heading_status;
controlP5.Textlabel t_desc_emulator_on; //Emulator on or off
controlP5.Textlabel t_desc_controlmode; //Control mode text label
controlP5.Textlabel t_desc_thresholds; //Text label for thresholds section
controlP5.Textlabel t_desc_left_thresh; //Text label for left thresh
controlP5.Textlabel t_desc_right_thresh; //Text label for right thresh
controlP5.Textlabel t_desc_neutral; //Text label for neutral
controlP5.Textlabel t_desc_emulator_type; //Mouse or Keyboard

controlP5.Textlabel thresholds;

controlP5.Textlabel t_mouse_x;
controlP5.Textlabel t_mouse_x_value;
controlP5.Textlabel t_mouse_y;
controlP5.Textlabel t_mouse_y_value;
controlP5.Textlabel t_desc_x_min;
controlP5.Textlabel t_desc_x_max;


controlP5.Textfield t_left_thresh;
controlP5.Textfield t_right_thresh;
controlP5.Textfield t_neutral;
controlP5.Textfield t_in_x_min;
controlP5.Textfield t_in_x_max;


// these variables will all get toggled when they are called to update the button text...
// so fill them with the opposite of the desired value
boolean emulator_on_toggle = true;        // 0 = off, 1 = on
boolean controlmode_toggle = false; //0 = wrist, 1 = forearm
boolean keyboard_mouse_toggle = true;     // 0 = mouse, 1 = keyboard
int mouse_speed = 6;                      // fraction of display; movement per heading

float controller_sensitivity_value = 1;   // default value for sensitivity
float controller_speed_value = 5;         // default value for speed

String[] port_list;                // 
int port_number;                   // selected port
boolean port_selected = false;     // if a port has been chosen
boolean port_setup = false;        // if a port has been set up

void setup() {
  pot_vals = new FloatList();
  x_vals = new FloatList();
  y_vals = new FloatList();
  z_vals = new FloatList();
  println(Serial.list());
//port = new Serial(this, Serial.list()[0], 115200); //USE THIS IF NOT USING PORT SELECT DROPDOWN
  frameCount = 1; //enable use of delay()

  size(400*guiScale,500*guiScale);
  
  // -------------------
  // GUI
   
  noStroke();
  
  cp5 = new ControlP5(this);
  
  PFont pfont = createFont("Arial", 24*guiScale, false);
  ControlFont font = new ControlFont(pfont, 12*guiScale);
  cp5.setControlFont(font);
  
  PFont pfont2 = createFont("Arial",28*guiScale, false);
  ControlFont font2 = new ControlFont(pfont2,13*guiScale);
  
  // Text label for heading status
  t_heading_status = cp5.addTextlabel("heading_status", "", 10*guiScale, 60*guiScale); // 10, 160
  t_heading_status.setFont(font);
  
  // Buttons to show status of command being received
  b_heading_left = cp5.addButton("left")
    .setValue(0)
    .setPosition(240*guiScale, 57*guiScale) // 240, 157
    .setSize(20*guiScale, 20*guiScale)
    .setId(1);
  
  b_heading_stop = cp5.addButton("stop")
    .setValue(0)
    .setPosition(270*guiScale, 57*guiScale) // 270, 157
    .setSize(20*guiScale, 20*guiScale)
    .setId(2);
  
  b_heading_right = cp5.addButton("right")
    .setValue(0)
    .setPosition(300*guiScale, 57*guiScale) // 300, 157
    .setSize(20*guiScale, 20*guiScale)
    .setId(3);
  
  // Text label for emulator on/off
  t_desc_emulator_on = cp5.addTextlabel("desc_emulator_on", "", 10*guiScale, 100*guiScale);
  
  // Button to start/stop toy controller emulation
 PImage  play_button = loadImage("play_button.png");
 play_button.resize(30*guiScale, 30*guiScale);
  b_emulator_on = cp5.addButton("emulator_on")
    .setValue(0)
    .setPosition(240*guiScale, 92*guiScale)
    .setImage(play_button)
    .setSize(30*guiScale, 30*guiScale)
    .setId(0);
  
  // Text label for control mode
  t_desc_controlmode = cp5.addTextlabel("desc_controlmode", "",10*guiScale, 140*guiScale).setFont(font);

  // Button to switch control mode
  b_controlmode = cp5.addButton("controlmode")
  .setValue(0)
  .setPosition(240*guiScale, 132*guiScale)
  .setSize(80*guiScale, 30*guiScale)
  .setId(0);
  
  // Text label for Thresholds
  t_desc_thresholds = cp5.addTextlabel("t_desc_thresholds","",10*guiScale,180*guiScale).setFont(font); 
    // Text label for left and right thresholds
    t_desc_left_thresh = cp5.addTextlabel("t_desc_left_thresh","",20*guiScale, 200*guiScale).setFont(font);
    t_desc_right_thresh = cp5.addTextlabel("t_desc_right_thresh","",20*guiScale, 230*guiScale).setFont(font);  
    t_desc_neutral = cp5.addTextlabel("t_desc_neutral","",20*guiScale, 260*guiScale).setFont(font);
  
  // Text input for thresholds
  t_left_thresh = cp5.addTextfield("left_thresh")
    .setValue(-45)
    .setPosition(240*guiScale, 200*guiScale)
    .setSize(30*guiScale, 20*guiScale)
    .setFont(font);
    t_left_thresh.captionLabel().setVisible(false);
  t_right_thresh = cp5.addTextfield("right_thresh")
    .setValue(45)
    .setPosition(240*guiScale, 230*guiScale)
    .setSize(30*guiScale, 20*guiScale)
    .setFont(font);
    t_right_thresh.captionLabel().setVisible(false);
  t_neutral = cp5.addTextfield("neutral")
    .setValue(0)
    .setPosition(240*guiScale, 260*guiScale)
    .setSize(30*guiScale,20*guiScale)
    .setFont(font);
    t_neutral.captionLabel().setVisible(false);
  
  // Button collection for thresholds  
  b_collect_left = cp5.addButton("collect_left")
    .setValue(0)
    .setPosition(280*guiScale, 200*guiScale)
    .setSize(64*guiScale, 20*guiScale)
    .setColorBackground(#FFFFFF)
    .setId(0)
    .setLabel("Capture")
    .setLabelVisible(true);
    b_collect_left.getCaptionLabel()
      .setFont(new ControlFont(pfont, 12*guiScale))
      .setSize(14*guiScale)
      .toUpperCase(false)
      .setColorBackground(#E36559)
      .align(CENTER,CENTER);
  b_collect_right = cp5.addButton("collect_right")
    .setValue(0)
    .setPosition(280*guiScale, 230*guiScale)
    .setSize(64*guiScale, 20*guiScale)
    .setColorBackground(#FFFFFF)
    .setId(0)
    .setLabel("Capture")
    .setLabelVisible(true);
    b_collect_right.getCaptionLabel()
      .setFixedSize(true)
      .setFont(font2)
      .setSize(14*guiScale)
      .toUpperCase(false)
      .setColorBackground(#E36559)
      .align(CENTER,CENTER);
  b_collect_neutral = cp5.addButton("collect_neutral")
    .setValue(0)
    .setPosition(280*guiScale, 260*guiScale)
    .setSize(64*guiScale,20*guiScale)
    .setColorBackground(#FFFFFF)
    .setId(0)
    .setLabel("Capture")
    .setLabelVisible(true);
    b_collect_neutral.getCaptionLabel()
      .setFont(font2)
      .setSize(14*guiScale)
      .toUpperCase(false)
      .setColorBackground(#E36559)
      .align(CENTER,CENTER);
      
   
  // Text label for emulator type
  t_desc_emulator_type = cp5.addTextlabel("desc_emulator_type", "", 10*guiScale, 360*guiScale); // 10, 180
  
  // Button to toggle between keyboard and mouse emulation, 'keyboard_mouse'
  b_keyboard_mouse = cp5.addButton("keyboard_mouse")
    .setValue(0)
    .setPosition(240*guiScale, 352*guiScale) // 240, 112
    .setSize(100*guiScale, 30*guiScale)
    .setId(4);
  
  // text labels for current mouse position
  t_mouse_x = cp5.addTextlabel("mouse_x", "", 240*guiScale, 390*guiScale); // 212
  t_mouse_x_value = cp5.addTextlabel("mouse_x_value", "", 265*guiScale, 390*guiScale);
  t_mouse_y = cp5.addTextlabel("mouse_y", "", 240*guiScale, 410*guiScale);
  t_mouse_y_value = cp5.addTextlabel("mouse_y_value", "", 265*guiScale, 410*guiScale);
  
  // text label for x-min
  t_desc_x_min = cp5.addTextlabel("desc_x_min", "", 10*guiScale, 440*guiScale); // 280
  
  // text input for x-min
  t_in_x_min = cp5.addTextfield("")
    .setValue(0)
    .setPosition(240*guiScale, 432*guiScale)
    .setSize(100*guiScale, 30*guiScale);
  t_in_x_min.setInputFilter(ControlP5.INTEGER);
  
  // text label for x-max
  t_desc_x_max = cp5.addTextlabel("desc_x_max", "", 10*guiScale, 480*guiScale); // 320
  
  // text input for x-max
  t_in_x_max = cp5.addTextfield(" ")
    .setValue(0)
    .setPosition(240*guiScale, 472*guiScale)
    .setSize(100*guiScale, 30*guiScale);
  t_in_x_max.setInputFilter(ControlP5.INTEGER);
  
  
  // --- this must be down here...
  
  // Text label for status of port  
  t_port_status = cp5.addTextlabel("port_status","",10*guiScale,35*guiScale);
  
  // DropdownList to display the values
  //ports = cp5.addDropdownList("ports-list",10,30,180,84);
  ddl_ports = cp5.addDropdownList("dropdownlist_ports")
    .setPosition(10*guiScale,30*guiScale)
    .setWidth(380*guiScale)
    .setHeight(140*guiScale);
  
  // This function can be used to refresh the COM ports in the list
  customize_dropdownlist(ddl_ports);
    
  t_port_status.setColorValue(0xFF0303);
  t_port_status.setValue("Not connected");

  t_heading_status.setColorValue(0xFF0303);
  t_heading_status.setValue("Controller heading status: ");
  
  t_desc_emulator_on.setColorValue(0xFF0303);
  t_desc_emulator_on.setValue("Toggle emulator ON/OFF");
  
  t_desc_controlmode.setColorValue(0xFF0303);
  t_desc_controlmode.setValue("Control Mode");
  
  t_desc_thresholds.setColorValue(0xFF0303);
  t_desc_thresholds.setValue("Thresholds: ");
  
  t_desc_left_thresh.setColorValue(0xFF0303);
  t_desc_left_thresh.setValue("Left (CCW):");  
  t_desc_right_thresh.setColorValue(0xFF0303);
  t_desc_right_thresh.setValue("Right (CW):");
  t_desc_neutral.setColorValue(0xFF0303);
  t_desc_neutral.setValue("Neutral:");
  
  
  t_left_thresh.setValue("-45"); //Initiates thresholds to display 45 and -45
  t_right_thresh.setValue("45");
  t_neutral.setValue("0"); //Initiates neutral to display 0
  
  t_desc_emulator_type.setColorValue(0xFF0303);
  t_desc_emulator_type.setValue("Toggle emulator Keyboard/Mouse");
  
  t_mouse_x.setColorValue(0xFF0303);
  t_mouse_x.setValue("x:");
  
  t_mouse_x_value.setColorValue(0xFF0303);
  t_mouse_x_value.setValue("0");
  
  t_mouse_y.setColorValue(0xFF0303);
  t_mouse_y.setValue("y:");
  
  t_mouse_y_value.setColorValue(0xFF0303);
  t_mouse_y_value.setValue("0");
  
  t_desc_x_min.setColorValue(0xFF0303);
  t_desc_x_min.setValue("Mouse horizontal left limit (x min):");
  
  t_in_x_min.setValue("0");
  
  t_desc_x_max.setColorValue(0xFF0303);
  t_desc_x_max.setValue("Mouse horizontal right limit (x max):");
  
  t_in_x_max.setValue("" + displayWidth);
  
  cp5.getController("emulator_on")
    .getCaptionLabel();
    
  cp5.getController("controlmode")
    .getCaptionLabel();
   
  cp5.getController("keyboard_mouse")
    .getCaptionLabel();
  
  cp5.getController("left")
    .getCaptionLabel();
  b_heading_left.captionLabel()
    .setText("");
    
  cp5.getController("stop")
    .getCaptionLabel();
  b_heading_stop.captionLabel()
    .setText("");
    
  cp5.getController("right")
    .getCaptionLabel();
  b_heading_right.captionLabel()
    .setText("");

 
  // -------------------
  
  // DEBUG
  //println("Display width:     " + displayWidth);
  //println("Cursor increment:  " + displayWidth/mouse_speed);
  
  
  // -------------------
  
  // Setup the robot to move the mouse around...
  try {
    robot = new Robot();
  }
  catch(AWTException e) {
    e.printStackTrace();
  }
  
  // Center the mouse
  robot.mouseMove(displayWidth/2, displayHeight/2);

}

void draw() {
  background(myColor);
//  int[] rxheading = {0, 0}; //Defined globally instead
  int heading = 0;
  
  while(port_selected == true && port_setup == false)
  {
    // DEBUG
    //println("starting serial port");
    start_serial(port_list);
  }
  
    /**
   * Heading codes:
   *
   * 0: Nothing
   * 3: Forward (right, because we are in the Northern Hemisphere...)
   * 6: Reverse (left)
   */  
  if (rxheading[0] >= 0) {
    if (rxheading[1] == 3) {
      heading = 1;
      b_heading_right.setColorBackground(color(13, 208, 255));  // Active
      b_heading_left.setColorBackground(color(90));   // Inactive
      b_heading_stop.setColorBackground(color(90));   // Inactive
    } else if (rxheading[1] == 6) {
      heading = -1;
      b_heading_right.setColorBackground(color(90));  // Inactive
      b_heading_left.setColorBackground(color(13, 208, 255));   // Active
      b_heading_stop.setColorBackground(color(90));   // Inactive
    } else {
      heading = 0;
      b_heading_right.setColorBackground(color(90));  // Inactive
      b_heading_left.setColorBackground(color(90));   // Inactive
      b_heading_stop.setColorBackground(color(13, 208, 255));   // Active
    }
  }

    /**
   * Control the Keyboard (0) or the Mouse (1) depending on the state
   * of the variable keyboard_mouse_toggle
   *
   */
  
  // If the emulator is not running, do nothing...
  if(!emulator_on_toggle) {
    return;
  }
  
  // Check if we are going to emulate the mouse or the keyboard...
  if (!keyboard_mouse_toggle) {
    // If we are moving the mouse pointer...
    PointerInfo a = MouseInfo.getPointerInfo();
    // Catch NullPointerException in case the pointer disappears...
    if (a != null) {
      Point b = a.getLocation();
      x = (int) b.getX();
      y = (int) b.getY();

      /*  OLD WAY OF EMULATING MOUSE
      // Move the x-coordinate of the mouse baed on the controller
      new_x = x + (heading * mouse_speed * (displayWidth / 100));
      if (new_x < x_window_limit) {
        new_x = x_window_limit;
      } else if (new_x > (displayWidth - x_window_limit)) {
        new_x = displayWidth - x_window_limit;
      }
      */                 
      
      // Move the x-coordinate of the mouse by increments of "increment"
      if (rxheading[0] >= 0) {
        //println(rxheading);
        //if (abs(rxheading[0] - old_rxheading) > controller_sensitivity_value) {
        if (abs(rxheading[1] - old_rxheading) > 1) {
          if (rxheading[1]==6) {
            new_x = x - 10;
          } else if (rxheading[1]==3) {
            new_x = x + 10;
          }

          
          if (new_x < Integer.parseInt(t_in_x_min.getText())) {
            new_x = Integer.parseInt(t_in_x_min.getText());
          } else if (new_x > Integer.parseInt(t_in_x_max.getText())) {
            new_x = Integer.parseInt(t_in_x_max.getText());
          }
          
          old_rxheading = rxheading[0];
        
      } else {
          new_x = x;
        }
        //new_x = x;
      } else {
        new_x = x;
      }
      //if (new_x < x_window_limit) {
      //  new_x = x_window_limit;
      //} else if (new_x > (displayWidth - x_window_limit)) {
      //  new_x = displayWidth - x_window_limit;
      //}
      
      // Keep the y-coordinate the same
      new_y = y;
      
      t_mouse_x_value.setValue(""+mouseX/guiScale); //THIS IS a DEV tool for locating positions in GUI
      t_mouse_y_value.setValue(""+mouseY/guiScale);
      
      //t_mouse_x_value.setValue(""+new_x);
      //t_mouse_y_value.setValue(""+new_y);
      
      // This is a little choppy...
      robot.mouseMove(new_x, new_y);
    }
  } else if (keyboard_mouse_toggle) {
  //  println(heading);
    // Toggle the keys
    if (heading == 1) {
      robot.keyRelease(KeyEvent.VK_LEFT);
      robot.keyPress(KeyEvent.VK_RIGHT);
    } else if (heading == -1) {
      robot.keyRelease(KeyEvent.VK_RIGHT);
      robot.keyPress(KeyEvent.VK_LEFT);
    } else {
      robot.keyRelease(KeyEvent.VK_LEFT);
      robot.keyRelease(KeyEvent.VK_RIGHT);
      // Do nothing...
      ;
    }
  }
  
}
    

void collectDynamic() { //This function is called when the emulator is turned on
  if (!controlmode_toggle) {//if we are in wrist mode
    float pot1 = 0;
    leftThresh = int(t_left_thresh.getText()); //CONSIDER CHANGING leftThresh to leftRoll
    rightThresh = int(t_right_thresh.getText());
    println(leftThresh);
    pot1=pot_vals.get(i-1);
    signal = (pot1*0.089433+29.72776)-180; //calculates flexion/extension angle
    println(signal);
    if (signal < leftThresh) { //may need OR statement to address the fact the pronation past 90degrees should still count.  yAcc<0 excludes actual supination
      //println("flexed");
      //fill(255,0,0);
      rxheading[1] = 6; // CURRENTLY CORRESPONDS to RIGHT HEADER
    } else if (signal > rightThresh) {
      //println("extended");
      //fill(0,0,255);
      rxheading[1] = 3; // CURRENTLY CORRESPONDS TO LEFT HEADER
    } else {
      //println("neutral");
      //fill(255,228,225);
      rxheading[1] = 0; // CURRENTLY CORRESPONDS TO LEFT HEADER
    }
  
  } else { //if we are in forearm mode
      
        float x1 = 0;
        float y1 = 0;
        float z1 = 0;
        float xAcc=0; //x acceleration in g's
        float yAcc=0; //y acceleration in g's 
        float zAcc=0; //z acceleration in g's
        
      //Gets thresholds from text entry fields
      leftThresh = float(t_left_thresh.getText()); 
      rightThresh = float(t_right_thresh.getText());
      //println(leftThresh);
      //println(rightThresh);
      
          x1=x_vals.get(i-1);
          y1=y_vals.get(i-1);
          z1=z_vals.get(i-1);
          xAcc=(x1-xZero)*Scale; //Converts voltage to acceleration in g's
          yAcc=(y1-yZero)*Scale; //Converts voltage to acceleration in g's
          zAcc=(z1-zZero)*Scale; //Converts voltage to acceleration in g's
      
      //Calculation of signal and pitch angle (4 options using    
        //Aerospace rotation sequence
          signal=180/PI*(atan(yAcc/zAcc)); //Approximation of signal angle in radians based on aerospace rotation sequence
          pitch=180/PI*(atan(-xAcc/sqrt(pow(yAcc,2)+pow(zAcc,2))));
          //println(pitch); //prints pitch angle in degrees
          //println(signal); //prints signal (supination/pronation or flexion/extensionsignal) angle in degrees
  
  
  //Check current signal angle against thresholds
      if ((signal < leftThresh || signal>rightThresh) && yAcc<0) { //This OR statement is simply to adress the fact the pronation past 90degrees should still count.  yAcc<0 excludes actual supination
        //println("Pronated");
        rxheading[1] = 6; // CURRENTLY CORRESPONDS to RIGHT HEADER
      } else if (signal > rightThresh && zAcc>0) {
        //println("Supinated");
        rxheading[1] = 3; // CURRENTLY CORRESPONDS TO LEFT HEADER
      } else {
        //println("neutral");
        rxheading[1] = 0; // CURRENTLY CORRESPONDS TO LEFT HEADER
      }
  }
}

void serialEvent (Serial myPort) { //called automatically whenever data from serial port is available
  
  // Read string until carriage return and save as accelString
  String accelString = myPort.readStringUntil('\n'); //defines accelString as a single line of output from the terminal

//Parsing
  if (accelString != null) {
    try {
      float[] accelVals = float(split(accelString, ',')); //splits line based on comma delimiter
//      println(accelVals);
//      println(mode);
//      println(mode);
//      println(controlmode_toggle); 
      if (!Float.isNaN(accelVals[0])) { //If a value for the acceleration was output 
        if (mode==4) { //If we are using adcplay
          pot_vals.append(accelVals[0]);
          x_vals.append(accelVals[1]);
          y_vals.append(accelVals[2]);
          z_vals.append(accelVals[3]);
//          println(pot_vals);
        } else {
            if (!controlmode_toggle) {
              pot_vals.append(accelVals[0]);
//              println(pot_vals);
          } else {
              x_vals.append(accelVals[0]);
              y_vals.append(accelVals[1]);
              z_vals.append(accelVals[2]);
//              println(x_vals);
          }
       }
          
        i = i + 1;   //Increments list index if actual acceleration values were appended
        if (mode==1) {
          if (!controlmode_toggle) {
            neutralSignal=round((pot_vals.get(i-1)*0.089433+29.72776)-180);
            println("Neutral");
            println(neutralSignal);
            t_neutral.setValue(str(neutralSignal));
          } else {
            neutralX[j]=x_vals.get(i-1);
            neutralY[j]=y_vals.get(i-1);
            neutralZ[j]=z_vals.get(i-1);
            if (j==9) { //if all 10 samples have been collected
              float[] neutralAvg={Descriptive.mean(neutralX), Descriptive.mean(neutralY), Descriptive.mean(neutralZ)};
              float[] neutralAcc={(neutralAvg[0]-xZero)*Scale, (neutralAvg[1]-yZero)*Scale, (neutralAvg[2]-zZero)*Scale};
  //            neutralSignal=atan(neutralAcc[1]/(neutralAcc[2]/abs(neutralAcc[2])*sqrt(pow(neutralAcc[2],2)+.01*pow(neutralAcc[0],2)))); //Approximation of signal angle in radians based on corrected aerospace rotation sequence
              neutralSignal=round(180/PI*(atan(neutralAcc[1]/neutralAcc[2]))); //uncorrected aerospace 
  //            neutralSignal=atan(neutralAcc[1]/sqrt(pow(neutralAcc[0],2)+pow(neutralAcc[2],2))); //Approximation of signal angle in radians based on aerospace rotation sequence 
              t_neutral.setValue(str(neutralSignal));
              println(neutralSignal);
            } 
          }
          j=j+1;
        } else if (mode==2) {
          if (!controlmode_toggle) {
            leftThresh=round((pot_vals.get(i-1)*0.089433+29.72776)-180);
            println("Left Thresh:");
            println(leftThresh);
            t_left_thresh.setValue(str(leftThresh));
          } else {
            leftSamplesX[j]=x_vals.get(i-1);
            leftSamplesY[j]=y_vals.get(i-1);
            leftSamplesZ[j]=z_vals.get(i-1);
            if (j==9) { //if all 10 samples have been collected
              float[] proAvg={Descriptive.mean(leftSamplesX), Descriptive.mean(leftSamplesY), Descriptive.mean(leftSamplesZ)};
              float[] proAcc={(proAvg[0]-xZero)*Scale, (proAvg[1]-yZero)*Scale, (proAvg[2]-zZero)*Scale};
              leftThresh=round(180/PI*(atan(proAcc[1]/proAcc[2]))); //uncorrected aerospace rotation sequence
              t_left_thresh.setValue(str(leftThresh));
              println(leftThresh);
            }
          }
          j=j+1;
        } else if (mode==3) {
          if (!controlmode_toggle) {
            rightThresh=round((pot_vals.get(i-1)*0.089433+29.72776)-180);
            println("Right Thresh");
            println(rightThresh);
            t_right_thresh.setValue(str(rightThresh));
          } else {
            rightSamplesX[j]=x_vals.get(i-1);
            rightSamplesY[j]=y_vals.get(i-1);
            rightSamplesZ[j]=z_vals.get(i-1);
            if (j==9) { //if all 10 samples have been collected
              float[] supAvg={Descriptive.mean(rightSamplesX), Descriptive.mean(rightSamplesY), Descriptive.mean(rightSamplesZ)};
              float[] supAcc={(supAvg[0]-xZero)*Scale, (supAvg[1]-yZero)*Scale, (supAvg[2]-zZero)*Scale};
              rightThresh=round(180/PI*(atan(supAcc[1]/supAcc[2]))); //uncorrected aerospace
              t_right_thresh.setValue(str(rightThresh));
              println(rightThresh);
            }
          }
          j=j+1;
        } else if (mode==4) {
          collectDynamic();
        }
      }
      
    }    catch(Exception e) {
      //println(e);
    }
  }

}
  
/** 
 * The function start_serial initializes the selected port
 *
 */
void start_serial(String[] port_list)
{  
  port_setup = true;
  println(port_list[port_number]);
  try {
    port = new Serial(this, port_list[port_number], 115200);
    //port.bufferUntil(',');
  }
  catch(RuntimeException e) {
    port_setup = false;
    port_selected = false;
    // This color does not update...
    t_port_status.setColorValue(0xFF0000);
    t_port_status.setValue("Error: cannot connect to port");
  }
  
  // Update port settings
  if (port_setup == true) {
    t_port_status.setColorValue(0x030303);
    t_port_status.setValue("Connected to port " + port_list[port_number]);
    // Send a carriage return to the port because sometimes it stops printing...
    port.write(13);
  }
}

/** 
 * GUI features
 *
 */
/* Dropdownlist event */
public void controlEvent(ControlEvent theEvent)
{
  //println(theEvent.getController().getName());
  
  if (theEvent.isGroup())
  {
    // Store the value of which box was selected
    float p = theEvent.group().value();
    port_number = int(p);
    println(port_number);
    // TODO: stop the serial connection and start a new one
    port_selected = true;
  }
}

/* Function emulator_on */
public void emulator_on(int theValue)
{
  emulator_on_toggle = !emulator_on_toggle;
  //println(emulator_on_toggle);
  cp5.controller("emulator_on").setCaptionLabel((emulator_on_toggle == true) ? "ON":"OFF");
  //println("on ?:" + emulator_on_toggle);
  if (emulator_on_toggle)  {
    // if the emulator is turned ON, send a keystroke to the serial port in case data has paused...
    mode=4;
    port.write(13);
    j=0;
    port.write("adcplay"); //Tells controller to collect data continuously
    port.bufferUntil('\n');
    port.write("\n");
    println(emulator_on_toggle);

  }
  if (!emulator_on_toggle) {
    mode=5; //stops collectDynamic()
    j=0; //starts j over to be incremented with each iteration of serialEvent()
    port.write("stop");
    port.bufferUntil('\n'); 
    port.write("\n");
  }
}

/* Function controlmode */
public void controlmode(int theValue) {
  if (emulator_on_toggle) { //If emulator is currently on, turn it off and change the label
    emulator_on_toggle = !emulator_on_toggle;
    mode=5; //stops collectDynamic()
    cp5.controller("emulator_on").setCaptionLabel((emulator_on_toggle == true) ? "ON":"OFF");
  }
  controlmode_toggle = !controlmode_toggle;
  cp5.controller("controlmode").setCaptionLabel((controlmode_toggle==false) ? "WRIST":"FOREARM");
  println(controlmode_toggle);
}

/* Function manual_left_thresh */
public void manual_left_thresh(float theValue)
{
  t_left_thresh.setValue("" + theValue);
}

/* Fuction manual_right_thresh */
public void manual_right_thresh(float theValue)
{
  t_right_thresh.setValue("" + theValue);
}

/* Function collect_left */
public void collect_left(int theValue)
{
  if (emulator_on_toggle) { //If emulator is currently on, turn it off and change the label
    emulator_on_toggle = !emulator_on_toggle;
    cp5.controller("emulator_on").setCaptionLabel((emulator_on_toggle == true) ? "ON":"OFF");
    mode=5;
  }
  mode=2;
  j=0; //starts j over to be incremented with each iteration of serialEvent()
  if (!controlmode_toggle) {
    port.write("adcsam pot 5");
    port.bufferUntil('\n');
    port.write("\n");
  } else {
    port.write("adcaccel 10 100");
    port.bufferUntil('\n'); 
    port.write("\n");
    println("Pronation Samples (y):");
  }
}

/* Function collect_right */
public void collect_right(int theValue)
{
  if (emulator_on_toggle) { //If emulator is currently on, turn it off and change the label
    emulator_on_toggle = !emulator_on_toggle;
    cp5.controller("emulator_on").setCaptionLabel((emulator_on_toggle == true) ? "ON":"OFF");
    mode=5;
  }
  mode=3; //
  j=0; //starts j over to be incremented with each iteration of serialEvent()
  if (!controlmode_toggle) {
    port.write("adcsam pot 5");
    port.bufferUntil('\n');
    port.write("\n");
  } else {
    port.write("adcaccel 10 100");
    port.bufferUntil('\n'); 
    port.write("\n");
    println("Supination Samples (y):");
  }
}

/* Function collect_neutral */
public void collect_neutral(int theValue)
{
  if (emulator_on_toggle) { //If emulator is currently on, turn it off and change the label
    emulator_on_toggle = !emulator_on_toggle;
    cp5.controller("emulator_on").setCaptionLabel((emulator_on_toggle == true) ? "ON":"OFF");
    mode=5;
  }
  mode=1; //
  j=0; //starts j over to be incremented with each iteration of serialEvent()
    if (!controlmode_toggle) {
    port.write("adcsam pot 5");
    port.bufferUntil('\n');
    port.write("\n");
  } else {
    port.write("adcaccel 10 100");
    port.bufferUntil('\n'); 
    port.write("\n");
    println("Neutral Samples (y):");
  }
}


/* Function keyboard_mouse */
public void keyboard_mouse(int theValue)
{
  keyboard_mouse_toggle = !keyboard_mouse_toggle;
  cp5.controller("keyboard_mouse").setCaptionLabel((keyboard_mouse_toggle == true) ? "Keyboard":"Mouse");
  //println("keyboard ?:" + keyboard_mouse_toggle);
}

/* Function in_x_min text field */
public void in_x_min(float theValue)
{
  //cp5.controller("in_x_min").setValue("" + theValue);
  t_in_x_min.setValue("" + theValue);
}

/* Function in_x_max text field */
public void in_x_max(float theValue)
{
  //cp5.controller("in_x_max").setValue("" + theValue);
  t_in_x_max.setValue("" + theValue);
}

/* Setup the DropdownList */
void customize_dropdownlist(DropdownList ddl)
{
  //
  ddl.setBackgroundColor(color(200));
  ddl.setItemHeight(20*guiScale);
  ddl.setBarHeight(20*guiScale);
  ddl.captionLabel().set("Select COM port");
  ddl.captionLabel().style().marginTop = 3;
  ddl.captionLabel().style().marginLeft = 3;
  ddl.valueLabel().style().marginTop = 3;
  
  // Store the serial port list in the string port_list (char array)
  port_list = port.list();
  
  for (int i = 0; i < port_list.length; i++) {
    ddl.addItem(port_list[i], i);
  }
  
  ddl.setColorBackground(color(60));
  ddl.setColorActive(color(255, 128));
}
