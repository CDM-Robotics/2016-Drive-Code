package org.usfirst.frc.team6072.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;     // Not used
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

import edu.wpi.first.wpilibj.VictorSP;        // PWM Motor Drive for VICTOR SP Modulule
//import edu.wpi.first.wpilibj.Victor;          
import edu.wpi.first.wpilibj.can.*;           // CAN Communication, probably not used
import edu.wpi.first.wpilibj.Compressor;      // Compressor and Pressure Switch on PCM
import edu.wpi.first.wpilibj.Solenoid;        // Solenoid Control on PCM
import edu.wpi.first.wpilibj.buttons.*;       // Joystick button
import edu.wpi.first.wpilibj.Relay;           // Relay output
import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj.DigitalInput;    // Digital Input on RIO
import edu.wpi.first.wpilibj.DigitalOutput;   // Digital Output on RIO 

import edu.wpi.first.wpilibj.CameraServer;    // USB Camera
import com.ni.vision.NIVision;                // USB Camera
import com.ni.vision.NIVision.DrawMode;       // Draw image on camera screen
import com.ni.vision.NIVision.Image;          // Image
import com.ni.vision.NIVision.ShapeMode;      // Draw shape


import edu.wpi.first.wpilibj.GenericHID;      // Not used



/*******************************************************************************************
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 *******************************************************************************************/
public class Robot extends IterativeRobot 
{
	//////////////////////////////////////////////////////////////////////
	/////////// CLASSES //////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////
	
	///////// JOYSTICK ////////////////////////
	Joystick myJoystick;           
	/////// JOYSTICK BUTTONS 1-12 /////////////
	JoystickButton myJoystickButton1;              
	JoystickButton myJoystickButton2;
	JoystickButton myJoystickButton3;
	JoystickButton myJoystickButton4;
	JoystickButton myJoystickButton5;
	JoystickButton myJoystickButton6;
	JoystickButton myJoystickButton7;
	JoystickButton myJoystickButton8;
	JoystickButton myJoystickButton9;
	JoystickButton myJoystickButton10;
	JoystickButton myJoystickButton11;
	JoystickButton myJoystickButton12;
	////// RELAYS 0-3 ON RIO /////////////////
	Relay                   myRelay0;              
	Relay                   myRelay1;  
	Relay                   myRelay2;  
	Relay                   myRelay3;  
    ////// DIGITAL I/O ON RIO /////////////////
	// DIGITAL I/O 0 TO 9
	DigitalInput            myDigitalInput0;
	DigitalOutput           myDigitalOutput1;
	
    ////// VictorSP speed control ////////////
	private SpeedController myLeftMotor;           
	private SpeedController myRightMotor;      
	///// Compressor & on PCM ////////////////
	private Compressor      myCompressor; 
	//// SOLENOIDS - PNEUMATIC ON PCM ////////
	private Solenoid        mySolenoid_Extend;     
	private Solenoid        mySolenoid_Retract;    
	//// CAMERA //////////////////////////////
	CameraServer server;
	int session;
    Image frame;
    
	//////////////////////////////////////////////////////////////////////
	////////// PNEUMTIC DEFINITIONS //////////////////////////////////////
	//////////////////////////////////////////////////////////////////////
	// TO BE USED WITH controlPneumatic()
	// INPUT:
	public static final int PNEUMATIC_OFF     = 0;         //STOP
	public static final int PNEUMATIC_RETRACT = 1;         //RETRACT
	public static final int PNEUMATIC_EXTEND  = 2;         //EXTEND
	// OUTPUT:
	public static final int PNEUMATIC_STATUS_DONE  = 0;    //DONE
	public static final int PNEUMATIC_STATUS_BUSY  = 1;    //BUSY
	public static final int PNEUMATIC_STATUS_ERROR = 2;    //ERROR
	// CURRENT POSITION
	public static final int PNEUMATIC_POSITION_UNKNOWN    = 0;
	public static final int PNEUMATIC_POSITION_RETRACTED  = 1;
	public static final int PNEUMATIC_POSITION_EXTENDED   = 2;
	int Pneumatic_Position =  PNEUMATIC_POSITION_UNKNOWN;
	// PRESSURE
	public static final boolean PRESSURE_LOW   = true;
	public static final boolean PRESSURE_HIGH  = false;
	
    ////////// MOTOR DEFINITION DEFINITIONS ///////////////////////////////
	public static final int ROBOT_STOP          = 0;
	public static final int ROBOT_FWD           = 1;
	public static final int ROBOT_REV           = 2;
	public static final int ROBOT_180_CW        = 3;
	public static final int ROBOT_180_CCW       = 4;
	public static final int ROBOT_SQUARE1       = 5;
	public static final int ROBOT_SQUARE2       = 6;
	public static final int ROBOT_SQUARE3       = 7;
	public static final int ROBOT_SQUARE4       = 8;
	
	////////// VARIABLES ///////////////////////////////////////////////////
	int RobotState    = ROBOT_STOP;
	int OldRobotState = ROBOT_STOP;
	boolean CameraStatus = false;
	
	
	
	/******************************************************************************/
    /* This function is run when the robot is first started up and should be      */
    /* used for any initialization code.                                          */
    /******************************************************************************/
    public void robotInit() {
    	
    	//We are not using this driver from the example
    	//myRobot = new RobotDrive(0,1);  
    	
    	// initialize left motor as VictorSP PWM1 
    	myLeftMotor        = new VictorSP(1);     
    	// initialize right motor as VictorSP PWM2
    	myRightMotor       = new VictorSP(2);   
    	
    	// initialize compressor on PCM. CAN ID IS 1.
    	myCompressor       = new Compressor(1);   
    	// initialize extend solenoid on PCM output 1
    	mySolenoid_Extend  = new Solenoid(1);
    	// initialize retract solenoid on PCM output 0
    	mySolenoid_Retract = new Solenoid(0);
    	
    	// initialize relays if needed
    	myRelay0            = new Relay(0);
    	myRelay1            = new Relay(1);
    	myRelay2            = new Relay(2);
    	myRelay3            = new Relay(3);
    	
    	// initialize Digital I/O on RIO
    	myDigitalInput0     = new DigitalInput(0);
    	myDigitalOutput1    = new DigitalOutput(1);
    	
    	// initialize Camera
    	frame = NIVision.imaqCreateImage(NIVision.ImageType.IMAGE_RGB, 0);
        // the camera name (ex "cam0") can be found through the roborio web interface
        session = NIVision.IMAQdxOpenCamera("cam0",NIVision.IMAQdxCameraControlMode.CameraControlModeController);
        NIVision.IMAQdxConfigureGrab(session);
        
    	
    	// initialize joystick. 
    	// NUMBER MUST MATCH NUMBER ORDER (0) ON FRC DRIVER STATION
    	myJoystick         = new Joystick(0);   
    	
    	// initialize buttons
    	myJoystickButton1   = new JoystickButton(myJoystick,1);
    	myJoystickButton2   = new JoystickButton(myJoystick,2);
    	myJoystickButton3   = new JoystickButton(myJoystick,3);
    	myJoystickButton4   = new JoystickButton(myJoystick,4);
    	myJoystickButton5   = new JoystickButton(myJoystick,5);
    	myJoystickButton6   = new JoystickButton(myJoystick,6);
    	myJoystickButton7   = new JoystickButton(myJoystick,7);
    	myJoystickButton8   = new JoystickButton(myJoystick,8);
    	myJoystickButton9   = new JoystickButton(myJoystick,9);
    	myJoystickButton10  = new JoystickButton(myJoystick,10);
    	myJoystickButton11  = new JoystickButton(myJoystick,11);
    	myJoystickButton12  = new JoystickButton(myJoystick,12);
    	
    	System.out.print("initializing system... \n");
        System.out.format("initializing system... \n");
    }
    
    /******************************************************************************/
    /* MY ROBOT MAIN APP - MODES: autonomous & tele-op        *********************/
    /******************************************************************************/
    public void myRobotApp()
    {
    	    
    	
    	 
    	    
    	    //////// Some delay so hardware has time to respond ///
		    Timer.delay(0.2); // 0.2 second
		    
	    	////// Process Camera //////////////////////////////////
		    processUSBCamera();
		    
    		////// Compressor ON //////////////////////////////////
    		controlCompressor(true);
            
            ///////////// Shoot ball////////// ////////////////////
            shootBall();

            //////////// Drive Robot //////////////////////////////
            driveRobot();
    		
            //////////// Example Relay output control //////////////
            controlRelay();
            
            ////////////Example digital I/O on RIO ///////////////////
            controlDigitalIO();
            
    }
    
    /******************************************************************************/
    /* This function handle USB CAMERA       **************************************/
    /******************************************************************************/
	public void processUSBCamera()
	{
		//MUST SELECT CAMERA_HW ON DRIVER DASHBOARD
	   
			   //start reading image
			   NIVision.IMAQdxStartAcquisition(session);
		  
			 
			   //stop reading image
			   //NIVision.IMAQdxStopAcquisition(session);
		
	
        /**
         * grab an image, draw the circle, and provide it for the camera server
         * which will in turn send it to the dashboard.
         */
         //   NIVision.Rect rect = new NIVision.Rect(250, 250 , 100, 100);
            NIVision.IMAQdxGrab(session, frame, 1);
         //   NIVision.imaqDrawShapeOnImage(frame, frame, rect,
         //   DrawMode.DRAW_VALUE, ShapeMode.SHAPE_OVAL, 0.0f);
            CameraServer.getInstance().setImage(frame);
            NIVision.IMAQdxStopAcquisition(session);
	 
	}
    
    /******************************************************************************/
    /* This function handle RIO Relays       **************************************/
    /******************************************************************************/
	public void controlDigitalIO()
	{
	   
		//// example code
		//// if digital input0 is 0, digital output1 is 0
		//// if digital input0 is 1, digital output1 is 1
		if ( myDigitalInput0.get() == true)
		{
			 myDigitalOutput1.set(true);
		}
		else myDigitalOutput1.set(false);
		
	}
    
    /******************************************************************************/
    /* This function handle RIO Relays       **************************************/
    /******************************************************************************/
	public void controlRelay()
	{
	    //// Relay1 output if used on RIO
        // myRelay1.set(Value.kOn);      // SET RELAY1 On-On
        // myRelay1.set(Value.kOff);     // SET RELAY1 Off-Off
        // myRelay1.set(Value.kForward); // SET RELAY1 On-Off
        // myRelay1.set(Value.kReverse); // SET RELAY1 Off-On
		
		//// example code, if button 7 pressed, turn on relay1
		if (myJoystickButton7.get() == true)
		{
			myRelay1.set(Value.kOn);
		}
		else myRelay1.set(Value.kOff);
		
	}
	
	/******************************************************************************/
    /* This function handle compressor power **************************************/
    /******************************************************************************/
	// Input: false is off; true is on
	public void controlCompressor(boolean CompOnOff)
	{
      
      if (CompOnOff == true)
      {
    	//Set closed loop control.
    	//Once this is set true, PCM hardware should turn on/off based on pressure
        myCompressor.setClosedLoopControl(true);
        myCompressor.start();
       
      }
      else
      {
    	//force stop
    	myCompressor.stop();
      }
      
     //myCompressor.getPressureSwitchValue()
      
	}
	
	/******************************************************************************
     * This function to shoot the ball  
	 * Return true if it's done.
	 * Return false if pressure it not high enough
    ******************************************************************************/
	public boolean shootBall()
	{
	  int iCount = 0;
	  int iDelay = 0;
		
	  /////// If button 1 pressed, shoot the ball /////////////////
	  if (myJoystickButton1.get() == true)
	  {	
		//// WAIT FOR PRESSURE ////////////////////
		if (  myCompressor.getPressureSwitchValue() == PRESSURE_HIGH)
		{
			
			
		    System.out.print("Shoot Ball!  \n");
    	 	System.out.format("Shoot Ball! \n");
			
			/// EXTEND
			controlPneumatic(PNEUMATIC_EXTEND);
			//WAIT 2 SECONDS, 
			for (iDelay = 0; iDelay < 20; iDelay++)
				{ driveRobot();      //keep driving while waiting...
			     Timer.delay(0.1); //0.1 second
				}
			processUSBCamera();//keep looking while waiting...
			
			/// TOGGLE A FEW TIMES UNTIL COMPRESSOR ON AGAIN
			/// DOING THIS WILL PRESSURIZE THE COMPRESSOR
			/// FOR NEXT EVENT
			
			for (iCount = 0; iCount < 2; iCount++)
			{
			controlPneumatic(PNEUMATIC_RETRACT);
			//WAIT 1 SECOND
			for (iDelay = 0; iDelay < 10; iDelay++)
			{driveRobot();      //keep driving while waiting...
		     Timer.delay(0.1);
			}
			processUSBCamera();//keep looking while waiting... 
			controlPneumatic(PNEUMATIC_EXTEND);
			//WAIT 1 SECOND
			for (iDelay = 0; iDelay < 10; iDelay++)
			{driveRobot();      //keep driving while waiting...
		     Timer.delay(0.1);
			} 
			processUSBCamera();//keep looking while waiting... 
			
			}
			
			//// MAKE SURE TO RETRACT IT
			controlPneumatic(PNEUMATIC_RETRACT);
			//WAIT 2 SECONDS
			for (iDelay = 0; iDelay < 20; iDelay++)
			{driveRobot();      //keep driving while waiting...
		     Timer.delay(0.1);
			} 
		
			controlPneumatic(PNEUMATIC_OFF);
			
			
			return true;
			
		}
		
		//PRESSURE WAS LOW
		else
		{
			 System.out.print("Low pressure! Can't shoot!  \n");
	    	 System.out.format("Low pressur! Can't shoot! \n");
	    	 	
			 return false;
		}
		
	  }
	  else return false;
		
		
	}
	
	
	
	/******************************************************************************
     * This function handle pneumatic  
	 * Input direction 1 for retract, 2 for extend, 0 for Stop
    ******************************************************************************/
	public void controlPneumatic(int Direction)
	{
		////////////// RETRACT ////////////////////
		if (Direction == PNEUMATIC_RETRACT)
		{
			  mySolenoid_Extend.set(true);
			  mySolenoid_Retract.set(false);	
			  Pneumatic_Position =  PNEUMATIC_POSITION_RETRACTED;
			  
		}
		///////////// EXTEND ////////////////////
		else if (Direction == PNEUMATIC_EXTEND)
		{
			
			mySolenoid_Extend.set(false);
			mySolenoid_Retract.set(true);
			Pneumatic_Position =  PNEUMATIC_POSITION_EXTENDED;
		}
		//////////// STOP ///////////////////////
		else
		{
			mySolenoid_Extend.set(false);
			mySolenoid_Retract.set(false);
			
		}
	}
	
	
	
	/******************************************************************************/
    /* This function drives robot based on joystick  ******************************/
	/* Input: x, y ****************************************************************/
    /******************************************************************************/
	public void driveRobot()
	{
		double XAxis = 0;
		double YAxis = 0;
	
		//////////////////////////////////////////////////////////////////////////
		//////// READ JOYSTICK COORDINATE ////////////////////////////////////////
		//////////////////////////////////////////////////////////////////////////
  	    XAxis = myJoystick.getX(); 
  	    YAxis = myJoystick.getY(); 
  	    
  	    ///////////////////////////////////////////////////////////////////////////
  	    /////// CHECK FOR ERROR ///////////////////////////////////////////////////
  	    ///////////////////////////////////////////////////////////////////////////
		if (XAxis > 1)
			XAxis = 1;
		else;
		if (YAxis > 1)
			YAxis = 1;
		else;
		if (XAxis < - 1)
			XAxis = -1;
		else;
		if (YAxis < - 1)
			YAxis = - 1;
		else;
		
	
		
		///////////////////////////////////////////////////////////////////////////////
		//////////// If joystick moved, determine next robot state ////////////////////
		///////////////////////////////////////////////////////////////////////////////
		if ( (XAxis > 0.2) || (XAxis < (-0.2)) || (YAxis > 0.2) || (YAxis < (-0.2)) )
		{
		  //System.out.format("X Coordinate: " + XAxis + " Y Coordinate: " + YAxis +"\n");
			
           ///////////////////////////////////////////////////////////
		   // +X AXIS ONLY; DO 180 CLOCKWISE /////////////////////////
		   ///////////////////////////////////////////////////////////
           if ( (XAxis>(0.2)) && ( (YAxis>(-0.2)) && (YAxis<0.2)) )
           {
        	    if ( (OldRobotState == ROBOT_180_CW) || (OldRobotState == ROBOT_STOP))
  	    	          RobotState = ROBOT_180_CW;
        	    else  RobotState = ROBOT_STOP;
           }
           ///////////////////////////////////////////////////////////
           // -X AXIS ONLY; DO 180 CCW ////////////////////////////////
           ///////////////////////////////////////////////////////////
           else if ( (XAxis<(-0.2)) && ( (YAxis>(-0.2)) && (YAxis<0.2)) )
           {
        	   if ( (OldRobotState == ROBOT_180_CCW) || (OldRobotState == ROBOT_STOP))
   	                 RobotState = ROBOT_180_CCW;
 	           else  RobotState = ROBOT_STOP;
           }
           
           ///////////////////////////////////////////////////////////
           // Y AXIS ONLY,  FORWARD //////////////////////////////////
           ///////////////////////////////////////////////////////////
           else if (  (YAxis < (-0.2)) && ( (XAxis>(-0.2)) && (XAxis<0.2)) )
           {
        	   
        	   if ( (OldRobotState == ROBOT_FWD) || (OldRobotState == ROBOT_STOP))
   	                 RobotState = ROBOT_FWD;
 	           else  RobotState = ROBOT_STOP;  
           }
			
           ///////////////////////////////////////////////////////////
           // Y AXIS ONLY,  REVERSE //////////////////////////////////
           ///////////////////////////////////////////////////////////
           else if (  (YAxis >0.2) && ( (XAxis>(-0.2)) && (XAxis<0.2)) )
           {
        	   if ( (OldRobotState == ROBOT_REV) || (OldRobotState == ROBOT_STOP))
   	                 RobotState = ROBOT_REV;
 	           else  RobotState = ROBOT_STOP;   
           }
           
           ///////////////////////////////////////////////////////////
		   /////////////////////SQURARE1 /////////////////////////////
           ///////////////////////////////////////////////////////////
           else if ( (XAxis>0.2) && (YAxis<(-0.2)) )
			{
        	   if ( (OldRobotState == ROBOT_SQUARE1) || (OldRobotState == ROBOT_STOP))
   	                 RobotState = ROBOT_SQUARE1;
 	           else  RobotState = ROBOT_STOP;
			}
				
			///////////////////////////////////////////////////////////
            /////////////////////SQURARE2 /////////////////////////////
			///////////////////////////////////////////////////////////
		   else if ( (XAxis<(-0.2)) && (YAxis<(-0.2)) )
	       {
			   if ( (OldRobotState == ROBOT_SQUARE2) || (OldRobotState == ROBOT_STOP))
 	                 RobotState = ROBOT_SQUARE2;
	           else  RobotState = ROBOT_STOP;
			}
		   ///////////////////////////////////////////////////////////
           /////////////////////SQURARE3 /////////////////////////////
		   ///////////////////////////////////////////////////////////
		   else if ( (XAxis<(-0.2)) && (YAxis>(0.2)) )
           {
			   if ( (OldRobotState == ROBOT_SQUARE3) || (OldRobotState == ROBOT_STOP))
 	                 RobotState = ROBOT_SQUARE3;
	           else  RobotState = ROBOT_STOP;
			}
			
		   ///////////////////////////////////////////////////////////
           /////////////////////SQURARE4 /////////////////////////////
		   ///////////////////////////////////////////////////////////
		   else if ( (XAxis>0.2) && (YAxis>0.2)) 
           {
			   if ( (OldRobotState == ROBOT_SQUARE4) || (OldRobotState == ROBOT_STOP))
 	                 RobotState = ROBOT_SQUARE4;
	           else  RobotState = ROBOT_STOP;
			}
		   else
		   {
			   RobotState = ROBOT_STOP;
		   }
			 
		}//END JOYSTICK MOVED
		else
		{
			RobotState = ROBOT_STOP;
		}
		
		
		///////////////////////////////////////////////////////////////////////////////
		////////// ROBOT STATE ////////////////////////////////////////////////////////
		///////////////////////////////////////////////////////////////////////////////
		switch (RobotState)
		{
		  default:
		  case ROBOT_STOP:
			   myLeftMotor.set(0);
		       myRightMotor.set(0);
		       //pause 0.5 second every time a coordinate is changed
		       //for motor health
		       if (OldRobotState != RobotState)
		           Timer.delay(0.5);
		       else;
			   OldRobotState = RobotState;
	          break;
	      
		  case ROBOT_180_CW:
			  myLeftMotor.set(XAxis*(-1));
  	          myRightMotor.set(XAxis);
			  OldRobotState = RobotState;
		      break;
		      
		  case ROBOT_180_CCW:
			  myLeftMotor.set(XAxis*(-1));
       	      myRightMotor.set(XAxis);
			  OldRobotState = RobotState;
		      break;
		      
		  case ROBOT_FWD:
			  myLeftMotor.set(YAxis);
       	      myRightMotor.set(YAxis);
			  OldRobotState = RobotState;
		      break;
		      
		  case ROBOT_REV:
			  myLeftMotor.set(YAxis);
       	       myRightMotor.set(YAxis);
			  OldRobotState = RobotState;
		      break;
		      
		  case ROBOT_SQUARE1:
			   //keep left wheel moving 
		       myLeftMotor.set(YAxis);      
		       //reduce speed on right wheel
		       myRightMotor.set(YAxis*(1-XAxis)); 
			  OldRobotState = RobotState;
		      break;
		      
		  case ROBOT_SQUARE2:
			   //Keep right wheel moving 
	    	   myRightMotor.set(YAxis); 
	    	   //Reduce speed on left wheel
		       myLeftMotor.set(YAxis*(1+XAxis));
			   OldRobotState = RobotState;
		      break;
		      
		  case ROBOT_SQUARE3:
			  //Keep right wheel moving 
       	      myRightMotor.set(YAxis);     
       	      //Reduce speed on left wheel
		      myLeftMotor.set(YAxis*(1+XAxis));     
			  OldRobotState = RobotState;
			  break;
    
		  case ROBOT_SQUARE4:
			   //keep left wheel moving 
       	       myLeftMotor.set(YAxis); 
       	       //reduce speed on right wheel
		       myRightMotor.set(YAxis*(1-XAxis)); 
		       OldRobotState = RobotState;
			
			  break;
		}//END SWITCH
			
	}///////////////////////////////////////////////////////////////////////////////
	
	
	/******************************************************************************/
    /* This function is run once each time the robot enters autonomous mode       */
    /******************************************************************************/
    public void autonomousInit() {
    	System.out.print("initializing autonomousInit \n");
    	System.out.format("initializing autonomousInit \n");	
    }
   /******************************************************************************/
   /* This function is called periodically during autonomous                     */
   /******************************************************************************/
	public void autonomousPeriodic() {
		
		
		while ( isAutonomous() && isEnabled() )
        {
          
		  myRobotApp();
        }
    	
   }
    
	
    /******************************************************************************/
    /* This function is called once each time the robot enters tele-operated mode */
    /******************************************************************************/
    public void teleopInit(){
    	
    	System.out.print("initializing teleop \n");
    	System.out.format("initializing teleop \n");
    }

    /******************************************************************************/
    /* This function is called periodically during operator control               */
    /******************************************************************************/
    public void teleopPeriodic() {
       //watch out for arcad drive - system crash  myRobot.arcadeDrive(stick);
        System.out.print("teleoperiodic \n");
        System.out.format("teleoperiodic \n");
        
       
        while (isOperatorControl() && isEnabled() )
        {
          
		  myRobotApp();
        }
    }
    
    
    
    /******************************************************************************/
    /* This function is called periodically during test mode                      */
    /******************************************************************************/
    public void testPeriodic() {
    	System.out.print("livewindow \n ");
    	System.out.format("livewindow \n ");
    	
    	LiveWindow.run();
    }
    
}
