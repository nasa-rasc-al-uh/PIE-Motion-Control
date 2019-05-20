#/ Controller version = 2.60
#/ Date = 5/18/2019 1:27 PM
#/ User remarks = 
#0
AUTOEXEC:

machineState = 0

opControlState = 10
coreState = 100
extractState = 150
drillState = 200
filterState = 250
running = 1

ECAT_OFFSET = 296

ECOUT(292, ECAT_OUT2);
ECOUT(294, ECAT_OUT3);

ECAT_OUT2 = 7910
ECAT_OUT3 = 7910


ECIN(309, ECAT_IN1);

xOffset = 177.64
currentHomingAxis = -1
isXHomed = 0
isZ1Homed = 0
isZ2Homed = 0

xAxisGroup(0) = X1
xAxisGroup(1) = X2

drillVel = VEL(Drill)*60
drillAccel = ACC(Drill)*60
drillJerk = JERK(Drill)*60

Z1TravelVel = VEL(Z1)
Z1Accel = ACC(Z1)
Z1Jerk = JERK(Z1)

Z2TravelVel = VEL(Z2)
Z2Accel = ACC(Z2)
Z2Jerk = JERK(Z2)

XVel = VEL(X1)
XAccel = ACC(X1)
XJerk = JERK(X1)

!On start up we set commutation status to active 
MFLAGS(Z1).#BRUSHOK = 1
WAIT 100
MFLAGS(Drill).#BRUSHOK = 1
ENABLE Z1
ENABLE Drill
!MFLAGS(X2).#DEFCON=0
!CONNECT RPOS(X2) = APOS(X1)
!DEPENDS X2,X1
!GROUP (X1,X2)
!SAFETYGROUP(X1,X2)
ENABLE X1
ENABLE X2

START 2,1
START 3,1
START 4,1
START 5,1
START 6,1
START 7,1
START 8,1

WHILE running

	if machineState = 0

		! Initialization
		DISP machineState
		machineState = opControlState

	elseif machineState = opControlState

		! Idle/Jog control handling
		
		if advanceState
			DISP machineState
			machineState = 50
			advanceState = 0
		end
		
	elseif machineState = 50
	
		! Init autoun
		
		isZ1Homed = 0
		isZ2Homed = 0
		isXHomed = 0
		
		homeZ1 = 1;
		WAIT 1000
		TILL isZ1Homed = 1
		homeZ2 = 1;
		WAIT 1000
		TILL isZ2Homed = 1
		homeX = 1;
		WAIT 1000
		TILL isXHomed = 1
		
		if advanceState
			DISP machineState
		end
		machineState = drillState
		
	elseif machineState = coreState

		! Coring
		if advanceState
			DISP machineState
		end
		
	elseif machineState = extractState
		
		! Water extraction
		if advanceState
			DISP machineState
		end
	elseif machineState = drillState
		
		! Drilling additional holes
		if advanceState
			DISP machineState
		end
	elseif machineState = filterState
		
		! Filtration
		if advanceState
			DISP machineState
		end
		
	elseif machineState = 300
	
		machineState = 0
		running = 0
	
	end
	
end

if running = 0
	machineState = 0
	DISABLE ALL
end

STOPALL
STOP

#1
! Operator Controlled Buffer
STOP

ON jogX
	if machineState = opControlState
		if isXHomed = 1
			ENABLE xAxisGroup(0)
			ENABLE xAxisGroup(1)
			
			if FPOS(X1) + jogX < 690
				PTP/re xAxisGroup, jogX, jogX
			end
			
			DISABLE xAxisGroup(0) ! Disable axis
			DISABLE xAxisGroup(1)
		end
		
		jogX = 0 ! Reset jog control
	end
RET

ON jogZ1
	if machineState = opControlState
		if isZ1Homed = 1
			ENABLE Z1
			PTP/re Z1, jogZ1
		end
		jogZ1 = 0
		
		DISABLE Z1
	end
RET

ON jogZ2
	if machineState = opControlState
		if isZ2Homed = 1
			ENABLE Z2
			
			if FPOS(Z2) + jogZ2 < 700
				PTP/re Z2, jogZ2
			end
		end
		
		jogZ2 = 0
		
		DISABLE Z2
	end
RET

ON jogDrill

	if machineState = opControlState
		ENABLE Drill
		
		JOG/v Drill, jogDrill/60
	end
RET

ON ^jogDrill

	KILL Drill
	DISABLE Drill
	
RET

ON jogBoreholePump
	ECAT_OUT1.1 = 1;
RET

ON ^jogBoreholePump
	ECAT_OUT1.1 = 0;
RET
		
if advanceState
	machineState = 50
	advanceState = 0
end

#2
! Digital Coring Buffer
local int numCores

WHILE running = 1
IF machineState = coreState
	
	IF numCores <> coreSamples
		DISP "Coring"
	ELSE
		!DISP "Advancing"
		!advanceState = 1
	END
	
	
	IF advanceState
		machineState = extractState
		advanceState = 0
	END
END
END
STOP

#3
! Drilling Buffer

local int localState
localState = 0
	
WHILE running=1
	if machineState = drillState
	
		if localState = 0
			ENABLE X1
			ENABLE X2
			
			PTP/re xAxisGroup, 500, 500 ! Move drill above the overburden
		
			real vel
			vel = Z1DrillVel
			ENABLE Z1
			
			JOG/v Z1, -vel
			WAIT 1000
			TILL RMS(Z1)>touchRMS ! Need to tune this, check for position error above threshold (e.g. until overburden is reached)
			
			localState = 10
			
		elseif localState = 10
			HALT Z1
			if FPOS(Z1) + 50 < 700
				PTP/re Z1, 50 ! Move up 50 mm away from overburden to prepare for drill spin up
			end
			localState = 50
			
		elseif localState = 50
		
			ENABLE Drill
			JOG Drill
			TILL ^MST(Drill).#ACC !Spin up drill and wait until it's done accelerating
			localState = 100
		
		elseif localState = 100
			!Begin pecking
			
			ENABLE Z1
			PTP/rev Z1, (-50 - peckDistance), Z1DrillVel ! First peck after moving back to the surface of the overburden
			PTP/re Z1, retractDistance ! Retract the drill
			localState = 150
		
		elseif localState = 150
		
			if FPOS(Z1) > 5 + 172
				if FPOS(Z1) > (peckDistance+retractDistance) + 172 ! Keep pecking until the last stroke, go to the bottom on the last stroke
					PTP/rev Z1, (-peckDistance - retractDistance), Z1DrillVel ! Peck
					PTP/re Z1, retractDistance ! Retract
				else
					PTP/ev Z1, 2 + 172, Z1DrillVel ! Move to the bottom of the travel
				end
			
			else
				localState = 200
				advanceState = 1
				homeZ1 = 1
				WAIT 1000
				TILL currentHomingAxis = -1
				HALT Drill
				DISABLE Drill
				DISABLE Z1
			end
		
		end
		
		if advanceState
			machineState = extractState
			advanceState = 0;
			localState = 0
		end
	end
END
STOP

#4
! Water Extraction Buffer

local int localState
localState = 0

WHILE running = 1
if machineState = extractState
	
	if localState = 0
		
		ENABLE X1
		ENABLE X2
		
		PTP/re xAxisGroup, xOffset, xOffset ! Move water extractor above the hole
		
		localState = 50;
	elseif localState = 50
		
		ENABLE Z2
		
		PTP/e Z2, 192 ! Move the water extractor down to the base of the hole
		
		localState = 100
		
	elseif localState = 100
	
		pellet1_h1_SetTemp = heaterSetpoint;
		
		WAIT 120000 ! We need to determine how long to wait before we start pumping
		
		localState = 150
		
	elseif localState = 150
		
		ECAT_OUT1.4 = 1; ! Open fill valve
		ECAT_OUT1.0 = 1; ! Turn on extractor pump
		
		WAIT 300000 ! Replace this to wait until the EC box is full TILL AIN0 = ...
		
		ECAT_OUT1.0 = 0; ! Turn off extractor pump
		pellet1_h1_SetTemp = 0;
		advanceState = 1;
		
	end

	if advanceState
		machineState = filterState
		advanceState = 0
	end
end
END
STOP

#5
! Filtration Buffer

local int localState
localState = 0
WHILE running = 1
if machineState = filterState

	if localState = 0
		! Close drain and fill valves
		ECAT_OUT1.3 = 0;
		ECAT_OUT1.4 = 0;
		localState = 50
		
	elseif localState = 50
		! Set amperage setpoint and begin EC process
		ECAT_OUT2 = 16383
		WAIT 900000 ! Wait 15 minutes
		localState = 100
	elseif localState = 100
		ECAT_OUT1.5 = backFlush
		ECAT_OUT1.6 = ~backFlush
		localState = 150
	elseif localState = 150
		ECAT_OUT1.3 = 1 ! Open drain valve
		ECAT_OUT1.1 = 1 ! Turn on the filtration pump
		! TILL AIN0 = ... <- Wait until the EC box is empty
		WAIT 10000	  	! <- Wait until all of the water in the tubes is pumped out
		ECAT_OUT1.1 = 0 ! Turn off the filtration pump
		ECAT_OUT1.3 = 0 ! Close drain valve
		! machineState = extractState ! Revert back to the water extraction state
		localState = 0
		advanceState = 1
	end
	
	if advanceState
		machineState = 300
		advanceState = 0
	end
end
END
STOP

#6
!Back flush monitor

WHILE running=1
! Check the flow rate out of the EC box and set backFlush=1 or 0
! Set the filtration state to 100
END

STOP

#7
while 1
	BLOCK
	framePwr = RMS(Z1)
	drillPwr = RMS(Drill)
	!totalPwr = drillPwr + framePwr;
	drillRPM = FVEL(Drill)*60;
	heatTemp = PELLET1_TC1/10;
	!DISP FAULT(1).#ENC
	
	!VEL(Drill) = drillVel
	!ACC(Drill) = drillAccel
	!JERK(Drill) = drillJerk

	!VEL(Z1) = Z1Vel
 	!ACC(Z1) = Z1Accel
	!JERK(Z1) = Z1Jerk
	END
END

ON persistentChanged

	VEL(Drill) = drillVel/60
	ACC(Drill) = drillAccel/60
	DEC(Drill) = drillAccel/60
	JERK(Drill) = drillJerk/60
	
	VEL(Z1) = Z1TravelVel
	ACC(Z1) = Z1Accel
	DEC(Z1) = Z1Accel
	JERK(Z1) = Z1Jerk
	
	VEL(Z2) = Z2TravelVel
	ACC(Z2) = Z2Accel
	DEC(Z2) = Z2Accel
	JERK(Z2) = Z2Jerk
	
	VEL(X1) = XVel
	VEL(X2) = XVel
	ACC(X1) = XAccel
	ACC(X2) = XAccel
	DEC(X1) = XAccel
	DEC(X2) = XAccel
	JERK(X1) = XJerk
	JERK(X2) = XJerk
	persistentChanged = 0;
	
RET

#8
!Buffer 8 - Temperature PID control loops

WAIT 10
TILL ^PST(9).#RUN ! Wait for buffer 9 to finish
global real bSetPoint, bInputTemp, bOutput, bITerm, bLastInput, bKp, bKi, bKd, bOutputMin, bOutputMax, bError, bDInput
int bed_state 
real pellet1_h1_SetPoint, pellet1_h1_InputTemp, pellet1_h1_Output, pellet1_h1_ITerm, pellet1_h1_LastInput, pellet1_h1_Kp, pellet1_h1_Ki, pellet1_h1_Kd, pellet1_h1_OutputMin, pellet1_h1_OutputMax, pellet1_h1_Error, pellet1_h1_DInput
int pellet1_h1_state, pellet1_h2_state

int sampleTime

real OVERTEMP, UNDERTEMP, SAFETEMP_LOW, SAFETEMP_HIGH
OVERTEMP	= 3300	! Upper safety limit [0.1 C]
UNDERTEMP	= 130	! Lower safety limit [0.1 C] - stop runaway from backwards thermocouple
! Faulted thermocouples must be in safe range to reenable. Must be inside of over/under temp range
SAFETEMP_HIGH	= 2000	! Upper limit for safe range [0.1 C]
SAFETEMP_LOW	= 140	! Lower limit for safe range [0.1 C]

pellet1_h1_SetTemp = 0
pellet1_h1_SetPoint = 0
pellet1_h1_state = 0

sampleTime = 50

pellet1_h1_OutputMin = 0
pellet1_h1_OutputMax = 1000


bOutputMin = 0
bOutputMax = 32767

PELLET1_TC1_OFFSET = 266        !CHANGE LATER
PELLET1_TC1_STATUS_OFFSET = 105  !CHANGE LATER

!Unsurpress this later when the Beckhoff is hooked up 
ECIN(PELLET1_TC1_OFFSET,PELLET1_TC1)
!ECIN(PELLET1_TC1_STATUS_OFFSET,PELLET1_TC1_STATUS)
ECOUT(ECAT_OFFSET,ECAT_OUT1)


if(^PELLET1_TC1_STATUS.6)
		pellet1_h1_state = 1
end

	
STARTPOINT:

ENABLEON
bed_state = 1

pellet1_h1_Kp = 10
pellet1_h1_Ki = 500
pellet1_h1_Kd = 0.5
pellet1_h1_LastInput = PELLET1_TC1 / 10   !bekhoff needs to be divided by 10 
pellet1_h1_ITerm = pellet1_h1_Output

if(pellet1_h1_ITerm > pellet1_h1_OutputMax)
	pellet1_h1_ITerm = pellet1_h1_OutputMax
elseif(pellet1_h1_ITerm < pellet1_h1_OutputMin)
	pellet1_h1_ITerm = pellet1_h1_OutputMin
end


while 1
	
	! E1
	!Heater 1 Block
	if(pellet1_h1_state)
		BLOCK
			
			!Set working variables
			pellet1_h1_SetPoint = pellet1_h1_SetTemp
			
			!Read temperature values
			pellet1_h1_InputTemp = PELLET1_TC1 / 10
			
			!Compute working error variables
			pellet1_h1_Error = pellet1_h1_SetPoint - pellet1_h1_InputTemp
			pellet1_h1_ITerm = pellet1_h1_ITerm + (pellet1_h1_Ki * pellet1_h1_Error)
			
			if(pellet1_h1_ITerm > pellet1_h1_OutputMax)
				pellet1_h1_ITerm = pellet1_h1_OutputMax
			elseif(pellet1_h1_ITerm < pellet1_h1_OutputMin)
				pellet1_h1_ITerm = pellet1_h1_OutputMin
			end
			
			!Compute PID output
			pellet1_h1_Output = (pellet1_h1_Kp * pellet1_h1_Error) + pellet1_h1_ITerm - (pellet1_h1_Kd * pellet1_h1_DInput)
			
			if(pellet1_h1_Output > 0) & (pellet1_h1_SetPoint <> 0)
				if(^ECAT_OUT1.2)       !Unpress these later ECAT_OUT is te variable that controls relay 
					ECAT_OUT1.2 = 1
				end
			else
				if(ECAT_OUT1.2)
					ECAT_OUT1.2 = 0
				end
			end
			
			!Remember lastInput
			pellet1_h1_LastInput = pellet1_h1_InputTemp
		
		END
	end
	
	!Heater 2 Block
	
	wait sampleTime
END
		
STOP



#9
!Buffer 9 

!Ecat setup
!Initialize parameters
!Homing
!Commutation
!Homing X
!Homing Y
!Extrusion formulas 

!Assign initial values to variables
STOP

ON homeAxis.1 = 1 !Z1 Axis
	Disp "ATTEMPTING TO HOME Z1 AXIS"
	if(^MFLAGS(Z1).#BRUSHOK)
		DISP "Z1 Axis is not commutated. Please enable the Z1 axis."
		homeAxis.2 = 0
		ret
	elseif(MST(Z1).#OPEN)
		DISP "Z1 Axis is operating in open-loop control. Please re-enable."
		homeAxis.2 = 0
		ret
    elseif(MFLAGS(Z1).#DEFCON = 0)
		DISP "Z1 Axis slave active. Please ungroup."
		homeAxis.2 = 0
		ret
	else	
		TILL currentHomingAxis = -1 ! Wait for no other axis to be homing
		currentHomingAxis = Z1
		CALL Index_Home_Z1
		MFLAGS(Z1).#HOME = 1
		WAIT 200
		isZ1Homed = 1
		DISP "Homing Z1 axis complete."
		DISABLE (Z1)
		WAIT 200
		ENABLE(Z1)
		WAIT 200
		homeAxis.1 = 0
		currentHomingAxis = -1
		ret	
	end	
ret

ON homeAxis.2 = 1 !Z2 Axis
	Disp "ATTEMPTING TO HOME Z2 AXIS"
	TILL currentHomingAxis = -1 ! Wait for no other axis to be homing
	currentHomingAxis = Z2
	CALL Index_Home_Z2
	MFLAGS(Z2).#HOME = 1
	WAIT 200
	isZ2Homed = 1
	DISP "Homing Z2 axis complete."
	DISABLE (Z2)
	WAIT 200
	ENABLE(Z2)
	WAIT 200
	homeAxis.2 = 0
	currentHomingAxis = -1
ret

ON homeAxis.3 = 1 !X Axis
	Disp "ATTEMPTING TO HOME X AXIS"
	TILL currentHomingAxis = -1 ! Wait for no other axis to be homing
	currentHomingAxis = X1
	CALL Index_Home_X
	MFLAGS(X1).#HOME = 1
	MFLAGS(X2).#HOME = 1
	WAIT 200
	isXHomed = 1
	DISP "Homing X axis complete."
	DISABLE (X1, X2)
	WAIT 200
	ENABLE(X1, X2)
	WAIT 200
	homeAxis.3 = 0
	currentHomingAxis = -1
RET

ON homeZ1 = 1
	homeAxis.1 = 1
	homeZ1 = 0
RET

ON homeZ2 = 1
	homeAxis.2 = 1
	homeZ2 = 0
RET

ON homeX = 1
	homeAxis.3 = 1
	homeX = 0
STOP

Index_Home_Z1:
!Begin homing to limit switch+index
	FMASK(Z1).#RL =0 !Disable lower left limit
	FDEF(Z1).#RL = 0 !Disable limit action
	FDEF(Z1).#LL = 0 !Disable limit action
	FMASK(Z1).#LL = 0 
	JERK(Z1) = 200
	KDEC(Z1) = 1000
	FMASK(Z1).#SRL = 0
	FMASK(Z1).#SLL = 0
	ENABLE Z1
	JOG/v (Z1), 10  !jog towards limit
	TILL ECAT_IN1.7=1
	KILL (Z1) 	!Stop moving immediately
	HALT (Z1)    !Stop moving immediately
	TILL MST(Z1).#MOVE = 0
	JOG/v (Z1),-1 !jog away from switch until unlatch
	TILL ECAT_IN1.7=0 !look for unlatch
	KILL (Z1) 	!Stop moving immediately
	HALT (Z1)    !Stop moving immediately
	TILL MST(Z1).#MOVE = 0
	Disable (Z1)
	TILL MST(Z1).#ENABLED = 0
	WAIT 1000
	set FPOS(Z1) = 700
	set APOS(Z1) = FPOS(Z1)
	Enable (Z1)
	WAIT 1000
	TILL MST(Z1).#ENABLED = 1
	!PTP/rev (Y), 50, 40 ! Move forward in case the axis is currently on the index
	TILL MST(Z1).#MOVE = 0
	DISP  "Homed Z1 successfully"
	FMASK(Z1).#RL = 1 !Enable lower left limit
	FDEF(Z1).#RL = 1 !Enable limit action
	FDEF(Z1).#LL = 1 !Enable limit action
	FMASK(Z1).#LL = 1 
	FMASK(Z1).#SRL = 1
	FMASK(Z1).#SLL = 1
	WAIT 500

RET
STOP

Index_Home_Z2:
!Begin homing to limit switch+index
	FMASK(Z2).#RL =0 !Disable lower left limit
	FDEF(Z2).#RL = 0 !Disable limit action
	FDEF(Z2).#LL = 0 !Disable limit action
	FMASK(Z2).#LL = 0 
	JERK(Z2) = 200
	KDEC(Z2) = 1000
	FMASK(Z2).#SRL = 0
	FMASK(Z2).#SLL = 0
	ENABLE Z2
	JOG/v (Z2), 10  !jog towards limit
	TILL ECAT_IN1.4=1
	KILL (Z2) 	!Stop moving immediately
	HALT (Z2)    !Stop moving immediately
	TILL MST(Z2).#MOVE = 0
	JOG/v (Z2),-1 !jog away from switch until unlatch
	TILL ECAT_IN1.4=0 !look for unlatch
	KILL (Z2) 	!Stop moving immediately
	HALT (Z2)    !Stop moving immediately
	TILL MST(Z2).#MOVE = 0
	Disable (Z2)
	TILL MST(Z2).#ENABLED = 0
	WAIT 1000
	set FPOS(Z2) = 700
	set APOS(Z2) = FPOS(Z1)
	Enable (Z2)
	WAIT 1000
	TILL MST(Z2).#ENABLED = 1
	!PTP/rev (Y), 50, 40 ! Move forward in case the axis is currently on the index
	TILL MST(Z2).#MOVE = 0
	DISP  "Homed Z2 successfully"
	FDEF(Z2).#LL = 1 !turn on limit switch
	FDEF(Z2).#RL = 1
	FMASK(Z2).#RL =1 !Enable lower left limit
	WAIT 500

RET
STOP

Index_Home_X:
	FMASK(X1).#RL =0 !Disable lower left limit
	FDEF(X1).#RL = 0 !Disable limit action
	FDEF(X1).#LL = 0 !Disable limit action
	FMASK(X1).#LL = 0 
	JERK(X1) = 200
	KDEC(X1) = 1000
	FMASK(X1).#SRL = 0
	FMASK(X1).#SLL = 0
	FMASK(X2).#RL =0 !Disable lower left limit
	FDEF(X2).#RL = 0 !Disable limit action
	FDEF(X2).#LL = 0 !Disable limit action
	FMASK(X2).#LL = 0 
	JERK(X2) = 200
	KDEC(X2) = 1000
	FMASK(X2).#SRL = 0
	FMASK(X2).#SLL = 0
	ENABLE (X1,X2)
	JOG/v (X1, X2), -10  !jog towards limit
	TILL ECAT_IN1.5=1 | ECAT_IN1.6=1
	KILL (X1,X2) 	!Stop moving immediately
	HALT (X1,X2)    !Stop moving immediately
	TILL MST(X1).#MOVE = 0 & MST(X2).#MOVE = 0
	IF ECAT_IN1.5=1
		JOG/v (X1),1 !jog away from switch until unlatch
		TILL ECAT_IN1.5=0 !look for unlatch
		KILL (X1) 	!Stop moving immediately
		HALT (X1)    !Stop moving immediately
		TILL MST(X1).#MOVE = 0
		Disable (X1)
		TILL MST(X1).#ENABLED = 0
		WAIT 1000
		set FPOS(X1) = 0
		set APOS(X1) = FPOS(X1)
		Enable (X1)
		WAIT 1000
		TILL MST(X1).#ENABLED = 1
		TILL MST(X1).#MOVE = 0
		FDEF(X1).#LL = 1 !turn on limit switch
		FDEF(X1).#RL = 1
		FMASK(X1).#RL =1 !Enable lower left limit
		WAIT 500
		
		ENABLE X2		! Finish homing X2
		JOG/v (X2), -10
		TILL ECAT_IN1.6=1
		KILL X2
		HALT X2
		TILL MST(X2).#MOVE = 0
		JOG/v (X2),1 !jog away from switch until unlatch
		TILL ECAT_IN1.6=0 !look for unlatch
		KILL (X2) 	!Stop moving immediately
		HALT (X2)    !Stop moving immediately
		TILL MST(X2).#MOVE = 0
		Disable (X2)
		TILL MST(X2).#ENABLED = 0
		WAIT 1000
		set FPOS(X2) = 0
		set APOS(X2) = FPOS(X2)
		Enable (X2)
		WAIT 1000
		TILL MST(X2).#ENABLED = 1
		TILL MST(X2).#MOVE = 0
		FDEF(X2).#LL = 1 !turn on limit switch
		FDEF(X2).#RL = 1
		FMASK(X2).#RL =1 !Enable lower left limit
		WAIT 500
	ELSEIF ECAT_IN1.6=1
		JOG/v (X2),1 !jog away from switch until unlatch
		TILL ECAT_IN1.6=0 !look for unlatch
		KILL (X2) 	!Stop moving immediately
		HALT (X2)    !Stop moving immediately
		TILL MST(X2).#MOVE = 0
		Disable (X2)
		TILL MST(X2).#ENABLED = 0
		WAIT 1000
		set FPOS(X2) = 0
		set APOS(X2) = FPOS(X2)
		Enable (X2)
		WAIT 1000
		TILL MST(X2).#ENABLED = 1
		TILL MST(X2).#MOVE = 0
		FDEF(X2).#LL = 1 !turn on limit switch
		FDEF(X2).#RL = 1
		FMASK(X2).#RL =1 !Enable lower left limit
		WAIT 500
		
		ENABLE X1		! Finish homing X1
		JOG/v (X1), -10
		TILL ECAT_IN1.5=1
		KILL X1
		HALT X1
		TILL MST(X1).#MOVE = 0
		JOG/v (X1),1 !jog away from switch until unlatch
		TILL ECAT_IN1.5=0 !look for unlatch
		KILL (X1) 	!Stop moving immediately
		HALT (X1)    !Stop moving immediately
		TILL MST(X1).#MOVE = 0
		Disable (X1)
		TILL MST(X1).#ENABLED = 0
		WAIT 1000
		set FPOS(X1) = 0
		set APOS(X1) = FPOS(X1)
		Enable (X1)
		WAIT 1000
		TILL MST(X1).#ENABLED = 1
		TILL MST(X1).#MOVE = 0
		FDEF(X1).#LL = 1 !turn on limit switch
		FDEF(X1).#RL = 1
		FMASK(X1).#RL =1 !Enable lower left limit
		WAIT 500
	END
RET
STOP

!Index_Home_X:
	!Begin homing to limit switch+index
!	FMASK(X).#RL =0 !Disable lower left limit
!	FMASK(XR).#RL =0 !Disable lower left limit
!	FDEF(X).#LL = 0 !Disable limit action
!	FDEF(XR).#LL = 0 !Disable limit action
!	FMASK(X).#LL = 0 
!	JERK(X) = 200
!	KDEC(X) = 300
!	FMASK(X).#SRL = 0
!	FMASK(X).#SLL = 0
!	JOG/v (X), -10  !jog towards limit
!	TILL ECAT_IN0.0=0
!	!KILL (X) 	!Stop moving immediately
!	HALT (X)    !Stop moving immediately
!	TILL MST(X).#MOVE = 0
!	JOG/v (X), 2 !jog away from switch until unlatch
!	TILL ECAT_IN0.0=1 !look for unlatch
!	set FPOS(X) = Work_Offset_X
!	WAIT 100
!	HALT X
!	HALT XR  !Stop moving immediately
!	WAIT 7000
!	TILL MST(X).#MOVE = 0
!	TILL MST(XR).#MOVE = 0
!	set APOS(X) = FPOS(X)
!	
!	Enable (X,XR)
!	WAIT 2000
!	TILL MST(X).#ENABLED = 1
!	TILL MST(XR).#ENABLED = 1
!	TILL MST(X).#ENABLED = 1
!	TILL MST(XR).#ENABLED = 1
!	PTP/rev (X), 10, 40 ! Move forward in case the axis is currently on the index
!	TILL MST(X).#MOVE = 0
!	DISP  "Homed X successfully"
!	FDEF(X).#LL = 1 !turn on limit switch
!	FDEF(X).#LL = 1
!	FMASK(X).#RL =1 !Enable lower left limit
!	FMASK(XR).#RL =1 !Enable lower left limit
!	WAIT 500
!	RET
!STOP

#10
ON MST(X1).#MOVE
		
	IF RPOS(Z1) < 600
		HALT (X1, X2)
		KILL (X1, X2)
		DISP "Z1 Axis too low to move gantry!"
	END


	IF RPOS(Z2) < 600
		HALT (X1, X2)
		KILL (X1, X2)
		DISP "Z2 Axis too low to move gantry!"
	END

RET


! Clamp the output voltage of the beckhoff module between 0-5V so the SyRen controller doesn't fry
ON ECAT_OUT2 > 16383
	ECAT_OUT2 = 16383
RET

ON ECAT_OUT2 < 0
	ECAT_OUT2 = 0
RET

ON ECAT_OUT3 > 16383
	ECAT_OUT3 = 16383
RET

ON ECAT_OUT3 < 0
	ECAT_OUT3 = 0
RET

ON ECAT_IN1.1 = 1
	
	IF currentHomingAxis <> Z1
		HALT Z1
		KILL Z1
	END
	
RET

#11
global real Z1_RMS_limit
global real Z1_Jog_state 
global real Z1_fmax
global real Z1_fmin 

!Configure user values
Z1_RMS_limit = 6
Z1_Jog_state =1
Z1_fmax = -10
Z1_fmin = 0

VEL(Z1) = -1

ENABLE Z1
ENABLE Drill

JOG Z1
DISP  "Drilling Started"

WHILE Z1_Jog_state = 1

	IF machineState <> coreState
		IF RMS(Z1) > Z1_RMS_limit & VEL(Z1) < Z1_fmin
			IMM VEL(Z1) = VEL(Z1) + 0.1
			DISP "Exceeded RMS, slowing feed"
		END
		
		IF RMS(Z1) < Z1_RMS_limit & VEL(Z1) > Z1_fmax
			IMM VEL(Z1) = VEL(Z1) - 0.1
			DISP "Feed increased"
		END
		
		WAIT 50
	END
END



STOP ! Ends program 


!Begin autoroutines
ON Z1_Jog_state =1 
	!JOG Z1, -
RET
ON Z1_Jog_state =0
	HALT ALL
	STOP 11
	WAIT 1000
	PTP/rev (Z1), 50, 35
RET

#13
!This buffer is ONLY for autoroutines that are run in background 
!Below are subroutines that are meant to be triggered by the dashboard. 

Enable_Drill:

	ENABLE Drill; 	WAIT 500
	COMMUT Drill,40, 100,500 !Axis, power, settling time

STOP


STOP



!! Below are all the automatically triggered routines. ON commands. These are eqvuiilant to interupts

ON MST(X1).#ENABLED 
	ENABLE X2
RET




#19
! THIS PROGRAM IS USED TO ADJUST A GANTRY SYSTEM
INT GLOBAL M_AXIS, S_AXIS ,BUFFER !"M" FOR MASTER AXIS, "S" FOR SLAVED
M_AXIS=4 ; S_AXIS=5 ;BUFFER=19! SET HERE THE INVOLVED AXIS NUMBERS A=4,
DISABLE(M_AXIS);DISABLE(S_AXIS)
!------ Configuration variables ---------------------
ERRI(S_AXIS)=ERRI(M_AXIS)
ERRV(S_AXIS)=ERRV(M_AXIS)
ERRA(S_AXIS)=ERRA(M_AXIS)
CERRI(S_AXIS)=CERRI(M_AXIS)
CERRV(S_AXIS)=CERRV(M_AXIS)
CERRA(S_AXIS)=CERRA(M_AXIS)
DELI(S_AXIS)=DELI(M_AXIS)
DELV(S_AXIS)=DELV(M_AXIS)
SLLIMIT(S_AXIS)=SLLIMIT(M_AXIS)
SRLIMIT(S_AXIS)=SRLIMIT(M_AXIS)
XVEL(S_AXIS)=XVEL(M_AXIS)
XACC(S_AXIS)=XACC(M_AXIS)
VELBRK(S_AXIS)=VELBRK(M_AXIS)
XRMS(S_AXIS)=XRMS(M_AXIS)
XRMST(S_AXIS)=XRMST(M_AXIS)
XCURI(S_AXIS)=XCURI(M_AXIS)
XCURV(S_AXIS)=XCURV(M_AXIS)
!------ Adjustment variables ---------------------
SLPKP(S_AXIS)=SLPKP(M_AXIS)
SLVKP(S_AXIS)=SLVKP(M_AXIS)
SLVKI(S_AXIS)=SLVKI(M_AXIS)
SLVLI(S_AXIS)=SLVLI(M_AXIS)
SLVSOF(S_AXIS)=SLVSOF(M_AXIS)
SLIOFFS(S_AXIS)=SLIOFFS(M_AXIS)
SLFRC(S_AXIS)=SLFRC(M_AXIS)
SLAFF(S_AXIS)=SLAFF(M_AXIS)
SLCPA(S_AXIS)=SLCPA(M_AXIS)
EFAC(S_AXIS)=EFAC(M_AXIS)
!------ Motion variables ---------------------
VEL(S_AXIS)=VEL(M_AXIS)
ACC(S_AXIS)=ACC(M_AXIS)
DEC(S_AXIS)=DEC(M_AXIS)
JERK(S_AXIS)=JERK(M_AXIS)
STOP
! These auto routines update the adjustment variables
! of the S_AXIS to match the M_AXIS
ON SLPKP(S_AXIS)<>SLPKP(M_AXIS); SLPKP(S_AXIS)=SLPKP(M_AXIS); RET
ON SLPKP(S_AXIS)<>SLPKP(M_AXIS); SLPKP(S_AXIS)=SLPKP(M_AXIS); RET
ON SLVKP(S_AXIS)<>SLVKP(M_AXIS); SLVKP(S_AXIS)=SLVKP(M_AXIS); RET
ON SLVKI(S_AXIS)<>SLVKI(M_AXIS); SLVKI(S_AXIS)=SLVKI(M_AXIS); RET
ON SLVLI(S_AXIS)<>SLVLI(M_AXIS); SLVLI(S_AXIS)=SLVLI(M_AXIS); RET
ON SLVSOF(S_AXIS)<>SLVSOF(M_AXIS); SLVSOF(S_AXIS)=SLVSOF(M_AXIS); RET
ON SLIOFFS(S_AXIS)<>SLIOFFS(M_AXIS); SLIOFFS(S_AXIS)=SLIOFFS(M_AXIS); RET
ON SLFRC(S_AXIS)<>SLFRC(M_AXIS); SLFRC(S_AXIS)=SLFRC(M_AXIS); RET
ON SLAFF(S_AXIS)<>SLAFF(M_AXIS); SLAFF(S_AXIS)=SLAFF(M_AXIS); RET
ON SLCPA(S_AXIS)<>SLCPA(M_AXIS); SLCPA(S_AXIS)=SLCPA(M_AXIS); RET
ON EFAC(S_AXIS)<>EFAC(M_AXIS); EFAC(S_AXIS)=EFAC(M_AXIS); RET
ON XVEL(S_AXIS)<>XVEL(M_AXIS); XVEL(S_AXIS)=XVEL(M_AXIS); RET

#60
! SpiiPlus Commutation Startup Program
! Date: 15/1/09 
! Program version: 6.5
! Author: Boaz Kramer   
! Retrieve Commutation Phase at a Detent Point.
!
! The following text provides a suggested commutation startup program. 
! It is recommended to use it as a part of the homing process.
! The user needs to customize the program, add safety checks and  test
!   before the program can be included in the application.
!
! Program clears bits 1,4,5,6 of MFLAGS and sets bit 8 of MFLAGS.
!
! Program moves motor away from limit switch, if activated during the process.
!
! If the program finishes :
!   1. Bit 9 of the variable MFLAGS is set (commutation OK).
!   2. Motor is left disabled (change if necessary)
! 
! If the program fails:
!   1. Bit 9 of the variable MFLAGS is cleared (commutation not OK).
!   2. Motor is left disabled.
!
! Program variable SP_Fail_Code is set according to the following:
! SP_Fail_Code=0 Program finished.
! SP_Fail_Code=1 Motor error (If occurs, check MERR variable)
! SP_Fail_Code=4 Unable to get out of the limit switch.
!
! The program changes the following motion variables:
!   VEL, ACC, DEC, KDEC, JERK.
! Make sure to change them back to your default motion parameters 
!   at the end of the program.
!
!*******************************************************************************
! PROGRAM VARIABLES
!
! All the variables used by the program start with "SP_".
!
! Input variables:

int  SP_Axis                      ! Axis to be commutated
real SP_Direction                 ! Search direction 
real SP_Settle_Time               ! Settling time at Detent Points [msec]
real SP_Search_Vel                ! Search velocity [user-units/sec]
real SP_Drive                     ! Actuating drive command [% of maximum]
real SP_Max_Search                ! Maximum search distance [user-units]
real SP_Init_Offset               ! Initial commutation offset [elec. degrees]

SP_Axis = 0          
SP_Drive = 10          
SP_Settle_Time = 100    
SP_Search_Vel  = 0.059999999928         
SP_Init_Offset = 0
SP_Direction =  -1
SP_Max_Search = 1.666666666

!   Output varibale:
int  SP_Fail_Code                 ! Faiure Code of the startup program

! Auxiliary variable:
real SP_Pitch                     ! Magnetic pitch [180 elec.deg. in user units]

! State Flag
int  SP_InCommutationStartup      ! Flag indicating commutation startup is in progress


SP_Pitch=SLCPRD(SP_Axis)/SLCNP(SP_Axis)*EFAC(SP_Axis)


!******************************************************************************* 
! INITIALIZE
FCLEAR(SP_Axis)   
disable(SP_Axis)                     
SETCONF(216,SP_Axis,0)                ! Reset commutation state
setconf(214,SP_Axis,SP_Init_Offset)   ! Set initial commutation phase
SP_Fail_Code=0                        ! Reset failure
SP_Direction=1                        ! Move in positive direction
SP_InCommutationStartup=1             ! commutation startup in progress
!******************************************************************************* 
! SET MOTION PROFILE FOR COMMUTATION STARTUP PROCESS 
!
!   WARNING: The following are the suggested motion parameters for the startup
!     process. Check that the values are suitable for your application.

ACC(SP_Axis)=SP_Search_Vel*10.; DEC(SP_Axis)=SP_Search_Vel*10. 
KDEC(SP_Axis)=SP_Search_Vel*50.; JERK(SP_Axis)=SP_Search_Vel*100.

!******************************************************************************* 
! STEP 1 - MOVE TO FIRST DETENT POINT 
!
! WARNING: The motor moves to a detent point by jump. 
!   The jump distance is up to one magnetic pitch in any direction.
!   The motor jumps to the closest detent point within its motion range.
!   If necessary modify initial detent point by changing the variable 
!     SP_Init_Offset between 0-360 electrical degrees.  
disp ""
disp "...Commutation Startup Program Running..."
enable(SP_Axis)
while (DCOM(SP_Axis)+0.05 < SP_Drive); DCOM(SP_Axis) = DCOM(SP_Axis) + 0.05; end
DCOM(SP_Axis) = SP_Drive
wait SP_Settle_Time
call Limit_Check
!******************************************************************************* 
! STEP 2 - MOVE TO SECOND DETENT POINT
!
!   The program moves the motor 90 electrical degrees in order to eliminate
!      a state of unstable equilibrium.

Move_Detent:
ptp/rv (SP_Axis), SP_Direction*SP_Pitch/2.,SP_Search_Vel
till ^AST(SP_Axis).#MOVE; wait SP_Settle_Time
call Limit_Check
disable(SP_Axis)

MFLAGS(SP_Axis).9=1               ! Set commutation state 
DCOM(SP_Axis) = 0

! If motor is to be left enabled after startup process delete the following line:

Finish:
SP_InCommutationStartup=0              ! commutation startup is finished
If SP_Fail_Code=0; disp "   Commutation Startup Finished."
else disp "   Commutation Startup Failed."; disp "   Failure Code = %i",SP_Fail_Code; end
STOP

!******************************************************************************* 
!   The following routine move the motor away from limit switches 

Limit_Check:
if MERR(SP_Axis) & MERR(SP_Axis)<>5010 & MERR(SP_Axis)<>5011; SP_Fail_Code=1; DISABLE(SP_Axis); DCOM(SP_Axis)=0; goto Finish; end
!   if MERR(SP_Axis); SP_Fail_Code=1; DISABLE(SP_Axis); DCOM(SP_Axis)=0; goto Finish; end 
if (FAULT(SP_Axis).#LL)|(FAULT(SP_Axis).#RL)
   if FAULT(SP_Axis).#LL; SP_Direction=1; else SP_Direction=-1; end
   ptp/rv (SP_Axis), SP_Direction*SP_Max_Search, SP_Search_Vel 
   till ((^FAULT(SP_Axis).#LL)&(^FAULT(SP_Axis).#RL))|(^AST(SP_Axis).#MOVE)
   if (FAULT(SP_Axis).#LL)|(FAULT(SP_Axis).#RL); SP_Fail_Code=4; DISABLE(SP_Axis); DCOM(SP_Axis)=0; goto Finish; end 
   kill(SP_Axis);  wait SP_Settle_Time; goto Move_Detent
end
ret

ON ((SP_InCommutationStartup = 1) & ((FAULT(SP_Axis)&0x30f80)>0 | (S_FAULT&0x30000000)>0))
SP_Fail_Code=1; DISABLE(SP_Axis); DCOM(SP_Axis)=0
CALL Finish 
RET

#62
AUTOEXEC:
WAIT 20000
!FOR CUMMUTATING Z1 AXIS 
!
!*******************************************************************************
! PROGRAM VARIABLES
int  SP_Axis                      ! Axis to be commutated
real SP_Direction                 ! Search direction 
real SP_Settle_Time               ! Settling time at Detent Points [msec]
real SP_Search_Vel                ! Search velocity [user-units/sec]
real SP_Drive                     ! Actuating drive command [% of maximum]
real SP_Max_Search                ! Maximum search distance [user-units]
real SP_Init_Offset               ! Initial commutation offset [elec. degrees]

SP_Axis = 0     
SP_Drive = 30       
SP_Settle_Time = 100    
SP_Search_Vel  = 0.017280000006912         
SP_Init_Offset = 0
SP_Direction =  -1
SP_Max_Search = 0.333333334

!   Output varibale:
int  SP_Fail_Code                 ! Faiure Code of the startup program

! Auxiliary variable:
real SP_Pitch                     ! Magnetic pitch [180 elec.deg. in user units]

! State Flag
int  SP_InCommutationStartup      ! Flag indicating commutation startup is in progress


SP_Pitch=SLCPRD(SP_Axis)/SLCNP(SP_Axis)*EFAC(SP_Axis)


!******************************************************************************* 
! INITIALIZE
FCLEAR(SP_Axis)   
disable(SP_Axis)                     
SETCONF(216,SP_Axis,0)                ! Reset commutation state
setconf(214,SP_Axis,SP_Init_Offset)   ! Set initial commutation phase
SP_Fail_Code=0                        ! Reset failure
SP_Direction=1                        ! Move in positive direction
SP_InCommutationStartup=1             ! commutation startup in progress
!******************************************************************************* 
! SET MOTION PROFILE FOR COMMUTATION STARTUP PROCESS 
!
!   WARNING: The following are the suggested motion parameters for the startup
!     process. Check that the values are suitable for your application.

ACC(SP_Axis)=SP_Search_Vel*10.; DEC(SP_Axis)=SP_Search_Vel*10. 
KDEC(SP_Axis)=SP_Search_Vel*50.; JERK(SP_Axis)=SP_Search_Vel*100.

!******************************************************************************* 
! STEP 1 - MOVE TO FIRST DETENT POINT 
!
! WARNING: The motor moves to a detent point by jump. 
!   The jump distance is up to one magnetic pitch in any direction.
!   The motor jumps to the closest detent point within its motion range.
!   If necessary modify initial detent point by changing the variable 
!     SP_Init_Offset between 0-360 electrical degrees.  
disp ""
disp "...Commutation Startup Program Running..."
enable(SP_Axis)
while (DCOM(SP_Axis)+0.05 < SP_Drive); DCOM(SP_Axis) = DCOM(SP_Axis) + 0.05; end
DCOM(SP_Axis) = SP_Drive
wait SP_Settle_Time
call Limit_Check
!******************************************************************************* 
! STEP 2 - MOVE TO SECOND DETENT POINT
!
!   The program moves the motor 90 electrical degrees in order to eliminate
!      a state of unstable equilibrium.

Move_Detent:
ptp/rv (SP_Axis), SP_Direction*SP_Pitch/2.,SP_Search_Vel
till ^AST(SP_Axis).#MOVE; wait SP_Settle_Time
call Limit_Check
disable(SP_Axis)

MFLAGS(SP_Axis).9=1               ! Set commutation state 
DCOM(SP_Axis) = 0

! If motor is to be left enabled after startup process delete the following line:

Finish:
SP_InCommutationStartup=0              ! commutation startup is finished
If SP_Fail_Code=0; disp "   Commutation Startup Finished."
else disp "   Commutation Startup Failed."; disp "   Failure Code = %i",SP_Fail_Code; end
STOP

!******************************************************************************* 
!   The following routine move the motor away from limit switches 

Limit_Check:
if MERR(SP_Axis) & MERR(SP_Axis)<>5010 & MERR(SP_Axis)<>5011; SP_Fail_Code=1; DISABLE(SP_Axis); DCOM(SP_Axis)=0; goto Finish; end
!   if MERR(SP_Axis); SP_Fail_Code=1; DISABLE(SP_Axis); DCOM(SP_Axis)=0; goto Finish; end 
if (FAULT(SP_Axis).#LL)|(FAULT(SP_Axis).#RL)
   if FAULT(SP_Axis).#LL; SP_Direction=1; else SP_Direction=-1; end
   ptp/rv (SP_Axis), SP_Direction*SP_Max_Search, SP_Search_Vel 
   till ((^FAULT(SP_Axis).#LL)&(^FAULT(SP_Axis).#RL))|(^AST(SP_Axis).#MOVE)
   if (FAULT(SP_Axis).#LL)|(FAULT(SP_Axis).#RL); SP_Fail_Code=4; DISABLE(SP_Axis); DCOM(SP_Axis)=0; goto Finish; end 
   kill(SP_Axis);  wait SP_Settle_Time; goto Move_Detent
end
ret

ON ((SP_InCommutationStartup = 1) & ((FAULT(SP_Axis)&0x30f80)>0 | (S_FAULT&0x30000000)>0))
SP_Fail_Code=1; DISABLE(SP_Axis); DCOM(SP_Axis)=0
CALL Finish 
RET

#63
AUTOEXEC:
WAIT 10000
!FOR CUMMUTATING DRILL AXIS 
!
!*******************************************************************************
! PROGRAM VARIABLES
int  SP_Axis                      ! Axis to be commutated
real SP_Direction                 ! Search direction 
real SP_Settle_Time               ! Settling time at Detent Points [msec]
real SP_Search_Vel                ! Search velocity [user-units/sec]
real SP_Drive                     ! Actuating drive command [% of maximum]
real SP_Max_Search                ! Maximum search distance [user-units]
real SP_Init_Offset               ! Initial commutation offset [elec. degrees]

SP_Axis = 1          
SP_Drive = 22          
SP_Settle_Time = 100    
SP_Search_Vel  = 0.017280000006912         
SP_Init_Offset = 0
SP_Direction =  -1
SP_Max_Search = 0.333333334

!   Output varibale:
int  SP_Fail_Code                 ! Faiure Code of the startup program

! Auxiliary variable:
real SP_Pitch                     ! Magnetic pitch [180 elec.deg. in user units]

! State Flag
int  SP_InCommutationStartup      ! Flag indicating commutation startup is in progress


SP_Pitch=SLCPRD(SP_Axis)/SLCNP(SP_Axis)*EFAC(SP_Axis)


!******************************************************************************* 
! INITIALIZE
FCLEAR(SP_Axis)   
disable(SP_Axis)                     
SETCONF(216,SP_Axis,0)                ! Reset commutation state
setconf(214,SP_Axis,SP_Init_Offset)   ! Set initial commutation phase
SP_Fail_Code=0                        ! Reset failure
SP_Direction=1                        ! Move in positive direction
SP_InCommutationStartup=1             ! commutation startup in progress
!******************************************************************************* 
! SET MOTION PROFILE FOR COMMUTATION STARTUP PROCESS 
!
!   WARNING: The following are the suggested motion parameters for the startup
!     process. Check that the values are suitable for your application.

ACC(SP_Axis)=SP_Search_Vel*10.; DEC(SP_Axis)=SP_Search_Vel*10. 
KDEC(SP_Axis)=SP_Search_Vel*50.; JERK(SP_Axis)=SP_Search_Vel*100.

!******************************************************************************* 
! STEP 1 - MOVE TO FIRST DETENT POINT 
!
! WARNING: The motor moves to a detent point by jump. 
!   The jump distance is up to one magnetic pitch in any direction.
!   The motor jumps to the closest detent point within its motion range.
!   If necessary modify initial detent point by changing the variable 
!     SP_Init_Offset between 0-360 electrical degrees.  
disp ""
disp "...Commutation Startup Program Running..."
enable(SP_Axis)
while (DCOM(SP_Axis)+0.05 < SP_Drive); DCOM(SP_Axis) = DCOM(SP_Axis) + 0.05; end
DCOM(SP_Axis) = SP_Drive
wait SP_Settle_Time
call Limit_Check
!******************************************************************************* 
! STEP 2 - MOVE TO SECOND DETENT POINT
!
!   The program moves the motor 90 electrical degrees in order to eliminate
!      a state of unstable equilibrium.

Move_Detent:
ptp/rv (SP_Axis), SP_Direction*SP_Pitch/2.,SP_Search_Vel
till ^AST(SP_Axis).#MOVE; wait SP_Settle_Time
call Limit_Check
disable(SP_Axis)

MFLAGS(SP_Axis).9=1               ! Set commutation state 
DCOM(SP_Axis) = 0

! If motor is to be left enabled after startup process delete the following line:

Finish:
SP_InCommutationStartup=0              ! commutation startup is finished
If SP_Fail_Code=0; disp "   Commutation Startup Finished."
else disp "   Commutation Startup Failed."; disp "   Failure Code = %i",SP_Fail_Code; end
STOP

!******************************************************************************* 
!   The following routine move the motor away from limit switches 

Limit_Check:
if MERR(SP_Axis) & MERR(SP_Axis)<>5010 & MERR(SP_Axis)<>5011; SP_Fail_Code=1; DISABLE(SP_Axis); DCOM(SP_Axis)=0; goto Finish; end
!   if MERR(SP_Axis); SP_Fail_Code=1; DISABLE(SP_Axis); DCOM(SP_Axis)=0; goto Finish; end 
if (FAULT(SP_Axis).#LL)|(FAULT(SP_Axis).#RL)
   if FAULT(SP_Axis).#LL; SP_Direction=1; else SP_Direction=-1; end
   ptp/rv (SP_Axis), SP_Direction*SP_Max_Search, SP_Search_Vel 
   till ((^FAULT(SP_Axis).#LL)&(^FAULT(SP_Axis).#RL))|(^AST(SP_Axis).#MOVE)
   if (FAULT(SP_Axis).#LL)|(FAULT(SP_Axis).#RL); SP_Fail_Code=4; DISABLE(SP_Axis); DCOM(SP_Axis)=0; goto Finish; end 
   kill(SP_Axis);  wait SP_Settle_Time; goto Move_Detent
end
ret

ON ((SP_InCommutationStartup = 1) & ((FAULT(SP_Axis)&0x30f80)>0 | (S_FAULT&0x30000000)>0))
SP_Fail_Code=1; DISABLE(SP_Axis); DCOM(SP_Axis)=0
CALL Finish 
RET

#A
!axisdef X=0,Y=1,Z=2,T=3,A=4,B=5,C=6,D=7
!axisdef x=0,y=1,z=2,t=3,a=4,b=5,c=6,d=7
global int I(100),I0,I1,I2,I3,I4,I5,I6,I7,I8,I9,I90,I91,I92,I93,I94,I95,I96,I97,I98,I99
global real V(100),V0,V1,V2,V3,V4,V5,V6,V7,V8,V9,V90,V91,V92,V93,V94,V95,V96,V97,V98,V99

global int xAxisGroup(2)

AXISDEF X1 = 2
AXISDEF X2 = 3
AXISDEF Z1 = 0
AXISDEF Z2 = 4
AXISDEF Drill = 1

!Global

! System globals
global int running
global int advanceState
global int machineState

! Global State Machine vars
global int opControlState
global int drillState
global int coreState
global int extractState
global int filterState

! Telemetry
global real drillWOB
global real drillRPM
global int heatTemp

global real framePwr
global real drillPwr
global real waterExPwr
global real filterPwr
global real totalPwr

global int holeCount

! Coring vars
global int coreSamples
global real coreRPM

! Drilling Vars
global real touchRMS
global real peckDistance
global real retractDistance
global real holeSpacing
global int numHoles

! Water Extract Vars
global real xOffset
global real heaterSetpoint

! Filtration vars
global int backFlush
global real ecAmpTarget
global real ecTime
global int enableSloshing
global real sloshAmplitude
global real sloshFrequency
global real sloshTime

! Heater control vars
global int pellet1_h1_SetTemp
global real PELLET1_TC1_OFFSET,PELLET1_TC1_STATUS_OFFSET
global real PELLET1_TC1, PELLET1_TC1_STATUS   !This is the temperature variable in c

! Digital and Analog IO
global int ECAT_OFFSET
global int ECAT_OUT1 ! Digital Out
global int ECAT_OUT2 ! Analog Out 1
global int ECAT_OUT3 ! Analog Out 2
global int ECAT_IN1  ! Digital In 1

! Jog Control vars
global real jogX
global real jogZ1
global real jogZ2
global real jogDrill
global int jogBoreholePump
global int homeX
global int homeZ1
global int homeZ2
global int isXHomed
global int isZ1Homed
global int isZ2Homed

! Homing vars
global int currentHomingAxis
global int homeAxis

!Persistent

global real drillVel
global real drillAccel
global real drillJerk

global real Z1TravelVel
global real Z1DrillVel
global real Z1Accel
global real Z1Jerk

global real Z2TravelVel
global real Z2ProbeVel
global real Z2Accel
global real Z2Jerk

global real XVel;
global real XAccel;
global real XJerk;

global int persistentChanged;


