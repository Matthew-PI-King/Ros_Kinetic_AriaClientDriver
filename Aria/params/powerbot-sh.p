;SectionFlags for : 
;  Robot parameter file

Section General settings
;SectionFlags for General settings: 
Class Pioneer            ; general type of robot
Subclass powerbot-sh     ; specific type of robot
RobotRadius 550          ; radius in mm
RobotDiagonal 240        ; half-height to diagonal of octagon
RobotWidth 680           ; width in mm
RobotLength 911          ; length in mm of the whole robot
RobotLengthFront 369     ; length in mm to the front of the robot (if this is 0
                         ; (or non existant) this value will be set to half of
                         ; RobotLength)
RobotLengthRear 542      ; length in mm to the rear of the robot (if this is 0
                         ; (or non existant) this value will be set to half of
                         ; RobotLength)
Holonomic true           ; turns in own radius
MaxRVelocity 360         ; absolute maximum degrees / sec
MaxVelocity 2000         ; absolute maximum mm / sec
MaxLatVelocity 0         ; absolute lateral maximum mm / sec
HasMoveCommand true      ; has built in move command
RequestIOPackets false   ; automatically request IO packets
RequestEncoderPackets false ; automatically request encoder packets
SwitchToBaudRate 38400   ; switch to this baud if non-0 and supported on robot

Section Conversion factors
;SectionFlags for Conversion factors: 
AngleConvFactor 0.001534 ; radians per angular unit (2PI/4096)
DistConvFactor 1         ; multiplier to mm from robot units
VelConvFactor 1          ; multiplier to mm/sec from robot units
RangeConvFactor 1        ; multiplier to mm from sonar units
DiffConvFactor 0.00373   ; ratio of angular velocity to wheel velocity (unused
                         ; in newer firmware that calculates and returns this)
Vel2Divisor 20           ; divisor for VEL2 commands
GyroScaler 1.626         ; Scaling factor for gyro readings

Section Accessories the robot has
;SectionFlags for Accessories the robot has: 
TableSensingIR false     ; if robot has upwards facing table sensing IR
NewTableSensingIR false  ; if table sensing IR are sent in IO packet
FrontBumpers true        ; if robot has a front bump ring
NumFrontBumpers 7        ; number of front bumpers on the robot
RearBumpers true         ; if the robot has a rear bump ring
NumRearBumpers 5         ; number of rear bumpers on the robot

Section Sonar parameters
;SectionFlags for Sonar parameters: 
SonarNum 32              ; number of sonar on the robot
;  SonarUnit <sonarNumber> <x position, mm> <y position, mm> <heading of disc,
;  degrees>
SonarUnit 0 152 278 90
SonarUnit 1 200 267 65
SonarUnit 2 241 238 45
SonarUnit 3 274 200 35
SonarUnit 4 300 153 25
SonarUnit 5 320 96 15
SonarUnit 6 332 33 5
SonarUnit 7 0 0 -180
SonarUnit 8 332 -33 -5
SonarUnit 9 320 -96 -15
SonarUnit 10 300 -153 -25
SonarUnit 11 274 -200 -35
SonarUnit 12 241 -238 -45
SonarUnit 13 200 -267 -65
SonarUnit 14 152 -278 -90
SonarUnit 15 0 0 -180
SonarUnit 16 -298 -278 -90
SonarUnit 17 -347 -267 -115
SonarUnit 18 -388 -238 -135
SonarUnit 19 -420 -200 -145
SonarUnit 20 -447 -153 -155
SonarUnit 21 -467 -96 -165
SonarUnit 22 -478 -33 -175
SonarUnit 23 0 0 -180
SonarUnit 24 -478 33 175
SonarUnit 25 -467 96 165
SonarUnit 26 -447 153 155
SonarUnit 27 -420 200 145
SonarUnit 28 -388 238 135
SonarUnit 29 -347 267 115
SonarUnit 30 -298 278 90
SonarUnit 31 0 0 -180

Section IR parameters
;SectionFlags for IR parameters: 
IRNum 0                  ; number of IRs on the robot
;  IRUnit <IR Number> <IR Type> <Persistance, cycles> <x position, mm> <y
;  position, mm>

Section Movement control parameters
;  if these are 0 the parameters from robot flash will be used, otherwise these
;  values will be used
;SectionFlags for Movement control parameters: 
SettableVelMaxes true    ; if TransVelMax and RotVelMax can be set
TransVelMax 0            ; maximum desired translational velocity for the robot
RotVelMax 0              ; maximum desired rotational velocity for the robot
SettableAccsDecs true    ; if the accel and decel parameters can be set
TransAccel 0             ; translational acceleration
TransDecel 0             ; translational deceleration
RotAccel 0               ; rotational acceleration
RotDecel 0               ; rotational deceleration
HasLatVel false          ; if the robot has lateral velocity
LatVelMax 0              ; maximum desired lateral velocity for the robot
LatAccel 0               ; lateral acceleration
LatDecel 0               ; lateral deceleration

Section GPS parameters
;SectionFlags for GPS parameters: 
GPSPX 0                  ; x location of gps receiver antenna on robot, mm
GPSPY 0                  ; y location of gps receiver antenna on robot, mm
GPSType standard         ; type of gps receiver (trimble, novatel, standard)
GPSPort COM3             ; port the gps is on
GPSBaud 9600             ; gps baud rate (9600, 19200, 38400, etc.)

Section Compass parameters
;SectionFlags for Compass parameters: 
CompassType robot        ; type of compass: robot (typical configuration), or
                         ; serialTCM (computer serial port)
CompassPort              ; serial port name, if CompassType is serialTCM

Section Laser parameters
;SectionFlags for Laser parameters: 
LaserType lms2xx         ; type of laser
LaserPortType serial     ; type of port the laser is on
LaserPort COM2           ; port the laser is on
LaserAutoConnect false   ; if the laser connector should autoconnect this laser
                         ; or not
LaserFlipped true        ; if the laser is upside-down or not
LaserPowerControlled true ; if the power to the laser is controlled by serial
LaserMaxRange 0          ; Max range to use for the laser, 0 to use default
                         ; (only use if you want to shorten it from the
                         ; default), mm
LaserCumulativeBufferSize 0 ; Cumulative buffer size to use for the laser, 0 to
                         ; use default
LaserX 251               ; x location of laser, mm
LaserY 0                 ; y location of laser, mm
LaserTh 0                ; rotation of laser, deg
LaserZ 0                 ; height of the laser off the ground, mm (0 means
                         ; unknown)
LaserIgnore              ; Readings within a degree of the listed degrees
                         ; (separated by a space) will be ignored
LaserStartDegrees        ; start degrees for the sensor (leave blank for
                         ; default, use this to constrain the range) (double)
LaserEndDegrees          ; start degrees for the sensor (leave blank for
                         ; default, use this to constrain the range) (double)
LaserDegreesChoice       ; degrees choice for the sensor (leave blank for
                         ; default, use this to constrain the range)
LaserIncrement           ; Increment for the sensor (leave blank for default,
                         ; use this to have a custom increment) (double)
LaserIncrementChoice     ; Increment for the sensor (leave blank for default,
                         ; use this to have a larger increment)
LaserUnitsChoice         ; Units for the sensor (leave blank for default, use
                         ; this to have a larger units)
LaserReflectorBitsChoice  ; ReflectorBits for the sensor (leave blank for
                         ; default, use this to have a larger units)
LaserStartingBaudChoice  ; StartingBaud for the sensor (leave blank for
                         ; default, use this to have a larger StartingBaud)
LaserAutoBaudChoice      ; AutoBaud for the sensor (leave blank for default,
                         ; use this to have a larger units)

Section Laser 2 parameters
;SectionFlags for Laser 2 parameters: 
LaserType                ; type of laser
LaserPortType            ; type of port the laser is on
LaserPort                ; port the laser is on
LaserAutoConnect false   ; if the laser connector should autoconnect this laser
                         ; or not
LaserFlipped false       ; if the laser is upside-down or not
LaserPowerControlled true ; if the power to the laser is controlled by serial
LaserMaxRange 0          ; Max range to use for the laser, 0 to use default
                         ; (only use if you want to shorten it from the
                         ; default), mm
LaserCumulativeBufferSize 0 ; Cumulative buffer size to use for the laser, 0 to
                         ; use default
LaserX 0                 ; x location of laser, mm
LaserY 0                 ; y location of laser, mm
LaserTh 0                ; rotation of laser, deg
LaserZ 0                 ; height of the laser off the ground, mm (0 means
                         ; unknown)
LaserIgnore              ; Readings within a degree of the listed degrees
                         ; (separated by a space) will be ignored
LaserStartDegrees        ; start degrees for the sensor (leave blank for
                         ; default, use this to constrain the range) (double)
LaserEndDegrees          ; start degrees for the sensor (leave blank for
                         ; default, use this to constrain the range) (double)
LaserDegreesChoice       ; degrees choice for the sensor (leave blank for
                         ; default, use this to constrain the range)
LaserIncrement           ; Increment for the sensor (leave blank for default,
                         ; use this to have a custom increment) (double)
LaserIncrementChoice     ; Increment for the sensor (leave blank for default,
                         ; use this to have a larger increment)
LaserUnitsChoice         ; Units for the sensor (leave blank for default, use
                         ; this to have a larger units)
LaserReflectorBitsChoice  ; ReflectorBits for the sensor (leave blank for
                         ; default, use this to have a larger units)
LaserStartingBaudChoice  ; StartingBaud for the sensor (leave blank for
                         ; default, use this to have a larger StartingBaud)
LaserAutoBaudChoice      ; AutoBaud for the sensor (leave blank for default,
                         ; use this to have a larger units)

Section Laser 3 parameters
;SectionFlags for Laser 3 parameters: 
LaserType                ; type of laser
LaserPortType            ; type of port the laser is on
LaserPort                ; port the laser is on
LaserAutoConnect false   ; if the laser connector should autoconnect this laser
                         ; or not
LaserFlipped false       ; if the laser is upside-down or not
LaserPowerControlled true ; if the power to the laser is controlled by serial
LaserMaxRange 0          ; Max range to use for the laser, 0 to use default
                         ; (only use if you want to shorten it from the
                         ; default), mm
LaserCumulativeBufferSize 0 ; Cumulative buffer size to use for the laser, 0 to
                         ; use default
LaserX 0                 ; x location of laser, mm
LaserY 0                 ; y location of laser, mm
LaserTh 0                ; rotation of laser, deg
LaserZ 0                 ; height of the laser off the ground, mm (0 means
                         ; unknown)
LaserIgnore              ; Readings within a degree of the listed degrees
                         ; (separated by a space) will be ignored
LaserStartDegrees        ; start degrees for the sensor (leave blank for
                         ; default, use this to constrain the range) (double)
LaserEndDegrees          ; start degrees for the sensor (leave blank for
                         ; default, use this to constrain the range) (double)
LaserDegreesChoice       ; degrees choice for the sensor (leave blank for
                         ; default, use this to constrain the range)
LaserIncrement           ; Increment for the sensor (leave blank for default,
                         ; use this to have a custom increment) (double)
LaserIncrementChoice     ; Increment for the sensor (leave blank for default,
                         ; use this to have a larger increment)
LaserUnitsChoice         ; Units for the sensor (leave blank for default, use
                         ; this to have a larger units)
LaserReflectorBitsChoice  ; ReflectorBits for the sensor (leave blank for
                         ; default, use this to have a larger units)
LaserStartingBaudChoice  ; StartingBaud for the sensor (leave blank for
                         ; default, use this to have a larger StartingBaud)
LaserAutoBaudChoice      ; AutoBaud for the sensor (leave blank for default,
                         ; use this to have a larger units)

Section Laser 4 parameters
;SectionFlags for Laser 4 parameters: 
LaserType                ; type of laser
LaserPortType            ; type of port the laser is on
LaserPort                ; port the laser is on
LaserAutoConnect false   ; if the laser connector should autoconnect this laser
                         ; or not
LaserFlipped false       ; if the laser is upside-down or not
LaserPowerControlled true ; if the power to the laser is controlled by serial
LaserMaxRange 0          ; Max range to use for the laser, 0 to use default
                         ; (only use if you want to shorten it from the
                         ; default), mm
LaserCumulativeBufferSize 0 ; Cumulative buffer size to use for the laser, 0 to
                         ; use default
LaserX 0                 ; x location of laser, mm
LaserY 0                 ; y location of laser, mm
LaserTh 0                ; rotation of laser, deg
LaserZ 0                 ; height of the laser off the ground, mm (0 means
                         ; unknown)
LaserIgnore              ; Readings within a degree of the listed degrees
                         ; (separated by a space) will be ignored
LaserStartDegrees        ; start degrees for the sensor (leave blank for
                         ; default, use this to constrain the range) (double)
LaserEndDegrees          ; start degrees for the sensor (leave blank for
                         ; default, use this to constrain the range) (double)
LaserDegreesChoice       ; degrees choice for the sensor (leave blank for
                         ; default, use this to constrain the range)
LaserIncrement           ; Increment for the sensor (leave blank for default,
                         ; use this to have a custom increment) (double)
LaserIncrementChoice     ; Increment for the sensor (leave blank for default,
                         ; use this to have a larger increment)
LaserUnitsChoice         ; Units for the sensor (leave blank for default, use
                         ; this to have a larger units)
LaserReflectorBitsChoice  ; ReflectorBits for the sensor (leave blank for
                         ; default, use this to have a larger units)
LaserStartingBaudChoice  ; StartingBaud for the sensor (leave blank for
                         ; default, use this to have a larger StartingBaud)
LaserAutoBaudChoice      ; AutoBaud for the sensor (leave blank for default,
                         ; use this to have a larger units)

Section Laser 5 parameters
;SectionFlags for Laser 5 parameters: 
LaserType                ; type of laser
LaserPortType            ; type of port the laser is on
LaserPort                ; port the laser is on
LaserAutoConnect false   ; if the laser connector should autoconnect this laser
                         ; or not
LaserFlipped false       ; if the laser is upside-down or not
LaserPowerControlled true ; if the power to the laser is controlled by serial
LaserMaxRange 0          ; Max range to use for the laser, 0 to use default
                         ; (only use if you want to shorten it from the
                         ; default), mm
LaserCumulativeBufferSize 0 ; Cumulative buffer size to use for the laser, 0 to
                         ; use default
LaserX 0                 ; x location of laser, mm
LaserY 0                 ; y location of laser, mm
LaserTh 0                ; rotation of laser, deg
LaserZ 0                 ; height of the laser off the ground, mm (0 means
                         ; unknown)
LaserIgnore              ; Readings within a degree of the listed degrees
                         ; (separated by a space) will be ignored
LaserStartDegrees        ; start degrees for the sensor (leave blank for
                         ; default, use this to constrain the range) (double)
LaserEndDegrees          ; start degrees for the sensor (leave blank for
                         ; default, use this to constrain the range) (double)
LaserDegreesChoice       ; degrees choice for the sensor (leave blank for
                         ; default, use this to constrain the range)
LaserIncrement           ; Increment for the sensor (leave blank for default,
                         ; use this to have a custom increment) (double)
LaserIncrementChoice     ; Increment for the sensor (leave blank for default,
                         ; use this to have a larger increment)
LaserUnitsChoice         ; Units for the sensor (leave blank for default, use
                         ; this to have a larger units)
LaserReflectorBitsChoice  ; ReflectorBits for the sensor (leave blank for
                         ; default, use this to have a larger units)
LaserStartingBaudChoice  ; StartingBaud for the sensor (leave blank for
                         ; default, use this to have a larger StartingBaud)
LaserAutoBaudChoice      ; AutoBaud for the sensor (leave blank for default,
                         ; use this to have a larger units)

Section Laser 6 parameters
;SectionFlags for Laser 6 parameters: 
LaserType                ; type of laser
LaserPortType            ; type of port the laser is on
LaserPort                ; port the laser is on
LaserAutoConnect false   ; if the laser connector should autoconnect this laser
                         ; or not
LaserFlipped false       ; if the laser is upside-down or not
LaserPowerControlled true ; if the power to the laser is controlled by serial
LaserMaxRange 0          ; Max range to use for the laser, 0 to use default
                         ; (only use if you want to shorten it from the
                         ; default), mm
LaserCumulativeBufferSize 0 ; Cumulative buffer size to use for the laser, 0 to
                         ; use default
LaserX 0                 ; x location of laser, mm
LaserY 0                 ; y location of laser, mm
LaserTh 0                ; rotation of laser, deg
LaserZ 0                 ; height of the laser off the ground, mm (0 means
                         ; unknown)
LaserIgnore              ; Readings within a degree of the listed degrees
                         ; (separated by a space) will be ignored
LaserStartDegrees        ; start degrees for the sensor (leave blank for
                         ; default, use this to constrain the range) (double)
LaserEndDegrees          ; start degrees for the sensor (leave blank for
                         ; default, use this to constrain the range) (double)
LaserDegreesChoice       ; degrees choice for the sensor (leave blank for
                         ; default, use this to constrain the range)
LaserIncrement           ; Increment for the sensor (leave blank for default,
                         ; use this to have a custom increment) (double)
LaserIncrementChoice     ; Increment for the sensor (leave blank for default,
                         ; use this to have a larger increment)
LaserUnitsChoice         ; Units for the sensor (leave blank for default, use
                         ; this to have a larger units)
LaserReflectorBitsChoice  ; ReflectorBits for the sensor (leave blank for
                         ; default, use this to have a larger units)
LaserStartingBaudChoice  ; StartingBaud for the sensor (leave blank for
                         ; default, use this to have a larger StartingBaud)
LaserAutoBaudChoice      ; AutoBaud for the sensor (leave blank for default,
                         ; use this to have a larger units)

Section Laser 7 parameters
;SectionFlags for Laser 7 parameters: 
LaserType                ; type of laser
LaserPortType            ; type of port the laser is on
LaserPort                ; port the laser is on
LaserAutoConnect false   ; if the laser connector should autoconnect this laser
                         ; or not
LaserFlipped false       ; if the laser is upside-down or not
LaserPowerControlled true ; if the power to the laser is controlled by serial
LaserMaxRange 0          ; Max range to use for the laser, 0 to use default
                         ; (only use if you want to shorten it from the
                         ; default), mm
LaserCumulativeBufferSize 0 ; Cumulative buffer size to use for the laser, 0 to
                         ; use default
LaserX 0                 ; x location of laser, mm
LaserY 0                 ; y location of laser, mm
LaserTh 0                ; rotation of laser, deg
LaserZ 0                 ; height of the laser off the ground, mm (0 means
                         ; unknown)
LaserIgnore              ; Readings within a degree of the listed degrees
                         ; (separated by a space) will be ignored
LaserStartDegrees        ; start degrees for the sensor (leave blank for
                         ; default, use this to constrain the range) (double)
LaserEndDegrees          ; start degrees for the sensor (leave blank for
                         ; default, use this to constrain the range) (double)
LaserDegreesChoice       ; degrees choice for the sensor (leave blank for
                         ; default, use this to constrain the range)
LaserIncrement           ; Increment for the sensor (leave blank for default,
                         ; use this to have a custom increment) (double)
LaserIncrementChoice     ; Increment for the sensor (leave blank for default,
                         ; use this to have a larger increment)
LaserUnitsChoice         ; Units for the sensor (leave blank for default, use
                         ; this to have a larger units)
LaserReflectorBitsChoice  ; ReflectorBits for the sensor (leave blank for
                         ; default, use this to have a larger units)
LaserStartingBaudChoice  ; StartingBaud for the sensor (leave blank for
                         ; default, use this to have a larger StartingBaud)
LaserAutoBaudChoice      ; AutoBaud for the sensor (leave blank for default,
                         ; use this to have a larger units)

Section Laser 8 parameters
;SectionFlags for Laser 8 parameters: 
LaserType                ; type of laser
LaserPortType            ; type of port the laser is on
LaserPort                ; port the laser is on
LaserAutoConnect false   ; if the laser connector should autoconnect this laser
                         ; or not
LaserFlipped false       ; if the laser is upside-down or not
LaserPowerControlled true ; if the power to the laser is controlled by serial
LaserMaxRange 0          ; Max range to use for the laser, 0 to use default
                         ; (only use if you want to shorten it from the
                         ; default), mm
LaserCumulativeBufferSize 0 ; Cumulative buffer size to use for the laser, 0 to
                         ; use default
LaserX 0                 ; x location of laser, mm
LaserY 0                 ; y location of laser, mm
LaserTh 0                ; rotation of laser, deg
LaserZ 0                 ; height of the laser off the ground, mm (0 means
                         ; unknown)
LaserIgnore              ; Readings within a degree of the listed degrees
                         ; (separated by a space) will be ignored
LaserStartDegrees        ; start degrees for the sensor (leave blank for
                         ; default, use this to constrain the range) (double)
LaserEndDegrees          ; start degrees for the sensor (leave blank for
                         ; default, use this to constrain the range) (double)
LaserDegreesChoice       ; degrees choice for the sensor (leave blank for
                         ; default, use this to constrain the range)
LaserIncrement           ; Increment for the sensor (leave blank for default,
                         ; use this to have a custom increment) (double)
LaserIncrementChoice     ; Increment for the sensor (leave blank for default,
                         ; use this to have a larger increment)
LaserUnitsChoice         ; Units for the sensor (leave blank for default, use
                         ; this to have a larger units)
LaserReflectorBitsChoice  ; ReflectorBits for the sensor (leave blank for
                         ; default, use this to have a larger units)
LaserStartingBaudChoice  ; StartingBaud for the sensor (leave blank for
                         ; default, use this to have a larger StartingBaud)
LaserAutoBaudChoice      ; AutoBaud for the sensor (leave blank for default,
                         ; use this to have a larger units)

Section Laser 9 parameters
;SectionFlags for Laser 9 parameters: 
LaserType                ; type of laser
LaserPortType            ; type of port the laser is on
LaserPort                ; port the laser is on
LaserAutoConnect false   ; if the laser connector should autoconnect this laser
                         ; or not
LaserFlipped false       ; if the laser is upside-down or not
LaserPowerControlled true ; if the power to the laser is controlled by serial
LaserMaxRange 0          ; Max range to use for the laser, 0 to use default
                         ; (only use if you want to shorten it from the
                         ; default), mm
LaserCumulativeBufferSize 0 ; Cumulative buffer size to use for the laser, 0 to
                         ; use default
LaserX 0                 ; x location of laser, mm
LaserY 0                 ; y location of laser, mm
LaserTh 0                ; rotation of laser, deg
LaserZ 0                 ; height of the laser off the ground, mm (0 means
                         ; unknown)
LaserIgnore              ; Readings within a degree of the listed degrees
                         ; (separated by a space) will be ignored
LaserStartDegrees        ; start degrees for the sensor (leave blank for
                         ; default, use this to constrain the range) (double)
LaserEndDegrees          ; start degrees for the sensor (leave blank for
                         ; default, use this to constrain the range) (double)
LaserDegreesChoice       ; degrees choice for the sensor (leave blank for
                         ; default, use this to constrain the range)
LaserIncrement           ; Increment for the sensor (leave blank for default,
                         ; use this to have a custom increment) (double)
LaserIncrementChoice     ; Increment for the sensor (leave blank for default,
                         ; use this to have a larger increment)
LaserUnitsChoice         ; Units for the sensor (leave blank for default, use
                         ; this to have a larger units)
LaserReflectorBitsChoice  ; ReflectorBits for the sensor (leave blank for
                         ; default, use this to have a larger units)
LaserStartingBaudChoice  ; StartingBaud for the sensor (leave blank for
                         ; default, use this to have a larger StartingBaud)
LaserAutoBaudChoice      ; AutoBaud for the sensor (leave blank for default,
                         ; use this to have a larger units)
