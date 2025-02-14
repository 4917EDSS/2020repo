{
    # Warning: This project type is for BRUSHLESS motors ONLY!
    # Ports for the left-side motors
    "leftMotorPorts": [5, 6, 7, 8],
    # Ports for the right-side motors
    "rightMotorPorts": [1, 2, 3, 4],
    # Note: Inversions of the slaves (i.e. any motor *after* the first on
    # each side of the drive) are *with respect to their master*.  This is
    # different from the other poject types!
    # Inversions for the left-side motors
    "leftMotorsInverted": [True, False, False, False],
    # Inversions for the right side motors
    "rightMotorsInverted": [True, False, False, False],
    # The total gear reduction between the motor and the wheels, expressed as
    # a fraction [motor turns]/[wheel turns]
    "gearing": 12,
    # Wheel diameter (in units of your choice - will dictate units of analysis)
    "wheelDiameter": 0.1905,
    # Your gyro type (one of "NavX", "Pigeon", "ADXRS450", "AnalogGyro", or "None")
    "gyroType": "NavX",
    # Whatever you put into the constructor of your gyro
    # Could be:
    # "SPI.Port.kMXP" (MXP SPI port for NavX or ADXRS450),
    # "SerialPort.Port.kMXP" (MXP Serial port for NavX),
    # "I2C.Port.kOnboard" (Onboard I2C port for NavX),
    # "0" (Pigeon CAN ID or AnalogGyro channel),
    # "new WPI_TalonSRX(3)" (Pigeon on a Talon SRX),
    # "" (NavX using default SPI, ADXRS450 using onboard CS0, or no gyro)
    "gyroPort": "SPI.Port.kMXP",
}

