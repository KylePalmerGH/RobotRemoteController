class RotaryJoystick():
    def rotary_inputs(self, robot):
        x=robot.drive_stick.getX()
        y=robot.drive_stick.getY()
        z=robot.drive_stick.getZ()
        MAX=max(x, y, z)
        MIN=min(x, y, z)
        if((x<=y) and (y<=z)):
            mid=y  
            angle=60-(mid-MIN)/(MAX-MIN)*60
        if((y<=x) and (x<=z)):
            mid=x
            angle=60+(mid-MIN)/(MAX-MIN)*60
        if((y<=z) and (z<=x)):
            mid=z
            angle=180-(mid-MIN)/(MAX-MIN)*60
        if((z<=y) and (y<=x)):
            mid=y
            angle=180+(mid-MIN)/(MAX-MIN)*60
        if((z<=x) and (x<=y)):
            mid=x
            angle=300-(mid-MIN)/(MAX-MIN)*60
        if((x<=z) and (z<=y)):
            mid=z
            angle=300+(mid-MIN)/(MAX-MIN)*60
        #print ("a=", angle)
        return angle
    def set_pid(self, p, i, d, val):
        self.p = p
        self.i = i
        self.d = d
        self.previous_input = val
        self.integral = 0

    def steer_pid(self, error):
        power = error * self.p
        if self.integral > 0 and (error * self.i < 0):
            self.integral = 0
        if (self.integral < 0) and (error * self.i > 0):
            self.integral = 0
        self.integral += error*self.i
        if (-20 < error) and (error < 20):
            power += self.integral
        else: 
            self.integral=0
        power += (error - self.previous_input) * self.d
        self.previous_input = error
        return power

    def interp(self, joy):
        ary = [ \
        [-1,-12],\
        [-.75,-1],\
        [-.5,-.2],\
        [-.25,0],\
        [.25,0],\
        [.5,.2],\
        [.75,1],\
        [1,12]]
        if joy <= ary[0][0]:
            return ary[0][1]
        if joy >= ary[len(ary) - 1][0]: 
            return ary[len(ary) - 1][1]
        for i in range(len(ary) - 1):
            if ((joy>=ary[i+0][0]) and (joy<=ary[i+1][0])): 
                return (joy-ary[i+0][0])*(ary[i+1][1]-ary[i+0][1])/(ary[i+1][0]-ary[i+0][0])+ary[i+0][1]
        return 0

    def leftright_power(robot, enable, left, right):
        #Takes in values from joysticks and returns motor power for both sides
       rotary_input(robot, angle)
        #Takes in x, y, and z from rotary joystick and converts it to angle
        #Takes in enable(switch) value and uses only joystick or uses joystick and aligns rotary joystick with robot
        #Uses PID to give turning power
        #Uses power array and joystick to give forward/backward power