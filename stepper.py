import RPi.GPIO as GPIO
import time

class StepperMotor:
	def __init__(self, pins):
		self.pins = pins
		self.halfstep_seq = [
			[1,0,0,0],
			[1,1,0,0],
			[0,1,0,0],
			[0,1,1,0],
			[0,0,1,0],
			[0,0,1,1],
			[0,0,0,1],
			[1,0,0,1]
		]

		self.halfstep_seq_back = [
			[1,0,0,1],
			[0,0,0,1],
			[0,0,1,1],
			[0,0,1,0],
			[0,1,1,0],
			[0,1,0,0],
			[1,1,0,0],
			[1,0,0,0]
		]

		self.remainingSteps = 0
		self.seqProgress = 0

	def init(self):
		for pin in self.pins:
			GPIO.setup(pin, GPIO.OUT)
			GPIO.output(pin, 0)

	def _subStep(self, seq):
		stepCompleted = False
		if (self.seqProgress >= len(seq)):
			self.seqProgress = 0
			stepCompleted = True
		for pin in range(len(self.pins)):
			GPIO.output(self.pins[pin], seq[self.seqProgress][pin])
		self.seqProgress = self.seqProgress + 1
		return stepCompleted

	def scheduleSteps(self, nbrSteps):
		self.remainingSteps = nbrSteps

	def update(self):
		if (self.remainingSteps > 0):
			stepCompleted = self._subStep(self.halfstep_seq)
			if stepCompleted:
				self.remainingSteps = self.remainingSteps - 1
		if (self.remainingSteps < 0):
                        stepCompleted = self._subStep(self.halfstep_seq_back)
			if stepCompleted:
				self.remainingSteps = self.remainingSteps + 1

	def isDone(self):
		return self.remainingSteps == 0


if __name__ == "__main__":
	GPIO.setmode(GPIO.BOARD)

	control_pins_stepper1 = [11,13,15,16]
	control_pins_stepper2 = [29,31,33,32]
	motor1 = StepperMotor(control_pins_stepper1)
	motor2 = StepperMotor(control_pins_stepper2)
	motor1.init()
	motor2.init()

	timeStep = 0.001
	direction = 1
	while True:
		if (motor1.isDone() and motor2.isDone()):
			motor1.scheduleSteps(512 * direction)
			motor2.scheduleSteps(512 * direction)
			direction = direction * -1

		motor1.update()
		motor2.update()

		time.sleep(timeStep)

	GPIO.cleanup()
