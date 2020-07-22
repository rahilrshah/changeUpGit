using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller Controller1;
extern motor Ldrive1;
extern motor Rdrive1;
extern motor Rdrive2;
extern motor Intake1;
extern motor Intake2;
extern motor Serial;
extern motor Index;
extern motor Ldrive2;
extern line middleBall;
extern line bottomBall;
extern line middleBall2;
extern line intakeBall;
extern sonar topBall;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );