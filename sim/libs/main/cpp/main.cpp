/*
 * This C++ source file was generated by the Gradle 'init' task.
 */
#include "main.h"
#include "robot/keyboard.h"

using WindowData = Config::Window;
using RobotData = Config::Robot;

// returns true if user presses r (reset)
int emptyKeyCycle = 0;
void FPSController() {
	// Determin FPS of program (ish)
	double ms = 1000/Config::Window::CPS; // 60 CPS = 16.6ms every cycle
	if (Config::Window::CPS == 0) {
		ms = 1;
	}

	// Use waitkey to wait and detect user input
	int key = cv::waitKey(ms);
	if (key == 27) {
		running = false;
	} else {
		Config::Keyboard::setKey(key);
	}
}

int main(int argc, char const *argv[]) {
	std::cout << "Simulator Start [PRESS ESC TO EXIT, & R TO RESET]" << std::endl;
	running = true;

	// Initialize Simulator
	controller.sim->Init();

	auto start = high_resolution_clock::now(); // Start timer
	auto stop = high_resolution_clock::now(); // End timer
	auto duration = duration_cast<milliseconds>(stop - start); // Duration since start
	double currentTime = 0, lastTime = 0, dt = 0, dt_counted = 0, avg_dt = 0; // Time values
	double count = 0; // Cycle counter, resets every second
	double ACTUAL_CPS = 0, cycles = 0; // cycles per second

	while (running) {
		currentTime = duration.count();
		dt = currentTime - lastTime;
		dt /= 1000; // Convert ms to s
		Config::Sim::setGlobalDT(dt);
		Config::Sim::setGlobalCPS(ACTUAL_CPS);


		controller.window->window_SIM_PRE_Update(); // ------- DRAW/UPDATE SIMULATOR START




		controller.sim->Periodic();
		controller.robot->update();
		controller.window->drawInfoLabel("Set CPS: " + std::to_string(WindowData::CPS) + " Actual CPS: " + std::to_string(ACTUAL_CPS));
		controller.window->drawInfoLabel("Avg Delta time/s: " + std::to_string(avg_dt) + " Actual Delta time: " + std::to_string(dt));
		controller.window->drawInfoLabel("Counter: " + std::to_string(count));
		controller.window->drawInfoLabel("Window Size: " + std::to_string(WindowData::Width) + "x" + std::to_string(WindowData::Height));




		controller.window->window_SIM_POST_Update(); // ------- DRAW/UPDATE SIMULATOR END

		FPSController();

		stop = high_resolution_clock::now();
		duration = duration_cast<milliseconds>(stop - start);

		if (count >= 1) {

			// Calc CPS
			count = 0;
			ACTUAL_CPS = cycles;

			// Calc avg dt
			avg_dt = dt_counted/1000;

			// Reset values for next second cycle
			dt_counted = 0;
			cycles = 0;
		}

		// Value counters
		count += dt;
		cycles += 1;
		dt_counted += dt;
		lastTime = currentTime;
	}



	std::cout << "Program end" << std::endl;

	return 0;
}