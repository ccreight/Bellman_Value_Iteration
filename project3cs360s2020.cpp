#include <vector>
#include <fstream>
#include <ostream>
#include <iostream>
#include <sstream>
#include <algorithm>
#include <float.h>

std::vector<std::vector<char>> charGrid;
std::vector<std::vector<double>> valueGrid;
std::vector<std::vector<double>> nextValues;

int gridSize;
const double GAMMA = 0.9;
const double EPSILON = 0.1;

int destX, destY;

void PathFinding();

// Read in the chefs from the input file
void ReadFile() {

	std::ifstream infile("input.txt");

	// Read in the number of chefs
	std::string str;
	getline(infile, str);
	gridSize = std::stoi(str);

	// Initialize the grid of characters to spaces
	for (int i = 0; i < gridSize; i++) {

		std::vector<char> chars;
		charGrid.push_back(chars);

		for (int j = 0; j < gridSize; j++) {
			charGrid[i].push_back(' ');
		}

	}

	getline(infile, str);
	int numObstacles = std::stoi(str);

	for (int i = 0; i < numObstacles; i++) {

		getline(infile, str);
		std::stringstream s(str);

		char g; // garbage
		int posX, posY; // position

		s >> posX >> g >> posY;
		charGrid[posX][posY] = 'o';

	}

	// Read in destination
	getline(infile, str);
	std::stringstream s(str);

	char g; // garbage
	int posX, posY;
	s >> posX >> g >> posY;
	destX = posX;
	destY = posY;

	charGrid[posX][posY] = '.';

	infile.close();

}

// Initialize the utility value grids to zero
void ProcessStartingValues() {

	for (int i = 0; i < gridSize; i++) {

		std::vector<double> values;
		valueGrid.push_back(values);
		std::vector<double> nextVals;
		nextValues.push_back(nextVals);

		for (int j = 0; j < gridSize; j++) {

			valueGrid[i].push_back(0);
			nextValues[i].push_back(0);

		}
	}

}

// For debugging
void PrintGrids() {

	for (int i = 0; i < gridSize; i++) {
		for (int j = 0; j < gridSize; j++) {
			std::cout << charGrid[j][i] << " ";
		}
		std::cout << std::endl;
	}

	std::cout << std::endl;

	for (int i = 0; i < gridSize; i++) {
		for (int j = 0; j < gridSize; j++) {
			std::cout << valueGrid[j][i] << " ";
		}
		std::cout << std::endl;
	}

}

void WriteFile() {

	std::ofstream myfile;
	myfile.open("output.txt");

	// Write each policy to the file
	for (int i = 0; i < gridSize; i++) {
		for (int j = 0; j < gridSize; j++) {
			myfile << charGrid[j][i];
		}
		myfile << "\n";
	}

	myfile.close();

}

void GetUtility(int x, int y) {

	// Neighbor values
	double tempLeft, tempRight, tempDown, tempUp;

	// Left bound
	if (x - 1 < 0) {
		tempLeft = valueGrid[x][y];
		if (charGrid[x][y] == 'o') {
			tempLeft = -101;
		}
	}
	else {
		tempLeft = valueGrid[x - 1][y];
		if (charGrid[x - 1][y] == 'o') {
			tempLeft = -101;
		}
	}

	// Right bound
	if (x + 1 >= gridSize) {
		tempRight = valueGrid[x][y];
		if (charGrid[x][y] == 'o') {
			tempRight = -101;
		}
	}
	else {
		tempRight = valueGrid[x + 1][y];
		if (charGrid[x + 1][y] == 'o') {
			tempRight = -101;
		}
	}

	// Upper bound
	if (y - 1 < 0) {
		tempUp = valueGrid[x][y];
		if (charGrid[x][y] == 'o') {
			tempUp = -101;
		}
	}
	else {
		tempUp = valueGrid[x][y - 1];
		if (charGrid[x][y - 1] == 'o') {
			tempUp = -101;
		}
	}

	// Lower bound
	if (y + 1 >= gridSize) {
		tempDown = valueGrid[x][y];
		if (charGrid[x][y] == 'o') {
			tempDown = -101;
		}
	}
	else {
		tempDown = valueGrid[x][y + 1];
		if (charGrid[x][y + 1] == 'o') {
			tempDown = -101;
		}
	}

	// Find utility of each movement
	double left = tempLeft * .7 + tempUp * .1 + tempDown * .1 + tempRight * .1;
	double right = tempRight * .7 + tempUp * .1 + tempDown * .1 + tempLeft * .1;
	double up = tempUp * .7 + tempLeft * .1 + tempRight * .1 + tempDown * .1;
	double down = tempDown * .7 + tempLeft * .1 + tempRight * .1 + tempUp * .1;
	double maxUtility = std::max(left, std::max(right, std::max(up, down)));

	nextValues[x][y] = GAMMA * maxUtility - 1; // R(x) = 1

	// Set the utility of an obstacle to always be -101
	if (charGrid[x][y] == 'o') {

		nextValues[x][y] = -101;
		return;

	}

	// Set utility of the destination to always be 99
	else if (charGrid[x][y] == '.') {

		nextValues[x][y] = 99;
		return;

	}

	// Set movement in character grid - tie-breaking order
	if (maxUtility == up) {
		charGrid[x][y] = '^';
	}
	else if (maxUtility == down) {
		charGrid[x][y] = 'v';
	}
	else if (maxUtility == right) {
		charGrid[x][y] = '>';
	}
	else {
		charGrid[x][y] = '<';
	}

	PrintGrids();

}

void ValueIteration() {

	while (true) {

		// True if we're below the epsilon
		bool epsilonMet = true;

		// Loop over every cell
		for (int i = 0; i < gridSize; i++) {

			for (int j = 0; j < gridSize; j++) {

				// Calculate the utility function for a cell
				GetUtility(i, j);

				// Check if we need to keep iterating - not refined enough
				if (abs(nextValues[i][j] - valueGrid[i][j]) > EPSILON* ((1 - GAMMA) / GAMMA)) {
					epsilonMet = false;
				}

			}

		}

		// We've narrowed it down enough - time to stop!
		if (epsilonMet) {
			break;
		}

		// Update to hold the next utility values
		for (int i = 0; i < gridSize; i++) {
			for (int j = 0; j < gridSize; j++) {
				valueGrid[i][j] = nextValues[i][j];
			}
		}



	}

}

int main() {

	ReadFile();
	ProcessStartingValues();
	ValueIteration();
	WriteFile();

}