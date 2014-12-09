/**
This module is designed to provide convenient value smoothing capabilities to the Arduino.
The user creates a new Smoother instance and "pushes" raw values into it, and can then "pull"
a running average of the values back from it. Raw value "pushes" and smooth value "pulls"
may happen concurrently, and a smoothed value may be "pulled" as soon as the first raw
value has been "pushed".

Author: Kevin Balke (fughilli@gmail.com)
**/

#ifndef SMOOTHER_H
#define SMOOTHER_H

#define CALC_ON_PUSH 0
#define CALC_ON_PULL 1

template <typename T, unsigned int S>
class Smoother
{
private:					
	T rawValues[S];					// Array to store raw values pushed by the user
	unsigned int valueIndex;		// The index to push a new raw value to
	bool loaded;					// Has the rawValues array been fully filled yet?
	unsigned char solveTime;		// When to solve for the running average
	T average;						// The stored running average
	
	//! Calculate the average value of the rawValues array
	inline void calculateAverage(void)					
	{
		average = (T)0;									// Reset the average to 0
		unsigned int i;									// Index variable (needs to be outside of for() scope)
		for(i = 0; i < (loaded?S:valueIndex); i++)		// Count up to S if the rawValues is fully loaded, 
														// otherwise count up to the current index (still filling)
		{
			average += rawValues[i];					// Compute sum
		}
		average /= (T)i;								// Divide by the number of elements counted
	}
	
public:
	//! Constructor
	Smoother(unsigned char _solveTime = CALC_ON_PULL)
	{
		valueIndex = 0;				// Raw value pushes will start at the beginning of the array
		loaded = false;				// The array has not been filled yet
		average = (T)0;				// Averaged value is 0, for now
		solveTime = _solveTime;		// By default, solve for the average when pulling a value
	}
	
	//! Load a new raw value into the rawValues array
	void pushValue(T rawValue) 
	{
		rawValues[valueIndex] = rawValue;	// Load a the given raw value into rawValues at the currently indexed location
		
		if(solveTime == CALC_ON_PUSH)		// If the user has specified to solve for the average after a push operation
		{
			calculateAverage();				// Calculate the average
		}
		
		valueIndex++;						// Increment the index
		if(valueIndex == S)					// If the index has reached the end of the rawValues array (for the first time?)
		{
			loaded = true;					// The rawValues array is fully loaded
			valueIndex = 0;					// Reset the index
		}
	}
	
	//! Get the smoothed (average) value from the rawValues array
	T pullValue(void)
	{
		if(solveTime == CALC_ON_PULL)		// If the user has specified to solve for the average after a pull operation 
											// (this is the default)
		{
			calculateAverage();				// Calculate the average
		}
		
		return average;						// Return the average back to the user
	}
};

#endif