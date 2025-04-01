/*
====================================================================================================
File: SoftwarePLL.cpp
Note: SoftwarePLL intentionally copied from https://github.com/michael1309/SoftwarePLL
See https://github.com/michael1309/SoftwarePLL/blob/master/README.md for details.
====================================================================================================
*/
#include "sick_lidar_localization/SoftwarePLL.h"

// #include "softwarePLL.h"
#include <iostream>
// #include <chrono>
// #include <thread>
#include <math.h>
#include <iterator>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>


std::map<std::string,SoftwarePLL*> SoftwarePLL::_instances; // list of SoftwarePLL instances, mapped by id

const double SoftwarePLL::MaxAllowedTimeDeviation_ = 0.1;
const uint64_t SoftwarePLL::MaxExtrapolationCounter_ = 5; // tries to match timestamps after diverging

SoftwarePLL::SoftwarePLL(int fifo_length) : FifoSize_(fifo_length)
{
	Reset();
	AllowedTimeDeviation(SoftwarePLL::MaxAllowedTimeDeviation_); // 1 ms
	NumberValInFifo_ = 0;
}

SoftwarePLL& SoftwarePLL::Instance(const std::string & id, int fifo_length)
{
	SoftwarePLL* pll = _instances[id];
	if(!pll)
	{
		pll = new SoftwarePLL(fifo_length);
		_instances[id] = pll;
	}
	return *pll;
}

SoftwarePLL::~SoftwarePLL()
{
	// Remove this from the list of SoftwarePLL instances
	for(std::map<std::string,SoftwarePLL*>::iterator iter = _instances.begin(); iter != _instances.end(); )
	{
		if(iter->second == this)
			iter = _instances.erase(iter);
		else
			iter++;
	}
}

void SoftwarePLL::Reset()
{
	TickFifo_ = std::vector<uint64_t>(FifoSize_, 0);
	ClockFifo_ = std::vector<double>(FifoSize_, 0); 
	IsInitialized(false);
	FirstTimeStamp_ = 0;
	FirstTick_ = 0;
	InterpolationSlope_ = 0;
}

bool SoftwarePLL::PushIntoFifo(double curTimeStamp, uint64_t curtick)
// update tick fifo and update clock (timestamp) fifo
{	
	if (curtick < TickFifo_[FifoSize_ - 1]) // Time pausability latest time cannot be in the past (device off)
	{
		Reset();		
	}		

	for (int i = 0; i < FifoSize_ - 1; i++)
	{
		TickFifo_[i] = TickFifo_[i + 1];
		ClockFifo_[i] = ClockFifo_[i + 1];
	}
	TickFifo_[FifoSize_ - 1] = curtick; // push most recent tick and timestamp into fifo
	ClockFifo_[FifoSize_ - 1] = curTimeStamp;

	if (NumberValInFifo_ < FifoSize_)
	{
		NumberValInFifo_++; // remember the number of valid number in fifo
	}
	FirstTick(TickFifo_[0]);
	FirstTimeStamp(ClockFifo_[0]);

	return(true);
}

double SoftwarePLL::ExtraPolateRelativeTimeStamp(uint64_t tick)
{
	double timeDiff;
	const uint64_t firstTick = FirstTick();
	if (tick < firstTick)
	{
		std::cout << "Negative extrapolated time difference. Reset...";
		timeDiff = 0.0;
		Reset();
	}
	else {
		timeDiff = (tick - firstTick) * this->InterpolationSlope();
	}
	
	return timeDiff;
}

void SoftwarePLL::UpdatePLL(uint32_t sec, uint32_t nanoSec, uint64_t curtick)
{
  if(curtick!=this->Lastcurtick_)
  {
    this->Lastcurtick_ = curtick;
    double start = sec + nanoSec * 1E-9;

		PushIntoFifo(start, curtick);
		bool bCheck = this->UpdateInterpolationSlope();
		if (bCheck)
		{			
			IsInitialized(true);
		} 
		else {
			return;
		}

    double relTimeStamp = ExtraPolateRelativeTimeStamp(curtick);
    double cmpTimeStamp = start - this->FirstTimeStamp();

    bool timeStampVerified = NearSameTimeStamp(relTimeStamp, cmpTimeStamp);
    if (!timeStampVerified) 
    {
      // BEGIN HANDLING Extrapolation divergence
      uint64_t tmp = ExtrapolationDivergenceCounter();
      tmp++;
      ExtrapolationDivergenceCounter(tmp);
      if (ExtrapolationDivergenceCounter() >= SoftwarePLL::MaxExtrapolationCounter_)
      {
        Reset(); // reset FIFO - maybe happened due to abrupt change of time base
      }
      // END HANDLING Extrapolation divergence
    }
		else
    {
      ExtrapolationDivergenceCounter(0);
    }
  }
}

bool SoftwarePLL::GetCorrectedTimeStamp(uint32_t& sec, uint32_t& nanoSec, uint64_t curtick)
{
	if (IsInitialized() == false)
	{
		return(false);
	}

	double relTimeStamp = ExtraPolateRelativeTimeStamp(curtick); // evtl. hier wg. Ueberlauf noch einmal pruefen
	double corrTime = relTimeStamp + this->FirstTimeStamp();
	sec = static_cast<uint32_t>(corrTime);
	double frac = corrTime - sec;
	nanoSec = static_cast<uint32_t>(1E9 * frac);
	return(true);
}

bool SoftwarePLL::NearSameTimeStamp(double relTimeStamp1, double relTimeStamp2)
{
	double dTAbs = fabs(relTimeStamp1 - relTimeStamp2);
	return (dTAbs < AllowedTimeDeviation());
}

bool SoftwarePLL::UpdateInterpolationSlope() // fifo already updated
{

	if (NumberValInFifo_ < FifoSize_)
	{
		return(false);
	}
	std::vector<uint64_t> tickFifoUnwrap;
	std::vector<double> clockFifoUnwrap;
	clockFifoUnwrap.resize(FifoSize_);
	tickFifoUnwrap.resize(FifoSize_);
	uint64_t tickOffset = 0;
	clockFifoUnwrap[0] = 0.00;
	tickFifoUnwrap[0] = 0;
	FirstTimeStamp(this->ClockFifo_[0]);
	FirstTick(this->TickFifo_[0]);

	// uint64_t tickDivisor = 0x100000000;



	for (int i = 1; i < FifoSize_; i++)  // typical 643 for 20ms -> round about 32150 --> near to 32768 standard clock in many watches
	{
		// if (TickFifo_[i] < TickFifo_[i - 1]) // Overflow (obsolete with uint64 tics)
		// {
		// 	 tickOffset += tickDivisor;
		// }
		tickFifoUnwrap[i] = tickOffset + TickFifo_[i] - FirstTick();
		clockFifoUnwrap[i] = (this->ClockFifo_[i] - FirstTimeStamp());
	}

	double sum_xy = 0.0;
	double 	sum_x = 0.0;
	double sum_y = 0.0;
	double sum_xx = 0.0;
	for (int i = 0; i < FifoSize_; i++)
	{
		sum_xy += tickFifoUnwrap[i] * clockFifoUnwrap[i];
		sum_x += tickFifoUnwrap[i];
		sum_y += clockFifoUnwrap[i];
		sum_xx += tickFifoUnwrap[i] * tickFifoUnwrap[i];
	}

	// calculate slope of regression line, interception is 0 by construction
	double m = (FifoSize_ * sum_xy - sum_x * sum_y) / (FifoSize_ * sum_xx - sum_x*sum_x);

	int matchCnt = 0;
	for (int i = 0; i < FifoSize_; i++)
	{
		double yesti = m * tickFifoUnwrap[i];
		if (this->NearSameTimeStamp(yesti, clockFifoUnwrap[i]))
		{
			matchCnt++;
		}
	}

	bool retVal = false;
	if (matchCnt == FifoSize_)
	{
		InterpolationSlope(m);
		retVal = true;
	}

	return(retVal);
}

/* 
Example CMakeLists.txt to generate test-binary-file for testing this class
--- CUT ---
#
#
# softwarePLL
#
#
cmake_minimum_required(VERSION 2.8)
cmake_policy(SET CMP0015 NEW)
project( softwarePLL )
#
#
add_definitions(-D${PROJECT_NAME}_MAINTEST)

MESSAGE( STATUS "CMKAKE for " ${PROJECT_NAME} )

include_directories( inc)
file( GLOB LIB_SOURCES src/ *.cpp )

if(WIN32)
else()
set(CMAKE_CXX_STANDARD 11)
endif()

add_executable( ${PROJECT_NAME} ${LIB_SOURCES} inc/${PROJECT_NAME}.h)
target_link_libraries( ${PROJECT_NAME})
--- CUT ---

*/
